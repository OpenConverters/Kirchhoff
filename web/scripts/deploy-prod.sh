#!/usr/bin/env bash
# Deploy the Kirchhoff site to kirchhoff.openconverters.com (Scaleway box
# 51.15.253.66, same host as kelvin.openconverters.com). Idempotent.
#
#   web/scripts/deploy-prod.sh                # build + rsync + sidecars + vhost + verify
#   SKIP_NGINX=1 web/scripts/deploy-prod.sh   # SPA-only redeploy (vhost untouched)
#
# Data note: the SPA reads /kelvin/{*.kidx,*.ndjson,manifest.json} from /cache/kelvin
# on the box (the SAME set Kelvin serves). This script does NOT touch it.
set -euo pipefail

HERE="$(cd "$(dirname "$0")/.." && pwd)"     # web/
HOST=root@51.15.253.66
SSH="ssh -i $HOME/.ssh/om_scaleway -o StrictHostKeyChecking=no"
DOCROOT=/opt/kirchhoff/dist

# 0. Guarantee the embedded WASM engine is CURRENT. The SPA's `sync-wasm` step only
# COPIES build-wasm-ng/kirchhoff.js into public/ — it does NOT rebuild it — so a
# stale local build silently ships an out-of-date engine. That bit us on 2026-07-22:
# a WASM built before Kelvin's shard format bumped to v4 kept getting copied through
# font/CSS redeploys, and every component selection then failed with "unsupported
# shard format version 4" against the v4 catalogue shards. Rebuild it incrementally
# here: a no-op when already fresh (needs no toolchain), a real rebuild when stale,
# and a LOUD failure if it is stale but the WASM toolchain is unavailable — never a
# silent stale deploy. Override with SKIP_WASM_BUILD=1 only if you just built it.
WASM_DIR="$HERE/../build-wasm-ng"
if [[ -z "${SKIP_WASM_BUILD:-}" ]]; then
  if [[ ! -f "$WASM_DIR/build.ninja" ]]; then
    echo "REFUSING to deploy: $WASM_DIR is not a configured emscripten build dir — the WASM" >&2
    echo "engine cannot be verified current. Configure + build it, or set SKIP_WASM_BUILD=1." >&2
    exit 1
  fi
  echo "Ensuring the WASM engine is up to date (ninja, incremental)…"
  ninja -C "$WASM_DIR" kirchhoff.js
fi

# 1. Clean-HEAD build (refuse to ship uncommitted work — the byte-verify rule).
if [[ -n "$(git -C "$HERE" status --porcelain -- . 2>/dev/null)" ]]; then
  echo "REFUSING to deploy: web/ has uncommitted changes (a build bundles the working tree)." >&2
  exit 1
fi
( cd "$HERE" && npm run build )

# 2. SPA rsync (the /kelvin data path is served off /cache, never from the docroot).
$SSH "$HOST" "mkdir -p $DOCROOT"
rsync -az --delete --exclude 'kelvin/' -e "$SSH" "$HERE/dist/" "$HOST:$DOCROOT/"

# 3. gzip sidecars (gzip_static on — a stale .gz silently serves old bytes to browsers).
$SSH "$HOST" "cd $DOCROOT && find . -type f \( -name '*.js' -o -name '*.css' -o -name '*.html' -o -name '*.svg' \) -not -name '*.gz' -exec gzip -kf9 {} \;"

# 4. nginx vhost + TLS (first run only; reuses Kelvin's conventions).
if [[ -z "${SKIP_NGINX:-}" ]]; then
  scp -i "$HOME/.ssh/om_scaleway" -o StrictHostKeyChecking=no "$HERE/scripts/nginx-kirchhoff.conf" "$HOST:/etc/nginx/sites-available/kirchhoff"
  $SSH "$HOST" "ln -sfn /etc/nginx/sites-available/kirchhoff /etc/nginx/sites-enabled/kirchhoff && nginx -t && systemctl reload nginx"
  $SSH "$HOST" "certbot --nginx -d kirchhoff.openconverters.com --non-interactive --agree-tos -m openmagnetics@protonmail.com && nginx -t && systemctl reload nginx"
fi

# 5. Byte-verify the live artifacts against this clean-HEAD build (per artifact).
BASE=https://kirchhoff.openconverters.com
if [[ -n "${SKIP_NGINX:-}" ]] || curl -sfI "$BASE/" >/dev/null 2>&1; then
  # Each artifact is checked over BOTH response paths, because nginx has gzip_static
  # on: a plain request serves <file>, but any browser (Accept-Encoding: gzip) gets
  # <file>.gz instead. A stale sidecar therefore ships old bytes to every real user
  # while a plain curl reports success — so checking only the plain path cannot
  # detect the very failure this message names. (Cost us a full cycle on 2026-07-20:
  # a corrected catalogue was live, plain curl verified clean, and browsers kept
  # loading the previous manifest from its stale .gz.)
  for f in kirchhoff.js index.html $(cd "$HERE/dist" && ls assets/*.js assets/*.css); do
    local_=$(sha256sum "$HERE/dist/$f" | cut -d' ' -f1)
    plain=$(curl -sf "$BASE/$f" | sha256sum | cut -d' ' -f1)
    if [[ "$plain" != "$local_" ]]; then
      echo "BYTE MISMATCH on $f (live $plain vs built $local_) — stray working-tree change or failed rsync" >&2
      exit 1
    fi
    gz=$(curl -sf --compressed "$BASE/$f" | sha256sum | cut -d' ' -f1)
    if [[ "$gz" != "$local_" ]]; then
      echo "STALE GZIP SIDECAR on $f — browsers receive $gz, built $local_." >&2
      echo "Regenerate it: ssh $HOST \"cd $DOCROOT && gzip -kf9 $f\"" >&2
      exit 1
    fi
    echo "verified $f (plain + gzip)"
  done
  echo "deploy verified."
else
  echo "https not answering yet (first deploy: run the nginx/certbot step)." >&2
fi
