import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { cpSync, existsSync, readdirSync, symlinkSync } from 'node:fs'
import { resolve } from 'node:path'

// public/kelvin/ is the Kelvin catalogue data set (shards + NDJSON, ~5.5 GB once its symlinks
// resolve). Vite copied it into dist/ verbatim, which filled the disk during the 2026-11-12 v20
// deploy; it is never deployed from dist/ (deploy-prod.sh rsyncs with --exclude kelvin/ and the
// box serves /kelvin from /cache/kelvin). So copy everything else, and LINK the data dir so
// `vite preview` (the e2e server) still serves it.
function publicWithoutData(dataDir = 'kelvin') {
  let outDir, publicDir
  return {
    name: 'public-without-data',
    apply: 'build',
    configResolved(c) {
      outDir = resolve(c.root, c.build.outDir)
      publicDir = c.publicDir
    },
    closeBundle() {
      for (const entry of readdirSync(publicDir)) {
        if (entry === dataDir) continue
        cpSync(resolve(publicDir, entry), resolve(outDir, entry), { recursive: true, dereference: true })
      }
      if (existsSync(resolve(publicDir, dataDir)))
        symlinkSync(resolve(publicDir, dataDir), resolve(outDir, dataDir), 'dir')
    },
  }
}

// Static SPA: everything (including the WASM engine) is served as plain files,
// so the built dist/ can be hosted anywhere (GitHub Pages, nginx, file://-adjacent).
export default defineConfig({
  base: './',
  plugins: [vue(), publicWithoutData()],
  build: {
    // kirchhoff.js is ~7.7 MB (the whole simulation engine); it is loaded from
    // public/ at runtime, not bundled, so keep rollup quiet about chunk size.
    chunkSizeWarningLimit: 1200,
    copyPublicDir: false,
  },
})
