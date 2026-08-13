# Kirchhoff as an MCP App

Exposes the Kirchhoff power-converter engine as [MCP](https://modelcontextprotocol.io)
tools, plus the web app's own **CIAS schematic** as an
[MCP Apps](https://modelcontextprotocol.io/extensions/apps/build) (SEP-1865) UI
resource.

The point: an engineer asks a chat assistant "design me a 48→12 V 24 W flyback",
gets a **schematic they can click**, and clicking a component sources real
catalogue parts for it — no re-explaining, no copy-pasting a netlist.

Hosts that understand MCP Apps (Claude web/Desktop, ChatGPT, VS Code Copilot)
render the widgets. Hosts that speak only plain MCP still get the tools and the
text answer — the `_meta` they don't understand is ignored.

## Build and run

```bash
# the engine
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && make -C build PyKirchhoff -j6

# the widgets
cd mcp && npm install && npm run build

python3 server.py                  # streamable HTTP on 127.0.0.1:8401/mcp
```

Python needs `mcp`, `uvicorn`, `starlette`, and a built `PyKirchhoff`. The widget
bundle must exist before the server can serve the UI resource — it raises rather
than serving a blank page.

## Use it from Claude (web or Desktop)

Claude reaches your server over the public internet, so a localhost port needs a
tunnel. Custom connectors require a paid plan (Pro, Max, or Team). Hertz uses
8400 and Kirchhoff 8401, so both can run at once as two separate connectors.

1. **Start the tunnel first** — you need its hostname before starting the server:

   ```bash
   npx cloudflared tunnel --url http://localhost:8401
   ```

2. **Start the server with that hostname allowed:**

   ```bash
   KELVIN_TAS_DATA_DIR=/path/to/TAS/data \
   KIRCHHOFF_PUBLIC_HOST=<random>.trycloudflare.com python3 server.py
   ```

   A pasted URL is fine — the scheme and path are stripped for you. `KIRCHHOFF_ALLOW_ANY_HOST=1` skips
   the check for throwaway tunnels (fine for a laptop, not for anything public).

   > **Why:** the MCP SDK enables DNS-rebinding protection by default and
   > rejects unrecognised `Host` headers with a bare `421 Invalid Host header`.
   > Behind a tunnel the Host is the *public* name, so every request 421s.
   > Claude then can't speak MCP, falls back to probing for OAuth, and reports
   > **"Couldn't register with <name>'s sign-in service"** — an authentication
   > error for what is actually a Host-header rejection. If you see that
   > message, `curl -i https://<tunnel>/mcp` before touching anything OAuth.

3. **Add the connector.** Claude → **Settings** → **Connectors** → **Add custom
   connector**, pasting the tunnel URL **with the `/mcp` path**. The bare
   hostname 404s, which produces the same misleading OAuth error. A connector
   added earlier serves a *cached* tool list — remove and re-add it after the
   tool surface changes.

4. **Ask.** For example:

   > Design a 48 V to 12 V, 24 W flyback at 100 kHz.

   Claude calls `design_converter`, the schematic renders inline, and clicking a
   component sources real catalogue parts for it and pushes that selection into
   its context for the next turn.

Good follow-ups once the schematic is up: *"what does Q1 actually see?"*
(`component_waveforms`), *"give me the ngspice deck"*, *"what else could replace
that controller?"* (`cross_reference`).

Unlike a browser-resident engine, nothing here needs WebAssembly in the widget
sandbox — the engine runs server-side in PyKirchhoff — so the host's CSP posture
on `wasm-unsafe-eval` is irrelevant to these widgets.

## Tools

Nineteen, curated to follow the flow the web app already has rather than
mirroring all ~47 engine entry points one-to-one (past ~25 tools, hosts start
picking the wrong one). All 24 topologies sit behind one `design_converter`.

| Group | Tools |
|---|---|
| Design | `list_topologies`, `design_converter`, `converter_diagnostics`, `realize_tas` |
| Simulate | `export_netlist`, `simulate`, `simulate_ac`, `component_waveforms` |
| Magnetics | `magnetic_inputs`, `topology_waveforms`, `operating_point` |
| Sourcing | `select_parts`, `bind_part`, `cross_reference` |
| EMI components | `design_cmc`, `design_dmc`, `propose_dmc`, `verify_dmc`, `design_current_transformer` |

`simulate` returns per-vector **statistics** (min/max/average/final), because
that is what the engine measures as the deck runs — a 150k-point transient never
crosses the wire. `component_waveforms` is the sampled-waveform tool, and the one
that charts.

Every SPICE run goes through Kirchhoff's **in-process libngspice**; the installed
`ngspice` binary is never invoked and is not a runtime dependency.

## Part sourcing

`select_parts` ranks real parts from the TAS catalogue. Point it at the
catalogue directory with `data_dir`, or set `KELVIN_TAS_DATA_DIR`. With neither
it refuses loudly rather than returning an empty list — "no candidates" and
"nobody looked" must not read the same.

```bash
export KELVIN_TAS_DATA_DIR=/path/to/TAS/data
```

## Widgets

Both import the web app's real modules out of `../web/src` rather than
reimplementing them, so there is one definition and two surfaces:

- **`schematic.html`** — `ciasSchematic.js`. The drawing is generated from the
  same CIAS bricks the ngspice deck is generated from, and **verified against the
  same netlist checker**: if the picture would disagree with the circuit it
  throws instead of rendering. Every component carries a `data-ref` hotspot;
  clicking one reports the selection to the model and calls `select_parts` for it
  (a widget-initiated tool call, no LLM turn).
- **`curves.html`** — `WaveformChart.vue`, dual V/A axes, for
  `component_waveforms`.

MCP App resources render in a deny-by-default CSP iframe, so each widget is built
as ONE self-contained file (`vite-plugin-singlefile`), one entry per build.

## Deployment note

The SDK's DNS-rebinding protection rejects an unrecognised `Host` with a bare
`421`, which remote hosts often surface as a misleading sign-in error. Behind a
tunnel or reverse proxy, name the public host in `KIRCHHOFF_PUBLIC_HOST` (or set
`KIRCHHOFF_ALLOW_ANY_HOST=1` for throwaway tunnels).

## Status

Spike. Not yet done: magnetics handoff to the OpenMagnetics adviser (the web app
does this with a cross-origin postMessage round-trip that has no server-side
equivalent yet). File references and auth are done — see below.

## Files and auth

Anything this server reads from disk takes ONE reference argument, the shared convention across
the OpenConverters MCP servers (`mcp/artifacts.py`, identical in Faraday, Hertz and Kirchhoff):

```
/path/to/file            a path on the machine running the server
file:///path/to/file     the same, as a URI
artifact://<id>          resolved against KIRCHHOFF_ARTIFACT_BASE, with KIRCHHOFF_ARTIFACT_TOKEN as a bearer token
https://host/path        fetched as-is
```

One argument rather than a path field and a URI field, so the orchestrator implements the
reference once. Large inputs therefore never travel through the tool arguments — which is to
say, never through the model context.

Auth is **off unless `KIRCHHOFF_AUTH_TOKEN` is set**. Set it and every request must carry
`Authorization: Bearer <token>`, and one that does not gets a plain 401 rather than a redirect.
It is a gate, not an identity: one shared token says the caller is allowed in, not who they
are. Anything needing per-user identity, audit or revocation wants a real IdP in front, and
TLS belongs on the proxy.
