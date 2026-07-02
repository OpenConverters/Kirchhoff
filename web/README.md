# Kirchhoff web — converter design bench

A static, fully client-side frontend for the Kirchhoff engine: pick one of the
24 implemented topologies, enter the spec, and SOLVE runs the analytical
engine **in WebAssembly in the browser** — no server, nothing leaves the page.

Visual identity: sibling of the Heaviside bench UI (same instrument
discipline — graticule, mono readouts, dark panels) but a different
instrument: amber-phosphor schematic-capture workstation instead of the aqua
CRT oscilloscope.

## Features

- **Topology picker** — all 24 engine topologies grouped by family, with
  working presets per topology (AC input + line frequency for PFC/Vienna,
  dual outputs for the isolated bucks…). Roadmap topologies shown greyed.
- **Spec form** — Vin range, outputs (multi-output supported), fs, and an
  advanced section: efficiency, isolation, ambient, extra operating points.
- **Schematic tab** — hand-drawn SVG power-path sketch (11 topologies so
  far), annotated with the designed values; every component is a clickable
  hotspot. MOSFET geometry ported from
  [chris-pikul/electronic-symbols](https://github.com/chris-pikul/electronic-symbols)
  (MIT).
- **BOM tab** — every TAS component with kind, stage, designed value and
  rating requirements; click for the full requirement set (this is exactly
  what a TAS-DB part matcher will consume later).
- **Waveforms tab** — a unified picker over **every component**: magnetics
  (per-winding current/voltage) *and* switches, diodes, capacitors and
  resistors (per-component V/I). Magnetic windings are synthesized 1:1 from
  the MAS processed descriptors (a JS port of MKF `WaveformProcessor::
  create_waveform`) or measured by ngspice; non-magnetic devices come from a
  single ngspice run (`component_waveforms`) that reads every device's
  terminal current via `.options savecurrents` and its voltage from the node
  differences — so a MOSFET shows its V_DS + switch current, a diode its V_AK
  + forward current, a cap its ripple voltage + current. Includes a MAS-Inputs
  JSON download for the MagneticAdviser.
- **Click any part** (schematic hotspot or BOM row) → the drawer shows its
  requirements *and* its simulated waveforms.
- **Diagnostics tab** — CCM/DCM, duty, per-OP winding stress table,
  capacitor ratings, computed magnetics.
- **Netlist tab** — generate/copy/download the ngspice or LTspice deck of
  the exact design (fidelity: ideal / datasheet / MKF models), with stop
  time and max step overrides.

## Develop / build

```bash
npm install
npm run dev      # syncs kirchhoff.js from ../build-wasm-kh, serves on :5173
npm run build    # static site in dist/ — host anywhere
```

`public/kirchhoff.js` is the single-file Emscripten build (WASM embedded,
ngspice linked in, ~14 MB) copied from `../build-wasm-ng/` by the
`sync-wasm` script. To rebuild it:

```bash
# 1. libngspice for WASM (once):
source ~/emsdk/emsdk_env.sh && ../deps/ngspice-wasm/build-wasm.sh
# 2. the Kirchhoff module:
cmake -S .. -B ../build-wasm-ng -DENABLE_NGSPICE=ON \
  -DNGSPICE_LIB=$PWD/../deps/ngspice-wasm/ngspice-42/release-wasm/src/.libs/libngspice.a \
  -DNGSPICE_INCLUDE_DIR=$PWD/../deps/ngspice-wasm/ngspice-42/src/include
cmake --build ../build-wasm-ng --target libKirchhoff -j
```

## Engines

Both engines run **in the browser**, inside a Web Worker (`src/worker.js`)
so multi-second transients never freeze the UI. **ngspice is the default**;
the transient runs `settlePeriods` (default 100) switching periods before
the `showPeriods` (default 5) displayed ones — both in Advanced, flowing to
the engine via `spec.config.tranStopTime` (DC topologies; PFC/Vienna manage
line-cycle-scale windows themselves). Slow converters (large output caps)
may need 400+ settle periods to fully converge.

- **analytical** — instant; full waveforms for every captured magnetic come
  from the engine's `analyticalWaveforms` (the `design_tas_full` /
  `process_converter` out-of-band capture), including the resonant /
  phase-shift `custom`-label families.
- **ngspice** — real libngspice-42 compiled to WASM and linked into
  `kirchhoff.js` (see `deps/ngspice-wasm/build-wasm.sh`). The engine
  dropdown runs the whole solve through the simulator (~1–5 s); the
  waveforms tab also has a per-magnetic "▶ ngspice this magnetic" button.
  The ngspice extraction rebuilds winding *currents* from the transient;
  voltages remain analytical.

## Known gaps / next steps

- **TAS DB part suggestions**: the part drawer has the placeholder; the
  requirement JSON shown there is the intended query payload.
- Schematic sketches for the remaining topologies (acf, weinberg, psfb,
  pshb, src, cllc, clllc, isolated bucks, pfc, vienna, two-switch forward).
- Builders design from `operatingPoints[0]` only — extra operating points
  in the form are carried in the TAS but don't produce extra stress rows yet.
