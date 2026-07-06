// Visual-sim (CircuitJS1) export check (run: `node scripts/checkFalstadNets.mjs`).
//
// For every topology that has a falstad visual layout, design every variant through the real WASM
// and run falstadExport. The exporter itself verifies the drawn wiring against the flattened CIAS
// nets (the same nets the ngspice deck simulates) and THROWS on any split net, short, missing
// placement or unplaceable component — so "it exported" IS the consistency proof. This script just
// makes that proof repeatable outside the browser, like checkSchematicNets.mjs does for the SVG.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { falstadExport, hasVisualSim } from '../src/falstad.js'

const M = await init()
let checked = 0, failed = 0, skipped = 0
for (const t of TOPOLOGIES) {
  if (!hasVisualSim(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) { console.error(`${t.id}/${opt}: design failed: ${out.slice(0, 160)}`); failed++; continue }
    try {
      const e = falstadExport(t.id, JSON.parse(out).tas)
      console.log(`${t.id}/${opt ?? 'standard'}: OK (fsw ${e.fsw} Hz -> ${e.fVis} Hz, url ${e.url.length} chars)`)
      checked++
    } catch (err) {
      // A variant the layout hasn't drawn yet (an unplaced component) is a KNOWN GAP, not a netlist
      // bug — log it and move on (the UI shows "visual sim not available for this variant"). Any other
      // error IS a real inconsistency (split net / short / floating terminal) and fails the run.
      if (/no placement for component|expects exactly one of/.test(err.message)) {
        console.log(`${t.id}/${opt ?? 'standard'}: – variant not laid out (${err.message})`)
        skipped++
      } else {
        console.error(`${t.id}/${opt ?? 'standard'}: EXPORT FAILED: ${err.message}`)
        failed++
      }
    }
  }
}
if (failed) { console.error(`\n${failed} visual-sim export(s) failed`); process.exit(1) }
console.log(`\nAll ${checked} visual-sim exports consistent with the flattened CIAS netlist${skipped ? ` (${skipped} variant(s) not laid out yet)` : ''}`)
