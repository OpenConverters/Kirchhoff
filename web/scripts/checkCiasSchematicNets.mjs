// CIAS-verified schematic check (run: `node scripts/checkCiasSchematicNets.mjs`).
//
// For every topology + variant, design through the real WASM and call renderVerifiedSchematic — the
// runtime path the app uses. It renders the native CIAS-generated layout where one exists (flyback)
// and the hand-authored SVG otherwise, then runs the shared netlist/isolation checker on the result
// and THROWS on any drift. So "it rendered" IS the proof that every schematic — generated or hand-
// drawn — agrees with the flattened CIAS netlist. Repeatable outside the browser.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { renderVerifiedSchematic } from '../src/ciasSchematic.js'
import { hasSchematic } from '../src/schematics.js'

const M = await init()
let ok = 0, failed = 0
for (const t of TOPOLOGIES) {
  if (!hasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) { console.error(`${t.id}/${opt}: design failed`); failed++; continue }
    try {
      const tas = JSON.parse(out).tas
      const svg = renderVerifiedSchematic(t.id, tas, opt ?? 'standard', extractBom(tas))
      console.log(`${t.id}/${opt ?? 'standard'}: OK (${svg.length} chars)`)
      ok++
    } catch (e) { console.error(`${t.id}/${opt ?? 'standard'}: FAILED: ${e.message}`); failed++ }
  }
}
if (failed) { console.error(`\n${failed} CIAS schematic(s) failed`); process.exit(1) }
console.log(`\nAll ${ok} CIAS-verified schematics consistent with the flattened netlist`)
