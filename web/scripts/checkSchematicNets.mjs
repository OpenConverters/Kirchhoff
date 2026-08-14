// Netlist-vs-drawing connectivity check (run: `node scripts/checkSchematicNets.mjs`).
//
// It FLATTENS the hierarchical TAS (per-stage circuits linked by inter-stage connections) into global
// nets, then verifies the rendered SVG against them:
//   A. Every net's drawn anchor terminals — MOSFET drain/source, diode anode/cathode (unambiguous) — are
//      wire-connected in one piece, or split only into pieces that each carry a ground/port reference
//      (multiple ground symbols = same net by convention; isolated primary/secondary get their own ref).
//   B. No two distinct global nets share a drawn wire (short check).
//   C. MAGNETICS: each transformer/inductor's drawn winding terminals reach exactly the set of global nets
//      its netlist pins sit on — catches a floating winding end, a winding/center-tap wired to the wrong
//      node, or a missing terminal (the connectivity half of "is the magnetic wired right").
//
// This tool drove fixes for EVERY power-connectivity defect it found: fsbb/dab/cllc/clllc floating grounds;
// cllc/clllc broken bridge leg (node_b) and floating VOUT rail; secFB bridge-diode refdes mapping; pshb
// missing primary ground and REVERSED clamp diode (DC1); boost-sync + acf sync-rectifier MOSFETs drawn
// source/drain-swapped (mirrored); and Vienna's bidirectional midpoint switch (cascode → common-source).
//
// WHAT IT STILL CANNOT VERIFY (a pure connectivity check is blind to this — see the companion
// checkTransformerPolarity.mjs which covers it with domain rules):
//   - Transformer DOT POLARITY / winding phasing. The TAS turnsRatio is UNSIGNED, so the netlist carries
//     no phase information; a swapped dot passes every net check yet flips output polarity / saturates the
//     core. Not machine-verifiable from connectivity alone.
// Gate nets are excluded (they connect via net-label flags, not wires).
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
// Renders through the SAME entry point the app uses (renderForAudit). Every topology is generated from
// CIAS now, but the rule stands: measure what the product draws, never a reconstruction of it.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { checkSchematic } from '../src/schematicCheck.js'
const M = await init()

// wireGraph + the connectivity/isolation checks now live in ../src/schematicCheck.js (shared with
// the runtime CIAS generator) so both validate against identical rules.
let flagged = 0, checked = 0
for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${t.id}${opt ? '/' + opt : ''}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
    const problems = checkSchematic({ svg, pins, tas })
    checked++
    if (problems.length) { flagged++; console.log((t.id + (opt ? '/' + opt : '')).padEnd(26), problems.join('  |  ')) }
  }
}

// Report the COUNT, and refuse to report success over nothing: this gate skips a topology with no CIAS
// layout, so "all consistent" over an empty sweep would be a clean bill of health for zero drawings.
if (!checked) throw new Error('checkSchematicNets verified 0 schematics — every topology was skipped')
console.log(flagged
  ? `\n${flagged} topology/variant combos flagged`
  : `\nAll ${checked} schematics: nets + magnetic windings consistent with the flattened netlist`)
process.exit(flagged ? 1 : 0)   // a gate that cannot fail is not a gate
