// Is every switch driven, and does a shared drive LABEL mean a shared drive? (run: `node scripts/checkGateDrive.mjs`)
//
// Gate drive is the one part of the drawing no other rule touches. checkSchematicNets excludes gate nets
// outright ("they connect via net-label flags, not wires"), rule G skips pin==='gate', and the only flag
// rule in auditSchematics counts how often a NAME occurs — so a switch drawn with no drive at all, or two
// switches sharing a label they must not share, is invisible to the whole suite.
//
// WHAT A GATE NET IS HERE, and why only ONE direction is a defect. The CIAS puts Q1 and Q4 of a bridge on
// one gate net, and Q1/Q2 of a two-switch forward on one — those switches cannot share a physical node
// (a high-side gate is referenced to its own floating source), so a gate "net" in the CIAS is a control
// SIGNAL identifier, not a conductor. That makes the two directions asymmetric:
//   drawn same, netlist different  → MERGE. A defect, always: the drawing tells the reader one signal
//                                    drives both switches when the design drives them separately. This is
//                                    the copy-paste failure a layout invites — duplicate a leg, forget to
//                                    renumber the flag — and shorting two real driver outputs is how a
//                                    bridge fails shoot-through.
//   drawn different, netlist same  → not a defect. Two gates carrying the SAME waveform are still two
//                                    separate physical gates, and drawing them with their own flags is
//                                    correct. Enforcing the converse would demand the drawing bond two
//                                    floating gate references — and, on the isolated topologies, draw a
//                                    control wire straight across the safety barrier.
// Plus the floor: every drawn gate terminal must actually have a drive stub on it.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { flattenNets } from '../src/cias.js'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const M = await init()
const only = process.argv[2]
// Importable: the rules below are exercised by tests that mutate a real drawing and require each rule to
// fire. A file that exits on import cannot be asked whether its rules can fail at all.
const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])

// The drawn drive of each switch: the dashed control stub standing on its gate terminal, and the signal
// flag at the stub's far end. Read from the finished drawing, never from the layout source.
export function drawnDrive(svg, pins) {
  const stubs = [...svg.matchAll(/<path class="sch-ctl" d="M\s*(-?[\d.]+)\s+(-?[\d.]+)\s*L\s*(-?[\d.]+)\s+(-?[\d.]+)"/g)]
    .map((m) => ({ x: +m[1], y: +m[2], x2: +m[3], y2: +m[4] }))
  const flags = [...svg.matchAll(/<text class="sch-sig"[^>]*x="(-?[\d.]+)"[^>]*y="(-?[\d.]+)"[^>]*>([^<]*)<\/text>/g)]
    .map((m) => ({ x: +m[1], y: +m[2], name: m[3] }))
  const out = new Map(), undriven = []
  for (const g of pins.filter((p) => p.pin === 'gate')) {
    const s = stubs.find((z) => Math.hypot(z.x - g.x, z.y - g.y) <= 3)
    if (!s) { undriven.push(g); continue }
    // the flag belongs to the stub that reaches it; 20 px is the stub's own length plus its text offset
    const f = flags.map((z) => ({ z, d: Math.hypot(z.x - s.x2, z.y - s.y2) })).sort((a, b) => a.d - b.d)[0]
    if (f && f.d <= 20) out.set(g.ref, f.z.name)
    else undriven.push(g)          // a stub going nowhere named is no drive either
  }
  return { drive: out, undriven }
}

let checked = 0, gates = 0, flagged = 0
if (isMain) for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = `${t.id}${opt ? '/' + opt : ''}`
    if (only && only !== key && only !== t.id) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) throw new Error(`${key}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
    checked++
    const pinNet = flattenNets(tas)
    const { drive, undriven } = drawnDrive(svg, pins)
    gates += pins.filter((p) => p.pin === 'gate').length
    const bad = []
    for (const g of undriven) bad.push(`UNDRIVEN ${g.ref}: its gate terminal at (${g.x},${g.y}) carries no named drive`)
    const refs = [...drive.keys()].filter((r) => pinNet.has(`${r}|gate`))
    for (let i = 0; i < refs.length; i++) for (let j = i + 1; j < refs.length; j++) {
      if (drive.get(refs[i]) !== drive.get(refs[j])) continue
      if (pinNet.get(`${refs[i]}|gate`) === pinNet.get(`${refs[j]}|gate`)) continue
      bad.push(`MERGE ${refs[i]} and ${refs[j]} are both drawn as '${drive.get(refs[i])}', but the CIAS drives them from ` +
               `${String(pinNet.get(`${refs[i]}|gate`)).replace(/^[CPX]:/, '')} and ${String(pinNet.get(`${refs[j]}|gate`)).replace(/^[CPX]:/, '')}`)
    }
    if (bad.length) { flagged++; console.log(`\n== ${key}`); for (const b of bad) console.log('   ' + b) }
  }
}
if (isMain) {
if (!checked) throw new Error('checkGateDrive examined 0 schematics')
// A drawing set with no gate terminals would satisfy every rule above without testing one.
if (!gates) throw new Error(`checkGateDrive examined ${checked} schematics and found 0 gate terminals — it tested nothing`)
console.log(flagged
  ? `\n${flagged} combo(s) whose drawn gate drive contradicts the CIAS`
  : `\nAll ${checked} schematics: every one of the ${gates} gate terminals is driven, and no two switches share a drive label the CIAS does not share`)
process.exit(flagged ? 1 : 0)   // a gate that cannot fail is not a gate
}
