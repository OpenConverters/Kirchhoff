// Transformer DOT-POLARITY / winding-phasing check (run: `node scripts/checkTransformerPolarity.mjs`).
//
// The companion checkSchematicNets.mjs verifies CONNECTIVITY (which node each winding end reaches) but is
// blind to PHASING: the TAS `turnsRatio` is unsigned, so the netlist carries no dot information. A swapped
// dot passes every net check yet inverts the output / saturates the core. Phasing therefore can't be
// derived from the netlist — it must be checked against DOMAIN RULES about how each converter family
// transfers energy, applied to the polarity dots the symbol library actually draws.
//
// Two rules are HARD physics (a violation is a real bug, and this tool fails on it):
//   • Flyback family (energy stored during on-time, released to the secondary during off-time) → the
//     primary and secondary dots sit on OPPOSITE winding ends. [flyback, isolated_buck_boost]
//   • Forward family (energy transferred directly during on-time, secondary diode conducts in phase) →
//     primary and secondary dots sit on the SAME end. [forward, two_switch_forward, acf, isolated_buck, ahb]
//
// For every OTHER isolated topology (bridges, resonant tanks, push-pull, DAB, Weinberg) the primary↔secondary
// phase is a design/rectifier-scheme CHOICE, not a physical law. For those this tool only asserts that the
// dots are PRESENT and well-formed (one per winding side) and reports the observed phase — the actual sign
// must be confirmed by a human against the control/rectification intent. That residual is the known limit
// of automation here; it is surfaced, not hidden.
import init from '../../build-wasm-ng/kirchhoff.js'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
const M = await init()

const OPPOSITE = new Set(['flyback', 'isolated_buck_boost'])                                   // energy-storage transfer
const SAME = new Set(['forward', 'two_switch_forward', 'acf', 'isolated_buck', 'ahb'])         // direct (forward) transfer
// everything else with a transformer: phase is a design choice → verify dots present, report, don't fail.

// Extract polarity dots (the symbol library draws them as `sch-fill` circles of r=2.3) and cluster them
// into transformers by x-proximity (a transformer's dots span ~14px; separate magnetics are >80px apart).
export function transformers(svg) {
  const dots = [...svg.matchAll(/<circle class="sch-fill" cx="([\d.]+)" cy="([\d.]+)" r="2\.3"\/>/g)]
    .map((m) => ({ x: +m[1], y: +m[2] })).sort((a, b) => a.x - b.x)
  const groups = []
  for (const d of dots) {
    const g = groups[groups.length - 1]
    if (g && d.x - g.maxx < 40) { g.dots.push(d); g.maxx = Math.max(g.maxx, d.x) }
    else groups.push({ dots: [d], maxx: d.x })
  }
  return groups.map((g) => {
    const cx = g.dots.reduce((s, d) => s + d.x, 0) / g.dots.length
    const left = g.dots.filter((d) => d.x < cx), right = g.dots.filter((d) => d.x >= cx)
    const top = (a) => a.length ? a.reduce((m, d) => (d.y < m.y ? d : m)) : null
    return { pri: top(left), sec: top(right), count: g.dots.length }
  })
}

// The transformers the CIAS actually carries — a magnetic with turns ratios. Asked of the netlist, not
// of the picture, so a transformer drawn WITHOUT dots cannot pass as "no transformer here".
const ciasTransformers = (tas) => {
  const out = []
  for (const st of tas.topology?.stages ?? []) for (const c of st.circuit?.components ?? [])
    if (c.data?.magnetic !== undefined && (c.data?.inputs?.designRequirements?.turnsRatios?.length ?? 0) > 0) out.push(c.name)
  return out
}

// Importable so the floor below can be shown to fire on a drawing with its dots removed.
const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])

let fail = 0, checked = 0, dotted = 0, convention = []
if (isMain) for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  // EVERY variant, not just the first. The phasing rule lives in the symbol, but WHICH symbol gets drawn
  // does not: centre-tapped and current-doubler secondaries are different drawings from the full-bridge
  // one. Testing options[0] alone left 14 of the 29 isolated combos unexamined — including ahb's other
  // two variants, which are in the HARD-RULE set, so a physics rule went unenforced on them.
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = `${t.id}${opt ? '/' + opt : ''}`
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${key}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    const { svg } = renderForAudit(t.id, tas, opt ?? 'standard')
    checked++
    const want = ciasTransformers(tas)
    const xfmrs = transformers(svg).filter((x) => x.pri && x.sec)
    // THE FLOOR. Previously a drawing whose dots did not cluster into a pri/sec pair fell through the
    // same `continue` as a non-isolated topology, so "dots present on every isolated transformer" was a
    // claim the gate could not have falsified. Compare against the netlist instead.
    if (xfmrs.length < want.length) {
      fail++
      console.log(`${key.padEnd(24)} MISSING DOTS: the CIAS carries ${want.length} transformer(s) (${want.join(', ')}) ` +
                  `but only ${xfmrs.length} drawn winding pair(s) carry polarity dots`)
      continue
    }
    dotted += xfmrs.length
    for (const x of xfmrs) {
      const phase = Math.abs(x.pri.y - x.sec.y) < 15 ? 'same' : 'opposite'
      const rule = OPPOSITE.has(t.id) ? 'opposite' : SAME.has(t.id) ? 'same' : null
      if (rule) {
        if (phase !== rule) { fail++; console.log(`${key.padEnd(24)} VIOLATION: must be ${rule.toUpperCase()}-phase (${rule === 'opposite' ? 'flyback' : 'forward'} family) but dots are ${phase.toUpperCase()}`) }
        else console.log(`${key.padEnd(24)} OK  ${phase} (${rule === 'opposite' ? 'flyback' : 'forward'}-family rule)`)
      } else {
        convention.push(`${key.padEnd(24)} dots present, ${phase}-phase (convention — verify sign vs rectifier/control intent)`)
      }
    }
  }
}
if (isMain) {
console.log('\n— phasing is a design choice for these; dots verified present, sign needs human sign-off —')
for (const c of convention) console.log('  ' + c)
if (!checked) throw new Error('checkTransformerPolarity inspected 0 schematics')
if (!dotted && !fail) throw new Error(`checkTransformerPolarity inspected ${checked} schematics and found 0 dotted windings — it tested nothing`)
console.log(fail
  ? `\n${fail} polarity finding(s)`
  : `\nAll hard-rule (flyback/forward) phasings correct across ${checked} topology/variant combos; ` +
    `every transformer the CIAS carries is drawn with polarity dots (${dotted} dotted winding pairs)`)
process.exit(fail ? 1 : 0)   // a gate that cannot fail is not a gate
}
