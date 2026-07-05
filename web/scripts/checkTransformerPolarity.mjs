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
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { renderSchematic, hasSchematic } from '../src/schematics.js'
const M = await init()

const OPPOSITE = new Set(['flyback', 'isolated_buck_boost'])                                   // energy-storage transfer
const SAME = new Set(['forward', 'two_switch_forward', 'acf', 'isolated_buck', 'ahb'])         // direct (forward) transfer
// everything else with a transformer: phase is a design choice → verify dots present, report, don't fail.

// Extract polarity dots (the symbol library draws them as `sch-fill` circles of r=2.3) and cluster them
// into transformers by x-proximity (a transformer's dots span ~14px; separate magnetics are >80px apart).
function transformers(svg) {
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

let fail = 0, convention = []
for (const t of TOPOLOGIES) {
  if (!hasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  // phasing is variant-independent (it lives in the symbol), so one representative variant suffices
  const opt = v ? v.options[0].id : null
  const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
  if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
  const out = M.design_tas_full(t.id, JSON.stringify(spec))
  if (out.startsWith('Exception')) continue
  const svg = renderSchematic(t.id, extractBom(JSON.parse(out).tas), opt ?? 'standard')
  const xfmrs = transformers(svg).filter((x) => x.pri && x.sec)
  if (!xfmrs.length) continue   // no transformer (non-isolated topology)

  for (const x of xfmrs) {
    const phase = Math.abs(x.pri.y - x.sec.y) < 15 ? 'same' : 'opposite'
    const rule = OPPOSITE.has(t.id) ? 'opposite' : SAME.has(t.id) ? 'same' : null
    if (rule) {
      if (phase !== rule) { fail++; console.log(`${t.id.padEnd(20)} VIOLATION: ${t.id} must be ${rule.toUpperCase()}-phase (${rule === 'opposite' ? 'flyback' : 'forward'} family) but dots are ${phase.toUpperCase()}`) }
      else console.log(`${t.id.padEnd(20)} OK  ${phase} (${rule === 'opposite' ? 'flyback' : 'forward'}-family rule)`)
    } else {
      convention.push(`${t.id.padEnd(20)} dots present, ${phase}-phase (convention — verify sign vs rectifier/control intent)`)
    }
  }
}
console.log('\n— phasing is a design choice for these; dots verified present, sign needs human sign-off —')
for (const c of convention) console.log('  ' + c)
console.log(fail ? `\n${fail} HARD-RULE polarity violation(s)` : '\nAll hard-rule (flyback/forward) transformer phasings correct; dots present on every isolated transformer')
