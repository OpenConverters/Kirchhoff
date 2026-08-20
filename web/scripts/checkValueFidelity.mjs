// Is the number PRINTED beside a part the number the simulation actually uses? (run: `node scripts/checkValueFidelity.mjs`)
//
// checkEquivalence asserts that CIAS, schematic, visual sim and ngspice deck contain the SAME PARTS — it
// compares refdes sets and instance names. It never compares a single number. checkSchematicValues asks
// only whether a drawn quantity is positive and finite. So a value could be printed in the wrong SI
// prefix, formatted from the wrong field, or bound to the neighbouring row, and the drawing would state
// one capacitance while the transient ran another — with every gate green. A reader takes the printed
// number to the bench; it has to be the number the design means.
//
// Covers R/L/C (deck instance "<letter>X") AND the magnetics, whose deck instances are per-winding
// ("L<ref>_pri", "L<ref>_sec1", ... coupled by K lines): the printed inductance must be the primary's
// simulated henries, and every printed turns ratio must be sqrt(Lpri/Lsec) of a simulated secondary —
// the drawing de-duplicates repeated identical ratios (bom.js headlineValue), so the comparison is
// set-to-set, not list-to-list. Until this half existed, all 95 magnetic labels (every L and T on the
// 39 schematics) fell through the instance-name match and were silently skipped — a "0 problems" that
// had compared none of them. A MOSFET's "RDS(on) <= x" stays out of scope on purpose: it is a device
// RATING, while the deck idealises the switch (.model SW Ron=0.01), so there is no simulated Rds to
// hold the label to.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const PREFIX = { y: 1e-24, z: 1e-21, a: 1e-18, f: 1e-15, p: 1e-12, n: 1e-9, µ: 1e-6, u: 1e-6, m: 1e-3,
                 k: 1e3, M: 1e6, G: 1e9, T: 1e12 }
// The terminator is a lookahead, not \b: Ω is not a word character, so \b after it never matches and
// every resistor value would be skipped (the bug that once made checkSchematicValues look thorough).
const QTY = /(-?\d+(?:\.\d+)?)\s*([yzafpnµumkMGT])?(Hz|F|H|Ω|V|A|W|s)(?![A-Za-z0-9])/g
export const quantities = (text) => [...text.matchAll(QTY)]
  .map((m) => ({ value: Number(m[1]) * (m[2] ? PREFIX[m[2]] : 1), unit: m[3], text: m[0] }))

// SPICE numbers are case-insensitive and MEG (not M) is 1e6 — M alone is milli. Getting this wrong
// would manufacture a 10^9 "mismatch" on every megohm and hide real ones behind the noise.
export function spiceNumber(s) {
  const m = /^([-+]?[\d.]+(?:[eE][-+]?\d+)?)\s*(meg|mil|[tgkmunpf])?/i.exec(String(s).trim())
  if (!m) return null
  const u = (m[2] ?? '').toLowerCase()
  const k = u === 'meg' ? 1e6 : u === 'mil' ? 25.4e-6 : u === '' ? 1
    : { t: 1e12, g: 1e9, k: 1e3, m: 1e-3, u: 1e-6, n: 1e-9, p: 1e-12, f: 1e-15 }[u]
  return +m[1] * k
}

const UNIT_OF = { R: 'Ω', L: 'H', C: 'F' }
const TOL = 0.02          // the drawing rounds for legibility; 2% is far tighter than any prefix slip

// { svg, deck } -> [problems]. Also returns how many values it actually compared, because a rule that
// silently compares nothing reports success just as loudly as one that compares everything.
export function valueFidelity(svg, deck) {
  const problems = []
  let compared = 0
  const deckVal = new Map()
  const coils = new Map()          // magnetic ref -> Map(winding -> simulated henries)
  for (const line of deck.split('\n')) {
    const tk = line.trim().split(/\s+/)
    if (!/^[RLC][A-Za-z0-9_]*$/.test(tk[0] ?? '') || tk.length < 4) continue
    const n = spiceNumber(tk[3])
    if (n === null) continue
    const w = /^L(.+)_(pri|sec\d+)$/.exec(tk[0])
    if (w) (coils.get(w[1]) ?? coils.set(w[1], new Map()).get(w[1])).set(w[2], n)
    else deckVal.set(tk[0], { v: n, type: tk[0][0] })
  }
  for (const g of svg.split('<g class="sch-hot').slice(1)) {
    const ref = g.match(/data-ref="([^"]+)"/)?.[1]
    if (!ref) continue
    const texts0 = [...g.matchAll(/<text[^>]*>([^<]*)<\/text>/g)].map((m) => m[1])
    const mag = coils.get(ref)
    if (mag) {
      const pri = mag.get('pri')
      const printedL = texts0.flatMap(quantities).find((q) => q.unit === 'H')
      if (pri === undefined) { problems.push(`${ref}: deck windings [${[...mag.keys()]}] carry no primary`); continue }
      if (!printedL) { problems.push(`${ref}: the deck simulates L${ref}_pri=${pri} H but the drawing prints no H value (${texts0.join(' | ')})`); continue }
      compared++
      const relL = Math.abs(printedL.value - pri) / pri
      if (relL > TOL) problems.push(`${ref}: the drawing says ${printedL.text} but the deck simulates L${ref}_pri=${pri} H (${(relL * 100).toFixed(0)}% apart)`)
      // the ratios, when the label prints any: each drawn n must be a simulated sqrt(Lpri/Lsec) and
      // every DISTINCT simulated ratio must be drawn (the label de-duplicates repeats, so sets match)
      const nTxt = texts0.join(' ').match(/n=([\d.]+(?:\s*\/\s*[\d.]+)*)/)
      const secs = [...mag].filter(([k]) => k !== 'pri').map(([, L]) => Math.sqrt(pri / L))
      if (nTxt) {
        const drawn = nTxt[1].split('/').map((x) => Number(x.trim()))
        const simSet = secs.filter((r, i) => secs.findIndex((q) => Math.abs(q - r) / r <= TOL) === i)
        for (const d of drawn) {
          compared++
          if (!secs.some((r) => Math.abs(d - r) / r <= TOL))
            problems.push(`${ref}: the drawing says n=${d} but no simulated winding pair has that ratio (deck: ${secs.map((r) => r.toFixed(3)).join(', ')})`)
        }
        for (const r of simSet)
          if (!drawn.some((d) => Math.abs(d - r) / r <= TOL))
            problems.push(`${ref}: the deck simulates a winding ratio ${r.toFixed(3)} the label n=${nTxt[1]} does not print`)
      } else if (secs.length) {
        problems.push(`${ref}: the deck couples ${secs.length} secondary winding(s) but the drawing prints no turns ratio`)
      }
      continue
    }
    let inst = null
    for (const [name, d] of deckVal)
      if (name === ref || name.slice(1) === ref || name === d.type + ref) { inst = { name, ...d }; break }
    if (!inst) continue
    const want = UNIT_OF[inst.type]
    const texts = texts0
    const printed = texts.flatMap(quantities).find((q) => q.unit === want)
    if (!printed) {
      problems.push(`${ref}: the deck simulates ${inst.name}=${inst.v} ${want} but the drawing prints no ${want} value (${texts.join(' | ')})`)
      continue
    }
    compared++
    const rel = Math.abs(printed.value - inst.v) / Math.max(Math.abs(inst.v), 1e-30)
    if (rel > TOL)
      problems.push(`${ref}: the drawing says ${printed.text} but the deck simulates ${inst.v} ${want} (${(rel * 100).toFixed(0)}% apart)`)
  }
  return { problems, compared }
}

const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])
if (isMain) {
  const M = await init()
  const only = process.argv[2]
  let flagged = 0, checked = 0, compared = 0
  for (const t of TOPOLOGIES) {
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
      const { svg } = renderForAudit(t.id, tas, opt ?? 'standard')
      const deck = M.generate_ngspice_circuit(JSON.stringify(tas), JSON.stringify({ origin: 'REQUIREMENTS' }))
      if (deck.startsWith('Exception')) throw new Error(`${key}: deck generation failed: ${deck.slice(0, 120)}`)
      const r = valueFidelity(svg, deck)
      checked++; compared += r.compared
      if (r.problems.length) { flagged++; console.log(`\n== ${key}`); for (const p of r.problems) console.log('   ' + p) }
    }
  }
  if (!checked) throw new Error('checkValueFidelity examined 0 schematics')
  if (!compared) throw new Error(`checkValueFidelity examined ${checked} schematics but compared 0 values — it tested nothing`)
  console.log(flagged
    ? `\n${flagged} schematic(s) printing a value the simulation does not use`
    : `\nAll ${checked} schematics: every one of the ${compared} printed R/L/C values is the value the deck simulates`)
  process.exit(flagged ? 1 : 0)   // a gate that cannot fail is not a gate
}
