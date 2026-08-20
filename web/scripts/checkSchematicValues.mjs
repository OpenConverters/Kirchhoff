// Does the drawing ever STATE something impossible? (run: `node scripts/checkSchematicValues.mjs`)
//
// Every other gate asks where the text sits, what colour it is and what glyphs it is made of. None asks
// whether the number is a thing that can exist. A schematic that prints "0 F" under a capacitor or
// "0 H · n=1 / 0" under a transformer is perfectly placed, perfectly legible and perfectly wrong — and a
// reader has no way to tell it from a real design.
//
// Both of those are reachable today (see ABT #747): a `Pout = 0` buck draws a 0 F output capacitor, and a
// `Vin,min = 0` forward draws a 0 H transformer with a turns ratio of zero, because the engine accepts
// those requirements instead of refusing them. This gate covers the drawings the app actually ships — the
// preset of every topology and variant — so that class can never arrive there unnoticed.
//
//   ZERO      a component drawn with a zero or negative quantity (0 F, 0 H, 0 Ω, 0 V)
//   RATIO     a transformer drawn with a turns ratio of zero (a winding that cannot transfer anything)
//   NONFINITE NaN / Infinity reaching the ink
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { renderForAudit } from '../src/ciasSchematic.js'

import { fileURLToPath } from 'node:url'
import path from 'node:path'
// Importable so the sweep can apply the same rule at every design point — the values are what CHANGE
// with the operating point, so checking them only at the preset is the thinnest possible coverage.
const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])
const M = isMain ? await init() : null

// "12.4 mΩ" / "0 F" / "1.37 mH · n=6.51" → the numbers with their SI prefixes resolved.
const PREFIX = { y: 1e-24, z: 1e-21, a: 1e-18, f: 1e-15, p: 1e-12, n: 1e-9, µ: 1e-6, u: 1e-6, m: 1e-3,
                 k: 1e3, M: 1e6, G: 1e9, T: 1e12 }
const UNIT = /^(Hz|F|H|Ω|V|A|W|s)$/
export function quantities(text) {
  const out = []
  // The terminator is a lookahead, NOT \b. \b is defined against [A-Za-z0-9_], and Ω is none of those —
  // so "100 Ω" has no word boundary after the unit and the original pattern matched NOTHING for ohms.
  // Every resistor value and every "RDS(on) ≤ 80 mΩ" walked past this gate while it reported a count that
  // made it look thorough. Hz is listed before H so the alternation cannot take the H and strand the z.
  for (const m of text.matchAll(/(-?\d+(?:\.\d+)?)\s*([yzafpnµumkMGT])?(Hz|F|H|Ω|V|A|W|s)(?![A-Za-z0-9])/g)) {
    const [, num, prefix, unit] = m
    if (!UNIT.test(unit)) continue
    out.push({ value: Number(num) * (prefix ? PREFIX[prefix] ?? 1 : 1), unit, text: m[0] })
  }
  return out
}

// { svg } -> { problems, checked }: every drawn value must be a quantity a part can have.
export function valueProblems(svg) {
  const problems = []
  let checked = 0
  for (const m of svg.matchAll(/<text class="sch-val"[^>]*>([^<]*)<\/text>/g)) {
    const text = m[1]
    checked++
    if (/NaN|Infinity|undefined|null/.test(text)) { problems.push(`NONFINITE "${text}"`); continue }
    for (const q of quantities(text))
      if (!(q.value > 0)) problems.push(`ZERO "${text}" — ${q.text} is not a quantity a part can have`)
    // "· n=6.51" or "· n=1 / 0": a zero ratio is a winding that transfers nothing.
    const n = text.match(/n=([\d./\s]+)/)?.[1]
    if (n) for (const r of n.split('/'))
      if (r.trim() && !(Number(r) > 0)) problems.push(`RATIO "${text}" — a turns ratio of ${r.trim()}`)
  }
  return { problems: [...new Set(problems)], checked }
}

let flagged = 0, checked = 0
if (isMain) for (const t of TOPOLOGIES) {
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = `${t.id}${opt ? '/' + opt : ''}`
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${key}: design failed: ${out.slice(0, 200)}`)
    const { svg } = renderForAudit(t.id, JSON.parse(out).tas, opt ?? 'standard')
    const r = valueProblems(svg)
    checked += r.checked
    if (r.problems.length) { flagged++; console.log(`\n== ${key}`); for (const p of r.problems) console.log('   ' + p) }
  }
}
if (isMain) {
if (!checked) throw new Error('checkSchematicValues read no value labels at all — nothing was rendered')
console.log(flagged
  ? `\n${flagged} schematic(s) stating a value no component can have`
  : `\nEvery one of the ${checked} values drawn across the 39 schematics is a quantity a part can have`)
process.exit(flagged ? 1 : 0)   // a gate that cannot fail is not a gate
}
