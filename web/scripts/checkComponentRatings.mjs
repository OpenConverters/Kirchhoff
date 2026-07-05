// Component-rating sanity check (run: `node scripts/checkComponentRatings.mjs`).
//
// The connectivity and polarity checkers verify WIRING; neither looks at the numbers in each component's
// designRequirements. That blind spot let a family of rating bugs ship: a Vienna rectifier diode rated
// 405 µA (sized off a line inductor's ~0 DC offset) in a 3 kW rectifier; negative, schema-invalid ratings
// on the inverting isolated_buck_boost output (−24 V cap, −3.57 A diode); PFC divider legs rated 160 W /
// 16 W (V²/R across the full bus instead of the divider share); and sync-FET body diodes emitted with an
// empty requirement block and mis-typed as MOSFETs. This checker guards every one of those classes:
//   • NEGATIVE or non-finite (NaN/Inf) on any rating leaf — schema `minimum: 0` says these can never be < 0.
//   • A power semiconductor whose current rating is absurdly small (< 1 mA) for the converter's power.
//   • Series-divider resistors whose power ratings are inconsistent with their resistances: two resistors
//     meeting at one node carry the SAME current, so P/R (= I²) must match. This catches the class where a
//     divider leg is rated V²/R across the FULL bus (a 1 kΩ leg → 160 W) instead of its divider share: its
//     P/R is then orders of magnitude off its partner's (Rv1 1.6e-5 vs a buggy Rv2 0.16). A bare-fraction
//     "> converter power" cap would MISS this (160 W < a 300 W converter), so the consistency test is used.
//   • A semiconductor with no deviceType (diode/mosfet) that is not tagged role:"bodyDiode" — an
//     un-orderable, mis-classifiable part.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
const M = await init()

// rating leaves whose value must be ≥ 0 (schema minimum:0), searched recursively
const RATING_KEYS = /voltage|current|power|resistance|capacitance|inductance|forward|reverse|rated|maximum|rds|drainSource/i
const CURRENT_KEYS = /ratedForwardCurrent|ratedCurrent|ratedDrainSourceCurrent|ratedContinuousCurrent/

// collect every numeric leaf (INCLUDING non-finite NaN/Inf, so the guard can flag them)
function leaves(obj, path = '') {
  const out = []
  if (typeof obj === 'number') return [[path, obj]]
  if (obj && typeof obj === 'object') for (const [k, v] of Object.entries(obj)) out.push(...leaves(v, path ? path + '.' + k : k))
  return out
}


let flagged = 0
for (const t of TOPOLOGIES) {
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) continue
    const tas = JSON.parse(out).tas
    // converter output power (sum of operating-point output powers) sets the sanity bands
    const op = tas.inputs?.operatingPoints?.[0]
    const outPower = (op?.outputs ?? []).reduce((s, o) => s + (o.power ?? 0), 0) || 1
    const problems = []
    const resR = new Map()   // resistor ref -> { R, P } for the series-divider consistency check
    const kind = new Map()   // ref -> 'resistor' | 'power' | 'control'  (for classifying divider nodes)
    for (const st of tas.topology?.stages ?? []) {
      for (const c of st.circuit?.components ?? []) {
        const dr = c.data?.inputs?.designRequirements
        kind.set(c.name, c.data?.resistor !== undefined ? 'resistor'
          : (c.data?.capacitor !== undefined || c.data?.semiconductor !== undefined || c.data?.magnetic !== undefined) ? 'power' : 'control')
        if (!dr) continue
        const isSemi = c.data?.semiconductor !== undefined
        const isRes = c.data?.resistor !== undefined
        // (1) negative / non-finite (NaN/Inf) rating leaves
        for (const [p, val] of leaves(dr)) {
          if (!RATING_KEYS.test(p)) continue
          if (!isFinite(val)) problems.push(`${c.name}.${p} = ${val} (non-finite rating)`)
          else if (val < 0) problems.push(`${c.name}.${p} = ${val} (negative — schema minimum:0)`)
        }
        // (2) power semiconductor with an absurdly small current rating
        if (isSemi && outPower > 5) for (const [p, val] of leaves(dr)) {
          if (CURRENT_KEYS.test(p.split('.').pop()) && val > 0 && val < 1e-3)
            problems.push(`${c.name}.${p} = ${(val * 1e6).toFixed(0)} µA (absurdly small for a ${outPower.toFixed(0)} W converter)`)
        }
        // (3) semiconductor with no deviceType and not a tagged body diode → un-orderable / mis-typed
        if (isSemi && dr.deviceType === undefined && dr.role !== 'bodyDiode')
          problems.push(`${c.name} semiconductor has no deviceType and no role:bodyDiode (mis-classified in BOM)`)
        if (isRes && typeof dr.powerRating === 'number' && typeof dr.resistance?.nominal === 'number')
          resR.set(c.name, { R: dr.resistance.nominal, P: dr.powerRating })
      }
    }
    // (4) series-divider consistency. A divider MIDPOINT is an internal node connecting exactly two
    // resistors and only high-impedance control/sense inputs (no rail port, no other power component) —
    // so the SAME current flows through both and P/R (= I²) must match. (A power rail like gnd/vout, by
    // contrast, ties many unrelated resistors and is skipped.) A leg mis-rated at V²/R across the full bus
    // is orders of magnitude off its partner and trips this.
    for (const st of tas.topology?.stages ?? []) {
      for (const conn of st.circuit?.connections ?? []) {
        const eps = conn.endpoints ?? []
        if (eps.some((e) => e.port !== undefined)) continue                          // touches a rail/port
        if (eps.some((e) => e.component !== undefined && kind.get(e.component) === 'power')) continue  // a cap/switch/magnetic sits here → not a clean divider tap
        const rrefs = [...new Set(eps.filter((e) => e.component !== undefined && kind.get(e.component) === 'resistor').map((e) => e.component))]
        if (rrefs.length !== 2) continue
        const rs = rrefs.map((n) => resR.get(n)).filter((r) => r && r.R > 0 && r.P > 0)
        if (rs.length !== 2) continue
        const i2 = rs.map((r) => r.P / r.R)
        if (Math.max(...i2) / Math.min(...i2) > 25)                                   // >5× current mismatch
          problems.push(`divider mismatch at ${conn.name}: ${rrefs.map((n) => `${n}(${resR.get(n).P.toFixed(3)}W/${resR.get(n).R}Ω)`).join(' vs ')} — power inconsistent with resistance (same series current ⇒ P∝R)`)
      }
    }
    if (problems.length) { flagged++; console.log((t.id + (opt ? '/' + opt : '')).padEnd(26), problems.join('  |  ')) }
  }
}
console.log(flagged ? `\n${flagged} topology/variant combos with rating problems` : '\nAll component ratings sane (no negatives, no absurd current/power, every semiconductor typed)')
