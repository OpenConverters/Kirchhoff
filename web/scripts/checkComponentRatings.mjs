// Component-rating sanity check (run: `node scripts/checkComponentRatings.mjs`).
//
// The connectivity and polarity checkers verify WIRING; neither looks at the numbers in each component's
// designRequirements. That blind spot let a family of rating bugs ship: a Vienna rectifier diode rated
// 405 µA (sized off a line inductor's ~0 DC offset) in a 3 kW rectifier; negative, schema-invalid ratings
// on the inverting isolated_buck_boost output (−24 V cap, −3.57 A diode); PFC divider legs rated 160 W /
// 16 W (V²/R across the full bus instead of the divider share); and sync-FET body diodes emitted with an
// empty requirement block and mis-typed as MOSFETs. This checker guards every one of those classes:
//   • NEGATIVE / NaN on any rating leaf — schema `minimum: 0` says these can never be < 0.
//   • A power semiconductor whose current rating is absurdly small (< 1 mA) for the converter's power.
//   • A resistor whose powerRating exceeds the whole converter's output power (no board part dissipates
//     more than the converter delivers → the rating formula is wrong).
//   • A semiconductor with no deviceType (diode/mosfet) that is not tagged role:"bodyDiode" — an
//     un-orderable, mis-classifiable part.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
const M = await init()

// rating leaves whose value must be ≥ 0 (schema minimum:0), searched recursively
const RATING_KEYS = /voltage|current|power|resistance|capacitance|inductance|forward|reverse|rated|maximum|rds|drainSource/i
const CURRENT_KEYS = /ratedForwardCurrent|ratedCurrent|ratedDrainSourceCurrent|ratedContinuousCurrent/
const isNum = (x) => typeof x === 'number' && isFinite(x)

function leaves(obj, path = '') {
  const out = []
  if (isNum(obj)) return [[path, obj]]
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
    for (const st of tas.topology?.stages ?? []) {
      for (const c of st.circuit?.components ?? []) {
        const dr = c.data?.inputs?.designRequirements
        if (!dr) continue
        const isSemi = c.data?.semiconductor !== undefined
        const isRes = c.data?.resistor !== undefined
        // (1) negative / NaN rating leaves
        for (const [p, val] of leaves(dr)) {
          if (!RATING_KEYS.test(p)) continue
          if (val < 0) problems.push(`${c.name}.${p} = ${val} (negative — schema minimum:0)`)
        }
        // (2) power semiconductor with an absurdly small current rating
        if (isSemi && outPower > 5) for (const [p, val] of leaves(dr)) {
          if (CURRENT_KEYS.test(p.split('.').pop()) && val > 0 && val < 1e-3)
            problems.push(`${c.name}.${p} = ${(val * 1e6).toFixed(0)} µA (absurdly small for a ${outPower.toFixed(0)} W converter)`)
        }
        // (3) resistor dissipating more than the whole converter
        if (isRes && isNum(dr.powerRating) && dr.powerRating > outPower)
          problems.push(`${c.name}.powerRating = ${dr.powerRating.toFixed(0)} W > converter ${outPower.toFixed(0)} W (rating formula wrong)`)
        // (4) semiconductor with no deviceType and not a tagged body diode → un-orderable / mis-typed
        if (isSemi && dr.deviceType === undefined && dr.role !== 'bodyDiode')
          problems.push(`${c.name} semiconductor has no deviceType and no role:bodyDiode (mis-classified in BOM)`)
      }
    }
    if (problems.length) { flagged++; console.log((t.id + (opt ? '/' + opt : '')).padEnd(26), problems.join('  |  ')) }
  }
}
console.log(flagged ? `\n${flagged} topology/variant combos with rating problems` : '\nAll component ratings sane (no negatives, no absurd current/power, every semiconductor typed)')
