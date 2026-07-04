// T1 — knob serialization: every knob of every topology, driven through the DOM, lands in the right
// channel (config[key] for CFG, designRequirements.<field> for PIN) with the exact value, and NO
// other knob leaks in. Uses previewSpec() (no solve) so the full matrix runs fast.
import { test } from '@playwright/test'
import { boot, selectTopology, setKnob, clearKnob, knobCatalog, ALL_TOPOLOGIES, expect } from './helpers.js'

// PIN knobs write these designRequirements fields (ComponentRequirements.hpp provided_*()).
const PIN_FIELD = {
  turnsRatio: 'turnsRatios',
  magnetizingInductance: 'magnetizingInductance',
  resonantInductance: 'desiredResonantInductance',
  resonantCapacitance: 'desiredResonantCapacitance',
  seriesInductance: 'desiredSeriesInductance',
}

// A distinct, in-range test value for a knob + the value expected in the spec.
function pick(k) {
  if (k.type === 'bool') return { value: !k.def, expected: !k.def }
  if (k.type === 'enum') {
    const alt = k.options.find((o) => o.id !== k.def) ?? k.options[0]
    return { value: alt.id, expected: alt.id }
  }
  // number/pin: a clearly-non-default in-range value
  let v
  if (k.def !== null && k.def !== undefined) v = k.def * 1.3 || 0.37
  else v = { turnsRatio: 3.7, magnetizingInductance: 1.23e-4, resonantInductance: 4.2e-6, resonantCapacitance: 4.7e-9, seriesInductance: 1.1e-6 }[k.pin] ?? 0.37
  if (k.min != null) v = Math.max(v, k.min)
  if (k.max != null) v = Math.min(v, k.max)
  if (k.int) v = Math.round(v) || 1
  return { value: v, expected: v }
}

// config keys the bench always emits regardless of knobs (not a leak).
const NON_KNOB_CONFIG = new Set(['tranStopTime', 'rectifier', 'rectifierType', 'mode'])

test.describe('T1 serialization', () => {
  for (const t of ALL_TOPOLOGIES) {
    const knobs = knobCatalog(t.id)
    if (!knobs.length) continue
    test(`${t.id}: each knob serializes to the right channel, in isolation`, async ({ page }) => {
      await boot(page)
      await selectTopology(page, t.id)
      for (const k of knobs) {
        // exactly one knob on at a time → clean isolation (cleared after each below)
        const { value, expected } = pick(k)
        await setKnob(page, k.key, value)
        const spec = await page.evaluate(() => window.__bench.previewSpec())

        if (k.pin) {
          const field = PIN_FIELD[k.pin]
          const got = k.pin === 'turnsRatio' ? spec.designRequirements[field]?.[0] : spec.designRequirements[field]
          expect(got, `${t.id}.${k.key} → designRequirements.${field}`).toBeCloseTo(expected, 9)
          // must NOT also appear in config
          expect(spec.config?.[k.key], `${k.key} must not leak into config`).toBeUndefined()
        } else {
          const got = spec.config?.[k.key]
          if (k.type === 'number') expect(got, `${t.id}.${k.key} → config.${k.key}`).toBeCloseTo(expected, 9)
          else expect(got, `${t.id}.${k.key} → config.${k.key}`).toBe(expected)
        }

        // Isolation: no OTHER knob's key/pin-field bled into the spec.
        for (const other of knobs) {
          if (other.key === k.key) continue
          if (!other.pin && !NON_KNOB_CONFIG.has(other.key)) {
            expect(spec.config?.[other.key], `${other.key} leaked while setting ${k.key}`).toBeUndefined()
          }
          if (other.pin && other.pin !== k.pin) {
            const f = PIN_FIELD[other.pin]
            expect(spec.designRequirements[f], `${f} leaked while setting ${k.key}`).toBeUndefined()
          }
        }
        await clearKnob(page, k.key) // back to auto before the next knob
      }
    })
  }
})

test('knob state resets when switching topology (no leak across a switch)', async ({ page }) => {
  await boot(page)
  // turn a Buck-only knob on, switch to LLC, assert the catalog swapped and no override survived.
  // maximumDutyCycle is a Buck knob LLC does not have; qualityFactor is LLC-only.
  await selectTopology(page, 'buck')
  await setKnob(page, 'maximumDutyCycle', 0.9)
  await selectTopology(page, 'llc')
  const st = await page.evaluate(() => ({
    keys: Object.keys(window.__bench.form.knobs),
    anyOn: Object.values(window.__bench.form.knobs).some((k) => k.on),
  }))
  expect(st.keys, 'LLC knob state, not Buck').toContain('qualityFactor')
  expect(st.keys, 'Buck-only knob gone').not.toContain('maximumDutyCycle')
  expect(st.anyOn, 'all LLC knobs start on auto').toBe(false)
})
