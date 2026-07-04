// T2 — per-knob physics: set a knob, SIMULATE, and measure the waveform to confirm it moved the way
// the knob promises. Assertions are DIFFERENTIAL (baseline vs override, or two override values) so the
// test never re-implements the engine's design math — it only checks the direction/ratio of change.
import { test } from '@playwright/test'
import { boot, selectTopology, setKnob, solve, windingWaveform, windingProcessed, expect } from './helpers.js'
import * as m from '../lib/measure.js'

// Solve the current design on the analytical engine and fail loudly on an engine error.
async function run(page) {
  const err = await solve(page, 'analytical')
  expect(err, `solve error: ${err}`).toBeNull()
}

test.describe('T2 physics', () => {
  test('buck: rippleRatio scales the inductor-current ripple', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'buck')

    await setKnob(page, 'rippleRatio', 0.2)
    await run(page)
    const low = m.rippleRatio(await windingWaveform(page, { winding: 0, side: 'current' }))

    await setKnob(page, 'rippleRatio', 0.4)
    await run(page)
    const high = m.rippleRatio(await windingWaveform(page, { winding: 0, side: 'current' }))

    // The inductor-current ripple ratio IS the knob (±20% for the real-inductor rounding),
    // and doubling the target roughly doubles the measured ripple.
    expect(low, `ripple@0.2 = ${low}`).toBeGreaterThan(0.1)
    expect(low).toBeLessThan(0.3)
    expect(high / low, `ripple ratio 0.4/0.2 = ${high / low}`).toBeGreaterThan(1.6)
    expect(high / low).toBeLessThan(2.4)
  })

  test('boost: rippleRatio scales the inductor-current ripple', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'boost')
    await setKnob(page, 'rippleRatio', 0.2)
    await run(page)
    const low = m.rippleRatio(await windingWaveform(page, { winding: 0, side: 'current' }))
    await setKnob(page, 'rippleRatio', 0.4)
    await run(page)
    const high = m.rippleRatio(await windingWaveform(page, { winding: 0, side: 'current' }))
    expect(high / low, `boost ripple 0.4/0.2 = ${high / low}`).toBeGreaterThan(1.6)
    expect(high / low).toBeLessThan(2.4)
  })

  test('buck: pinned inductance sets the ripple inversely (2× L → ½ ripple)', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'buck')
    // establish the auto inductor's ripple, then pin L and 2L
    await setKnob(page, 'pinL', 100e-6)
    await run(page)
    const rL = m.pkpk(await windingWaveform(page, { winding: 0, side: 'current' }))
    await setKnob(page, 'pinL', 200e-6)
    await run(page)
    const r2L = m.pkpk(await windingWaveform(page, { winding: 0, side: 'current' }))
    expect(rL / r2L, `pkpk(L)/pkpk(2L) = ${rL / r2L}`).toBeGreaterThan(1.7)
    expect(rL / r2L).toBeLessThan(2.3)
  })

  test('forward: maxDutyCycle caps the operating duty', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'forward')
    await setKnob(page, 'maxDutyCycle', 0.45)
    await run(page)
    const d45 = (await windingProcessed(page, { winding: 0, side: 'current' }))?.dutyCycle
    await setKnob(page, 'maxDutyCycle', 0.30)
    await run(page)
    const d30 = (await windingProcessed(page, { winding: 0, side: 'current' }))?.dutyCycle
    // A tighter duty ceiling forces a lower operating duty (the transformer turns ratio rescales).
    expect(d45, `duty@0.45 = ${d45}`).toBeGreaterThan(d30)
  })

  test('dab: phase-shift knob sets the bridge-to-bridge phase', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'dab')
    async function measurePhase() {
      const pv = await windingWaveform(page, { winding: 0, side: 'voltage' })
      const sv = await windingWaveform(page, { winding: 1, side: 'voltage' })
      if (!pv || !sv) return null
      return Math.abs(m.phaseBetween(pv, sv))
    }
    await setKnob(page, 'dabPhaseShiftDeg', 20)
    await run(page)
    const p20 = await measurePhase()
    await setKnob(page, 'dabPhaseShiftDeg', 40)
    await run(page)
    const p40 = await measurePhase()
    test.skip(p20 === null || p40 === null, 'DAB winding voltages not exposed as sampled waveforms')
    expect(p40, `phase@40°=${p40} vs phase@20°=${p20}`).toBeGreaterThan(p20)
  })
})
