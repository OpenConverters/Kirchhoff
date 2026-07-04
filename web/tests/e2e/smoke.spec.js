// T0 — universal smoke: every topology designs through the real GUI flow and the main magnetic
// produces a physically-sane current waveform. Also a baked-in consistency check that the engine's
// own processed RMS matches an independent recomputation from the waveform samples (measure.js).
import { test } from '@playwright/test'
import { boot, selectTopology, solve, windingWaveform, windingProcessed, readSpec, ALL_TOPOLOGIES, expect } from './helpers.js'
import * as m from '../lib/measure.js'

// Topologies whose analytical path emits NO per-winding switching waveform (only design + envelope
// stats). A new topology dropping its winding waveform is a regression and must fail — so this list
// is an explicit allow-list with a reason, not a silent skip.
const NO_ANALYTICAL_WINDING_WAVEFORM = {
  vienna: '3-phase line-cycle envelope; boost inductors carry no synthesized per-winding switching waveform',
}

test.describe('T0 smoke — analytical', () => {
  for (const t of ALL_TOPOLOGIES) {
    test(`${t.id} designs and yields a sane main-magnetic waveform`, async ({ page }) => {
      await boot(page)
      await selectTopology(page, t.id)
      const err = await solve(page, 'analytical')
      expect(err, `${t.id} solve error: ${err}`).toBeNull()

      // Universal: a valid design realized (spec + TAS + at least one magnetic).
      const spec = await readSpec(page)
      expect(spec?.designRequirements, 'spec has designRequirements').toBeTruthy()
      const built = await page.evaluate(() => ({
        tas: !!window.__bench.result?.tas,
        mags: (window.__bench.waveMagnetics || []).length,
      }))
      expect(built.tas, `${t.id} realized a TAS`).toBe(true)
      expect(built.mags, `${t.id} has ≥1 magnetic`).toBeGreaterThan(0)

      const wf = await windingWaveform(page, { winding: 0, side: 'current' })
      if (!wf) {
        expect(Object.keys(NO_ANALYTICAL_WINDING_WAVEFORM),
          `${t.id} unexpectedly has no analytical winding waveform`).toContain(t.id)
        return
      }
      expect(wf.data.length, 'waveform has samples').toBeGreaterThan(2)
      expect(wf.data.length).toBe(wf.time.length)

      const proc = await windingProcessed(page, { winding: 0, side: 'current' })
      expect(proc, 'engine emitted processed stats').toBeTruthy()
      expect(Number.isFinite(proc.rms) && proc.rms > 0, `RMS finite > 0 (got ${proc.rms})`).toBe(true)

      // Consistency: the emitted samples and the engine's processed RMS must agree (±3%) — for BOTH
      // closed-form and custom labels. The WaveformProcessor time-weighted-RMS fix made every custom
      // bridge/resonant current (incl. AHB) self-consistent, so this is a hard gate for all topologies.
      const measured = m.rms(wf)
      const rel = Math.abs(measured - proc.rms) / proc.rms
      expect(rel, `${t.id} measured RMS ${measured} vs processed ${proc.rms} (rel ${rel})`).toBeLessThan(0.03)
    })
  }
})

// A small real-transient smoke: the ngspice WASM path also designs for a few representative
// topologies. Tolerant of a libngspice-less build (component sim may report unavailable) — the
// design + main-magnetic operating point are what must exist.
test.describe('T0 smoke — ngspice', () => {
  for (const id of ['buck', 'flyback', 'llc']) {
    test(`${id} designs under the ngspice engine`, async ({ page }) => {
      await boot(page)
      await selectTopology(page, id)
      const err = await solve(page, 'ngspice')
      expect(err, `${id} ngspice error: ${err}`).toBeNull()
      const proc = await windingProcessed(page, { winding: 0, side: 'current' })
      expect(proc && Number.isFinite(proc.rms) && proc.rms > 0, `${id} main-magnetic RMS`).toBe(true)
    })
  }
})
