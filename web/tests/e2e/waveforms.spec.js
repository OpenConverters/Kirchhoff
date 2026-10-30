// Waveform picker — asserted on what the PANE RENDERS, not on window.__bench. The bench hook held the
// component run while the pane said "No simulated waveform for Cr" for every part: the app never
// provided componentWaves to the OutputPanes, and every hook-reading gate stayed green.
//
// Rule under test: after an ngspice solve every entry in the waveform dropdown (each magnetic, each
// switch / diode / capacitor / resistor) draws its current AND voltage, and every one is labelled
// measured — no magnetic voltage is spliced in from the analytical capture.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, expect } from './helpers.js'

const TOPOLOGIES = ['llc', 'cllc', 'psfb']

test.describe('waveform picker — every entry draws simulated V and I', () => {
  for (const id of TOPOLOGIES) {
    test(`${id}: ngspice solve fills every dropdown entry`, async ({ page }) => {
      await boot(page)
      await selectTopology(page, id)
      const err = await solve(page, 'ngspice')
      expect(err, `${id} solve error: ${err}`).toBeNull()
      // every magnetic + the component run must land (the solve fires them in the background)
      await page.waitForFunction(() => window.__bench.componentWaves
        && window.__bench.waveMagnetics.every((m) => window.__bench.ngspiceOps[m.name]), null, { timeout: 180_000 })

      await page.locator('select.pane-select').first().selectOption('waveforms')
      const pane = page.locator('.view-fill').first()
      const target = pane.locator('.wave-toolbar select').first()
      const options = await target.locator('option').evaluateAll((os) => os.map((o) => o.value))
      expect(options.filter((v) => v.startsWith('dev:')).length, 'dropdown lists components').toBeGreaterThan(0)

      for (const v of options) {
        await target.selectOption(v)
        const readout = pane.locator('.wave-readout')
        await expect(readout, `${id} ${v}: readout`).toBeVisible()
        await expect(pane.locator('.wave-empty, .err-banner'), `${id} ${v}: no empty/error state`).toHaveCount(0)
        const text = (await readout.innerText()).replace(/\s+/g, ' ')
        expect(text, `${id} ${v}: current is simulated`).toMatch(/current · measured/)
        expect(text, `${id} ${v}: voltage is simulated`).toMatch(/V\w* · measured|voltage · measured/)
        expect(text, `${id} ${v}: nothing analytical`).not.toMatch(/analytical/)
      }
    })
  }
})
