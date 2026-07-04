// Kelvin sourcing — the PartDrawer surfaces real TAS-DB candidate parts for a designed converter.
// Drives the full GUI flow (design flyback -> open a component -> Find real parts -> candidate
// table) through the DOM + the __bench openPart hook, against the in-browser WASM + hosted shards.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, expect } from './helpers.js'

async function openKind(page, kind) {
  const ref = await page.evaluate((k) => window.__bench.bomRows.find((r) => r.kind === k)?.ref, kind)
  expect(ref, `a ${kind} exists in the BOM`).toBeTruthy()
  await page.evaluate((r) => window.__bench.openPart(r), ref)
  await expect(page.getByTestId('kelvin-section')).toBeVisible()
  return ref
}

test.describe('Kelvin candidate sourcing', () => {
  test('flyback MOSFET drawer sources real parts', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    await openKind(page, 'MOSFET')
    await page.getByTestId('find-parts').click()

    // Candidate table renders with ≥1 real part; top row is the deterministic default.
    await expect(page.getByTestId('kelvin-candidates')).toBeVisible({ timeout: 15000 })
    const n = await page.getByTestId('kelvin-candidate').count()
    expect(n, 'at least one candidate').toBeGreaterThan(0)
    const topMpn = await page.getByTestId('kelvin-candidate').first().locator('.mpn').innerText()
    expect(topMpn.trim().length, 'top candidate has an MPN').toBeGreaterThan(0)
  })

  test('capacitor drawer sources real parts (large shard lazy-loads)', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    await openKind(page, 'Capacitor')
    await page.getByTestId('find-parts').click()
    await expect(page.getByTestId('kelvin-candidates')).toBeVisible({ timeout: 20000 })
    expect(await page.getByTestId('kelvin-candidate').count()).toBeGreaterThan(0)
  })
})
