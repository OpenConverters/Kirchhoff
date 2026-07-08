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

  test('binding a MOSFET candidate Range-fetches its record and re-sims the design', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    const ref = await openKind(page, 'MOSFET')
    await page.getByTestId('find-parts').click()
    await expect(page.getByTestId('kelvin-candidates')).toBeVisible({ timeout: 15000 })

    const topMpn = (await page.getByTestId('kelvin-candidate').first().locator('.mpn').innerText()).trim()

    // "use" the top candidate: the PartDrawer Range-fetches that one record from the hosted NDJSON
    // (bytes=srcOffset-srcLength → 206) and binds it via Kelvin's bind_part; App swaps the TAS + re-sims.
    await page.getByTestId('use-part').first().click()
    await expect(page.getByTestId('bound-tag')).toBeVisible({ timeout: 15000 })
    await expect(page.getByTestId('kelvin-bind-error')).toHaveCount(0)

    // The real part is now in the design TAS (bind_part wrote data.semiconductor verbatim) — the
    // deterministic default's MPN, bound onto the MOSFET we opened.
    const boundRef = await page.evaluate((r) => {
      const comps = window.__bench.result?.tas?.topology?.stages?.flatMap((s) => s.circuit?.components ?? []) ?? []
      const c = comps.find((x) => x.name === r)
      return c?.data?.semiconductor?.mosfet?.manufacturerInfo?.reference ?? null
    }, ref)
    expect(boundRef, 'bound MPN is written into the TAS').toBe(topMpn)

    // Binding kicked a fresh component sim (App.onBound → fetchComponentWaves) so the verdict table
    // re-validates against the real part — the re-sim produced waveforms for the bound ref.
    await page.waitForFunction(
      (r) => window.__bench.componentWaves?.components?.some((c) => c.ref === r),
      ref, { timeout: 30000 })
  })

  test('magnetic drawer offers all three sourcing paths (suggest / design in OM / catalog)', async ({ page }) => {
    // Originally magnetics had ONLY the OM handoff ("not a catalog table"); since the Kelvin
    // magnetic family landed, the drawer deliberately offers the catalog alongside it, and the
    // adviser-suggestions list (ABT #177) is the third path. Assert all three surfaces.
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    const ref = await page.evaluate(() =>
      window.__bench.bomRows.find((r) => r.kind === 'Transformer' || r.kind === 'Inductor')?.ref)
    expect(ref, 'flyback has a magnetic').toBeTruthy()
    await page.evaluate((r) => window.__bench.openPart(r), ref)

    await expect(page.getByTestId('magnetic-suggest-section')).toBeVisible()   // 1: adviser suggestions
    await expect(page.getByTestId('suggest-magnetics')).toBeVisible()
    await expect(page.getByTestId('magnetic-section')).toBeVisible()           // 2: interactive OM design
    await expect(page.getByTestId('design-magnetic')).toBeVisible()
    await expect(page.getByTestId('kelvin-section')).toBeVisible()             // 3: TAS-DB catalog
    // The per-magnetic simulation-model selector rides along in the same drawer.
    await expect(page.getByTestId('magnetic-model-select')).toBeVisible()
  })

  test('bind refuses a shard/catalog version mismatch (safety guard)', async ({ page }) => {
    // Serve a manifest whose mosfet.sourceSize disagrees with the hosted NDJSON. Selection (shard-only)
    // must still work, but binding — which Range-reads the NDJSON by the shard's byte offsets — must
    // refuse rather than risk reading the wrong record from a mismatched catalog.
    await page.route('**/kelvin/manifest.json', async (route) => {
      const res = await route.fetch()
      const m = await res.json()
      m.families.mosfet.sourceSize += 1 // deliberately wrong vs the real NDJSON size
      await route.fulfill({ json: m })
    })
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    await openKind(page, 'MOSFET')
    await page.getByTestId('find-parts').click()
    await expect(page.getByTestId('kelvin-candidates')).toBeVisible({ timeout: 15000 }) // select unaffected

    await page.getByTestId('use-part').first().click()
    await expect(page.getByTestId('kelvin-bind-error')).toContainText('version mismatch', { timeout: 15000 })
    await expect(page.getByTestId('bound-tag')).toHaveCount(0)
  })

  test('capacitor drawer sources real parts (large shard lazy-loads)', async ({ page }) => {
    await boot(page)
    await selectTopology(page, 'flyback')
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()

    await openKind(page, 'Capacitor')
    await page.getByTestId('find-parts').click()
    await expect(page.getByTestId('kelvin-candidates')).toBeVisible({ timeout: 20000 })
    expect(await page.getByTestId('kelvin-candidate').count()).toBeGreaterThan(0)

    // Manufacturer diversity cap (maxManufacturerFraction=0.2) must actually take effect in the WASM:
    // the flyback bulk-cap set is vendor-skewed, so uncapped it comes back all one manufacturer;
    // capped it spans several. >1 distinct vendor proves the cap reached apply_mfr_policy.
    const vendors = await page.$$eval('[data-testid=kelvin-candidate] .mfr',
      els => [...new Set(els.map(e => e.textContent.trim()))])
    expect(vendors.length, `candidate list spans multiple manufacturers (got ${vendors.join(', ')})`).toBeGreaterThan(1)
  })
})
