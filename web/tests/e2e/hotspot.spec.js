// Clicking a component on the schematic opens THAT component — driven with a real pointer.
//
// The schematic is the app's main navigation surface: every part is a hotspot, and a click is supposed
// to open that part's drawer (candidate sourcing, the magnetics handoff, the per-part waveform). Nothing
// had ever exercised that path. The Kelvin e2e opens drawers through `window.__bench.openPart(ref)` —
// an affordance whose own comment says "the schematic click is hard to script" — so the handler it
// bypasses, `ev.target.closest('[data-ref]')`, was never once driven by a pointer in a test.
//
// scripts/checkSchematicHotspots.mjs proves the routing INSIDE the SVG (whose click target covers whose
// ink) across all 39 combos. This proves the other half, which that gate cannot see: that in the running
// app a real click at those coordinates reaches the handler and opens the right drawer. Anything laid
// over the pane — a mask, a tooltip, a transformed wrapper, `pointer-events` on the wrong element —
// breaks this while every offline gate stays green.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, expect } from './helpers.js'

// Four topologies, EVERY part in each: a two-terminal-passive layout with a clamp (flyback), a
// synchronous half-bridge whose FETs stack (buck), an isolated resonant one with the densest secondary
// (llc), and the one with a control block and divider network (pfc). Clicking every part of every combo
// would add 39 slow tests for the same one line of app code.
const CASES = [['flyback', null], ['buck', 'synchronous'], ['llc', 'fullBridge'], ['pfc', null]]

// A click point ON each part's own ink, chosen the way the offline gate samples: walk the stroke, keep
// the first point where the app itself would resolve this part. Deriving it from the hitbox centre
// instead would test a point the user has no reason to click (the middle of a capacitor is a gap).
async function clickPoints(page) {
  return page.evaluate(() => {
    const svg = document.querySelector('.schematic-frame svg')
    const out = []
    for (const g of svg.querySelectorAll('g.sch-hot:not(.sch-ann)')) {
      const ref = g.dataset.ref
      let best = null
      for (const e of g.querySelectorAll('.sch-sym, .sch-fill')) {
        if (best || typeof e.getTotalLength !== 'function') continue
        let len = 0
        try { len = e.getTotalLength() } catch { continue }
        if (!len) continue
        const m = svg.getScreenCTM()
        for (let i = 1; i < 12 && !best; i++) {
          const p = e.getPointAtLength((len * i) / 12)
          const x = m.a * p.x + m.c * p.y + m.e, y = m.b * p.x + m.d * p.y + m.f
          if (document.elementFromPoint(x, y)?.closest('[data-ref]')?.dataset.ref === ref)
            best = { ref, x, y }
        }
      }
      if (best) out.push(best)
    }
    return out
  })
}

for (const [id, opt] of CASES) {
  const name = `${id}${opt ? '/' + opt : ''}`
  test(`${name}: clicking a component opens that component's drawer`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    if (opt) await page.evaluate((x) => { window.__bench.form.variant = x }, opt)
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()
    await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })
    await page.evaluate(() => document.fonts.ready)

    const points = await clickPoints(page)
    // An empty list would make every assertion below vacuous — the classic green-because-nothing-ran.
    expect(points.length, 'components with a clickable point on their own ink').toBeGreaterThan(4)

    const wrong = []
    for (const p of points) {
      await page.mouse.click(p.x, p.y)
      const drawer = page.locator('aside.drawer')
      const opened = await drawer.isVisible().catch(() => false)
      const got = opened ? (await drawer.locator('h3').first().textContent())?.trim() : null
      if (got !== p.ref) wrong.push(`${p.ref}: clicking its symbol at ${p.x.toFixed(0)},${p.y.toFixed(0)} opened ${got ?? 'no drawer'}`)
      // ...and the drawing says WHICH part is open. The stylesheet has always defined `.sch-hot.selected`
      // and nothing applied it, so the reader had to find the part by name in a 20-symbol drawing.
      const marked = await page.evaluate((ref) => {
        const on = [...document.querySelectorAll('.schematic-frame g.sch-hot.selected')].map((g) => g.dataset.ref)
        return on.length === 1 && on[0] === ref
      }, p.ref)
      if (opened && !marked) wrong.push(`${p.ref}: its drawer opened but the schematic does not mark it as selected`)
      if (opened) await page.keyboard.press('Escape')
    }
    expect(wrong, `schematic clicks that opened the wrong part:\n${wrong.join('\n')}`).toEqual([])

    // KEYBOARD (ABT #693). The same components are role="button" tab stops; a button that answers the
    // mouse but not Enter/Space is not operable (WCAG 2.1.1). Tab from the pane and activate what lands.
    const first = page.locator('.schematic-frame g.sch-hot:not(.sch-ann)').first()
    const firstRef = await first.getAttribute('data-ref')
    await first.focus()
    const focused = await page.evaluate(() => document.activeElement?.dataset?.ref ?? null)
    expect(focused, 'a component takes keyboard focus').toBe(firstRef)
    await page.keyboard.press('Enter')
    await expect(page.locator('aside.drawer')).toBeVisible()
    expect((await page.locator('aside.drawer h3').first().textContent())?.trim(),
      'Enter on a focused component opens its drawer').toBe(firstRef)
    await page.keyboard.press('Escape')
    await expect(page.locator('aside.drawer')).toBeHidden()
    // ...and Space too, whose default (scrolling the page) must be suppressed.
    const scrollBefore = await page.evaluate(() => window.scrollY)
    await first.focus()
    await page.keyboard.press(' ')
    await expect(page.locator('aside.drawer')).toBeVisible()
    expect(await page.evaluate(() => window.scrollY), 'Space activated instead of scrolling').toBe(scrollBefore)
    await page.keyboard.press('Escape')
  })
}

// TARGET SIZE (WCAG 2.5.8 AA). Measured in the running app, a component's click target is 6.5–28 CSS px
// — a resistor is 7 px wide — because the drawing is scaled to fit the pane. The spacing exception does
// not rescue it either: llc's closest target centres are 20 px apart, inside the 24 px circle the
// exception requires to stay clear. Enlarging the hitboxes is not the fix; they would start covering
// each other's symbols, which is exactly the misroute defect checkSchematicHotspots.mjs exists to catch.
//
// What makes this conform is the EQUIVALENT CONTROL exception: the same drawer opens from the BOM view's
// rows, on the same page, at 838 × 31.6 px. That argument is invisible in the code and one refactor from
// being false, so it is pinned here: every component that is clickable on the drawing must also be
// reachable from a row that meets the criterion and opens the same part.
for (const [id, opt] of [['flyback', null], ['llc', 'fullBridge']]) {
  test(`${id}: every component is also reachable through a target that meets WCAG 2.5.8`, async ({ page }) => {
    await boot(page)
    await selectTopology(page, id)
    if (opt) await page.evaluate((x) => { window.__bench.form.variant = x }, opt)
    expect(await solve(page, 'analytical'), 'solve error').toBeNull()
    await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })

    const refs = await page.evaluate(() => [...document.querySelectorAll('.schematic-frame g.sch-hot:not(.sch-ann)')]
      .map((g) => g.dataset.ref))
    expect(refs.length, 'clickable components on the drawing').toBeGreaterThan(4)
    // For the record, and so a future change that shrinks the drawing further is visible in the log.
    const smallest = await page.evaluate(() => Math.min(...[...document.querySelectorAll('.schematic-frame rect.sch-hitbox')]
      .flatMap((r) => { const b = r.getBoundingClientRect(); return [b.width, b.height] })))
    console.log(`${id}: smallest schematic target ${smallest.toFixed(1)} px (conformance rests on the BOM rows below)`)

    await page.locator('.pane-select').last().selectOption('bom')
    const table = page.locator('table.data-table')
    await expect(table).toBeVisible()
    const bad = []
    for (const ref of refs) {
      const row = table.locator('tbody tr').filter({ has: page.locator(`td:text-is("${ref}")`) }).first()
      if (!(await row.count())) { bad.push(`${ref}: clickable on the drawing but absent from the BOM`); continue }
      const box = await row.boundingBox()
      if (!box || box.width < 24 || box.height < 24)
        bad.push(`${ref}: its BOM row is ${box ? `${box.width.toFixed(0)}x${box.height.toFixed(0)}` : 'not laid out'} — under the 24x24 minimum`)
      await row.click()
      const got = (await page.locator('aside.drawer h3').first().textContent())?.trim()
      if (got !== ref) bad.push(`${ref}: its BOM row opened ${got ?? 'no drawer'} — not the same function as the symbol`)
      await page.keyboard.press('Escape')
    }
    expect(bad, `components with no conforming way to reach them:\n${bad.join('\n')}`).toEqual([])
  })
}
