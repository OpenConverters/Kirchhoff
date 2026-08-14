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

// FONT COVERAGE. Every character the schematic prints must come from a font the app SHIPS. Ω and ≤ are in
// no IBM Plex Mono subset fontsource publishes, so both silently fell through to a system font: measured
// here, Ω rendered at 0.743 em against the 0.6 em mono advance — a proportional glyph among monospaced
// digits, on every resistor label, and a tofu box on any host without U+03A9. `document.fonts.check()`
// returns true for them either way, which is exactly why it went unnoticed for so long.
//
// The test is the advance width: in a monospaced face every glyph has the SAME advance, so a character
// that measures differently is not being drawn from that face. src/style.css now ships the missing glyphs
// (DejaVu Sans Mono subset, 0.602 em — within 0.3% of Plex), and this keeps it that way for whatever
// character a future value string introduces.
test('every character the schematic prints comes from a font the app ships', async ({ page }) => {
  await boot(page)
  const seen = new Set()
  for (const [id, opt] of CASES) {
    await selectTopology(page, id)
    if (opt) await page.evaluate((x) => { window.__bench.form.variant = x }, opt)
    expect(await solve(page, 'analytical'), `${id}: solve error`).toBeNull()
    await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })
    await page.evaluate(() => document.fonts.ready)
    for (const c of await page.evaluate(() => [...new Set([...document.querySelectorAll('.schematic-frame text')]
      .flatMap((t) => [...t.textContent]))])) seen.add(c)
  }
  expect(seen.size, 'characters drawn on the schematics').toBeGreaterThan(20)

  // Ask the document which of the app's OWN faces claims each character. Width-based font detection is
  // useless here: this host resolves serif, cursive and the bare default to the same font, so every
  // sentinel trick measures identical numbers whether or not a fallback is in play. document.fonts holds
  // exactly the @font-faces the app loads (system fonts never appear there), and each carries the
  // unicode-range it covers — so "is there a shipped face that claims this codepoint?" is answerable
  // exactly, on any machine. Ω was claimed by none of them, which is the bug this pins.
  const bad = await page.evaluate((chars) => {
    const faces = [...document.fonts].filter((f) => f.status === 'loaded')
    const claims = (range, cp) => (range || 'U+0-10FFFF').split(',').some((part) => {
      const m = part.trim().replace(/^U\+/i, '')
      if (m.includes('?')) { const lo = parseInt(m.replace(/\?/g, '0'), 16), hi = parseInt(m.replace(/\?/g, 'F'), 16); return cp >= lo && cp <= hi }
      const [a, b] = m.split('-')
      const lo = parseInt(a, 16), hi = b === undefined ? lo : parseInt(b, 16)
      return cp >= lo && cp <= hi
    })
    const out = []
    for (const c of chars) {
      if (c === ' ') continue
      const cp = c.codePointAt(0)
      const by = faces.filter((f) => claims(f.unicodeRange, cp)).map((f) => f.family)
      if (!by.length)
        out.push(`U+${cp.toString(16).toUpperCase().padStart(4, '0')} '${c}' is covered by NONE of the ` +
                 `${faces.length} faces the app loads — it is drawn by whatever the host happens to have, ` +
                 `or not at all`)
    }
    return out
  }, [...seen])
  expect(bad, `characters drawn from a font the app does not ship:\n${bad.join('\n')}`).toEqual([])
})

// DOES THE DRAWING FOLLOW THE DESIGN? Everything else in this suite inspects ONE render of a freshly
// solved design. The schematic is a Vue computed over the solved TAS and the BOM, injected as raw HTML —
// so the failure worth fearing is not a wrong line, it is a RIGHT line carrying yesterday's numbers:
// re-solve at another frequency and, if anything in that chain fails to invalidate, the pane keeps
// showing the previous design's values while the BOM beside it shows the new ones. Nothing has ever
// asked the two to agree, or asked the drawing to change when the design does.
test('the drawing shows the design that is loaded, not the one before it', async ({ page }) => {
  const drawn = () => page.evaluate(() => Object.fromEntries([...document.querySelectorAll('.schematic-frame g.sch-hot')]
    .map((g) => [g.dataset.ref, g.querySelector('text.sch-val')?.textContent ?? null])
    .filter(([, v]) => v)))
  const listed = async () => {
    await page.locator('.pane-select').last().selectOption('bom')
    const rows = await page.evaluate(() => Object.fromEntries([...document.querySelectorAll('table.data-table tbody tr')]
      .map((r) => [...r.querySelectorAll('td')].map((td) => td.textContent.trim()))
      .map(([ref, , , value]) => [ref, value])))
    await page.locator('.pane-select').last().selectOption('schematic')
    return rows
  }

  await boot(page)
  await selectTopology(page, 'buck')
  expect(await solve(page, 'analytical'), 'solve error').toBeNull()
  await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })

  const first = await drawn()
  expect(Object.keys(first).length, 'components drawn with a value').toBeGreaterThan(2)
  // The drawing and the BOM are two views of one design; a reader compares them constantly.
  const bom1 = await listed()
  for (const [ref, value] of Object.entries(first))
    expect(bom1[ref], `${ref}: the drawing says ${value}, the BOM says ${bom1[ref]}`).toBe(value)

  // Re-solve the same topology at 4x the switching frequency. Every reactive component must shrink;
  // if the pane still shows the old numbers, the drawing is stale.
  const fs = await page.evaluate(() => window.__bench.form.fs)
  await page.evaluate((f) => { window.__bench.form.fs = f * 4 }, fs)
  expect(await solve(page, 'analytical'), 'solve error at 4x fsw').toBeNull()
  await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })

  const second = await drawn()
  const moved = Object.keys(second).filter((ref) => second[ref] !== first[ref])
  expect(moved.length, `4x the switching frequency and every drawn value is unchanged: ${JSON.stringify(first)}`).toBeGreaterThan(0)
  const bom2 = await listed()
  for (const [ref, value] of Object.entries(second))
    expect(bom2[ref], `after re-solving, ${ref}: the drawing says ${value}, the BOM says ${bom2[ref]}`).toBe(value)
})

// PRINTING AFTER USING THE APP. There is no schematic export, so Ctrl+P is the only way to get the
// drawing onto paper, and the realistic moment to press it is AFTER clicking the part you care about.
// That state used to print: the part drawer covering 63% of the frame, a 55%-black mask across the whole
// sheet, and the selected component drawn in amber (#ffce85 on white is 1.4:1) inside a glow. The base
// print palette had only ever been checked on a freshly rendered, untouched page — and making the
// selection ring work (it was dead CSS) is what put it on paper in the first place.
test('printing while a part is open prints the drawing, not the app', async ({ page }) => {
  await boot(page)
  await selectTopology(page, 'buck')
  expect(await solve(page, 'analytical'), 'solve error').toBeNull()
  await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })
  await page.evaluate(() => window.__bench.openPart('L1'))
  await expect(page.locator('aside.drawer')).toBeVisible()

  await page.emulateMedia({ media: 'print' })
  const printed = await page.evaluate(() => {
    const sel = document.querySelector('.schematic-frame g.sch-hot.selected')
    const cs = (el) => (el ? getComputedStyle(el) : null)
    const frame = document.querySelector('.schematic-frame')?.getBoundingClientRect()
    const cover = (el) => {
      const b = el?.getBoundingClientRect()
      if (!b || !frame || cs(el)?.display === 'none') return 0
      return Math.max(0, Math.min(frame.right, b.right) - Math.max(frame.left, b.left)) *
             Math.max(0, Math.min(frame.bottom, b.bottom) - Math.max(frame.top, b.top))
    }
    return {
      ref: sel?.dataset.ref ?? null,
      symStroke: cs(sel?.querySelector('.sch-sym'))?.stroke,
      symFilter: cs(sel?.querySelector('.sch-sym'))?.filter,
      drawerCover: Math.round(cover(document.querySelector('aside.drawer'))),
      maskCover: Math.round(cover(document.querySelector('.drawer-mask'))),
    }
  })
  await page.emulateMedia({ media: 'screen' })

  expect(printed.ref, 'a part is selected while printing').toBe('L1')
  expect(printed.drawerCover, 'the part drawer covers the drawing on paper').toBe(0)
  expect(printed.maskCover, 'the modal mask washes over the drawing on paper').toBe(0)
  expect(printed.symFilter, 'the selected part keeps its screen glow on paper').toBe('none')
  // Whatever the selected part is drawn in must be the print ink, not the amber the screen uses.
  const [r, g, b] = printed.symStroke.match(/\d+/g).map(Number)
  const lum = (c) => { const s = c / 255; return s <= 0.04045 ? s / 12.92 : ((s + 0.055) / 1.055) ** 2.4 }
  const L = 0.2126 * lum(r) + 0.7152 * lum(g) + 0.0722 * lum(b)
  const contrast = 1.05 / (L + 0.05)
  expect(contrast, `the selected part prints as ${printed.symStroke} — ${contrast.toFixed(2)}:1 on paper`).toBeGreaterThan(3)
})

// BINDING A REAL PART IS THE OTHER WAY THE DESIGN CHANGES. Solving is not the only path: Kelvin binds a
// catalogue MPN into the TAS, which replaces the design under the drawing without any re-solve. The
// schematic is re-generated from that TAS and RE-VERIFIED against its CIAS on every render, so a bind
// that perturbs the netlist would turn the pane into the "Schematic ≠ netlist" banner — and the Kelvin
// e2e sources candidates but has never bound one, so nothing has ever watched the drawing survive it.
test('binding a real part leaves the schematic drawn, verified and agreeing with the BOM', async ({ page }) => {
  await boot(page)
  await selectTopology(page, 'flyback')
  expect(await solve(page, 'analytical'), 'solve error').toBeNull()
  await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })

  const shot = () => page.evaluate(() => Object.fromEntries([...document.querySelectorAll('.schematic-frame g.sch-hot')]
    .map((g) => [g.dataset.ref, g.querySelector('text.sch-val')?.textContent ?? ''])))
  const before = await shot()
  expect(Object.keys(before).length, 'components drawn before the bind').toBeGreaterThan(3)

  await page.evaluate(() => window.__bench.openPart('Q1'))
  await expect(page.getByTestId('kelvin-section')).toBeVisible()
  await page.getByTestId('find-parts').click()
  const table = page.getByTestId('kelvin-candidates')
  const empty = page.getByTestId('kelvin-empty')
  const err = page.getByTestId('kelvin-error')
  await expect(table.or(empty).or(err), 'the drawer settled').toBeVisible({ timeout: 120_000 })
  test.skip(!(await table.isVisible()), 'no catalogue candidates available in this environment')

  const mpn = (await page.getByTestId('kelvin-candidate').first().locator('.mpn').innerText()).trim()
  expect(mpn.length, 'the candidate about to be bound has an MPN').toBeGreaterThan(0)
  await page.getByTestId('use-part').first().click()
  await expect(page.getByTestId('bound-tag').first()).toBeVisible({ timeout: 60_000 })
  await page.keyboard.press('Escape')

  // The drawing must still BE a drawing — a bind that perturbed the netlist would show the banner.
  await expect(page.locator('.sch-error'), 'the schematic fell back to its netlist-drift banner').toHaveCount(0)
  await expect(page.locator('.schematic-frame svg').first()).toBeVisible()
  const after = await shot()
  expect(Object.keys(after).sort(), 'the bound design draws the same components').toEqual(Object.keys(before).sort())

  // ...and still agrees with the BOM, which the bind rewrote underneath it.
  await page.locator('.pane-select').last().selectOption('bom')
  const bom = await page.evaluate(() => Object.fromEntries([...document.querySelectorAll('table.data-table tbody tr')]
    .map((r) => [...r.querySelectorAll('td')].map((td) => td.textContent.trim())).map(([ref, , , value]) => [ref, value])))
  for (const [ref, value] of Object.entries(after))
    if (value) expect(bom[ref], `${ref}: the drawing says ${value}, the BOM says ${bom[ref]}`).toBe(value)
})

// WHEN THE DRAWING REFUSES TO BE DRAWN. The generator verifies every render against the CIAS and throws
// rather than draw something unverified, which is the right call — but the resulting banner is the ONLY
// signal the schematic is unavailable, and nothing had ever looked at it. It is reachable without any
// contrivance: forward's engine designs a second output (ABT #752) that no layout can place.
test('a schematic that cannot be drawn says so, out loud and in the palette', async ({ page }) => {
  await boot(page)
  await selectTopology(page, 'forward')
  expect(await solve(page, 'analytical'), 'the one-output design solves').toBeNull()
  await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })

  await page.getByRole('button', { name: '+ output' }).click()
  expect(await page.evaluate(() => window.__bench.form.outputs.length), 'the form took a second output').toBe(2)
  expect(await solve(page, 'analytical'), 'the two-output forward designs').toBeNull()

  const banner = page.locator('.sch-error')
  await expect(banner, 'the pane says why there is no drawing').toBeVisible({ timeout: 30000 })
  // No stale drawing left behind next to the banner.
  expect(await page.locator('.schematic-frame svg').count(), 'a drawing survived alongside the error').toBe(0)
  // Announced: replacing the pane's content without a live region tells a screen reader nothing.
  expect(await banner.getAttribute('role'), 'the banner is a live region').toBe('alert')
  const text = (await banner.textContent()) ?? ''
  expect(text, 'the banner names the topology').toMatch(/Forward/i)
  expect(text, 'the banner says what could not be drawn').toMatch(/placement|netlist|component/i)

  // ...and it is readable: the palette's fault colour against the pane, not a hard-coded stand-in.
  const ratio = await banner.evaluate((el) => {
    const px = (c) => c.match(/[\d.]+/g).slice(0, 3).map(Number)
    const lum = ([r, g, b]) => { const f = (v) => { const s = v / 255; return s <= 0.04045 ? s / 12.92 : ((s + 0.055) / 1.055) ** 2.4 }
      return 0.2126 * f(r) + 0.7152 * f(g) + 0.0722 * f(b) }
    let bg = el, bgc = 'rgba(0, 0, 0, 0)'
    while (bg && (bgc === 'rgba(0, 0, 0, 0)' || bgc === 'transparent')) { bgc = getComputedStyle(bg).backgroundColor; bg = bg.parentElement }
    const [l1, l2] = [lum(px(getComputedStyle(el).color)), lum(px(bgc))].sort((a, b) => b - a)
    return (l1 + 0.05) / (l2 + 0.05)
  })
  expect(ratio, `the banner sits at ${ratio.toFixed(2)}:1 against the pane`).toBeGreaterThan(4.5)
})
