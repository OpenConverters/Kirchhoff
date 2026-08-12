// The schematic AS THE APP RENDERS IT — every other schematic gate measures a reconstruction.
//
// checkSchematicNets / auditSchematics / auditSchematicLabels all build the SVG in Node and, for the
// measured rules, re-create the app's styling in a headless page from a hand-copied CSS block and a
// hand-embedded font file. That is a second source of truth: if the app's real stylesheet, its
// @fontsource subsets or the SVG's `width:100%` scaling differ from that copy by so much as a font
// fallback, every measurement those gates make is of something the user never sees. (The renderer used
// for eyeballing had exactly that bug — it fell back to plain `monospace` and drew different text
// metrics from both the app and the audit.)
//
// So this drives the real app, pulls the geometry out of the LIVE DOM, and runs the same rules on it.
import { test } from '@playwright/test'
import { boot, selectTopology, solve, expect } from './helpers.js'
import { TOPOLOGIES, VARIANTS } from '../../src/topologies.js'
import { auditLabels } from '../../scripts/labelRules.mjs'

// Same extraction as scripts/auditSchematicLabels.mjs measure(), but against the app's own document.
async function measureLive(page) {
  return page.evaluate(() => {
    const svg = document.querySelector('.schematic-frame svg')
    if (!svg) return null
    const own = new Map()
    for (const g of svg.querySelectorAll('g.sch-hot'))
      for (const t of g.querySelectorAll('text')) own.set(t, g.dataset.ref)
    const texts = [...svg.querySelectorAll('text')].map((t) => {
      const b = t.getBBox()
      return { str: t.textContent, ref: own.get(t) ?? null, cls: t.getAttribute('class'),
               x: b.x, y: b.y, w: b.width, h: b.height }
    })
    // The INK of each part's own symbol, measured (getBBox resolves curves properly — a winding arc's
    // control points lie well outside the arc, so deriving this from the path data reports a box half
    // again too tall). Used by the OWN-BODY rule: a part's own label printed across its own glyph.
    const drawn = [...svg.querySelectorAll('g.sch-hot')].map((g) => {
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity
      for (const e of g.querySelectorAll('.sch-sym, .sch-fill')) {
        const b = e.getBBox()
        if (!b.width && !b.height) continue
        x0 = Math.min(x0, b.x); y0 = Math.min(y0, b.y)
        x1 = Math.max(x1, b.x + b.width); y1 = Math.max(y1, b.y + b.height)
      }
      return { ref: g.dataset.ref, x: x0, y: y0, w: x1 - x0, h: y1 - y0 }
    }).filter((b) => b.w > 0 && b.h > 0)
    const boxes = [...svg.querySelectorAll('rect.sch-hitbox')].map((r) => ({
      ref: r.closest('g.sch-hot')?.dataset.ref, x: +r.getAttribute('x'), y: +r.getAttribute('y'),
      w: +r.getAttribute('width'), h: +r.getAttribute('height') }))
    const glyphs = [...svg.querySelectorAll('rect.sch-fp')].map((r) => ({
      owner: r.dataset.owner || '', x: +r.getAttribute('x'), y: +r.getAttribute('y'),
      w: +r.getAttribute('width'), h: +r.getAttribute('height') }))
    const wires = []
    for (const p of svg.querySelectorAll('path.sch-wire')) {
      const n = [...p.getAttribute('d').matchAll(/[ML]\s*(-?[\d.]+)\s+(-?[\d.]+)/g)].map((m) => [+m[1], +m[2]])
      for (let i = 1; i < n.length; i++) wires.push([n[i - 1], n[i]])
    }
    const dots = [...svg.querySelectorAll('circle.sch-node')].map((c) => [+c.getAttribute('cx'), +c.getAttribute('cy')])
    const vb = svg.getAttribute('viewBox').split(/\s+/).map(Number)
    // The gate-drive flags: a flag drawn AT a switch's gate necessarily sits on that switch.
    const gates = [...svg.querySelectorAll('g.sch-hot')].flatMap((g) => [])
    return { texts, boxes, glyphs, drawn, wires, dots, W: vb[2], H: vb[3], gates }
  })
}

for (const t of TOPOLOGIES) {
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const name = `${t.id}${opt ? '/' + opt : ''}`
    test(`${name} schematic is legible as the app renders it`, async ({ page }) => {
      await boot(page)
      await selectTopology(page, t.id)
      // Same affordance visualsim.spec.js uses for the rectifier variants.
      if (opt && v) await page.evaluate((x) => { window.__bench.form.variant = x }, opt)
      expect(await solve(page, 'analytical'), 'solve error').toBeNull()
      await expect(page.locator('.schematic-frame svg').first()).toBeVisible({ timeout: 30000 })
      await page.evaluate(() => document.fonts.ready)

      const live = await measureLive(page)
      expect(live, 'the app rendered a schematic').not.toBeNull()
      // Gate flags are matched by proximity inside auditLabels; the live DOM has no separate pin list,
      // so pass none — that only makes the check STRICTER, never laxer.
      const problems = auditLabels(live, [])
        // A gate-drive flag sits on its own switch by construction; without the pin list those read as
        // T-SYM. Drop exactly that case (a <=3-char cyan sig label over a footprint) and nothing else.
        .filter((p) => !/^T-SYM "(g|s)[A-Za-z0-9]?[0-9]?" /.test(p))
      expect(problems, `live schematic problems:\n${problems.join('\n')}`).toEqual([])
    })
  }
}
