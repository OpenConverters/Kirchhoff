// Legibility audit of the rendered SVG schematics (run: `node scripts/auditSchematicLabels.mjs`).
//
// auditSchematics.mjs proves the WIRING reads right; this proves the drawing is LEGIBLE. A label lying
// across a wire, two labels printed on top of each other, or two symbols overlapping are invisible to
// every net rule, yet they are the first thing a reader trips over.
//
// Boxes are MEASURED, not estimated: the SVG is rendered in headless Chromium with the app's real font
// (IBM Plex Mono, embedded from node_modules) and every <text> is asked for its own getBBox(). An
// earlier version estimated advance widths at 0.6 em and produced ~120 candidates that were mostly 1-2 px
// arithmetic artifacts — with one that turned out to be a genuinely unreadable label. Estimating was
// worse than useless: it buried the real defect in noise.
//
//   T-WIRE    a text box lying across a wire segment
//   T-TEXT    two text boxes overlapping
//   T-SYM     a text box lying across a component footprint that is not its own
//   OWN-BODY  a part's own ref/value printed across the part's own symbol (every other rule exempts
//             a part's own labels, so this is the one blind spot they share)
//   SYM-SYM   two component footprints overlapping
//   FALSE-DOT a junction dot at a pure CROSSING (neither wire has an endpoint there)
//   DUP-REF   the same refdes drawn twice
//   OOB       a label outside the viewBox
//
// TOL px of overlap are tolerated: glyph boxes include side bearing, so a label that merely touches a
// rail reads fine. Anything deeper is a real collision.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { hasSchematic } from '../src/schematics.js'
// Renders through the SAME entry point the app uses: for a topology with a CIAS layout the product
// draws THAT, not the hand-authored art, so auditing collectPins() directly measured a drawing the
// user never sees (see renderForAudit in ciasSchematic.js).
import { renderForAudit } from '../src/ciasSchematic.js'
import { chromium } from '@playwright/test'

// The app's OWN stylesheet, sliced out of src/style.css — see harnessCss.mjs. It used to be a hand
// copy here; a copy of the numbers that decide every glyph box is a copy that will drift.
import { HARNESS_CSS as CSS } from './harnessCss.mjs'

import { fileURLToPath } from 'node:url'
import path from 'node:path'
// Dual-purpose (see auditSchematics.mjs): as a CLI it sweeps every topology; imported, it exposes
// measure()/auditLabels() so checkSweep.mjs can measure its own renders on its own page.
const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])
const M = await init()
const browser = isMain ? await chromium.launch() : null
const page = isMain ? await browser.newPage() : null
const only = process.argv[2]

// Ask the browser for the true geometry: text boxes (getBBox), hitboxes, wire segments, dots.
async function measure(page, svg) {
  await page.setContent(`<style>${CSS}</style>${svg}`)
  await page.evaluate(() => document.fonts.ready)
  return page.evaluate(() => {
    const own = new Map()
    for (const g of document.querySelectorAll('g.sch-hot'))
      for (const t of g.querySelectorAll('text')) own.set(t, g.dataset.ref)
    const texts = [...document.querySelectorAll('text')].map((t) => {
      const b = t.getBBox()
      return { str: t.textContent, ref: own.get(t) ?? null, cls: t.getAttribute('class'),
               x: b.x, y: b.y, w: b.width, h: b.height }
    })
    // The INK of each part's own symbol, measured (getBBox resolves curves properly — a winding arc's
    // control points lie well outside the arc, so deriving this from the path data reports a box half
    // again too tall). Used by the OWN-BODY rule: a part's own label printed across its own glyph.
    const drawn = [...document.querySelectorAll('g.sch-hot')].map((g) => {
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity
      for (const e of g.querySelectorAll('.sch-sym, .sch-fill')) {
        const b = e.getBBox()
        if (!b.width && !b.height) continue
        x0 = Math.min(x0, b.x); y0 = Math.min(y0, b.y)
        x1 = Math.max(x1, b.x + b.width); y1 = Math.max(y1, b.y + b.height)
      }
      return { ref: g.dataset.ref, x: x0, y: y0, w: x1 - x0, h: y1 - y0 }
    }).filter((b) => b.w > 0 && b.h > 0)
    const boxes = [...document.querySelectorAll('rect.sch-hitbox')].map((r) => ({
      ref: r.closest('g.sch-hot')?.dataset.ref, x: +r.getAttribute('x'), y: +r.getAttribute('y'),
      w: +r.getAttribute('width'), h: +r.getAttribute('height') }))
    // Non-BOM glyphs (source, ground, port, load) carry a sch-fp footprint instead of a hitbox.
    const glyphs = [...document.querySelectorAll('rect.sch-fp')].map((r) => ({
      owner: r.dataset.owner || '', x: +r.getAttribute('x'), y: +r.getAttribute('y'),
      w: +r.getAttribute('width'), h: +r.getAttribute('height') }))
    const wires = []
    for (const p of document.querySelectorAll('path.sch-wire')) {
      const n = [...p.getAttribute('d').matchAll(/[ML]\s*(-?[\d.]+)\s+(-?[\d.]+)/g)].map((m) => [+m[1], +m[2]])
      for (let i = 1; i < n.length; i++) wires.push([n[i - 1], n[i]])
    }
    const dots = [...document.querySelectorAll('circle.sch-node')].map((c) => [+c.getAttribute('cx'), +c.getAttribute('cy')])
    const vb = document.querySelector('svg').getAttribute('viewBox').split(/\s+/).map(Number)
    return { texts, boxes, glyphs, drawn, wires, dots, W: vb[2], H: vb[3] }
  })
}

// The rules themselves live in labelRules.mjs — engine-free, so the live-app gate
// (tests/e2e/schematic.spec.js) can apply the SAME code to the app's own DOM.
export { auditLabels } from './labelRules.mjs'
import { auditLabels } from './labelRules.mjs'

export { measure }

let flagged = 0, total = 0
if (isMain) for (const t of TOPOLOGIES) {
  if (!hasSchematic(t.id)) continue
  if (only && t.id !== only) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${t.id}${opt ? '/' + opt : ''}: design failed: ${out.slice(0, 200)}`)
    const { svg, pins } = renderForAudit(t.id, JSON.parse(out).tas, opt ?? 'standard')
    // A gate-drive flag is drawn AT its switch's gate pin, so it necessarily lies on that switch's
    // footprint — that is its own label, not a collision. Over any OTHER part it still counts.
    const gates = pins.filter((q) => q.pin === 'gate')
    // ONE implementation of the rules: the CLI re-inlined all seven, so a fix to the shared
    // auditLabels() (used by checkSweep) silently missed this sweep, and vice versa.
    const p = auditLabels(await measure(page, svg), gates)

    const uniq = [...new Set(p)]
    total += uniq.length
    if (uniq.length) { flagged++; console.log(`\n== ${t.id}${opt ? '/' + opt : ''}  (${uniq.length})`); for (const x of uniq) console.log('   ' + x) }
  }
}
if (isMain) {
  await browser.close()
  console.log(flagged ? `\n${total} collision(s) in ${flagged} combo(s)` : '\nclean')
  process.exit(flagged ? 1 : 0)
}
