// Does clicking a part on the schematic select THAT part? (run: `node scripts/checkSchematicHotspots.mjs`)
//
// The schematic is not a picture — it is the app's main navigation surface. Every component is wrapped
// in `<g class="sch-hot" data-ref="…">` holding a TRANSPARENT `rect.sch-hitbox`, and App.vue resolves a
// click with `ev.target.closest('[data-ref]')`. A transparent fill is hit-testable, so that rectangle
// swallows every pointer event inside it — including events over a NEIGHBOUR's symbol, if the boxes
// overlap and the neighbour was painted first. Nothing in the suite has ever asked where a click lands:
// the net rules read the netlist, the label rules read text boxes, the contrast gate reads colours.
// A drawing can be electrically perfect, legible, printable — and still open the wrong part's drawer.
//
// So ask the browser the only question that matters here: put the pointer on a part's OWN INK and see
// which data-ref the app would resolve.
//   MISROUTE  a point on this part's own symbol resolves to a DIFFERENT part (its hitbox is on top)
//   DEAD      a point on this part's own symbol resolves to no part at all, and what covers it is NOT a
//             conductor. A wire, control stub or junction dot lying over a lead where they connect is
//             the drawing working as intended — clicking the wire selects nothing because there is
//             nothing to select — and every capacitor's two lead tips are exactly that. Flagging those
//             would report house convention, not a defect.
//   INERT     something is drawn without a working click target (.sch-ann) that is NOT one of the TAS
//             components the BOM excludes on purpose — i.e. a real part the user cannot open. This is
//             what keeps the .sch-ann escape hatch from quietly swallowing a missing part.
// Points are sampled along the real stroke geometry (getPointAtLength), not from bounding boxes: a
// bounding box of a transformer arc is mostly empty space, and hit-testing empty space proves nothing.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { HARNESS_CSS as CSS } from './harnessCss.mjs'
import { chromium } from '@playwright/test'

const M = await init()
const browser = await chromium.launch()
const page = await browser.newPage()
const only = process.argv[2]

// Zero tolerance, and it has to be: the defect this exists to catch (flyback's clamp resistor sitting
// on the clamp capacitor's bottom lead tip, taking every click there) was FIVE PERCENT of that
// capacitor's ink — one probe point. A 10 % share threshold, which looks reasonable, was measured
// against the real bug and would have passed it. There is no legitimate reason for one part's click
// target to cover another part's ink at all, so any single point is a finding.
const SHARE = Number(process.env.KH_HOTSPOT_SHARE ?? 0)

async function probe(svg) {
  const [, , W, H] = svg.match(/viewBox="([-\d.]+) ([-\d.]+) ([\d.]+) ([\d.]+)"/).slice(1).map(Number)
  await page.setViewportSize({ width: Math.ceil(W) + 20, height: Math.ceil(H) + 20 })
  await page.setContent(`<style>${CSS}svg{width:${W}px;height:${H}px}</style>${svg}`)
  await page.evaluate(() => document.fonts.ready)
  return page.evaluate(() => {
    const root = document.querySelector('svg')
    const m = root.getScreenCTM()
    const out = []
    for (const g of document.querySelectorAll('g.sch-hot:not(.sch-ann)')) {
      const ref = g.dataset.ref
      const hits = []
      for (const e of g.querySelectorAll('.sch-sym, .sch-fill')) {
        if (typeof e.getTotalLength !== 'function') continue
        let len = 0
        try { len = e.getTotalLength() } catch { continue }
        if (!len) continue
        const n = Math.max(4, Math.min(40, Math.round(len / 4)))
        for (let i = 0; i <= n; i++) {
          const p = e.getPointAtLength((len * i) / n)
          const sx = m.a * p.x + m.c * p.y + m.e, sy = m.b * p.x + m.d * p.y + m.f
          const hit = document.elementFromPoint(sx, sy)
          hits.push({ kind: 'symbol', x: Math.round(p.x), y: Math.round(p.y), got: hit?.closest('[data-ref]')?.dataset.ref ?? null,
                      by: hit ? `${hit.tagName}.${hit.getAttribute('class') ?? ''}` : 'nothing' })
        }
      }
      // The refdes/value block is the other thing a reader points at — it is the part's NAME, printed in
      // the clear away from the symbol, and it sits inside the same hot group. Whether a click there
      // reaches the part depends on paint order against every hitbox it happens to lie in, which is a
      // different question from the symbol's own ink and has to be probed separately.
      for (const t of g.querySelectorAll('text')) {
        const b = t.getBBox()
        for (let i = 1; i <= 12; i++) {
          const p = { x: b.x + (b.width * i) / 13, y: b.y + b.height / 2 }
          const sx = m.a * p.x + m.c * p.y + m.e, sy = m.b * p.x + m.d * p.y + m.f
          const hit = document.elementFromPoint(sx, sy)
          // Every point INSIDE the label's box counts, glyph or gap: a reader aims at the word, not at
          // the ink of one stem. Filtering to glyph hits would skip precisely the case this looks for —
          // a foreign hitbox lying over the label returns its rect, never the text.
          hits.push({ kind: `label "${t.textContent}"`, x: Math.round(p.x), y: Math.round(p.y),
                      got: hit?.closest('[data-ref]')?.dataset.ref ?? null,
                      by: hit ? `${hit.tagName}.${hit.getAttribute('class') ?? ''}` : 'nothing' })
        }
      }
      out.push({ ref, hits })
    }
    return { parts: out, annotations: [...document.querySelectorAll('g.sch-ann')].map((g) => g.dataset.ref) }
  })
}

// Which refs are allowed to be drawn WITHOUT a working click target. Exactly the TAS components the BOM
// leaves out on purpose (src/bom.js): a FET's intrinsic body diode, and the ngspice convergence aids.
// Anything else drawn inert would be a real part the user cannot open — the failure this pins down.
const annotatable = (tas) => {
  const out = new Set()
  for (const st of tas.topology?.stages ?? [])
    for (const c of st.circuit?.components ?? []) {
      const req = c.data?.inputs?.designRequirements ?? {}
      if (req.role === 'bodyDiode' || req.name === '__kh_numerical_aid__') out.add(c.name)
    }
  return out
}

let flagged = 0, totalPts = 0, badPts = 0
for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = opt ? `${t.id}/${opt}` : t.id
    if (only && only !== key && only !== t.id) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${key}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    const { svg } = renderForAudit(t.id, tas, opt ?? 'standard')
    const { parts, annotations } = await probe(svg)
    const bad = []
    // WELL-FORMEDNESS. setContent parses the drawing as HTML, which forgives nearly everything; an SVG
    // that is going anywhere else (a file export, a PDF, any XML consumer) has to be a real document. A
    // stray & or < from a component name or a value string would go unnoticed in the app and break the
    // moment the drawing leaves it — so parse it strictly, once per combo, and report what the parser says.
    const xmlError = await page.evaluate((src) => {
      const doc = new DOMParser().parseFromString(src, 'image/svg+xml')
      return doc.querySelector('parsererror')?.textContent?.replace(/\s+/g, ' ').trim().slice(0, 200) ?? null
    }, svg)
    if (xmlError) bad.push(`MALFORMED the drawing is not well-formed XML: ${xmlError}`)
    const allowed = annotatable(tas)
    for (const ref of annotations)
      if (!allowed.has(ref))
        bad.push(`INERT ${ref}: drawn without a working click target, but it is not a body diode or a ` +
                 `numerical aid — it is a part the user cannot open`)
    // A part that contributed NO probe points was never tested: its symbol may be drawn with elements the
    // sampler cannot walk (getTotalLength is an SVGGeometryElement method — a <g> wrapper or an <image>
    // has none), and it would sail through this gate invisibly, which is the failure mode this whole file
    // exists to prevent in the drawing.
    for (const { ref, hits } of parts)
      if (!hits.length) bad.push(`UNPROBED ${ref}: no walkable geometry — this gate never tested it`)
    for (const { ref, hits } of parts) {
      // symbol ink and each label are judged on their OWN denominator: a 5-glyph refdes swamped by 200
      // points of transformer winding would never clear any share threshold.
      for (const kind of new Set(hits.map((h) => h.kind))) {
        const pts = hits.filter((h) => h.kind === kind)
        totalPts += pts.length
        const wrong = pts.filter((h) => h.got && h.got !== ref)
        // a conductor drawn over the lead it connects to is not a lost click — see the header
        const CONDUCTOR = /sch-(wire|ctl|node|sig)/
        const dead = pts.filter((h) => !h.got && !CONDUCTOR.test(h.by))
        badPts += wrong.length + dead.length
        const by = new Map()
        for (const h of wrong) by.set(h.got, (by.get(h.got) ?? 0) + 1)
        const where = kind === 'symbol' ? 'its own symbol' : `its ${kind}`
        for (const [other, n] of by)
          if (n > 0 && n / pts.length >= SHARE)
            bad.push(`MISROUTE ${ref}: ${(100 * n / pts.length).toFixed(0)}% of ${where} clicks through to ${other} ` +
                     `(e.g. ${wrong.find((h) => h.got === other).x},${wrong.find((h) => h.got === other).y})`)
        if (dead.length > 0 && dead.length / pts.length >= SHARE)
          bad.push(`DEAD ${ref}: ${(100 * dead.length / pts.length).toFixed(0)}% of ${where} selects nothing ` +
                   `(e.g. ${dead[0].x},${dead[0].y} covered by ${[...new Set(dead.map((d) => d.by))].join(', ')})`)
      }
    }
    if (bad.length) { flagged++; console.log(`\n== ${key}`); for (const b of bad) console.log('   ' + b) }
  }
}
await browser.close()
console.log(`\n${totalPts} probe points on component ink; ${badPts} resolve to the wrong part or to nothing`)
// A sweep that probed nothing would print "0 bad" and exit 0 — green because it never looked.
if (!totalPts) throw new Error('checkSchematicHotspots probed 0 points: no clickable component ink was found at all')
console.log(flagged ? `${flagged} combo(s) with a click that misses its part` : 'every component click resolves to the component under the pointer')
process.exit(flagged ? 1 : 0)   // a gate that cannot fail is not a gate
