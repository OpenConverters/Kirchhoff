// Is every refdes label near the part it names? (run: `node scripts/checkLabelAnchoring.mjs`)
//
// The collision rules only ask whether a label sits ON something. They never ask whether it sits NEAR
// its own component — so a label pushed clear of a wire can end up floating in white space, closer to a
// neighbour than to the part it labels. That is a real misreading risk on a dense secondary (pshb's Dr1
// drifted ~46 px from its diode while every rule stayed green), and it is invisible to every other check.
//
// A fixed distance threshold just reports house convention (psfb parks T1's label 112 px away on
// purpose, to clear the secondary). The threshold-free question is the one that actually misleads:
// is the label closer to a DIFFERENT component than to the one it names?
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { collectPins, hasSchematic } from '../src/schematics.js'
// Renders through the SAME entry point the app uses: for a topology with a CIAS layout the product
// draws THAT, not the hand-authored art, so auditing collectPins() directly measured a drawing the
// user never sees (see renderForAudit in ciasSchematic.js).
import { renderForAudit } from '../src/ciasSchematic.js'
import { measure } from './auditSchematicLabels.mjs'
import { chromium } from '@playwright/test'

const M = await init()
const browser = await chromium.launch()
const page = await browser.newPage()
const only = process.argv[2]

// Rectilinear gap between two boxes (0 if they touch or overlap).
const gap = (a, b) => Math.hypot(
  Math.max(0, Math.max(a.x - (b.x + b.w), b.x - (a.x + a.w))),
  Math.max(0, Math.max(a.y - (b.y + b.h), b.y - (a.y + a.h))))

let flagged = 0, worst = []
for (const t of TOPOLOGIES) {
  if (!hasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = opt ? `${t.id}/${opt}` : t.id
    if (only && only !== key && only !== t.id) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${t.id}${opt ? '/' + opt : ''}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    const { svg } = renderForAudit(t.id, tas, opt ?? 'standard')
    const { texts, boxes } = await measure(page, svg)
    const bad = []
    for (const tx of texts) {
      if (!tx.ref) continue                                  // only labels that belong to a part
      const own = boxes.find((b) => b.ref === tx.ref)
      if (!own) continue
      const dOwn = gap(tx, own)
      let near = null, dNear = Infinity
      for (const b of boxes) {
        if (b.ref === tx.ref) continue
        const d = gap(tx, b)
        if (d < dNear) { dNear = d; near = b.ref }
      }
      // A label ALIGNED with its part (above/below it, or beside it at the same height) reads
      // unambiguously even when some neighbour is nearer in a straight line — the eye uses the
      // alignment. Only a label that is diagonal from its own part, aligned with it on neither axis,
      // is genuinely up for grabs.
      const alignedX = Math.min(tx.x + tx.w, own.x + own.w) - Math.max(tx.x, own.x) > 0
      const alignedY = Math.min(tx.y + tx.h, own.y + own.h) - Math.max(tx.y, own.y) > 0
      worst.push({ key, ref: tx.ref, str: tx.str, d: dOwn })
      if (!alignedX && !alignedY && dNear < dOwn)
        bad.push(`${tx.ref}: "${tx.str}" is ${dOwn.toFixed(0)} px from ${tx.ref}, aligned with neither axis of it, and only ${dNear.toFixed(0)} px from ${near}`)
    }
    if (bad.length) { flagged++; console.log(`\n== ${key}`); for (const b of [...new Set(bad)]) console.log('   ' + b) }
  }
}
worst.sort((a, b) => b.d - a.d)
console.log(`\nwidest gaps: ${worst.slice(0, 5).map((w) => `${w.key}/${w.ref} ${w.d.toFixed(0)}px`).join(', ')}`)
console.log(flagged ? `\n${flagged} combo(s) where a label sits nearer another part than its own` : '\nevery label is nearer its own part than any other')
await browser.close()
process.exit(flagged ? 1 : 0)
