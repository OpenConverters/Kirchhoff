// Triage aid for T-WIRE collisions (run: `node scripts/triageLabels.mjs [topology[/variant]]`).
//
// auditSchematicLabels tells you THAT a label lies across a wire; fixing it needs the measured box and
// what room there is on each side. This prints, per collision: the label's measured box, the offending
// wire, and how far the label would have to move (or which side is clear) — so a fix is a decision, not
// a guess-and-recheck loop.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
// Renders through the SAME entry point the app uses (renderForAudit). Every topology is generated from
// CIAS now, but the rule stands: measure what the product draws, never a reconstruction of it.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { measure } from './auditSchematicLabels.mjs'
import { chromium } from '@playwright/test'

const M = await init()
const browser = await chromium.launch()
const page = await browser.newPage()
const only = process.argv[2]

for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = opt ? `${t.id}/${opt}` : t.id
    if (only && only !== key && only !== t.id) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) continue
    const { svg } = renderForAudit(t.id, JSON.parse(out).tas, opt ?? 'standard')
    const { texts, wires } = await measure(page, svg)
    const hits = []
    for (const tx of texts) for (const s of wires) {
      const [p, q] = s
      const [x0, x1] = [Math.min(p[0], q[0]), Math.max(p[0], q[0])]
      const [y0, y1] = [Math.min(p[1], q[1]), Math.max(p[1], q[1])]
      const horiz = p[1] === q[1] && p[1] > tx.y + 2 && p[1] < tx.y + tx.h - 2 &&
        Math.min(tx.x + tx.w, x1) - Math.max(tx.x, x0) > 2
      const vert = p[0] === q[0] && p[0] > tx.x + 2 && p[0] < tx.x + tx.w - 2 &&
        Math.min(tx.y + tx.h, y1) - Math.max(tx.y, y0) > 2
      if (!horiz && !vert) continue
      hits.push(vert
        ? `[${tx.ref ?? "-"}] "${tx.str}" box x[${tx.x.toFixed(0)},${(tx.x + tx.w).toFixed(0)}] y[${tx.y.toFixed(0)},${(tx.y + tx.h).toFixed(0)}] ` +
          `| VERTICAL wire x=${p[0]} y[${y0},${y1}] | move label left by ${(tx.x + tx.w - p[0] + 3).toFixed(0)} or right by ${(p[0] - tx.x + 3).toFixed(0)}`
        : `[${tx.ref ?? "-"}] "${tx.str}" box x[${tx.x.toFixed(0)},${(tx.x + tx.w).toFixed(0)}] y[${tx.y.toFixed(0)},${(tx.y + tx.h).toFixed(0)}] ` +
          `| HORIZONTAL wire y=${p[1]} x[${x0},${x1}] | move label up by ${(tx.y + tx.h - p[1] + 3).toFixed(0)} or down by ${(p[1] - tx.y + 3).toFixed(0)}`)
    }
    if (hits.length) { console.log(`\n== ${key}`); for (const h of [...new Set(hits)]) console.log('   ' + h) }
  }
}
await browser.close()
