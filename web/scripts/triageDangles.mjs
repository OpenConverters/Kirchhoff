// Triage helper for rule E (dangling wire ends): for every flagged endpoint, print what is NEAREST
// to it (wire, registered pin, component hitbox) so a human can tell a real drawing artifact from a
// symbol the checker doesn't know about.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
// Renders through the SAME entry point the app uses (renderForAudit). Every topology is generated from
// CIAS now, but the rule stands: measure what the product draws, never a reconstruction of it.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { wireGraph, danglingEnds } from '../src/schematicCheck.js'
const M = await init()

const only = process.argv[2]
for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  if (only && t.id !== only) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) continue
    const tas = JSON.parse(out).tas
    const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
    const bad = danglingEnds(svg, pins)
    if (!bad.length) continue
    const { wires } = wireGraph(svg)
    const boxes = [...svg.matchAll(/<rect class="sch-hitbox" x="([-\d.]+)" y="([-\d.]+)" width="([\d.]+)" height="([\d.]+)"/g)].map((m) => m.slice(1).map(Number))
    const refs = [...svg.matchAll(/data-ref="([^"]+)"/g)].map((m) => m[1])
    const d2seg = (p, [a, b]) => {
      const dx = b[0] - a[0], dy = b[1] - a[1], l2 = dx * dx + dy * dy
      if (l2 < 1e-9) return Math.hypot(p[0] - a[0], p[1] - a[1])
      const s = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
      return Math.hypot(p[0] - (a[0] + s * dx), p[1] - (a[1] + s * dy))
    }
    console.log(`\n== ${t.id}${opt ? '/' + opt : ''}`)
    for (const msg of bad) {
      const [x, y] = msg.match(/\((-?[\d.]+),(-?[\d.]+)\)/).slice(1).map(Number)
      let bw = Infinity, bwSeg = null
      wires.forEach((s, i) => { const d = d2seg([x, y], s); if (d > 0.001 && d < bw && !(s[0][0] === x && s[0][1] === y) && !(s[1][0] === x && s[1][1] === y)) { bw = d; bwSeg = s } })
      let bp = Infinity, bpn = null
      for (const p of pins) { const d = Math.hypot(p.x - x, p.y - y); if (d < bp) { bp = d; bpn = `${p.ref}|${p.pin}` } }
      let bb = Infinity, bbn = null
      boxes.forEach(([bx, by, bwid, bh], i) => {
        const dx = Math.max(bx - x, 0, x - (bx + bwid)), dy = Math.max(by - y, 0, y - (by + bh))
        const d = Math.hypot(dx, dy); if (d < bb) { bb = d; bbn = refs[i] }
      })
      console.log(`  (${x},${y})  nearest wire ${bw.toFixed(1)} px ${bwSeg ? JSON.stringify(bwSeg) : ''} | nearest pin ${bp.toFixed(1)} px ${bpn} | nearest part ${bb.toFixed(1)} px ${bbn}`)
    }
  }
}
