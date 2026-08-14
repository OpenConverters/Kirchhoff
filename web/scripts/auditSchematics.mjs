// Drawing-quality audit of the rendered SVG schematics (run: `node scripts/auditSchematics.mjs`).
//
// checkSchematicNets.mjs proves the drawing matches the netlist (rules A-F) — this proves it READS
// right. Every class below was found flagging a real defect on layouts that passed every net rule:
// a snubber drawn across the wrong node, a diode wired into the middle of an inductor coil, a rail
// redrawn on top of another for 135 px, a wire straight through a FET body.
//
// PHANTOM is measured against the TAS, not the BOM: body diodes (role:"bodyDiode") ARE real TAS
// components, they are only filtered OUT of the BOM because they are intrinsic to their MOSFET. So
// buck/boost's 'D2' and isolated_buck's 'DS1'/'DS2' are legitimate annotations, not inventions.
//
// The checks:
//   1 PHANTOM   a data-ref drawn that the BOM does not contain (the parity guard only checks BOM→drawn)
//   2 OVERLAP   two collinear wire segments overlapping (a wire redrawn on top of another)
//   3 THROUGH   a wire segment passing straight THROUGH a component footprint (in one side, out the other)
//   4 T-DOT     a wire endpoint landing in the INTERIOR of another wire (a real T-junction) with no dot
//   5 DOT       a junction dot that sits on no wire at all
//   6 CROSS-DOT a wire crossing that sits within 15 px of a junction dot on one of the crossing wires,
//               so the two read as one three-way node (psfb drew its tank shorted to the switch node)
//   7 BOUNDS    drawn geometry outside the viewBox
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
// Renders through the SAME entry point the app uses (renderForAudit). Every topology is generated from
// CIAS now, but the rule stands: measure what the product draws, never a reconstruction of it.
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
// Dual-purpose: `node scripts/auditSchematics.mjs` sweeps every topology; importing it just exposes
// auditDrawing() so other checkers (checkSweep.mjs) can apply the same rules to their own renders.
const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])
const M = await init()

const seg = (svg) => {
  const out = []
  for (const m of svg.matchAll(/<path class="sch-wire" d="([^"]+)"/g)) {
    const n = [...m[1].matchAll(/[ML]\s*(-?[\d.]+)\s+(-?[\d.]+)/g)].map((z) => [+z[1], +z[2]])
    for (let i = 1; i < n.length; i++) out.push([n[i - 1], n[i]])
  }
  return out
}
const boxes = (svg) => {
  const refs = [...svg.matchAll(/data-ref="([^"]+)"/g)].map((m) => m[1])
  return [...svg.matchAll(/<rect class="sch-hitbox" x="([-\d.]+)" y="([-\d.]+)" width="([\d.]+)" height="([\d.]+)"/g)]
    .map((m, i) => ({ ref: refs[i], x: +m[1], y: +m[2], w: +m[3], h: +m[4] }))
}
const dots = (svg) => [...svg.matchAll(/<circle class="sch-node" cx="([-\d.]+)" cy="([-\d.]+)"/g)].map((m) => [+m[1], +m[2]])
// The DRAWN body per refdes, which is what a reader sees — not the hitbox. A MOSFET/BJT is a circle
// inscribed in a square hitbox, so a wire clipping the hitbox CORNER is nowhere near the glyph; judging
// bodies by the rect alone reported psfb's QD and three other switches as "wire inside the body" when
// the wire passes cleanly outside the circle. Round glyphs are measured as circles, everything else as
// its rect.
const bodies = (svg) => {
  const out = []
  for (const g of svg.matchAll(/<g class="sch-hot" data-ref="([^"]+)">([\s\S]*?)<\/g>/g)) {
    const [, ref, inner] = g
    const c = inner.match(/<circle class="sch-sym" cx="([-\d.]+)" cy="([-\d.]+)" r="([\d.]+)"/)
    const r = inner.match(/<rect class="sch-hitbox" x="([-\d.]+)" y="([-\d.]+)" width="([\d.]+)" height="([\d.]+)"/)
    if (c && +c[3] > 8) out.push({ ref, kind: 'circle', cx: +c[1], cy: +c[2], r: +c[3] })
    else if (r) out.push({ ref, kind: 'rect', x: +r[1], y: +r[2], w: +r[3], h: +r[4] })
  }
  return out
}
// Length of the part of an axis-aligned segment that lies strictly INSIDE a drawn body (0 if none).
const insideRun = (s, body) => {
  const k = s[0][0] === s[1][0] ? 1 : (s[0][1] === s[1][1] ? 0 : -1)   // 1 = vertical, 0 = horizontal
  if (k < 0) return 0
  const fixed = k === 1 ? s[0][0] : s[0][1]
  const [slo, shi] = [Math.min(s[0][k], s[1][k]), Math.max(s[0][k], s[1][k])]
  let blo, bhi
  if (body.kind === 'circle') {
    const perp = k === 1 ? body.cx : body.cy
    const half = Math.sqrt(Math.max(0, body.r * body.r - (fixed - perp) ** 2))
    if (half <= 0) return 0
    const par = k === 1 ? body.cy : body.cx
    ;[blo, bhi] = [par - half, par + half]
  } else {
    const [flo, fhi] = k === 1 ? [body.x, body.x + body.w] : [body.y, body.y + body.h]
    if (fixed <= flo || fixed >= fhi) return 0
    ;[blo, bhi] = k === 1 ? [body.y, body.y + body.h] : [body.x, body.x + body.w]
  }
  return Math.max(0, Math.min(shi, bhi) - Math.max(slo, blo))
}
const near = (a, b, t = 3) => Math.abs(a[0] - b[0]) <= t && Math.abs(a[1] - b[1]) <= t
const onSpan = (p, [a, b]) => {          // strictly INSIDE the segment (not at either end)
  if (a[0] === b[0] && Math.abs(p[0] - a[0]) <= 2) {
    const [lo, hi] = [Math.min(a[1], b[1]), Math.max(a[1], b[1])]
    return p[1] > lo + 2 && p[1] < hi - 2
  }
  if (a[1] === b[1] && Math.abs(p[1] - a[1]) <= 2) {
    const [lo, hi] = [Math.min(a[0], b[0]), Math.max(a[0], b[0])]
    return p[0] > lo + 2 && p[0] < hi - 2
  }
  return false
}
const inBox = (p, bx, pad = 0) =>
  p[0] > bx.x - pad && p[0] < bx.x + bx.w + pad && p[1] > bx.y - pad && p[1] < bx.y + bx.h + pad

// One rendered schematic vs its TAS -> string[] problems. Exported so checkSweep.mjs can run the same
// rules over many design points without duplicating them.
export function auditDrawing(svg, tasRefs, pins = []) {
    const S = seg(svg), B = boxes(svg), D = dots(svg), BODY = bodies(svg)
    // How much overlap counts as "drawn over the body". A circle body is the REAL drawn radius, so a few
    // px of grazing is all the slack needed. A rect body is the HITBOX, which is padded past the glyph
    // (a cap's plates are ~20 px inside its 40 px box, a transformer's terminals sit 12 px in from the
    // winding-block edge) — measured against that, a clean wire routinely clips a corner by ~12 px.
    // Verified case by case against zoomed renders; every real defect found here runs 14 px or more.
    const leadTol = (b) => (b.kind === 'circle' ? 6 : 12)
    // A wire endpoint inside a component's body is a CONNECTION only if it lands on one of that
    // component's own pins. Exempting every inside-endpoint (what rule 3 used to do) let a wire enter
    // a body, cross it and leave: DAB routed the primary mid-node -> Lr straight through the U1 PWM
    // block, its corner vertex 2 px inside the block, and every rule stayed silent because that vertex
    // read as "connects to U1". Blocks like U1 declare no pins at all, so nothing may end inside them.
    const [, , W, H] = svg.match(/viewBox="([-\d.]+) ([-\d.]+) ([\d.]+) ([\d.]+)"/).slice(1).map(Number)
    const problems = []
    for (const b of B) if (!tasRefs.has(b.ref)) problems.push(`PHANTOM ${b.ref} drawn but is not a TAS component at all`)
    for (let i = 0; i < S.length; i++) for (let j = i + 1; j < S.length; j++) {
      const [a, b] = [S[i], S[j]]
      const vert = a[0][0] === a[1][0] && b[0][0] === b[1][0] && a[0][0] === b[0][0]
      const horz = a[0][1] === a[1][1] && b[0][1] === b[1][1] && a[0][1] === b[0][1]
      if (!vert && !horz) continue
      const k = vert ? 1 : 0
      const lo = Math.max(Math.min(a[0][k], a[1][k]), Math.min(b[0][k], b[1][k]))
      const hi = Math.min(Math.max(a[0][k], a[1][k]), Math.max(b[0][k], b[1][k]))
      if (hi - lo > 0.5) problems.push(`OVERLAP wires ${JSON.stringify(a)} / ${JSON.stringify(b)} share ${(hi - lo).toFixed(0)} px`)
    }
    // Rule 3, rewritten: NO wire may be drawn over a component's body, except the leads of that
    // component's own pins. The old form asked only "does a wire pass clean through a hitbox", and
    // exempted any segment with an endpoint inside — so a wire that entered a body and stopped, or
    // entered and turned, was invisible to it. DAB routed the primary mid-node -> Lr straight across the
    // U1 PWM block; cllc/clllc ran the mid-node down through the middle of Lr1's coil and landed
    // junction dots inside two switch bodies. All of it passed every rule.
    for (const s of S) for (const b of BODY) {
      const run = insideRun(s, b)
      if (run <= leadTol(b)) continue
      // A wire that REACHES one of this part's own pins overlaps the glyph by design — a cap's lead runs
      // from its pin in to its plate, a transformer's from its pin in to the winding. So: exempt when the
      // segment is collinear with one of the part's pins and its span actually reaches that pin. A wire
      // merely passing over the glyph satisfies neither, whatever direction it runs.
      const k = s[0][0] === s[1][0] ? 1 : 0        // 1 = vertical
      const [slo, shi] = [Math.min(s[0][k], s[1][k]), Math.max(s[0][k], s[1][k])]
      const reachesPin = pins.some((q) => q.ref === b.ref &&
        Math.abs((k === 1 ? q.x : q.y) - (k === 1 ? s[0][0] : s[0][1])) <= 2 &&
        (k === 1 ? q.y : q.x) >= slo - 2 && (k === 1 ? q.y : q.x) <= shi + 2)
      if (reachesPin) continue
      problems.push(`OVER-BODY wire ${JSON.stringify(s)} is drawn over ${b.ref}'s body for ${run.toFixed(0)} px (on no pin lead)`)
    }
    for (const s of S) for (const e of s) {
      const tee = S.some((o) => o !== s && onSpan(e, o))
      if (tee && !D.some((d) => near(d, e, 4))) problems.push(`T-DOT junction at (${e}) has no dot`)
    }
    // The other way three conductors meet: not a wire ending inside another's span, but several ENDS
    // arriving at one point — two wire ends plus a component terminal is a part tapping the corner where
    // two wires join, and that is a junction. T-DOT above cannot see it. Counted the way the CIAS
    // generator derives its dots: a polyline END is one conductor, an interior CORNER is two (it is the
    // same wire turning, which is why a plain corner never earns a dot), a terminal is one. A gate pin is
    // driven by a net-label flag, not a wire, and a ground/port glyph is a reference rather than a
    // conductor joining the node — house style puts no dot on either.
    const meet = new Map()
    const bump = (k, n) => meet.set(k, (meet.get(k) ?? 0) + n)
    for (const m of svg.matchAll(/<path class="sch-wire" d="([^"]+)"/g)) {
      const n = [...m[1].matchAll(/[ML]\s*(-?[\d.]+)\s+(-?[\d.]+)/g)].map((z) => [+z[1], +z[2]])
      n.forEach((pt, i) => bump(`${pt[0]},${pt[1]}`, i === 0 || i === n.length - 1 ? 1 : 2))
    }
    for (const q of pins)
      if (q.pin !== 'gate' && !['@gnd', '@sgnd', '@port'].includes(q.ref)) bump(`${q.x},${q.y}`, 1)
    for (const [k, n] of meet) {
      if (n < 3) continue
      const e = k.split(',').map(Number)
      if (!D.some((d) => near(d, e, 4))) problems.push(`T-DOT ${n} conductors meet at (${e}) with no dot`)
    }
    for (const d of D) if (!S.some((s) => onSpan(d, s) || near(d, s[0], 3) || near(d, s[1], 3)))
      problems.push(`DOT at (${d}) sits on no wire`)
    // ORPHAN-FLAG. Gate drive and sense are drawn as net LABELS, not wires: two flags carrying the same
    // name are the same net (standard practice, and the only way to avoid dashed spaghetti across the
    // power path). A name that appears exactly once therefore connects to nothing — a switch nobody
    // drives, or a controller pin driving nobody — and every rule here is blind to it, because a flag
    // is a text and a stub rather than a wire.
    const flagCount = new Map()
    for (const m of svg.matchAll(/<text class="sch-sig"[^>]*>([^<]*)<\/text>/g))
      flagCount.set(m[1], (flagCount.get(m[1]) ?? 0) + 1)
    for (const [name, n] of flagCount)
      if (n < 2) problems.push(`ORPHAN-FLAG signal '${name}' is labelled once — it names a net with one end`)
    // CROSS-DOT. A reader resolves a crossing by looking for a dot, so a crossing is only unambiguous
    // while the nearest dot plainly belongs somewhere else. psfb ran the Lr→T1 tank through the leg-C
    // column 10 px above midC's junction dot; pshb took the primary return across the NPC stack 10 px
    // below bridge_a's. Both drew what looks like a single three-way node — the first says the resonant
    // tank is shorted to the switch node, the second that the neutral is, which is the exact short an
    // NPC leg's clamp diodes exist to prevent. Every net rule passed: the nets ARE separate, it is the
    // picture that lies. 15 px is one line of the schematic's own type — nearer than that the dot and
    // the crossing read as one blob. (Legitimate crossings clear their dots by 20 px+: dab, llc, src.)
    const isH = (s) => s[0][1] === s[1][1], isV = (s) => s[0][0] === s[1][0]
    for (const a of S.filter(isH)) for (const b of S.filter(isV)) {
      const y = a[0][1], x = b[0][0]
      const [x0, x1] = [Math.min(a[0][0], a[1][0]), Math.max(a[0][0], a[1][0])]
      const [y0, y1] = [Math.min(b[0][1], b[1][1]), Math.max(b[0][1], b[1][1])]
      // strictly inside BOTH spans — a corner or a T is a junction, not a crossing, and has its own rule
      if (!(x > x0 + 1 && x < x1 - 1 && y > y0 + 1 && y < y1 - 1)) continue
      for (const d of D) {
        if (!((d[1] === y && d[0] >= x0 && d[0] <= x1) || (d[0] === x && d[1] >= y0 && d[1] <= y1))) continue
        const gap = Math.hypot(d[0] - x, d[1] - y)
        if (gap < 15) problems.push(`CROSS-DOT wires cross at (${x},${y}) ${gap.toFixed(0)} px from the junction dot at (${d}) on the same wire — reads as one node`)
      }
    }
    for (const s of S) for (const e of s)
      if (e[0] < 0 || e[1] < 0 || e[0] > W || e[1] > H) problems.push(`BOUNDS wire end (${e}) outside ${W}x${H}`)
    for (const b of B)
      if (b.x < 0 || b.y < 0 || b.x + b.w > W || b.y + b.h > H) problems.push(`BOUNDS ${b.ref} outside ${W}x${H}`)
    return [...new Set(problems)]
}

let flagged = 0, audited = 0
if (isMain) for (const t of TOPOLOGIES) {
  if (!hasCiasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a topology this gate may skip: skipping it silently is how a
    // sweep reports "clean" over a schematic it never rendered.
    if (out.startsWith('Exception')) throw new Error(`${t.id}${opt ? '/' + opt : ''}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    const rows = extractBom(tas)
    const tasRefs = new Set((tas?.topology?.stages ?? []).flatMap((st) => (st.circuit?.components ?? []).map((c) => c.name)))
    const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
    // ONE implementation of the rules — the CLI used to re-inline all six, which is how a rule fix
    // silently reaches checkSweep but not this sweep (or the reverse).
    const problems = auditDrawing(svg, tasRefs, pins)
    audited++

    const uniq = [...new Set(problems)]
    if (uniq.length) { flagged++; console.log(`\n== ${t.id}${opt ? '/' + opt : ''}`); for (const p of uniq) console.log('   ' + p) }
  }
}
if (isMain) {
  // Say what was covered, and refuse to report success over nothing: this sweep skips a topology with no
  // CIAS layout, and "clean" over an empty sweep is a clean bill of health for zero drawings.
  if (!audited) throw new Error('auditSchematics drew 0 schematics — every topology was skipped')
  console.log(flagged ? `\n${flagged} of ${audited} combo(s) flagged` : `\nclean — ${audited} drawings audited`)
  process.exit(flagged ? 1 : 0)   // a gate that cannot fail is not a gate
}
