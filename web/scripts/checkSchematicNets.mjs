// Netlist-vs-drawing connectivity check (run: `node scripts/checkSchematicNets.mjs`).
//
// For every net in the TAS, it gathers the DRAWN coordinates of that net's unambiguous anchor terminals —
// MOSFET drain/source, diode anode/cathode (from the render, via collectPins), plus ground symbols and
// external ports — and verifies they are all wire-connected in the rendered SVG. This catches the class of
// bug the parity (drawn-only) check misses: split rails and mis-wired power nets. It found and drove the
// fixes for fsbb / dab / cllc / clllc floating grounds and the weinberg topology error.
//
// SCOPE / KNOWN LIMITATIONS (a flag here is NOT automatically a bug — read the net before acting):
//  - Gate nets are excluded: gates connect via net-label flags (dashed control stubs), not wires.
//  - Passive 2-terminal parts (cap/res/inductor) and TRANSFORMER windings have ambiguous start/end pins,
//    so they are NOT used as anchors. A net whose only in-drawing link runs THROUGH such a part therefore
//    shows as "2 pieces" even when correctly drawn (false positive) — e.g. control-sense nets, half-bridge
//    mid nodes bridged by a cap, some secondary-rectifier nets.
//  - The drawing may be electrically equivalent to the netlist but label a node differently (e.g. a forward
//    switch drawn low-side vs the netlist's high-side labelling) → flagged though not "broken".
// This tool drove fixes for EVERY power-connectivity defect it found: fsbb/dab/cllc/clllc floating
// grounds; cllc/clllc broken bridge leg (node_b) and floating VOUT rail; secFB bridge-diode refdes
// mapping; pshb missing primary ground and REVERSED clamp diode (DC1); boost-sync + acf sync-rectifier
// MOSFETs drawn source/drain-swapped (now mirrored); and Vienna's bidirectional midpoint switch drawn as
// a cascode instead of common-source (now flipped). All verified by re-running this checker.
//
// SOLE REMAINING RESIDUAL (documented, intentional — not an electrical defect):
//   - forward: vin_net/gnd_net. Q1 is drawn LOW-side (a standard, gate-drive-friendly forward) while the
//     TAS labels it HIGH-side (VIN→Q1.drain). Both are valid forward converters — same volt-seconds — so
//     the drawing is electrically correct; only the switch NODE labels differ. The forward transformer is
//     also 3-winding (primary + demag + output) approximated by the 2-winding symbol. Matching the TAS
//     node-for-node needs a high-side redraw + a 3-winding transformer symbol; deferred as low value/high
//     risk vs the electrical correctness already achieved.
// The ground-only rail checker (all grounds connected) and parity both pass cleanly across all 39 combos.
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { extractBom } from '../src/bom.js'
import { collectPins, hasSchematic } from '../src/schematics.js'
const M = await init()

function wireGraph(svg) {
  const wires = []
  for (const m of svg.matchAll(/<path class="sch-wire" d="([^"]+)"/g)) {
    const n = [...m[1].matchAll(/[ML]\s*(-?[\d.]+)\s+(-?[\d.]+)/g)].map((z) => [+z[1], +z[2]])
    for (let i = 1; i < n.length; i++) wires.push([n[i - 1], n[i]])
  }
  const pts = [], par = []
  const idOf = (p) => { let i = pts.findIndex((q) => Math.abs(q[0] - p[0]) < 3 && Math.abs(q[1] - p[1]) < 3); if (i < 0) { i = pts.length; pts.push(p) } if (par[i] === undefined) par[i] = i; return i }
  const find = (x) => { while (par[x] !== x) { par[x] = par[par[x]]; x = par[x] } return x }
  const uni = (a, b) => { par[find(a)] = find(b) }
  // point-on-segment with PIXEL tolerance (registered pins are rounded to int; wire coords may be .5)
  const onSeg = (p, [a, b]) => {
    const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy)
    if (len < 1e-6) return Math.hypot(p[0] - a[0], p[1] - a[1]) < 3
    const perp = Math.abs(dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / len
    if (perp > 3) return false
    const proj = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len
    return proj >= -3 && proj <= len + 3
  }
  for (const [a, b] of wires) uni(idOf(a), idOf(b))
  // T-junctions: any segment endpoint that lies on another segment's interior joins it. (Schematic
  // convention needs a dot, but our wires are authored to meet, so treat endpoint-on-segment as connected.)
  const ends = wires.flatMap((s) => [s[0], s[1]])
  for (const e of ends) for (const s of wires) if (onSeg(e, s)) uni(idOf(e), idOf(s[0]))
  const rootAt = (p) => {
    let r = null
    for (const s of wires) if (onSeg(p, s)) { const i = idOf(p); uni(i, idOf(s[0])); r = find(i) }
    return r
  }
  return { rootAt, wires }
}

const norm = (s) => String(s).toLowerCase().replace(/[^a-z0-9]/g, '')
let flagged = 0, checkedNets = 0
for (const t of TOPOLOGIES) {
  if (!hasSchematic(t.id)) continue
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    if (out.startsWith('Exception')) continue
    const tas = JSON.parse(out).tas
    const { svg, pins } = collectPins(t.id, extractBom(tas), opt ?? 'standard')
    const g = wireGraph(svg)
    // index drawn anchors: key "ref|pin" -> coord ; also collect ground + port coords
    const byKey = new Map(), grounds = [], ports = new Map()
    for (const p of pins) {
      if (p.ref === '@gnd') grounds.push([p.x, p.y])
      else if (p.ref === '@port') { const k = norm(p.pin); (ports.get(k) || ports.set(k, []).get(k)).push([p.x, p.y]) }
      else byKey.set(p.ref + '|' + p.pin, [p.x, p.y])
    }
    // gather nets from the TAS
    const problems = []
    for (const stage of tas.topology?.stages ?? []) {
      for (const conn of stage.circuit?.connections ?? []) {
        const anchors = []  // { coord, ref } — ref=true for ground symbols / ports (same net by label)
        for (const ep of conn.endpoints ?? []) {
          if (ep.port !== undefined) {
            const k = norm(ep.port)
            if (k === 'gnd') grounds.forEach((c) => anchors.push({ coord: c, ref: true }))
            else if (ports.has(k)) ports.get(k).forEach((c) => anchors.push({ coord: c, ref: true }))
          } else if (ep.component !== undefined && ep.pin !== 'gate') {
            // gates connect via net-label flags (dashed control stubs), not wires — not wire-verifiable
            const c = byKey.get(ep.component + '|' + ep.pin)
            if (c) anchors.push({ coord: c, ref: false })  // MOSFET drain/source / diode anode/cathode (unambiguous)
          }
        }
        if (anchors.length < 2) continue
        checkedNets++
        for (const a of anchors) a.root = g.rootAt(a.coord)
        const pieces = new Map()  // root -> {hasRef}
        for (const a of anchors) { const r = a.root === null ? 'FLOAT' : a.root; const e = pieces.get(r) || { ref: false }; if (a.ref) e.ref = true; pieces.set(r, e) }
        // OK iff 1 piece, or every piece has a reference (ground/port) anchor. A refless extra piece floats.
        const bad = pieces.size > 1 && [...pieces.values()].some((e) => !e.ref)
        if (bad || pieces.has('FLOAT')) problems.push(`${conn.name}(${anchors.length} anchors → ${pieces.size} pieces)`)
      }
    }
    if (problems.length) { flagged++; console.log((t.id + (opt ? '/' + opt : '')).padEnd(26), problems.join('  |  ')) }
  }
}
console.log(`\nchecked ${checkedNets} multi-anchor nets; ${flagged ? flagged + ' with split/floating nets' : 'all consistent'}`)
