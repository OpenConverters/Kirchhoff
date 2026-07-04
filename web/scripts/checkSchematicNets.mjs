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
// Residual flags as of this commit have been triaged: predominantly the above false-positive categories,
// plus a couple worth a look in the ORIGINAL secondary-rectifier / resonant-bridge code (not the layout
// rewrite): cllc/clllc `node_b`. The ground-only rail checker (all grounds connected) passes cleanly.
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
  const onSeg = (p, [a, b]) => { const cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]); if (Math.abs(cr) > 3) return false; const dd = (p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1]); const l2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2; return dd >= -3 && dd <= l2 + 3 }
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
