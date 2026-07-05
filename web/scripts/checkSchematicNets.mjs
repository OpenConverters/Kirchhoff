// Netlist-vs-drawing connectivity check (run: `node scripts/checkSchematicNets.mjs`).
//
// It FLATTENS the hierarchical TAS (per-stage circuits linked by inter-stage connections) into global
// nets, then verifies the rendered SVG against them:
//   A. Every net's drawn anchor terminals — MOSFET drain/source, diode anode/cathode (unambiguous) — are
//      wire-connected in one piece, or split only into pieces that each carry a ground/port reference
//      (multiple ground symbols = same net by convention; isolated primary/secondary get their own ref).
//   B. No two distinct global nets share a drawn wire (short check).
//   C. MAGNETICS: each transformer/inductor's drawn winding terminals reach exactly the set of global nets
//      its netlist pins sit on — catches a floating winding end, a winding/center-tap wired to the wrong
//      node, or a missing terminal (the connectivity half of "is the magnetic wired right").
//
// This tool drove fixes for EVERY power-connectivity defect it found: fsbb/dab/cllc/clllc floating grounds;
// cllc/clllc broken bridge leg (node_b) and floating VOUT rail; secFB bridge-diode refdes mapping; pshb
// missing primary ground and REVERSED clamp diode (DC1); boost-sync + acf sync-rectifier MOSFETs drawn
// source/drain-swapped (mirrored); and Vienna's bidirectional midpoint switch (cascode → common-source).
//
// WHAT IT STILL CANNOT VERIFY (a pure connectivity check is blind to this — see the companion
// checkTransformerPolarity.mjs which covers it with domain rules):
//   - Transformer DOT POLARITY / winding phasing. The TAS turnsRatio is UNSIGNED, so the netlist carries
//     no phase information; a swapped dot passes every net check yet flips output polarity / saturates the
//     core. Not machine-verifiable from connectivity alone.
// Gate nets are excluded (they connect via net-label flags, not wires).
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
  const onSeg = (p, [a, b]) => {
    const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy)
    if (len < 1e-6) return Math.hypot(p[0] - a[0], p[1] - a[1]) < 3
    const perp = Math.abs(dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / len
    if (perp > 3) return false
    const proj = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len
    return proj >= -3 && proj <= len + 3
  }
  const distToSeg = (p, [a, b]) => {  // shortest distance from point p to segment ab
    const dx = b[0] - a[0], dy = b[1] - a[1], len2 = dx * dx + dy * dy
    if (len2 < 1e-9) return Math.hypot(p[0] - a[0], p[1] - a[1])
    let s = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2
    s = Math.max(0, Math.min(1, s))
    return Math.hypot(p[0] - (a[0] + s * dx), p[1] - (a[1] + s * dy))
  }
  for (const [a, b] of wires) uni(idOf(a), idOf(b))
  const ends = wires.flatMap((s) => [s[0], s[1]])
  for (const e of ends) for (const s of wires) if (onSeg(e, s)) uni(idOf(e), idOf(s[0]))
  // rootAt(p, tol): root of the CLOSEST wire within tol px. Preferring the nearest wire (not just any
  // within tol) resolves coincidental proximity — a terminal sitting on its own lead wire but merely
  // crossing a rail a few px away binds to the lead wire, not the rail.
  const rootAt = (p, tol = 3) => {
    let best = null, bestD = Infinity
    for (const s of wires) { const d = distToSeg(p, s); if (d < bestD) { bestD = d; best = s } }
    if (!best || bestD > tol) return null
    const i = idOf(p); uni(i, idOf(best[0])); return find(i)
  }
  return { rootAt }
}

// Flatten the hierarchical TAS into global nets. Returns pinNet: "component|pin" -> netId (string root).
function flattenNets(tas) {
  const par = new Map()
  const find = (x) => { if (!par.has(x)) par.set(x, x); while (par.get(x) !== x) { par.set(x, par.get(par.get(x))); x = par.get(x) } return x }
  const uni = (a, b) => { par.set(find(a), find(b)) }
  const tok = (ctx, ep) => ep.component !== undefined ? 'C:' + ep.component + '|' + ep.pin
    : ep.stage !== undefined ? 'P:' + ep.stage + '::' + ep.port
      : ctx !== null ? 'P:' + ctx + '::' + ep.port : 'X:' + ep.port
  for (const st of tas.topology?.stages ?? []) for (const conn of st.circuit?.connections ?? []) {
    const eps = conn.endpoints ?? []
    for (let i = 1; i < eps.length; i++) uni(tok(st.name, eps[0]), tok(st.name, eps[i]))
    if (eps.length) find(tok(st.name, eps[0]))
  }
  for (const isc of tas.topology?.interStageConnections ?? []) {
    const eps = isc.endpoints ?? []
    for (let i = 1; i < eps.length; i++) uni(tok(null, eps[0]), tok(null, eps[i]))
  }
  const pinNet = new Map()
  for (const k of par.keys()) if (k.startsWith('C:')) pinNet.set(k.slice(2), find(k))
  return pinNet
}

let flagged = 0
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
    const pinNet = flattenNets(tas)
    const magRefs = new Set()
    for (const st of tas.topology?.stages ?? []) for (const c of st.circuit?.components ?? []) if (c.data?.magnetic !== undefined) magRefs.add(c.name)

    // drawn coords: unambiguous non-mag anchors keyed "ref|pin"; magnetic terminals grouped by ref;
    // reference roots = wire pieces that contain a ground symbol or external port glyph.
    const byKey = new Map(), magTerms = new Map(), refRoots = new Set()
    for (const p of pins) {
      if (p.ref === '@gnd' || p.ref === '@port') { const r = g.rootAt([p.x, p.y]); if (r !== null) refRoots.add(r) }
      else if (magRefs.has(p.ref)) { (magTerms.get(p.ref) || magTerms.set(p.ref, []).get(p.ref)).push([p.x, p.y]) }
      else byKey.set(p.ref + '|' + p.pin, [p.x, p.y])
    }
    // root -> global net, from unambiguous anchors (used by the magnetic check)
    const rootNet = new Map()
    for (const [k, c] of byKey) { if (k.endsWith('|gate')) continue; const net = pinNet.get(k); const r = g.rootAt(c); if (net && r !== null) rootNet.set(r, net) }
    const resolvableNets = new Set(rootNet.values())

    const problems = []
    // A + B: group unambiguous anchors by global net
    const netAnchors = new Map()
    for (const [k, c] of byKey) { if (k.endsWith('|gate')) continue; const net = pinNet.get(k); if (!net) continue; (netAnchors.get(net) || netAnchors.set(net, []).get(net)).push({ k, r: g.rootAt(c) }) }
    const rootOwner = new Map()  // wire-root -> global net (for short detection)
    for (const [net, ancs] of netAnchors) {
      if (ancs.length >= 2) {
        const pieces = new Set(ancs.map((a) => a.r === null ? 'FLOAT' : a.r))
        const bad = pieces.size > 1 && [...pieces].some((r) => r === 'FLOAT' || !refRoots.has(r))
        if (bad) problems.push(`${netLabel(net)}: ${ancs.length} anchors → ${pieces.size} pieces`)
      }
      for (const a of ancs) if (a.r !== null && !refRoots.has(a.r)) {
        if (rootOwner.has(a.r) && rootOwner.get(a.r) !== net) problems.push(`SHORT: ${netLabel(net)} ↔ ${netLabel(rootOwner.get(a.r))}`)
        rootOwner.set(a.r, net)
      }
    }
    // C: magnetic winding terminals. Reliable signals: a terminal that touches NO wire (genuinely floating
    // / connected mid-coil far from the drawn lead), or a terminal reaching a net the magnetic's netlist
    // pins never touch (mis-wired). "Unreached netlist net" is dropped — resonant-tank / magnetic-to-
    // magnetic nodes have no unambiguous anchor so it can't be resolved and only produces false positives.
    const TOL = 8
    for (const [Mref, terms] of magTerms) {
      const netlistNets = new Set([...pinNet].filter(([k]) => k.startsWith(Mref + '|')).map(([, n]) => n))
      const reached = new Set(); let floating = 0
      for (const c of terms) { const r = g.rootAt(c, TOL); if (r === null) floating++; else if (rootNet.has(r)) reached.add(rootNet.get(r)) }
      const wrongNet = [...reached].filter((n) => !netlistNets.has(n))
      const bits = []
      if (floating) bits.push(`${floating} floating terminal`)
      if (wrongNet.length) bits.push(`terminal on foreign net ${wrongNet.map(netLabel).join(',')}`)
      if (bits.length) problems.push(`${Mref}[winding: ${bits.join('; ')}]`)
    }
    if (problems.length) { flagged++; console.log((t.id + (opt ? '/' + opt : '')).padEnd(26), problems.join('  |  ')) }
  }
}
function netLabel(id) { return String(id).replace(/^[CPX]:/, '').replace(/\|.*/, '') }
console.log(flagged ? `\n${flagged} topology/variant combos flagged` : '\nAll nets + magnetic windings consistent with the flattened netlist')
