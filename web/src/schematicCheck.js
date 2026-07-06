// Netlist-vs-drawing connectivity + isolation checks, extracted verbatim from
// scripts/checkSchematicNets.mjs so BOTH the offline checker (hand-authored layouts) and the
// runtime CIAS generator (ciasSchematic.js) validate against the exact same rules. Given a rendered
// SVG, the recorded terminal pins, and the design's TAS, it returns a list of human-readable
// problems (empty = the drawing is electrically consistent with the flattened CIAS netlist).
//
// The rules (unchanged from the previous agent's suite):
//   A/B  every global net's unambiguous anchors (MOSFET drain/source, diode anode/cathode) sit on one
//        wire piece — or split only across ground/port reference pieces — and no wire piece carries two
//        distinct nets (short).
//   C    each magnetic winding terminal touches a wire and never reaches a net its netlist pins don't.
//   D    isolation barrier: an isolated (secondary-winding) topology must draw a DISTINCT secondary
//        return, never wire-bonded to primary earth.
import { flattenNets } from './cias.js'

export function wireGraph(svg) {
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
  const distToSeg = (p, [a, b]) => {
    const dx = b[0] - a[0], dy = b[1] - a[1], len2 = dx * dx + dy * dy
    if (len2 < 1e-9) return Math.hypot(p[0] - a[0], p[1] - a[1])
    let s = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2
    s = Math.max(0, Math.min(1, s))
    return Math.hypot(p[0] - (a[0] + s * dx), p[1] - (a[1] + s * dy))
  }
  for (const [a, b] of wires) uni(idOf(a), idOf(b))
  const ends = wires.flatMap((s) => [s[0], s[1]])
  for (const e of ends) for (const s of wires) if (onSeg(e, s)) uni(idOf(e), idOf(s[0]))
  const rootAt = (p, tol = 3) => {
    let best = null, bestD = Infinity
    for (const s of wires) { const d = distToSeg(p, s); if (d < bestD) { bestD = d; best = s } }
    if (!best || bestD > tol) return null
    const i = idOf(p); uni(i, idOf(best[0])); return find(i)
  }
  return { rootAt }
}

const netLabel = (id) => String(id).replace(/^[CPX]:/, '').replace(/\|.*/, '')

// { svg, pins:[{ref,pin,x,y}], tas } -> string[] problems. Mirrors checkSchematicNets.mjs exactly.
export function checkSchematic({ svg, pins, tas }) {
  const g = wireGraph(svg)
  const pinNet = flattenNets(tas)
  const magRefs = new Set()
  for (const st of tas.topology?.stages ?? []) for (const c of st.circuit?.components ?? []) if (c.data?.magnetic !== undefined) magRefs.add(c.name)

  const byKey = new Map(), magTerms = new Map(), refRoots = new Set()
  const primGndCoords = [], secRtnCoords = []
  for (const p of pins) {
    if (p.ref === '@gnd' || p.ref === '@sgnd' || p.ref === '@port') {
      const r = g.rootAt([p.x, p.y]); if (r !== null) refRoots.add(r)
      if (p.ref === '@gnd') primGndCoords.push([p.x, p.y])
      else if (p.ref === '@sgnd' || /rtn/i.test(p.pin)) secRtnCoords.push([p.x, p.y])
    }
    else if (magRefs.has(p.ref)) { (magTerms.get(p.ref) || magTerms.set(p.ref, []).get(p.ref)).push([p.x, p.y]) }
    else byKey.set(p.ref + '|' + p.pin, [p.x, p.y])
  }
  const rootNet = new Map()
  for (const [k, c] of byKey) { if (k.endsWith('|gate')) continue; const net = pinNet.get(k); const r = g.rootAt(c); if (net && r !== null) rootNet.set(r, net) }

  const problems = []
  const netAnchors = new Map()
  for (const [k, c] of byKey) { if (k.endsWith('|gate')) continue; const net = pinNet.get(k); if (!net) continue; (netAnchors.get(net) || netAnchors.set(net, []).get(net)).push({ k, r: g.rootAt(c) }) }
  const rootOwner = new Map()
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
  const isolated = [...pinNet.keys()].some((k) => magRefs.has(k.split('|')[0]) && /secondary/.test(k))
  if (isolated) {
    const primRoots = new Set(primGndCoords.map((c) => g.rootAt(c)).filter((r) => r !== null))
    const secRoots = new Set(secRtnCoords.map((c) => g.rootAt(c)).filter((r) => r !== null))
    if (!secRtnCoords.length) problems.push('ISOLATION: no distinct secondary return drawn (primary earth used on both sides of the barrier)')
    else if ([...primRoots].some((r) => secRoots.has(r))) problems.push('ISOLATION: primary ground bonded to secondary return by a wire (barrier shorted)')
  }
  return problems
}
