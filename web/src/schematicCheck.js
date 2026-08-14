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
//   F    a two-terminal passive (R/C) straddles its OWN two nets — never drawn shorted out, never with
//        an end on a foreign node (the netlist calls their pins 1/2, so no other rule anchored them).
//   E    no wire ends in mid-air: every polyline endpoint continues onto another wire or lands on a
//        component terminal/footprint. A/B only look at anchor pins, so a rail overshooting its last
//        leg or a part wired 5 px short of its rail passes every net rule while LOOKING disconnected.
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
  return { rootAt, wires }
}

// E. No wire may END in mid-air. Every polyline endpoint must either continue onto another wire piece
// (a T/corner/crossing) or land on a component terminal registered by collectPins (regPin). An endpoint
// that satisfies neither is a drawing artifact — a rail that overshoots the last leg, a stub left behind
// when a part moved, a bleeder wired 5 px short of its return rail — and reads to the user as a broken
// connection even when the netlist check above passes (it only looks at anchors, never at wire ends).
export function danglingEnds(svg, pins) {
  const { wires } = wireGraph(svg)
  const TOL = 4
  // A wire may also stop ON a component's own lead art (several symbols draw a lead the wire overlaps
  // rather than butts against — outTail's Cout stops 10 px inside the cap lead). Landing anywhere in the
  // part's hitbox reads as attached, so treat the footprint as connected territory.
  const boxes = [...svg.matchAll(/<rect class="sch-hitbox" x="([-\d.]+)" y="([-\d.]+)" width="([\d.]+)" height="([\d.]+)"/g)]
    .map((m) => m.slice(1).map(Number))
  const inBox = (p) => boxes.some(([x, y, w, h]) =>
    p[0] >= x - TOL && p[0] <= x + w + TOL && p[1] >= y - TOL && p[1] <= y + h + TOL)
  const near = (p, q) => Math.abs(p[0] - q[0]) <= TOL && Math.abs(p[1] - q[1]) <= TOL
  const onSeg = (p, [a, b], i, self) => {
    if (i === self) return false
    const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy)
    if (len < 1e-6) return near(p, a)
    if (Math.abs(dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / len > TOL) return false
    const proj = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len
    return proj >= -TOL && proj <= len + TOL
  }
  const out = []
  wires.forEach((seg, i) => {
    for (const e of seg) {
      if (wires.some((s, j) => onSeg(e, s, j, i))) continue
      if (pins.some((p) => near(e, [p.x, p.y]))) continue
      if (inBox(e)) continue
      out.push(`wire end (${e[0]},${e[1]}) touches nothing`)
    }
  })
  return [...new Set(out)]
}

const netLabel = (id) => String(id).replace(/^[CPX]:/, '').replace(/\|.*/, '')

// G. Nets that NO anchor identifies — the node between two passives (an RC snubber's midpoint, a
// resonant tank's Cr–Lr join). Rules A/B only see nets with a MOSFET/diode/port terminal on them, and
// rule F only rejects a passive terminal that lands on an *identified* net, so everything about a
// passive-only node was taken on trust: two series parts drawn 5 px apart read as connected and passed,
// and two different passive-only nodes bonded together passed.
//
// Identify them instead: seed every drawn node that an anchor names, then propagate through each
// two-terminal passive — if one end sits on net X and the netlist says the part spans {X, Y}, its other
// end IS net Y. Then apply rule A's own test to the result: a net may not appear on two distinct drawn
// nodes (unless the pieces are joined by ground/port symbols, the split rule A already allows), and a
// node may not carry two nets.
function passiveOnlyNets({ g, pins, pinNet, magRefs, refRoots, twoTerm }) {
  const problems = []
  // A drawn "node" is a wire piece, or — for leads that butt together with no wire between them, which
  // is how a series pair is drawn — the terminal point itself, shared by anything within TOUCH px.
  const TOUCH = 4
  const synth = []
  const nodeOf = (c) => {
    const r = g.rootAt(c, TOUCH)
    if (r !== null) return 'W' + r
    let i = synth.findIndex((s) => Math.abs(s[0] - c[0]) <= TOUCH && Math.abs(s[1] - c[1]) <= TOUCH)
    if (i < 0) { i = synth.length; synth.push(c) }
    return 'T' + i
  }
  // A piece carrying a ground/port symbol may legitimately be one of several drawn for the same net
  // (rule A's own exemption), so it is never evidence of a conflict.
  const isRef = (node) => String(node).startsWith('W') && refRoots.has(Number(String(node).slice(1)))
  const label = new Map()
  const setLabel = (node, net, who) => {
    const had = label.get(node)
    if (had === undefined) { label.set(node, net); return true }
    if (had !== net && !isRef(node)) problems.push(`${who}: drawn node carries both ${netLabel(had)} and ${netLabel(net)}`)
    return false
  }
  for (const p of pins) {
    if (p.ref.startsWith('@') || magRefs.has(p.ref) || p.pin === 'gate') continue
    const net = pinNet.get(`${p.ref}|${p.pin}`)
    if (net) setLabel(nodeOf([p.x, p.y]), net, p.ref)
  }
  // propagate through the passives until nothing new is learned
  const parts = [...twoTerm].map(([ref, terms]) => ({ ref, terms, own: [pinNet.get(`${ref}|1`), pinNet.get(`${ref}|2`)] }))
    .filter((p) => p.terms.length === 2 && p.own[0] && p.own[1] && p.own[0] !== p.own[1])
  for (let pass = 0; pass < parts.length + 1; pass++) {
    let learned = false
    for (const { ref, terms, own } of parts) {
      const [n0, n1] = terms.map(nodeOf)
      const [l0, l1] = [label.get(n0), label.get(n1)]
      // A known end that carries a net this part does not belong to is the foreign-net case rule F can
      // only catch on an ANCHORED piece. Propagation reaches the unanchored ones, so say so here rather
      // than blindly assign the other net and lose the evidence.
      for (const [n, l] of [[n0, l0], [n1, l1]])
        if (l !== undefined && !own.includes(l)) { label.set(n, l); problems.push(`${ref}: terminal on foreign net ${netLabel(l)}`) }
      if (l0 !== undefined && own.includes(l0) && l1 === undefined) learned = setLabel(n1, own[0] === l0 ? own[1] : own[0], ref) || learned
      else if (l1 !== undefined && own.includes(l1) && l0 === undefined) learned = setLabel(n0, own[0] === l1 ? own[1] : own[0], ref) || learned
    }
    if (!learned) break
  }
  // The other direction, and the one that needs no labels at all: two parts that SHARE a net must share
  // a drawn node. Nothing else asks this of a passive-only net, so a series pair drawn 5 px apart — the
  // exact "wired short of its rail" defect the dangling-wire rule exists to catch, but between two
  // symbol leads instead of a wire end — read as connected and passed every check.
  const nodesOf = new Map()
  for (const p of pins) {
    if (p.ref.startsWith('@') || p.pin === 'gate') continue
    ;(nodesOf.get(p.ref) || nodesOf.set(p.ref, new Set()).get(p.ref)).add(nodeOf([p.x, p.y]))
  }
  const refsOfNet = new Map()
  for (const [k, net] of pinNet) {
    const [ref, pin] = k.split('|')
    if (pin === 'gate' || !nodesOf.has(ref)) continue          // control blocks declare pins but draw no terminals
    ;(refsOfNet.get(net) || refsOfNet.set(net, new Set()).get(net)).add(ref)
  }
  for (const [net, refs] of refsOfNet) {
    const list = [...refs]
    for (let i = 0; i < list.length; i++) for (let j = i + 1; j < list.length; j++) {
      const [a, b] = [nodesOf.get(list[i]), nodesOf.get(list[j])]
      if ([...a].some((n) => b.has(n))) continue
      // a net may legally be drawn as several pieces joined by ground/port symbols (rule A allows it)
      if ([...a].some(isRef) && [...b].some(isRef)) continue
      problems.push(`${netLabel(net)}: ${list[i]} and ${list[j]} share this net but no drawn node`)
    }
  }
  // rule A's test, now over the identified nodes: one net, one node (bar reference-joined pieces)
  const nodesOfNet = new Map()
  for (const [node, net] of label) {
    if (String(node).startsWith('W') && refRoots.has(Number(String(node).slice(1)))) continue
    ;(nodesOfNet.get(net) || nodesOfNet.set(net, new Set()).get(net)).add(node)
  }
  for (const [net, nodes] of nodesOfNet)
    if (nodes.size > 1) problems.push(`${netLabel(net)}: drawn as ${nodes.size} separate nodes (parts that share it are not connected)`)
  return [...new Set(problems)]
}

// { svg, pins:[{ref,pin,x,y}], tas } -> string[] problems. Mirrors checkSchematicNets.mjs exactly.
export function checkSchematic({ svg, pins, tas }) {
  const g = wireGraph(svg)
  const pinNet = flattenNets(tas)
  // FLOOR — the checker must refuse to certify what it cannot see. Every rule below is of the form "for
  // each drawn anchor, check X", so an EMPTY drawing satisfies all of them: fed a blank <svg> and an
  // empty pin list, this function returned [] for buck — a clean bill of health for a blank page. That
  // matters beyond the offline gate, because buildCias runs this at RENDER time to decide whether the
  // app may draw at all. So first establish that the drawing actually contains the netlist: every
  // component the netlist connects must have registered terminals, and there must be wires.
  const floor = []
  // Sourced from the DRAWING, not the netlist: the netlist also carries things the drawing deliberately
  // does not anchor — a FET's body diode is drawn as an annotation, and the ngspice convergence aids are
  // not drawn at all — so requiring terminals for every connected component flags six healthy fsbb parts.
  // What must hold is that everything the drawing CLAIMS to show is anchored, and that there is a drawing.
  const wires = svg.match(/<path class="sch-wire"/g)?.length ?? 0
  const wired = new Set(pins.map((p) => p.ref))
  if (!wires) floor.push('the drawing contains no wires at all')
  // Each hot group with its body, so a part can be judged by HOW it is connected. A control block —
  // the controller IC, and pfc/vienna's analog blocks (integrators, multipliers, comparators) — reaches
  // the design through dashed .sch-ctl stubs and net-label flags rather than wires, which is exactly why
  // the net rules exclude gate nets. Those anchor nothing by design; every other drawn part must anchor.
  // Split on the group openers rather than matching to </g>: several symbols nest a <g transform> of
  // their own (a flipped FET), and the opener carries tabindex/role/aria-label after data-ref.
  const groups = svg.split('<g class="sch-hot').slice(1).map((chunk) => ({
    ann: /^[^">]*\bsch-ann\b/.test(chunk),
    ref: chunk.match(/data-ref="([^"]+)"/)?.[1] ?? null,
    body: chunk,
  })).filter((x) => x.ref)
  if (!groups.length) floor.push('the drawing contains no components at all')
  for (const { ann, ref, body } of groups)
    if (!ann && !wired.has(ref) && !/class="sch-ctl"/.test(body))
      floor.push(`${ref} is drawn but registered no terminals — nothing about its connections can be verified`)
  if (floor.length) return floor
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
    // The short check must cover REFERENCE pieces too. `refRoots` exists so rule A can accept one net
    // split across several ground/port symbols — but a piece carrying a ground symbol still may not
    // carry two DIFFERENT nets. Exempting it hid a real short: weinberg's S1 source-to-ground wire ran
    // straight down through S2, bonding S2's drain and T1's primary-B bottom onto the ground rail.
    for (const a of ancs) if (a.r !== null) {
      if (rootOwner.has(a.r) && rootOwner.get(a.r) !== net) problems.push(`SHORT: ${netLabel(net)} ↔ ${netLabel(rootOwner.get(a.r))}`)
      rootOwner.set(a.r, net)
    }
  }
  // 4 px, not 8: at 8 a winding terminal counted as reaching a wire that stopped 5 px short of it, which
  // is exactly how acf's secondary sat unconnected to its rectifier while every check passed. The whole
  // file now uses one tolerance for "this terminal touches that wire".
  const TOL = 4
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
  // F. Two-terminal passives (R/C, drawn terminals recorded as p0/p1) must straddle their OWN two nets.
  // The netlist names their pins '1'/'2', so nothing above ever anchored them: a resistor or capacitor
  // drawn shorted out, or with one end landing on a foreign node, passed every rule. Orientation is not
  // asserted (which drawn end is pin 1 is a layout choice) — only that the two ends are DIFFERENT pieces
  // and that neither sits on a net the part does not belong to. This is what caught the flybuck's Vpri
  // rail being drawn straight through T1's secondary terminal.
  const twoTerm = new Map()
  for (const p of pins) if (p.pin === 'p0' || p.pin === 'p1') {
    if (magRefs.has(p.ref) || p.ref.startsWith('@')) continue
    ;(twoTerm.get(p.ref) || twoTerm.set(p.ref, []).get(p.ref)).push([p.x, p.y])
  }
  for (const [ref, terms] of twoTerm) {
    const own = new Set([pinNet.get(`${ref}|1`), pinNet.get(`${ref}|2`)].filter(Boolean))
    if (own.size !== 2 || terms.length !== 2) continue
    const [r0, r1] = terms.map((c) => g.rootAt(c, 4))
    if (r0 !== null && r0 === r1) { problems.push(`${ref}: both terminals on one wire piece (drawn shorted out)`); continue }
    for (const r of [r0, r1]) {
      const n = r === null ? null : rootNet.get(r)
      if (n && !own.has(n)) problems.push(`${ref}: terminal on foreign net ${netLabel(n)}`)
    }
  }
  problems.push(...passiveOnlyNets({ g, pins, pinNet, magRefs, refRoots, twoTerm }))

  problems.push(...danglingEnds(svg, pins))
  return problems
}
