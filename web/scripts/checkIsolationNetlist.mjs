// Does the NETLIST honour the isolation barrier the schematic draws? (run: `node scripts/checkIsolationNetlist.mjs`)
//
// schematicCheck.js rule D already forbids the DRAWING from bonding primary earth to the secondary
// return — it demands a distinct secondary return glyph and no wire between the two. But rule D reads
// drawn wires only. Nothing has ever asked the same question of the CIAS the drawing is generated FROM.
//
// Between two rules, the barrier fell through the gap:
//   · rule D compares the drawing to itself, never to the netlist;
//   · rule A ("one net may not be drawn as two separate nodes") is the rule that WOULD have noticed a
//     single net drawn once at @gnd and once at @sgnd — but it exempts pieces carrying a ground/port
//     symbol, because a net legitimately splits across several ground glyphs. That exemption is exactly
//     what a fused barrier looks like, so rule A cannot fire on it either.
// The equivalence gate does not close it: it compares refdes SETS, never connectivity.
//
// So ask it directly. Build the galvanic islands of the flattened CIAS — nets joined by every component
// that actually conducts between its own terminals, with the magnetics REMOVED, since a transformer is
// precisely the part that does not conduct across. In an isolated design the primary winding's island
// and the secondary winding's island must be different islands. If they are the same one, the netlist
// says the two sides are one node while the drawing says they are two, and the ngspice deck simulates a
// converter that is not isolated at all.
//
// Excluded from the bridging set, and why each is legitimate:
//   magnetics    the barrier itself — a coupled winding transfers power without conducting
//   controller   drives gates; its pins are not a power path
//   numerical aids (__kh_numerical_aid__)  ngspice convergence parts, stripped at real fidelity and
//                absent from the BOM; a snubber the solver needs must not be able to define isolation
//   gate pins    a gate is driven, not conducted through
import init from '../../build-wasm-ng/kirchhoff.js'
import { TOPOLOGIES, VARIANTS, buildSpec } from '../src/topologies.js'
import { renderForAudit, hasCiasSchematic } from '../src/ciasSchematic.js'
import { flattenNets } from '../src/cias.js'
import { wireGraph } from '../src/schematicCheck.js'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const M = await init()
const only = process.argv[2]
const label = (n) => String(n).replace(/^[CPX]:/, '')

// The galvanic graph: nets are nodes, a conducting component is an edge between the nets on its pins.
export // Every CIAS token -> its net, including the STAGE PORTS that flattenNets drops. flattenNets returns
// component pins only, but an external port is named as a stage port, and that is precisely what the
// direction test below has to look up. Same union-find, same net identity; nothing is filtered out.
function allTokens(tas) {
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
  const out = new Map()
  for (const k of par.keys()) out.set(k, find(k))
  return out
}

function islands(tas) {
  const nets = flattenNets(tas)
  const tokenNet = allTokens(tas)
  const skip = new Set()
  const mag = new Set()
  for (const st of tas.topology?.stages ?? []) for (const c of st.circuit?.components ?? []) {
    const req = c.data?.inputs?.designRequirements ?? {}
    if (c.data?.magnetic !== undefined) { mag.add(c.name); skip.add(c.name) }
    if (c.data?.controller !== undefined) skip.add(c.name)
    if (req.name === '__kh_numerical_aid__') skip.add(c.name)
  }
  const byRef = new Map()
  for (const [k, n] of nets) {
    const [ref, pin] = k.split('|')
    ;(byRef.get(ref) ?? byRef.set(ref, []).get(ref)).push({ pin, net: n })
  }
  const par = new Map()
  const find = (x) => { if (!par.has(x)) par.set(x, x); while (par.get(x) !== x) { par.set(x, par.get(par.get(x))); x = par.get(x) } return x }
  const edges = new Map()          // "netA netB" -> ref, kept so a breach can name the path
  const join = (a, b, ref) => {
    par.set(find(a), find(b))
    if (!edges.has(a + ' ' + b)) edges.set(a + ' ' + b, ref)
    if (!edges.has(b + ' ' + a)) edges.set(b + ' ' + a, ref)
  }
  // A magnetic is not automatically a barrier. One with a SINGLE winding is a plain inductor and it
  // CONDUCTS between its terminals — excluding those was an error that made a resonant secondary (one
  // reached only through Lr2) look like an island of its own, and so turned a fused barrier into a pass.
  // Even a real transformer refuses to conduct only BETWEEN windings: each winding is a coil whose own
  // two ends are joined. So join within each winding group, and never across groups.
  const coil = (pin) => pin.replace(/_(start|end)$/, '')
  for (const [ref, ps] of byRef) {
    if (mag.has(ref)) {
      const groups = new Map()
      for (const p of ps) (groups.get(coil(p.pin)) ?? groups.set(coil(p.pin), []).get(coil(p.pin))).push(p.net)
      for (const [, ns] of groups) for (let i = 1; i < ns.length; i++) join(ns[0], ns[i], ref)
      continue
    }
    if (skip.has(ref)) continue
    const power = [...new Set(ps.filter((p) => p.pin !== 'gate').map((p) => p.net))]
    for (let i = 1; i < power.length; i++) join(power[0], power[i], ref)
  }
  // Tokens, not just component pins: an external port is named as a STAGE port, and the island a
  // direction sits on is exactly what this gate asks about.
  const tokenRoot = (tok) => (tokenNet.has(tok) ? find(tokenNet.get(tok)) : null)
  return { nets, mag, find, edges, tokenRoot }
}

// The evidence: the actual chain of parts that carries the primary return across to the secondary.
// A breach reported as "these two nets are the same island" is a claim; the chain is a proof, and it is
// what someone fixing the engine needs to see.
function bridgePath(from, to, edges) {
  const adj = new Map()
  for (const [k, ref] of edges) {
    const [a, b] = k.split(' ')
    ;(adj.get(a) ?? adj.set(a, []).get(a)).push({ b, ref })
  }
  const prev = new Map([[from, null]])
  const q = [from]
  while (q.length) {
    const cur = q.shift()
    if (cur === to) break
    for (const { b, ref } of adj.get(cur) ?? []) if (!prev.has(b)) { prev.set(b, { from: cur, ref }); q.push(b) }
  }
  if (!prev.has(to)) return null
  const out = []
  for (let n = to; prev.get(n); n = prev.get(n).from) out.unshift(`${label(prev.get(n).from)} —${prev.get(n).ref}→ ${label(n)}`)
  return out
}

// Importable so the rule can be exercised against a netlist that KEEPS the barrier — a gate that only
// ever fires proves nothing about what it is measuring.
const isMain = process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])

let isolated = 0, breached = 0, checked = 0
if (isMain) for (const t of TOPOLOGIES) {
  const v = VARIANTS[t.id]
  for (const opt of (v ? v.options.map((o) => o.id) : [null])) {
    const key = `${t.id}${opt ? '/' + opt : ''}`
    if (only && only !== key && only !== t.id) continue
    const spec = buildSpec({ ...t.preset, variant: opt ?? 'standard' }, t.id)
    if (opt && v) spec.config = { ...(spec.config ?? {}), [v.key]: opt }
    const out = M.design_tas_full(t.id, JSON.stringify(spec))
    // A design that throws is not a combo this gate may pass over in silence.
    if (out.startsWith('Exception')) throw new Error(`${key}: design failed: ${out.slice(0, 200)}`)
    const tas = JSON.parse(out).tas
    checked++
    const { nets, find, tokenRoot } = islands(tas)
    // COMPARE THE TWO VIEWS DIRECTLY. Earlier forms of this rule tried to infer the barrier from the
    // netlist alone — first from winding NAMES (wrong: push_pull calls a primary half 'secondary1'),
    // then from a multi-winding magnetic spanning two islands (wrong: weinberg's L1 is a coupled
    // inductor that is meant to conduct across), then from input rail vs output rail (wrong: the
    // flybuck's declared output is its PRIMARY buck rail and its isolated secondary is internal).
    // The claim being checked was never a property of the netlist on its own — it is the DRAWING's:
    // it prints two different reference symbols. So resolve those two symbols to their nets and ask
    // whether the netlist agrees they are two nodes.
    if (!hasCiasSchematic(t.id)) continue
    const { svg, pins } = renderForAudit(t.id, tas, opt ?? 'standard')
    const earthPins = pins.filter((p) => p.ref === '@gnd')
    const rtnPins = pins.filter((p) => p.ref === '@sgnd' || (p.ref === '@port' && /rtn/i.test(p.pin)))
    if (!rtnPins.length) continue                 // the drawing asserts no barrier; nothing to check
    isolated++
    // A reference glyph carries no net of its own: it names the piece of wire it stands on. Resolve it
    // the way rule D does — through the drawn wire graph — then take the net of any component terminal
    // sitting on that same piece.
    const g = wireGraph(svg)
    const pinNet = flattenNets(tas)
    const netAtRoot = new Map()
    for (const p of pins) {
      if (p.ref.startsWith('@') || p.pin === 'gate') continue
      const net = pinNet.get(`${p.ref}|${p.pin}`) ?? pinNet.get(`${p.ref}|${p.pin === 'p0' ? '1' : p.pin === 'p1' ? '2' : p.pin}`)
      const r = g.rootAt([p.x, p.y], 4)
      if (net && r !== null && !netAtRoot.has(r)) netAtRoot.set(r, net)
    }
    const islandsOf = (ps) => {
      const out = new Set()
      for (const p of ps) {
        const r = g.rootAt([p.x, p.y], 4)
        if (r !== null && netAtRoot.has(r)) out.add(find(netAtRoot.get(r)))
      }
      return out
    }
    const eIsl = islandsOf(earthPins), rIsl = islandsOf(rtnPins)
    // Where a reference glyph stands on a piece whose only terminal is a WINDING, the drawing cannot be
    // resolved that way: a magnetic's drawn terminal names (p0/s0/sct) are not its netlist winding names.
    // Fall back to the CIAS's own reference PORTS — the declared primary earth and isolated return — which
    // is the same question asked of the other view rather than a weaker version of it.
    if (!eIsl.size || !rIsl.size) {
      const refIsl = new Map()
      for (const ic of tas.topology?.interStageConnections ?? []) {
        if (ic.kind !== 'externalPort' || !/gnd|rtn/i.test(ic.name ?? '')) continue
        for (const ep of ic.endpoints ?? []) {
          const r = tokenRoot(ep.component !== undefined ? 'C:' + ep.component + '|' + ep.pin
                                                         : 'P:' + ep.stage + '::' + ep.port)
          if (r !== null) (refIsl.get(ic.name) ?? refIsl.set(ic.name, new Set()).get(ic.name)).add(r)
        }
      }
      const distinct = new Set([...refIsl.values()].flatMap((v) => [...v]))
      if (refIsl.size >= 2 && distinct.size >= 2) {
        console.log(`✓ ${key.padEnd(26)} netlist keeps the barrier (via its declared reference ports ${[...refIsl.keys()].join('/')})`)
        continue
      }
      if (refIsl.size) {
        breached++
        console.log(`\n✗ ${key}`)
        console.log(`    the drawing prints ${earthPins.length} primary earth and ${rtnPins.length} isolated-return symbol(s),`)
        console.log(`    but the CIAS declares ${refIsl.size} reference port(s) resolving to ${distinct.size} island(s)`)
        continue
      }
    }
    // Refuse to pass on evidence that was never obtained: an unresolvable reference is not a clean bill.
    if (!eIsl.size || !rIsl.size) {
      breached++
      console.log(`\n✗ ${key}`)
      console.log(`    could not resolve the drawn ${!eIsl.size ? 'primary earth' : 'isolated return'} to a net` +
                  ` (${earthPins.length} earth / ${rtnPins.length} return terminals drawn) — not checked, not passed`)
      continue
    }
    const shared = [...rIsl].filter((r) => eIsl.has(r))
    if (!shared.length) { console.log(`✓ ${key.padEnd(26)} netlist keeps the barrier`); continue }
    breached++
    console.log(`\n✗ ${key}`)
    console.log(`    the drawing prints ${earthPins.length} primary earth and ${rtnPins.length} isolated-return` +
                ` symbol(s), but the CIAS puts them on ONE galvanic island (${String(shared[0]).replace(/^[CPX]:/, '')})`)
    console.log(`    rule D requires the two kept apart in the drawing; the netlist it is drawn from does not keep them apart`)
  }
}

// Refuse to report success over nothing — the failure mode this whole suite keeps rediscovering.
if (isMain) {
if (!checked) throw new Error('checkIsolationNetlist examined 0 designs')
if (!isolated) throw new Error(`checkIsolationNetlist examined ${checked} designs and found no isolated one — it can never have tested anything`)
console.log(breached
  ? `\n${breached} of ${isolated} isolated combos: the netlist bonds the two sides of the barrier the schematic draws (${checked} designs examined)`
  : `\nAll ${isolated} isolated combos: the netlist keeps primary and secondary galvanically separate (${checked} designs examined)`)
process.exit(breached ? 1 : 0)   // a gate that cannot fail is not a gate
}
