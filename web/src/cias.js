// Shared CIAS helpers: walk the TAS's inline CIAS bricks (stage.circuit) — the SAME structure the
// ngspice deck is generated from (CiasToNgspiceConverter) — so every consumer (falstad export,
// net checkers, future CIAS-driven schematic) reads one source of truth.

// Flatten the hierarchical TAS (per-stage circuits linked by inter-stage connections) into global
// nets. Returns pinNet: "component|pin" -> netId (string union-find root). Moved verbatim from
// scripts/checkSchematicNets.mjs so the offline checker and the runtime emitters share one impl.
// Every CIAS token -> its net: component pins AND the stage ports that name the external connections.
// flattenNets is the component-pin view of this; referenceNets needs the stage ports too.
export function flattenTokens(tas) {
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

export function flattenNets(tas) {
  const pinNet = new Map()
  for (const [k, net] of flattenTokens(tas)) if (k.startsWith('C:')) pinNet.set(k.slice(2), net)
  return pinNet
}

// The nets a SIMULATOR ties to its single ground. An isolated design declares TWO returns — primary
// earth and the isolated secondary return — and since ABT #778 they are genuinely different CIAS nodes,
// which is the whole point: the netlist now carries the barrier the schematic draws. But a simulator has
// only ONE ground. ngspice collapses every externalPort named *gnd* to node 0 (NgspiceNodes.hpp), and
// CircuitJS1 has a single global ground symbol. So a visual layout that puts both returns on one ground
// rail is not drawing a short — it is drawing the reference the simulation needs.
//
// That aliasing is DECLARED here rather than waived inside each exporter's short check, so there is one
// place that says which nets are references and why. A short between two non-reference nets still fails.
export function referenceNets(tas) {
  const tokens = flattenTokens(tas)
  const out = new Set()
  for (const ic of tas?.topology?.interStageConnections ?? []) {
    if (ic.kind !== 'externalPort' || !/gnd|rtn/i.test(ic.name ?? '')) continue
    for (const ep of ic.endpoints ?? []) {
      const tok = ep.component !== undefined ? 'C:' + ep.component + '|' + ep.pin
        : ep.stage !== undefined ? 'P:' + ep.stage + '::' + ep.port : null
      if (tok && tokens.has(tok)) out.add(tokens.get(tok))
    }
  }
  return out
}

// Every power-path component of the TAS: [{ ref, stage, data, req }]. Skips what the BOM skips
// (body diodes, ABT #96 numerical aids) plus controller parts — none of those carry power.
export function ciasComponents(tas) {
  const out = []
  for (const stage of tas?.topology?.stages ?? []) {
    for (const comp of stage.circuit?.components ?? []) {
      const data = comp.data ?? {}
      const req = data.inputs?.designRequirements ?? {}
      if (req.role === 'bodyDiode') continue
      if (req.name === '__kh_numerical_aid__') continue
      if (data.controller !== undefined) continue
      out.push({ ref: comp.name, stage: stage.name, data, req })
    }
  }
  return out
}

// Collapse a dimensionWithTolerance ({nominal,minimum,maximum} | bare number) to a scalar with the
// canonical resolve_dimensional_values semantics (PEAS/src/Dimension.hpp): nominal -> (min+max)/2
// -> max -> min; THROWS when none is present (no silent 0).
export function resolveDim(x, what = 'value') {
  if (typeof x === 'number') return x
  if (x && typeof x === 'object') {
    if (typeof x.nominal === 'number') return x.nominal
    if (typeof x.minimum === 'number' && typeof x.maximum === 'number') return (x.minimum + x.maximum) / 2
    if (typeof x.maximum === 'number') return x.maximum
    if (typeof x.minimum === 'number') return x.minimum
  }
  throw new Error(`cannot resolve dimensional value for ${what}: no nominal/minimum/maximum present`)
}
