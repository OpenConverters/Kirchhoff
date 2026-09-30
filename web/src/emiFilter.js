// EMI line filter as a CIAS brick, and the ngspice AC decks that measure its insertion loss.
//
// Semantics ported from Hertz (web/src/ciasFilter.js, cpp/include/hertz/SpiceDeck.hpp) — not imported
// from it: Kirchhoff owns this copy because its schematic generator draws it.
//
// TOPOLOGY (Würth ANP015 Fig. 13), read from the mains/LISN side, per stage: an X capacitor across the
// lines on the stage's INPUT net, then the common-mode choke, then a Y capacitor from each line to PE on
// the stage's OUTPUT net. Stage s+1's input net is stage s's output net. The two capacitor kinds sit on
// opposite sides of the choke because the impedance-mismatch rule points opposite ways per mode: a
// converter is a LOW impedance differentially (the choke's leakage faces it, X faces the mains) and a
// HIGH impedance in common mode (the Y capacitors face it).
//
//   refs   CMC{s}, C_X{s}, C_YL{s}, C_YN{s}             (s = 1..stages)
//   ports  line_in, neutral_in   (LISN / mains side)
//          line_out, neutral_out (equipment side)
//          pe                    (Y-capacitor return)
//   pins   CMC: P1 (line in), P2 (line out), S1 (neutral in), S2 (neutral out)
//          P1 and S1 are the DOTTED ends, so common-mode current (line and neutral flowing the same way)
//          enters both windings at the dot and sees the magnetising inductance, while differential-mode
//          current sees only the leakage.
//          capacitors: '1', '2' (X: 1 on line, 2 on neutral; Y: 1 on the line, 2 on PE)
//
// Topology 'dc' is the same two-rail brick for a DC supply (+ / return instead of L / N; the Y return is
// the chassis). The net names are identical; only the brick name and the port descriptions differ.
//
// PART BINDING. A bound slot is the URI 'TAS/data/<catalogue>.ndjson?partNumber=<mpn>'; an unbound one is
// 'TAS/data/<catalogue>.ndjson?placeholder=<ref>' — the CIAS schema's own form for a deliberately
// unresolvable pre-sourcing slot — so the brick is schema-valid from the moment it exists, before any part
// has been chosen. There is never an intermediate invalid object.

export const EMI_FILTER_TOPOLOGIES = Object.freeze(['mains', 'dc'])
export const EMI_FILTER_MAX_STAGES = 4

const CATALOGUE = { cmc: 'TAS/data/magnetics.ndjson', cx: 'TAS/data/capacitors.ndjson', cy: 'TAS/data/capacitors.ndjson' }
// The URI value grammar (CIAS.json component.data): one key=value, the value free of & = ? and whitespace
// is not excluded by the schema, but a part number with a blank in it is a scrape defect, so it is refused.
const URI_VALUE = /^[^&=?\s]+$/
const URI_RE = /^([^\s?]+\.ndjson)\?([A-Za-z][A-Za-z0-9]*)=([^&=?]+)$/

function checkStages(stages) {
  if (!Number.isInteger(stages) || stages < 1 || stages > EMI_FILTER_MAX_STAGES)
    throw new Error(`EMI filter: stages must be an integer 1..${EMI_FILTER_MAX_STAGES}, got ${JSON.stringify(stages)}`)
}

function checkTopology(topology) {
  if (!EMI_FILTER_TOPOLOGIES.includes(topology))
    throw new Error(`EMI filter: topology must be one of ${EMI_FILTER_TOPOLOGIES.join(', ')}, got ${JSON.stringify(topology)}`)
}

// Every component of an n-stage filter: [{ ref, kind: 'cmc'|'cx'|'cy', stage }], in drawing order.
export function emiFilterComponents(stages) {
  checkStages(stages)
  const out = []
  for (let s = 1; s <= stages; s++) {
    out.push({ ref: `C_X${s}`, kind: 'cx', stage: s })
    out.push({ ref: `CMC${s}`, kind: 'cmc', stage: s })
    out.push({ ref: `C_YL${s}`, kind: 'cy', stage: s })
    out.push({ ref: `C_YN${s}`, kind: 'cy', stage: s })
  }
  return out
}

// The brick's nets (CIAS connections). Net line_{k} / neutral_{k} is the rail between stage k and k+1:
// line_0 is the LISN-side input, line_{stages} the equipment-side output.
export function emiFilterNets(stages) {
  checkStages(stages)
  const nets = []
  const rails = [['line', 'P1', 'P2', 'C_YL', '1'], ['neutral', 'S1', 'S2', 'C_YN', '2']]
  for (let k = 0; k <= stages; k++) {
    for (const [rail, pinIn, pinOut, yRef, xPin] of rails) {
      const eps = []
      if (k === 0) eps.push({ port: `${rail}_in` })
      if (k >= 1) {
        eps.push({ component: `CMC${k}`, pin: pinOut })
        eps.push({ component: `${yRef}${k}`, pin: '1' })
      }
      if (k < stages) {
        eps.push({ component: `C_X${k + 1}`, pin: xPin })
        eps.push({ component: `CMC${k + 1}`, pin: pinIn })
      }
      if (k === stages) eps.push({ port: `${rail}_out` })
      nets.push({ name: `${rail}_${k}`, endpoints: eps })
    }
  }
  const pe = { name: 'pe', endpoints: [{ port: 'pe' }] }
  for (let s = 1; s <= stages; s++) pe.endpoints.push({ component: `C_YL${s}`, pin: '2' }, { component: `C_YN${s}`, pin: '2' })
  nets.push(pe)
  return nets
}

function brickPorts(topology) {
  const dc = topology === 'dc'
  return [
    { name: 'line_in', description: dc ? 'positive supply, LISN side' : 'mains line, LISN side' },
    { name: 'neutral_in', description: dc ? 'supply return, LISN side' : 'mains neutral, LISN side' },
    { name: 'line_out', description: dc ? 'positive supply, equipment side' : 'line, equipment side' },
    { name: 'neutral_out', description: dc ? 'supply return, equipment side' : 'neutral, equipment side' },
    { name: 'pe', description: dc ? 'chassis (Y-capacitor return)' : 'protective earth (Y-capacitor return)' },
  ]
}

const brickName = (topology, stages) =>
  topology === 'dc' ? `dc-supply-filter-${stages}stage` : `single-phase-line-filter-${stages}stage`

// buildEmiFilterCias({ stages, topology, bindings }) -> a schema-valid CIAS brick.
//   stages    1..4
//   topology  'mains' | 'dc'
//   bindings  { [ref]: { mpn } } — the chosen part per slot. A slot absent from bindings is emitted as a
//             placeholder URI. A key that names no slot of this filter, or a binding without an mpn,
//             throws (a typo must not silently leave a part unbound).
export function buildEmiFilterCias({ stages, topology, bindings } = {}) {
  checkStages(stages)
  checkTopology(topology)
  if (bindings === null || typeof bindings !== 'object' || Array.isArray(bindings))
    throw new Error('EMI filter: bindings must be an object { [ref]: { mpn } } (use {} when nothing is bound yet)')
  const comps = emiFilterComponents(stages)
  const known = new Set(comps.map((c) => c.ref))
  for (const [ref, b] of Object.entries(bindings)) {
    if (!known.has(ref)) throw new Error(`EMI filter: binding for '${ref}', which is not a slot of a ${stages}-stage filter (slots: ${[...known].join(', ')})`)
    if (b === null || typeof b !== 'object' || typeof b.mpn !== 'string' || !b.mpn)
      throw new Error(`EMI filter: binding for '${ref}' has no mpn`)
    if (!URI_VALUE.test(b.mpn))
      throw new Error(`EMI filter: part number '${b.mpn}' for '${ref}' cannot be written as a CIAS part URI (contains &, =, ? or whitespace)`)
  }
  return {
    name: brickName(topology, stages),
    ports: brickPorts(topology),
    components: comps.map((c) => ({
      name: c.ref,
      data: bindings[c.ref]
        ? `${CATALOGUE[c.kind]}?partNumber=${bindings[c.ref].mpn}`
        : `${CATALOGUE[c.kind]}?placeholder=${c.ref}`,
    })),
    connections: emiFilterNets(stages),
  }
}

const canonicalNets = (connections) => connections
  .map((n) => n.endpoints.map((e) => (e.port !== undefined ? `port:${e.port}` : `${e.component}|${e.pin}`)).sort().join(','))
  .sort().join(';')

// Read a brick back: which EMI filter it is, and what each slot is bound to. THROWS unless the brick is
// exactly the n-stage filter this module builds (same refs, same catalogues, same nets, same ports) —
// the drawing and the deck both rely on that shape, so anything else is refused rather than guessed at.
// -> { stages, topology, slots: [{ ref, kind, stage, mpn|null }] }
export function parseEmiFilterCias(cias) {
  if (!cias || typeof cias !== 'object') throw new Error('EMI filter: no CIAS brick given')
  const m = /^(single-phase-line|dc-supply)-filter-(\d+)stage$/.exec(cias.name ?? '')
  if (!m) throw new Error(`EMI filter: '${cias.name}' is not an EMI line-filter brick name`)
  const topology = m[1] === 'dc-supply' ? 'dc' : 'mains'
  const stages = Number(m[2])
  checkStages(stages)
  const expected = emiFilterComponents(stages)
  const byRef = new Map((cias.components ?? []).map((c) => [c.name, c]))
  if (byRef.size !== (cias.components ?? []).length) throw new Error('EMI filter: duplicate component names in the brick')
  const extra = [...byRef.keys()].filter((r) => !expected.some((e) => e.ref === r))
  const missing = expected.filter((e) => !byRef.has(e.ref)).map((e) => e.ref)
  if (extra.length || missing.length)
    throw new Error(`EMI filter: brick components differ from a ${stages}-stage filter (missing: ${missing.join(', ') || '—'}; extra: ${extra.join(', ') || '—'})`)
  const slots = expected.map((e) => {
    const data = byRef.get(e.ref).data
    if (typeof data !== 'string') throw new Error(`EMI filter: component '${e.ref}' is inline PEAS; this filter binds parts by URI only`)
    const u = URI_RE.exec(data)
    if (!u) throw new Error(`EMI filter: component '${e.ref}' has a malformed part URI '${data}'`)
    if (u[1] !== CATALOGUE[e.kind]) throw new Error(`EMI filter: component '${e.ref}' points into '${u[1]}', expected '${CATALOGUE[e.kind]}'`)
    if (u[2] === 'partNumber') return { ...e, mpn: u[3] }
    if (u[2] === 'placeholder') return { ...e, mpn: null }
    throw new Error(`EMI filter: component '${e.ref}' URI key '${u[2]}' is neither partNumber nor placeholder`)
  })
  const portNames = (cias.ports ?? []).map((p) => p.name).join(',')
  const wantPorts = brickPorts(topology).map((p) => p.name).join(',')
  if (portNames !== wantPorts) throw new Error(`EMI filter: brick ports [${portNames}] differ from [${wantPorts}]`)
  if (canonicalNets(cias.connections ?? []) !== canonicalNets(emiFilterNets(stages)))
    throw new Error(`EMI filter: brick connections are not the ${stages}-stage ANP015 filter netlist`)
  return { stages, topology, slots }
}

// BOM rows for KhSchematic / a host's part list: [{ ref, kind, role, stage, value, partLabel, bound }].
//   kind       'CommonModeChoke' | 'Capacitor'       (what a `selectable` predicate usually filters on)
//   role       'cmc' | 'cx' | 'cy'
//   value      the display value for the second label line, from `values[ref]` when the host passes one
//              (e.g. '1 mH', '470 nF'); omitted otherwise — nothing is invented
//   partLabel  the bound part number, drawn as a third label line; absent while the slot is a placeholder
export function emiFilterBom(cias, { values = {} } = {}) {
  const { slots } = parseEmiFilterCias(cias)
  for (const ref of Object.keys(values)) if (!slots.some((s) => s.ref === ref)) throw new Error(`EMI filter: value given for unknown slot '${ref}'`)
  return slots.map((s) => ({
    ref: s.ref,
    kind: s.kind === 'cmc' ? 'CommonModeChoke' : 'Capacitor',
    role: s.kind,
    stage: s.stage,
    ...(values[s.ref] ? { value: String(values[s.ref]) } : {}),
    ...(s.mpn ? { partLabel: s.mpn } : {}),
    bound: !!s.mpn,
  }))
}

// ── ngspice AC decks ────────────────────────────────────────────────────────────────────────────────
//
// Bench (Hertz SpiceDeck semantics): one LISN per line on the mains side (line_in / neutral_in), the
// noise source on the equipment side (line_out / neutral_out), PE = SPICE ground. The reference deck is
// the same bench with the source wired straight to the LISNs. Insertion loss = the ratio of the LISN
// receiver-port voltages of the two runs; the LISN's own EUT->receiver divider cancels in the ratio.
//
// LISN (single-line, simplified V-network): L from EUT to mains, C from EUT to the receiver port, the
// receiver's R from that port to ground. The mains side of the line LISN is an ideal 0 V source (an AC
// short), of the neutral LISN 1 mOhm to ground — Hertz's bench, verbatim. Valid 150 kHz–108 MHz
// (the 1 uF mains capacitor and the CISPR band-A branch are omitted, as in Hertz).
export const CISPR16_LISN = Object.freeze({ name: 'CISPR16 50uH', inductanceH: 50e-6, couplingCapacitanceF: 0.1e-6, measuringImpedanceOhm: 50 })
export const CISPR25_LISN = Object.freeze({ name: 'CISPR25 5uH', inductanceH: 5e-6, couplingCapacitanceF: 0.1e-6, measuringImpedanceOhm: 50 })

// The noise-source impedance the deck assumes, per mode (Hertz ABT #827). It is REQUIRED: an insertion
// loss without its source impedance is not a number anyone can act on.
//   cmStrayF     CM: stray capacitance from the switching node to earth; 0 = ideal drive (each line
//                through 1 mOhm) — the optimistic bound for the choke, and a bound under which the Y
//                capacitors, facing a short, can do nothing
//   dmSourceOhm  DM: series resistance standing in for the converter's bulk capacitance; 0 = ideal
//                voltage drive
export const IDEAL_NOISE_SOURCE = Object.freeze({ cmStrayF: 0, dmSourceOhm: 0 })

// The receiver-port node per line; pass one to insertionLossDb.
export const EMI_PROBES = Object.freeze({ line: 'meas_line', neutral: 'meas_neut' })

// MKF's export_magnetic_as_subcircuit writes a two-winding choke as
//   .subckt <partNumber> P1+ P1- P2+ P2-      (winding 1 start/end, winding 2 start/end; P1+ and P2+ dotted)
// The brick's CMC pins map onto it so that both lines enter at the dot: CM current then sees the
// magnetising inductance and DM current only the leakage. Swapping one winding (S1->P2-) inverts that,
// and the insertion-loss check (scripts/checkEmiFilterAc.mjs) fails on it.
export const MKF_CMC_PIN_MAP = Object.freeze({ P1: 'P1+', P2: 'P1-', S1: 'P2+', S2: 'P2-' })

const LISN_SUBCKT = 'kh_emi_lisn'

const finitePos = (x) => typeof x === 'number' && Number.isFinite(x) && x > 0
const num = (x) => {
  if (!Number.isFinite(x)) throw new Error(`EMI filter deck: non-finite value ${x}`)
  return Number(x).toPrecision(9).replace(/\.?0+(e|$)/, '$1')
}

// The datasheet ideal capacitor model as a subckt: Cs in series with ESR and ESL, the insulation
// resistance across the dielectric (in parallel with Cs). Every parameter is required. An ESR or ESL of
// exactly 0 omits that element (0 is a stated value, not a missing one); an insulation resistance of
// Infinity omits the parallel resistor.
// -> { subcktName, text, pinMap: { '1': 'a', '2': 'b' } }, ready to put in `models`.
export function capacitorSubckt({ subcktName, capacitanceF, esrOhm, eslH, insulationResistanceOhm } = {}) {
  if (typeof subcktName !== 'string' || !/^[A-Za-z0-9_]+$/.test(subcktName))
    throw new Error(`capacitorSubckt: subcktName must be [A-Za-z0-9_]+, got ${JSON.stringify(subcktName)}`)
  if (!finitePos(capacitanceF)) throw new Error(`capacitorSubckt ${subcktName}: capacitanceF must be > 0`)
  if (!(typeof esrOhm === 'number' && esrOhm >= 0 && Number.isFinite(esrOhm))) throw new Error(`capacitorSubckt ${subcktName}: esrOhm must be a number >= 0`)
  if (!(typeof eslH === 'number' && eslH >= 0 && Number.isFinite(eslH))) throw new Error(`capacitorSubckt ${subcktName}: eslH must be a number >= 0`)
  if (!(typeof insulationResistanceOhm === 'number' && insulationResistanceOhm > 0))
    throw new Error(`capacitorSubckt ${subcktName}: insulationResistanceOhm must be > 0 (Infinity to omit it)`)
  const lines = [`.subckt ${subcktName} a b`]
  let node = 'a'
  if (esrOhm > 0) { lines.push(`Rs a n_rs ${num(esrOhm)}`); node = 'n_rs' }
  if (eslH > 0) { lines.push(`Ls ${node} n_ls ${num(eslH)}`); node = 'n_ls' }
  lines.push(`Cs ${node} b ${num(capacitanceF)}`)
  if (Number.isFinite(insulationResistanceOhm)) lines.push(`Riso ${node} b ${num(insulationResistanceOhm)}`)
  lines.push(`.ends ${subcktName}`)
  return { subcktName, text: lines.join('\n') + '\n', pinMap: { 1: 'a', 2: 'b' } }
}

// The port list of `.subckt <name> ...` in `text`. Exactly one definition of that name must be present.
function subcktPorts(text, name) {
  const heads = String(text).split(/\r?\n/).map((l) => l.trim().split(/\s+/)).filter((t) => /^\.subckt$/i.test(t[0]))
  const mine = heads.filter((t) => (t[1] ?? '').toLowerCase() === name.toLowerCase())
  if (mine.length !== 1) throw new Error(`EMI filter deck: model text defines .subckt ${name} ${mine.length} times (expected once)`)
  const ports = []
  for (const tok of mine[0].slice(2)) {
    if (/^params:$/i.test(tok) || tok.includes('=')) break
    ports.push(tok)
  }
  if (!ports.length) throw new Error(`EMI filter deck: .subckt ${name} declares no ports`)
  return ports
}

const nodeName = (s) => String(s).toLowerCase().replace(/[^a-z0-9_]/g, '_')

function checkSweep({ fStart, fStop, pointsPerDecade }) {
  if (!finitePos(fStart)) throw new Error('EMI filter deck: fStart must be > 0 Hz')
  if (!finitePos(fStop) || !(fStop > fStart)) throw new Error('EMI filter deck: fStop must be > fStart')
  if (!Number.isInteger(pointsPerDecade) || pointsPerDecade < 1) throw new Error('EMI filter deck: pointsPerDecade must be a positive integer')
}

function checkLisn(lisn) {
  if (!lisn || !finitePos(lisn.inductanceH) || !finitePos(lisn.couplingCapacitanceF) || !finitePos(lisn.measuringImpedanceOhm))
    throw new Error('EMI filter deck: lisn needs inductanceH, couplingCapacitanceF and measuringImpedanceOhm (> 0); see CISPR16_LISN')
}

function checkSource(source) {
  if (!source || typeof source.cmStrayF !== 'number' || typeof source.dmSourceOhm !== 'number' ||
      !(source.cmStrayF >= 0) || !(source.dmSourceOhm >= 0) || !Number.isFinite(source.cmStrayF) || !Number.isFinite(source.dmSourceOhm))
    throw new Error('EMI filter deck: source needs cmStrayF (F, >= 0) and dmSourceOhm (Ohm, >= 0); IDEAL_NOISE_SOURCE is the ideal drive')
}

function sourceNote(source, mode) {
  if (mode === 'cm') return source.cmStrayF > 0
    ? `CM source: ${num(source.cmStrayF * 1e12)} pF stray to earth`
    : 'CM source: ideal drive through 1 mOhm per line - a bound: the Y capacitors face a short'
  return source.dmSourceOhm > 0
    ? `DM source: ${num(source.dmSourceOhm)} Ohm series (the converter's bulk capacitance)`
    : 'DM source: ideal 0 Ohm voltage drive - optimistic for the series leakage'
}

// The bench around a filter (or around nothing): LISNs on the mains nodes, the source on the equipment
// nodes, the sweep. `inLine/inNeut` are where the LISNs attach, `outLine/outNeut` where the source does.
function bench({ title, body, lisn, mode, source, sweep, inLine, inNeut, outLine, outNeut }) {
  if (mode !== 'cm' && mode !== 'dm') throw new Error(`EMI filter deck: mode must be 'cm' or 'dm', got ${JSON.stringify(mode)}`)
  checkLisn(lisn)
  checkSource(source)
  checkSweep(sweep)
  const L = []
  L.push(`* ${title}`)
  L.push(`* LISN: ${lisn.name ?? 'custom'} (${num(lisn.inductanceH)} H, ${num(lisn.couplingCapacitanceF)} F, ${num(lisn.measuringImpedanceOhm)} Ohm) per line`)
  L.push(`* ${sourceNote(source, mode)}`)
  L.push(`.subckt ${LISN_SUBCKT} eut mains meas`)
  L.push(`L1 eut mains ${num(lisn.inductanceH)}`)
  L.push(`C1 eut meas ${num(lisn.couplingCapacitanceF)}`)
  L.push(`R1 meas 0 ${num(lisn.measuringImpedanceOhm)}`)
  L.push(`.ends ${LISN_SUBCKT}`)
  L.push(...body)
  L.push(`Xlisn_line ${inLine} mains_line ${EMI_PROBES.line} ${LISN_SUBCKT}`)
  L.push(`Xlisn_neut ${inNeut} mains_neut ${EMI_PROBES.neutral} ${LISN_SUBCKT}`)
  L.push('Vmains mains_line 0 DC 0')
  L.push('Rmains_neut mains_neut 0 1m')
  if (mode === 'cm') {
    L.push('* COMMON-MODE drive: both lines together against PE (exercises choke + Y capacitors)')
    if (source.cmStrayF > 0) {
      L.push('Vnoise cm_drive 0 AC 1')
      L.push(`Cstray cm_drive cm_src ${num(source.cmStrayF)}`)
    } else {
      L.push('Vnoise cm_src 0 AC 1')
    }
    L.push(`Rsrc_line cm_src ${outLine} 1m`)
    L.push(`Rsrc_neut cm_src ${outNeut} 1m`)
  } else {
    L.push('* DIFFERENTIAL-MODE drive: line against neutral (exercises leakage + X capacitors)')
    if (source.dmSourceOhm > 0) {
      L.push(`Vnoise dm_drive ${outNeut} AC 1`)
      L.push(`Rsrc_dm dm_drive ${outLine} ${num(source.dmSourceOhm)}`)
    } else {
      L.push(`Vnoise ${outLine} ${outNeut} AC 1`)
    }
  }
  L.push(`.ac dec ${sweep.pointsPerDecade} ${num(sweep.fStart)} ${num(sweep.fStop)}`)
  L.push('.end')
  return L.join('\n') + '\n'
}

// buildEmiFilterAcDeck({ cias, models, lisn, mode, source, fStart, fStop, pointsPerDecade }) -> deck text
//   cias    an EMI filter brick (buildEmiFilterCias); placeholders are fine — a model per ref is what runs
//   models  { [ref]: { subcktName, text, pinMap } } for EVERY component of the brick:
//             subcktName  the .subckt name inside `text` (may start with a digit, as MKF's part numbers do)
//             text        the .subckt definition (its .param lines may live inside it)
//             pinMap      { <CIAS pin>: <subckt port> } covering every CIAS pin of that ref and every port
//                         of the subckt. CMC from MKF: MKF_CMC_PIN_MAP. capacitorSubckt() returns its own.
//           A missing model throws. Two refs may share one subckt (same name AND identical text).
//   lisn    CISPR16_LISN | CISPR25_LISN | { inductanceH, couplingCapacitanceF, measuringImpedanceOhm }
//   mode    'cm' | 'dm'
//   source  { cmStrayF, dmSourceOhm } — IDEAL_NOISE_SOURCE for the ideal drive
//   fStart, fStop, pointsPerDecade — the .ac dec sweep
export function buildEmiFilterAcDeck({ cias, models, lisn, mode, source, fStart, fStop, pointsPerDecade } = {}) {
  if (mode !== 'cm' && mode !== 'dm') throw new Error(`EMI filter deck: mode must be 'cm' or 'dm', got ${JSON.stringify(mode)}`)
  const { stages, slots } = parseEmiFilterCias(cias)
  if (!models || typeof models !== 'object') throw new Error('EMI filter deck: models { [ref]: { subcktName, text, pinMap } } is required')
  const refs = new Set(slots.map((s) => s.ref))
  for (const r of Object.keys(models)) if (!refs.has(r)) throw new Error(`EMI filter deck: model given for '${r}', which is not in the brick`)
  // pin -> SPICE node, from the brick's own connections (pe is ground)
  const pinNode = new Map()
  const portNode = new Map()
  for (const net of cias.connections) {
    const node = net.name === 'pe' ? '0' : `n_${nodeName(net.name)}`
    for (const e of net.endpoints) {
      if (e.port !== undefined) portNode.set(e.port, node)
      else pinNode.set(`${e.component}|${e.pin}`, node)
    }
  }
  const defs = new Map()   // subcktName(lower) -> text
  const body = ['* ---- filter models ----']
  const instances = ['* ---- filter ----']
  for (const s of slots) {
    const m = models[s.ref]
    if (!m) throw new Error(`EMI filter deck: no model for '${s.ref}'`)
    if (typeof m.subcktName !== 'string' || !m.subcktName || /\s/.test(m.subcktName)) throw new Error(`EMI filter deck: model for '${s.ref}' has no subcktName`)
    if (typeof m.text !== 'string' || !m.text.trim()) throw new Error(`EMI filter deck: model for '${s.ref}' has no subckt text`)
    if (!m.pinMap || typeof m.pinMap !== 'object') throw new Error(`EMI filter deck: model for '${s.ref}' has no pinMap`)
    if (m.subcktName.toLowerCase() === LISN_SUBCKT) throw new Error(`EMI filter deck: subckt name '${m.subcktName}' is reserved for the bench LISN`)
    const ports = subcktPorts(m.text, m.subcktName)
    const ciasPins = [...pinNode.keys()].filter((k) => k.startsWith(`${s.ref}|`)).map((k) => k.slice(s.ref.length + 1))
    const mapped = Object.keys(m.pinMap)
    const unmappedPins = ciasPins.filter((p) => !(p in m.pinMap))
    const strayPins = mapped.filter((p) => !ciasPins.includes(p))
    if (unmappedPins.length || strayPins.length)
      throw new Error(`EMI filter deck: pinMap of '${s.ref}' must cover exactly its CIAS pins [${ciasPins.join(', ')}] (unmapped: ${unmappedPins.join(', ') || '—'}; unknown: ${strayPins.join(', ') || '—'})`)
    const targets = mapped.map((p) => m.pinMap[p])
    if (new Set(targets).size !== targets.length) throw new Error(`EMI filter deck: pinMap of '${s.ref}' maps two pins onto one subckt port`)
    const missingPorts = ports.filter((p) => !targets.includes(p))
    const badTargets = targets.filter((t) => !ports.includes(t))
    if (missingPorts.length || badTargets.length)
      throw new Error(`EMI filter deck: pinMap of '${s.ref}' must reach exactly the ports of .subckt ${m.subcktName} [${ports.join(' ')}] (unreached: ${missingPorts.join(', ') || '—'}; not ports: ${badTargets.join(', ') || '—'})`)
    const key = m.subcktName.toLowerCase()
    if (defs.has(key)) {
      if (defs.get(key) !== m.text) throw new Error(`EMI filter deck: two different models are both named .subckt ${m.subcktName}`)
    } else {
      defs.set(key, m.text)
      body.push(m.text.trimEnd())
    }
    const byPort = new Map(mapped.map((p) => [m.pinMap[p], pinNode.get(`${s.ref}|${p}`)]))
    instances.push(`X${nodeName(s.ref)} ${ports.map((p) => byPort.get(p)).join(' ')} ${m.subcktName}`)
  }
  return bench({
    title: `Kirchhoff EMI line filter, ${stages} stage(s), ${mode.toUpperCase()} insertion-loss bench`,
    body: [...body, ...instances],
    lisn, mode, source, sweep: { fStart, fStop, pointsPerDecade },
    inLine: portNode.get('line_in'), inNeut: portNode.get('neutral_in'),
    outLine: portNode.get('line_out'), outNeut: portNode.get('neutral_out'),
  })
}

// buildLisnReferenceDeck({ lisn, mode, source, fStart, fStop, pointsPerDecade }) -> deck text
// The same bench with NO filter: the source drives the LISNs directly. Use the SAME arguments as the
// filter deck, or the ratio is not an insertion loss.
export function buildLisnReferenceDeck({ lisn, mode, source, fStart, fStop, pointsPerDecade } = {}) {
  return bench({
    title: `Kirchhoff EMI REFERENCE bench (no filter), ${mode} - IL = vdb(ref) - vdb(filter) at the receiver port`,
    body: [], lisn, mode, source, sweep: { fStart, fStop, pointsPerDecade },
    inLine: 'src_line', inNeut: 'src_neut', outLine: 'src_line', outNeut: 'src_neut',
  })
}

function vectorOf(result, probe, which) {
  if (!result || result.success !== true) throw new Error(`insertionLossDb: the ${which} run failed: ${result?.error || 'no result'}`)
  if (!Array.isArray(result.frequenciesHz) || !result.frequenciesHz.length) throw new Error(`insertionLossDb: the ${which} run has no frequency vector`)
  const want = nodeName(String(probe).replace(/^v\((.*)\)$/i, '$1'))
  const key = Object.keys(result.vectors ?? {}).find((k) => nodeName(k.replace(/^v\((.*)\)$/i, '$1')) === want)
  if (!key) throw new Error(`insertionLossDb: the ${which} run has no vector '${probe}' (has: ${Object.keys(result.vectors ?? {}).join(', ')})`)
  const v = result.vectors[key]
  if (!Array.isArray(v.re) || !Array.isArray(v.im) || v.re.length !== result.frequenciesHz.length || v.im.length !== v.re.length)
    throw new Error(`insertionLossDb: vector '${key}' of the ${which} run is not a complex vector over the sweep`)
  return v
}

// insertionLossDb(refResult, filtResult, probe) -> { frequenciesHz, ilDb }
// Both results are run_ngspice_ac's parsed output ({ success, error, frequenciesHz, vectors }) for the
// reference and the filter deck. IL(f) = 20·log10(|V_ref| / |V_filt|) at `probe` (EMI_PROBES.line /
// .neutral). The two sweeps must be the same points.
export function insertionLossDb(refResult, filtResult, probe) {
  if (typeof probe !== 'string' || !probe) throw new Error('insertionLossDb: probe (e.g. EMI_PROBES.line) is required')
  const r = vectorOf(refResult, probe, 'reference')
  const f = vectorOf(filtResult, probe, 'filter')
  const fr = refResult.frequenciesHz, ff = filtResult.frequenciesHz
  if (fr.length !== ff.length || fr.some((x, i) => Math.abs(x - ff[i]) > 1e-9 * Math.abs(x)))
    throw new Error('insertionLossDb: the reference and filter runs were swept over different frequencies')
  const ilDb = fr.map((_, i) => {
    const a = Math.hypot(r.re[i], r.im[i]), b = Math.hypot(f.re[i], f.im[i])
    if (!(a > 0) || !(b > 0)) throw new Error(`insertionLossDb: zero response at ${fr[i]} Hz (${a === 0 ? 'reference' : 'filter'}) — no ratio exists`)
    return 20 * Math.log10(a / b)
  })
  return { frequenciesHz: [...fr], ilDb }
}
