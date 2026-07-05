// Facade over the Kirchhoff WASM engine, hosted in a Web Worker (src/worker.js)
// so multi-second ngspice transients never block the UI. Every API call is
// string-in/string-out JSON; errors cross the boundary as strings starting
// with "Exception: " — we re-throw those so callers see real errors.

let worker = null
let seq = 0
const pending = new Map()

function getWorker() {
  if (!worker) {
    worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' })
    worker.onmessage = (ev) => {
      const { id, ok, result, error } = ev.data
      const p = pending.get(id)
      if (!p) return
      pending.delete(id)
      ok ? p.resolve(result) : p.reject(new Error(error))
    }
    worker.onerror = (e) => {
      for (const p of pending.values()) p.reject(new Error(e.message ?? 'engine worker crashed'))
      pending.clear()
    }
  }
  return worker
}

function rpc(fn, ...args) {
  return new Promise((resolve, reject) => {
    const id = ++seq
    pending.set(id, { resolve, reject })
    getWorker().postMessage({ id, fn, args })
  })
}

async function call(fn, ...args) {
  const out = await rpc(fn, ...args)
  if (typeof out === 'string' && out.startsWith('Exception: ')) {
    throw new Error(out.slice('Exception: '.length))
  }
  return out
}

const callJson = async (fn, ...args) => JSON.parse(await call(fn, ...args))

// Resolves once the WASM module is compiled and ready inside the worker.
export function loadEngine() {
  return rpc('__init__')
}

export function processConverter(topology, spec, engine = 'analytical') {
  return callJson('process_converter', topology, JSON.stringify(spec), engine)
}

export function topologyWaveforms(tas) {
  return callJson('topology_waveforms', JSON.stringify(tas))
}

// Per-component V/I (switches, diodes, caps, resistors) from one ngspice run.
// Returns { engine, referencePeriod, components:[...] } or { success:false } if no libngspice.
export function componentWaveforms(tas, fidelity = { origin: 'REQUIREMENTS' }) {
  return callJson('component_waveforms', JSON.stringify(tas), JSON.stringify(fidelity))
}

// Add requirements-derived datasheet models (real Rds(on)/Vf) to every semiconductor so a
// DATASHEET-fidelity sim renders real-conduction devices. Returns the realized TAS.
export function realizeTas(tas) {
  return callJson('realize_tas', JSON.stringify(tas))
}

export function extractOperatingPoint(tas, engine = 'analytical', magneticName = '') {
  return callJson('extract_operating_point', JSON.stringify(tas), engine, magneticName)
}

export function mainMagneticInputs(tas) {
  return callJson('main_magnetic_inputs', JSON.stringify(tas))
}

export function generateNetlist(tas, flavor = 'ngspice', fidelity = { origin: 'REQUIREMENTS' }) {
  const fn = flavor === 'ltspice' ? 'generate_ltspice_circuit' : 'generate_ngspice_circuit'
  return call(fn, JSON.stringify(tas), JSON.stringify(fidelity))
}

export function simulateNgspice(tas, fidelity = { origin: 'REQUIREMENTS' }) {
  return callJson('simulate_ngspice', JSON.stringify(tas), JSON.stringify(fidelity))
}

// ── Kelvin component sourcing (real parts from the TAS DB) ──────────────────
// Prebuilt per-family index shards (.kidx) are hosted next to the SPA (public/kelvin/) and
// loaded into the WASM engine on first use of a category; selection then runs fully in-browser
// over the shard (no backend). Candidates carry MPN / manufacturer / margins / evidence + the
// record's byte span — the full datasheet envelope is fetched on demand (Range) only when binding.
const KELVIN_BASE = '/kelvin'
const KIND_TO_CATEGORY = {
  MOSFET: 'mosfet', Diode: 'diode', Capacitor: 'capacitor', Resistor: 'resistor',
  Controller: 'controller', IGBT: 'igbt', BJT: 'bjt', Varistor: 'varistor',
}
const _kelvinShards = new Map() // family -> Promise<meta>

export function kelvinCategoryFor(kind) {
  return KIND_TO_CATEGORY[kind] ?? null
}

function ensureShard(category) {
  if (!_kelvinShards.has(category)) {
    _kelvinShards.set(category, (async () => {
      const res = await fetch(`${KELVIN_BASE}/${category}.kidx`)
      if (!res.ok) throw new Error(`Kelvin shard '${category}' not hosted (HTTP ${res.status})`)
      const bytes = new Uint8Array(await res.arrayBuffer())
      return callJson('kelvin_load_shard', category, bytes)
    })())
  }
  return _kelvinShards.get(category)
}

// Given a BOM row's kind + its designRequirements (+ converter context for controllers/HV FETs),
// return a Kelvin SelectionResult ({candidates, rejections, ...}) or {error:'NoCandidates',...}.
export async function selectCandidates(kind, requirements, context = {}) {
  const category = kelvinCategoryFor(kind)
  if (!category) throw new Error(`no Kelvin category for kind '${kind}'`)
  await ensureShard(category)
  const options = { maxCandidates: 12 }
  if (context.switchingFrequency) options.switchingFrequency = context.switchingFrequency
  if (context.inputVoltage) options.inputVoltage = context.inputVoltage
  if (context.topology) options.topology = context.topology
  return callJson('kelvin_select', category, JSON.stringify(requirements), JSON.stringify(options))
}

// Cross-reference: given an ORIGINAL part's spec block and a candidate list
// (e.g. from selectCandidates over the original's value), return Kelvin's
// deterministic scored substitutes — {category, original_verified, candidates:
// [{mpn, status, penalty, params, ...}]}, ranked best-first. Runs fully
// in-browser over the WASM ranker, NO LLM (Kirchhoff's program-only mode; the
// same authority Heaviside runs its LLM chooser over). `originalVerified=false`
// applies the honesty cap (an unidentified original never yields 'recommended').
export function crossReference(kind, original, candidates, { originalVerified = true, maxResults = 12 } = {}) {
  const category = kelvinCategoryFor(kind)
  if (!category) throw new Error(`no Kelvin category for kind '${kind}'`)
  const options = { original_verified: originalVerified, max_results: maxResults }
  return callJson(
    'cross_reference_string',
    category,
    JSON.stringify(original),
    JSON.stringify(candidates),
    JSON.stringify(options),
  )
}
