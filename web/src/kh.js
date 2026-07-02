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
