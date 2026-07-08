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

// fidelity is the NGSPICE deck directive — base origin + optional per-component origin overrides
// ({ origin, components: { T1: 'MKF_MODEL', ... } }), same shape generateNetlist/simulateNgspice take.
export function extractOperatingPoint(tas, engine = 'analytical', magneticName = '', fidelity = { origin: 'REQUIREMENTS' }) {
  return callJson('extract_operating_point', JSON.stringify(tas), engine, magneticName, JSON.stringify(fidelity))
}

// ref names which magnetic to design (BOM ref == component name). '' → the main magnetic. Multi-magnetic
// topologies (LLC/CLLC/CLLLC transformer + resonant inductor, SEPIC/Cuk/Zeta two inductors, PSFB/DAB)
// need the ref so the drawer designs the magnetic the user actually opened, not always the main one.
export function mainMagneticInputs(tas, ref = '') {
  return callJson('main_magnetic_inputs', JSON.stringify(tas), ref)
}

// The MAS inputs KH emits carry only processed stats (waveform: null); OM's magnetic adviser needs a
// real ≥2-point time-domain waveform and won't reliably reconstruct one from stats ("Waveform must have
// at least 2 data points"). KH already computed the per-winding waveforms — splice them in, keyed by
// winding index within each operating point. `windings` is the BOM row's `windings` (analyticalWaveforms
// excitationsPerWinding). Mutates + returns `inputs`.
export function enrichMagneticWaveforms(inputs, windings) {
  if (!Array.isArray(windings) || !windings.length || !Array.isArray(inputs?.operatingPoints)) return inputs
  for (const op of inputs.operatingPoints) {
    const exs = op?.excitationsPerWinding
    if (!Array.isArray(exs)) continue
    for (let w = 0; w < exs.length && w < windings.length; w++) {
      for (const side of ['voltage', 'current']) {
        // Only splice a genuinely 'custom' shape (QRM valley-switching): MKF reconstructs the named shapes
        // (rectangular, flybackSecondaryWithDeadtime, …) from stats and splicing those breaks it. NOTE: the
        // JS reshape below is still an approximation — a full fix builds the waveform via MKF's own
        // WaveformProcessor in C++ (consistent waveform+processed+harmonics). See ABT #132.
        if (exs[w]?.[side]?.processed?.label !== 'custom') continue
        // Replace the whole side with ConverterAnalytical's version — MKF's WaveformProcessor already built
        // it consistently (waveform + harmonics + processed, canonical MAS format). Don't hand-reshape.
        const src = windings[w]?.[side]
        if (Array.isArray(src?.waveform?.data) && src.waveform.data.length >= 2) exs[w][side] = JSON.parse(JSON.stringify(src))  // strip Vue reactivity → structured-cloneable for postMessage
      }
    }
  }
  return inputs
}

// ── Magnetics: OpenMagnetics adviser handoff ────────────────────────────────────────────────────
// Magnetics aren't catalog-matched — they're custom-designed by MKF's adviser. So instead of a
// candidate table the drawer exports this component's MAS Inputs, opens the OpenMagnetics magnetic
// adviser in a new tab, hands the Inputs over via a cross-origin postMessage handshake, and — when
// the user picks one of the advised designs — receives the MAS magnetic back to bind into the TAS.
// Origin is overridable for local dev (window.__OPENMAGNETICS_ORIGIN__).
// Origin of the OpenMagnetics app. Prod default; overridable for local dev via a ?om=<origin> query
// param on the KH page or window.__OPENMAGNETICS_ORIGIN__.
function resolveOmOrigin() {
  if (typeof window === 'undefined') return 'https://openmagnetics.com'
  const q = new URLSearchParams(window.location.search).get('om')  // ?om=<origin> overrides for local dev
  return q || window.__OPENMAGNETICS_ORIGIN__ || 'https://openmagnetics.com'
}
const OPENMAGNETICS_ORIGIN = resolveOmOrigin()

// Open the OM adviser tab with these MAS Inputs and run one handoff round-trip. Two modes:
//   'design' — the user picks ONE design in the OM UI; resolves with { mas }.
//   'advise' — the adviser auto-runs headlessly, posts the WHOLE advise list and closes the tab;
//              resolves with { advises: [{mas, scoringPerFilter, weightedTotalScoring}] }.
// Rejects if the popup is blocked or the tab closes without a reply. `ref` labels the round-trip so
// a stray message from an unrelated OM tab can't bind the wrong component. An OLD deployed OM that
// predates 'advise' ignores the mode and behaves like 'design' (user picks there, single 'magnetic'
// reply) — accepted in either mode so the flow degrades gracefully instead of hanging.
function openMagneticsHandoff(ref, inputs, mode) {
  return new Promise((resolve, reject) => {
    // Open the OM home (it preloads the magnetic engine); the handoff bridge navigates to the adviser
    // once it has our inputs, so the adviser mounts with the MAS already loaded (no reset race).
    const omWin = window.open(`${OPENMAGNETICS_ORIGIN}/?handoff=kirchhoff`, '_blank')
    if (!omWin) return reject(new Error('pop-up blocked — allow pop-ups to use OpenMagnetics'))
    let settled = false
    const cleanup = () => { settled = true; window.removeEventListener('message', onMessage); clearInterval(poll) }
    function onMessage(ev) {
      if (ev.origin !== OPENMAGNETICS_ORIGIN || ev.data?.source !== 'openmagnetics') return
      if (ev.data.type === 'ready') {
        // Surface a structured-clone failure LOUDLY. A throw here (this is an event handler) would
        // otherwise die silently and the whole handoff would no-op with zero feedback — which is
        // exactly what a Vue reactive proxy in `inputs` once did. inputs must be plain cloneable data.
        try {
          omWin.postMessage({ source: 'kirchhoff', type: 'mas-inputs', ref, inputs, mode }, OPENMAGNETICS_ORIGIN)
        } catch (e) {
          cleanup()
          reject(new Error(`could not send MAS inputs to OpenMagnetics (non-cloneable data — a Vue reactive object? see enrichMagneticWaveforms): ${e.message}`))
        }
      } else if (ev.data.type === 'magnetic' && ev.data.ref === ref) {
        cleanup(); resolve({ mas: ev.data.mas, advises: [{ mas: ev.data.mas }], droppedInvalid: 0 })
      } else if (ev.data.type === 'magnetics' && ev.data.ref === ref) {
        cleanup(); resolve({ mas: ev.data.advises?.[0]?.mas ?? null, advises: ev.data.advises ?? [],
                             droppedInvalid: ev.data.droppedInvalid ?? 0 })
      }
    }
    window.addEventListener('message', onMessage)
    const poll = setInterval(() => {
      // The advise-mode tab closes itself right after posting — give an in-flight message a grace
      // tick to land before treating the close as an abort.
      if (omWin.closed && !settled)
        setTimeout(() => {
          if (!settled) { cleanup(); reject(new Error('OpenMagnetics tab closed without sending a design')) }
        }, 1500)
    }, 1000)
  })
}

// Interactive design round-trip — resolves with the single MAS the user chose in OM.
export async function designMagneticInOpenMagnetics(ref, inputs) {
  return (await openMagneticsHandoff(ref, inputs, 'design')).mas
}

// Headless adviser round-trip — resolves with { advises, droppedInvalid }: the advise LIST
// (candidate-table material: each row a full MAS with its NGSPICE subcircuit attached, plus the
// adviser's scoring) and how many adviser results were dropped for failing MKF's coil validity
// filters (OM marks those with an "INVALID (failed validity filters): " reference prefix — they
// are not buildable designs, so they are filtered, and the count says why the list is short).
export async function suggestMagneticsInOpenMagnetics(ref, inputs) {
  const INVALID_PREFIX = 'INVALID (failed validity filters): '
  const r = await openMagneticsHandoff(ref, inputs, 'advise')
  // Defensive re-filter: an OM deploy that predates the OM-side filter still marks them.
  const advises = r.advises.filter((a) => !(a?.mas?.magnetic?.manufacturerInfo?.reference ?? '').startsWith(INVALID_PREFIX))
  return { advises, droppedInvalid: (r.droppedInvalid ?? 0) + (r.advises.length - advises.length) }
}

// Bind an OM-advised magnetic (a full MAS) into the TAS component's data.magnetic, verbatim (already
// schema-valid). Reuses Kelvin's bind_part — the envelope's first key ('magnetic') is the family slot.
export function bindMagnetic(tas, ref, mas) {
  const magnetic = mas?.magnetic ?? mas
  return callJson('bind_part', JSON.stringify(tas), ref, JSON.stringify({ magnetic }))
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
  // Magnetics also get a Kelvin catalog lookup (real off-the-shelf inductors/transformers), ALONGSIDE
  // the OpenMagnetics custom-design adviser. The selector ranks the whole catalog and returns matches
  // even when none fully meet the spec (see Kelvin select_magnetic).
  Inductor: 'magnetic', Transformer: 'magnetic',
}
const _kelvinShards = new Map() // family -> Promise<meta>
let _manifestPromise = null

export function kelvinCategoryFor(kind) {
  return KIND_TO_CATEGORY[kind] ?? null
}

// The deploy pairing record (per family: buildId + sourceSize) that kelvin-index writes beside the
// shards. Fetched once with cache:'no-store' so a redeploy is seen immediately. Required — it drives
// shard cache-busting and the bind-time version guard; a shards-without-manifest deploy is a bug we
// surface rather than paper over.
function kelvinManifest() {
  if (!_manifestPromise) {
    _manifestPromise = (async () => {
      const res = await fetch(`${KELVIN_BASE}/manifest.json`, { cache: 'no-store' })
      if (!res.ok) throw new Error(`Kelvin manifest not hosted (HTTP ${res.status}) — deploy manifest.json with the shards`)
      const m = await res.json()
      if (!m?.families) throw new Error('Kelvin manifest malformed (no families)')
      return m
    })()
  }
  return _manifestPromise
}

async function manifestEntry(category) {
  const m = await kelvinManifest()
  const e = m.families[category]
  if (!e) throw new Error(`Kelvin manifest has no entry for '${category}'`)
  return e
}

// Shard bytes, cache-busted by buildId in Cache Storage so they persist across reloads and a new
// build (new buildId → new URL) orphans the stale entry (which we evict). Falls back to a plain
// fetch where Cache Storage is unavailable (a non-secure context).
async function fetchShardBytes(category, buildId) {
  const url = `${KELVIN_BASE}/${category}.kidx?b=${buildId}`
  if (typeof caches === 'undefined') {
    const res = await fetch(url)
    if (!res.ok) throw new Error(`Kelvin shard '${category}' not hosted (HTTP ${res.status})`)
    return new Uint8Array(await res.arrayBuffer())
  }
  const cache = await caches.open('kelvin-shards')
  const hit = await cache.match(url)
  if (hit) return new Uint8Array(await hit.arrayBuffer())
  for (const req of await cache.keys()) {            // drop older builds of this family
    if (req.url.includes(`/${category}.kidx?`) && !req.url.endsWith(`b=${buildId}`)) await cache.delete(req)
  }
  const res = await fetch(url)
  if (!res.ok) throw new Error(`Kelvin shard '${category}' not hosted (HTTP ${res.status})`)
  await cache.put(url, res.clone())
  return new Uint8Array(await res.arrayBuffer())
}

function ensureShard(category) {
  if (!_kelvinShards.has(category)) {
    _kelvinShards.set(category, (async () => {
      const entry = await manifestEntry(category)
      const bytes = await fetchShardBytes(category, entry.buildId)
      return callJson('kelvin_load_shard', category, bytes)
    })())
  }
  return _kelvinShards.get(category)
}

// Given a BOM row's kind + its designRequirements (+ converter context for controllers/HV FETs),
// return a Kelvin SelectionResult ({candidates, rejections, ...}) or {error:'NoCandidates',...}.
// Cap any single manufacturer at 20% of the candidate list so the ranked Pareto spans many vendors
// (Kelvin's diversity policy — apply_mfr_policy; without it e.g. a flyback cap list comes back all
// Panasonic). Kelvin defaults to no cap to stay parity-locked, so the consumer opts in here.
const KELVIN_MAX_MFR_FRACTION = 0.2

export async function selectCandidates(kind, requirements, context = {}) {
  const category = kelvinCategoryFor(kind)
  if (!category) throw new Error(`no Kelvin category for kind '${kind}'`)
  await ensureShard(category)
  const options = { maxCandidates: 12, maxManufacturerFraction: KELVIN_MAX_MFR_FRACTION }
  if (context.switchingFrequency) options.switchingFrequency = context.switchingFrequency
  if (context.inputVoltage) options.inputVoltage = context.inputVoltage
  if (context.topology) options.topology = context.topology
  // Manufacturer controls (settings; GUI-wired later). The diversity cap defaults
  // to KELVIN_MAX_MFR_FRACTION above; an optional allowlist restricts to named vendors.
  if (context.maxManufacturerFraction != null) options.maxManufacturerFraction = context.maxManufacturerFraction
  if (context.manufacturerAllowlist) options.manufacturerAllowlist = context.manufacturerAllowlist
  return callJson('kelvin_select', category, JSON.stringify(requirements), JSON.stringify(options))
}

// Bind (Tier 2): pull the ONE chosen record out of the hosted NDJSON by its byte span and write it
// into the TAS. Only that record crosses the wire — an HTTP Range read of a few KB, never the
// ~hundreds-of-MB catalog. The static host must serve <category>.ndjson byte-identical to what the
// shard was built from (same buildId) and honour Range (a 206). No backend, no API — the server just
// hands back a file slice; bind_part runs in WASM. Returns the new (schema-valid) TAS object.
export async function bindPart(tas, ref, kind, candidate) {
  let envelope = candidate.envelope   // only set when a native RecordFetcher was injected — never in-browser
  if (envelope === undefined) {
    const category = kelvinCategoryFor(kind)
    if (!category) throw new Error(`no Kelvin category for kind '${kind}'`)
    const { srcOffset, srcLength } = candidate
    if (typeof srcOffset !== 'number' || typeof srcLength !== 'number') {
      throw new Error(`candidate '${candidate.mpn}' carries no byte span (srcOffset/srcLength) to fetch its record`)
    }
    const entry = await manifestEntry(category)
    const res = await fetch(`${KELVIN_BASE}/${category}.ndjson`,
      { headers: { Range: `bytes=${srcOffset}-${srcOffset + srcLength - 1}` } })
    // A 206 is a genuine ranged read. A 200 means the host IGNORED Range and would stream the whole
    // catalog — refuse without reading the body rather than download hundreds of MB into the tab.
    if (res.status !== 206) {
      throw new Error(res.ok
        ? `${category}.ndjson host ignored Range (HTTP ${res.status}) — enable byte-range serving`
        : `record fetch failed (HTTP ${res.status}) — is ${category}.ndjson hosted next to the shard?`)
    }
    // Version guard: the hosted NDJSON's total size (from Content-Range) MUST equal the size the shard
    // was indexed against (manifest.sourceSize). A mismatch means shard and catalog are from different
    // deploys, so this candidate's byte offsets could point at the wrong record — refuse the bind
    // rather than write a wrong part into the design.
    const total = Number(res.headers.get('Content-Range')?.split('/')[1])
    if (Number.isFinite(total) && total !== entry.sourceSize) {
      throw new Error(`${category}: shard/catalog version mismatch (NDJSON ${total} B vs shard-indexed ${entry.sourceSize} B) — redeploy shards + NDJSON + manifest together`)
    }
    envelope = JSON.parse(await res.text())
  }
  return callJson('bind_part', JSON.stringify(tas), ref, JSON.stringify(envelope))
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
