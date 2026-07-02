<script setup>
import { computed, onMounted, onUnmounted, reactive, ref, watch } from 'vue'
import { FAMILIES, PLANNED, TOPOLOGIES, buildSpec, topologyById } from './topologies.js'
import { loadEngine, processConverter, topologyWaveforms, extractOperatingPoint, componentWaveforms, generateNetlist } from './kh.js'
import { extractBom } from './bom.js'
import { hasSchematic, renderSchematic } from './schematics.js'
import { si, pct } from './units.js'
import WavePane from './components/WavePane.vue'
import PartDrawer from './components/PartDrawer.vue'

// ── engine boot ────────────────────────────────────────────────────────────
const engineState = ref('loading')
onMounted(async () => {
  try {
    await loadEngine()
    engineState.value = 'ready'
  } catch (e) {
    engineState.value = 'error'
    runError.value = `WASM engine failed to load: ${e.message}`
  }
})

// ── topology & spec form ───────────────────────────────────────────────────
const topoId = ref('flyback')
// ngspice by default: the real transient is the reference; analytical stays one
// click away for instant iteration. 100 periods settle the converter, 2 shown.
const form = reactive({ engine: 'ngspice', settlePeriods: 100, showPeriods: 2 })

function selectTopology(id) {
  topoId.value = id
  const p = topologyById(id).preset
  Object.assign(form, {
    inputType: p.inputType,
    vinMin: p.vinMin, vinNom: p.vinNom, vinMax: p.vinMax,
    fs: p.fs, efficiency: p.efficiency, ambient: p.ambient,
    isolation: p.isolation, lineFrequency: p.lineFrequency,
    minOutputs: p.minOutputs, maxOutputs: p.maxOutputs,
    outputs: p.outputs.map((o) => ({ ...o })),
    ops: [{ name: 'full_load', vin: p.vinNom, ambient: p.ambient, powers: p.outputs.map((o) => o.power) }],
  })
  result.value = null
  runError.value = null
  bomRows.value = []
  waveMagnetics.value = []
  ngspiceOps.value = {}
  componentWaves.value = null
  selectedPart.value = null
}
const topo = computed(() => topologyById(topoId.value))
const grouped = computed(() =>
  FAMILIES.map((f) => ({
    family: f,
    items: [
      ...TOPOLOGIES.filter((t) => t.family === f),
      ...PLANNED.filter((t) => t.family === f).map((t) => ({ ...t, planned: true })),
    ],
  }))
)

function addOutput() {
  form.outputs.push({ name: `out${form.outputs.length + 1}`, voltage: 12, power: 20 })
  for (const op of form.ops) op.powers.push(20)
}
function removeOutput(i) {
  form.outputs.splice(i, 1)
  for (const op of form.ops) op.powers.splice(i, 1)
}
function addOp() {
  form.ops.push({
    name: `op_${form.ops.length + 1}`,
    vin: form.vinNom,
    ambient: form.ambient,
    powers: form.outputs.map((o) => o.power / 2),
  })
}

// ── solve ──────────────────────────────────────────────────────────────────
const running = ref(false)
const runError = ref(null)
const result = ref(null)
const bomRows = ref([])
const waveMagnetics = ref([])
const activeTab = ref('schematic')
const ngspiceOps = ref({})        // magnetic name -> simulated MAS::OperatingPoint
const ngspiceBusy = ref(false)
const componentWaves = ref(null)  // { referencePeriod, components:[{ref,kind,voltage,current}] } | null
const componentBusy = ref(false)

async function solve() {
  running.value = true
  runError.value = null
  result.value = null
  deck.value = ''
  ngspiceOps.value = {}
  componentWaves.value = null
  try {
    const spec = buildSpec(form)
    const res = await processConverter(topoId.value, spec, form.engine)
    result.value = res
    waveMagnetics.value = await topologyWaveforms(res.tas)
    waveMagIdx.value = Math.max(0, waveMagnetics.value.findIndex((m) => m.isMain))
    waveOpIdx.value = 0
    // an ngspice solve already simulated the main magnetic — cache its operating point, and pull
    // every non-magnetic component's V/I from the same simulator in one more call.
    if (form.engine === 'ngspice') {
      const main = waveMagnetics.value.find((m) => m.isMain)
      if (main) ngspiceOps.value = { [main.name]: res.operatingPoint }
      fetchComponentWaves()   // fire-and-forget; fills the component picker when it lands
    }
    bomRows.value = enrichBom(extractBom(res.tas), res.analyticalWaveforms)
    activeTab.value = hasSchematic(topoId.value) ? 'schematic' : 'bom'
  } catch (e) {
    runError.value = e.message
  } finally {
    running.value = false
  }
}

// One ngspice run → every non-magnetic component's V/I. Cached on result; the picker reads it.
// A generation counter guards against a stale overwrite: each call claims the newest generation, and
// only that generation may write the shared state — so a slower run from a superseded solve is dropped
// (rather than clobbering the current design's data or stranding it behind a `busy` flag).
let cwGen = 0
async function fetchComponentWaves() {
  if (!result.value) return
  const gen = ++cwGen
  componentBusy.value = true
  try {
    const cw = await componentWaveforms(result.value.tas)
    if (gen !== cwGen) return                // superseded by a newer solve — discard
    componentWaves.value = cw?.success === false ? null : cw
  } catch (e) {
    if (gen === cwGen) runError.value = e.message
  } finally {
    if (gen === cwGen) componentBusy.value = false
  }
}

// Magnetics rows get the FULL analytical operating point (waveforms incl. resonant/custom labels)
// captured by the engine during the build — richer than the stripped excitations baked in the TAS.
function enrichBom(rows, analyticalWaveforms) {
  for (const r of rows) {
    const full = analyticalWaveforms?.[r.ref]
    if (full?.excitationsPerWinding?.length) r.windings = full.excitationsPerWinding
  }
  return rows
}

// ── schematic ──────────────────────────────────────────────────────────────
const schematicSvg = computed(() =>
  result.value ? renderSchematic(topoId.value, bomRows.value) : null
)
const selectedPart = ref(null)
// The simulated V/I for the open part, as an excitation WavePane can render (non-magnetics only;
// magnetics keep their per-winding block in the drawer).
const selectedPartWave = computed(() => {
  const c = componentWaves.value?.components?.find((x) => x.ref === selectedPart.value?.ref)
  if (!c) return null
  const f = componentWaves.value?.referencePeriod ? 1 / componentWaves.value.referencePeriod : 0
  return { name: `${c.ref} · ${c.voltage?.label ?? 'V'}`, frequency: f, current: c.current, voltage: c.voltage }
})

selectTopology(topoId.value)

function schematicClick(ev) {
  const g = ev.target.closest('[data-ref]')
  if (!g) return
  openPart(g.dataset.ref)
}
function openPart(ref_) {
  const row = bomRows.value.find((r) => r.ref === ref_)
  if (row) selectedPart.value = row
}
function onKey(e) {
  if (e.key === 'Escape') selectedPart.value = null
}
onMounted(() => window.addEventListener('keydown', onKey))
onUnmounted(() => window.removeEventListener('keydown', onKey))

// ── waveforms tab ──────────────────────────────────────────────────────────
const waveMagIdx = ref(0)
const waveOpIdx = ref(0)
const waveMag = computed(() => waveMagnetics.value[waveMagIdx.value] ?? null)
const waveOps = computed(() => waveMag.value?.inputs?.operatingPoints ?? [])

// Excitation source hierarchy per magnetic: simulated (ngspice, on demand or from an
// ngspice solve) > full analytical capture (waveforms incl. custom labels) > the
// stripped TAS excitations (WavePane re-synthesizes standard labels from those).
//
// The ngspice extraction only rebuilds winding CURRENTS from the simulated inductor
// branches — it leaves voltage without a waveform. So in ngspice mode we splice the
// analytical voltage (which we already captured, per winding) back in: current is
// then measured, voltage predicted. Each is labelled so nothing is silently mixed.
const waveSource = computed(() => {
  const name = waveMag.value?.name
  if (!name) return { excitations: [], kind: 'none', voltageKind: null }
  const analyticalFull = result.value?.analyticalWaveforms?.[name]?.excitationsPerWinding
  const sim = ngspiceOps.value[name]
  if (sim) {
    const excitations = (sim.excitationsPerWinding ?? []).map((e, i) => {
      const av = analyticalFull?.[i]?.voltage
      const hasSimV = e.voltage?.waveform?.data?.length > 1
      if (!hasSimV && av?.waveform?.data?.length > 1) return { ...e, voltage: av }
      return e
    })
    return {
      excitations,
      kind: 'ngspice',
      // voltage came from the analytical capture only when the sim didn't provide it
      voltageKind: analyticalFull ? 'analytical' : null,
    }
  }
  const full = result.value?.analyticalWaveforms?.[name]
  if (full?.excitationsPerWinding?.length)
    return { excitations: full.excitationsPerWinding, kind: 'analytical (full waveforms)', voltageKind: null }
  return {
    excitations: waveOps.value[waveOpIdx.value]?.excitationsPerWinding ?? [],
    kind: 'analytical (processed)',
    voltageKind: null,
  }
})
const waveExcitations = computed(() => waveSource.value.excitations)

// ── unified waveform picker: magnetics (per-winding) + devices (per-component V/I) ──────────
// Target is "mag:<idx>" or "dev:<ref>". Devices come from the BOM (always listed) and get their
// waveforms from componentWaves once an ngspice run has filled it.
const waveTarget = ref('mag:0')
watch(waveMagnetics, () => { waveTarget.value = `mag:${waveMagIdx.value}` })

const DEVICE_GROUPS = [
  ['Switches', 'MOSFET'],
  ['Diodes', 'Diode'],
  ['Capacitors', 'Capacitor'],
  ['Resistors', 'Resistor'],
]
const deviceGroups = computed(() =>
  DEVICE_GROUPS
    .map(([label, kind]) => ({ label, items: bomRows.value.filter((r) => r.kind === kind) }))
    .filter((g) => g.items.length)
)

const targetIsMagnetic = computed(() => waveTarget.value.startsWith('mag:'))
watch(waveTarget, (t) => { if (t.startsWith('mag:')) waveMagIdx.value = Number(t.slice(4)) })

const deviceRef = computed(() => (waveTarget.value.startsWith('dev:') ? waveTarget.value.slice(4) : null))
const deviceComp = computed(
  () => componentWaves.value?.components?.find((c) => c.ref === deviceRef.value) ?? null
)
// A component reshaped as an excitation so WavePane renders it unchanged. Its voltage is a real
// simulated node difference (V_DS / V_AK / V_C), so — unlike the magnetics — nothing is analytical.
const deviceExcitation = computed(() => {
  const c = deviceComp.value
  if (!c) return null
  const f = componentWaves.value?.referencePeriod ? 1 / componentWaves.value.referencePeriod : 0
  return { name: `${c.ref} · ${c.voltage?.label ?? 'V'}`, frequency: f, current: c.current, voltage: c.voltage }
})

// ── component stress table (DVT): simulated peak stress vs the part's rating, with margin ──────
// Maps each device kind to the requirement keys that bound it. Caps are voltage-limited (their
// ripple-current spec is a demand ON the part, not a rating); resistors are power-limited.
const RATING = {
  mosfet: { v: 'ratedDrainSourceVoltage', i: 'ratedContinuousDrainCurrent' },
  diode: { v: 'ratedReverseVoltage', i: 'ratedForwardCurrent' },
  capacitor: { v: 'ratedVoltage' },
  resistor: { p: 'ratedPower' },
}
function ratedOf(req, key) {
  const v = req?.[key]
  if (v === null || v === undefined) return null
  return typeof v === 'number' ? v : (v.nominal ?? v.maximum ?? null)
}
// ratio = stress/rating → verdict. Designs derate ~20%, so a healthy part sits ≤0.85; 0.85–1.0 is a
// thin margin (warn); >1.0 exceeds the rating (fail).
function verdict(ratio) {
  if (ratio === null || !Number.isFinite(ratio)) return null
  if (ratio <= 0.85) return 'ok'
  if (ratio <= 1.0) return 'warn'
  return 'bad'
}
const componentStress = computed(() => {
  const comps = componentWaves.value?.components
  if (!comps?.length) return []
  const reqByRef = new Map(bomRows.value.map((r) => [r.ref, r.requirements]))
  const rows = []
  for (const c of comps) {
    const req = reqByRef.get(c.ref) ?? {}
    const map = RATING[c.kind] ?? {}
    const vp = c.voltage?.processed ?? {}
    const ip = c.current?.processed ?? {}
    const row = { ref: c.ref, kind: c.kind }
    // VOLTAGE is limited by the absolute-max rating → compare the peak.
    if (map.v) {
      const vRated = ratedOf(req, map.v)
      const vStress = Math.abs(vp.peak ?? NaN)
      row.v = { stress: vStress, rated: vRated, ratio: vRated ? vStress / vRated : null }
    }
    // CURRENT ratings are NOT peak: a MOSFET's continuous rating is thermal (compare RMS); a diode's
    // forward-current rating is an average (compare the mean). Comparing the instantaneous peak against
    // either would spuriously read "over" on every switching part.
    if (map.i) {
      const iRated = ratedOf(req, map.i)
      const iStress = c.kind === 'mosfet' ? Math.abs(ip.rms ?? NaN) : Math.abs(ip.average ?? NaN)
      const basis = c.kind === 'mosfet' ? 'rms' : 'avg'
      row.i = { stress: iStress, rated: iRated, ratio: iRated ? iStress / iRated : null, unit: 'A', basis }
    }
    // POWER limit (resistor): V and I are in phase across an R, so P_avg = Vrms·Irms.
    if (map.p) {
      const p = (vp.rms ?? 0) * (ip.rms ?? 0)
      const pRated = ratedOf(req, map.p)
      row.i = { stress: p, rated: pRated, ratio: pRated ? p / pRated : null, unit: 'W', basis: 'avg' }
    }
    row.worst = Math.max(row.v?.ratio ?? 0, row.i?.ratio ?? 0)
    row.verdict = verdict(row.worst || null)
    rows.push(row)
  }
  // worst-stressed first — the parts a reviewer checks first
  rows.sort((a, b) => (b.worst ?? 0) - (a.worst ?? 0))
  return rows
})
const stressSummary = computed(() => {
  const s = { ok: 0, warn: 0, bad: 0 }
  for (const r of componentStress.value) if (r.verdict) s[r.verdict]++
  return s
})

async function simulateMagnetic() {
  if (!result.value || !waveMag.value) return
  ngspiceBusy.value = true
  runError.value = null
  try {
    const op = await extractOperatingPoint(result.value.tas, 'ngspice', waveMag.value.name)
    ngspiceOps.value = { ...ngspiceOps.value, [waveMag.value.name]: op }
  } catch (e) {
    runError.value = e.message
  } finally {
    ngspiceBusy.value = false
  }
}

// ── diagnostics ────────────────────────────────────────────────────────────
const diag = computed(() => result.value?.diagnostics ?? null)
const mainExcNames = computed(
  () => (result.value?.operatingPoint?.excitationsPerWinding ?? []).map((e) => e.name)
)

// ── netlist tab ────────────────────────────────────────────────────────────
const deck = ref('')
const deckFlavor = ref('ngspice')
const deckFidelity = ref('REQUIREMENTS')
const simStop = ref(null)   // seconds; null = keep the design default
const simStep = ref(null)
const deckBusy = ref(false)

const designStop = computed(() => result.value?.tas?.simulation?.analyses?.[0]?.stopTime ?? null)
const designStep = computed(() => result.value?.tas?.simulation?.analyses?.[0]?.maximumTimeStep ?? null)
const periodsShown = computed(() => {
  const stop = simStop.value ?? designStop.value
  return stop && form.fs ? Math.round(stop * form.fs) : null
})

async function makeDeck() {
  if (!result.value) return
  deckBusy.value = true
  try {
    const tas = JSON.parse(JSON.stringify(result.value.tas))
    const an = tas.simulation?.analyses?.[0]
    if (an) {
      if (simStop.value) an.stopTime = simStop.value
      if (simStep.value) an.maximumTimeStep = simStep.value
    }
    deck.value = await generateNetlist(tas, deckFlavor.value, { origin: deckFidelity.value })
  } catch (e) {
    deck.value = `* generation failed:\n* ${e.message}`
  } finally {
    deckBusy.value = false
  }
}
function copyDeck() {
  navigator.clipboard?.writeText(deck.value)
}
function downloadDeck() {
  const ext = deckFlavor.value === 'ltspice' ? 'asc.cir' : 'cir'
  const a = document.createElement('a')
  a.href = URL.createObjectURL(new Blob([deck.value], { type: 'text/plain' }))
  a.download = `${topoId.value}.${ext}`
  a.click()
  URL.revokeObjectURL(a.href)
}
function downloadMagneticInputs() {
  if (!waveMag.value) return
  const a = document.createElement('a')
  a.href = URL.createObjectURL(
    new Blob([JSON.stringify(waveMag.value.inputs, null, 2)], { type: 'application/json' })
  )
  a.download = `${topoId.value}_${waveMag.value.name}_mas_inputs.json`
  a.click()
  URL.revokeObjectURL(a.href)
}

const TABS = [
  ['schematic', 'Schematic'],
  ['bom', 'BOM'],
  ['waveforms', 'Waveforms'],
  ['diagnostics', 'Diagnostics'],
  ['netlist', 'Netlist'],
]
</script>

<template>
  <div class="wrap">
    <!-- ── header: drawing-sheet title block ─────────────────────────── -->
    <header class="kh">
      <div class="kh-sheet">
        <div class="kh-brand">
          <svg width="52" height="52" viewBox="0 0 64 64" aria-hidden="true">
            <g stroke="var(--amber)" stroke-width="3.5" stroke-linecap="round" fill="none">
              <path d="M10 32h20" /><path d="M32 34v18" /><path d="M34 30 52 14" />
            </g>
            <circle cx="32" cy="32" r="5" fill="var(--amber)" />
          </svg>
          <div>
            <div class="kh-title">KIRCHHOFF</div>
            <div class="kh-sub">
              power-converter design bench · runs entirely in your browser · <b>Σ I = 0</b>
            </div>
          </div>
        </div>
        <table class="kh-titleblock" aria-hidden="true">
          <tbody>
            <tr>
              <td>ENGINE</td>
              <td class="v">
                <span class="led" :class="engineState === 'ready' ? 'on' : engineState === 'error' ? 'err' : ''"></span>
                <span v-if="engineState === 'ready'">WASM ONLINE</span>
                <span v-else-if="engineState === 'error'">FAILED</span>
                <span v-else>LOADING…</span>
              </td>
              <td>SOLVERS</td>
              <td class="v">ANALYTICAL · NGSPICE</td>
            </tr>
            <tr>
              <td>TOPOLOGIES</td>
              <td class="v">{{ TOPOLOGIES.length }}</td>
              <td>SHEET</td>
              <td class="v">01 / 01</td>
            </tr>
          </tbody>
        </table>
      </div>
    </header>

    <!-- ── 1 · topology ───────────────────────────────────────────────── -->
    <section class="panel">
      <div class="section-label">
        <span class="idx">1</span> Topology
        <span class="hint">{{ topo.name }} selected</span>
      </div>
      <template v-for="g in grouped" :key="g.family">
        <div class="topo-family">{{ g.family }}</div>
        <div class="topo-grid">
          <button
            v-for="t in g.items" :key="t.id"
            class="topo-card"
            :class="{ active: t.id === topoId, planned: t.planned }"
            :disabled="t.planned"
            @click="selectTopology(t.id)"
          >
            <div class="t-name">
              {{ t.name }}
              <span v-if="t.planned" class="t-tag">planned</span>
            </div>
            <div class="t-desc">{{ t.desc }}</div>
          </button>
        </div>
      </template>
    </section>

    <!-- ── 2 · specification ──────────────────────────────────────────── -->
    <section class="panel">
      <div class="section-label"><span class="idx">2</span> Specification</div>

      <div class="grid4">
        <label class="fld" v-if="form.inputType === 'dc'">
          <span class="fld-label">Vin min <span class="u">V</span></span>
          <input class="fld-in" type="number" v-model.number="form.vinMin" placeholder="optional" />
        </label>
        <label class="fld">
          <span class="fld-label">{{ form.inputType === 'dc' ? 'Vin nominal' : 'Vac (rms)' }} <span class="u">V</span></span>
          <input class="fld-in" type="number" v-model.number="form.vinNom" />
        </label>
        <label class="fld" v-if="form.inputType === 'dc'">
          <span class="fld-label">Vin max <span class="u">V</span></span>
          <input class="fld-in" type="number" v-model.number="form.vinMax" placeholder="optional" />
        </label>
        <label class="fld" v-else>
          <span class="fld-label">Line frequency <span class="u">Hz</span></span>
          <input class="fld-in" type="number" v-model.number="form.lineFrequency" />
        </label>
        <label class="fld">
          <span class="fld-label">Switching freq <span class="u">Hz</span></span>
          <input class="fld-in" type="number" v-model.number="form.fs" step="1000" />
        </label>
      </div>

      <div style="margin-top: 0.9rem">
        <table class="row-table">
          <thead>
            <tr>
              <th>Output</th><th>Voltage (V)</th><th>Power (W)</th><th></th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="(o, i) in form.outputs" :key="i">
              <td><input class="fld-in" v-model="o.name" /></td>
              <td><input class="fld-in" type="number" v-model.number="o.voltage" /></td>
              <td><input class="fld-in" type="number" v-model.number="o.power" @change="form.ops.forEach((op) => (op.powers[i] = o.power))" /></td>
              <td>
                <button
                  v-if="form.outputs.length > form.minOutputs"
                  class="row-btn" @click="removeOutput(i)" title="remove output"
                >×</button>
              </td>
            </tr>
          </tbody>
        </table>
        <button v-if="form.outputs.length < form.maxOutputs" class="row-btn" style="margin-top: 0.3rem" @click="addOutput">
          + output
        </button>
        <span v-if="form.minOutputs > 1" class="chip" style="margin-left: 0.6rem">
          this topology needs ≥ {{ form.minOutputs }} outputs
        </span>
      </div>

      <!-- simulation controls: always visible (engine + how long to run / how much to show) -->
      <div class="section-label" style="margin-top: 1.1rem">
        Simulation
        <span class="hint">engine · transient length · window</span>
      </div>
      <div class="grid3">
        <label class="fld">
          <span class="fld-label">Solve engine</span>
          <select class="fld-in" v-model="form.engine">
            <option value="ngspice">ngspice — full transient (seconds)</option>
            <option value="analytical">analytical — instant</option>
          </select>
        </label>
        <label class="fld" v-if="form.inputType === 'dc'">
          <span class="fld-label">Steady-state cycles</span>
          <input
            class="fld-in" type="number" min="1" step="10" v-model.number="form.settlePeriods"
            title="switching cycles the transient runs to settle before the shown window; slow converters (large output caps) may need 400+ to fully converge"
          />
        </label>
        <label class="fld">
          <span class="fld-label">Cycles shown</span>
          <input
            class="fld-in" type="number" min="1" max="50" v-model.number="form.showPeriods"
            title="how many steady-state switching cycles the waveform plots repeat across"
          />
        </label>
      </div>

      <details class="adv">
        <summary>Advanced — efficiency, isolation, operating points</summary>
        <div class="adv-body">
          <div class="grid3">
            <label class="fld">
              <span class="fld-label">Target efficiency <span class="u">0–1</span></span>
              <input class="fld-in" type="number" step="0.01" min="0.5" max="1" v-model.number="form.efficiency" />
            </label>
            <label class="fld">
              <span class="fld-label">Ambient <span class="u">°C</span></span>
              <input class="fld-in" type="number" v-model.number="form.ambient" />
            </label>
            <label class="fld">
              <span class="fld-label">Isolation <span class="u">V</span></span>
              <input class="fld-in" type="number" v-model.number="form.isolation" placeholder="none" />
            </label>
          </div>

          <div style="margin-top: 0.9rem">
            <table class="row-table">
              <thead>
                <tr>
                  <th>Operating point</th><th>Vin (V)</th><th>Ambient (°C)</th>
                  <th v-for="(o, i) in form.outputs" :key="i">{{ o.name }} (W)</th>
                  <th></th>
                </tr>
              </thead>
              <tbody>
                <tr v-for="(op, j) in form.ops" :key="j">
                  <td><input class="fld-in" v-model="op.name" /></td>
                  <td><input class="fld-in" type="number" v-model.number="op.vin" /></td>
                  <td><input class="fld-in" type="number" v-model.number="op.ambient" /></td>
                  <td v-for="(o, i) in form.outputs" :key="i">
                    <input class="fld-in" type="number" v-model.number="op.powers[i]" />
                  </td>
                  <td>
                    <button v-if="form.ops.length > 1" class="row-btn" @click="form.ops.splice(j, 1)">×</button>
                  </td>
                </tr>
              </tbody>
            </table>
            <button class="row-btn" style="margin-top: 0.3rem" @click="addOp">+ operating point</button>
          </div>
        </div>
      </details>

      <div style="margin-top: 1.1rem; display: flex; gap: 0.8rem; align-items: center">
        <button class="btn" :disabled="running || engineState !== 'ready'" @click="solve">
          {{ running ? (form.engine === 'ngspice' ? 'Simulating…' : 'Solving…') : 'Solve' }}
        </button>
        <div v-if="engineState === 'loading'" class="boot">
          <div class="spin"></div> loading the WASM engine (≈14 MB, includes ngspice)…
        </div>
        <span v-else-if="result" class="chip ok">design ready</span>
      </div>

      <div v-if="runError" class="err-banner"><b>ENGINE ▸</b> {{ runError }}</div>
    </section>

    <!-- ── 3 · results ────────────────────────────────────────────────── -->
    <section v-if="result" class="panel">
      <div class="section-label">
        <span class="idx">3</span> Results
        <span class="hint">
          {{ topo.name }} · fsw {{ si(diag?.switchingFrequency, 'Hz') }} ·
          {{ diag?.isCcm ? 'CCM' : 'DCM' }} · duty {{ pct(diag?.dutyCycle) }}
        </span>
      </div>

      <div class="tabs">
        <button
          v-for="[id, label] in TABS" :key="id"
          :class="{ active: activeTab === id }" @click="activeTab = id"
        >{{ label }}</button>
      </div>

      <!-- schematic -->
      <div v-if="activeTab === 'schematic'">
        <template v-if="schematicSvg">
          <div class="schematic-frame" v-html="schematicSvg" @click="schematicClick"></div>
          <div class="sch-caption">
            Power-path sketch — click any component for its requirements, stresses and waveforms.
            Snubbers, balancing networks and the controller are listed in the BOM tab.
          </div>
        </template>
        <div v-else class="wave-empty">
          No schematic sketch for <code>{{ topo.name }}</code> yet — the BOM tab has every
          component with its full requirement set. Schematic coverage is growing topology by topology.
        </div>
      </div>

      <!-- BOM -->
      <div v-if="activeTab === 'bom'">
        <table class="data-table">
          <thead>
            <tr>
              <th>Ref</th><th>Kind</th><th>Stage</th><th>Designed value</th><th>Rating req.</th>
            </tr>
          </thead>
          <tbody>
            <tr
              v-for="r in bomRows" :key="r.ref" class="clickable"
              :class="{ selected: selectedPart?.ref === r.ref }"
              @click="openPart(r.ref)"
            >
              <td style="color: var(--amber-hi)">{{ r.ref }}</td>
              <td>{{ r.kind }}</td>
              <td class="dim">{{ r.stage }}</td>
              <td>{{ r.value }}</td>
              <td class="dim">{{ r.rating }}</td>
            </tr>
          </tbody>
        </table>
      </div>

      <!-- waveforms -->
      <div v-if="activeTab === 'waveforms'">
        <div class="wave-toolbar">
          <!-- unified picker: magnetics (per-winding) + every device (per-component V/I) -->
          <select class="fld-in" style="width: auto" v-model="waveTarget">
            <optgroup label="Magnetics">
              <option v-for="(m, i) in waveMagnetics" :key="'m' + i" :value="`mag:${i}`">
                {{ m.name }}{{ m.isMain ? ' (main magnetic)' : '' }}
              </option>
            </optgroup>
            <optgroup v-for="g in deviceGroups" :key="g.label" :label="g.label">
              <option v-for="r in g.items" :key="r.ref" :value="`dev:${r.ref}`">{{ r.ref }} — {{ r.value }}</option>
            </optgroup>
          </select>
          <select
            v-if="targetIsMagnetic && waveOps.length > 1"
            class="fld-in" style="width: auto" v-model.number="waveOpIdx"
          >
            <option v-for="(op, i) in waveOps" :key="i" :value="i">{{ op.name ?? `OP ${i + 1}` }}</option>
          </select>
          <span v-if="targetIsMagnetic" class="chip" :class="waveSource.kind === 'ngspice' ? 'cyan' : 'amber'">
            {{ waveSource.kind }}
          </span>
          <span v-else class="chip cyan">ngspice</span>
          <div class="sep"></div>
          <button
            v-if="targetIsMagnetic && !ngspiceOps[waveMag?.name]"
            class="btn ghost" :disabled="ngspiceBusy" @click="simulateMagnetic"
            title="run the full ngspice transient for this magnetic, in-browser"
          >
            {{ ngspiceBusy ? 'Simulating…' : '▶ ngspice this magnetic' }}
          </button>
          <button v-if="targetIsMagnetic" class="btn ghost" @click="downloadMagneticInputs">
            ⭳ MAS inputs (for MagneticAdviser)
          </button>
        </div>

        <!-- magnetic: one pane per winding (as before) -->
        <template v-if="targetIsMagnetic">
          <div v-for="(exc, i) in waveExcitations" :key="i" style="margin-bottom: 1.2rem">
            <div class="mono" style="font-size: 0.74rem; color: var(--amber-hi); margin-bottom: 0.35rem">
              ▸ {{ exc.name ?? `winding ${i}` }}
              <span class="chip" style="margin-left: 0.5rem">{{ si(exc.frequency, 'Hz') }}</span>
            </div>
            <WavePane :excitation="exc" :source-kind="waveSource.kind" :periods="form.showPeriods" />
          </div>
          <div class="wave-readout">
            <span class="i"><b>—</b> current (left axis){{ waveSource.kind === 'ngspice' ? ' · measured' : '' }}</span>
            <span class="v"><b>—</b> voltage (right axis){{ waveSource.voltageKind === 'analytical' ? ' · analytical' : '' }}</span>
          </div>
        </template>

        <!-- device: one pane (V across terminals + terminal current), all measured by ngspice -->
        <template v-else>
          <template v-if="deviceExcitation">
            <div class="mono" style="font-size: 0.74rem; color: var(--amber-hi); margin-bottom: 0.35rem">
              ▸ {{ deviceExcitation.name }}
              <span class="chip" style="margin-left: 0.5rem">{{ si(deviceExcitation.frequency, 'Hz') }}</span>
            </div>
            <WavePane :excitation="deviceExcitation" source-kind="ngspice" :periods="form.showPeriods" />
            <div class="wave-readout">
              <span class="i"><b>—</b> terminal current · measured</span>
              <span class="v"><b>—</b> {{ deviceComp?.voltage?.label ?? 'voltage' }} · measured</span>
            </div>
          </template>
          <div v-else-if="componentBusy" class="boot" style="margin-top: 0.6rem">
            <div class="spin"></div> simulating all components (one ngspice run)…
          </div>
          <div v-else class="wave-empty">
            Component waveforms come from ngspice. Solve with the ngspice engine, or
            <button class="btn ghost" :disabled="componentBusy" @click="fetchComponentWaves" style="margin-left: 0.3rem">
              ▶ run ngspice for all components
            </button>
          </div>
        </template>
      </div>

      <!-- diagnostics -->
      <div v-if="activeTab === 'diagnostics'">
        <div class="stat-row">
          <div class="stat"><div class="k">switching freq</div><div class="n">{{ si(diag.switchingFrequency, 'Hz') }}</div></div>
          <div class="stat"><div class="k">duty cycle</div><div class="n">{{ pct(diag.dutyCycle) }}</div></div>
          <div class="stat"><div class="k">conduction</div><div class="n">{{ diag.isCcm ? 'CCM' : 'DCM' }}</div></div>
          <div class="stat i"><div class="k">primary Ipk</div><div class="n">{{ si(diag.primaryPeakCurrent, 'A') }}</div></div>
          <div class="stat i"><div class="k">primary Irms</div><div class="n">{{ si(diag.primaryRmsCurrent, 'A') }}</div></div>
          <div class="stat"><div class="k">Lm (main)</div><div class="n">{{ si(diag.computed?.magnetizingInductance, 'H') }}</div></div>
          <div class="stat" v-if="diag.computed?.turnsRatio"><div class="k">turns ratio</div><div class="n">{{ diag.computed.turnsRatio.toPrecision(4) }}</div></div>
          <div class="stat" v-if="diag.computed?.resonantCapacitance"><div class="k">Cr</div><div class="n">{{ si(diag.computed.resonantCapacitance, 'F') }}</div></div>
        </div>

        <!-- component stress (DVT): simulated peak vs rating, worst-stressed first -->
        <div class="section-label" style="margin-top: 1.4rem">
          Component stress
          <span class="hint" v-if="componentStress.length">
            <span class="chip ok">{{ stressSummary.ok }} ok</span>
            <span v-if="stressSummary.warn" class="chip" style="margin-left: 0.35rem">{{ stressSummary.warn }} tight</span>
            <span v-if="stressSummary.bad" class="chip bad" style="margin-left: 0.35rem">{{ stressSummary.bad }} over</span>
          </span>
          <span class="hint" v-else>simulated stress vs rating · needs an ngspice run</span>
        </div>
        <div v-if="componentStress.length" class="sch-caption" style="margin: -0.3rem 0 0.5rem">
          voltage: peak vs absolute-max rating · current: MOSFET RMS / diode average vs its rating ·
          margin = headroom before the rating (derating target ≈ 20%).
        </div>
        <table v-if="componentStress.length" class="data-table">
          <thead>
            <tr>
              <th>Ref</th><th>Kind</th>
              <th class="num">V peak</th><th class="num">V rated</th><th class="num">margin</th>
              <th class="num">I / P</th><th class="num">rated</th><th class="num">margin</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="s in componentStress" :key="s.ref" class="clickable" @click="openPart(s.ref)">
              <td style="color: var(--amber-hi)">{{ s.ref }}</td>
              <td class="dim">{{ s.kind }}</td>
              <td class="num">{{ s.v ? si(s.v.stress, 'V') : '—' }}</td>
              <td class="num dim">{{ s.v?.rated ? si(s.v.rated, 'V') : '—' }}</td>
              <td class="num">
                <span v-if="s.v?.ratio != null" class="chip" :class="verdict(s.v.ratio)">{{ pct(1 - s.v.ratio, 0) }}</span>
                <span v-else class="dim">—</span>
              </td>
              <td class="num">
                {{ s.i ? si(s.i.stress, s.i.unit) : '—' }}<span v-if="s.i?.basis" class="dim" style="font-size: 0.85em"> {{ s.i.basis }}</span>
              </td>
              <td class="num dim">{{ s.i?.rated ? si(s.i.rated, s.i.unit) : '—' }}</td>
              <td class="num">
                <span v-if="s.i?.ratio != null" class="chip" :class="verdict(s.i.ratio)">{{ pct(1 - s.i.ratio, 0) }}</span>
                <span v-else class="dim">—</span>
              </td>
              <td>
                <span v-if="s.verdict" class="chip" :class="s.verdict">
                  {{ s.verdict === 'ok' ? 'pass' : s.verdict === 'warn' ? 'tight' : 'over' }}
                </span>
              </td>
            </tr>
          </tbody>
        </table>
        <div v-else class="wave-empty">
          Per-component voltage/current stress vs each part's rating, with derating margin — from the
          ngspice run. Solve with the ngspice engine, or
          <button class="btn ghost" :disabled="componentBusy" @click="fetchComponentWaves" style="margin-left: 0.3rem">
            ▶ run ngspice for the stress table
          </button>
        </div>

        <div class="section-label" style="margin-top: 1.2rem">Magnetics</div>
        <table class="data-table">
          <thead><tr><th>Name</th><th>Windings</th><th>Lm</th><th>Turns ratios</th><th></th></tr></thead>
          <tbody>
            <tr v-for="m in diag.magnetics" :key="m.name" class="clickable" @click="openPart(m.name)">
              <td style="color: var(--amber-hi)">{{ m.name }}</td>
              <td>{{ m.windings }}</td>
              <td>{{ si(m.magnetizingInductance, 'H') }}</td>
              <td class="dim">{{ m.turnsRatios?.map((r) => r.toPrecision(4)).join(' / ') || '—' }}</td>
              <td><span v-if="m.isMain" class="chip amber">main</span></td>
            </tr>
          </tbody>
        </table>

        <div class="section-label" style="margin-top: 1.2rem">Capacitors</div>
        <table class="data-table">
          <thead><tr><th>Name</th><th>Capacitance</th><th>Rated voltage</th><th>Role</th></tr></thead>
          <tbody>
            <tr v-for="c in diag.capacitors" :key="c.name" class="clickable" @click="openPart(c.name)">
              <td style="color: var(--amber-hi)">{{ c.name }}</td>
              <td>{{ si(c.capacitance, 'F') }}</td>
              <td>{{ si(c.ratedVoltage, 'V') }}</td>
              <td class="dim">{{ c.role }}</td>
            </tr>
          </tbody>
        </table>

        <template v-for="(op, i) in diag.operatingPoints" :key="i">
          <div class="section-label" style="margin-top: 1.2rem">
            {{ op.operatingPointName ?? form.ops[i]?.name ?? `Operating point ${i + 1}` }}
            <span class="hint">
              <span class="chip" :class="op.isCcm ? 'ok' : 'cyan'">{{ op.isCcm ? 'CCM' : 'DCM' }}</span>
            </span>
          </div>
          <table class="data-table">
            <thead>
              <tr>
                <th>Winding (main magnetic)</th><th class="num">f</th>
                <th class="num">I pk</th><th class="num">I rms</th><th class="num">I pk-pk</th>
                <th class="num">V pk</th><th class="num">V rms</th><th class="num">V pk-pk</th>
                <th class="num">duty</th>
              </tr>
            </thead>
            <tbody>
              <tr v-for="(w, k) in op.windings" :key="k">
                <td>{{ mainExcNames[k] ?? `winding ${k}` }}</td>
                <td class="num dim">{{ si(w.frequency, 'Hz') }}</td>
                <td class="num" style="color: var(--amber)">{{ si(w.current_peak, 'A') }}</td>
                <td class="num">{{ si(w.current_rms, 'A') }}</td>
                <td class="num dim">{{ si(w.current_peakToPeak, 'A') }}</td>
                <td class="num" style="color: var(--cyan)">{{ si(w.voltage_peak, 'V') }}</td>
                <td class="num">{{ si(w.voltage_rms, 'V') }}</td>
                <td class="num dim">{{ si(w.voltage_peakToPeak, 'V') }}</td>
                <td class="num dim">{{ pct(w.current_dutyCycle) }}</td>
              </tr>
            </tbody>
          </table>
        </template>
      </div>

      <!-- netlist -->
      <div v-if="activeTab === 'netlist'">
        <div class="wave-toolbar">
          <select class="fld-in" style="width: auto" v-model="deckFlavor">
            <option value="ngspice">ngspice deck</option>
            <option value="ltspice">LTspice deck</option>
          </select>
          <select class="fld-in" style="width: auto" v-model="deckFidelity" title="component model fidelity">
            <option value="REQUIREMENTS">ideal (requirements)</option>
            <option value="DATASHEET">datasheet models</option>
            <option value="MKF_MODEL">MKF models</option>
          </select>
          <label class="fld" style="max-width: 150px">
            <span class="fld-label">stop time <span class="u">s</span></span>
            <input class="fld-in" type="number" step="0.0005" v-model.number="simStop" :placeholder="designStop != null ? String(designStop) : ''" />
          </label>
          <label class="fld" style="max-width: 150px">
            <span class="fld-label">max step <span class="u">s</span></span>
            <input class="fld-in" type="number" step="1e-8" v-model.number="simStep" :placeholder="designStep != null ? String(designStep) : ''" />
          </label>
          <span v-if="periodsShown" class="chip">≈ {{ periodsShown }} switching periods</span>
          <div class="sep"></div>
          <button class="btn ghost" :disabled="deckBusy" @click="makeDeck">Generate</button>
          <button class="btn ghost" :disabled="!deck" @click="copyDeck">Copy</button>
          <button class="btn ghost" :disabled="!deck" @click="downloadDeck">⭳ Download</button>
        </div>
        <pre v-if="deck" class="deck">{{ deck }}</pre>
        <div v-else class="wave-empty">
          Generate a runnable SPICE netlist of this exact design — the same circuit the in-browser
          ngspice engine simulates. Download it to reproduce or extend the simulation on your
          desktop (<code>ngspice {{ topoId }}.cir</code>) or in LTspice.
        </div>
      </div>
    </section>

    <div class="footnote">
      KIRCHHOFF · OpenConverters — every solve runs locally in WebAssembly; your design never
      leaves this page. Sibling instrument of HEAVISIDE.
    </div>

    <PartDrawer
      :part="selectedPart"
      :device-wave="selectedPartWave"
      :periods="form.showPeriods"
      @close="selectedPart = null"
    />
  </div>
</template>
