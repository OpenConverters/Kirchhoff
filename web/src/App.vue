<script setup>
import { computed, onMounted, onUnmounted, provide, reactive, ref, watch } from 'vue'
import { FAMILIES, FAMILY_SHORT, PLANNED, TOPOLOGIES, buildSpec, topologyById } from './topologies.js'
import { loadEngine, processConverter, topologyWaveforms, extractOperatingPoint, componentWaveforms, realizeTas, generateNetlist } from './kh.js'
import { extractBom } from './bom.js'
import { hasSchematic, renderSchematic } from './schematics.js'
import { si, pct } from './units.js'
import PartDrawer from './components/PartDrawer.vue'
import FamilyDial from './components/FamilyDial.vue'
import OutputPane from './components/OutputPane.vue'

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
// models: 'ideal' switches, or 'datasheet' — real-conduction semiconductor models
// derived from the design requirements (real Rds(on) / forward drop).
const form = reactive({ engine: 'ngspice', settlePeriods: 100, showPeriods: 2, models: 'ideal' })

function selectTopology(id) {
  topoId.value = id
  const t = topologyById(id)
  if (t) family.value = t.family   // keep the dial in sync when a converter is picked directly
  const p = t.preset
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
  realizedTas.value = null
  selectedPart.value = null
}
const topo = computed(() => topologyById(topoId.value))

// The rotary dial selects a family; the topology list shows only that family's converters.
const family = ref(topo.value?.family ?? FAMILIES[0])
const familyTopologies = computed(() => [
  ...TOPOLOGIES.filter((t) => t.family === family.value),
  ...PLANNED.filter((t) => t.family === family.value).map((t) => ({ ...t, planned: true })),
])
// Turning the dial to a new family auto-selects that family's first converter (unless the current one
// already belongs to it — e.g. on first mount).
watch(family, (f) => {
  if (topo.value?.family === f) return
  const first = TOPOLOGIES.find((t) => t.family === f)
  if (first) selectTopology(first.id)
})

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
// the two output panes default to schematic (left) + waveforms (right)
const paneA = ref('schematic')
const paneB = ref('waveforms')
const ngspiceOps = ref({})        // magnetic name -> simulated MAS::OperatingPoint
const ngspiceBusy = ref(false)
const componentWaves = ref(null)  // { referencePeriod, components:[{ref,kind,voltage,current}] } | null
const componentBusy = ref(false)
const realizedTas = ref(null)     // datasheet-realized TAS (real-conduction models), when models==='datasheet'
// Every ngspice re-sim (components, per-magnetic, netlist) runs against this: the realized TAS when the
// user picked datasheet models, else the plain design. A realized part renders real regardless of the
// deck fidelity (infer_fidelity keys off the per-component binding), so the fidelity arg stays default.
const simTas = computed(() => realizedTas.value ?? result.value?.tas ?? null)

async function solve() {
  running.value = true
  runError.value = null
  result.value = null
  deck.value = ''
  ngspiceOps.value = {}
  componentWaves.value = null
  realizedTas.value = null
  try {
    const spec = buildSpec(form)
    const res = await processConverter(topoId.value, spec, form.engine)
    result.value = res
    // datasheet models: realize the design once so every ngspice re-sim uses real-conduction parts
    if (form.models === 'datasheet') realizedTas.value = await realizeTas(res.tas)
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
    // a topology without a schematic sketch: show the BOM in the left pane instead
    if (!hasSchematic(topoId.value) && paneA.value === 'schematic') paneA.value = 'bom'
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
    const cw = await componentWaveforms(simTas.value)
    if (gen !== cwGen) return                // superseded by a newer solve — discard
    componentWaves.value = cw?.success === false ? null : cw
  } catch (e) {
    if (gen === cwGen) runError.value = e.message
  } finally {
    if (gen === cwGen) componentBusy.value = false
  }
}

// Switching component models between ideal/datasheet after a solve: re-realize (or drop) and re-run the
// component sim + any per-magnetic sims so every ngspice view reflects the chosen model set.
watch(() => form.models, async (m) => {
  if (!result.value) return
  realizedTas.value = m === 'datasheet' ? await realizeTas(result.value.tas) : null
  ngspiceOps.value = {}          // per-magnetic sims were run against the other model set
  componentWaves.value = null
  fetchComponentWaves()
})

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
    const op = await extractOperatingPoint(simTas.value, 'ngspice', waveMag.value.name)
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
    const tas = JSON.parse(JSON.stringify(simTas.value))
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

// Everything the two OutputPanes render is shared through this context (they are pure views).
provide('kh', {
  result, topo, diag, bomRows, selectedPart, schematicSvg, schematicClick, openPart,
  waveTarget, waveMagnetics, deviceGroups, targetIsMagnetic, waveOps, waveOpIdx, waveSource,
  waveExcitations, waveMag, ngspiceOps, ngspiceBusy, simulateMagnetic, downloadMagneticInputs,
  deviceExcitation, deviceComp, componentBusy, fetchComponentWaves,
  componentStress, stressSummary, verdict, mainExcNames, form,
  deck, deckFlavor, deckFidelity, simStop, simStep, deckBusy, designStop, designStep, periodsShown,
  makeDeck, copyDeck, downloadDeck, si, pct,
})
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
              power-converter design bench · nothing leaves the bench · <b>Σ I = 0</b>
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

    <!-- ── workbench: controls sidebar + two output panes ─────────────────── -->
    <div class="workbench">
      <!-- left: topology dial + list, spec, simulation, solve -->
      <aside class="controls panel">
        <!-- pinned: the family dial + the Solve button stay on top while the rest of the form scrolls -->
        <div class="topo-head">
          <div class="section-label"><span class="idx">1</span> Topology <span class="hint">{{ topo.name }}</span></div>
          <div class="dial-solve">
            <FamilyDial v-model="family" :families="FAMILIES" :short="FAMILY_SHORT" />
            <div class="solve-col">
              <button class="btn solve-btn" :disabled="running || engineState !== 'ready'" @click="solve">
                {{ running ? (form.engine === 'ngspice' ? 'Simulating…' : 'Solving…') : 'Solve' }}
              </button>
              <div v-if="engineState === 'loading'" class="boot"><div class="spin"></div> loading…</div>
              <span v-else-if="result" class="chip ok">ready</span>
              <span v-else class="hint" style="font-size: 0.62rem">set spec ▸ solve</span>
            </div>
          </div>
          <div v-if="runError" class="err-banner" style="margin-top: 0.5rem"><b>ENGINE ▸</b> {{ runError }}</div>
        </div>
        <div class="topo-list">
          <button
            v-for="t in familyTopologies" :key="t.id"
            class="topo-card" :class="{ active: t.id === topoId, planned: t.planned }"
            :disabled="t.planned" @click="selectTopology(t.id)"
          >
            <div class="t-name">{{ t.name }}<span v-if="t.planned" class="t-tag">planned</span></div>
            <div class="t-desc">{{ t.desc }}</div>
          </button>
        </div>

        <div class="section-label" style="margin-top: 1rem"><span class="idx">2</span> Specification</div>
        <div class="grid2">
          <label class="fld" v-if="form.inputType === 'dc'">
            <span class="fld-label">Vin min <span class="u">V</span></span>
            <input class="fld-in" type="number" v-model.number="form.vinMin" placeholder="opt" />
          </label>
          <label class="fld">
            <span class="fld-label">{{ form.inputType === 'dc' ? 'Vin nom' : 'Vac rms' }} <span class="u">V</span></span>
            <input class="fld-in" type="number" v-model.number="form.vinNom" />
          </label>
          <label class="fld" v-if="form.inputType === 'dc'">
            <span class="fld-label">Vin max <span class="u">V</span></span>
            <input class="fld-in" type="number" v-model.number="form.vinMax" placeholder="opt" />
          </label>
          <label class="fld" v-else>
            <span class="fld-label">Line freq <span class="u">Hz</span></span>
            <input class="fld-in" type="number" v-model.number="form.lineFrequency" />
          </label>
          <label class="fld">
            <span class="fld-label">Switching <span class="u">Hz</span></span>
            <input class="fld-in" type="number" v-model.number="form.fs" step="1000" />
          </label>
        </div>

        <table class="row-table" style="margin-top: 0.6rem">
          <thead><tr><th>Out</th><th>V</th><th>W</th><th></th></tr></thead>
          <tbody>
            <tr v-for="(o, i) in form.outputs" :key="i">
              <td><input class="fld-in" v-model="o.name" /></td>
              <td><input class="fld-in" type="number" v-model.number="o.voltage" /></td>
              <td><input class="fld-in" type="number" v-model.number="o.power" @change="form.ops.forEach((op) => (op.powers[i] = o.power))" /></td>
              <td><button v-if="form.outputs.length > form.minOutputs" class="row-btn" @click="removeOutput(i)">×</button></td>
            </tr>
          </tbody>
        </table>
        <button v-if="form.outputs.length < form.maxOutputs" class="row-btn" style="margin-top: 0.3rem" @click="addOutput">+ output</button>
        <span v-if="form.minOutputs > 1" class="chip" style="margin-left: 0.5rem">needs ≥ {{ form.minOutputs }}</span>

        <div class="section-label" style="margin-top: 1rem">Simulation</div>
        <div class="grid2">
          <label class="fld">
            <span class="fld-label">Engine</span>
            <select class="fld-in" v-model="form.engine">
              <option value="ngspice">ngspice</option>
              <option value="analytical">analytical</option>
            </select>
          </label>
          <label class="fld">
            <span class="fld-label">Models</span>
            <select class="fld-in" v-model="form.models"
                    title="ideal switches, or datasheet-derived real-conduction models (real Rds(on) / forward drop)">
              <option value="ideal">ideal</option>
              <option value="datasheet">datasheet</option>
              <option value="mkf" disabled>MKF magnetics</option>
            </select>
          </label>
          <label class="fld" v-if="form.inputType === 'dc'">
            <span class="fld-label">Settle cyc</span>
            <input class="fld-in" type="number" min="1" step="10" v-model.number="form.settlePeriods"
                   title="switching cycles the transient settles before the shown window" />
          </label>
          <label class="fld">
            <span class="fld-label">Cyc shown</span>
            <input class="fld-in" type="number" min="1" max="50" v-model.number="form.showPeriods" />
          </label>
        </div>

        <details class="adv">
          <summary>Advanced — efficiency, isolation, operating points</summary>
          <div class="adv-body">
            <div class="grid2">
              <label class="fld">
                <span class="fld-label">Efficiency <span class="u">0–1</span></span>
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
            <table class="row-table" style="margin-top: 0.6rem">
              <thead>
                <tr><th>OP</th><th>Vin</th><th>°C</th><th v-for="(o, i) in form.outputs" :key="i">{{ o.name }} W</th><th></th></tr>
              </thead>
              <tbody>
                <tr v-for="(op, j) in form.ops" :key="j">
                  <td><input class="fld-in" v-model="op.name" /></td>
                  <td><input class="fld-in" type="number" v-model.number="op.vin" /></td>
                  <td><input class="fld-in" type="number" v-model.number="op.ambient" /></td>
                  <td v-for="(o, i) in form.outputs" :key="i"><input class="fld-in" type="number" v-model.number="op.powers[i]" /></td>
                  <td><button v-if="form.ops.length > 1" class="row-btn" @click="form.ops.splice(j, 1)">×</button></td>
                </tr>
              </tbody>
            </table>
            <button class="row-btn" style="margin-top: 0.3rem" @click="addOp">+ operating point</button>
          </div>
        </details>

      </aside>

      <!-- right: two selectable output panes -->
      <main class="workspace">
        <div v-if="!result" class="workspace-empty">
          <div>
            <div class="ws-title">{{ topo.name }}</div>
            <div class="ws-sub">{{ topo.desc }}</div>
            <div class="ws-hint">Set the spec and press <b>Solve</b> — the schematic, waveforms,
              BOM, diagnostics and netlist appear here, in two panes you can switch independently.</div>
          </div>
        </div>
        <div v-else class="panes">
          <div class="results-strip">
            {{ topo.name }} · fsw {{ si(diag?.switchingFrequency, 'Hz') }} ·
            {{ diag?.isCcm ? 'CCM' : 'DCM' }} · duty {{ pct(diag?.dutyCycle) }}
            <span v-if="form.models === 'datasheet'" class="chip cyan" style="margin-left: 0.5rem">datasheet models</span>
          </div>
          <div class="pane-grid">
            <OutputPane v-model:view="paneA" />
            <OutputPane v-model:view="paneB" />
          </div>
        </div>
      </main>
    </div>

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
