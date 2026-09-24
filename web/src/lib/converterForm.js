// The converter-form state machine behind <KhConverterForm>: which topology/variant is picked, the spec
// form, the three-stage fold, engine boot, and the solve status. It is a composable rather than
// component-local state so a host that needs to READ the form (the Kirchhoff app's header, results
// strip, exports and test hook all do) can own it and hand it to the component as `state`; a host that
// only wants a result lets the component create its own.
import { computed, ref, reactive, watch } from 'vue'
import { FAMILIES, TOPOLOGIES, PLANNED, buildSpec, topologyById, variantAxis, defaultVariant, knobsFor, knobGroups } from '../topologies.js'
import { si } from '../units.js'
import { useKirchhoff } from './context.js'

// engine:   a createKirchhoff() instance (default: the injected one).
// topology: the topology id the form opens on.
// onReset:  called whenever a (re)selected topology resets the form — the host clears whatever it
//           derived from the previous design there.
export function useConverterForm({ engine = null, topology = 'flyback', onReset = null } = {}) {
  const kh = engine ?? useKirchhoff()
  if (!topologyById(topology)) throw new Error(`useConverterForm: unknown topology '${topology}'`)

  // ── engine boot ──────────────────────────────────────────────────────────
  const engineState = ref('loading')
  const running = ref(false)
  const runError = ref(null)
  const result = ref(null)
  const lastSpec = ref(null)   // the exact spec JSON last sent to the engine (bench/test read-back)
  let booted = false
  async function bootEngine() {
    if (booted) return
    booted = true
    try {
      await kh.loadEngine()
      engineState.value = 'ready'
    } catch (e) {
      engineState.value = 'error'
      runError.value = `WASM engine failed to load: ${e.message}`
    }
  }

  // ── topology & spec form ─────────────────────────────────────────────────
  const topoId = ref(topology)
  // The bench needs a working default topology (flyback) so the spec form is populated, but the card
  // grid should NOT look pre-committed on load — no card is highlighted until the user actually picks
  // one. `topoTouched` flips on the first explicit card click (see pickTopology).
  const topoTouched = ref(false)
  // Same for the variant cards: a default variant is set so the design is valid, but no variant card
  // is highlighted until the user actually picks one (reset whenever the topology changes).
  const variantTouched = ref(false)
  // ngspice by default: the real transient is the reference; analytical stays one
  // click away for instant iteration. 100 periods settle the converter, 2 shown.
  // models: 'ideal' switches, or 'datasheet' — real-conduction semiconductor models
  // derived from the design requirements (real Rds(on) / forward drop).
  const form = reactive({ engine: 'ngspice', settlePeriods: 100, showPeriods: 2, models: 'ideal' })

  function selectTopology(id, { initial = false } = {}) {
    topoId.value = id
    const t = topologyById(id)
    if (t) family.value = t.family   // keep the dial in sync when a converter is picked directly
    form.variant = defaultVariant(id)
    const p = t.preset
    Object.assign(form, {
      inputType: p.inputType,
      vinMin: p.vinMin, vinNom: p.vinNom, vinMax: p.vinMax,
      fs: p.fs, efficiency: p.efficiency, ambient: p.ambient,
      isolation: p.isolation, lineFrequency: p.lineFrequency,
      minOutputs: p.minOutputs, maxOutputs: p.maxOutputs,
      outputs: p.outputs.map((o) => ({ ...o })),
      ops: [{ name: 'full_load', vin: p.vinNom, ambient: p.ambient, powers: p.outputs.map((o) => o.power) }],
      // Advanced knobs: fresh per topology (no leak across a switch), seeded off with the
      // C++ default as the starting value so toggling override on gives something to edit.
      knobs: Object.fromEntries(knobsFor(id).map((k) => [k.key, { on: false, value: k.def }])),
    })
    result.value = null
    runError.value = null
    if (!initial && onReset) onReset()
  }
  const topo = computed(() => topologyById(topoId.value))

  // The rotary dial selects a family; the topology list shows only that family's converters.
  const family = ref(topo.value?.family ?? FAMILIES[0])
  const familyTopologies = computed(() => [
    ...TOPOLOGIES.filter((t) => t.family === family.value),
    ...PLANNED.filter((t) => t.family === family.value).map((t) => ({ ...t, planned: true })),
  ])
  // Header count: topologies AND their variants (e.g. flyback's 4 conduction modes count as 4;
  // a topology with no variant axis counts as 1 via the STANDARD single-option axis).
  const topologyVariantCount = computed(() =>
    TOPOLOGIES.reduce((sum, t) => sum + variantAxis(t.id).options.length, 0),
  )
  // Turning the dial to a new family auto-selects that family's first converter (unless the current one
  // already belongs to it — e.g. on first mount).
  watch(family, (f) => {
    if (topo.value?.family === f) return
    const first = TOPOLOGIES.find((t) => t.family === f)
    if (first) selectTopology(first.id)
  })

  // ── the three-stage control flow: Topology ▸ Variant ▸ Spec ────────────────
  // One stage is open at a time; picking in a stage collapses it and opens the next,
  // and any stage header re-opens that stage (Fallout-terminal fold, see style.css).
  const stage = ref('topology')          // 'topology' | 'variant' | 'spec'
  const axis = computed(() => variantAxis(topoId.value))
  const variantOptions = computed(() => axis.value.options)
  const currentVariant = computed(() => variantOptions.value.find((o) => o.id === form.variant) ?? variantOptions.value[0])
  // A single-option axis (no real variant) is trivially "chosen" — the header reads Standard.
  const hasVariantChoice = computed(() => variantOptions.value.length > 1)

  // Advanced per-topology knobs, grouped into tiers for the "Topology knobs" fold.
  const advGroups = computed(() => knobGroups(topoId.value))
  const hasKnobs = computed(() => knobsFor(topoId.value).length > 0)
  // The greyed placeholder shown while a knob is on auto — the C++ builder's own default.
  function knobPlaceholder(k) {
    if (k.def === null || k.def === undefined) return 'auto'
    if (k.type === 'bool') return k.def ? 'on' : 'off'
    if (k.type === 'enum') return k.options.find((o) => o.id === k.def)?.name ?? String(k.def)
    if (k.unit === 'H' || k.unit === 'F' || k.unit === 'Hz') return si(k.def, k.unit)
    return String(k.def)
  }

  function pickTopology(id) {
    topoTouched.value = true
    variantTouched.value = false   // new topology → its variants start unselected
    selectTopology(id)
    kh.track('topology_select', { target: id, name: topologyById(id)?.name, family: family.value })
    // a topology with a single canonical build has nothing to choose — skip straight to the spec
    stage.value = hasVariantChoice.value ? 'variant' : 'spec'
  }
  function pickVariant(id) {
    variantTouched.value = true
    form.variant = id
    kh.track('variant_select', { target: id, topology: topoId.value })
    stage.value = 'spec'
  }

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

  // The plain solve: one engine call, the converter result. A host with a longer pipeline (the
  // Kirchhoff app also realizes models, extracts waveforms, builds the BOM …) passes its own `solver`
  // to <KhConverterForm> and drives these same refs from it.
  async function solve() {
    running.value = true
    runError.value = null
    result.value = null
    try {
      const spec = buildSpec(form, topoId.value)
      lastSpec.value = spec
      result.value = await kh.processConverter(topoId.value, spec, form.engine)
    } catch (e) {
      runError.value = e.message
    } finally {
      running.value = false
    }
    return result.value
  }

  selectTopology(topoId.value, { initial: true })

  return {
    engine: kh,
    engineState, bootEngine, running, runError, result, lastSpec,
    topoId, topo, topoTouched, variantTouched, form, family, familyTopologies, topologyVariantCount,
    stage, axis, variantOptions, currentVariant, hasVariantChoice, advGroups, hasKnobs, knobPlaceholder,
    selectTopology, pickTopology, pickVariant, addOutput, removeOutput, addOp, solve,
    previewSpec: () => buildSpec(form, topoId.value),
  }
}
