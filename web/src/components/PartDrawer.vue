<script setup>
import { computed, ref, watch } from 'vue'
import { requirementRows } from '../bom.js'
import { selectCandidates, kelvinCategoryFor, bindPart, mainMagneticInputs, enrichMagneticWaveforms, designMagneticInOpenMagnetics } from '../kh.js'
import WavePane from './WavePane.vue'

const props = defineProps({
  part: { type: Object, default: null },       // a BOM row
  tas: { type: Object, default: null },         // the current design TAS (for binding a chosen part)
  deviceWave: { type: Object, default: null }, // simulated V/I excitation for a non-magnetic device
  periods: { type: Number, default: 1 },
  context: { type: Object, default: () => ({}) }, // { topology, inputVoltage, switchingFrequency }
})
const emit = defineEmits(['close', 'bound', 'bound-magnetic'])

const rows = computed(() => requirementRows(props.part?.requirements))

// Magnetics aren't catalog-matched — they're custom-designed. The drawer offers a handoff to the
// OpenMagnetics adviser instead of a candidate table.
const isMagnetic = computed(() => props.part?.kind === 'Inductor' || props.part?.kind === 'Transformer')

// ── Kelvin candidate sourcing ──────────────────────────────────────────────
const state = ref('idle')       // idle | loading | ok | empty | error | unsupported
const result = ref(null)        // Kelvin SelectionResult
const errMsg = ref('')

// Optional single-manufacturer restriction. When set, Kelvin selects ONLY over this vendor
// (manufacturerAllowlist) and the per-vendor diversity cap is lifted (maxManufacturerFraction:1) so the
// whole result can come from it. Empty ⇒ the default cross-vendor selection with the diversity cap.
const onlyMfr = ref('')

const category = computed(() => (props.part ? kelvinCategoryFor(props.part.kind) : null))
// Distinct vendors offered in the restrict-to-one-manufacturer dropdown. Captured from the FIRST
// (unrestricted) search and kept across restricted re-queries, so you can switch between vendors even
// though a restricted result only contains one. Empty until the first search runs.
const manufacturers = ref([])

async function findParts() {
  if (!props.part || !category.value) { state.value = 'unsupported'; return }
  state.value = 'loading'
  result.value = null
  errMsg.value = ''
  try {
    // A single-manufacturer restriction (manufacturerAllowlist) is applied inside Kelvin over the whole
    // gate-passing pool — Kelvin returns that vendor's best-fitting parts, not a post-filtered top-N. The
    // diversity cap is lifted (maxManufacturerFraction:1) so the whole result can be that one vendor.
    const ctx = onlyMfr.value
      ? { ...props.context, manufacturerAllowlist: [onlyMfr.value], maxManufacturerFraction: 1 }
      : props.context
    const r = await selectCandidates(props.part.kind, props.part.requirements, ctx)
    if (r.error === 'NoCandidates') { result.value = r; state.value = 'empty' }
    else { result.value = r; state.value = 'ok' }
    // Vendor dropdown ← Kelvin's manufacturer FACET: every vendor with a fitting part (the full
    // gate-passing pool), not just the vendors of the top-N shown. Complete even under a restriction, so
    // it's safe to refresh on every search. (Fallback to candidates' vendors for an old engine w/o facet.)
    if (Array.isArray(r.manufacturers)) manufacturers.value = r.manufacturers
    else if (!onlyMfr.value && Array.isArray(r.candidates))
      manufacturers.value = [...new Set(r.candidates.map((c) => c.manufacturer).filter(Boolean))].sort()
  } catch (e) {
    errMsg.value = e?.message ?? String(e)
    state.value = 'error'
  }
}

// Re-query when the manufacturer restriction changes, but only once a search has been run (so it doesn't
// fire on the initial idle state). Clearing it re-runs unrestricted.
function onMfrChange() {
  if (state.value === 'ok' || state.value === 'empty' || state.value === 'error') findParts()
}

// ── Bind a chosen candidate into the design (Tier 2: Range-fetch its record → bind_part → re-sim) ──
const binding = ref(null)   // mpn currently being bound
const boundMpn = ref(null)  // mpn last successfully bound into this component
const bindErr = ref('')

async function useCandidate(c) {
  if (!props.tas) { bindErr.value = 'no design loaded to bind into'; return }
  binding.value = c.mpn
  bindErr.value = ''
  try {
    const tas = await bindPart(props.tas, props.part.ref, props.part.kind, c)
    boundMpn.value = c.mpn
    emit('bound', { ref: props.part.ref, mpn: c.mpn, tas })  // App swaps the TAS in and re-sims
  } catch (e) {
    bindErr.value = e?.message ?? String(e)
  } finally {
    binding.value = null
  }
}

// ── Magnetics: hand off to the OpenMagnetics adviser (export MAS Inputs → advise → bind back) ──────
const magState = ref('idle')  // idle | exporting | advising | done | error
const magMpn = ref(null)      // reference of the design brought back
const magErr = ref('')

async function designMagnetic() {
  if (!props.tas) { magErr.value = 'no design loaded'; magState.value = 'error'; return }
  magErr.value = ''
  try {
    magState.value = 'exporting'
    const inputs = await mainMagneticInputs(props.tas, props.part.ref)   // THIS magnetic's MAS Inputs (by ref)
    enrichMagneticWaveforms(inputs, props.part.windings) // splice real samples for custom shapes (QRM)
    magState.value = 'advising'                          // OM tab is open + advising
    const mas = await designMagneticInOpenMagnetics(props.part.ref, inputs)
    magMpn.value = mas?.magnetic?.manufacturerInfo?.reference ?? 'OpenMagnetics design'
    magState.value = 'done'
    emit('bound-magnetic', { ref: props.part.ref, mas })  // App binds it into data.magnetic + re-sims
  } catch (e) {
    magErr.value = e?.message ?? String(e)
    magState.value = 'error'
  }
}

// Reset when the selected component changes (don't auto-fetch big shards — user clicks).
watch(() => props.part?.ref, () => {
  state.value = category.value ? 'idle' : 'unsupported'
  result.value = null
  errMsg.value = ''
  onlyMfr.value = ''
  manufacturers.value = []
  binding.value = null
  boundMpn.value = null
  bindErr.value = ''
  magState.value = 'idle'
  magMpn.value = null
  magErr.value = ''
})

const candidates = computed(() => result.value?.candidates ?? [])
const considered = computed(() => result.value?.totalRowsConsidered ?? 0)
const alternatives = computed(() => result.value?.alternativesConsidered ?? 0)

// Top rejection reasons for the empty state.
const topRejections = computed(() => {
  const rej = result.value?.rejections ?? {}
  return Object.entries(rej).sort((a, b) => b[1] - a[1]).slice(0, 5)
})

function margin(cand, key) {
  const m = cand.margins?.[key]
  if (m === null || m === undefined) return null
  return m
}
function fmtx(v) { return v == null ? '—' : `×${v >= 100 ? Math.round(v) : v.toFixed(1)}` }
</script>

<template>
  <Teleport to="body">
    <template v-if="part">
      <div class="drawer-mask" @click="emit('close')"></div>
      <aside class="drawer" role="dialog" :aria-label="`Component ${part.ref}`">
        <button class="close" @click="emit('close')">ESC</button>
        <div class="mono" style="font-size: 0.66rem; letter-spacing: 0.12em; color: var(--ink-dim)">
          {{ part.kind.toUpperCase() }} · STAGE {{ part.stage }}
        </div>
        <h3>{{ part.ref }}</h3>
        <div>
          <span class="chip amber">{{ part.value }}</span>
          <span v-if="part.rating !== '—'" class="chip" style="margin-left: 0.4rem">rated {{ part.rating }}</span>
        </div>

        <table class="kv">
          <tbody>
            <tr v-for="[k, v] in rows" :key="k">
              <td class="k">{{ k }}</td>
              <td>{{ v }}</td>
            </tr>
          </tbody>
        </table>

        <template v-if="part.windings?.length">
          <div class="section-label" style="margin-top: 1.2rem">Winding waveforms</div>
          <div v-for="(exc, i) in part.windings" :key="i" style="margin-bottom: 1rem">
            <div class="mono" style="font-size: 0.7rem; color: var(--amber-hi); margin-bottom: 0.3rem">
              {{ exc.name ?? `winding ${i}` }}
            </div>
            <WavePane :excitation="exc" />
          </div>
        </template>

        <template v-if="deviceWave">
          <div class="section-label" style="margin-top: 1.2rem">
            Simulated waveforms
            <span class="hint">{{ deviceWave.name }}</span>
          </div>
          <WavePane :excitation="deviceWave" source-kind="ngspice" :periods="periods" />
        </template>

        <!-- ── Magnetics: design via the OpenMagnetics adviser (no catalog match) ── -->
        <template v-if="isMagnetic">
          <div class="section-label" style="margin-top: 1.2rem" data-testid="magnetic-section">
            Magnetic design
            <span class="hint">custom-designed by the OpenMagnetics adviser</span>
          </div>
          <button
            v-if="magState === 'idle' || magState === 'error'"
            class="find-parts"
            data-testid="design-magnetic"
            :disabled="!tas"
            @click="designMagnetic"
          >
            Design this magnetic in OpenMagnetics →
          </button>
          <div v-else-if="magState === 'exporting'" class="suggest-box">Exporting MAS inputs…</div>
          <div v-else-if="magState === 'advising'" class="suggest-box" data-testid="magnetic-advising">
            OpenMagnetics is running the magnetic adviser in a new tab — pick a design there and send it back.
          </div>
          <div v-else-if="magState === 'done'" class="suggest-box" data-testid="magnetic-done">
            <span class="bound-tag">✓ bound</span> OpenMagnetics design <b>{{ magMpn }}</b> — re-simulated.
          </div>
          <div v-if="magErr" class="suggest-box err" data-testid="magnetic-error" style="margin-top: 0.4rem">
            {{ magErr }}
          </div>
          <div class="hint" style="margin-top: 0.35rem">
            The adviser opens with this component's operating point pre-loaded; the chosen core+winding
            design binds straight back into the converter.
          </div>
        </template>

        <!-- ── TAS DB candidate parts (Kelvin) ── shown for any Kelvin category, INCLUDING magnetics
             (where it sits alongside the OpenMagnetics custom-design adviser above as a second option). -->
        <template v-if="category">
        <div class="section-label" style="margin-top: 1.2rem" data-testid="kelvin-section">
          {{ isMagnetic ? 'or pick a real catalog magnetic' : 'TAS DB candidates' }}
          <span v-if="state === 'ok'" class="hint">{{ alternatives }} of {{ considered }} parts fit</span>
        </div>

        <!-- Restrict selection to ONE manufacturer (Kelvin manufacturerAllowlist over the full pool).
             Appears once a search has found the vendors; "All manufacturers" clears the restriction. -->
        <div v-if="manufacturers.length" class="mfr-restrict" data-testid="mfr-restrict">
          <label class="mfr-label">Manufacturer</label>
          <select
            class="mfr-select" v-model="onlyMfr" @change="onMfrChange"
            data-testid="mfr-only-select"
          >
            <option value="">All manufacturers</option>
            <option v-for="m in manufacturers" :key="m" :value="m">{{ m }}</option>
          </select>
        </div>

        <div v-if="state === 'unsupported'" class="suggest-box">
          Real-part sourcing isn't available for this component kind yet
          (this pane covers semiconductors, capacitors, resistors and controllers).
        </div>

        <button
          v-else-if="state === 'idle'"
          class="find-parts"
          data-testid="find-parts"
          @click="findParts"
        >
          Find real parts matching these requirements →
        </button>

        <div v-else-if="state === 'loading'" class="suggest-box" data-testid="kelvin-loading">
          Searching the TAS database…
        </div>

        <div v-else-if="state === 'error'" class="suggest-box err" data-testid="kelvin-error">
          Sourcing failed: {{ errMsg }}
        </div>

        <div v-else-if="state === 'empty'" class="suggest-box" data-testid="kelvin-empty">
          <b>No part satisfies these requirements</b> ({{ considered }} considered).
          <table class="kv" style="margin-top: 0.5rem">
            <tbody>
              <tr v-for="[reason, n] in topRejections" :key="reason">
                <td class="k">{{ reason }}</td><td>{{ n }}</td>
              </tr>
            </tbody>
          </table>
        </div>

        <table v-else-if="state === 'ok'" class="cand" data-testid="kelvin-candidates">
          <thead>
            <tr><th>MPN</th><th>mfr</th><th>margins</th><th></th><th></th></tr>
          </thead>
          <tbody>
            <tr v-for="(c, i) in candidates" :key="c.mpn" :class="{ top: i === 0, bound: boundMpn === c.mpn }" data-testid="kelvin-candidate">
              <td class="mono mpn">{{ c.mpn }}</td>
              <td class="mfr">{{ c.manufacturer }}</td>
              <td class="mono margins">
                <span v-if="margin(c, 'vds_margin') != null" title="Vds rating / required">{{ fmtx(margin(c, 'vds_margin')) }}V</span>
                <span v-if="margin(c, 'vrrm_margin') != null" title="Vrrm rating / required">{{ fmtx(margin(c, 'vrrm_margin')) }}V</span>
                <span v-if="margin(c, 'v_margin') != null" title="Voltage rating / required">{{ fmtx(margin(c, 'v_margin')) }}V</span>
                <span v-if="margin(c, 'id_margin') != null" title="Id rating / required">{{ fmtx(margin(c, 'id_margin')) }}A</span>
                <span v-if="margin(c, 'if_avg_margin') != null" title="If rating / required">{{ fmtx(margin(c, 'if_avg_margin')) }}A</span>
                <span v-if="margin(c, 'rds_on_headroom') != null" title="Rds(on) headroom">Rds {{ fmtx(margin(c, 'rds_on_headroom')) }}</span>
              </td>
              <td>
                <span v-if="c.evidence?.datasheetUsable === false" class="badge warn" title="datasheet link unusable">⚠</span>
                <span v-if="c.evidence?.thermalPresent" class="badge ok" title="thermal data present">θ</span>
              </td>
              <td class="use-cell">
                <span v-if="boundMpn === c.mpn" class="bound-tag" data-testid="bound-tag" title="bound into the design">✓ bound</span>
                <button
                  v-else
                  class="use-btn"
                  :disabled="binding != null"
                  data-testid="use-part"
                  @click="useCandidate(c)"
                >{{ binding === c.mpn ? '…' : 'use' }}</button>
              </td>
            </tr>
          </tbody>
        </table>
        <div v-if="state === 'ok' && bindErr" class="suggest-box err" data-testid="kelvin-bind-error" style="margin-top: 0.4rem">
          Binding failed: {{ bindErr }}
        </div>
        <div v-if="state === 'ok'" class="hint" style="margin-top: 0.35rem">
          Top row is the deterministic default. Ranked by
          {{ result?.tiebreaker?.replace(/_/g, ' ') }}. <b>“use”</b> binds the real part and re-simulates.
        </div>
        </template>
      </aside>
    </template>
  </Teleport>
</template>

<style scoped>
.find-parts {
  width: 100%; margin-top: 0.5rem; padding: 0.6rem; cursor: pointer;
  background: var(--amber-dim, #3a2f10); color: var(--amber-hi, #ffcf6b);
  border: 1px solid var(--amber-hi, #ffcf6b); border-radius: 4px; font: inherit;
}
.find-parts:hover { background: var(--amber-hi, #ffcf6b); color: #000; }
.suggest-box.err { color: #ff8a8a; }
table.cand { width: 100%; margin-top: 0.5rem; border-collapse: collapse; font-size: 0.72rem; }
table.cand th { text-align: left; color: var(--ink-dim); font-weight: normal; padding: 0.2rem 0.3rem; border-bottom: 1px solid var(--rule, #333); }
table.cand td { padding: 0.25rem 0.3rem; border-bottom: 1px solid var(--rule-dim, #222); vertical-align: top; }
table.cand tr.top { background: rgba(255, 207, 107, 0.08); }
table.cand tr.bound { background: rgba(127, 208, 138, 0.12); }
table.cand .mpn { color: var(--amber-hi, #ffcf6b); }
.use-cell { white-space: nowrap; text-align: right; }
.use-btn {
  padding: 0.1rem 0.5rem; cursor: pointer; font: inherit; font-size: 0.68rem;
  background: transparent; color: var(--amber-hi, #ffcf6b);
  border: 1px solid var(--amber-hi, #ffcf6b); border-radius: 3px;
}
.use-btn:hover:not(:disabled) { background: var(--amber-hi, #ffcf6b); color: #000; }
.use-btn:disabled { opacity: 0.4; cursor: default; }
.bound-tag { color: #7fd08a; font-size: 0.68rem; white-space: nowrap; }
table.cand .mfr { color: var(--ink-dim); font-size: 0.68rem; }
table.cand .margins span { margin-right: 0.4rem; white-space: nowrap; }
.badge { display: inline-block; padding: 0 0.25rem; border-radius: 3px; font-size: 0.7rem; }
.badge.ok { color: #7fd08a; } .badge.warn { color: #ffb454; }
.mfr-restrict { margin-top: 0.4rem; display: flex; align-items: center; gap: 0.5rem; }
.mfr-label { color: var(--ink-dim); font-size: 0.7rem; white-space: nowrap; }
.mfr-select {
  flex: 1; padding: 0.35rem 0.5rem; font: inherit; font-size: 0.72rem; cursor: pointer;
  background: var(--panel, #1a1a1a); color: var(--ink, #ddd);
  border: 1px solid var(--rule, #333); border-radius: 3px;
}
.mfr-select:focus { outline: none; border-color: var(--amber-hi, #ffcf6b); }
</style>
