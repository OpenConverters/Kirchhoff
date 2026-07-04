<script setup>
import { computed, ref, watch } from 'vue'
import { requirementRows } from '../bom.js'
import { selectCandidates, kelvinCategoryFor } from '../kh.js'
import WavePane from './WavePane.vue'

const props = defineProps({
  part: { type: Object, default: null },       // a BOM row
  deviceWave: { type: Object, default: null }, // simulated V/I excitation for a non-magnetic device
  periods: { type: Number, default: 1 },
  context: { type: Object, default: () => ({}) }, // { topology, inputVoltage, switchingFrequency }
})
const emit = defineEmits(['close'])

const rows = computed(() => requirementRows(props.part?.requirements))

// ── Kelvin candidate sourcing ──────────────────────────────────────────────
const state = ref('idle')       // idle | loading | ok | empty | error | unsupported
const result = ref(null)        // Kelvin SelectionResult
const errMsg = ref('')

const category = computed(() => (props.part ? kelvinCategoryFor(props.part.kind) : null))

async function findParts() {
  if (!props.part || !category.value) { state.value = 'unsupported'; return }
  state.value = 'loading'
  result.value = null
  errMsg.value = ''
  try {
    const r = await selectCandidates(props.part.kind, props.part.requirements, props.context)
    if (r.error === 'NoCandidates') { result.value = r; state.value = 'empty' }
    else { result.value = r; state.value = 'ok' }
  } catch (e) {
    errMsg.value = e?.message ?? String(e)
    state.value = 'error'
  }
}

// Reset when the selected component changes (don't auto-fetch big shards — user clicks).
watch(() => props.part?.ref, () => {
  state.value = category.value ? 'idle' : 'unsupported'
  result.value = null
  errMsg.value = ''
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

        <!-- ── TAS DB candidate parts (Kelvin) ── -->
        <div class="section-label" style="margin-top: 1.2rem" data-testid="kelvin-section">
          TAS DB candidates
          <span v-if="state === 'ok'" class="hint">{{ alternatives }} of {{ considered }} parts fit</span>
        </div>

        <div v-if="state === 'unsupported'" class="suggest-box">
          Real-part sourcing isn't available for this component kind yet (magnetics are designed by
          MKF's adviser; this pane covers semiconductors, capacitors, resistors and controllers).
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
            <tr><th>MPN</th><th>mfr</th><th>margins</th><th></th></tr>
          </thead>
          <tbody>
            <tr v-for="(c, i) in candidates" :key="c.mpn" :class="{ top: i === 0 }" data-testid="kelvin-candidate">
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
            </tr>
          </tbody>
        </table>
        <div v-if="state === 'ok'" class="hint" style="margin-top: 0.35rem">
          Top row is the deterministic default. Ranked by
          {{ result?.tiebreaker?.replace(/_/g, ' ') }}.
        </div>
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
table.cand .mpn { color: var(--amber-hi, #ffcf6b); }
table.cand .mfr { color: var(--ink-dim); font-size: 0.68rem; }
table.cand .margins span { margin-right: 0.4rem; white-space: nowrap; }
.badge { display: inline-block; padding: 0 0.25rem; border-radius: 3px; font-size: 0.7rem; }
.badge.ok { color: #7fd08a; } .badge.warn { color: #ffb454; }
</style>
