<script setup>
// One component's simulated current + voltage (the ngspice component run, kh.componentWaveforms), drawn
// with the bench's WavePane/WaveformChart. Renders as a fragment: the name line, the chart + stat tiles,
// and the trace legend.
//
// Props
//   componentRef  the part to show (a BOM ref: 'Q1', 'D1', 'Cout', …). Required.
//   waves         a componentWaveforms() payload { referencePeriod, components:[{ref,kind,voltage,current}] }.
//                 Omitted → the component runs it itself from `tas` (+ `fidelity`) on the engine.
//   tas           the design to simulate when `waves` is not given
//   fidelity      the NGSPICE fidelity directive for that run (default { origin: 'REQUIREMENTS' })
//   engine        a createKirchhoff() instance (default: the injected one; only needed with `tas`)
//   periods       switching periods to draw (default 1)
//   fill          fill the parent's height (the bench's pane layout)
// Events
//   loaded(waves) after a self-run simulation lands
//   error(message) when that run fails
// Slots
//   empty         when the component has no waveform (not simulated, or not in the run)
import { computed, inject, ref, watch } from 'vue'
import WavePane from '../components/WavePane.vue'
import { si } from '../units.js'
import { KIRCHHOFF_KEY } from './context.js'

const props = defineProps({
  componentRef: { type: String, required: true },
  waves: { type: Object, default: null },
  tas: { type: Object, default: null },
  fidelity: { type: Object, default: () => ({ origin: 'REQUIREMENTS' }) },
  engine: { type: Object, default: null },
  periods: { type: Number, default: 1 },
  fill: { type: Boolean, default: false },
})
const emit = defineEmits(['loaded', 'error'])

// An engine is only needed when this instance has to simulate: a host that passes `waves` needs none.
const injected = inject(KIRCHHOFF_KEY, null)
const ownWaves = ref(null)
const busy = ref(false)
const failure = ref(null)
let gen = 0
watch(() => [props.waves, props.tas, props.fidelity], async () => {
  if (props.waves || !props.tas) return
  const g = ++gen
  busy.value = true
  failure.value = null
  try {
    const engine = props.engine ?? injected
    if (!engine) throw new Error('no Kirchhoff engine: app.use(createKirchhoff({ wasmUrl })) or pass an `engine` prop')
    const cw = await engine.componentWaveforms(props.tas, props.fidelity)
    if (g !== gen) return
    if (cw?.success === false) throw new Error('this engine build has no ngspice — component waveforms need it')
    ownWaves.value = cw
    emit('loaded', cw)
  } catch (e) {
    if (g !== gen) return
    failure.value = e?.message ?? String(e)
    emit('error', failure.value)
  } finally {
    if (g === gen) busy.value = false
  }
}, { immediate: true })

const payload = computed(() => props.waves ?? ownWaves.value)
const comp = computed(() => payload.value?.components?.find((c) => c.ref === props.componentRef) ?? null)
// A component reshaped as an excitation so WavePane renders it unchanged. Its voltage is a real
// simulated node difference (V_DS / V_AK / V_C), so — unlike the magnetics — nothing is analytical.
const excitation = computed(() => {
  const c = comp.value
  if (!c) return null
  const f = payload.value?.referencePeriod ? 1 / payload.value.referencePeriod : 0
  return { name: `${c.ref} · ${c.voltage?.label ?? 'V'}`, frequency: f, current: c.current, voltage: c.voltage }
})
</script>

<template>
  <template v-if="excitation">
    <div class="mono wave-name">▸ {{ excitation.name }}
      <span class="chip" style="margin-left: 0.4rem">{{ si(excitation.frequency, 'Hz') }}</span>
    </div>
    <WavePane :excitation="excitation" source-kind="ngspice" :periods="periods" :fill="fill" />
    <div class="wave-readout">
      <span class="i"><b>—</b> current · measured</span>
      <span class="v"><b>—</b> {{ comp?.voltage?.label ?? 'V' }} · measured</span>
    </div>
  </template>
  <div v-else-if="busy" class="boot" style="margin-top: 0.6rem"><div class="spin"></div> simulating components…</div>
  <div v-else-if="failure" class="err-banner"><b>ENGINE ▸</b> {{ failure }}</div>
  <slot v-else name="empty">
    <div class="wave-empty">No simulated waveform for <code>{{ componentRef }}</code>.</div>
  </slot>
</template>
