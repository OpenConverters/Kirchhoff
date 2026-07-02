<script setup>
// One winding excitation: chart if the labels have a closed form,
// stat tiles always, and an honest note when only stats exist.
import { computed } from 'vue'
import WaveformChart from './WaveformChart.vue'
import { synthesizeWaveform } from '../synth.js'
import { si, pct } from '../units.js'

const props = defineProps({
  excitation: { type: Object, required: true }, // MAS OperatingPointExcitation
  sourceKind: { type: String, default: 'analytical' }, // 'ngspice' | 'analytical...'
  periods: { type: Number, default: 1 }, // periodic steady state: repeat the cycle N times
  fill: { type: Boolean, default: false }, // fill the parent's height (chart scales, stats compact below)
})

const freq = computed(() => props.excitation.frequency)

// All sources carry exactly ONE steady-state switching cycle; the multi-period
// view repeats it (that is what periodic steady state means — no data invented).
function tile(data, time, n) {
  if (!(n > 1)) return { data, time }
  const T = time[time.length - 1] - time[0]
  const d = [], t = []
  for (let k = 0; k < n; ++k) {
    for (let i = 0; i < data.length; ++i) {
      d.push(data[i])
      t.push(time[i] + k * T)
    }
  }
  return { data: d, time: t }
}

function signalTraces(side, unit) {
  const sig = props.excitation[side]
  if (!sig) return null
  // prefer real waveform data (ngspice / full analytical) over synthesis
  if (sig.waveform?.data?.length > 1 && sig.waveform?.time?.length === sig.waveform.data.length) {
    const { data, time } = tile(sig.waveform.data, sig.waveform.time, props.periods)
    return { label: side, unit, data, time, synthetic: false }
  }
  const wf = synthesizeWaveform(sig.processed, freq.value)
  if (wf) {
    const { data, time } = tile(wf.data, wf.time, props.periods)
    return { label: side, unit, data, time, synthetic: true }
  }
  return null
}

const traces = computed(() =>
  [signalTraces('current', 'A'), signalTraces('voltage', 'V')].filter(Boolean)
)

const stats = computed(() => {
  const out = []
  for (const [side, unit, cls] of [['current', 'A', 'i'], ['voltage', 'V', 'v']]) {
    const p = props.excitation[side]?.processed
    if (!p) continue
    out.push(
      { k: `${side} peak`, n: si(p.peak, unit), cls },
      { k: `${side} RMS`, n: si(p.rms, unit), cls },
      { k: `${side} pk-pk`, n: si(p.peakToPeak, unit), cls },
      { k: `${side} offset`, n: si(p.offset, unit), cls },
    )
  }
  const d = props.excitation.current?.processed?.dutyCycle
  if (d !== null && d !== undefined) out.push({ k: 'duty cycle', n: pct(d), cls: '' })
  return out
})

const missing = computed(() => {
  const m = []
  if (props.excitation.current && !traces.value.some((t) => t.unit === 'A')) m.push('current')
  if (props.excitation.voltage && !traces.value.some((t) => t.unit === 'V')) m.push('voltage')
  return m
})
</script>

<template>
  <div :class="{ 'wavepane-fill': fill }">
    <!-- fill mode uses a wide viewBox aspect so the contain-fit nearly fills the pane width
         (a stacked pane is wide-and-short); the SVG then scales to that slot with no distortion -->
    <WaveformChart v-if="traces.length" :traces="traces" :height="fill ? 150 : 240" :class="{ 'chart-fill': fill }" />
    <div v-if="missing.length" class="wave-empty" style="margin-top: 0.6rem">
      <template v-if="sourceKind === 'ngspice'">
        The ngspice extraction rebuilds winding <b>currents</b> from the simulation; the
        {{ missing.join(' and ') }} of this winding is reported through the analytical stress
        figures below.
      </template>
      <template v-else>
        The {{ missing.join(' and ') }} waveform of this winding carries only analytical stress
        figures here (<code>label: custom</code>, no closed shape). Use “▶ ngspice this magnetic”
        in the toolbar for the full simulated trace.
      </template>
    </div>
    <div class="stat-row" :class="{ compact: fill }">
      <div v-for="s in stats" :key="s.k" class="stat" :class="s.cls">
        <div class="k">{{ s.k }}</div>
        <div class="n">{{ s.n }}</div>
      </div>
    </div>
  </div>
</template>
