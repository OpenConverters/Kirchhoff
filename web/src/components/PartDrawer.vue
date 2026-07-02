<script setup>
import { computed } from 'vue'
import { requirementRows } from '../bom.js'
import WavePane from './WavePane.vue'

const props = defineProps({
  part: { type: Object, default: null },       // a BOM row
  deviceWave: { type: Object, default: null }, // simulated V/I excitation for a non-magnetic device
  periods: { type: Number, default: 1 },
})
const emit = defineEmits(['close'])

const rows = computed(() => requirementRows(props.part?.requirements))
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

        <!-- non-magnetic device: simulated terminal V/I from the ngspice run -->
        <template v-if="deviceWave">
          <div class="section-label" style="margin-top: 1.2rem">
            Simulated waveforms
            <span class="hint">{{ deviceWave.name }}</span>
          </div>
          <WavePane :excitation="deviceWave" source-kind="ngspice" :periods="periods" />
        </template>

        <div class="suggest-box">
          <b>TAS DB SUGGESTIONS</b><br />
          Coming soon: real parts matching these requirements, pulled from the TAS component
          database (and MKF's MagneticAdviser for the magnetics). The requirement set above is
          exactly what the matcher will consume.
        </div>
      </aside>
    </template>
  </Teleport>
</template>
