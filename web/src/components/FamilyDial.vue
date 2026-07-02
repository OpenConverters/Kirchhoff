<script setup>
// A rotary instrument-mode selector — the converter-family "dial". One detent per family around a
// ~280° arc, a pointer that swings to the selected detent, and the full family name read out in the
// centre. Click a detent to jump; click the knob to advance; arrow keys rotate. Pure retro EDA.
import { computed } from 'vue'

const props = defineProps({
  modelValue: { type: String, required: true },  // selected family
  families: { type: Array, required: true },      // full family names, in dial order
  short: { type: Object, required: true },        // family -> short label
})
const emit = defineEmits(['update:modelValue'])

const CX = 70, CY = 66, R = 46, RTICK = 40, RLABEL = 62, RPTR = 34
const START = -140, SWEEP = 280
const LINEH = 8.4   // label line height (px in viewBox units)

const detents = computed(() => {
  const n = props.families.length
  return props.families.map((fam, i) => {
    const deg = START + (n > 1 ? (SWEEP * i) / (n - 1) : 0)
    const a = (deg * Math.PI) / 180
    const sin = Math.sin(a), cos = Math.cos(a)
    const raw = props.short[fam] ?? fam
    const lines = Array.isArray(raw) ? raw : [raw]   // a label may be one or two words, stacked
    return {
      fam, deg, lines,
      tx: CX + RTICK * sin, ty: CY - RTICK * cos,
      ox: CX + R * sin, oy: CY - R * cos,
      lx: CX + RLABEL * sin, ly: CY - RLABEL * cos,
    }
  })
})
// vertical-centre a 1- or 2-line label about its anchor point
const lineY = (d, li) => d.ly + (li - (d.lines.length - 1) / 2) * LINEH + 3
const idx = computed(() => Math.max(0, props.families.indexOf(props.modelValue)))
const pointer = computed(() => {
  const d = detents.value[idx.value]
  const a = (d.deg * Math.PI) / 180
  return { x: CX + RPTR * Math.sin(a), y: CY - RPTR * Math.cos(a) }
})

const select = (fam) => emit('update:modelValue', fam)
const advance = (step) => {
  const n = props.families.length
  emit('update:modelValue', props.families[(idx.value + step + n) % n])
}
</script>

<template>
  <div class="dial" @keydown.left.prevent="advance(-1)" @keydown.right.prevent="advance(1)"
       @keydown.up.prevent="advance(-1)" @keydown.down.prevent="advance(1)" tabindex="0"
       role="listbox" :aria-label="`Converter family: ${modelValue}`">
    <svg viewBox="-30 -6 200 146" xmlns="http://www.w3.org/2000/svg">
      <!-- dial face -->
      <circle class="dial-face" :cx="CX" :cy="CY" :r="R + 6" />
      <circle class="dial-ring" :cx="CX" :cy="CY" :r="R" />
      <!-- detents -->
      <g v-for="d in detents" :key="d.fam" class="detent" :class="{ on: d.fam === modelValue }"
         @click="select(d.fam)" role="option" :aria-selected="d.fam === modelValue">
        <line :x1="d.tx" :y1="d.ty" :x2="d.ox" :y2="d.oy" />
        <text v-for="(ln, li) in d.lines" :key="li" :x="d.lx" :y="lineY(d, li)">{{ ln }}</text>
      </g>
      <!-- pointer + knob (click to advance) -->
      <g class="knob" @click="advance(1)">
        <line class="ptr" :x1="CX" :y1="CY" :x2="pointer.x" :y2="pointer.y" />
        <circle class="knob-body" :cx="CX" :cy="CY" r="17" />
        <circle class="knob-grip" :cx="CX" :cy="CY" r="17" />
      </g>
    </svg>
    <div class="dial-readout">{{ modelValue }}</div>
  </div>
</template>

<style scoped>
.dial { text-align: center; outline: none; }
.dial:focus-visible { outline: 2px solid var(--amber); outline-offset: 3px; border-radius: 8px; }
.dial svg { width: 100%; max-width: 230px; height: auto; }
.dial-face { fill: #100c07; stroke: var(--line); stroke-width: 1; }
.dial-ring { fill: none; stroke: var(--line-soft); stroke-width: 1; }
.detent line { stroke: var(--line); stroke-width: 2; }
.detent text {
  font-family: var(--mono); font-size: 8.4px; fill: var(--ink-dim);
  text-anchor: middle; letter-spacing: 0.02em;
}
.detent { cursor: pointer; }
.detent:hover text { fill: var(--amber-hi); }
.detent.on line { stroke: var(--amber); }
.detent.on text { fill: var(--amber); }
.knob { cursor: pointer; }
.ptr { stroke: var(--amber); stroke-width: 2.5; stroke-linecap: round; filter: drop-shadow(0 0 3px var(--amber)); }
.knob-body { fill: #1b140b; stroke: var(--amber-deep); stroke-width: 1.5; }
.knob-grip { fill: none; stroke: var(--amber-deep); stroke-width: 1; stroke-dasharray: 2 3; }
.knob:hover .knob-body { stroke: var(--amber); }
.dial-readout {
  font-family: var(--disp); font-size: 0.74rem; letter-spacing: 0.1em; text-transform: uppercase;
  color: var(--amber); margin-top: 0.15rem; min-height: 1.1em;
}
@media (prefers-reduced-motion: reduce) { .ptr { filter: none; } }
</style>
