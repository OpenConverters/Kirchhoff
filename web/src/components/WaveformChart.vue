<script setup>
import { computed } from 'vue'
import { si } from '../units.js'

// traces: [{ label, unit ('A'|'V'), data: number[], time: number[] }]
// Currents plot against the left axis (amber), voltages the right (cyan).
const props = defineProps({
  traces: { type: Array, required: true },
  height: { type: Number, default: 240 },
})

const W = 760
const PAD = { l: 62, r: 62, t: 14, b: 30 }

const COLORS = { A: 'var(--amber)', V: 'var(--cyan)' }

const layout = computed(() => {
  const traces = props.traces.filter((t) => t?.data?.length > 1)
  if (!traces.length) return null
  const h = props.height
  const plotW = W - PAD.l - PAD.r
  const plotH = h - PAD.t - PAD.b

  const tMax = Math.max(...traces.map((t) => t.time[t.time.length - 1]))
  const axes = {}
  for (const t of traces) {
    const ax = (axes[t.unit] ??= { min: Infinity, max: -Infinity })
    for (const v of t.data) {
      if (v < ax.min) ax.min = v
      if (v > ax.max) ax.max = v
    }
  }
  for (const ax of Object.values(axes)) {
    if (ax.max === ax.min) { ax.max += 1; ax.min -= 1 }
    const m = (ax.max - ax.min) * 0.08
    ax.max += m; ax.min -= m
    // keep zero visible when the signal straddles or hugs it
    if (ax.min > 0 && ax.min < (ax.max - ax.min) * 0.5) ax.min = 0
  }

  const sx = (t) => PAD.l + (t / tMax) * plotW
  const sy = (v, ax) => PAD.t + (1 - (v - ax.min) / (ax.max - ax.min)) * plotH

  const paths = traces.map((t) => {
    const ax = axes[t.unit]
    let d = ''
    for (let i = 0; i < t.data.length; ++i) {
      d += `${i ? 'L' : 'M'} ${sx(t.time[i]).toFixed(2)} ${sy(t.data[i], ax).toFixed(2)} `
    }
    return { d, color: COLORS[t.unit] ?? 'var(--ink)', label: t.label, unit: t.unit }
  })

  const gridY = []
  for (let i = 0; i <= 4; ++i) gridY.push(PAD.t + (plotH * i) / 4)

  const leftTicks = axes.A
    ? gridY.map((y, i) => ({ y, label: si(axes.A.max - ((axes.A.max - axes.A.min) * i) / 4, 'A', 3) }))
    : []
  const rightTicks = axes.V
    ? gridY.map((y, i) => ({ y, label: si(axes.V.max - ((axes.V.max - axes.V.min) * i) / 4, 'V', 3) }))
    : []
  const xTicks = []
  for (let i = 0; i <= 4; ++i) {
    xTicks.push({ x: PAD.l + (plotW * i) / 4, label: si((tMax * i) / 4, 's', 3) })
  }
  // zero lines when an axis straddles zero
  const zeros = []
  for (const [unit, ax] of Object.entries(axes)) {
    if (ax.min < 0 && ax.max > 0) zeros.push({ y: sy(0, ax), color: COLORS[unit] })
  }
  return { h, paths, gridY, leftTicks, rightTicks, xTicks, zeros, plotW }
})
</script>

<template>
  <div v-if="layout" class="schematic-frame">
    <svg :viewBox="`0 0 ${W} ${layout.h}`" xmlns="http://www.w3.org/2000/svg" role="img">
      <line
        v-for="(y, i) in layout.gridY" :key="'g' + i"
        :x1="PAD.l" :x2="W - PAD.r" :y1="y" :y2="y"
        stroke="var(--grat-strong)" stroke-width="1"
      />
      <line
        v-for="(t, i) in layout.xTicks" :key="'x' + i"
        :x1="t.x" :x2="t.x" :y1="PAD.t" :y2="layout.h - PAD.b"
        stroke="var(--grat)" stroke-width="1"
      />
      <line
        v-for="(z, i) in layout.zeros" :key="'z' + i"
        :x1="PAD.l" :x2="W - PAD.r" :y1="z.y" :y2="z.y"
        :stroke="z.color" stroke-width="0.6" stroke-dasharray="5 5" opacity="0.5"
      />
      <path
        v-for="(p, i) in layout.paths" :key="'p' + i"
        :d="p.d" fill="none" :stroke="p.color" stroke-width="1.8"
        stroke-linejoin="round"
        :style="`filter: drop-shadow(0 0 4px ${p.color})`"
      />
      <text
        v-for="(t, i) in layout.leftTicks" :key="'l' + i"
        :x="PAD.l - 7" :y="t.y + 3.5" text-anchor="end"
        fill="var(--amber)" font-size="10" font-family="var(--mono)"
      >{{ t.label }}</text>
      <text
        v-for="(t, i) in layout.rightTicks" :key="'r' + i"
        :x="W - PAD.r + 7" :y="t.y + 3.5" text-anchor="start"
        fill="var(--cyan)" font-size="10" font-family="var(--mono)"
      >{{ t.label }}</text>
      <text
        v-for="(t, i) in layout.xTicks" :key="'xt' + i"
        :x="t.x" :y="layout.h - 10" text-anchor="middle"
        fill="var(--ink-dim)" font-size="10" font-family="var(--mono)"
      >{{ t.label }}</text>
    </svg>
  </div>
</template>
