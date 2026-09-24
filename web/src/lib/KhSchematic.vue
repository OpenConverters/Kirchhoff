<script setup>
// The CIAS-verified schematic of a solved design (ciasSchematic.js renderVerifiedSchematic), with its
// parts as buttons. Renders as a fragment — the frame, or the refusal banner, or the empty note — so a
// host places it inside its own layout box.
//
// Props
//   topology      topology id (required)
//   tas | result  the design: a TAS, or processConverter's result (its .tas)
//   variant       the topology variant the design was built with (informational; the TAS is drawn)
//   bom           BOM rows (bom.js extractBom); default: extracted from the TAS
//   topologyName  the name the refusal banner quotes (default: the topology's own name)
//   selectable    (component) => boolean, component = { ref, kind, ...bomRow } (kind as bom.js names
//                 it: 'MOSFET', 'Diode', 'Capacitor', 'Inductor', 'Transformer', …). Parts it rejects
//                 are drawn muted and are neither clickable nor focusable. Default: every part.
//   selectedRef   the part drawn selected
// Events
//   select(ref)   a selectable part was clicked, or activated with Enter/Space
// Slots
//   caption       under the frame, only when a drawing was produced
//   empty         when there is no design (default: a one-line note)
import { computed, nextTick, ref, watch } from 'vue'
import { renderVerifiedSchematic } from '../ciasSchematic.js'
import { extractBom } from '../bom.js'
import { topologyById } from '../topologies.js'

const props = defineProps({
  topology: { type: String, required: true },
  tas: { type: Object, default: null },
  result: { type: Object, default: null },
  variant: { type: String, default: null },
  bom: { type: Array, default: null },
  topologyName: { type: String, default: null },
  selectable: { type: Function, default: null },
  selectedRef: { type: String, default: null },
})
const emit = defineEmits(['select'])

const designTas = computed(() => props.tas ?? props.result?.tas ?? null)
const rows = computed(() => props.bom ?? (designTas.value ? extractBom(designTas.value) : []))
const name = computed(() => props.topologyName ?? topologyById(props.topology)?.name ?? props.topology)

// Prefer a loud refusal over a wrong picture: if the generator throws (a real netlist drift), the
// banner says so and nothing is drawn in its place.
const error = ref(null)
const svg = computed(() => {
  error.value = null
  if (!designTas.value) return null
  try { return renderVerifiedSchematic(props.topology, designTas.value, props.variant, rows.value) }
  catch (e) { error.value = e?.message ?? String(e); return null }
})

function isSelectable(ref_) {
  if (!props.selectable) return true
  const row = rows.value.find((r) => r.ref === ref_)
  return !!props.selectable(row ? { ...row } : { ref: ref_, kind: null })
}

// The SVG is injected with v-html, so selection and muting are toggled on the live nodes rather than
// re-rendering the drawing (a re-render on every selection would also throw away scroll/hover state).
const frame = ref(null)
watch([() => props.selectedRef, svg], async ([ref_]) => {
  await nextTick()
  const el = frame.value
  if (!el) return
  for (const g of el.querySelectorAll('g.sch-hot.selected')) g.classList.remove('selected')
  if (ref_) el.querySelector(`g.sch-hot[data-ref="${CSS.escape(ref_)}"]:not(.sch-ann)`)?.classList.add('selected')
}, { flush: 'post' })

watch([svg, () => props.selectable, rows], async () => {
  if (!props.selectable) return      // nothing is ever muted: leave the drawing exactly as generated
  await nextTick()
  const el = frame.value
  if (!el) return
  for (const g of el.querySelectorAll('g.sch-hot[data-ref]:not(.sch-ann)')) {
    const muted = !isSelectable(g.dataset.ref)
    g.classList.toggle('kh-sch-muted', muted)
    if (muted) {
      if (g.hasAttribute('tabindex')) g.dataset.khTabindex = g.getAttribute('tabindex')
      g.setAttribute('tabindex', '-1')
      g.setAttribute('aria-disabled', 'true')
    } else {
      if (g.dataset.khTabindex !== undefined) { g.setAttribute('tabindex', g.dataset.khTabindex); delete g.dataset.khTabindex }
      g.removeAttribute('aria-disabled')
    }
  }
}, { flush: 'post', immediate: true })

function target(ev) {
  const g = ev.target.closest?.('[data-ref]')
  // .sch-ann marks something drawn that is not an orderable part (a FET's intrinsic body diode): it has
  // no BOM row, so there is nothing to open and the stylesheet gives it no cursor or hover either.
  if (!g || g.classList.contains('sch-ann') || g.classList.contains('kh-sch-muted')) return null
  return g.dataset.ref
}
function onClick(ev) {
  const r = target(ev)
  if (r) emit('select', r)
}
// The same activation from the keyboard (ABT #693): every component is a role="button" tab stop, and a
// button that answers the mouse but not Enter/Space is not operable (WCAG 2.1.1). Space is prevented
// because its default is to scroll the page out from under the drawing.
function onKey(ev) {
  if (ev.key !== 'Enter' && ev.key !== ' ') return
  const r = target(ev)
  if (!r) return
  ev.preventDefault()
  emit('select', r)
}
</script>

<template>
  <!-- role=alert: when the generator refuses to draw (a netlist drift, or a design whose components
       no layout can place) this banner REPLACES the drawing, and without a live region a screen
       reader is told nothing at all — the pane simply stops having a schematic in it. -->
  <div v-if="error" class="wave-empty sch-error" role="alert">
    Schematic ≠ netlist for <code>{{ name }}</code>: {{ error }}
  </div>
  <template v-else-if="svg">
    <div ref="frame" class="schematic-frame fit" v-html="svg" @click="onClick" @keydown="onKey"></div>
    <slot name="caption" />
  </template>
  <slot v-else name="empty">
    <div class="wave-empty">No design to draw yet — solve one first.</div>
  </slot>
</template>
