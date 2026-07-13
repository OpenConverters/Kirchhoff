<script setup>
// One selectable output pane. The workbench shows two of these side by side (default Schematic +
// Waveforms). All design state + methods come from the injected `kh` context App provides, so the pane
// is a pure view: pick a view type from its header dropdown, render it.
import { computed, inject, ref, watch } from 'vue'
import { trackEvent } from '../telemetry.js'
import WavePane from './WavePane.vue'

const props = defineProps({ view: { type: String, required: true } })
const emit = defineEmits(['update:view'])
const view = computed({ get: () => props.view, set: (v) => emit('update:view', v) })
// Which result view the user looks at (waveforms / diagnostics / bom / netlist / visual).
watch(() => props.view, (v) => trackEvent('pane_view', { target: v, topology: topo?.value?.topology }))

const VIEWS = [
  ['schematic', 'Schematic'],
  ['waveforms', 'Waveforms'],
  ['bom', 'BOM'],
  ['diagnostics', 'Diagnostics'],
  ['netlist', 'Netlist'],
  ['visual', 'Visual sim'],
]

const kh = inject('kh')
const {
  result, topo, diag, bomRows, selectedPart, schematicSvg, schematicError, schematicClick, openPart,
  waveTarget, waveMagnetics, deviceGroups, targetIsMagnetic, waveOps, waveOpIdx, waveSource,
  waveExcitations, waveMag, ngspiceOps, ngspiceBusy, simulateMagnetic, downloadMagneticInputs,
  deviceExcitation, deviceComp, componentBusy, fetchComponentWaves,
  componentStress, stressSummary, verdict, mainExcNames, form, visualSim, visualScopeSet,
  deck, deckFlavor, deckFidelity, simStop, simStep, deckBusy, designStop, designStep, periodsShown,
  makeDeck, copyDeck, downloadDeck, si, pct,
} = kh

// Visual-sim panel height + a real drag handle. The iframe swallows pointer events over its area
// (and setPointerCapture does NOT redirect them away from a cross-document iframe), so dragging the
// grip UP over the iframe would silently stop. Fix: on pointerdown we synchronously insert a
// full-window overlay ABOVE the iframe — every move during the drag then lands on the overlay (in
// this document) and reaches our listener, in both directions. Created imperatively so it exists
// before the very first move (a Vue v-if would render a frame too late).
const falstadH = ref(600)
const resizing = ref(false)
// The pane is a FIXED-height grid cell (pane-grid: 1fr 1fr), so growing the card in-place only scrolls
// the pane — it can't actually enlarge the visible sim. Maximize escapes the pane into a full-viewport
// overlay, which is the real "make it bigger". Esc or the ✕ closes it.
const maximized = ref(false)
function onEsc(e) { if (e.key === 'Escape') maximized.value = false }
watch(maximized, (on) => {
  if (on) document.addEventListener('keydown', onEsc)
  else document.removeEventListener('keydown', onEsc)
})
function startResize(e) {
  e.preventDefault()
  const startY = e.clientY, startH = falstadH.value
  resizing.value = true
  const overlay = document.createElement('div')
  overlay.style.cssText = 'position:fixed;inset:0;z-index:99999;cursor:ns-resize'
  document.body.appendChild(overlay)
  const onMove = (ev) => {
    falstadH.value = Math.max(300, Math.min(Math.round(window.innerHeight * 0.92), startH + (ev.clientY - startY)))
  }
  const onUp = () => {
    resizing.value = false
    overlay.remove()
    window.removeEventListener('pointermove', onMove)
    window.removeEventListener('pointerup', onUp)
  }
  window.addEventListener('pointermove', onMove)
  window.addEventListener('pointerup', onUp)
}

// A magnetic can carry several windings; show one at a time with a per-pane selector (kept local so the
// two panes can look at different windings of the same magnetic). Reset when the magnetic changes.
const windingIdx = ref(0)
watch([waveTarget, waveOpIdx, () => waveExcitations.value.length], () => { windingIdx.value = 0 })
const winding = computed(() => waveExcitations.value[windingIdx.value] ?? null)
</script>

<template>
  <div class="pane">
    <div class="pane-head">
      <select class="pane-select" v-model="view">
        <option v-for="[id, label] in VIEWS" :key="id" :value="id">{{ label }}</option>
      </select>
    </div>

    <div class="pane-body">
      <!-- schematic (scaled to fit the pane, no crop) -->
      <div v-if="view === 'schematic'" class="view-fill">
        <div v-if="schematicError" class="wave-empty sch-error">
          Schematic ≠ netlist for <code>{{ topo.name }}</code>: {{ schematicError }}
        </div>
        <template v-else-if="schematicSvg">
          <div class="schematic-frame fit" v-html="schematicSvg" @click="schematicClick"></div>
          <div class="sch-caption">Power-path sketch (generated from the CIAS netlist) — click any component for its details.</div>
        </template>
        <div v-else class="wave-empty">
          No schematic sketch for <code>{{ topo.name }}</code> yet — see the BOM view for every component.
        </div>
      </div>

      <!-- BOM -->
      <div v-else-if="view === 'bom'">
        <table class="data-table">
          <thead><tr><th>Ref</th><th>Kind</th><th>Stage</th><th>Value</th><th>Rating</th></tr></thead>
          <tbody>
            <tr v-for="r in bomRows" :key="r.ref" class="clickable"
                :class="{ selected: selectedPart?.ref === r.ref }" @click="openPart(r.ref)">
              <td style="color: var(--amber-hi)">{{ r.ref }}</td>
              <td>{{ r.kind }}</td>
              <td class="dim">{{ r.stage }}</td>
              <td>{{ r.value }}</td>
              <td class="dim">{{ r.rating }}</td>
            </tr>
          </tbody>
        </table>
      </div>

      <!-- waveforms -->
      <div v-else-if="view === 'waveforms'" class="view-fill">
        <div class="wave-toolbar">
          <select class="fld-in" style="width: auto" v-model="waveTarget">
            <optgroup label="Magnetics">
              <option v-for="(m, i) in waveMagnetics" :key="'m' + i" :value="`mag:${i}`">
                {{ m.name }}{{ m.isMain ? ' (main)' : '' }}
              </option>
            </optgroup>
            <optgroup v-for="g in deviceGroups" :key="g.label" :label="g.label">
              <option v-for="r in g.items" :key="r.ref" :value="`dev:${r.ref}`">{{ r.ref }} — {{ r.value }}</option>
            </optgroup>
          </select>
          <select v-if="targetIsMagnetic && waveOps.length > 1" class="fld-in" style="width: auto" v-model.number="waveOpIdx">
            <option v-for="(op, i) in waveOps" :key="i" :value="i">{{ op.name ?? `OP ${i + 1}` }}</option>
          </select>
          <!-- winding selector: one winding at a time when the magnetic has several -->
          <select v-if="targetIsMagnetic && waveExcitations.length > 1"
                  class="fld-in" style="width: auto" v-model.number="windingIdx">
            <option v-for="(exc, i) in waveExcitations" :key="i" :value="i">{{ exc.name ?? `winding ${i}` }}</option>
          </select>
          <span v-if="targetIsMagnetic" class="chip" :class="waveSource.kind === 'ngspice' ? 'cyan' : 'amber'">{{ waveSource.kind }}</span>
          <span v-else class="chip cyan">ngspice</span>
          <div class="sep"></div>
          <button v-if="targetIsMagnetic && !ngspiceOps[waveMag?.name]" class="btn ghost"
                  :disabled="ngspiceBusy" @click="simulateMagnetic">
            {{ ngspiceBusy ? 'Simulating…' : '▶ ngspice' }}
          </button>
        </div>

        <template v-if="targetIsMagnetic">
          <template v-if="winding">
            <div class="mono wave-name">▸ {{ winding.name ?? `winding ${windingIdx}` }}
              <span class="chip" style="margin-left: 0.4rem">{{ si(winding.frequency, 'Hz') }}</span>
            </div>
            <WavePane :excitation="winding" :source-kind="waveSource.kind" :periods="form.showPeriods" fill />
          </template>
          <div class="wave-readout">
            <span class="i"><b>—</b> current{{ waveSource.kind === 'ngspice' ? ' · measured' : '' }}</span>
            <span class="v"><b>—</b> voltage{{ waveSource.voltageKind === 'analytical' ? ' · analytical' : '' }}</span>
          </div>
        </template>
        <template v-else>
          <template v-if="deviceExcitation">
            <div class="mono wave-name">▸ {{ deviceExcitation.name }}
              <span class="chip" style="margin-left: 0.4rem">{{ si(deviceExcitation.frequency, 'Hz') }}</span>
            </div>
            <WavePane :excitation="deviceExcitation" source-kind="ngspice" :periods="form.showPeriods" fill />
            <div class="wave-readout">
              <span class="i"><b>—</b> current · measured</span>
              <span class="v"><b>—</b> {{ deviceComp?.voltage?.label ?? 'V' }} · measured</span>
            </div>
          </template>
          <div v-else-if="componentBusy" class="boot" style="margin-top: 0.6rem"><div class="spin"></div> simulating components…</div>
          <div v-else class="wave-empty">
            Component waveforms need an ngspice run.
            <button class="btn ghost" :disabled="componentBusy" @click="fetchComponentWaves" style="margin-left: 0.3rem">▶ run ngspice</button>
          </div>
        </template>
      </div>

      <!-- diagnostics -->
      <div v-else-if="view === 'diagnostics'">
        <div class="stat-row">
          <div class="stat"><div class="k">fsw</div><div class="n">{{ si(diag.switchingFrequency, 'Hz') }}</div></div>
          <div class="stat"><div class="k">duty</div><div class="n">{{ pct(diag.dutyCycle) }}</div></div>
          <div class="stat"><div class="k">mode</div><div class="n">{{ diag.isCcm ? 'CCM' : 'DCM' }}</div></div>
          <div class="stat i"><div class="k">Ipk (pri)</div><div class="n">{{ si(diag.primaryPeakCurrent, 'A') }}</div></div>
          <div class="stat"><div class="k">Lm</div><div class="n">{{ si(diag.computed?.magnetizingInductance, 'H') }}</div></div>
        </div>

        <div class="section-label" style="margin-top: 1.1rem">
          Component stress
          <span class="hint" v-if="componentStress.length">
            <span class="chip ok">{{ stressSummary.ok }} ok</span>
            <span v-if="stressSummary.warn" class="chip" style="margin-left: 0.3rem">{{ stressSummary.warn }} tight</span>
            <span v-if="stressSummary.bad" class="chip bad" style="margin-left: 0.3rem">{{ stressSummary.bad }} over</span>
          </span>
          <span class="hint" v-else>needs an ngspice run</span>
        </div>
        <table v-if="componentStress.length" class="data-table">
          <thead><tr><th>Ref</th><th>Kind</th><th class="num">V pk</th><th class="num">rated</th><th class="num">mgn</th>
            <th class="num">I/P</th><th class="num">rated</th><th class="num">mgn</th><th></th></tr></thead>
          <tbody>
            <tr v-for="s in componentStress" :key="s.ref" class="clickable" @click="openPart(s.ref)">
              <td style="color: var(--amber-hi)">{{ s.ref }}</td>
              <td class="dim">{{ s.kind }}</td>
              <td class="num">{{ s.v ? si(s.v.stress, 'V') : '—' }}</td>
              <td class="num dim">{{ s.v?.rated ? si(s.v.rated, 'V') : '—' }}</td>
              <td class="num"><span v-if="s.v?.ratio != null" class="chip" :class="verdict(s.v.ratio)">{{ pct(1 - s.v.ratio, 0) }}</span><span v-else class="dim">—</span></td>
              <td class="num">{{ s.i ? si(s.i.stress, s.i.unit) : '—' }}<span v-if="s.i?.basis" class="dim" style="font-size: 0.85em"> {{ s.i.basis }}</span></td>
              <td class="num dim">{{ s.i?.rated ? si(s.i.rated, s.i.unit) : '—' }}</td>
              <td class="num"><span v-if="s.i?.ratio != null" class="chip" :class="verdict(s.i.ratio)">{{ pct(1 - s.i.ratio, 0) }}</span><span v-else class="dim">—</span></td>
              <td><span v-if="s.verdict" class="chip" :class="s.verdict">{{ s.verdict === 'ok' ? 'pass' : s.verdict === 'warn' ? 'tight' : 'over' }}</span></td>
            </tr>
          </tbody>
        </table>
        <div v-else class="wave-empty">
          Per-component stress vs rating from ngspice.
          <button class="btn ghost" :disabled="componentBusy" @click="fetchComponentWaves" style="margin-left: 0.3rem">▶ run ngspice</button>
        </div>

        <div class="section-label" style="margin-top: 1.1rem">Magnetics</div>
        <table class="data-table">
          <thead><tr><th>Name</th><th>Wnd</th><th>Lm</th><th>n</th><th></th></tr></thead>
          <tbody>
            <tr v-for="m in diag.magnetics" :key="m.name" class="clickable" @click="openPart(m.name)">
              <td style="color: var(--amber-hi)">{{ m.name }}</td><td>{{ m.windings }}</td>
              <td>{{ si(m.magnetizingInductance, 'H') }}</td>
              <td class="dim">{{ m.turnsRatios?.map((r) => r.toPrecision(4)).join(' / ') || '—' }}</td>
              <td><span v-if="m.isMain" class="chip amber">main</span></td>
            </tr>
          </tbody>
        </table>
      </div>

      <!-- netlist -->
      <div v-else-if="view === 'netlist'">
        <div class="wave-toolbar">
          <select class="fld-in" style="width: auto" v-model="deckFlavor">
            <option value="ngspice">ngspice</option><option value="ltspice">LTspice</option>
          </select>
          <select class="fld-in" style="width: auto" v-model="deckFidelity">
            <option value="REQUIREMENTS">ideal</option><option value="DATASHEET">datasheet</option><option value="MKF_MODEL">MKF</option>
          </select>
          <span v-if="periodsShown" class="chip">≈ {{ periodsShown }} periods</span>
          <div class="sep"></div>
          <button class="btn ghost" :disabled="deckBusy" @click="makeDeck">Generate</button>
          <button class="btn ghost" :disabled="!deck" @click="copyDeck">Copy</button>
          <button class="btn ghost" :disabled="!deck" @click="downloadDeck">⭳</button>
        </div>
        <pre v-if="deck" class="deck">{{ deck }}</pre>
        <div v-else class="wave-empty">Generate a runnable ngspice / LTspice deck of this exact design.</div>
      </div>

      <!-- visual simulation: the CIAS-driven design animated in CircuitJS1 (Falstad), time-scaled -->
      <div v-else-if="view === 'visual'" class="visual-view">
        <template v-if="visualSim?.url">
          <div class="viz-toolbar">
            <label class="viz-scope-label">Scopes</label>
            <select class="fld-in viz-scope-sel" v-model="visualScopeSet">
              <option value="overview">Overview — switch · rectifier · output</option>
              <option value="magnetic">Magnetic — current + voltage over the magnetic</option>
              <option value="switch">Switch — V_ds + drain current</option>
              <option value="rectifier">Rectifier — diode/SR voltage + current</option>
              <option value="output">Output — voltage ripple + load current</option>
            </select>
          </div>
          <!-- Resizable so the whole sim (circuit + scopes) fits without CircuitJS1's own scrollbar;
               drag the handle bar below to grow/shrink it. Default height fits the page. -->
          <div class="falstad-wrap" :style="{ height: falstadH + 'px' }">
            <iframe class="falstad-frame" :src="visualSim.url" title="CircuitJS1 visual simulation"></iframe>
            <button class="viz-max-btn" @click="maximized = true" title="Maximize the simulation (Esc to close)">⛶ Maximize</button>
          </div>
          <div class="falstad-grip" :class="{ dragging: resizing }" @pointerdown="startResize"
               title="Drag to resize the simulation">
            <span class="grip-dots">⣿⣿⣿</span>
          </div>

          <!-- Maximize: a full-viewport overlay (teleported to body so the pane's fixed height / overflow
               can't clip it). This is what actually makes the sim bigger. -->
          <Teleport to="body">
            <div v-if="maximized" class="viz-overlay">
              <div class="viz-overlay-bar">
                <span class="viz-overlay-title">Visual sim — <b>{{ topo.name }}</b></span>
                <button class="viz-close-btn" @click="maximized = false">✕ Close (Esc)</button>
              </div>
              <iframe class="viz-overlay-frame" :src="visualSim.url" title="CircuitJS1 visual simulation (maximized)"></iframe>
            </div>
          </Teleport>
          <div class="sch-caption">
            Animated toy sim (CircuitJS1) generated from the same CIAS circuit as ngspice —
            time-scaled ×{{ Math.round(visualSim.scale) }} ({{ si(visualSim.fsw, 'Hz') }} → {{ si(visualSim.fVis, 'Hz') }},
            L·C rescaled so the waveforms are identical). For intuition only; the numbers come from ngspice.
            Use <b>⛶ Maximize</b> for a big view, or the handle below to nudge its height.
            <a :href="visualSim.url" target="_blank" rel="noopener">Open full simulator ↗</a>
          </div>
        </template>
        <div v-else-if="visualSim?.error" class="wave-empty">Visual sim unavailable: {{ visualSim.error }}</div>
        <div v-else-if="visualSim?.unsupported" class="wave-empty">
          No visual-sim layout for <code>{{ topo.name }}</code> yet — flyback first, more topologies coming.
        </div>
        <div v-else class="wave-empty">Solve a design to see the energy conversion in action.</div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.pane {
  display: flex; flex-direction: column; min-width: 0;
  height: 100%; min-height: 0;
  border: 1px solid var(--line); border-radius: 6px;
  background: linear-gradient(180deg, var(--panel-hi), var(--panel));
}
.pane-head {
  display: flex; align-items: center; gap: 0.5rem;
  padding: 0.4rem 0.5rem; border-bottom: 1px solid var(--line-soft);
}
.pane-select {
  font-family: var(--disp); font-size: 0.8rem; letter-spacing: 0.14em; text-transform: uppercase;
  color: var(--amber); background: rgba(255, 179, 71, 0.06);
  border: 1px solid var(--amber-deep); border-radius: 4px; padding: 0.25rem 0.6rem; cursor: pointer;
}
.pane-body { padding: 0.7rem; overflow: auto; flex: 1; min-height: 0; }
/* The visual view must NOT be height:100% of the (fixed-height) pane-body, or the resizable card gets
   shrunk to fit and the drag-handle has nothing to expand. Instead it owns its height and the pane-body
   scrolls if the card is taller than the pane. */
.visual-view { display: flex; flex-direction: column; }
.viz-toolbar { display: flex; align-items: center; gap: 0.5rem; margin-bottom: 0.5rem; }
.viz-scope-label { font-family: var(--disp); font-size: 0.72rem; letter-spacing: 0.12em; text-transform: uppercase; color: var(--amber-deep); }
.viz-scope-sel { width: auto; }
/* Resizable panel (height driven by JS via the grip below — the iframe covers the native resize grip).
   flex-shrink:0 keeps the set height authoritative within the flex column. */
.falstad-wrap {
  position: relative;
  width: 100%; flex-shrink: 0; overflow: hidden;
  border: 1px solid var(--line-soft); border-radius: 4px 4px 0 0;
}
/* Maximize button floating over the top-right of the sim. */
.viz-max-btn {
  position: absolute; top: 6px; right: 6px; z-index: 5;
  font-family: var(--disp); font-size: 0.72rem; letter-spacing: 0.08em;
  color: var(--amber); background: rgba(12, 12, 12, 0.82);
  border: 1px solid var(--amber-deep); border-radius: 4px; padding: 0.2rem 0.5rem; cursor: pointer;
}
.viz-max-btn:hover { background: rgba(255, 179, 71, 0.18); }
/* Full-viewport maximized overlay. */
.viz-overlay {
  position: fixed; inset: 0; z-index: 10000;
  display: flex; flex-direction: column; gap: 0.5rem;
  padding: 1.4vh 1.4vw; background: rgba(6, 6, 6, 0.94);
}
.viz-overlay-bar { display: flex; align-items: center; justify-content: space-between; }
.viz-overlay-title { font-family: var(--disp); letter-spacing: 0.1em; color: var(--amber); text-transform: uppercase; font-size: 0.85rem; }
.viz-overlay-title b { color: var(--amber-hi); }
.viz-close-btn {
  font-family: var(--disp); font-size: 0.78rem; letter-spacing: 0.08em;
  color: var(--amber); background: rgba(255, 179, 71, 0.06);
  border: 1px solid var(--amber-deep); border-radius: 4px; padding: 0.3rem 0.7rem; cursor: pointer;
}
.viz-close-btn:hover { background: rgba(255, 179, 71, 0.18); }
.viz-overlay-frame { flex: 1; width: 100%; border: 1px solid var(--line); border-radius: 4px; background: #0c0c0c; }
.falstad-frame { width: 100%; height: 100%; border: 0; display: block; background: #0c0c0c; }
/* Drag handle bar under the sim — the whole bar resizes (not just a tiny corner). */
.falstad-grip {
  flex-shrink: 0; height: 18px; display: flex; align-items: center; justify-content: center;
  cursor: ns-resize; user-select: none; touch-action: none;
  border: 1px solid var(--line-soft); border-top: 0; border-radius: 0 0 4px 4px;
  background: linear-gradient(180deg, var(--panel-hi), var(--panel));
  color: var(--amber-deep);
}
.falstad-grip:hover, .falstad-grip.dragging { color: var(--amber); background: rgba(255, 179, 71, 0.12); }
.grip-dots { font-size: 9px; letter-spacing: 2px; line-height: 1; }
.sch-error { color: var(--err, #ff6b6b); white-space: pre-wrap; }
.wave-name { font-size: 0.72rem; color: var(--amber-hi); margin-bottom: 0.3rem; }
</style>
