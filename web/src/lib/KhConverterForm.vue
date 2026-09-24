<script setup>
// Topology picker + spec form + Solve, as the Kirchhoff bench draws it: a three-stage fold
// (Topology ▸ Variant ▸ Specification & Simulation) with the family dial, the per-topology knobs and the
// operating-point table. Emits `solved` with the engine's converter result.
//
// Props
//   state   a useConverterForm() object when the host owns the form state; omitted → the component
//           creates its own (with `engine`/`topology`).
//   engine  a createKirchhoff() instance (default: the injected one).
//   topology  initial topology id when the component owns its state (default 'flyback').
//   solver  async () => void. Replaces the default solve (one processConverter call); it is expected to
//           drive state.running / state.result / state.runError itself.
// Events
//   solved(result)   after a solve that produced a result (result = processConverter's output)
//   error(message)   after a solve that failed
import { onMounted } from 'vue'
import { FAMILIES, FAMILY_SHORT } from '../topologies.js'
import FamilyDial from '../components/FamilyDial.vue'
import { useConverterForm } from './converterForm.js'

const props = defineProps({
  state: { type: Object, default: null },
  engine: { type: Object, default: null },
  topology: { type: String, default: 'flyback' },
  solver: { type: Function, default: null },
})
const emit = defineEmits(['solved', 'error'])

const s = props.state ?? useConverterForm({ engine: props.engine, topology: props.topology })
const {
  engineState, running, runError, result, topo, topoId, topoTouched, variantTouched, form, family, familyTopologies,
  stage, axis, variantOptions, currentVariant, hasVariantChoice, advGroups, hasKnobs, knobPlaceholder,
  pickTopology, pickVariant, addOutput, removeOutput, addOp,
} = s

// Header clicks and the dial write the shared state's refs directly (they are the host's too).
const openStage = (id) => { s.stage.value = id }
const setFamily = (f) => { s.family.value = f }

onMounted(() => s.bootEngine())

async function onSolve() {
  if (props.solver) await props.solver()
  else await s.solve()
  if (runError.value) emit('error', runError.value)
  else if (result.value) emit('solved', result.value)
}
</script>

<template>
  <!-- three-stage terminal fold: Topology ▸ Variant ▸ Spec. One open at a time; a header re-opens its stage. -->
  <div class="acc">
    <!-- Stage 1 — Topology -->
    <section class="acc-stage" :class="{ open: stage === 'topology' }">
      <button class="acc-head" data-testid="stage-topology" @click="openStage('topology')">
        <span class="idx">1</span><span class="acc-title">Topology</span>
        <span class="acc-pick">{{ topo.name }}</span><span class="acc-chev">▸</span>
      </button>
      <div class="acc-fold"><div class="acc-inner"><div class="acc-pad">
        <FamilyDial :model-value="family" @update:model-value="setFamily" :families="FAMILIES" :short="FAMILY_SHORT" />
        <div class="topo-list">
          <button
            v-for="t in familyTopologies" :key="t.id" :data-testid="`topo-${t.id}`"
            class="topo-card" :class="{ active: topoTouched && t.id === topoId, planned: t.planned }"
            :disabled="t.planned" @click="pickTopology(t.id)"
          >
            <div class="t-name">{{ t.name }}<span v-if="t.planned" class="t-tag">planned</span></div>
            <div class="t-desc">{{ t.desc }}</div>
          </button>
        </div>
      </div></div></div>
    </section>

    <!-- Stage 2 — Variant (only when the topology actually has a choice) -->
    <section v-if="hasVariantChoice" class="acc-stage" :class="{ open: stage === 'variant' }">
      <button class="acc-head" @click="openStage('variant')">
        <span class="idx">2</span><span class="acc-title">{{ axis.label }}</span>
        <span class="acc-pick">{{ currentVariant.name }}</span><span class="acc-chev">▸</span>
      </button>
      <div class="acc-fold"><div class="acc-inner"><div class="acc-pad">
        <div class="variant-list">
          <button
            v-for="v in variantOptions" :key="v.id"
            class="topo-card variant-card" :class="{ active: variantTouched && v.id === form.variant }"
            @click="pickVariant(v.id)"
          >
            <div class="t-name">{{ v.name }}</div>
            <div class="t-desc">{{ v.desc }}</div>
          </button>
        </div>
      </div></div></div>
    </section>

    <!-- Stage 3 — Specification & Simulation -->
    <section class="acc-stage" :class="{ open: stage === 'spec' }">
      <button class="acc-head" data-testid="stage-spec" @click="openStage('spec')">
        <span class="idx">{{ hasVariantChoice ? '3' : '2' }}</span><span class="acc-title">Specification &amp; Simulation</span>
        <span class="acc-chev">▸</span>
      </button>
      <div class="acc-fold"><div class="acc-inner"><div class="acc-pad">
  <div class="grid2">
    <label class="fld" v-if="form.inputType === 'dc'">
      <span class="fld-label">Vin min <span class="u">V</span></span>
      <input class="fld-in" type="number" v-model.number="form.vinMin" placeholder="opt" />
    </label>
    <label class="fld">
      <span class="fld-label">{{ form.inputType === 'dc' ? 'Vin nom' : 'Vac rms' }} <span class="u">V</span></span>
      <input class="fld-in" type="number" v-model.number="form.vinNom" />
    </label>
    <label class="fld" v-if="form.inputType === 'dc'">
      <span class="fld-label">Vin max <span class="u">V</span></span>
      <input class="fld-in" type="number" v-model.number="form.vinMax" placeholder="opt" />
    </label>
    <label class="fld" v-else>
      <span class="fld-label">Line freq <span class="u">Hz</span></span>
      <input class="fld-in" type="number" v-model.number="form.lineFrequency" />
    </label>
    <label class="fld">
      <span class="fld-label">Switching <span class="u">Hz</span></span>
      <input class="fld-in" type="number" v-model.number="form.fs" step="1000" />
    </label>
  </div>

  <table class="row-table" style="margin-top: 0.6rem">
    <thead><tr><th>Out</th><th>V</th><th>W</th><th></th></tr></thead>
    <tbody>
      <tr v-for="(o, i) in form.outputs" :key="i">
        <td><input class="fld-in" v-model="o.name" /></td>
        <td><input class="fld-in" type="number" v-model.number="o.voltage" /></td>
        <td><input class="fld-in" type="number" v-model.number="o.power" @change="form.ops.forEach((op) => (op.powers[i] = o.power))" /></td>
        <td><button v-if="form.outputs.length > form.minOutputs" class="row-btn" @click="removeOutput(i)">×</button></td>
      </tr>
    </tbody>
  </table>
  <button v-if="form.outputs.length < form.maxOutputs" class="row-btn" style="margin-top: 0.3rem" @click="addOutput">+ output</button>
  <span v-if="form.minOutputs > 1" class="chip" style="margin-left: 0.5rem">needs ≥ {{ form.minOutputs }}</span>

  <div class="section-label" style="margin-top: 1rem">Simulation</div>
  <div class="grid2">
    <label class="fld">
      <span class="fld-label">Engine</span>
      <select class="fld-in" v-model="form.engine">
        <option value="ngspice">ngspice</option>
        <option value="analytical">analytical</option>
      </select>
    </label>
    <label class="fld">
      <span class="fld-label">Models</span>
      <select class="fld-in" v-model="form.models"
              title="ideal switches, or datasheet-derived real-conduction models (real Rds(on) / forward drop)">
        <!-- Global semis/passives models. MAGNETIC model choice (ideal/datasheet/MKF) is
             per-component — click the magnetic and pick its Simulation model in the drawer. -->
        <option value="ideal">ideal</option>
        <option value="datasheet">datasheet</option>
      </select>
    </label>
    <label class="fld" v-if="form.inputType === 'dc'">
      <span class="fld-label">Settle cyc</span>
      <input class="fld-in" type="number" min="1" step="10" v-model.number="form.settlePeriods"
             title="switching cycles the transient settles before the shown window" />
    </label>
    <label class="fld">
      <span class="fld-label">Cyc shown</span>
      <input class="fld-in" type="number" min="1" max="50" v-model.number="form.showPeriods" />
    </label>
  </div>

  <details class="adv">
    <summary>Advanced — efficiency, isolation, operating points</summary>
    <div class="adv-body">
      <div class="grid2">
        <label class="fld">
          <span class="fld-label">Efficiency <span class="u">0–1</span></span>
          <input class="fld-in" type="number" step="0.01" min="0.5" max="1" v-model.number="form.efficiency" />
        </label>
        <label class="fld">
          <span class="fld-label">Ambient <span class="u">°C</span></span>
          <input class="fld-in" type="number" v-model.number="form.ambient" />
        </label>
        <label class="fld">
          <span class="fld-label">Isolation <span class="u">V</span></span>
          <input class="fld-in" type="number" v-model.number="form.isolation" placeholder="none" />
        </label>
      </div>
      <table class="row-table" style="margin-top: 0.6rem">
        <thead>
          <tr><th>OP</th><th>Vin</th><th>°C</th><th v-for="(o, i) in form.outputs" :key="i">{{ o.name }} W</th><th></th></tr>
        </thead>
        <tbody>
          <tr v-for="(op, j) in form.ops" :key="j">
            <td><input class="fld-in" v-model="op.name" /></td>
            <td><input class="fld-in" type="number" v-model.number="op.vin" /></td>
            <td><input class="fld-in" type="number" v-model.number="op.ambient" /></td>
            <td v-for="(o, i) in form.outputs" :key="i"><input class="fld-in" type="number" v-model.number="op.powers[i]" /></td>
            <td><button v-if="form.ops.length > 1" class="row-btn" @click="form.ops.splice(j, 1)">×</button></td>
          </tr>
        </tbody>
      </table>
      <button class="row-btn" style="margin-top: 0.3rem" @click="addOp">+ operating point</button>
    </div>
  </details>

  <details v-if="hasKnobs" class="adv" data-testid="knobs-fold">
    <summary>Topology knobs — override auto-designed parameters</summary>
    <div class="adv-body">
      <div v-for="g in advGroups" :key="g.id" class="knob-group" :data-testid="`knob-group-${g.id}`">
        <div class="section-label">{{ g.label }}</div>
        <div v-for="k in g.knobs" :key="k.key" class="knob-row">
          <label class="knob-toggle" :title="k.tip">
            <input type="checkbox" v-model="form.knobs[k.key].on" :data-testid="`knob-${k.key}-auto`" />
            <span class="knob-name">{{ k.label }}</span>
            <span v-if="k.sym" class="knob-sym">{{ k.sym }}</span>
            <span v-if="k.unit" class="u">{{ k.unit }}</span>
          </label>
          <div class="knob-ctl">
            <template v-if="form.knobs[k.key].on">
              <select v-if="k.type === 'enum'" class="fld-in" v-model="form.knobs[k.key].value" :data-testid="`knob-${k.key}-input`">
                <option v-for="o in k.options" :key="o.id" :value="o.id">{{ o.name }}</option>
              </select>
              <label v-else-if="k.type === 'bool'" class="knob-bool">
                <input type="checkbox" v-model="form.knobs[k.key].value" :data-testid="`knob-${k.key}-input`" />
                <span>{{ form.knobs[k.key].value ? 'on' : 'off' }}</span>
              </label>
              <input v-else class="fld-in" type="number" :step="k.step ?? 'any'" :min="k.min" :max="k.max"
                     v-model.number="form.knobs[k.key].value" :data-testid="`knob-${k.key}-input`" />
            </template>
            <span v-else class="knob-auto">auto · {{ knobPlaceholder(k) }}</span>
          </div>
        </div>
      </div>
    </div>
  </details>

        <div class="solve-row">
          <button class="btn solve-btn" data-testid="solve" :disabled="running || engineState !== 'ready'" @click="onSolve">
            {{ running ? (form.engine === 'ngspice' ? 'Simulating…' : 'Solving…') : 'Solve' }}
          </button>
          <div v-if="engineState === 'loading'" class="boot"><div class="spin"></div> loading…</div>
          <span v-else-if="result" class="chip ok">ready</span>
          <span v-else class="hint" style="font-size: 0.62rem">set spec ▸ solve</span>
        </div>
        <div v-if="runError" class="err-banner" data-testid="error-banner" style="margin-top: 0.5rem"><b>ENGINE ▸</b> {{ runError }}</div>
      </div></div></div>
    </section>
  </div>
</template>
