// @openconverters/kirchhoff-vue — Kirchhoff's converter bench as reusable Vue 3 pieces.
//
//   import { createKirchhoff, KhConverterForm, KhSchematic, KhWaveforms } from '@openconverters/kirchhoff-vue'
//   import '@openconverters/kirchhoff-vue/kirchhoff-vue.css'      // dist-lib/kirchhoff-vue.css
//
//   const kh = createKirchhoff({ wasmUrl: '/assets/kirchhoff.js' })     // kelvinUrl: null → no part sourcing
//   createApp(App).use(kh).mount('#app')
//
//   <div class="kh-root">                                               <!-- every rule is scoped under it -->
//     <KhConverterForm @solved="(r) => (design = r)" />
//     <KhSchematic v-if="design" topology="flyback" :result="design"
//                  :selectable="(c) => c.kind === 'Capacitor'" :selected-ref="picked" @select="(r) => (picked = r)" />
//     <KhWaveforms v-if="picked" :component-ref="picked" :tas="design.tas" />
//   </div>
//
// Importing this module has no side effects: no worker, no fetch, no telemetry, no window globals.
// The stylesheet declares nothing outside `.kh-root` except its own @font-face and @keyframes names.
import '../style.css'
import './themes/wurth.css'

import { createKirchhoff as createEngine } from '../kh.js'
// The worker is inlined into the package (a blob: worker, created with the page's origin), so a host's
// bundler has no worker file to find, copy or — as Vite does with small assets — turn into a data: URL,
// whose opaque origin could not import the engine.
import InlineEngineWorker from '../worker.js?worker&inline'

// createKirchhoff({ wasmUrl, kelvinUrl = null, telemetry = false, openMagneticsOrigin, createWorker })
// — see src/kh.js. Here createWorker defaults to the inlined worker.
export function createKirchhoff(options = {}) {
  return createEngine({ createWorker: () => new InlineEngineWorker(), ...options })
}
export { kelvinCategoryFor, KIRCHHOFF_KEY } from '../kh.js'
export { useKirchhoff } from './context.js'
export { useConverterForm } from './converterForm.js'

export { default as KhConverterForm } from './KhConverterForm.vue'
export { default as KhSchematic } from './KhSchematic.vue'
export { default as KhWaveforms } from './KhWaveforms.vue'
// The building blocks the three are made of, for hosts that compose their own views.
export { default as WaveformChart } from '../components/WaveformChart.vue'
export { default as WavePane } from '../components/WavePane.vue'
export { default as FamilyDial } from '../components/FamilyDial.vue'

export {
  FAMILIES, FAMILY_SHORT, TOPOLOGIES, PLANNED, VARIANTS, KNOBS, KNOB_TIERS,
  topologyById, variantAxis, defaultVariant, knobsFor, knobGroups, buildSpec,
} from '../topologies.js'
export { extractBom, requirementRows } from '../bom.js'
export { renderVerifiedSchematic, hasCiasSchematic } from '../ciasSchematic.js'
export { symbols, withPinRecording } from '../schematics.js'
export {
  resolveExcitations, magneticSignals, componentSignals, designSignals, toCsv, designExcitationsJson, stripNulls, tile,
} from '../waveExport.js'
export { si, pct } from '../units.js'
