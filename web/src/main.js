import { createApp } from 'vue'
import App from './App.vue'
// Self-hosted fonts (was Google Fonts CDN — see security assessment N1). Only the
// weights the UI actually uses are bundled; Vite fingerprints + serves them same-origin.
import '@fontsource/share-tech-mono/400.css'
import '@fontsource/ibm-plex-sans/400.css'
import '@fontsource/ibm-plex-sans/500.css'
import '@fontsource/ibm-plex-sans/600.css'
import '@fontsource/ibm-plex-sans/700.css'
import '@fontsource/ibm-plex-mono/400.css'
import '@fontsource/ibm-plex-mono/500.css'
import '@fontsource/ibm-plex-mono/600.css'
import '@fontsource/ibm-plex-mono/700.css'
// The package stylesheet (every rule scoped under .kh-root — index.html puts that class on <html>, so
// the whole page is the bench) and the app's own page-level rules (body background, scrollbars).
import './style.css'
import './app.css'
import { initTelemetry, trackEvent } from './telemetry.js'
import { createKirchhoff } from './kh.js'

// Interaction telemetry (production-only; no-ops on localhost / the dev server).
// Umami website-id for kirchhoff.openconverters.com, registered in the shared OM
// Umami instance. Leave null until registered — Umami then no-ops while the
// same-origin /telemetry pipeline still records every event.
const UMAMI_WEBSITE_ID = '43a37de8-b618-4a51-bdb7-01140b2206f1' // kirchhoff.openconverters.com (OM Umami)
initTelemetry({ site: 'kirchhoff', umamiWebsiteId: UMAMI_WEBSITE_ID })
trackEvent('app_open')

// Origin of the OpenMagnetics app the magnetic-adviser handoff opens. Prod default; overridable for
// local dev via a ?om=<origin> query param on the KH page or window.__OPENMAGNETICS_ORIGIN__.
function resolveOmOrigin() {
  const q = new URLSearchParams(window.location.search).get('om')
  return q || window.__OPENMAGNETICS_ORIGIN__ || 'https://openmagnetics.com'
}

// The engine, with the paths this site serves it from: the WASM bundle and the Kelvin shards at the
// ORIGIN ROOT (not relative to the page), and product telemetry on.
const kh = createKirchhoff({
  wasmUrl: new URL('kirchhoff.js', window.location.origin + '/').href,
  kelvinUrl: '/kelvin',
  openMagneticsOrigin: resolveOmOrigin(),
  telemetry: true,
  // The engine worker, emitted by Vite as its own file next to the bundle.
  createWorker: () => new Worker(new URL('./worker.js', import.meta.url), { type: 'module' }),
})

createApp(App).use(kh).mount('#app')
