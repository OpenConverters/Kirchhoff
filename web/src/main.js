import { createApp } from 'vue'
import App from './App.vue'
import './style.css'
import { initTelemetry, trackEvent } from './telemetry.js'

// Interaction telemetry (production-only; no-ops on localhost / the dev server).
// Umami website-id for kirchhoff.openconverters.com, registered in the shared OM
// Umami instance. Leave null until registered — Umami then no-ops while the
// same-origin /telemetry pipeline still records every event.
const UMAMI_WEBSITE_ID = '43a37de8-b618-4a51-bdb7-01140b2206f1' // kirchhoff.openconverters.com (OM Umami)
initTelemetry({ site: 'kirchhoff', umamiWebsiteId: UMAMI_WEBSITE_ID })
trackEvent('app_open')

createApp(App).mount('#app')
