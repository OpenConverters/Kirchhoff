import { defineConfig, devices } from '@playwright/test'

// Kirchhoff topology-bench e2e. HEADLESS ALWAYS (house rule — never --headed).
// The app is a static SPA; `vite preview` serves the production build (which runs
// sync-wasm, copying the freshly built kirchhoff.js from ../build-wasm-ng). A missing
// WASM build fails the build step loudly — the tests need the real engine, not a stub.
// Port is overridable because 4173 is Vite's default and therefore collides with every other Vite
// preview on the machine: `KH_E2E_PORT=4183 npx playwright test`.
const PORT = Number(process.env.KH_E2E_PORT ?? 4173)
const BASE = `http://localhost:${PORT}`

export default defineConfig({
  testDir: './tests/e2e',
  // WASM ngspice transients run for seconds; give each test room but keep it bounded.
  timeout: 120_000,
  expect: { timeout: 15_000 },
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  // One shared preview server + WASM-loading browsers: too many parallel workers contend and cause
  // DOM-render races. 3 balances speed against flakiness (retries absorb the rest).
  workers: 3,
  // A rare DOM-render race under many parallel workers on the shared preview server gets one retry;
  // a real assertion failure still fails (it fails on the retry too). Not a mask for product bugs.
  retries: process.env.CI ? 2 : 1,
  reporter: [['list'], ['html', { open: 'never' }]],
  use: {
    baseURL: BASE,
    headless: true,
    trace: 'retain-on-failure',
  },
  projects: [
    { name: 'smoke', testMatch: /smoke\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    { name: 'knobs', testMatch: /(serialization|physics)\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    { name: 'kelvin', testMatch: /kelvin\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    { name: 'visualsim', testMatch: /visualsim\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    // Measures the schematic in the LIVE app (real stylesheet, real fonts) — every other schematic
    // gate measures a reconstruction of it.
    { name: 'schematic', testMatch: /schematic\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    // ...and again in a NARROW window. SVG text is sized in CSS px, so it does NOT scale with the
    // drawing: the smaller the pane, the larger every label is relative to the circuit (measured up to
    // +14% in user units at 800 px). A layout tuned at one width is therefore not proven at another.
    { name: 'schematic-narrow', testMatch: /schematic\.spec\.js/,
      use: { ...devices['Desktop Chrome'], viewport: { width: 900, height: 900 } } },
    // The schematic is also the app's navigation surface: a REAL pointer click on a part must open that
    // part's drawer. Every other schematic gate measures the picture, never the click.
    { name: 'hotspot', testMatch: /hotspot\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
  ],
  webServer: {
    command: `npm run build && npm run preview -- --port ${PORT}`,
    // Point the health check at an asset only THIS app serves, not at `/`. reuseExistingServer means
    // Playwright attaches to whatever already answers on the port — and something else on this machine
    // listens on 4173 (it redirects to /el-magnetic/). A `/` check accepts that redirect happily, so the
    // whole suite attached to a foreign app and reported 193 timeouts instead of "wrong server". Asking
    // for the WASM bundle makes a foreign server fail the check and the run stop with a server error.
    url: `${BASE}/kirchhoff.js`,
    reuseExistingServer: !process.env.CI,
    timeout: 180_000,
  },
})
