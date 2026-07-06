import { defineConfig, devices } from '@playwright/test'

// Kirchhoff topology-bench e2e. HEADLESS ALWAYS (house rule — never --headed).
// The app is a static SPA; `vite preview` serves the production build (which runs
// sync-wasm, copying the freshly built kirchhoff.js from ../build-wasm-ng). A missing
// WASM build fails the build step loudly — the tests need the real engine, not a stub.
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
    baseURL: 'http://localhost:4173',
    headless: true,
    trace: 'retain-on-failure',
  },
  projects: [
    { name: 'smoke', testMatch: /smoke\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    { name: 'knobs', testMatch: /(serialization|physics)\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    { name: 'kelvin', testMatch: /kelvin\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
    { name: 'visualsim', testMatch: /visualsim\.spec\.js/, use: { ...devices['Desktop Chrome'] } },
  ],
  webServer: {
    command: 'npm run build && npm run preview -- --port 4173',
    url: 'http://localhost:4173',
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
})
