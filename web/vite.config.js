import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// Static SPA: everything (including the WASM engine) is served as plain files,
// so the built dist/ can be hosted anywhere (GitHub Pages, nginx, file://-adjacent).
export default defineConfig({
  base: './',
  plugins: [vue()],
  build: {
    // kirchhoff.js is ~7.7 MB (the whole simulation engine); it is loaded from
    // public/ at runtime, not bundled, so keep rollup quiet about chunk size.
    chunkSizeWarningLimit: 1200,
  },
})
