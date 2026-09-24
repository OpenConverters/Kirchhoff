import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// The reusable package (src/lib/index.js) as an ES module + one stylesheet, Vue left to the host:
//   npx vite build --config vite.lib.config.js   →  dist-lib/kirchhoff-vue.js, dist-lib/kirchhoff-vue.css
// The WASM engine is NOT bundled: the host serves kirchhoff.js and passes its URL to createKirchhoff().
// The engine worker is inlined into the module (src/lib/index.js imports it `?worker&inline`).
export default defineConfig({
  plugins: [vue()],
  base: './',                     // any emitted asset resolves relative to the module, not the host's root
  publicDir: false,               // public/ is the app's (WASM, Kelvin shards, CircuitJS1) — not the package's
  build: {
    outDir: 'dist-lib',
    emptyOutDir: true,
    lib: {
      entry: 'src/lib/index.js',
      formats: ['es'],
      fileName: () => 'kirchhoff-vue.js',
      cssFileName: 'kirchhoff-vue',
    },
    rollupOptions: { external: ['vue'] },
    chunkSizeWarningLimit: 1200,
  },
  worker: { format: 'es' },
})
