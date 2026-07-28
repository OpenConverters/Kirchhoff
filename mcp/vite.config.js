import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";
import { viteSingleFile } from "vite-plugin-singlefile";

// MCP App resources render in a deny-by-default CSP iframe, so each widget must
// be ONE self-contained file: no external script/style/font requests.
//
// The Vue plugin is here so the widgets can import the web app's real modules
// (ciasSchematic.js, WaveformChart.vue) straight out of ../web/src instead of
// reimplementing them — the schematic an engineer clicks in Claude is drawn by
// the same generator, and verified against the same CIAS netlist, as the one on
// the web app. One definition, two surfaces.
//
// vite-plugin-singlefile inlines a single entry per build, so widgets are built
// one at a time via INPUT and emptyOutDir is off (the second build must not
// wipe the first).
export default defineConfig({
  plugins: [vue(), viteSingleFile()],
  build: {
    outDir: "dist",
    emptyOutDir: false,
    rollupOptions: { input: process.env.INPUT || "schematic.html" },
  },
});
