/**
 * Kirchhoff waveform widget — the web app's own WaveformChart, driven by a
 * component_waveforms tool result.
 *
 * Imports `web/src/components/WaveformChart.vue` directly, so the traces an
 * engineer reads in Claude are drawn by the same component (and the same dual
 * V/A axes) as on the web app.
 */
import { createApp, h, reactive } from "vue";
import { App } from "@modelcontextprotocol/ext-apps";
import "../../web/src/style.css";
import WaveformChart from "../../web/src/components/WaveformChart.vue";

const app = new App({ name: "Kirchhoff Waveforms", version: "0.1.0" });

const state = reactive({ title: "", subtitle: "", note: "", traces: [] });

/**
 * Server series are [[t, v], ...] point pairs; WaveformChart wants parallel time/data arrays
 * plus the unit that selects its axis.
 *
 * WHICH AXIS A TRACE BELONGS TO IS NOW SAID, NOT GUESSED. The payload is a `curves` result
 * under the pipeline contract: it declares `axes.y` and `axes.y2`, and every series names the
 * one it is measured against. This widget used to read a per-series `unit` string and treat
 * "A" as "the right-hand axis" — a convention that silently put the first unit nobody thought
 * of on the wrong scale.
 */
function toTrace(s, axes) {
  const time = [];
  const data = [];
  for (const [t, v] of s.points || []) {
    if (!Number.isFinite(t) || !Number.isFinite(v)) continue;
    time.push(t);
    data.push(v);
  }
  const axis = s.axis === "y2" ? axes?.y2 : axes?.y;
  return { label: s.name, unit: axis?.unit || "V", time, data };
}

const Root = {
  setup() {
    return () =>
      h("div", { class: "wrap" }, [
        h("h1", state.title || "Waveforms"),
        state.subtitle ? h("div", { class: "sub" }, state.subtitle) : null,
        state.note ? h("div", { class: "err" }, state.note) : null,
        state.traces.length
          ? h(WaveformChart, { traces: state.traces, height: 300 })
          : h("div", { class: "readout muted" }, "No traces in the tool result."),
      ]);
  },
};

createApp(Root).mount("#app");

app.ontoolresult = (result) => {
  const sc = result.structuredContent;
  if (sc?.mode !== "curves" || !Array.isArray(sc.series)) {
    state.title = "No waveform data in tool result.";
    return;
  }
  state.title = sc.title || "Waveforms";
  state.subtitle = sc.subtitle || "";
  state.note = sc.caveat || "";
  state.traces = sc.series.map((s) => toTrace(s, sc.axes)).filter((t) => t.data.length > 1);
};

await app.connect();
