/**
 * Kirchhoff frequency-domain widget — the MCP App.
 *
 * The waveform widget next door is the web app's WaveformChart: dual V/A axes against TIME.
 * Two results here are neither — an AC sweep and a filter attenuation check are dB against a
 * LOG frequency axis — and feeding them to it renders something mislabelled, which is worse
 * than not rendering, because a reader cannot tell a mislabelled axis from a wrong result
 * (ABT #689).
 *
 * Written as plain SVG rather than importing a chart from another repo: Kirchhoff's web app
 * has no log chart to reuse, and a copy of Hertz's would be a third cross-repo widget copy.
 *
 * Payload:
 *   { title, subtitle, x_label, y_label, note?,
 *     series:  [{ name, points: [[f_hz, dB], …], kind?: "line"|"required"|"points" }],
 *     markers: [{ f_hz, value, ok }]   — pass/fail per frequency, drawn on top }
 */
import { App } from "@modelcontextprotocol/ext-apps";

const app = new App({ name: "Kirchhoff Bode", version: "0.1.0" });

const state = { title: "", subtitle: "", note: "", xLabel: "Hz", yLabel: "dB",
                series: [], markers: [], error: "" };

const NS = "http://www.w3.org/2000/svg";
const svgEl = (tag, attrs = {}) => {
  const n = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v !== null && v !== undefined) n.setAttribute(k, String(v));
  }
  return n;
};
const el = (tag, attrs = {}, ...kids) => {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") n.className = v;
    else if (v !== null && v !== undefined) n.setAttribute(k, v);
  }
  for (const kid of kids.flat()) {
    if (kid === null || kid === undefined || kid === false) continue;
    n.append(kid instanceof Node ? kid : document.createTextNode(String(kid)));
  }
  return n;
};

/** Engineering-notation frequency label: 150 k, 1 M, 30 M. */
function fLabel(f) {
  if (f >= 1e9) return `${trim(f / 1e9)} G`;
  if (f >= 1e6) return `${trim(f / 1e6)} M`;
  if (f >= 1e3) return `${trim(f / 1e3)} k`;
  return trim(f);
}
const trim = (x) => String(Number(Number(x).toPrecision(3)));

const W = 720, H = 340, PAD_L = 54, PAD_R = 14, PAD_T = 12, PAD_B = 34;

function draw() {
  const pts = state.series.flatMap((s) => s.points || []);
  if (!pts.length) return el("div", { class: "readout muted" }, "No curve in the tool result.");

  // A frequency axis is logarithmic, so a non-positive frequency has no place on it — and
  // silently dropping it would shorten a sweep without saying so.
  const dropped = pts.filter(([f]) => !(f > 0) || !Number.isFinite(f)).length;
  const good = pts.filter(([f, v]) => f > 0 && Number.isFinite(f) && Number.isFinite(v));
  if (!good.length) return el("div", { class: "err" }, "No point had a positive frequency.");

  const fMin = Math.min(...good.map(([f]) => f));
  const fMax = Math.max(...good.map(([f]) => f));
  let vMin = Math.min(...good.map(([, v]) => v));
  let vMax = Math.max(...good.map(([, v]) => v));
  if (vMax - vMin < 1e-9) { vMin -= 1; vMax += 1; }
  const pad = (vMax - vMin) * 0.08;
  vMin -= pad; vMax += pad;

  const lx0 = Math.log10(fMin), lx1 = Math.log10(fMax);
  const X = (f) => PAD_L + (Math.log10(f) - lx0) / Math.max(lx1 - lx0, 1e-12) * (W - PAD_L - PAD_R);
  const Y = (v) => PAD_T + (vMax - v) / (vMax - vMin) * (H - PAD_T - PAD_B);

  const svg = svgEl("svg", { viewBox: `0 0 ${W} ${H}`, class: "chart",
                             role: "img", "aria-label": state.title || "frequency response" });

  // decade gridlines — the reason to use a log axis at all is that decades are even
  for (let d = Math.floor(lx0); d <= Math.ceil(lx1); d++) {
    const f = 10 ** d;
    if (f < fMin * 0.999 || f > fMax * 1.001) continue;
    svg.append(svgEl("line", { x1: X(f), y1: PAD_T, x2: X(f), y2: H - PAD_B, class: "grid" }));
    const t = svgEl("text", { x: X(f), y: H - PAD_B + 14, class: "tick", "text-anchor": "middle" });
    t.textContent = fLabel(f);
    svg.append(t);
  }
  for (let i = 0; i <= 4; i++) {
    const v = vMin + (vMax - vMin) * i / 4;
    svg.append(svgEl("line", { x1: PAD_L, y1: Y(v), x2: W - PAD_R, y2: Y(v), class: "grid" }));
    const t = svgEl("text", { x: PAD_L - 6, y: Y(v) + 3.5, class: "tick", "text-anchor": "end" });
    t.textContent = trim(v);
    svg.append(t);
  }

  state.series.forEach((s, i) => {
    const p = (s.points || []).filter(([f, v]) => f > 0 && Number.isFinite(v));
    if (p.length < 2) return;
    const d = p.map(([f, v], j) => `${j ? "L" : "M"}${X(f).toFixed(2)},${Y(v).toFixed(2)}`).join("");
    svg.append(svgEl("path", { d, class: `line s${i % 4}${s.kind === "required" ? " required" : ""}` }));
  });

  // The verdict, drawn where it happened: a filled dot passed, a hollow one did not.
  for (const m of state.markers) {
    if (!(m.f_hz > 0) || !Number.isFinite(m.value)) continue;
    svg.append(svgEl("circle", { cx: X(m.f_hz), cy: Y(m.value), r: 3.6,
                                 class: m.ok ? "mk pass" : "mk fail" }));
  }

  const xt = svgEl("text", { x: (PAD_L + W - PAD_R) / 2, y: H - 4, class: "axis",
                             "text-anchor": "middle" });
  xt.textContent = state.xLabel;
  svg.append(xt);
  const yt = svgEl("text", { x: 12, y: (PAD_T + H - PAD_B) / 2, class: "axis",
                             "text-anchor": "middle",
                             transform: `rotate(-90 12 ${(PAD_T + H - PAD_B) / 2})` });
  yt.textContent = state.yLabel;
  svg.append(yt);

  const wrap = el("div", {}, svg);
  if (dropped) {
    wrap.append(el("div", { class: "hint" },
      `${dropped} point(s) had no positive frequency and are not on a log axis.`));
  }
  return wrap;
}

function render() {
  const root = document.getElementById("app");
  root.textContent = "";
  if (state.error) { root.append(el("div", { class: "err" }, state.error)); return; }
  root.append(el("h1", {}, state.title || "Frequency response"));
  if (state.subtitle) root.append(el("div", { class: "sub" }, state.subtitle));
  if (state.note) root.append(el("div", { class: "err" }, state.note));
  root.append(draw());
  if (state.series.length) {
    root.append(el("div", { class: "legend" }, state.series.map((s, i) =>
      el("span", { class: `key s${i % 4}${s.kind === "required" ? " required" : ""}` }, s.name))));
  }
  const pass = state.markers.filter((m) => m.ok).length;
  if (state.markers.length) {
    root.append(el("div", { class: "hint" },
      `${pass}/${state.markers.length} frequencies meet the requirement — filled dots pass.`));
  }
}

app.ontoolresult = (result) => {
  const sc = result?.structuredContent;
  if (!sc || !Array.isArray(sc.series)) {
    state.error = "No frequency-domain data in the tool result.";
    render();
    return;
  }
  state.title = sc.title || "Frequency response";
  state.subtitle = sc.subtitle || "";
  state.note = sc.note || "";
  state.xLabel = sc.x_label || "Hz";
  state.yLabel = sc.y_label || "dB";
  state.series = sc.series;
  state.markers = Array.isArray(sc.markers) ? sc.markers : [];
  state.error = "";
  render();
};

render();
await app.connect();
