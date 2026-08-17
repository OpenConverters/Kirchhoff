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
 * TWO payload shapes, both from the pipeline contract, because the two questions differ:
 *
 *   mode "curves"  — an AC sweep. axes.x is log frequency, axes.y is magnitude in dB, and
 *                    axes.y2 is PHASE in degrees when the sweep carries it. Each series names
 *                    the axis it belongs to; degrees and decibels are drawn on their own
 *                    scales, because a phase curve squeezed onto a dB range is unreadable and
 *                    a shared axis label would be a lie about one of them.
 *
 *   mode "verdict" — a filter attenuation check. Same axes and series, plus `exceedances`:
 *                    every frequency that breached the requirement, drawn where it happened.
 *                    Only breaches are listed, so a frequency with no dot passed.
 */
import { App } from "@modelcontextprotocol/ext-apps";

const app = new App({ name: "Kirchhoff Bode", version: "0.1.0" });

const state = { title: "", subtitle: "", note: "", xLabel: "frequency (Hz)", yLabel: "dB",
                y2Label: "", series: [], exceedances: [], tally: null, error: "" };

/** A series belongs to the second ordinate only if it says so. */
const onY2 = (s) => s.axis === "y2";
const axisLabel = (a) => (a ? [a.label, a.unit && `(${a.unit})`].filter(Boolean).join(" ") : "");

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

const W = 720, H = 340, PAD_L = 54, PAD_T = 12, PAD_B = 34;
// Room on the right only when a second ordinate is actually drawn there.
const padR = () => (state.y2Label ? 54 : 14);

function draw() {
  const pts = state.series.flatMap((s) => s.points || []);
  if (!pts.length) return el("div", { class: "readout muted" }, "No curve in the tool result.");
  const PAD_R = padR();

  // A frequency axis is logarithmic, so a non-positive frequency has no place on it — and
  // silently dropping it would shorten a sweep without saying so.
  const dropped = pts.filter(([f]) => !(f > 0) || !Number.isFinite(f)).length;
  const good = pts.filter(([f, v]) => f > 0 && Number.isFinite(f) && Number.isFinite(v));
  if (!good.length) return el("div", { class: "err" }, "No point had a positive frequency.");

  const fMin = Math.min(...good.map(([f]) => f));
  const fMax = Math.max(...good.map(([f]) => f));

  /** One vertical scale per ordinate: dB and degrees do not share a range. */
  function scaleFor(series) {
    const vs = series.flatMap((s) => (s.points || [])
      .filter(([f, v]) => f > 0 && Number.isFinite(f) && Number.isFinite(v))
      .map(([, v]) => v));
    if (!vs.length) return null;
    let lo = Math.min(...vs), hi = Math.max(...vs);
    if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.08;
    return { lo: lo - pad, hi: hi + pad };
  }

  const yScale = scaleFor(state.series.filter((s) => !onY2(s))) || { lo: -1, hi: 1 };
  const y2Scale = scaleFor(state.series.filter(onY2));

  const lx0 = Math.log10(fMin), lx1 = Math.log10(fMax);
  const X = (f) => PAD_L + (Math.log10(f) - lx0) / Math.max(lx1 - lx0, 1e-12) * (W - PAD_L - PAD_R);
  const on = (scale) => (v) =>
    PAD_T + (scale.hi - v) / (scale.hi - scale.lo) * (H - PAD_T - PAD_B);
  const Y = on(yScale);
  const Y2 = y2Scale ? on(y2Scale) : Y;

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
    const v = yScale.lo + (yScale.hi - yScale.lo) * i / 4;
    svg.append(svgEl("line", { x1: PAD_L, y1: Y(v), x2: W - PAD_R, y2: Y(v), class: "grid" }));
    const t = svgEl("text", { x: PAD_L - 6, y: Y(v) + 3.5, class: "tick", "text-anchor": "end" });
    t.textContent = trim(v);
    svg.append(t);
    if (y2Scale) {
      const v2 = y2Scale.lo + (y2Scale.hi - y2Scale.lo) * i / 4;
      const t2 = svgEl("text", { x: W - PAD_R + 6, y: Y2(v2) + 3.5, class: "tick",
                                 "text-anchor": "start" });
      t2.textContent = trim(v2);
      svg.append(t2);
    }
  }

  state.series.forEach((s, i) => {
    const p = (s.points || []).filter(([f, v]) => f > 0 && Number.isFinite(v));
    if (p.length < 2) return;
    const y = onY2(s) ? Y2 : Y;
    const d = p.map(([f, v], j) => `${j ? "L" : "M"}${X(f).toFixed(2)},${y(v).toFixed(2)}`).join("");
    // A LIMIT line is a threshold, not data: it is drawn differently because reading one as
    // the other is how a design gets signed off against the wrong curve.
    svg.append(svgEl("path", { d, class: `line s${i % 4}${s.kind === "limit" ? " required" : ""}`
                               + (onY2(s) ? " y2" : "") }));
  });

  // Every breach, drawn where it happened. Only exceedances are listed — a frequency with no
  // dot met the requirement, which is why nothing here says "pass".
  for (const e of state.exceedances) {
    if (!(e.at > 0) || !Number.isFinite(e.value)) continue;
    svg.append(svgEl("circle", { cx: X(e.at), cy: Y(e.value), r: 3.6, class: "mk fail" }));
  }

  const xt = svgEl("text", { x: (PAD_L + W - PAD_R) / 2, y: H - 4, class: "axis",
                             "text-anchor": "middle" });
  xt.textContent = state.xLabel;
  svg.append(xt);
  const mid = (PAD_T + H - PAD_B) / 2;
  const yt = svgEl("text", { x: 12, y: mid, class: "axis", "text-anchor": "middle",
                             transform: `rotate(-90 12 ${mid})` });
  yt.textContent = state.yLabel;
  svg.append(yt);
  if (y2Scale && state.y2Label) {
    const y2t = svgEl("text", { x: W - 8, y: mid, class: "axis", "text-anchor": "middle",
                                transform: `rotate(90 ${W - 8} ${mid})` });
    y2t.textContent = state.y2Label;
    svg.append(y2t);
  }

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
      el("span", { class: `key s${i % 4}${s.kind === "limit" ? " required" : ""}` },
         onY2(s) ? `${s.name} (right)` : s.name))));
  }
  if (state.tally) {
    const { passed, tested, unmeasured } = state.tally;
    root.append(el("div", { class: "hint" },
      `${passed}/${tested} frequencies meet the requirement — hollow dots are the breaches`
      + (unmeasured ? `; ${unmeasured} returned no measurement and are neither` : "") + "."));
  }
}

app.ontoolresult = (result) => {
  const sc = result?.structuredContent;
  if (!Array.isArray(sc?.series) || (sc.mode !== "curves" && sc.mode !== "verdict")) {
    state.error = "No frequency-domain data in the tool result.";
    render();
    return;
  }
  const axes = sc.axes || {};
  state.title = sc.title || (sc.mode === "verdict" ? sc.criterion : "Frequency response");
  state.subtitle = sc.subtitle || (sc.mode === "verdict" ? `verdict: ${sc.verdict}` : "");
  state.note = sc.caveat || "";
  state.xLabel = axisLabel(axes.x) || "frequency (Hz)";
  state.yLabel = axisLabel(axes.y) || "dB";
  state.y2Label = axisLabel(axes.y2);
  state.series = sc.series;
  state.exceedances = Array.isArray(sc.exceedances) ? sc.exceedances : [];
  const m = sc.measurements || {};
  state.tally = sc.mode === "verdict" && m.frequenciesTested
    ? { passed: m.frequenciesPassed?.value ?? 0, tested: m.frequenciesTested.value,
        unmeasured: m.frequenciesUnmeasured?.value ?? 0 }
    : null;
  state.error = "";
  render();
};

render();
await app.connect();
