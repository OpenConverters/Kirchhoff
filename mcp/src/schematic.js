/**
 * Kirchhoff schematic widget — the web app's own CIAS-generated drawing, driven
 * by a design_converter tool result.
 *
 * This imports `web/src/ciasSchematic.js` directly rather than reimplementing
 * anything, so the schematic in Claude is generated from the same CIAS bricks
 * the ngspice deck is generated from AND verified against the same netlist
 * checker. If the drawing would disagree with the circuit, this throws instead
 * of rendering a wrong picture.
 *
 * Clicking a component reports the selection to the model and asks the server
 * for real catalogue parts for it — a widget-initiated tool call, no LLM turn.
 */
import { createApp, h, reactive } from "vue";
import { App } from "@modelcontextprotocol/ext-apps";
// The web app's design tokens (--ink, --amber, --cyan, ...). The schematic
// symbols bind them directly, so without this its strokes resolve to nothing.
import "../../web/src/style.css";
import { renderVerifiedSchematic } from "../../web/src/ciasSchematic.js";
import { extractBom } from "../../web/src/bom.js";
import { hasVisualSim, falstadExport } from "../../web/src/falstad.js";

const app = new App({ name: "Kirchhoff Schematic", version: "0.1.0" });

const state = reactive({
  topology: "",
  summary: "",
  note: "",
  svg: "",
  error: "",
  bom: [],
  selected: "",
  candidates: null,
  bound: {},
  falstadUrl: "",
  busy: false,
  tas: null,
});

/**
 * Strip Vue reactivity before anything crosses postMessage.
 *
 * `reactive()` hands back Proxy objects and the structured-clone algorithm
 * refuses to clone a Proxy — the bridge fails with "[object Object] could not
 * be cloned", which reads like a protocol error but is purely local.
 */
const plain = (v) => JSON.parse(JSON.stringify(v));

/**
 * Push to model context, always carrying the standing facts.
 *
 * updateModelContext OVERWRITES rather than appending — that is what keeps forty
 * clicks costing one line instead of forty. But it also means a naive per-event
 * update silently DROPS what was reported before, so the BOM would vanish the
 * moment anything else happened. Every update therefore re-states the design's
 * standing facts (BOM, what is fitted) alongside the new event.
 */
async function reportContext(eventText, extra = {}) {
  const bom = state.bom.map((r) => ({ ref: r.ref, kind: r.kind ?? null,
                                      value: r.value ?? null }));
  const fitted = Object.entries(state.bound).map(([r, m]) => `${r}=${m}`);
  const lines = [eventText];
  if (bom.length) {
    lines.push(`[bom] ${state.topology}: `
      + bom.map((r) => `${r.ref}${r.value ? ` ${r.value}` : ""}`).join(", "));
  }
  if (fitted.length) lines.push(`[fitted] ${fitted.join(", ")}`);
  await app.updateModelContext({
    content: [{ type: "text", text: lines.join("\n") }],
    structuredContent: plain({ topology: state.topology, bom,
                               bound: state.bound, ...extra }),
  });
}

function ingest(sc) {
  state.error = "";
  state.note = "";
  state.candidates = null;
  state.selected = "";
  state.topology = sc.topology || "";
  state.tas = sc.tas || null;
  if (!state.tas) {
    state.error = "No TAS in the tool result.";
    return;
  }
  try {
    state.bom = extractBom(state.tas) || [];
  } catch (e) {
    state.bom = [];
  }
  try {
    // Throws on any drift between the drawing and the CIAS netlist — surfaced,
    // never papered over with a picture that does not match the circuit.
    const svg = renderVerifiedSchematic(state.topology, state.tas, undefined, state.bom);
    state.svg = svg || "";
    if (!svg) {
      state.error = `No schematic layout for '${state.topology}' yet — the design itself is fine.`;
    }
  } catch (e) {
    state.svg = "";
    state.error = `Schematic refused to render: ${e.message}`;
  }
}

/** Click a component: report the selection, then fetch real catalogue parts. */
async function selectComponent(ref) {
  const row = state.bom.find((r) => r.ref === ref);
  state.selected = ref;
  await reportContext(
    `[selected] ${ref}${row?.value ? ` = ${row.value}` : ""}`
    + `${row?.kind ? ` (${row.kind})` : ""}`,
    { selected_ref: ref });
  await loadCandidates(ref);
}

async function loadCandidates(ref) {
  state.busy = true;
  state.candidates = null;
  try {
    const res = await app.callServerTool({
      name: "select_parts",
      arguments: plain({ tas: state.tas, options: { topology: state.topology } }),
    });
    const sc = res.structuredContent || {};
    const hit = (sc.components || []).find((c) => c.ref === ref);
    state.candidates = hit?.selection?.candidates ?? [];
    if (!state.candidates.length) {
      state.error = hit?.error
        ? `No part fits ${ref}: ${hit.error}`
        : `No candidates returned for ${ref}.`;
    }
  } catch (e) {
    // A missing catalogue is the common case here and says so plainly, rather
    // than looking like "nothing fits".
    state.error = `Part sourcing unavailable: ${e.message}`;
  } finally {
    state.busy = false;
  }
}

/**
 * Fit a chosen candidate: stamp its datasheet envelope into the design and
 * redraw from the TAS the server returns, so the schematic shows what is
 * actually bound rather than what was merely offered.
 */
async function bindCandidate(candidate) {
  if (!state.selected) return;
  state.busy = true;
  state.error = "";
  try {
    const res = await app.callServerTool({
      name: "bind_part",
      arguments: plain({ tas: state.tas, ref: state.selected,
                         envelope: candidate.envelope }),
    });
    const bound = res.structuredContent?.tas;
    if (!bound) throw new Error("bind_part returned no TAS");
    state.bound[state.selected] = candidate.mpn;
    const ref = state.selected;
    const keep = state.candidates;                     // survive the redraw
    ingest({ topology: state.topology, tas: bound });   // redraw from the bound design
    state.selected = ref;
    state.candidates = keep;                            // ...so the list stays put
    await reportContext(
      `[fitted] ${ref} = ${candidate.mpn}`
      + `${candidate.manufacturer ? ` (${candidate.manufacturer})` : ""} — that `
      + `component is now at DATASHEET fidelity.`);
  } catch (e) {
    state.error = `Could not bind ${candidate.mpn}: ${e.message}`;
  } finally {
    state.busy = false;
  }
}

/**
 * The visual-sim export, from the web app's own falstad.js — the schematic and
 * the simulator are then the same circuit by construction.
 */
async function exportFalstad() {
  try {
    // Returns {text, url, fsw, vin, vout, ...} — the url opens the running sim.
    const sim = falstadExport(state.topology, state.tas, "overview");
    state.falstadUrl = sim.url;
    await reportContext(
      `[falstad] ${state.topology} visual sim ready `
      + `(${sim.text.split("\n").length} elements, Vin ${sim.vin} V → Vout ${sim.vout} V, `
      + `f_sw ${sim.fsw} Hz): ${sim.url}`,
      { falstad: { url: sim.url, elements: sim.text.split("\n").length } });
    state.note = "Falstad circuit exported — the link is in the conversation.";
  } catch (e) {
    state.error = `Falstad export failed: ${e.message}`;
  }
}

/** Delegate clicks off the raw SVG string — the hotspots carry data-ref. */
function onClick(ev) {
  const hit = ev.target.closest?.("[data-ref]");
  if (hit) selectComponent(hit.getAttribute("data-ref"));
}

const Root = {
  setup() {
    return () =>
      h("div", { class: "wrap" }, [
        h("h1", state.topology ? `${state.topology} converter` : "Converter"),
        state.summary ? h("div", { class: "sub" }, state.summary) : null,
        state.error ? h("div", { class: "err" }, state.error) : null,
        state.note ? h("div", { class: "readout" }, state.note) : null,
        state.svg
          ? h("div", { onClick, innerHTML: state.svg })
          : h("div", { class: "readout muted" }, "No schematic to show."),
        state.selected
          ? h("div", { class: "readout" },
              state.busy
                ? `Sourcing parts for ${state.selected}…`
                : `${state.selected} selected` +
                  (state.candidates?.length ? ` — ${state.candidates.length} candidate part(s)` : ""))
          : h("div", { class: "readout muted" }, "Click a component to source it."),
        state.candidates?.length
          ? h("table", { class: "tbl" }, [
              h("thead", h("tr", [h("th", "MPN"), h("th", "Manufacturer"), h("th", "")])),
              h("tbody", state.candidates.slice(0, 12).map((c) =>
                h("tr", { class: state.bound[state.selected] === c.mpn ? "bound" : "" }, [
                  h("td", c.mpn),
                  h("td", c.manufacturer || "—"),
                  h("td", state.bound[state.selected] === c.mpn
                    ? h("span", { class: "tag" }, "fitted")
                    : h("button", {
                        disabled: state.busy,
                        onClick: () => bindCandidate(c),
                      }, "Fit")),
                ]))),
            ])
          : null,
        h("div", { class: "foot" }, [
          hasVisualSim(state.topology)
            ? h("button", { onClick: exportFalstad, disabled: !state.tas },
                "Export to Falstad")
            : null,
          Object.keys(state.bound).length
            ? h("span", { class: "muted" },
                ` ${Object.entries(state.bound).map(([r, m]) => `${r}=${m}`).join(", ")}`)
            : null,
        ]),
      ]);
  },
};

createApp(Root).mount("#app");

// Handlers before connect(): the host may push the tool result during the
// ui/initialize handshake, and a late listener misses it.
app.ontoolresult = async (result) => {
  const sc = result.structuredContent;
  if (!sc || !sc.tas) {
    state.error = "No design in the tool result.";
    return;
  }
  state.summary = (result.content?.[0]?.text || "").split("\n")[0] || "";
  ingest(sc);
  state.bound = {};
  // The BOM comes from the web app's own bom.js — there is no C++ extractBom,
  // and a second implementation on the server would be free to drift from it.
  if (state.bom.length) {
    await reportContext(`[design] ${state.topology} converter designed.`);
  }
};

await app.connect();
