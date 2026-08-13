/**
 * Kirchhoff candidate picker — the MCP App.
 *
 * The sourcing tools (select_parts, select_candidates, cross_reference, bind_part) all end in
 * the same human decision: choose one part from a ranked list. Until now they returned it as
 * text, so the engineer could read the answer but not act on it (ABT #655).
 *
 * The candidates ARE Kelvin's — Kirchhoff sources through the same engine — so this is
 * Kelvin's picker with its envelope reader widened for the per-component shape select_parts
 * returns. ABT #663 asks websharedcomponents for the one component both servers should
 * import; this is a marked copy until that lands, not a fork.
 */
import { App } from "@modelcontextprotocol/ext-apps";

const app = new App({ name: "Kirchhoff picker", version: "0.1.0" });

const state = {
  mode: "",
  category: "",
  candidates: [],
  original: null,
  originalSpecs: null,
  tiebreaker: null,
  considered: null,
  poolSize: null,
  caveat: null,
  selected: null,
  expanded: new Set(),
  error: "",
};

const el = (tag, attrs = {}, ...kids) => {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") n.className = v;
    else if (k === "onclick") n.onclick = v;
    else if (v !== null && v !== undefined) n.setAttribute(k, v);
  }
  for (const kid of kids.flat()) {
    if (kid === null || kid === undefined || kid === false) continue;
    n.append(kid instanceof Node ? kid : document.createTextNode(String(kid)));
  }
  return n;
};

const STATUS_LABEL = {
  recommended: "recommended",
  partial: "partial",
  no_substitute: "not a substitute",
};

const fmt = (v) => {
  if (v === null || v === undefined || v === "") return "—";
  if (typeof v === "number") {
    if (Math.abs(v) >= 1000 || (Math.abs(v) < 0.01 && v !== 0)) return v.toPrecision(3);
    return String(Math.round(v * 10000) / 10000);
  }
  return String(v);
};

// How candidates become columns lives in columns.js so it can be tested without a host
// bridge — this module connects to the host at import, so logic left in here can only be
// checked by rendering the real GUI, which is how the ranked views got away with deriving
// one column each.
import { columnsFor, specsOf, valueFor } from "./columns.js";

function verdictPills(params) {
  if (!Array.isArray(params) || !params.length) return null;
  return el("div", { class: "pills" },
    params.map((p) =>
      el("span", { class: `pill ${p.verdict ?? ""}` }, `${p.name}: ${p.verdict ?? "?"}`)));
}

function detailPanel(row) {
  const kids = [];
  const pills = verdictPills(row.params);
  if (pills) kids.push(pills);
  if (Array.isArray(row.notes) && row.notes.length) {
    kids.push(el("ul", { class: "notes" }, row.notes.map((n) => el("li", {}, n))));
  }
  if (row.margins && typeof row.margins === "object") {
    const ms = Object.entries(row.margins).filter(([, v]) => v !== null && v !== undefined);
    if (ms.length) {
      kids.push(el("div", { class: "margins" },
        ms.map(([k, v]) => el("span", { class: "margin" }, `${k} ×${fmt(v)}`))));
    }
  }
  if (!kids.length) kids.push(el("div", { class: "muted" }, "No further detail recorded."));
  return el("td", { class: "detail", colspan: "99" }, kids);
}

function render() {
  const root = document.getElementById("app");
  root.textContent = "";

  if (state.error) {
    root.append(el("div", { class: "err" }, state.error));
    return;
  }
  if (!state.candidates.length) {
    root.append(el("div", { class: "muted pad" }, "Waiting for candidates…"));
    return;
  }

  const isCross = state.mode === "crossref";
  const cols = columnsFor(state.candidates);

  // header
  const sub = isCross
    ? `substitutes for ${state.original?.mpn ?? "the original"} · ${state.poolSize ?? "?"} parts considered`
    : `${state.category} · ranked by ${state.tiebreaker ?? "the engine"} · ${state.considered ?? "?"} rows considered`;
  root.append(
    el("div", { class: "head" },
      el("h1", {}, isCross ? "Cross-reference" : "Candidates"),
      el("div", { class: "sub" }, sub)));

  if (isCross && state.originalSpecs) {
    root.append(el("div", { class: "orig" },
      el("span", { class: "origlabel" }, "original"),
      Object.entries(state.originalSpecs)
        .filter(([, v]) => v !== null && v !== undefined && v !== "")
        .map(([k, v]) => el("span", { class: "ospec" }, `${k} ${fmt(v)}`))));
  }

  const head = el("tr", {},
    el("th", {}, "Part"),
    isCross ? el("th", {}, "Verdict") : null,
    cols.map((c) => el("th", { class: c.kind }, c.label)),
    el("th", {}, ""));

  const body = [];
  for (const row of state.candidates) {
    const open = state.expanded.has(row.mpn);
    const chosen = state.selected === row.mpn;
    body.push(el("tr", { class: `row${chosen ? " chosen" : ""}`, onclick: () => toggle(row) },
      el("td", {},
        el("div", { class: "mpn" }, row.mpn ?? "?"),
        row.manufacturer ? el("div", { class: "maker" }, row.manufacturer) : null),
      isCross
        ? el("td", {},
            el("span", { class: `status ${row.status ?? ""}` },
              STATUS_LABEL[row.status] ?? row.status ?? "—"),
            typeof row.penalty === "number"
              ? el("div", { class: "pen" }, `penalty ${fmt(row.penalty)}`)
              : null)
        : null,
      // A margin is a ratio and reads as one; a spec is a magnitude. Absent stays an em dash
      // in both cases — never 0, which would be a measurement.
      cols.map((c) => {
        const v = valueFor(row, c);
        return el("td", { class: c.kind },
          c.kind === "margin" && typeof v === "number" ? `×${fmt(v)}` : fmt(v));
      }),
      el("td", { class: "act" },
        el("button", {
          class: chosen ? "btn chosen" : "btn",
          onclick: (e) => { e.stopPropagation(); choose(row); },
        }, chosen ? "selected" : "use this"))));
    if (open) body.push(el("tr", { class: "detailrow" }, detailPanel(row)));
  }

  root.append(el("table", { class: "tbl" },
    el("thead", {}, head), el("tbody", {}, body)));
  if (state.caveat) root.append(el("div", { class: "hint" }, state.caveat));
  root.append(el("div", { class: "hint" },
    "Click a row for the engineering detail. “Use this” tells the assistant your choice."));
}

function toggle(row) {
  if (!row.mpn) return;
  if (state.expanded.has(row.mpn)) state.expanded.delete(row.mpn);
  else state.expanded.add(row.mpn);
  render();
}

/**
 * Report the user's choice to the model.
 *
 * updateModelContext OVERWRITES rather than appends, so the message restates
 * the standing facts (what was being chosen, and from what) alongside the
 * selection — otherwise the model loses the context of the pick.
 */
async function choose(row) {
  state.selected = row.mpn;
  render();
  const what = state.mode === "crossref"
    ? `substitute for ${state.original?.mpn ?? "the original"}`
    : `${state.category} from the ranked candidates`;
  const specs = Object.entries(row.specs ?? {})
    .filter(([, v]) => v !== null && v !== undefined && v !== "")
    .map(([k, v]) => `${k} ${fmt(v)}`).join(", ");
  const lines = [`[user selected] ${row.mpn} as the ${what}.`];
  if (specs) lines.push(`[specs] ${specs}`);
  if (row.status) lines.push(`[verdict] ${row.status}${row.grade ? ` (${row.grade})` : ""}`);
  await app.updateModelContext({
    content: [{ type: "text", text: lines.join("\n") }],
    structuredContent: JSON.parse(JSON.stringify({
      selected: {
        mpn: row.mpn, manufacturer: row.manufacturer, category: state.category,
        specs: row.specs ?? null, status: row.status ?? null,
        params: row.params_full ?? row.params ?? null,
      },
      context: { mode: state.mode, original: state.original?.mpn ?? null },
    })),
  });
}

/**
 * Normalise a tool payload.
 *
 * Every recommender in the ecosystem returns the same *idea* — a ranked
 * candidate list — under slightly different envelope names. This widget is
 * meant to serve all of them (ABT #663), so it accepts the aliases rather than
 * forcing one server's vocabulary on the rest.
 */
/**
 * select_parts answers for a WHOLE design: components[], each with its own ranked list. It is
 * flattened into one table with the reference designator carried per row, because the question
 * an engineer asks of a BOM is "which part for Q1", not "show me twelve lists".
 */
function fromComponents(components) {
  const rows = [];
  for (const c of components ?? []) {
    const cands = (c.selection?.candidates) ?? [];
    for (const cand of cands) {
      rows.push({ ...cand, _ref: c.ref, _family: c.family, _chosen: cand.mpn === c.mpn });
    }
    if (!cands.length && c.error) {
      rows.push({ mpn: `${c.ref}: UNFILLED`, manufacturer: null, notes: [c.error], _ref: c.ref });
    }
  }
  return rows;
}

function ingest(sc) {
  state.error = "";
  state.mode = sc.mode ?? (Array.isArray(sc.components) ? "bom" : "");
  state.category = sc.category ?? sc.family ?? "";
  // `candidates` is the envelope Kelvin normalises to, but a ranked list arrives
  // from other servers as `ranked` or as a plain page of `rows`. Accept all three
  // rather than render "waiting for candidates" at a payload that has them.
  state.candidates = Array.isArray(sc.components)
    ? fromComponents(sc.components)
    : ([sc.candidates, sc.ranked, sc.rows].find(Array.isArray) ?? []);
  const orig = sc.original ?? null;
  state.original = typeof orig === "string" ? { mpn: orig } : orig;
  state.originalSpecs = sc.originalSpecs
    ?? (state.original ? specsOf(state.original) : null);
  state.tiebreaker = sc.tiebreaker ?? null;
  state.considered = sc.totalRowsConsidered ?? sc.total ?? null;
  state.poolSize = sc.poolSize ?? sc.poolTotal ?? null;
  state.caveat = sc.caveat ?? null;
  state.selected = null;
  state.expanded = new Set();
  render();
}

app.ontoolresult = async (result) => {
  const sc = result?.structuredContent;
  if (!sc) {
    state.error = "The tool returned no structured content for this widget.";
    render();
    return;
  }
  ingest(sc);
};

render();
await app.connect();
