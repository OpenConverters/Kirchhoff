"""Kirchhoff MCP App server — power-converter design and simulation in a chat.

Exposes the Kirchhoff C++ engine (through the ``PyKirchhoff`` pybind11 module)
as MCP tools, and ships the web app's own CIAS schematic as an MCP Apps UI
resource (SEP-1865) — so the schematic an engineer clicks in Claude is rendered
by the same `ciasSchematic.js` that draws it on the web, from one definition.

SPICE runs go through Kirchhoff's IN-PROCESS libngspice (`run_ngspice_console` /
`simulate_ngspice`); the installed `ngspice` binary is never invoked.

Run:
    python mcp/server.py                 # streamable HTTP on 127.0.0.1:8401/mcp
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
# Where PyKirchhoff might be. KIRCHHOFF_BUILD first (the name Heaviside already uses for the
# same question), then the build dirs this repo actually produces — a checkout that builds to
# build-native could not find its own module and reported it as "not built".
_SEARCHED = tuple(Path(p) for p in (os.environ.get("KIRCHHOFF_BUILD"),) if p) + (
    _REPO / "build", _REPO / "build-latest", _REPO / "build-native")
_FOUND = [p for d in _SEARCHED for p in list(d.glob("PyKirchhoff*.so")) + list(d.glob("PyKirchhoff*.pyd"))]
for _module in _FOUND:
    sys.path.insert(0, str(_module.parent))
    break

try:
    import PyKirchhoff as kh
except ImportError as error:                                       # pragma: no cover
    # A built module that the RUNNING interpreter cannot load is the common
    # failure here (`python` is 3.10 on many boxes while the build targeted the
    # `python3` on PATH), and "not built" would send you off rebuilding
    # something that is already there. Say which it is.
    _running = f"{sys.version_info.major}.{sys.version_info.minor}"
    if _FOUND:
        _tags = ", ".join(sorted({p.name for p in _FOUND}))
        raise ImportError(
            f"PyKirchhoff IS built ({_tags}) but this interpreter cannot load it: "
            f"you are running Python {_running} ({sys.executable}).\n"
            f"Run the server with the interpreter the module was built for -- usually:\n"
            f"  python3 mcp/server.py\n"
            f"or rebuild against this one:\n"
            f"  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release "
            f"-DPython3_EXECUTABLE={sys.executable} && make -C build PyKirchhoff -j6"
        ) from error
    raise ImportError(
        "PyKirchhoff is not built. Build it with:\n"
        "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && make -C build PyKirchhoff -j6\n"
        f"(looked in {' and '.join(str(d) for d in _SEARCHED)})"
    ) from error

from mcp.server.fastmcp import FastMCP                 # noqa: E402
from mcp.server.transport_security import (            # noqa: E402
    TransportSecuritySettings,
)
from mcp.types import CallToolResult, TextContent      # noqa: E402

from artifacts import resolved                          # noqa: E402

# --- MCP Apps wire constants (from @modelcontextprotocol/ext-apps 1.7.5) -----
UI_RESOURCE_MIME = "text/html;profile=mcp-app"
UI_SCHEMATIC_URI = "ui://kirchhoff/schematic.html"   # CIAS schematic, click-to-select
UI_CURVES_URI = "ui://kirchhoff/curves.html"         # waveform / sweep chart
UI_PICKER_URI = "ui://kirchhoff/picker.html"         # ranked candidate parts, click to choose
UI_BODE_URI = "ui://kirchhoff/bode.html"             # dB against a LOG frequency axis


def _ui_meta(uri: str) -> dict:
    """registerAppTool() emits BOTH the flat key and the nested object, so hosts
    reading either form find it. Mirror that exactly."""
    return {"ui/resourceUri": uri, "ui": {"resourceUri": uri}}


UI_SCHEMATIC_META = _ui_meta(UI_SCHEMATIC_URI)
UI_CURVES_META = _ui_meta(UI_CURVES_URI)
UI_PICKER_META = _ui_meta(UI_PICKER_URI)
UI_BODE_META = _ui_meta(UI_BODE_URI)

# Every topology with a design + build_tas pair in the engine (BIND_DESIGN in
# src/bindings.cpp). Kept as data so list_topologies and the argument validation
# cannot drift apart.
TOPOLOGIES = (
    "buck", "boost", "flyback", "forward", "two_switch_forward", "sepic", "cuk", "zeta",
    "push_pull", "psfb", "pshb", "ahb", "acf", "fsbb", "llc", "cllc", "clllc", "src",
    "dab", "isolated_buck", "isolated_buck_boost", "weinberg", "pfc", "vienna",
)

DEFAULT_FIDELITY = {"origin": "REQUIREMENTS"}

# A chart has ~1000 useful pixels across; a 200k-point transient does not need to
# cross the wire. Decimation is PEAK-PRESERVING (min AND max per bin) — averaging
# or strided sampling would hide the very overshoot the engineer is looking for.
MAX_TRACE_POINTS = 900

# The SDK's DNS-rebinding protection rejects any Host header it does not
# recognise with a bare "421 Invalid Host header". Behind a tunnel or reverse
# proxy the Host is the PUBLIC name, not localhost — so every request dies at
# 421, and a remote host that cannot speak MCP typically falls back to probing
# for OAuth, surfacing as a misleading "couldn't register with the sign-in
# service" error. Name the public host here (KIRCHHOFF_PUBLIC_HOST), or set
# KIRCHHOFF_ALLOW_ANY_HOST=1 for throwaway tunnels whose name changes per run.
_public_host = os.environ.get("KIRCHHOFF_PUBLIC_HOST", "").strip()
# Accept a pasted URL, not just a bare hostname. A Host header carries neither
# scheme nor path, so "https://x.trycloudflare.com/mcp" would never match the
# incoming "x.trycloudflare.com" — and the resulting 421 surfaces in Claude as a
# sign-in failure, sending you after an OAuth problem that does not exist.
if "://" in _public_host:
    _public_host = _public_host.split("://", 1)[1]
_public_host = _public_host.split("/", 1)[0].strip()
if os.environ.get("KIRCHHOFF_ALLOW_ANY_HOST") == "1":
    _security = TransportSecuritySettings(enable_dns_rebinding_protection=False)
else:
    _allowed = ["127.0.0.1:8401", "localhost:8401", "127.0.0.1", "localhost"]
    if _public_host:
        _allowed += [_public_host, f"{_public_host}:443"]
    # allowed_origins is matched EXACTLY (or with a trailing ":*" port wildcard)
    # — a bare "*" is not a wildcard, just a literal that never matches, so it
    # reads as "allow everything" while 403-ing every browser-resident host.
    # Name the origins that actually call: Claude, a local reference host, and
    # the tunnel itself. KIRCHHOFF_ALLOWED_ORIGINS adds more, comma-separated.
    _origins = ["https://claude.ai", "https://www.claude.ai",
                "http://localhost:*", "http://127.0.0.1:*"]
    if _public_host:
        _origins.append(f"https://{_public_host}")
    _origins += [o.strip() for o in
                 os.environ.get("KIRCHHOFF_ALLOWED_ORIGINS", "").split(",") if o.strip()]
    _security = TransportSecuritySettings(allowed_hosts=_allowed, allowed_origins=_origins)

mcp = FastMCP("Kirchhoff", host="127.0.0.1", port=8401, transport_security=_security)


# --- helpers ----------------------------------------------------------------

def _eng(value: float, unit: str) -> str:
    """Engineering-notation label, e.g. 3.3 mH / 4.7 nF."""
    if value is None:
        return "-"
    for factor, prefix in ((1e-12, "p"), (1e-9, "n"), (1e-6, "µ"), (1e-3, "m"), (1.0, "")):
        if abs(value) < factor * 1000.0:
            return f"{value / factor:.3g} {prefix}{unit}"
    if abs(value) < 1e6:
        return f"{value / 1e3:.3g} k{unit}"
    return f"{value / 1e6:.3g} M{unit}"


def _topology_id(topology: str) -> str:
    key = topology.strip().lower().replace("-", "_").replace(" ", "_")
    if key not in TOPOLOGIES:
        raise ValueError(
            f"unknown topology {topology!r} -- one of: {', '.join(TOPOLOGIES)}"
        )
    return key


def _decimate(xs: list[float], ys: list[float]) -> list[list[float]]:
    """Peak-preserving reduction to <= MAX_TRACE_POINTS points.

    Both the MIN and the MAX of each bin are kept, in time order: a transient's
    ringing peak is the whole reason to look at the plot, and a strided sample
    would walk straight past it.
    """
    n = len(xs)
    if n <= MAX_TRACE_POINTS:
        return [[float(a), float(b)] for a, b in zip(xs, ys)]
    bins = MAX_TRACE_POINTS // 2
    step = n / bins
    out = []
    for b in range(bins):
        lo, hi = int(b * step), max(int(b * step) + 1, int((b + 1) * step))
        window = range(lo, min(hi, n))
        i_min = min(window, key=lambda i: ys[i])
        i_max = max(window, key=lambda i: ys[i])
        for i in sorted((i_min, i_max)):
            out.append([float(xs[i]), float(ys[i])])
    return out


def _fidelity(fidelity: dict | None) -> dict:
    return fidelity if fidelity else dict(DEFAULT_FIDELITY)


def _result(summary: str, payload: dict) -> CallToolResult:
    """Two channels: a compact digest for the model, the payload for the widget.

    Returning a plain dict from a FastMCP tool instead emits NO
    structuredContent at all and serialises the WHOLE payload into `content` —
    half a megabyte of parts catalogue into the context window on one
    select_parts call, and a widget that receives nothing. Every tool here
    therefore builds its result explicitly, which FastMCP passes through
    verbatim.
    """
    return CallToolResult(
        content=[TextContent(type="text", text=summary)],
        structuredContent=payload,
    )


# --- the pipeline contract --------------------------------------------------
# Every payload is a result under Moebius's contracts/pipeline_result.json, which is what
# lets an orchestrator, a widget and the next server read this engine without each of them
# learning its private shapes. Four helpers cover the whole surface; the branch a tool
# belongs to is a statement about the QUESTION it answers:
#
#   document  — one schema-governed artifact: a TAS, a MAS Inputs block, a SPICE deck.
#   quantity  — numbers the engine computed, with the model that computed them.
#   catalogue — what this pipeline can answer about.
#   bom / ranked / verdict — sourcing a design, ranking parts, judging against a criterion.
#
# The rule that decides between `document` and `design`: a document is ONE artifact, a design
# result RANKS several. Returning a list of one under a ranked branch advertises a comparison
# that never happened.

def _document_result(summary: str, *, schema: str, operation: str, document: dict,
                     subject: str | None = None, version: str | None = None,
                     companions: dict | None = None, diagnostics: list | None = None,
                     changed: list | None = None, derived_from: str | None = None,
                     view: str | None = None, caveat: str | None = None) -> CallToolResult:
    """A `document` result — the artifact, and WHICH schema governs it.

    `schema` is not decoration: a consumer holding 40 kB of JSON that does not say what it is
    cannot validate it, render it or hand it on. `operation` says whether the engine produced
    the document or transformed one it was given, which decides whether a consumer may diff
    the output against its input.
    """
    payload: dict = {
        "mode": "document",
        "schema": {"name": schema, **({"version": version} if version else {})},
        "operation": operation,
        "document": document,
    }
    for key, value in (("subject", subject), ("companions", companions),
                       ("diagnostics", diagnostics), ("changed", changed),
                       ("derivedFrom", derived_from), ("view", view), ("caveat", caveat)):
        if value:
            payload[key] = value
    return _result(summary, payload)


def _quantities(pairs: dict) -> dict:
    """Named computed values, unit BESIDE the value. Entries whose value is None are dropped —
    'the engine did not compute it' and 'it is zero' are different facts."""
    out = {}
    for name, (value, unit, *rest) in pairs.items():
        if value is None:
            continue
        entry = {"value": float(value) if isinstance(value, (int, float))
                 and not isinstance(value, bool) else value, "unit": unit}
        if rest and rest[0]:
            entry["label"] = rest[0]
        out[name] = entry
    return out


def _quantity_result(summary: str, *, subject: str, model: str, quantities: dict | None = None,
                     statistics: dict | None = None, conditions: dict | None = None,
                     caveat: str | None = None) -> CallToolResult:
    """A `quantity` result — what was computed, of what, by which model.

    `model` is required by the contract and it is the field that makes the number checkable:
    an inductance from a closed form and one extracted from a transient are different claims,
    and a consumer that cannot tell them apart cannot weigh either.
    """
    payload: dict = {"mode": "quantity", "subject": subject, "model": model}
    for key, value in (("conditions", conditions), ("quantities", quantities),
                       ("statistics", statistics), ("caveat", caveat)):
        if value:
            payload[key] = value
    return _result(summary, payload)


def _curves_result(title, subtitle, series, summary, *, x_axis, y_axis, y2_axis=None,
                   note=None, markers=None, topology=None):
    """A `curves` result — one or more curves over declared axes.

    THE SERIES CARRY THEIR AXIS, not their unit. This engine's plots are dual: volts and amps
    against time, magnitude and phase against frequency. Before the contract's `axes.y2`, the
    widget inferred the axis from a per-series unit string, which meant every consumer had to
    know that "A" means "the right-hand axis" — a convention, not a contract, and one that
    silently mislabels the first series whose unit nobody thought of.

    `data` is gone deliberately. A curves result that also smuggled the tool's own object made
    the payload two shapes at once; a tool whose answer is an operating point returns a
    `document` and charts it there.
    """
    payload: dict = {
        "mode": "curves",
        "title": title,
        "axes": {"x": x_axis, "y": y_axis, **({"y2": y2_axis} if y2_axis else {})},
        "series": series,
    }
    if subtitle:
        payload["subtitle"] = subtitle
    if markers:
        payload["markers"] = markers
    if topology:
        payload["topology"] = topology
    if note:
        payload["caveat"] = note
    return _result(summary, payload)


def _axis(label: str, unit: str, scale: str | None = None) -> dict:
    return {"label": label, "unit": unit, **({"scale": scale} if scale else {})}


# The two ordinates every V/A plot here uses. Named once so a trace cannot be put on the
# voltage axis in one tool and the current axis in another.
V_AXIS = _axis("voltage", "V")
A_AXIS = _axis("current", "A")
TIME_AXIS = _axis("time", "s")


def _va_series(name: str, unit: str, points: list) -> dict:
    """One trace, on the axis its quantity belongs to."""
    return {"name": name, "points": points, "axis": "y2" if unit == "A" else "y"}


# What a `candidate` may carry under the contract. Anything else the engine attaches is
# BOOKKEEPING and moves under an underscore, which marks it uninterpretable by consumers —
# `envelope` is the datasheet blob bind_part takes back verbatim, and srcOffset/srcLength/line
# are where the row sat in the catalogue file. None of it describes the part to a reader.
CANDIDATE_FIELDS = ("mpn", "manufacturer", "specs", "status", "grade", "penalty", "direction",
                    "footprint", "params", "notes", "margins", "row", "sortKey", "evidence",
                    "record")


def _candidate(cand: dict) -> dict:
    """One engine candidate as the contract's `candidate`."""
    out = {k: v for k, v in cand.items() if k in CANDIDATE_FIELDS}
    out.setdefault("mpn", cand.get("mpn") or "(unnamed)")
    for key, value in cand.items():
        if key in CANDIDATE_FIELDS:
            continue
        out[key if key.startswith("_") else f"_{key}"] = value
    return out


def _components_of(tas: dict) -> dict:
    """Every component in a TAS, by reference designator.

    They live per stage — tas.topology.stages[].circuit.components[] — so a caller asking
    'what is Q1' would otherwise have to know the nesting.
    """
    out = {}
    for stage in (tas.get("topology") or {}).get("stages") or []:
        for component in ((stage.get("circuit") or {}).get("components") or []):
            ref = component.get("name") or component.get("reference")
            if ref:
                out[ref] = component
    return out


def _changed_components(before: dict, after: dict, change: str,
                        fidelity: str | None = None) -> list[dict]:
    """Which refs a transform actually touched, and what happened to them.

    Computed by comparing the documents rather than assumed from the tool's name: realize_tas
    attaches models to the SEMICONDUCTORS and leaves the passives alone, and a `changed` list
    naming every component would be as useless as none at all.
    """
    old, new = _components_of(before), _components_of(after)
    out = []
    for ref, component in new.items():
        if json.dumps(old.get(ref), sort_keys=True) == json.dumps(component, sort_keys=True):
            continue
        entry = {"ref": ref, "change": change}
        if fidelity:
            entry["fidelity"] = fidelity
        out.append(entry)
    return out


def _realized_refs(tas: dict, realized: dict) -> list[dict]:
    return _changed_components(tas, realized, "datasheet device model attached", "DATASHEET")


def _sized_quantities(diag: dict) -> dict:
    """The engine's component sizing as named quantities: 'C1.capacitance' -> 4.7 µF.

    Keyed by ref and property so two components of the same family cannot collide, and so a
    consumer reading the value never has to know which family's unit applies — the unit is on
    the entry.
    """
    units = {"capacitors": ("capacitance", "F"), "inductors": ("inductance", "H"),
             "resistors": ("resistance", "ohm"), "magnetics": ("inductance", "H")}
    out = {}
    for family, entries in sorted(diag.items()):
        if not isinstance(entries, list):
            continue
        key, unit = units.get(family, (None, None))
        if not key:
            continue
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            ref = entry.get("name") or entry.get("ref")
            value = entry.get(key)
            if ref and isinstance(value, (int, float)) and not isinstance(value, bool):
                out[f"{ref}.{key}"] = {"value": float(value), "unit": unit, "label": ref}
    return out


def _diagnostics_lines(diag: dict) -> list[str] | None:
    """The sizing, one line per family, for a document's `diagnostics`.

    The contract's diagnostics are PROSE — what the engine wants to say about the document it
    just produced. The numbers themselves are not prose, which is why converter_diagnostics
    returns them as a `quantity` result instead of duplicating them here as a blob.
    """
    digest = _diagnostics_digest(diag)
    return [line for line in digest.splitlines() if line.strip()] or None


def _diagnostics_digest(diag: dict) -> str:
    """One line per component family the engine sized, values in engineering units."""
    units = {"capacitors": ("capacitance", "F"), "inductors": ("inductance", "H"),
             "resistors": ("resistance", "Ω"), "magnetics": ("inductance", "H")}
    lines = []
    for family, entries in sorted(diag.items()):
        if not isinstance(entries, list) or not entries:
            continue
        key, unit = units.get(family, (None, ""))
        parts = []
        for e in entries[:8]:
            if not isinstance(e, dict):
                continue
            name = e.get("name") or e.get("ref") or "?"
            value = e.get(key) if key else None
            parts.append(f"{name} {_eng(value, unit)}" if value is not None else name)
        if parts:
            lines.append(f"{family}: " + ", ".join(parts))
    return "\n".join(lines) if lines else "(engine reported no sized components)"


# Component kind -> Kelvin family, mirroring web/src/kh.js KIND_TO_CATEGORY.
# Magnetics get a catalogue lookup (real off-the-shelf parts) ALONGSIDE the
# OpenMagnetics custom-design route.
KIND_TO_CATEGORY = {
    "MOSFET": "mosfet", "Diode": "diode", "Capacitor": "capacitor", "Resistor": "resistor",
    "Controller": "controller", "IGBT": "igbt", "BJT": "bjt", "Varistor": "varistor",
    "Inductor": "magnetic", "Transformer": "magnetic",
}
# Cap any one manufacturer at 20% of the ranked list so the Pareto front spans
# vendors; Kelvin itself defaults to no cap to stay parity-locked.
KELVIN_MAX_MFR_FRACTION = 0.2
_KELVIN_SHARDS = _REPO / "web" / "public" / "kelvin"
_loaded_shards: set[str] = set()


def _ensure_shard(category: str) -> None:
    """Load a family's prebuilt index shard once per process.

    Raises rather than selecting over nothing: an unloaded family yields "no
    candidates", which reads as "nothing fits" when the truth is "nobody looked".
    """
    if category in _loaded_shards:
        return
    path = _KELVIN_SHARDS / f"{category}.kidx"
    if not path.exists():
        raise ValueError(
            f"no Kelvin index shard for '{category}' at {path} -- build the shards "
            f"(kelvin-index) or use select_parts with a data_dir instead")
    try:
        kh.kelvin_load_shard(category, path.read_bytes())
    except Exception as error:                                     # noqa: BLE001
        # A shard built by an older kelvin-index than the linked Kelvin is a
        # STALE ARTIFACT, not a missing feature: say so and name the working
        # alternative rather than letting it read as "this component has no parts".
        if "shard format version" in str(error):
            raise ValueError(
                f"the '{category}' index shard at {path} was built by an older "
                f"kelvin-index than the Kelvin linked into this engine, so it cannot "
                f"be read. Rebuild the shards, or use select_parts (which reads the "
                f"NDJSON catalogue directly and is unaffected)."
            ) from error
        raise
    _loaded_shards.add(category)


def _kelvin_data_dir(data_dir: str | None) -> str:
    """The TAS parts catalogue directory, from the argument or the environment.

    Raises rather than returning an empty string: Kelvin with no catalogue
    reports "no candidates", which reads as "nothing fits" when the truth is
    "nobody looked".
    """
    resolved = (data_dir or os.environ.get("KELVIN_TAS_DATA_DIR", "")).strip()
    if not resolved:
        raise ValueError(
            "no Kelvin parts catalogue: pass data_dir, or set KELVIN_TAS_DATA_DIR to the "
            "directory holding the TAS NDJSON catalogues (capacitor.ndjson, mosfet.ndjson, …)"
        )
    if not Path(resolved).is_dir():
        raise ValueError(f"Kelvin data_dir {resolved!r} is not a directory")
    return resolved


def _excitation_series(operating_point: dict, prefix: str = "") -> list[dict]:
    """MAS excitationsPerWinding -> the curves widget's V/A series.

    An excitation carries the winding's voltage and current as {waveform:{time,data}}. Only
    a pair with matching, non-empty arrays becomes a trace: a signal the engine described by
    processed STATS alone (peak/rms, no samples) has no curve, and inventing one from the
    stats would draw a shape the engine never computed.
    """
    series = []
    for i, exc in enumerate(operating_point.get("excitationsPerWinding") or []):
        name = exc.get("name") or f"winding {i + 1}"
        for quantity, unit in (("voltage", "V"), ("current", "A")):
            waveform = ((exc.get(quantity) or {}).get("waveform")) or {}
            time, data = waveform.get("time"), waveform.get("data")
            if not (isinstance(time, list) and isinstance(data, list)
                    and len(time) == len(data) and len(time) > 1):
                continue
            series.append(_va_series(f"{prefix}{name} {quantity}", unit,
                                     _decimate(time, data)))
    return series



def _bode_result(title, subtitle, series, summary, y_label="magnitude", y_unit="dB",
                 y2_axis=None, markers=None, note=None):
    """A frequency-domain `curves` result.

    Separate from _curves_result only in its axes: one quantity — or two, with phase on the
    second — against a LOG frequency abscissa. The scale is declared rather than left to be
    inferred: a decade sweep drawn linearly is unreadable, and nothing in the points says so.
    """
    return _curves_result(
        title, subtitle, series, summary,
        x_axis=_axis("frequency", "Hz", "log"), y_axis=_axis(y_label, y_unit),
        y2_axis=y2_axis, markers=markers, note=note)



def _deck_text(deck: str | None, deck_ref: str | None) -> str:
    """A SPICE deck, given inline or by reference. Exactly one."""
    if deck and deck_ref:
        raise ValueError("pass deck OR deck_ref, not both — which one is the circuit is not "
                         "something this server should guess")
    if deck_ref:
        with resolved(deck_ref, "KIRCHHOFF", "deck") as path:
            return path.read_text(encoding="utf-8", errors="replace")
    if not deck:
        raise ValueError("no deck given: pass deck (the netlist itself) or deck_ref (a path, "
                         "artifact://<id> or URL)")
    return deck



# --- tools: design ----------------------------------------------------------

@mcp.tool(
    title="List supported topologies",
    description="Every converter topology the engine can design, and the spec shape they take.",

    structured_output=False,
)
def list_topologies() -> CallToolResult:
    """The topologies design_converter accepts, plus a worked spec skeleton."""
    return _result(
        f"{len(TOPOLOGIES)} topologies: {', '.join(TOPOLOGIES)}.\n"
        f"Spec is designRequirements + operatingPoints in SI units; see the "
        f"structured output for a worked skeleton.",
        # A `catalogue` result. The families are the topologies, plus ONE entry carrying the
        # spec skeleton — each says which KIND it is, because a caller reading `spec` as a
        # topology name would call design_converter with nonsense.
        {
        "mode": "catalogue",
        "families": (
            [{"name": t, "kind": "topology"} for t in TOPOLOGIES]
            + [{"name": "spec", "kind": "skeleton", "document": {
                "designRequirements": {
                    "efficiency": 1.0,
                    "inputVoltage": {"minimum": 45.6, "nominal": 48, "maximum": 50.4},
                    "switchingFrequency": {"nominal": 100000},
                    "outputs": [{"name": "out", "voltage": {"nominal": 12}}],
                },
                "operatingPoints": [{"inputVoltage": 48, "outputs": [{"power": 24}]}],
            }}]),
        "units": "SI throughout (V, A, W, Hz, H, F, Ω)",
        })


@mcp.tool(
    title="Design a converter",
    description=(
        "Design a power converter from a high-level spec and return its full TAS "
        "topology document, the engine's component sizing, and a clickable schematic. "
        "Pass the returned `tas` to the simulate / netlist / sourcing tools."
    ),
    meta=UI_SCHEMATIC_META,
    structured_output=False,
)
def design_converter(topology: str, spec: dict, engine: str = "analytical") -> CallToolResult:
    """Design a converter.

    Args:
        topology: one of list_topologies() -- 'flyback', 'llc', 'vienna', ...
        spec: designRequirements + operatingPoints, SI units (see list_topologies).
        engine: 'analytical' (closed-form) or 'ngspice' (simulate to extract).
    """
    topology_id = _topology_id(topology)
    if engine not in ("analytical", "ngspice"):
        raise ValueError(f"engine must be 'analytical' or 'ngspice' -- got {engine!r}")
    result = kh.process_converter(topology_id, spec, engine)
    tas = result["tas"]
    diagnostics = result.get("diagnostics") or {}

    summary = (
        f"{topology_id} designed ({engine}).\n{_diagnostics_digest(diagnostics)}\n"
        f"The schematic is CIAS-generated and click-selectable: pick a component to "
        f"source or cross-reference it."
    )
    # A `document` result: ONE TAS, produced from a spec. The magnetic's operating point rides
    # as a COMPANION rather than a second top-level field — it is a subordinate document with
    # its own schema (MAS), and the companion key names which magnetic it belongs to, because
    # a flyback has T1 and its output filter and 'the operating point' would silently mean one
    # of them.
    operating_point = result.get("operatingPoint")
    companions = {}
    if operating_point:
        magnetic = (next((m.get("name") for m in (kh.topology_waveforms(tas) or [])
                          if m.get("isMain")), None) or "main magnetic")
        companions[magnetic] = {"schema": {"name": "MAS"}, "subject": magnetic,
                                "document": operating_point}
    return _document_result(
        summary, schema="TAS", operation="produced", document=tas, subject=topology_id,
        companions=companions or None,
        # The engine's sizing travels as diagnostics: it describes how this document was
        # arrived at, and a consumer that wants the numbers on their own calls
        # converter_diagnostics, which returns them as quantities.
        diagnostics=_diagnostics_lines(diagnostics),
        view=UI_SCHEMATIC_URI)


@mcp.tool(
    title="Converter diagnostics",
    description="The engine's component sizing and design diagnostics for an assembled TAS.",

    structured_output=False,
)
def converter_diagnostics(tas: dict) -> CallToolResult:
    """Sized components and design diagnostics for a TAS from design_converter."""
    diag = kh.diagnostics(tas)
    quantities = _sized_quantities(diag)
    if not quantities:
        raise RuntimeError(
            "the engine reported no sized components for this TAS — which is a result about "
            "the design, not an empty answer: check it came from design_converter")
    return _quantity_result(
        _diagnostics_digest(diag),
        subject=str((tas.get("topology") or {}).get("name")
                    or tas.get("name") or "the assembled converter"),
        model="Kirchhoff analytical component sizing",
        quantities=quantities)


@mcp.tool(
    title="Add datasheet device models",
    description=(
        "Add requirements-derived datasheet models (real Rds(on) / Vf) to every "
        "semiconductor, so a DATASHEET-fidelity deck renders real conduction losses "
        "instead of ideal switches."
    ),
    structured_output=False,
)
def realize_tas(tas: dict) -> CallToolResult:
    """Returns the realized TAS; feed it to export_netlist/simulate with DATASHEET fidelity."""
    realized = kh.realize_tas(tas)
    return _document_result(
        "Datasheet device models added. Pass this TAS to export_netlist or simulate "
        'with fidelity {"origin": "DATASHEET"} to see real conduction losses.',
        schema="TAS", operation="transformed", document=realized,
        # `changed` is required on a transform, and this is why: without it a consumer has to
        # diff two 40 kB documents to find out which semiconductors grew a model.
        changed=_realized_refs(tas, realized) or ["every semiconductor in the design"])


# --- tools: simulation ------------------------------------------------------

@mcp.tool(
    title="Export a SPICE netlist",
    description=(
        "Assemble a TAS into a runnable SPICE deck, in the ngspice or LTspice dialect. "
        "The deck is self-contained: it runs its own transient analysis and measures "
        "the output."
    ),

    structured_output=False,
)
def export_netlist(tas: dict, flavor: str = "ngspice", fidelity: dict | None = None) -> CallToolResult:
    """Netlist text for a designed converter.

    Args:
        tas: a TAS document from design_converter.
        flavor: 'ngspice' or 'ltspice'.
        fidelity: model selection, e.g. {"origin": "REQUIREMENTS"|"DATASHEET"|"MKF_MODEL"}.
    """
    if flavor not in ("ngspice", "ltspice"):
        raise ValueError(f"flavor must be 'ngspice' or 'ltspice' -- got {flavor!r}")
    emit = kh.tas_to_ngspice if flavor == "ngspice" else kh.tas_to_ltspice
    deck = emit(tas, _fidelity(fidelity))
    lines = deck.splitlines()
    shown = deck if len(lines) <= 400 else "\n".join(lines[:400]) + \
        f"\n* … {len(lines) - 400} more lines (full deck in the structured output)"
    # A deck is a document whose destination is a FILE, but it also travels onward to
    # run_deck and simulate_ac, so it goes inline rather than behind a reference — the same
    # rule documentResult states for a TAS a widget must render and hand on.
    return _document_result(
        f"{flavor} deck, {len(lines)} lines:\n\n{shown}",
        schema="spice-deck", version=flavor, operation="produced",
        document={"text": deck, "lines": len(lines)})


@mcp.tool(
    title="Simulate a converter",
    description=(
        "Run a designed converter through the IN-PROCESS libngspice and report what "
        "each saved node/branch measured (min, max, average, final value) over the "
        "transient. Never shells out to an installed ngspice binary. For sampled "
        "waveforms to plot, use component_waveforms."
    ),
    structured_output=False,
)
def simulate(tas: dict, fidelity: dict | None = None) -> CallToolResult:
    """Transient simulation of an assembled TAS.

    The engine returns per-vector STATISTICS rather than the raw samples — the
    deck measures as it runs, so a 150k-point transient never has to cross the
    wire. `component_waveforms` is the sampled-waveform tool.

    Args:
        tas: a TAS document from design_converter.
        fidelity: {"origin": "REQUIREMENTS"|"DATASHEET"|"MKF_MODEL"}.
    """
    result = kh.simulate_ngspice(tas, _fidelity(fidelity))
    if not result.get("success"):
        raise RuntimeError(f"ngspice run failed: {result.get('error') or 'no detail reported'}")
    saved = result.get("vectors") or {}
    if not saved:
        raise RuntimeError("the ngspice run saved no vectors")
    measurements = [
        {"vector": name, "min": stats.get("min"), "max": stats.get("max"),
         "average": stats.get("average"), "final": stats.get("last")}
        for name, stats in sorted(saved.items())
    ]
    rows = "\n".join(
        f"  {m['vector']}: {m['min']:.4g} .. {m['max']:.4g}, avg {m['average']:.4g}, "
        f"final {m['final']:.4g}"
        for m in measurements[:20] if m["min"] is not None)
    more = f"\n  … {len(measurements) - 20} more vectors" if len(measurements) > 20 else ""
    # STATISTICS, not quantities: a vector reduced over a run has no single value, and the
    # window it was reduced over travels with it — min/max from the first two switching cycles
    # says something different from min/max in steady state, and nothing else in the payload
    # would say which this was.
    t_start, t_end = result.get("tStart"), result.get("tEnd")
    if not isinstance(t_start, (int, float)) or not isinstance(t_end, (int, float)):
        # No silent 0..0: a statistic whose window is invented cannot be compared with the
        # next run, and 'reduced over nothing' is exactly the claim a default would make.
        raise RuntimeError(
            f"the ngspice run reported no transient window (tStart={t_start!r}, "
            f"tEnd={t_end!r}), so its statistics cannot say what they were measured over")
    window = {"from": float(t_start), "to": float(t_end), "axis": "time", "unit": "s"}
    statistics = {}
    for m in measurements:
        if m["min"] is None or m["max"] is None:
            continue
        # ngspice names a branch current i(vsense) and a node voltage v(sw); the unit follows
        # the vector's own kind rather than being assumed.
        unit = "A" if m["vector"].lower().startswith(("i(", "@")) else "V"
        entry = {"minimum": float(m["min"]), "maximum": float(m["max"]),
                 "unit": unit, "over": window}
        if isinstance(m.get("average"), (int, float)):
            entry["average"] = float(m["average"])
        if isinstance(m.get("final"), (int, float)):
            entry["final"] = float(m["final"])
        statistics[m["vector"]] = entry
    if not statistics:
        raise RuntimeError("the ngspice run measured no vector with usable statistics")
    return _quantity_result(
        f"Transient {result.get('tStart', 0.0):.4g}-{result.get('tEnd', 0.0):.4g} s, "
        f"{result.get('points')} points, {len(measurements)} vectors "
        f"(in-process libngspice):\n{rows}{more}",
        subject=str((tas.get("topology") or {}).get("name") or "the assembled converter"),
        model="ngspice transient, in-process libngspice",
        statistics=statistics,
        conditions={"points": {"value": result.get("points"), "unit": "1",
                               "label": "samples in the run"},
                    "fidelity": {"value": str(_fidelity(fidelity).get("origin"))}})


@mcp.tool(
    title="AC sweep a raw deck",
    description=(
        "Run a raw .ac SPICE deck through the in-process libngspice and return the "
        "complex sweep {frequenciesHz, vectors:{name:{re,im}}}."
    ),

    meta=UI_BODE_META,
    structured_output=False,
)
def simulate_ac(deck: str | None = None, deck_ref: str | None = None) -> CallToolResult:
    """Complex AC sweep of a hand-written or exported .ac deck.

    Args:
        deck: the deck itself.
        deck_ref: the deck by reference — see run_deck.
    """
    deck = _deck_text(deck, deck_ref)
    result = kh.run_ngspice_ac(deck)
    if not result.get("success"):
        raise RuntimeError(f"ngspice .ac run failed: {result.get('error') or 'no detail reported'}")
    freqs = result.get("frequenciesHz") or []
    vectors = result.get("vectors") or {}
    # Magnitude in dB is what a sweep is read as, and PHASE is the other half of the answer:
    # a loop with 12 dB of gain margin and 3 degrees of phase margin is unstable, and a
    # magnitude-only payload cannot say so. Both are computed from the same complex vectors.
    series, phase_series = [], []
    for name in sorted(vectors):
        v = vectors[name] or {}
        re_, im = v.get("re") or [], v.get("im") or []
        if len(re_) != len(freqs) or len(im) != len(freqs):
            continue
        points, phase_points = [], []
        for f, a, b in zip(freqs, re_, im):
            mag = (a * a + b * b) ** 0.5
            if f > 0 and mag > 0:
                points.append([float(f), 20.0 * math.log10(mag)])
                phase_points.append([float(f), math.degrees(math.atan2(b, a))])
        if len(points) > 1:
            series.append({"name": name, "axis": "y",
                           "points": _decimate([p[0] for p in points],
                                               [p[1] for p in points])})
            phase_series.append({"name": f"{name} phase", "axis": "y2",
                                 "points": _decimate([p[0] for p in phase_points],
                                                     [p[1] for p in phase_points])})
    summary = (f"AC sweep: {len(freqs)} points"
               + (f" from {freqs[0]:.4g} to {freqs[-1]:.4g} Hz" if freqs else "")
               + f", {len(vectors)} vector(s): {', '.join(sorted(vectors))[:200]}")
    if not series:
        # A sweep whose magnitudes are all zero, or whose arrays do not line up with the
        # frequency axis, has nothing to plot — and saying so beats an empty chart.
        return _document_result(
            summary + ". No vector had a plottable magnitude against frequency.",
            schema="ngspice-ac-sweep", operation="produced", document=result)
    return _bode_result(
        "AC sweep", f"{len(series)} vector(s), {len(freqs)} points", series + phase_series,
        summary + f"; {len(series)} charted as magnitude"
        + (f", {len(phase_series)} as phase" if phase_series else "") + ".",
        y_label="magnitude", y_unit="dB",
        # Phase on the second ordinate. A Bode plot is magnitude AND phase, and before the
        # contract had `axes.y2` the only ways to carry it were to drop it or to put degrees
        # on a decibel axis — the first loses the answer to "what is my phase margin", the
        # second mislabels every point of it.
        y2_axis=_axis("phase", "deg") if phase_series else None)


@mcp.tool(
    title="Per-component waveforms",
    description=(
        "Per-component voltage and current (switches, diodes, capacitors, resistors) "
        "from ONE ngspice run — what each part actually sees, for stress checking."
    ),
    meta=UI_CURVES_META,
    structured_output=False,
)
def component_waveforms(tas: dict, fidelity: dict | None = None,
                        component: str | None = None) -> CallToolResult:
    """V/I per power component.

    Args:
        tas: a TAS document from design_converter.
        fidelity: {"origin": ...}.
        component: chart only this ref (e.g. 'Q1'); default charts every one.
    """
    result = kh.component_waveforms(tas, _fidelity(fidelity))
    components = result.get("components") or []
    if not components:
        raise RuntimeError("the run produced no component waveforms")
    refs = [c.get("ref") for c in components]
    if component is not None:
        components = [c for c in components if c.get("ref") == component]
        if not components:
            raise ValueError(
                f"no component {component!r} in this design -- have: "
                f"{', '.join(str(r) for r in refs)}")

    # Each quantity carries {label, processed, waveform:{time, data}} — one
    # switching period, already resampled by the engine.
    series, stress = [], []
    for c in components:
        ref = c.get("ref") or "?"
        for quantity in ("voltage", "current"):
            block = c.get(quantity) or {}
            waveform = block.get("waveform") or {}
            time, data = waveform.get("time"), waveform.get("data")
            if not (isinstance(time, list) and isinstance(data, list) and len(time) == len(data)
                    and time):
                continue
            label = block.get("label") or quantity
            unit = "V" if quantity == "voltage" else "A"
            series.append(_va_series(f"{ref} {label}", unit, _decimate(time, data)))
            processed = block.get("processed") or {}
            stress.append(f"{ref} {label} peak {_eng(processed.get('peak'), unit)}, "
                          f"rms {_eng(processed.get('rms') or processed.get('average'), unit)}")
    if not series:
        raise RuntimeError("component waveforms carried no aligned time/data vectors")
    summary = (
        f"{len(components)} component(s) over one switching period "
        f"(reference period {result.get('referencePeriod')} s, engine {result.get('engine')}).\n"
        + "\n".join(stress[:12])
    )
    return _curves_result("Component waveforms", f"{len(series)} trace(s)", series, summary,
                          x_axis=TIME_AXIS, y_axis=V_AXIS, y2_axis=A_AXIS)


# --- tools: magnetics -------------------------------------------------------

@mcp.tool(
    title="Magnetic design inputs",
    description=(
        "A magnetic's MAS Inputs (designRequirements + operatingPoints) extracted from "
        "a designed converter — the handoff to the OpenMagnetics magnetic adviser."
    ),
    structured_output=False,
)
def magnetic_inputs(tas: dict, magnetic: str = "", enrich: bool = True) -> CallToolResult:
    """MAS Inputs for one magnetic.

    Args:
        tas: a TAS document from design_converter.
        magnetic: component name / BOM ref; '' picks the main magnetic (most windings).
        enrich: take the FULL-waveform inputs (real time-domain samples) instead of
            the processed-stats-only form. A 'custom'-shaped excitation (QRM valley
            switching, resonant bridge legs) cannot be reconstructed from stats, and
            the adviser refuses it with "Waveform must have at least 2 data points".
    """
    if enrich:
        # The engine already emits full-waveform inputs per magnetic; use them
        # rather than splicing waveforms into the stats form by hand (which is
        # what the web app's enrichMagneticWaveforms does in JS, and a second
        # implementation here would be free to drift from it).
        magnetics = kh.topology_waveforms(tas)
        if not magnetics:
            raise RuntimeError("this design carries no magnetics")
        if magnetic:
            hit = next((m for m in magnetics if m.get("name") == magnetic), None)
            if hit is None:
                raise ValueError(
                    f"no magnetic {magnetic!r} in this design -- have: "
                    f"{', '.join(str(m.get('name')) for m in magnetics)}")
        else:
            hit = next((m for m in magnetics if m.get("isMain")), magnetics[0])
        inputs = hit["inputs"]
        ops = inputs.get("operatingPoints") or []
        windings = len((ops[0] or {}).get("excitationsPerWinding") or []) if ops else 0
        return _document_result(
            f"Full-waveform MAS Inputs for {hit.get('name')}: {len(ops)} operating "
            f"point(s), {windings} winding(s), real time-domain samples included.",
            schema="MAS Inputs", operation="produced", document=inputs,
            derived_from=f"the {hit.get('name')} winding of this TAS",
            diagnostics=[f"full-waveform form: real time-domain samples, "
                         f"{windings} winding(s) over {len(ops)} operating point(s)"])
    inputs = kh.main_magnetic_inputs(tas, magnetic)
    ops = inputs.get("operatingPoints") or []
    windings = len((ops[0] or {}).get("excitationsPerWinding") or []) if ops else 0
    return _document_result(
        f"MAS Inputs for {magnetic or 'the main magnetic'}: {len(ops)} operating point(s), "
        f"{windings} winding(s). Feed to the OpenMagnetics adviser.",
        schema="MAS Inputs", operation="produced", document=inputs,
        derived_from=f"the {magnetic or 'main'} magnetic of this TAS",
        # Not a footnote: an adviser refuses a 'custom'-shaped excitation described by stats
        # alone with "Waveform must have at least 2 data points", and the caller needs to know
        # which form they are holding BEFORE they pass it on.
        diagnostics=["processed-statistics form: no time-domain samples — re-run with "
                     "enrich=true if the excitation is custom-shaped"])


@mcp.tool(
    title="All magnetics, full waveforms",
    description=(
        "Full-waveform MAS Inputs for EVERY magnetic in a design (real time-domain "
        "samples, not just processed stats) — what a 'custom'-shaped excitation "
        "(QRM valley switching, resonant legs) needs before advising. The main magnetic "
        "is the document, the rest are companions keyed by name."
    ),
    structured_output=False,
)
def topology_waveforms(tas: dict) -> CallToolResult:
    """MAS Inputs for every magnetic in the design, the main one first.

    NO CHART. This tool used to draw every magnetic's excitations, which made its payload two
    answers at once — a set of documents and a plot — and under the pipeline contract a result
    is one or the other. The documents are what the tool is FOR (they are what the adviser
    consumes), so they are what it returns; component_waveforms is the charting tool, and it
    charts the transformer's real V/I from the same run.
    """
    magnetics = kh.topology_waveforms(tas)
    if not magnetics:
        raise RuntimeError("this design carries no magnetics")
    names = [m.get("name") for m in magnetics]
    main = next((m for m in magnetics if m.get("isMain")), magnetics[0])
    # The main magnetic is the document; the others are companions, keyed by their own names.
    # A fixed 'the magnetic' key would be lossy the moment a design has more than one, which
    # a flyback with an output filter already does.
    companions = {}
    for m in magnetics:
        if m is main:
            continue
        name = m.get("name") or "unnamed magnetic"
        companions[name] = {"schema": {"name": "MAS Inputs"}, "subject": name,
                            "document": m.get("inputs") or {}}
    charted = sum(1 for m in magnetics
                  if _excitation_series(((m.get("inputs") or {}).get("operatingPoints") or [{}])[0]))
    summary = (f"{len(magnetics)} magnetic(s) with full waveforms: "
               f"{', '.join(str(n) for n in names)}"
               + (f" (main: {main.get('name')})" if main.get("isMain") else ""))
    return _document_result(
        summary + f". {charted} carry real time-domain samples.",
        schema="MAS Inputs", operation="produced", document=main.get("inputs") or {},
        companions=companions or None,
        derived_from="every magnetic in this TAS",
        # "Nobody looked" versus "there is nothing there": an excitation described by processed
        # statistics alone is refused by the adviser, and the caller has to know before it asks.
        diagnostics=([f"{len(magnetics) - charted} magnetic(s) carry processed statistics "
                      f"only — the adviser refuses a custom-shaped excitation without samples"]
                     if charted < len(magnetics) else None))


@mcp.tool(
    title="Magnetic operating point",
    description=(
        "A magnetic's MAS operating point, computed analytically or extracted from an "
        "ngspice run of the real circuit. The document itself — to chart the same "
        "waveforms, use component_waveforms."
    ),

    structured_output=False,
)
def operating_point(tas: dict, engine: str = "analytical", magnetic: str = "",
                    fidelity: dict | None = None) -> CallToolResult:
    """Excitations per winding for one magnetic.

    Args:
        engine: 'analytical' (closed form) or 'ngspice' (from a simulation).
        magnetic: component name; '' picks the main magnetic.
    """
    if engine not in ("analytical", "ngspice"):
        raise ValueError(f"engine must be 'analytical' or 'ngspice' -- got {engine!r}")
    op = kh.extract_operating_point(tas, engine, magnetic, _fidelity(fidelity))
    windings = len(op.get("excitationsPerWinding") or [])
    sampled = len(_excitation_series(op))
    who = magnetic or "the main magnetic"
    # The MAS operating point IS the answer — it is what the adviser and the loss models take.
    # It used to be returned as a chart with the document riding alongside, which made the
    # payload two shapes at once; the chart of the same waveforms is component_waveforms.
    return _document_result(
        f"Operating point for {who} ({engine}): {windings} winding excitation(s), "
        + (f"{sampled} with time-domain samples." if sampled
           else "described by processed statistics only — no time-domain samples."),
        schema="MAS", version="operatingPoint", operation="produced", document=op,
        derived_from=f"{who} of this TAS, {engine}",
        diagnostics=None if sampled else [
            "processed statistics only: an adviser refuses a custom-shaped excitation "
            "without at least two data points per waveform"])


# --- tools: parts sourcing --------------------------------------------------

@mcp.tool(
    title="Source real parts",
    description=(
        "Rank real catalogue parts for every fillable component in a design (Kelvin "
        "sourcing over the TAS parts DB). Returns candidates with MPN, manufacturer "
        "and the margins that ranked them."
    ),
    meta=UI_PICKER_META,
    structured_output=False,
)
def select_parts(tas: dict, data_dir: str | None = None, options: dict | None = None) -> CallToolResult:
    """Candidate parts per component.

    Args:
        tas: a TAS document from design_converter.
        data_dir: TAS catalogue directory; defaults to $KELVIN_TAS_DATA_DIR.
        options: selector options, e.g. {"topology": "flyback", "maxCandidates": 12}.
    """
    result = kh.select_components(tas, _kelvin_data_dir(data_dir), "", options or {})
    components = result.get("components") or []
    filled = sum(1 for c in components if c.get("filled"))
    # The full result carries every candidate's datasheet envelope — hundreds of
    # kilobytes. Only the digest goes to the model; the payload is the widget's.
    lines = []
    for c in components:
        n = len((c.get("selection") or {}).get("candidates") or [])
        if c.get("filled"):
            lines.append(f"  {c['ref']} ({c.get('family')}): {c.get('mpn')} — {n} candidate(s)")
        elif c.get("deferred"):
            # NOBODY LOOKED. A controller is deferred until the topology and operating point
            # are known; reporting that as "no candidate met the requirements" would send an
            # engineer hunting for a part that was never searched for.
            lines.append(f"  {c['ref']} ({c.get('family')}): not sourced — deferred"
                         + (f" ({c['deferred']})" if isinstance(c.get("deferred"), str) else ""))
        else:
            why = c.get("error") or "no candidate met the requirements"
            gates = c.get("rejections") or {}
            lines.append(
                f"  {c['ref']} ({c.get('family')}): UNFILLED — {why}"
                + (" · rejections: "
                   + ", ".join(f"{k}={v}" for k, v in
                               sorted(gates.items(), key=lambda kv: -kv[1])[:4]) if gates else ""))
    deferred = sum(1 for c in components if not c.get("filled") and c.get("deferred"))
    # A `bom` result: N questions with one answer each. The candidates ride on their own LINE
    # rather than in one pooled list, because "which part for Q1" is the question and a flat
    # list forgets which position each part was ranked for.
    #
    # `unsourced` versus `no_substitute` is the distinction the digest above also makes: a
    # controller is DEFERRED until the topology is known, and reporting that as "nothing met
    # the requirements" states a negative result nobody established.
    bom_lines = []
    for c in components:
        candidates = (c.get("selection") or {}).get("candidates") or []
        if c.get("filled"):
            status = "recommended"
        elif c.get("deferred"):
            status = "unsourced"
        else:
            status = "no_substitute"
        line = {"ref": c.get("ref") or "?", "status": status,
                "mpn": c.get("mpn") or None,
                "manufacturer": (c.get("manufacturer")
                                 or (candidates[0].get("manufacturer") if candidates else None)),
                "kind": c.get("family") or c.get("kind") or "",
                "candidates": [_candidate(cand) for cand in candidates]}
        notes = c.get("error") or (c.get("deferred") if isinstance(c.get("deferred"), str) else "")
        if notes:
            line["notes"] = str(notes)
        bom_lines.append({k: v for k, v in line.items() if v not in ("", [], None)
                          or k in ("mpn", "manufacturer")})
    diagnostics = [f"{c.get('ref')}: {c.get('error') or c.get('deferred')}"
                   for c in components
                   if not c.get("filled") and (c.get("error") or c.get("deferred"))]
    payload = {"mode": "bom", "lines": bom_lines, "total": len(components), "sourced": filled}
    topology = (tas.get("topology") or {}).get("name")
    if topology:
        payload["topology"] = str(topology)
    if diagnostics:
        payload["diagnostics"] = diagnostics
    return _result(
        f"{filled}/{len(components)} components sourced from the catalogue"
        + (f" ({deferred} deferred — not searched for, rather than not found)" if deferred else "")
        + ":\n" + "\n".join(lines),
        payload)


@mcp.tool(
    title="Rank parts for one component",
    description=(
        "Rank a single component's candidates from the prebuilt Kelvin index shards, "
        "with converter context (switching frequency, input voltage, topology) and a "
        "manufacturer-diversity cap. The per-component entry point; select_parts walks "
        "a whole design instead."
    ),
    meta=UI_PICKER_META,
    structured_output=False,
)
def select_candidates(kind: str, requirements: dict, context: dict | None = None,
                      max_candidates: int = 12) -> CallToolResult:
    """Candidates for one component kind.

    Args:
        kind: 'MOSFET', 'Diode', 'Capacitor', 'Resistor', 'Controller', 'IGBT',
            'BJT', 'Varistor', 'Inductor' or 'Transformer'.
        requirements: that component's designRequirements block.
        context: optional {switchingFrequency, inputVoltage, topology,
            maxManufacturerFraction, manufacturerAllowlist}.
    """
    category = KIND_TO_CATEGORY.get(kind) or KIND_TO_CATEGORY.get(kind.capitalize())
    if not category:
        raise ValueError(
            f"no Kelvin category for kind {kind!r} -- one of: {', '.join(sorted(KIND_TO_CATEGORY))}")
    _ensure_shard(category)
    options = {"maxCandidates": max_candidates,
               # Kelvin defaults to no cap to stay parity-locked; the consumer
               # opts in, or one vendor fills the whole ranked list.
               "maxManufacturerFraction": KELVIN_MAX_MFR_FRACTION}
    for key in ("switchingFrequency", "inputVoltage", "topology",
                "maxManufacturerFraction", "manufacturerAllowlist"):
        if (context or {}).get(key) is not None:
            options[key] = context[key]
    result = kh.kelvin_select(category, requirements, options)
    cands = result.get("candidates") or []
    # A `search` result — the ranked branch. Even the empty case is one: "nothing qualified"
    # is an answer with a reason attached, and the rejections say WHICH gate rejected how
    # many, which is the only way to answer "why is my part not here".
    payload = {"mode": "search", "family": category,
               "candidates": [_candidate(c) for c in cands]}
    # `total` is what SURVIVED the gates, `catalogueTotal` is what the family holds — the
    # engine calls them alternativesConsidered and totalRowsConsidered, and the pair reconciles
    # with `rejections`: catalogueTotal - sum(rejections) = total.
    if isinstance(result.get("alternativesConsidered"), int):
        payload["total"] = result["alternativesConsidered"]
    if isinstance(result.get("totalRowsConsidered"), int):
        payload["catalogueTotal"] = result["totalRowsConsidered"]
    if isinstance(result.get("rejections"), dict):
        payload["rejections"] = result["rejections"]
    if result.get("tiebreaker"):
        payload["tiebreaker"] = str(result["tiebreaker"])
    # Which vendors are represented is the answer to "did one manufacturer fill the list",
    # which is why the diversity cap exists at all. A facet is where that fact belongs.
    manufacturers = result.get("manufacturers")
    if isinstance(manufacturers, list) and manufacturers:
        counts = {}
        for c in cands:
            name = c.get("manufacturer")
            if name:
                counts[name] = counts.get(name, 0) + 1
        payload["facets"] = {"manufacturer": {"values": [
            {"value": name, "count": counts.get(name, 0)} for name in manufacturers]}}
    payload["shown"] = len(cands)
    if result.get("error"):
        payload["caveat"] = str(result["error"])
        return _result(
            f"No {kind} candidate met the requirements ({result['error']}); "
            f"{len(result.get('rejections') or [])} rejection reason(s) recorded.",
            payload)
    detail = "\n".join(f"  {c.get('mpn')} — {c.get('manufacturer')}" for c in cands[:12])
    return _result(f"{len(cands)} {kind} candidate(s), best first:\n{detail}", payload)


@mcp.tool(
    title="Magnetic inputs from a spec",
    description=(
        "A magnetic's MAS Inputs straight from a converter spec, for ANY topology, "
        "without designing or carrying a TAS — the entry point the OpenMagnetics "
        "wizards consume."
    ),
    structured_output=False,
)
def design_magnetic_inputs(topology: str, spec: dict) -> CallToolResult:
    """MAS Inputs for the main magnetic of `topology` built for `spec`."""
    inputs = kh.design_magnetic_inputs(_topology_id(topology), spec)
    ops = inputs.get("operatingPoints") or []
    return _document_result(
        f"MAS Inputs for a {topology} magnetic: {len(ops)} operating point(s), "
        f"straight from the spec (no TAS needed).",
        schema="MAS Inputs", operation="produced", document=inputs,
        derived_from=f"a {topology} spec")


@mcp.tool(
    title="PFC conduction mode",
    description=(
        "Which conduction mode (CCM / BCM / DCM) a PFC stage runs in for a given "
        "boost inductor."
    ),
    structured_output=False,
)
def pfc_mode(spec: dict, inductance: float) -> CallToolResult:
    """Conduction mode of a PFC stage.

    Args:
        inductance: the boost inductor under consideration, H.
    """
    result = kh.determine_pfc_mode(spec, inductance)
    critical = result.get("criticalInductance")
    # The conduction mode is a CATEGORY, not a number, so it carries no unit — the contract
    # allows exactly that, and making it claim the dimensionless '1' would have meant two
    # different things under one field.
    quantities = {"conductionMode": {"value": str(result.get("actualMode") or "unknown"),
                                     "label": "conduction mode"}}
    quantities |= _quantities({
        "criticalInductance": (critical, "H", "boundary between CCM and BCM/DCM"),
        "inductance": (inductance, "H", "the boost inductor under consideration"),
    })
    return _quantity_result(
        f"PFC runs in {result.get('actualMode')} with a {_eng(inductance, 'H')} boost "
        f"inductor (critical inductance {_eng(critical, 'H')} — above it CCM, below it BCM/DCM).",
        subject="the PFC boost stage", model="Kirchhoff critical-inductance boundary",
        quantities=quantities)


@mcp.tool(
    title="Run a raw SPICE deck",
    description=(
        "Run any ngspice deck through the IN-PROCESS libngspice, executing its "
        "`.control … .endc` block, and return the console output (with .meas results). "
        "The in-process replacement for `ngspice -b deck.cir`."
    ),
    structured_output=False,
)
def run_deck(deck: str | None = None, timeout_s: float = 600.0,
             deck_ref: str | None = None) -> CallToolResult:
    """Console output of a raw deck run in-process.

    Args:
        deck: the deck itself.
        deck_ref: the deck by reference instead — a local path, file://, artifact://<id>
            (resolved against KIRCHHOFF_ARTIFACT_BASE) or an https:// URL. The shared
            convention across the OpenConverters servers (artifacts.py, ABT #661/#656), so a
            long deck never travels through the tool arguments.
    """
    deck = _deck_text(deck, deck_ref)
    output = kh.run_ngspice_console(deck, timeout_s)
    lines = output.splitlines()
    measured = [l for l in lines if "=" in l and not l.strip().startswith("*")]
    tail = "\n".join(measured[-25:]) or "\n".join(lines[-25:])
    return _document_result(
        f"Deck ran in-process ({len(lines)} lines of console output). Tail:\n{tail}",
        schema="ngspice-console", operation="produced",
        document={"text": output, "lines": len(lines)},
        # The .meas lines are the reason anyone runs a raw deck; naming them saves a consumer
        # re-deciding which of 400 console lines were measurements.
        diagnostics=measured[-25:] or None)


@mcp.tool(
    title="Bind a part into the design",
    description=(
        "Stamp a chosen candidate's datasheet envelope into one component, returning "
        "the updated TAS at DATASHEET fidelity."
    ),

    meta=UI_PICKER_META,
    structured_output=False,
)
def bind_part(tas: dict, ref: str, envelope: dict) -> CallToolResult:
    """Bind a real part.

    Args:
        ref: the component reference in the design, e.g. 'Q1'.
        envelope: the candidate's full datasheet envelope (from select_parts).
    """
    bound = kh.bind_part(tas, ref, envelope)
    mpn = envelope.get("mpn") or (envelope.get("manufacturerInfo") or {}).get("reference")
    return _document_result(
        f"Bound {mpn or 'the chosen part'} into {ref}; the returned TAS is at DATASHEET "
        f"fidelity for that component.",
        schema="TAS", operation="transformed", document=bound,
        # The ref is an ARGUMENT to this call, so the delta is known before the engine runs —
        # which is why the contract can require `changed` rather than hope for it.
        changed=[{"ref": ref, "change": "bound to a catalogue part",
                  "mpn": mpn or None, "fidelity": "DATASHEET"}])


@mcp.tool(
    title="Cross-reference a part",
    description=(
        "Deterministic, program-only substitute ranking for an original part — no LLM "
        "judgement. Returns candidates scored best-first with the penalty that ranked them."
    ),

    meta=UI_PICKER_META,
    structured_output=False,
)
def cross_reference(category: str, original: dict, candidates: list[dict],
                    original_verified: bool = True, max_results: int = 12) -> CallToolResult:
    """Scored drop-in substitutes.

    Args:
        category: 'mosfet', 'diode', 'capacitor', 'resistor', 'magnetic', ...
        original: the original part's parameter block.
        candidates: parts to rank (e.g. from select_parts).
        original_verified: False applies the honesty cap — an unidentified original
            never yields a 'recommended' verdict.
    """
    result = kh.cross_reference(category, original, candidates,
                                {"original_verified": original_verified,
                                 "max_results": max_results})
    ranked = result.get("candidates") or []
    detail = "\n".join(
        f"  {c.get('mpn')}: {c.get('status')} (penalty {c.get('penalty')})" for c in ranked[:12])
    # A `crossref` result. `original` is an OBJECT with an mpn, never a bare string: a
    # comparison whose subject is a name and whose candidates are parameter blocks cannot be
    # rendered side by side, which is the entire point of a cross-reference.
    original_mpn = (original.get("mpn")
                    or (original.get("manufacturerInfo") or {}).get("reference") or "(unnamed)")
    payload = {"mode": "crossref", "family": category,
               "candidates": [_candidate(c) for c in ranked],
               "original": {"mpn": str(original_mpn)},
               "originalSpecs": {k: v for k, v in original.items() if k != "manufacturerInfo"},
               "shown": len(ranked)}
    if isinstance(result.get("total"), int):
        payload["total"] = result["total"]
    if isinstance(result.get("rejections"), dict):
        payload["rejections"] = result["rejections"]
    if not original_verified:
        # The honesty cap, as a field rather than only a sentence: an unidentified original
        # can never yield a 'recommended' verdict, and a consumer must be able to act on that
        # without parsing the digest.
        payload["originalVerified"] = False
        payload["caveat"] = ("the original was not verified, so no candidate is marked "
                             "'recommended'")
    return _result(
        f"{len(ranked)} substitute(s) for the {category}, best first"
        + (" — original unverified, so nothing is 'recommended'"
           if not original_verified else "") + f":\n{detail}",
        payload)


# --- tools: EMI filter components ------------------------------------------

@mcp.tool(
    title="Design a common-mode choke",
    description=(
        "Size a common-mode choke from an EMI spec and return the MAS Inputs for the "
        "magnetic adviser, plus the impedance the sizing was driven by."
    ),

    structured_output=False,
)
def design_cmc(spec: dict) -> CallToolResult:
    """CMC component design.

    Args:
        spec: operatingVoltage, operatingCurrent, lineFrequency, ambientTemperature,
            and ONE of minimumImpedance[], targetInsertionLoss[],
            parasiticCap_pF + dvdt_V_ns, or desiredInductance.
    """
    result = kh.design_cmc_inputs(spec)
    d = result.get("cmcDiagnostics") or {}
    # MAS Inputs for the adviser, with the sizing that drove them as diagnostics. The
    # inductance and the frequency it was chosen at are the two numbers a reader checks, so
    # they are stated rather than left inside a nested block.
    return _document_result(
        f"CMC sized: L = {_eng(d.get('computedInductance'), 'H')} at the dominant "
        f"frequency {_eng(d.get('dominantFrequency'), 'Hz')} "
        f"(|Z| {_eng(d.get('dominantImpedance'), 'Ω')}). MAS Inputs are in the "
        f"structured output for the magnetic adviser.",
        schema="MAS Inputs", operation="produced",
        document={k: v for k, v in result.items() if k != "cmcDiagnostics"},
        derived_from="a common-mode filter spec",
        diagnostics=[f"computed inductance {_eng(d.get('computedInductance'), 'H')}",
                     f"dominant frequency {_eng(d.get('dominantFrequency'), 'Hz')}",
                     f"|Z| at the dominant frequency {_eng(d.get('dominantImpedance'), 'Ω')}"])


@mcp.tool(
    title="Design a differential-mode choke",
    description=(
        "Size a differential-mode choke from an EMI spec and return the MAS Inputs for "
        "the magnetic adviser, plus the computed inductance and frequency band."
    ),

    structured_output=False,
)
def design_dmc(spec: dict) -> CallToolResult:
    """DMC component design.

    Args:
        spec: configuration, inputVoltage, operatingCurrent, lineFrequency,
            switchingFrequency?, ambientTemperature, and one of
            minimumImpedance[] / minimumInductance.
    """
    result = kh.design_dmc_inputs(spec)
    d = result.get("dmcDiagnostics") or {}
    return _document_result(
        f"DMC sized: L = {_eng(d.get('computedInductance'), 'H')}, "
        f"{d.get('numberWindings')} winding(s), band "
        f"{_eng(d.get('computedMinFrequency'), 'Hz')}–{_eng(d.get('computedMaxFrequency'), 'Hz')} "
        f"(|Z| at f_min {_eng(d.get('impedanceAtMinFrequency'), 'Ω')}).",
        schema="MAS Inputs", operation="produced",
        document={k: v for k, v in result.items() if k != "dmcDiagnostics"},
        derived_from="a differential-mode filter spec",
        diagnostics=[f"computed inductance {_eng(d.get('computedInductance'), 'H')}",
                     f"{d.get('numberWindings')} winding(s)",
                     f"band {_eng(d.get('computedMinFrequency'), 'Hz')} to "
                     f"{_eng(d.get('computedMaxFrequency'), 'Hz')}",
                     f"|Z| at f_min {_eng(d.get('impedanceAtMinFrequency'), 'Ω')}"])


@mcp.tool(
    title="Propose a DM filter LC",
    description=(
        "'Help me size it' for a differential-mode filter: the LC pair, cutoff and "
        "target attenuation. Feed the proposed inductance back to design_dmc."
    ),

    structured_output=False,
)
def propose_dmc(spec: dict) -> CallToolResult:
    """LC sizing for a DM filter stage."""
    result = kh.propose_dmc_design(spec)
    return _quantity_result(
        f"Proposed LC: L = {_eng(result.get('inductance'), 'H')}, "
        f"C = {_eng(result.get('capacitance'), 'F')}, "
        f"cutoff {_eng(result.get('cutoffFrequency'), 'Hz')}, target attenuation "
        f"{result.get('targetAttenuation_dB')} dB. Re-run design_dmc with this "
        f"inductance as minimumInductance.",
        subject="a differential-mode filter stage",
        model="Kirchhoff LC sizing from the attenuation target",
        quantities=_quantities({
            "inductance": (result.get("inductance"), "H"),
            "capacitance": (result.get("capacitance"), "F"),
            "cutoffFrequency": (result.get("cutoffFrequency"), "Hz"),
            # The unit was in the NAME here (`targetAttenuation_dB`), which is exactly what
            # the contract forbids: it has to be renamed the day it is reported in nepers.
            "targetAttenuation": (result.get("targetAttenuation_dB"), "dB"),
        }))


@mcp.tool(
    title="Verify DM filter attenuation",
    description=(
        "Check a DMC + filter capacitor against the required attenuation at every test "
        "frequency, simulated in-process. Reports measured vs theoretical vs required."
    ),

    meta=UI_BODE_META,
    structured_output=False,
)
def verify_dmc(spec: dict, inductance: float, capacitance: float = 0.0) -> CallToolResult:
    """Per-frequency pass/fail for a DM filter.

    Args:
        inductance: the choke inductance under test, H.
        capacitance: shared filter capacitor, F; 0 auto-sizes from the spec.
    """
    rows = kh.verify_dmc_attenuation(spec, inductance, capacitance)
    passed = sum(1 for r in rows if r.get("passed"))
    detail = "\n".join(
        f"  {r.get('frequency'):.0f} Hz: required {r.get('requiredAttenuation'):.1f} dB, "
        f"measured {r.get('measuredAttenuation') if r.get('measuredAttenuation') is not None else 'n/a'}"
        f" -> {'PASS' if r.get('passed') else 'FAIL'}"
        for r in rows[:15])
    summary = f"{passed}/{len(rows)} frequencies meet the requirement:\n{detail}"

    def curve(key):
        return [[float(r["frequency"]), float(r[key])] for r in rows
                if isinstance(r.get("frequency"), (int, float)) and r["frequency"] > 0
                and isinstance(r.get(key), (int, float))]

    series = []
    for key, name, kind in (("requiredAttenuation", "required", "limit"),
                            ("measuredAttenuation", "measured (simulated)", "measured"),
                            ("theoreticalAttenuation", "theoretical", "modelled")):
        pts = curve(key)
        if len(pts) > 1:
            series.append({"name": name, "points": pts, "kind": kind})

    # A `verdict` result: this is the one tool here that judges rather than computes, and the
    # contract's verdict branch carries the parts a reader must act on without reading prose —
    # what it was judged against, whether it is provisional, and every point that breached it.
    #
    # `kind` on each series is not decoration either: presenting the THEORETICAL curve as the
    # measured one would turn a closed form into a simulation result.
    # ONLY frequencies that were measured and missed. A frequency the simulation could not
    # measure has not exceeded anything — listing it with a fabricated value and a zero margin
    # would put a breach on the chart that nobody observed. Those are counted separately, in
    # `frequenciesUnmeasured`, and named in the caveat.
    exceedances = [
        {"at": float(r["frequency"]),
         "value": float(r["measuredAttenuation"]),
         "limit": float(r["requiredAttenuation"]),
         "margin": float(r["measuredAttenuation"]) - float(r["requiredAttenuation"])}
        for r in rows
        if not r.get("passed") and isinstance(r.get("frequency"), (int, float))
        and isinstance(r.get("requiredAttenuation"), (int, float))
        and isinstance(r.get("measuredAttenuation"), (int, float))]
    unmeasured = sum(1 for r in rows if not isinstance(r.get("measuredAttenuation"), (int, float)))
    payload = {
        "mode": "verdict",
        # 'unverified' is absence of data, never a synonym for a failure: a frequency the
        # simulation could not measure has not passed and has not failed.
        "verdict": ("pass" if passed == len(rows) and not unmeasured
                    else "unverified" if unmeasured == len(rows) else "fail"),
        "criterion": f"the spec's required DM attenuation at {len(rows)} test frequencies",
        # Simulated attenuation of an LC pair, not a measurement of hardware in a chamber.
        "provisional": True,
        "measurements": {
            "inductance": {"value": float(inductance), "unit": "H"},
            "capacitance": {"value": float(capacitance), "unit": "F",
                            "label": "0 means auto-sized from the spec"},
            "frequenciesPassed": {"value": passed, "unit": "1"},
            "frequenciesTested": {"value": len(rows), "unit": "1"},
            "frequenciesUnmeasured": {"value": unmeasured, "unit": "1"},
        },
    }
    if exceedances:
        payload["exceedances"] = sorted(exceedances, key=lambda e: e["margin"])
        payload["worst"] = payload["exceedances"][0]
    if series:
        payload["axes"] = {"x": _axis("frequency", "Hz", "log"),
                           "y": _axis("attenuation", "dB")}
        payload["series"] = series
    if unmeasured:
        payload["caveat"] = (f"{unmeasured} of {len(rows)} frequencies returned no measured "
                             f"attenuation — they are neither a pass nor a failure")
    return _result(summary, payload)


@mcp.tool(
    title="Design a current transformer",
    description=(
        "Size a burden-resistor current-sense transformer and return its MAS Inputs "
        "(2-winding transformer + the sensing operating point)."
    ),
    structured_output=False,
)
def design_current_transformer(spec: dict) -> CallToolResult:
    """Current-transformer component design.

    Args:
        spec: waveformLabel, maximumPrimaryCurrentPeak, frequency, turnsRatio,
            burdenResistor, ambientTemperature (+ optional secondaryDcResistance,
            dutyCycle, diodeVoltageDrop).

    waveformLabel takes the MAS camelCase spelling — 'sinusoidal',
    'unipolarRectangular' or 'unipolarTriangular'. (The engine's rejection
    message names the C++ enum constants, SINUSOIDAL / UNIPOLAR_RECTANGULAR /
    UNIPOLAR_TRIANGULAR, which the JSON parser does NOT accept.)
    """
    label = spec.get("waveformLabel")
    allowed = ("sinusoidal", "unipolarRectangular", "unipolarTriangular")
    if label is not None and label not in allowed:
        raise ValueError(
            f"waveformLabel {label!r} is not a MAS label -- use one of {', '.join(allowed)}"
        )
    result = kh.design_current_transformer_inputs(spec)
    ops = result.get("operatingPoints") or []
    windings = len((ops[0] or {}).get("excitationsPerWinding") or []) if ops else 0
    return _document_result(
        f"Current transformer sized: MAS Inputs with {windings} winding(s) and "
        f"{len(ops)} operating point(s), ready for the magnetic adviser.",
        schema="MAS Inputs", operation="produced", document=result,
        derived_from="a current-sense transformer spec")


# --- the MCP Apps UI resources ---------------------------------------------

def _widget(filename: str) -> str:
    bundle = Path(__file__).parent / "dist" / filename
    if not bundle.exists():
        raise FileNotFoundError(
            f"{bundle} missing -- build the widgets first: cd mcp && npm install && npm run build"
        )
    return bundle.read_text(encoding="utf-8")


@mcp.resource(
    UI_SCHEMATIC_URI,
    name="kirchhoff-schematic-widget",
    title="Kirchhoff schematic",
    mime_type=UI_RESOURCE_MIME,
)
def schematic_widget() -> str:
    """The web app's own CIAS-generated schematic, with click-to-select components."""
    return _widget("schematic.html")


@mcp.resource(
    UI_CURVES_URI,
    name="kirchhoff-curves-widget",
    title="Kirchhoff waveforms",
    mime_type=UI_RESOURCE_MIME,
)
def curves_widget() -> str:
    """Transient / sweep chart for simulation results."""
    return _widget("curves.html")


@mcp.resource(
    UI_BODE_URI,
    name="kirchhoff-bode-widget",
    title="Kirchhoff frequency response",
    mime_type=UI_RESOURCE_MIME,
)
def bode_widget() -> str:
    """dB against a LOG frequency axis — an AC sweep, or a filter's attenuation against the
    requirement it is judged by, with the per-frequency verdict drawn on the curve."""
    return _widget("bode.html")


@mcp.resource(
    UI_PICKER_URI,
    name="kirchhoff-picker-widget",
    title="Kirchhoff part picker",
    mime_type=UI_RESOURCE_MIME,
)
def picker_widget() -> str:
    """Ranked candidate parts with their specs, verdicts and margins — click one and the
    choice goes back to the model. Every sourcing tool ends in the same decision, so they
    share it."""
    return _widget("picker.html")



def _auth_middleware(app, prefix: str):
    """Optional bearer-token auth in front of the transport.

    OFF unless {PREFIX}_AUTH_TOKEN is set, because the default deployment is loopback and a
    token nobody configured would be security theatre with a support cost. Set it and every
    request must carry `Authorization: Bearer <token>`; the MCP endpoints are all that is
    protected, and the failure is a plain 401 rather than a redirect, so a client sees what
    happened instead of guessing at OAuth (ABT #656).

    This is a gate, not an identity: one shared token says the caller is allowed in, not who
    they are. Anything needing per-user identity wants a real IdP in front, and this is not a
    substitute for one.
    """
    import os as _os

    token = _os.environ.get(f"{prefix}_AUTH_TOKEN", "").strip()
    if not token:
        return app

    from starlette.responses import PlainTextResponse

    class _BearerGate:
        def __init__(self, inner):
            self.inner = inner

        async def __call__(self, scope, receive, send):
            if scope.get("type") != "http":
                await self.inner(scope, receive, send)
                return
            headers = {k.decode().lower(): v.decode() for k, v in scope.get("headers") or []}
            if headers.get("authorization", "") != f"Bearer {token}":
                response = PlainTextResponse(
                    f"401 Unauthorized: this server requires a bearer token "
                    f"({prefix}_AUTH_TOKEN).", status_code=401)
                await response(scope, receive, send)
                return
            await self.inner(scope, receive, send)

    return _BearerGate(app)


def build_app():
    """Starlette app with CORS.

    Browser-resident MCP hosts fetch /mcp from page JavaScript, so without these
    headers the connection dies at the preflight -- and the streamable transport
    additionally needs to READ `Mcp-Session-Id` off the response, which
    cross-origin JS cannot do unless the header is explicitly exposed.
    """
    from starlette.middleware.cors import CORSMiddleware

    app = mcp.streamable_http_app()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],          # tighten to your host origins in production
        allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
        allow_headers=["*"],
        expose_headers=["Mcp-Session-Id"],
    )
    return _auth_middleware(app, "KIRCHHOFF")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(build_app(), host=mcp.settings.host, port=mcp.settings.port)
