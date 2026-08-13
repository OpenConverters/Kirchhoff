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


def _curves_result(title, subtitle, x_label, y_label, series, summary, note=None, data=None):
    """Tool result for the curves widget.

    Two channels, as in the Hertz server: `content` is the digest the model
    reads, `structuredContent` is the payload only the widget reads.

    `data` carries the tool's OWN result alongside the chart. A tool that returns an
    operating point and happens to draw it must still return the operating point — charting
    a result is not a reason to stop returning it, and a caller that wants to feed it onward
    would otherwise have to call twice.
    """
    return CallToolResult(
        content=[TextContent(type="text", text=summary)],
        structuredContent={
            **(data or {}),
            "title": title, "subtitle": subtitle,
            "x_label": x_label, "y_label": y_label,
            "series": series, "note": note,
        },
    )


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
            series.append({"name": f"{prefix}{name} {quantity}", "unit": unit,
                           "points": _decimate(time, data)})
    return series



def _bode_result(title, subtitle, series, summary, y_label="dB", x_label="frequency (Hz)",
                 markers=None, note=None, data=None):
    """Tool result for the frequency-domain widget.

    Separate from _curves_result because the axes are genuinely different: that one is dual
    V/A against time and picks its axis from the unit, this one is one quantity against a log
    frequency axis. Sharing a widget between them would mean labelling one of the two wrongly.
    """
    return CallToolResult(
        content=[TextContent(type="text", text=summary)],
        structuredContent={
            **(data or {}),
            "title": title, "subtitle": subtitle,
            "x_label": x_label, "y_label": y_label,
            "series": series, "markers": markers or [], "note": note,
        },
    )



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
        {
        "topologies": list(TOPOLOGIES),
        "count": len(TOPOLOGIES),
        "spec_shape": {
            "designRequirements": {
                "efficiency": 1.0,
                "inputVoltage": {"minimum": 45.6, "nominal": 48, "maximum": 50.4},
                "switchingFrequency": {"nominal": 100000},
                "outputs": [{"name": "out", "voltage": {"nominal": 12}}],
            },
            "operatingPoints": [{"inputVoltage": 48, "outputs": [{"power": 24}]}],
        },
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
    return CallToolResult(
        content=[TextContent(type="text", text=summary)],
        structuredContent={
            "topology": topology_id,
            "tas": tas,
            "diagnostics": diagnostics,
            "operating_point": result.get("operatingPoint"),
        },
    )


@mcp.tool(
    title="Converter diagnostics",
    description="The engine's component sizing and design diagnostics for an assembled TAS.",

    structured_output=False,
)
def converter_diagnostics(tas: dict) -> CallToolResult:
    """Sized components and design diagnostics for a TAS from design_converter."""
    diag = kh.diagnostics(tas)
    return _result(_diagnostics_digest(diag), {"diagnostics": diag})


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
    return _result(
        "Datasheet device models added. Pass this TAS to export_netlist or simulate "
        'with fidelity {"origin": "DATASHEET"} to see real conduction losses.',
        {"tas": realized})


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
    return _result(f"{flavor} deck, {len(lines)} lines:\n\n{shown}",
                   {"flavor": flavor, "netlist": deck, "lines": len(lines)})


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
    return _result(
        f"Transient {result.get('tStart', 0.0):.4g}-{result.get('tEnd', 0.0):.4g} s, "
        f"{result.get('points')} points, {len(measurements)} vectors "
        f"(in-process libngspice):\n{rows}{more}",
        {
            "measurements": measurements,
            "t_start_s": result.get("tStart"),
            "t_end_s": result.get("tEnd"),
            "points": result.get("points"),
            "engine": "in-process libngspice",
        })


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
    # Magnitude in dB is what a sweep is read as. The complex arrays stay in the payload —
    # phase, real and imaginary parts are all still there for a caller that wants them.
    series = []
    for name in sorted(vectors):
        v = vectors[name] or {}
        re_, im = v.get("re") or [], v.get("im") or []
        if len(re_) != len(freqs) or len(im) != len(freqs):
            continue
        points = []
        for f, a, b in zip(freqs, re_, im):
            mag = (a * a + b * b) ** 0.5
            if f > 0 and mag > 0:
                points.append([float(f), 20.0 * math.log10(mag)])
        if len(points) > 1:
            series.append({"name": name, "points": _decimate([p[0] for p in points],
                                                             [p[1] for p in points])})
    summary = (f"AC sweep: {len(freqs)} points"
               + (f" from {freqs[0]:.4g} to {freqs[-1]:.4g} Hz" if freqs else "")
               + f", {len(vectors)} vector(s): {', '.join(sorted(vectors))[:200]}")
    if not series:
        # A sweep whose magnitudes are all zero, or whose arrays do not line up with the
        # frequency axis, has nothing to plot — and saying so beats an empty chart.
        return _result(summary + ". No vector had a plottable magnitude against frequency.",
                       result)
    return _bode_result(
        "AC sweep", f"{len(series)} vector(s), {len(freqs)} points", series,
        summary + f"; {len(series)} charted as magnitude.",
        y_label="magnitude (dB)", data=result)


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
            series.append({"name": f"{ref} {label}", "unit": unit,
                           "points": _decimate(time, data)})
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
    return _curves_result("Component waveforms", f"{len(series)} trace(s)",
                          "time (s)", "V / A", series, summary)


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
        return _result(
            f"Full-waveform MAS Inputs for {hit.get('name')}: {len(ops)} operating "
            f"point(s), {windings} winding(s), real time-domain samples included.",
            {"inputs": inputs, "magnetic": hit.get("name"), "enriched": True})
    inputs = kh.main_magnetic_inputs(tas, magnetic)
    ops = inputs.get("operatingPoints") or []
    windings = len((ops[0] or {}).get("excitationsPerWinding") or []) if ops else 0
    return _result(
        f"MAS Inputs for {magnetic or 'the main magnetic'}: {len(ops)} operating point(s), "
        f"{windings} winding(s). Feed to the OpenMagnetics adviser.",
        {"inputs": inputs})


@mcp.tool(
    title="All magnetics, full waveforms",
    description=(
        "Full-waveform MAS Inputs for EVERY magnetic in a design (real time-domain "
        "samples, not just processed stats) — what a 'custom'-shaped excitation "
        "(QRM valley switching, resonant legs) needs before advising."
    ),
    meta=UI_CURVES_META,
    structured_output=False,
)
def topology_waveforms(tas: dict) -> CallToolResult:
    """One entry per magnetic: {name, isMain, inputs}."""
    magnetics = kh.topology_waveforms(tas)
    names = [m.get("name") for m in magnetics]
    main = next((m.get("name") for m in magnetics if m.get("isMain")), None)
    # Every magnetic's first operating point, charted together and labelled by magnetic:
    # the point of asking for ALL of them is to compare them.
    series = []
    for m in magnetics:
        ops = ((m.get("inputs") or {}).get("operatingPoints")) or []
        if ops:
            series += _excitation_series(ops[0], prefix=f"{m.get('name') or '?'} ")
    summary = (f"{len(magnetics)} magnetic(s) with full waveforms: "
               f"{', '.join(str(n) for n in names)}" + (f" (main: {main})" if main else ""))
    if not series:
        return _result(
            summary + ". No time-domain samples reached the excitations, so there is nothing "
            "to chart — which is itself the answer if you were about to advise on them.",
            {"magnetics": magnetics, "count": len(magnetics), "names": names})
    return _curves_result(
        "Magnetic excitations", f"{len(magnetics)} magnetic(s), {len(series)} waveform(s)",
        "time (s)", "V / A", series,
        summary + f", {len(series)} charted waveform(s).",
        data={"magnetics": magnetics, "count": len(magnetics), "names": names})


@mcp.tool(
    title="Magnetic operating point",
    description=(
        "A magnetic's MAS operating point, computed analytically or extracted from an "
        "ngspice run of the real circuit."
    ),

    meta=UI_CURVES_META,
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
    series = _excitation_series(op)
    who = magnetic or "the main magnetic"
    if not series:
        # An excitation the engine described by processed statistics alone has no curve.
        # Saying so beats an empty chart, and beats drawing a shape from the stats that the
        # engine never computed.
        return _result(
            f"Operating point for {who} ({engine}): {windings} winding excitation(s). The "
            f"engine described them by processed statistics only, so there are no "
            f"time-domain samples to chart.",
            {"operating_point": op})
    return _curves_result(
        f"{magnetic or 'Main magnetic'} — operating point ({engine})",
        f"{windings} winding excitation(s), {len(series)} waveform(s)",
        "time (s)", "V / A", series,
        f"Operating point for {who} ({engine}): {windings} winding excitation(s), "
        f"{len(series)} charted waveform(s).",
        data={"operating_point": op})


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
    return _result(
        f"{filled}/{len(components)} components sourced from the catalogue"
        + (f" ({deferred} deferred — not searched for, rather than not found)" if deferred else "")
        + ":\n" + "\n".join(lines),
        {
            "components": components,
            "filled": filled,
            "total": len(components),
        })


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
    if result.get("error"):
        return _result(
            f"No {kind} candidate met the requirements ({result['error']}); "
            f"{len(result.get('rejections') or [])} rejection reason(s) recorded.",
            result)
    cands = result.get("candidates") or []
    detail = "\n".join(f"  {c.get('mpn')} — {c.get('manufacturer')}" for c in cands[:12])
    return _result(f"{len(cands)} {kind} candidate(s), best first:\n{detail}", result)


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
    return _result(
        f"MAS Inputs for a {topology} magnetic: {len(ops)} operating point(s), "
        f"straight from the spec (no TAS needed).",
        {"inputs": inputs})


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
    return _result(
        f"PFC runs in {result.get('actualMode')} with a {_eng(inductance, 'H')} boost "
        f"inductor (critical inductance {_eng(critical, 'H')} — above it CCM, below it BCM/DCM).",
        result)


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
    return _result(f"Deck ran in-process ({len(lines)} lines of console output). Tail:\n{tail}",
                   {"console": output, "lines": len(lines)})


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
    return _result(
        f"Bound {mpn or 'the chosen part'} into {ref}; the returned TAS is at DATASHEET "
        f"fidelity for that component.",
        {"tas": bound})


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
    return _result(
        f"{len(ranked)} substitute(s) for the {category}, best first"
        + (" — original unverified, so nothing is 'recommended'"
           if not original_verified else "") + f":\n{detail}",
        result)


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
    return _result(
        f"CMC sized: L = {_eng(d.get('computedInductance'), 'H')} at the dominant "
        f"frequency {_eng(d.get('dominantFrequency'), 'Hz')} "
        f"(|Z| {_eng(d.get('dominantImpedance'), 'Ω')}). MAS Inputs are in the "
        f"structured output for the magnetic adviser.",
        result)


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
    return _result(
        f"DMC sized: L = {_eng(d.get('computedInductance'), 'H')}, "
        f"{d.get('numberWindings')} winding(s), band "
        f"{_eng(d.get('computedMinFrequency'), 'Hz')}–{_eng(d.get('computedMaxFrequency'), 'Hz')} "
        f"(|Z| at f_min {_eng(d.get('impedanceAtMinFrequency'), 'Ω')}).",
        result)


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
    return _result(
        f"Proposed LC: L = {_eng(result.get('inductance'), 'H')}, "
        f"C = {_eng(result.get('capacitance'), 'F')}, "
        f"cutoff {_eng(result.get('cutoffFrequency'), 'Hz')}, target attenuation "
        f"{result.get('targetAttenuation_dB')} dB. Re-run design_dmc with this "
        f"inductance as minimumInductance.",
        result)


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
    payload = {"frequencies": rows, "passed": passed, "total": len(rows)}

    def curve(key):
        return [[float(r["frequency"]), float(r[key])] for r in rows
                if isinstance(r.get("frequency"), (int, float)) and r["frequency"] > 0
                and isinstance(r.get(key), (int, float))]

    series = []
    for key, name, kind in (("requiredAttenuation", "required", "required"),
                            ("measuredAttenuation", "measured (simulated)", "line"),
                            ("theoreticalAttenuation", "theoretical", "line")):
        pts = curve(key)
        if len(pts) > 1:
            series.append({"name": name, "points": pts, "kind": kind})
    if not series:
        return _result(summary, payload)
    # The verdict belongs ON the curve, at the frequency where it was decided.
    markers = [{"f_hz": float(r["frequency"]),
                "value": float(r.get("measuredAttenuation") if isinstance(
                    r.get("measuredAttenuation"), (int, float)) else r["requiredAttenuation"]),
                "ok": bool(r.get("passed"))}
               for r in rows if isinstance(r.get("frequency"), (int, float)) and r["frequency"] > 0]
    return _bode_result(
        "DM filter attenuation", f"{passed}/{len(rows)} frequencies meet the requirement",
        series, summary, y_label="attenuation (dB)", markers=markers, data=payload)


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
    return _result(
        f"Current transformer sized: MAS Inputs with {windings} winding(s) and "
        f"{len(ops)} operating point(s), ready for the magnetic adviser.",
        result)


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
