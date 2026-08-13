"""Smoke test for the Kirchhoff MCP server — the widget-bearing tools, against the real engine.

Not a unit test: it designs a real converter, simulates it, sources real parts, and asserts
that every tool advertising a widget actually returns something that widget can draw. A tool
that advertises a UI and returns nothing for it renders as a broken panel and nothing
server-side complains — that is ABT #651's failure mode, and this is what catches it here.

    KIRCHHOFF_BUILD=… KELVIN_TAS_DATA_DIR=… python3 mcp/smoke.py [--skip-sourcing]
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

import server as S                                            # noqa: E402

SKIP_SOURCING = "--skip-sourcing" in sys.argv
FAILURES: list[str] = []

SPEC = {
    "designRequirements": {
        "efficiency": 1.0,
        "inputVoltage": {"minimum": 45.6, "nominal": 48, "maximum": 50.4},
        "switchingFrequency": {"nominal": 100_000},
        "outputs": [{"name": "out", "voltage": {"nominal": 12}}],
    },
    "operatingPoints": [{"inputVoltage": 48, "outputs": [{"power": 24}]}],
}


def check(label: str, condition: bool, detail: str = "") -> None:
    print(f"  {'ok  ' if condition else 'FAIL'}  {label}" + (f" — {detail}" if detail else ""))
    if not condition:
        FAILURES.append(label)


def text(result) -> str:
    return "\n".join(c.text for c in result.content)


def drawable(series: list) -> bool:
    """A series the curves widget can actually draw: >1 finite point, a V or A unit."""
    return bool(series) and all(
        s.get("unit") in ("V", "A") and len(s.get("points") or []) > 1 for s in series)


def main() -> int:
    print("the widget surface")
    tools = asyncio.run(S.mcp.list_tools())
    ui = {t.name: (t.meta or {}).get("ui/resourceUri") for t in tools
          if (t.meta or {}).get("ui/resourceUri")}
    check("every advertised widget has a bundle behind it",
          all(Path(S.__file__).parent.joinpath("dist", uri.rsplit("/", 1)[-1]).exists()
              for uri in ui.values()),
          ", ".join(sorted({u.rsplit('/', 1)[-1] for u in ui.values()})))
    check("the sourcing tools carry the picker",
          {n for n, u in ui.items() if u.endswith("picker.html")}
          == {"select_parts", "select_candidates", "cross_reference", "bind_part"},
          ", ".join(sorted(n for n, u in ui.items() if u.endswith("picker.html"))))
    check("the waveform tools carry the curves widget",
          {n for n, u in ui.items() if u.endswith("curves.html")}
          == {"component_waveforms", "operating_point", "topology_waveforms"},
          ", ".join(sorted(n for n, u in ui.items() if u.endswith("curves.html"))))

    print("design_converter(flyback)")
    r = S.design_converter("flyback", SPEC)
    tas = r.structuredContent["tas"]
    check("a TAS came back", bool(tas.get("topology")), "flyback")

    print("operating_point  [curves]")
    r = S.operating_point(tas)
    sc = r.structuredContent
    check("the operating point is still returned, not replaced by a chart",
          bool(sc.get("operating_point", {}).get("excitationsPerWinding")))
    check("its excitations are drawable by the curves widget", drawable(sc.get("series") or []),
          f"{len(sc.get('series') or [])} traces, "
          f"{(sc.get('series') or [{}])[0].get('name')}")
    check("the digest says how many were charted", "charted waveform" in text(r))

    print("topology_waveforms  [curves]")
    r = S.topology_waveforms(tas)
    sc = r.structuredContent
    check("every magnetic is still returned", sc.get("count", 0) >= 1,
          ", ".join(str(n) for n in sc.get("names") or []))
    check("its excitations are drawable", drawable(sc.get("series") or []),
          f"{len(sc.get('series') or [])} traces")
    check("traces are labelled by magnetic, so they can be told apart",
          all(any(str(n) in s["name"] for n in sc["names"]) for s in sc["series"]))

    print("component_waveforms  [curves, pre-existing]")
    r = S.component_waveforms(tas)
    check("component traces are drawable", drawable(r.structuredContent.get("series") or []),
          f"{len(r.structuredContent.get('series') or [])} traces")

    print("simulate — deliberately NOT charted")
    r = S.simulate(tas)
    sc = r.structuredContent
    check("it returns per-vector statistics", bool(sc.get("measurements")),
          f"{len(sc.get('measurements') or [])} vectors")
    check("it carries no series, and no widget claims it can draw one",
          "series" not in sc and "simulate" not in
          {t.name for t in tools if (t.meta or {}).get("ui/resourceUri")})

    if SKIP_SOURCING:
        print("sourcing: SKIPPED (--skip-sourcing)")
    else:
        print("select_parts  [picker]")
        r = S.select_parts(tas)
        sc = r.structuredContent
        components = sc["components"]
        check("the design was sourced", sc["filled"] > 0, f"{sc['filled']}/{sc['total']} filled")
        filled = [c for c in components if c.get("filled")]
        check("every filled component offers a ranked list to choose from",
              all((c.get("selection") or {}).get("candidates") for c in filled),
              f"{sum(len(c['selection']['candidates']) for c in filled)} candidates over "
              f"{len(filled)} components")
        check("each candidate carries something the picker can rank on",
              all(any(c["selection"]["candidates"][0].get(k)
                      for k in ("margins", "sortKey", "deviation")) for c in filled))
        # "nobody looked" and "nothing fit" are different answers and must not print alike:
        # a controller is DEFERRED until the topology is known, not rejected by a gate.
        unfilled = [c for c in components if not c.get("filled")]
        check("every unfilled component says which kind of unfilled it is",
              all(c.get("deferred") or c.get("error") for c in unfilled),
              ", ".join(f"{c['ref']}={'deferred' if c.get('deferred') else 'rejected'}"
                        for c in unfilled))
        check("a deferred component is not reported as 'no candidate met the requirements'",
              not any(c.get("deferred") for c in unfilled)
              or "deferred" in text(r))

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED: " + "; ".join(FAILURES))
        return 1
    print("all smoke checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
