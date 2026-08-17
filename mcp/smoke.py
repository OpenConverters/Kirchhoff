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


def drawable(sc: dict) -> bool:
    """A `curves` payload the widget can actually draw.

    Every series must name an ordinate that the payload DECLARES. The old form of this check
    read a per-series unit string and accepted "V" or "A", which is the convention the
    contract's axes replaced: a trace whose axis is undeclared has no scale to be drawn on.
    """
    series = sc.get("series") or []
    axes = sc.get("axes") or {}
    return bool(series) and all(
        axes.get("y2" if s.get("axis") == "y2" else "y", {}).get("unit")
        and len(s.get("points") or []) > 1 for s in series)


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
    check("the frequency-domain tools carry the bode widget",
          {n for n, u in ui.items() if u.endswith("bode.html")} == {"simulate_ac", "verify_dmc"},
          ", ".join(sorted(n for n, u in ui.items() if u.endswith("bode.html"))))
    # component_waveforms is the ONLY charting tool now. operating_point and
    # topology_waveforms return the MAS documents they are for — a payload cannot be a
    # document and a chart at once, and the documents are what an adviser consumes.
    check("the waveform tools carry the curves widget",
          {n for n, u in ui.items() if u.endswith("curves.html")} == {"component_waveforms"},
          ", ".join(sorted(n for n, u in ui.items() if u.endswith("curves.html"))))

    print("design_converter(flyback)")
    r = S.design_converter("flyback", SPEC)
    sc = r.structuredContent
    tas = sc["document"]
    check("a TAS came back", bool(tas.get("topology")), "flyback")
    check("the document says which schema governs it and what it is of",
          sc["schema"]["name"] == "TAS" and sc["subject"] == "flyback"
          and sc["operation"] == "produced")
    check("the magnetic's operating point rides as a companion, keyed by the magnetic",
          all(c.get("subject") and c.get("schema", {}).get("name")
              for c in (sc.get("companions") or {}).values()),
          ", ".join(sc.get("companions") or {}))

    print("operating_point  [document]")
    r = S.operating_point(tas)
    sc = r.structuredContent
    check("the operating point is the answer, not a chart of it",
          sc["mode"] == "document"
          and bool(sc["document"].get("excitationsPerWinding")))
    check("it names the MAS schema and the magnetic it came from",
          sc["schema"]["name"] == "MAS" and bool(sc.get("derivedFrom")))

    print("topology_waveforms  [document + companions]")
    r = S.topology_waveforms(tas)
    sc = r.structuredContent
    check("the main magnetic is the document", bool(sc["document"].get("operatingPoints")))
    check("every other magnetic is a companion named after itself",
          all(name == c["subject"] for name, c in (sc.get("companions") or {}).items()),
          ", ".join(sc.get("companions") or {}) or "one magnetic in this design")

    print("component_waveforms  [curves]")
    r = S.component_waveforms(tas)
    sc = r.structuredContent
    check("component traces are drawable", drawable(sc), f"{len(sc.get('series') or [])} traces")
    # Volts and amps on one plot: the second ordinate is what keeps the currents off the
    # voltage scale, and every trace says which one it belongs to.
    check("volts and amps are on their own declared axes",
          sc["axes"]["y"]["unit"] == "V" and sc["axes"]["y2"]["unit"] == "A"
          and {s.get("axis") for s in sc["series"]} <= {"y", "y2"},
          f"{sum(1 for s in sc['series'] if s.get('axis') == 'y2')} of {len(sc['series'])} "
          f"traces on the current axis")

    print("simulate — deliberately NOT charted")
    r = S.simulate(tas)
    sc = r.structuredContent
    check("it returns per-vector statistics", bool(sc.get("statistics")),
          f"{len(sc.get('statistics') or {})} vectors")
    check("every statistic says what it is in and over what window",
          all(v.get("unit") and v.get("over", {}).get("axis") == "time"
              for v in sc["statistics"].values()))
    check("it says which model measured them", "libngspice" in sc["model"], sc["model"])
    check("it carries no series, and no widget claims it can draw one",
          "series" not in sc and "simulate" not in
          {t.name for t in tools if (t.meta or {}).get("ui/resourceUri")})

    print("simulate_ac  [bode] — an RC low-pass whose answer is known")
    deck = ("RC lowpass\nV1 in 0 AC 1\nR1 in out 1k\nC1 out 0 159n\n"
            ".ac dec 20 10 1meg\n.end\n")
    r = S.simulate_ac(deck)
    sc = r.structuredContent
    out = next((x for x in sc.get("series") or [] if x["name"] == "out"), None)
    check("the sweep is charted as magnitude against frequency", out is not None,
          ", ".join(x["name"] for x in sc.get("series") or []))
    if out:
        at = lambda f: min(out["points"], key=lambda pt: abs(pt[0] - f))[1]
        # R=1k, C=159n -> fc ~= 1 kHz. If the dB conversion were wrong this is where it shows.
        check("the plotted magnitudes are the circuit's, not a scaling of it",
              abs(at(10)) < 0.1 and abs(at(1000) + 3.01) < 0.3 and abs(at(1e6) + 60) < 1.5,
              f"{at(10):.2f} dB at 10 Hz, {at(1000):.2f} at the 1 kHz corner, "
              f"{at(1e6):.2f} at 1 MHz")
    # Phase is the other half of a Bode answer — a magnitude-only payload cannot say what the
    # phase margin is, and degrees on a decibel axis would mislabel every point of it.
    phase = [x for x in sc.get("series") or [] if x.get("axis") == "y2"]
    check("phase is carried too, on its own ordinate",
          bool(phase) and sc["axes"]["y2"]["unit"] == "deg",
          f"{len(phase)} phase trace(s)")
    # The OUTPUT node's phase, not whichever trace came first — `in` is the source and is
    # flat at 0 degrees by definition, so checking it would pass against any phase code at all.
    out_phase = next((x for x in phase if x["name"].startswith("out")), None)
    if out_phase and out:
        at_phase = lambda f: min(out_phase["points"], key=lambda pt: abs(pt[0] - f))[1]
        # An RC low-pass is -45 deg at its corner and approaches -90 well above it.
        check("the plotted phase is the circuit's",
              abs(at_phase(1000) + 45) < 5 and abs(at_phase(1e6) + 90) < 2,
              f"{at_phase(1000):.1f} deg at the corner, {at_phase(1e6):.1f} deg at 1 MHz")

    print("verify_dmc  [bode]")
    r = S.verify_dmc({"inputVoltage": 230, "operatingCurrent": 2.0, "lineFrequency": 50,
                      "switchingFrequency": 100_000, "ambientTemperature": 25,
                      "minimumInductance": 1e-3}, 1e-3)
    sc = r.structuredContent
    names = [x["name"] for x in sc.get("series") or []]
    check("required, measured and theoretical are all drawn", len(names) >= 2, ", ".join(names))
    check("the requirement is marked as a limit, not another measurement",
          any(x.get("kind") == "limit" for x in sc.get("series") or []))
    check("it is a verdict against a named criterion, and says it is provisional",
          sc["mode"] == "verdict" and sc["verdict"] in ("pass", "warn", "fail", "unverified")
          and sc["criterion"] and sc["provisional"] is True,
          f"{sc['verdict']} — {sc['criterion']}")
    check("the tally reconciles with the breaches",
          sc["measurements"]["frequenciesPassed"]["value"]
          + len(sc.get("exceedances") or [])
          + sc["measurements"]["frequenciesUnmeasured"]["value"]
          == sc["measurements"]["frequenciesTested"]["value"],
          f"{sc['measurements']['frequenciesPassed']['value']}"
          f"/{sc['measurements']['frequenciesTested']['value']} pass, "
          f"{len(sc.get('exceedances') or [])} breach")
    check("every breach says how far it missed by",
          all("margin" in e and "limit" in e for e in sc.get("exceedances") or []))

    if SKIP_SOURCING:
        print("sourcing: SKIPPED (--skip-sourcing)")
    else:
        print("select_parts  [picker]")
        r = S.select_parts(tas)
        sc = r.structuredContent
        lines = sc["lines"]
        check("the design was sourced as a BOM", sc["mode"] == "bom" and sc["sourced"] > 0,
              f"{sc['sourced']}/{sc['total']} lines sourced")
        check("every line keeps its own reference designator",
              all(l.get("ref") for l in lines), ", ".join(l["ref"] for l in lines))
        filled = [l for l in lines if l["status"] == "recommended"]
        check("every sourced line offers the ranked list it was chosen from",
              all(l.get("candidates") for l in filled),
              f"{sum(len(l['candidates']) for l in filled)} candidates over "
              f"{len(filled)} lines")
        check("each candidate carries something the picker can rank on",
              all(any(l["candidates"][0].get(k) for k in ("margins", "sortKey", "specs"))
                  for l in filled))
        # "nobody looked" and "nothing fit" are different answers and must not read alike: a
        # controller is DEFERRED until the topology is known, not rejected by a gate.
        unsourced = [l for l in lines if l["status"] != "recommended"]
        check("every unsourced line says WHICH kind of unsourced it is",
              all(l["status"] in ("unsourced", "no_substitute") for l in unsourced),
              ", ".join(f"{l['ref']}={l['status']}" for l in unsourced))
        check("a deferred line is not reported as 'no candidate met the requirements'",
              not any(l["status"] == "unsourced" for l in unsourced) or "deferred" in text(r))
        check("what could not be done is said in the payload, not only the digest",
              not unsourced or bool(sc.get("diagnostics")),
              "; ".join(sc.get("diagnostics") or [])[:80])

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED: " + "; ".join(FAILURES))
        return 1
    print("all smoke checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
