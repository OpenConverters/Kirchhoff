"""Smoke/integration test for scripts/regulate.py — closed-loop regulated operating point.

Exercises all three control modalities (duty / phase-shift / frequency) end to end: design a real-fidelity
(DATASHEET) deck, run the regulation search, and assert it drives Vout onto target with a plausible
efficiency. Slower than the C++ suite (each call runs ~20 ngspice sims), so it lives on its own.

Run:  python3 -m pytest tests/test_regulate.py -v     (needs the built PyKirchhoff in ./build)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import PyKirchhoff  # noqa: E402
import regulate as R  # noqa: E402


def _spec(vin, vout, p, fsw):
    return {
        "designRequirements": {"efficiency": 1.0,
                               "inputVoltage": {"minimum": vin * 0.95, "nominal": vin, "maximum": vin * 1.05},
                               "switchingFrequency": {"nominal": fsw},
                               "outputs": [{"name": "out", "voltage": {"nominal": vout}}]},
        "operatingPoints": [{"inputVoltage": vin, "outputs": [{"power": p}]}],
    }


def _check(design_fn, topo, vin, vout, p, fsw):
    tas = design_fn(_spec(vin, vout, p, fsw))
    R.bind_datasheet_semis(tas)
    r = R.simulate_regulated(tas, vout, topo, fidelity={"origin": "DATASHEET"})
    assert r["regulated"], f"{topo} did not regulate to target (vout={r.get('vout')}, target={vout})"
    assert abs(r["vout"] - vout) <= 0.02 * vout, f"{topo} Vout {r['vout']:.3f} off target {vout}"
    assert 0.5 < r["efficiency"] <= 1.0, f"{topo} efficiency {r['efficiency']:.3f} implausible"
    return r


def test_regulate_duty_buck():
    _check(PyKirchhoff.design_buck_tas, "buck", 12, 5, 50, 400000)


def test_regulate_duty_boost():       # ABT #28's example topology
    _check(PyKirchhoff.design_boost_tas, "boost", 12, 24, 108, 250000)


def test_regulate_phase_psfb():
    _check(PyKirchhoff.design_psfb_tas, "psfb", 400, 12, 600, 100000)


def test_regulate_frequency_llc():
    _check(PyKirchhoff.design_llc_tas, "llc", 400, 48, 960, 100000)
