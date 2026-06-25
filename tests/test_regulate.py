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
    assert r["steady_state"], f"{topo} regulated point is not steady-state (drifting/oscillating)"
    assert abs(r["vout"] - vout) <= 0.02 * vout, f"{topo} Vout {r['vout']:.3f} off target {vout}"
    assert 0.5 < r["efficiency"] <= 1.0, f"{topo} efficiency {r['efficiency']:.3f} implausible"
    return r


# --- Independent analytical conduction-loss efficiency, to VALIDATE the simulated number (not just bound it).
# The bound test parts are Ron=0.02 and a DIDEAL-style diode Vf=0.8 V @ 1 A (Vf(I)=0.8+Vt*ln(I)). The estimate
# ignores switching/body-diode/DCR loss, so it agrees with the sim to a few %, but it is dead-wrong (off by
# tens of %) if the tool mis-measures power (e.g. drops a rail) — which is exactly what we want to catch.
import math
_RON, _VT = 0.02, 0.025852


def _vf(i):
    return 0.8 + _VT * math.log(max(i, 1e-9))


def _eta_buck(vin, vout, pout):
    iout, d = pout / vout, vout / vin
    losses = iout ** 2 * _RON * d + _vf(iout) * iout * (1 - d)   # switch conduction + freewheel-diode drop
    return pout / (pout + losses)


def _eta_boost(vin, vout, pout):
    iout, d = pout / vout, 1 - vin / vout
    il = iout / (1 - d)                                          # inductor/input current
    losses = il ** 2 * _RON * d + _vf(iout) * iout               # switch conduction + output-diode drop
    return pout / (pout + losses)


def test_regulate_duty_buck():
    r = _check(PyKirchhoff.design_buck_tas, "buck", 12, 5, 50, 400000)
    eta = _eta_buck(12, 5, 50)
    assert abs(r["efficiency"] - eta) <= 0.04, f"buck eff sim {r['efficiency']:.3f} vs analytical {eta:.3f}"


def test_regulate_duty_boost():       # ABT #28's example topology
    r = _check(PyKirchhoff.design_boost_tas, "boost", 12, 24, 108, 250000)
    eta = _eta_boost(12, 24, 108)
    assert abs(r["efficiency"] - eta) <= 0.04, f"boost eff sim {r['efficiency']:.3f} vs analytical {eta:.3f}"


def test_regulate_phase_psfb():
    _check(PyKirchhoff.design_psfb_tas, "psfb", 400, 12, 600, 100000)


def test_regulate_frequency_llc():
    _check(PyKirchhoff.design_llc_tas, "llc", 400, 48, 960, 100000)


def test_regulate_dual_output_isolated_buck():
    # Dual-output flybuck: the secondary load lives INSIDE the stage subckt, so the efficiency must reach it
    # via the hierarchical node. A correct full-converter efficiency (both rails) is plausible (>60%); the
    # old primary-only measurement read a broken ~25%.
    ib = {"designRequirements": {"efficiency": 1.0,
                                 "inputVoltage": {"minimum": 45, "nominal": 48, "maximum": 51},
                                 "switchingFrequency": {"nominal": 200000},
                                 "outputs": [{"name": "out", "voltage": {"nominal": 5}},
                                             {"name": "vsec", "voltage": {"nominal": 12}}]},
          "operatingPoints": [{"inputVoltage": 48, "outputs": [{"power": 2.5}, {"power": 5}]}]}
    tas = PyKirchhoff.design_isolated_buck_tas(ib)
    R.bind_datasheet_semis(tas)
    r = R.simulate_regulated(tas, 5.0, "isolated_buck", fidelity={"origin": "DATASHEET"})
    assert r["regulated"], f"isolated_buck primary did not regulate (vout={r.get('vout')})"
    assert r["steady_state"], "isolated_buck regulated point is not steady-state"
    assert 0.6 < r["efficiency"] <= 1.0, f"isolated_buck full-converter efficiency {r['efficiency']:.3f} " \
                                         "implausible (secondary rail not counted?)"


def _stamp_subckt(tas, text, ref):
    for st in tas["topology"]["stages"]:
        c = st.get("circuit")
        if isinstance(c, dict):
            for comp in c["components"]:
                d = comp.get("data", {})
                if "magnetic" in d:
                    d["magnetic"]["modelOutputs"] = {"spiceSubcircuit": {"reference": ref, "text": text}}
    return tas


_BOOST_SPEC = {"designRequirements": {"efficiency": 0.9,
                                      "inputVoltage": {"minimum": 11.4, "nominal": 12, "maximum": 12.6},
                                      "switchingFrequency": {"nominal": 100000},
                                      "outputs": [{"name": "out", "voltage": {"nominal": 24}}]},
               "operatingPoints": [{"inputVoltage": 12, "outputs": [{"power": 24}]}]}


def test_magnetic_saturation_verdict():
    # An MKF_MODEL core whose Isat (0.41 A) is far below the boost's peak inductor current (~2.5 A) saturates;
    # Kirchhoff must report an explicit 'magnetic saturated' verdict rather than grind the bisect through
    # diverging decks and return an opaque converged=False (HS ABT #33). Short-circuits, so no ngspice run.
    sat_sub = (".subckt MKF_SAT P1+ P1-\n"
               "Rdc_1 P1+ n1 0.028\n"
               "Vmag_sense_1 n1 Lmag_flux_1 0\n"
               "BLmag_1 Lmag_flux_1 P1- V='Lmag_1_L0/(1+pow(abs(I(Vmag_sense_1))/Lmag_1_Isat,2))*ddt(I(Vmag_sense_1))'\n"
               ".param Lmag_1_L0=160.3u\n.param Lmag_1_Isat=0.411617\n.ends")
    tas = _stamp_subckt(PyKirchhoff.design_boost_tas(_BOOST_SPEC), sat_sub, "MKF_SAT")
    f = R.saturation_findings(tas)
    assert f and f[0]["isat"] == 0.411617 and f[0]["operating_current"] > f[0]["isat"] and f[0]["ratio"] > 1.0, f
    r = R.simulate_regulated(tas, 24.0, "boost", fidelity={"origin": "MKF_MODEL"})
    assert r["converged"] is False and r["regulated"] is False
    assert r.get("saturated"), "result must carry the saturation findings"
    assert "saturated" in r.get("reason", "").lower()


def test_no_false_saturation_on_linear_core():
    # A linear Rdc+Lmag MKF subcircuit (no _Isat param) must NOT trip the saturation verdict.
    lin_sub = ".subckt MKF_LIN P1+ P1-\nRdc1 P1+ na 0.028\nLmag1 na P1- 160.3u\n.ends"
    tas = _stamp_subckt(PyKirchhoff.design_boost_tas(_BOOST_SPEC), lin_sub, "MKF_LIN")
    assert R.saturation_findings(tas) == [], "linear core wrongly flagged as saturated"


def test_saturation_skips_multiwinding():
    # A transformer's winding (load) currents largely cancel in the core, so a winding-current-vs-Isat test
    # would false-positive. Saturation detection is restricted to single-winding inductors (where the winding
    # current IS the magnetizing current). A 2-winding magnetic — even with winding currents far above Isat —
    # must NOT be flagged; transformer saturation (magnetizing-current based) is a documented follow-up.
    sub = ".subckt T P1 P2 S1 S2\n.param Lmag_1_Isat=0.1\n.ends"

    def _exc(offset):
        return {"current": {"processed": {"offset": offset, "peak": offset * 1.1}}}

    tas = {"topology": {"stages": [{"circuit": {"components": [
        {"name": "T1", "data": {"magnetic": {"modelOutputs": {"spiceSubcircuit": {"text": sub}}},
                                "inputs": {"operatingPoints": [
                                    {"excitationsPerWinding": [_exc(2.0), _exc(5.0)]}]}}}]}}]}}
    assert R.saturation_findings(tas) == [], "multi-winding magnetic must not be (naively) flagged"


def test_saturation_uses_operating_not_peak_current():
    # The verdict is on the DC operating (offset) current, not the peak — so a deck whose peak nudges over Isat
    # but whose DC operating current is within Isat is NOT flagged (it usually still simulates).
    sub = ".subckt L P1 P2\n.param Lmag_1_Isat=1.0\n.ends"
    one = {"topology": {"stages": [{"circuit": {"components": [
        {"name": "L1", "data": {"magnetic": {"modelOutputs": {"spiceSubcircuit": {"text": sub}}},
                                "inputs": {"operatingPoints": [{"excitationsPerWinding": [
                                    {"current": {"processed": {"offset": 0.9, "peak": 1.4}}}]}]}}}]}}]}}
    assert R.saturation_findings(one) == [], "peak-only excursion must not trip the operating-current verdict"
