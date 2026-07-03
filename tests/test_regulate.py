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

    # ABT #97: the simulated Cout current must be the de-spiked PHYSICAL ripple (rms ~ the analytical output-cap
    # ripple), NOT the raw switching-DISPLACEMENT artifact (hundreds of A peak-to-peak on this ~4.8 A-rms cap).
    cout = r["component_excitations"]["Cout"]["excitation"]
    assert "current" in cout, "boost Cout emitted no simulated current (abt #97 de-spiking regressed?)"
    ic = cout["current"]["processed"]
    d = 1.0 - 12.0 / 24.0                                # boost duty at the regulated point (~0.5)
    iout = 108.0 / 24.0                                  # output current 4.5 A
    ana_rms = iout * math.sqrt(d / (1.0 - d))           # CCM output-cap ripple rms ~ 4.5 A (ignores L-ripple)
    assert 0.7 * ana_rms <= ic["rms"] <= 1.6 * ana_rms, \
        f"boost Cout current rms {ic['rms']:.3f} A implausible vs analytical {ana_rms:.3f} A"
    # A clean physical peak is a small multiple of the ripple rms; the displacement artifact was ~140x it.
    assert ic["peak"] <= 8.0 * ic["rms"] and ic["peak"] < 40.0, \
        f"boost Cout current peak {ic['peak']:.1f} A is a switching-displacement artifact, not the real ripple"


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


def _mag_tas(sub_text, *, offset, peak=None, datasheet_isat=None, core_coil=None):
    """Minimal single-winding-inductor TAS: one magnetic carrying the stamped subckt and (optionally) a
    datasheet Isat and/or a sourced core+coil, at a chosen DC operating (offset) current."""
    mag = dict(core_coil) if core_coil else {}
    mag["modelOutputs"] = {"spiceSubcircuit": {"text": sub_text}}
    if datasheet_isat is not None:
        mag["manufacturerInfo"] = {"datasheetInfo": {"electrical": [{"saturationCurrentPeak": datasheet_isat}]}}
    proc = {"offset": offset, "peak": offset * 1.1 if peak is None else peak}
    return {"topology": {"stages": [{"circuit": {"components": [
        {"name": "L1", "data": {"magnetic": mag,
            "inputs": {"operatingPoints": [{"conditions": {"ambientTemperature": 25.0},
                "excitationsPerWinding": [{"current": {"processed": proc}}]}]}}}]}}]}}


def test_saturation_via_datasheet_when_no_architecture():
    # No sourced core/coil -> the authoritative Isat is the DATASHEET rating (saturationCurrentPeak), NOT the
    # subcircuit's emitted value. operating 2.34 A > datasheet 0.41 A -> saturated (source='datasheet').
    sub = ".subckt M P1 P2\n.param Lmag_1_Isat=99\n.ends"   # subcircuit value deliberately bogus -> ignored
    tas = _mag_tas(sub, offset=2.34, datasheet_isat=0.41)
    f = R.saturation_findings(tas)
    assert len(f) == 1 and f[0]["kind"] == "saturated" and f[0]["isat_source"] == "datasheet" \
        and abs(f[0]["isat"] - 0.41) < 1e-9 and f[0]["ratio"] > 1.0, f
    r = R.simulate_regulated(tas, 24.0, "boost", fidelity={"origin": "MKF_MODEL"})
    assert r["converged"] is False and r.get("saturated") and "saturated" in r["reason"].lower()


def test_subcircuit_isat_is_never_the_authority():
    # A subcircuit with an _Isat but NO architecture AND NO datasheet -> the verdict cannot use the subcircuit
    # as a source of truth, so it makes NO claim (the ABT #33 lesson: never trust the exporter's emitted value).
    sub = ".subckt M P1 P2\n.param Lmag_1_Isat=0.1\n.ends"
    assert R.saturation_findings(_mag_tas(sub, offset=2.0)) == [], "subcircuit Isat must not be authoritative"


def test_no_false_saturation_on_linear_core():
    # A linear Rdc+Lmag subcircuit (no _Isat) on a requirements-only magnetic -> no Isat source -> no finding.
    lin_sub = ".subckt MKF_LIN P1+ P1-\nRdc1 P1+ na 0.028\nLmag1 na P1- 160.3u\n.ends"
    tas = _stamp_subckt(PyKirchhoff.design_boost_tas(_BOOST_SPEC), lin_sub, "MKF_LIN")
    assert R.saturation_findings(tas) == [], "linear core wrongly flagged as saturated"


def test_saturation_skips_multiwinding():
    # A transformer's winding (load) currents largely cancel in the core, so a winding-current-vs-Isat test
    # would false-positive. Restricted to single-winding inductors; a 2-winding magnetic is skipped even with a
    # datasheet Isat and winding currents far above it.
    sub = ".subckt T P1 P2 S1 S2\n.param Lmag_1_Isat=0.1\n.ends"

    def _exc(offset):
        return {"current": {"processed": {"offset": offset, "peak": offset * 1.1}}}

    tas = {"topology": {"stages": [{"circuit": {"components": [
        {"name": "T1", "data": {"magnetic": {
            "manufacturerInfo": {"datasheetInfo": {"electrical": [{"saturationCurrentPeak": 0.1}]}},
            "modelOutputs": {"spiceSubcircuit": {"text": sub}}},
            "inputs": {"operatingPoints": [{"excitationsPerWinding": [_exc(2.0), _exc(5.0)]}]}}}]}}]}}
    assert R.saturation_findings(tas) == [], "multi-winding magnetic must not be flagged"


def test_saturation_uses_operating_not_peak_current():
    # On the DC operating (offset) current, not the peak: peak 1.4 A > datasheet 1.0 A but operating 0.9 A <
    # 1.0 A -> NOT flagged.
    tas = _mag_tas(".subckt L P1 P2\n.ends", offset=0.9, peak=1.4, datasheet_isat=1.0)
    assert R.saturation_findings(tas) == [], "peak-only excursion must not trip the operating-current verdict"


def test_exporter_isat_mismatch_via_model():
    # The cross-check: with a REAL core, judge from calculate_saturation_current and CATCH a subcircuit whose
    # emitted Isat disagrees (the ABT #33 self-check). Needs PyOM.
    #
    # The magnetic seed comes from Kirchhoff's OWN magnetic-design path (design_<topo>_tas ->
    # main_magnetic_inputs -> PyOM.calculate_advised_magnetics_fast), NOT MKF's deleted
    # design_magnetics_from_converter (removed in the KH cutover; ABT #73/#98). KH designs the boost's main
    # inductor inputs; PyOM turns them into a real core+coil we can compute a model Isat for.
    import pytest
    pyom = pytest.importorskip("PyOpenMagnetics")
    boost_tas = PyKirchhoff.design_boost_tas(_BOOST_SPEC)
    inputs = pyom.process_inputs(PyKirchhoff.main_magnetic_inputs(boost_tas))
    mag = pyom.calculate_advised_magnetics_fast(inputs, 1, "available cores")["data"][0]["mas"]["magnetic"]
    isat = pyom.calculate_saturation_current(mag, 25.0)
    iop, bogus = 0.4 * isat, 0.1 * isat
    # subcircuit Isat ~10x low, operating below the REAL Isat -> exporter mismatch (the core is fine)
    f = R.saturation_findings(_mag_tas(f".subckt M P1 P2\n.param Lmag_1_Isat={bogus}\n.ends", offset=iop, core_coil=mag))
    assert len(f) == 1 and f[0]["kind"] == "exporter_isat_mismatch" and f[0]["isat_authoritative_source"] == "model", f
    assert abs(f[0]["isat_authoritative"] - isat) < 1e-3 and abs(f[0]["isat_subcircuit"] - bogus) < 1e-6
    # subcircuit agrees with the model -> no finding
    assert R.saturation_findings(_mag_tas(f".subckt M P1 P2\n.param Lmag_1_Isat={isat}\n.ends", offset=iop, core_coil=mag)) == []
    # genuine saturation (operating above the real Isat) -> kind saturated, source model
    g = R.saturation_findings(_mag_tas(f".subckt M P1 P2\n.param Lmag_1_Isat={isat}\n.ends", offset=isat * 1.5, core_coil=mag))
    assert len(g) == 1 and g[0]["kind"] == "saturated" and g[0]["isat_source"] == "model", g
