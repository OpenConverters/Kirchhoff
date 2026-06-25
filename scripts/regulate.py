#!/usr/bin/env python3
"""Closed-loop REGULATED operating point for a Kirchhoff design.

An open-loop fixed-duty deck does not regulate: at real (DATASHEET / MKF_MODEL) fidelity the design
duty/phase/frequency is wrong for the lossy operating point, so Vout drifts off target and the measured
efficiency is an open-loop artifact (high circulating current). This wraps the deck in the SAME closed-loop
search HS's MKF path uses: bisect the converter's one control variable (duty for the buck family, phase-shift
for psfb/pshb/dab, frequency for the resonant family) re-simulating until Vout hits target, then report the
REGULATED operating point + efficiency.

Topology-agnostic and ADDITIVE: it never touches the open-loop decks (so the MKF-equivalence pinning stays
valid) — it only sweeps the existing `simulation.stimulus` control field.

    from regulate import simulate_regulated
    spec = {...}                                            # same spec you pass to design_<topo>_tas
    tas  = PyKirchhoff.design_boost_tas(spec)               # (optionally bind DATASHEET/MKF_MODEL parts into tas)
    r = simulate_regulated(tas, target_vout=24.0, topology="boost", fidelity={"origin": "DATASHEET"})
    # -> {"converged": True, "vout": 24.0, "control": "duty", "value": 0.521, "efficiency": 0.83, "pin":..,"pout":..}
"""
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build"))
import PyKirchhoff  # noqa: E402

# control variable per topology: (stimulus waveform field, search bracket). The Vout-vs-control direction is
# auto-detected (sampling the bracket ends), so we never hardcode a possibly-wrong monotonicity.
_CONTROL = {
    "duty":      ("dutyCycle", (0.04, 0.96)),   # buck family
    "phase":     ("phaseDeg",  (15.0, 170.0)),  # psfb/pshb/dab: phase-shift of the lagging leg, ~0..180 deg
    "frequency": ("frequency", None),           # resonant: bracket derived from the design fsw
}
_TOPO_CONTROL = {
    **{t: "duty" for t in ("boost", "buck", "flyback", "forward", "two_switch_forward", "sepic", "cuk",
                           "zeta", "ahb", "acf", "fsbb", "weinberg", "isolated_buck", "isolated_buck_boost",
                           "push_pull")},
    **{t: "phase" for t in ("psfb", "pshb", "dab")},
    **{t: "frequency" for t in ("llc", "src", "cllc", "clllc")},
}


# A schema-valid DATASHEET MOSFET/diode, for binding a seed TAS into a real-fidelity deck. HS fills its own
# sourced parts; this mirrors that so the tool is self-testable (and usable as a quick default).
_REAL_MOSFET = {"semiconductor": {"mosfet": {"manufacturerInfo": {"name": "T", "datasheetInfo": {
    "part": {"partNumber": "TESTFET", "technology": "Si"},
    "electrical": {"drainSourceVoltage": 650, "onResistance": 0.02, "continuousDrainCurrent": 30,
                   "gateThresholdVoltage": {"nominal": 3.0}, "totalGateCharge": 1.5e-7,
                   "outputCapacitance": 1e-9, "bodyDiodeForwardVoltage": 0.9}}}}}}
_REAL_DIODE = {"semiconductor": {"diode": {"manufacturerInfo": {"name": "T", "datasheetInfo": {
    "part": {"partNumber": "TESTDIODE", "technology": "SiC"},
    "electrical": {"reverseVoltage": 650, "forwardVoltage": 0.8, "forwardCurrent": 10,
                   "junctionCapacitance": 1e-9}}}}}}


def bind_datasheet_semis(tas):
    """Replace every seed semiconductor in the TAS with a DATASHEET part (in place). Returns (nfet, ndio)."""
    nfet = ndio = 0
    for st in tas.get("topology", {}).get("stages", []):
        circ = st.get("circuit")
        if not isinstance(circ, dict):
            continue
        for c in circ.get("components", []):
            d = c.get("data")
            if isinstance(d, dict) and "semiconductor" in d:
                if "mosfet" in d["semiconductor"]:
                    c["data"] = dict(_REAL_MOSFET); nfet += 1
                elif "diode" in d["semiconductor"]:
                    c["data"] = dict(_REAL_DIODE); ndio += 1
    return nfet, ndio


def _design_fsw(tas):
    for st in tas.get("simulation", {}).get("stimulus", []):
        wf = st.get("waveform", {})
        if wf.get("type") == "pwm" and "frequency" in wf:
            return float(wf["frequency"])
    return 100000.0


def _set_control(tas, field, value):
    """Set the control field across the pwm stimuli.

    duty / frequency: apply to every pwm leg uniformly.
    phaseDeg: a phase-shifted bridge has a FIXED reference leg (phases 0 / 180) and a LAGGING leg whose two
        switches sit at φ and φ+180. We hold the reference leg and move only the lagging leg, so `value` is the
        bridge phase-shift φ — not a blanket overwrite (which would collapse the bridge)."""
    for st in tas.get("simulation", {}).get("stimulus", []):
        wf = st.get("waveform", {})
        if wf.get("type") != "pwm":
            continue
        if field == "phaseDeg":
            p = float(wf.get("phaseDeg", 0.0))
            if p in (0.0, 180.0):
                continue                                   # reference leg — fixed
            wf["phaseDeg"] = value if p < 180.0 else (value + 180.0)   # lagging leg: φ and φ+180
        else:
            wf[field] = value


def _parse_source_and_load(deck):
    vin = vin_name = rload = vout_cap = None
    for ln in deck.splitlines():
        m = re.match(r"^(V\w+)\s+\S+\s+\S+\s+DC\s+([-\d.eE+]+)", ln)
        if m and vin is None:                       # the input DC source (gate drives are PULSE, not DC)
            vin_name, vin = m.group(1), float(m.group(2))
        m = re.match(r"^Rload\s+(\S+)\s+\S+\s+([-\d.eE+]+)", ln)
        if m:
            rload = float(m.group(2))
    return vin_name, vin, rload


def _simulate(tas, fidelity, tag):
    """Render + run the deck; return (vout, iin) averaged over the last period, or None on non-convergence.
    Reads fsw from the (possibly just-modified) stimulus, so frequency-controlled topologies settle correctly."""
    deck = PyKirchhoff.tas_to_ngspice(tas, fidelity)
    vin_name, vin, rload = _parse_source_and_load(deck)
    fsw = _design_fsw(tas)
    period = 1.0 / fsw
    settle = max(600.0 * period, 4e-3)              # generous steady-state settle (regulation is steady-state)
    tstep = period / 200.0
    deck = re.sub(r"\.tran\s+\S+\s+\S+\s+\S+\s+\S+",
                  f".tran {tstep:.12g} {settle:.12g} 0 {tstep:.12g}", deck)
    cpos = deck.rfind("\n.control")
    if cpos != -1:
        deck = deck[:cpos]
    frm, to = settle - period, settle
    deck += (f"\n.control\nrun\n"
             f"meas tran vout avg v(Vout) from={frm:.12g} to={to:.12g}\n"
             f"meas tran iin avg i({vin_name}) from={frm:.12g} to={to:.12g}\n"
             f"print vout iin\n.endc\n.end\n")
    with tempfile.NamedTemporaryFile("w", suffix=f"_{tag}.cir", delete=False) as f:
        f.write(deck)
        path = f.name
    try:
        out = subprocess.run(["ngspice", "-b", path], capture_output=True, text=True, timeout=300)
        text = out.stdout + out.stderr
    finally:
        os.unlink(path)
    if "Timestep too small" in text or "simulation(s) aborted" in text:
        return None
    def grab(name):
        m = re.search(rf"{name}\s*=\s*([-\d.eE+]+)", text)
        return float(m.group(1)) if m else None
    vout, iin = grab("vout"), grab("iin")
    if vout is None or iin is None:
        return None
    return vout, iin, vin, rload


def simulate_regulated(tas, target_vout, topology, fidelity=None, tol=0.003, max_iter=22):
    """Bisect the topology's control variable until the simulated Vout hits target_vout; return the regulated
    operating point + efficiency. Returns {"converged": False, ...} if the deck never converged."""
    if fidelity is None:
        fidelity = {"origin": "DATASHEET"}
    ctrl = _TOPO_CONTROL.get(topology)
    if ctrl is None:
        raise ValueError(f"no control-variable mapping for topology '{topology}'")
    field, bracket = _CONTROL[ctrl]
    fsw = _design_fsw(tas)
    lo, hi = bracket if bracket is not None else (0.35 * fsw, 3.0 * fsw)

    def at(x):
        _set_control(tas, field, x)
        return _simulate(tas, fidelity, f"{topology}_{field}")

    # Auto-detect monotonicity from the bracket ends (no hardcoded direction). Both must converge to orient.
    rlo, rhi = at(lo), at(hi)
    if rlo is None or rhi is None:
        # one end diverged — nudge the bracket inward once and retry the ends
        lo, hi = lo + 0.15 * (hi - lo), hi - 0.15 * (hi - lo)
        rlo, rhi = at(lo), at(hi)
    increasing = True
    if rlo is not None and rhi is not None:
        increasing = rhi[0] > rlo[0]

    best = None
    for i in range(max_iter):
        mid = 0.5 * (lo + hi)
        r = at(mid)
        if r is None:                               # non-convergent point: shrink the bracket toward the middle
            hi = mid - 0.02 * (hi - lo)
            continue
        vout, iin, vin, rload = r
        if best is None or abs(vout - target_vout) < abs(best[1] - target_vout):
            best = (mid, vout, iin, vin, rload)
        if abs(vout - target_vout) <= tol * target_vout:
            break
        if (vout < target_vout) == increasing:
            lo = mid
        else:
            hi = mid

    if best is None:
        return {"converged": False, "control": ctrl, "topology": topology}
    value, vout, iin, vin, rload = best
    pin = abs(vin * iin)
    pout = vout * vout / rload if rload else float("nan")
    return {
        "converged": True,
        "topology": topology,
        "control": ctrl,
        "value": value,                 # the regulated duty / phaseDeg / frequency
        "vout": vout,
        "target_vout": target_vout,
        "pin": pin,
        "pout": pout,
        "efficiency": (pout / pin) if pin else float("nan"),
        "iin": iin,
    }


if __name__ == "__main__":
    # demo / smoke: a real-fidelity boost — open-loop (fixed design duty) drifts off target with a misleading
    # efficiency; the regulated search hits the Vout target and reports the true operating-point efficiency.
    import copy
    spec = {
        "designRequirements": {"efficiency": 1.0,
                               "inputVoltage": {"minimum": 11.4, "nominal": 12, "maximum": 12.6},
                               "switchingFrequency": {"nominal": 250000},
                               "outputs": [{"name": "out", "voltage": {"nominal": 24}}]},
        "operatingPoints": [{"inputVoltage": 12, "outputs": [{"power": 108}]}],
    }
    tas = PyKirchhoff.design_boost_tas(spec)
    bind_datasheet_semis(tas)
    fid = {"origin": "DATASHEET"}
    fsw = _design_fsw(tas)
    d_des = tas["simulation"]["stimulus"][0]["waveform"]["dutyCycle"]
    ol = _simulate(copy.deepcopy(tas), fid, "boost_ol")
    if ol:
        vout, iin, vin, rload = ol
        print(f"open-loop  (duty={d_des:.4f}): Vout={vout:.3f} V  eff={vout*vout/rload/abs(vin*iin)*100:.1f}%")
    r = simulate_regulated(copy.deepcopy(tas), target_vout=24.0, topology="boost", fidelity=fid)
    print(f"regulated  (duty={r['value']:.4f}): Vout={r['vout']:.3f} V  eff={r['efficiency']*100:.1f}%  "
          f"(target {r['target_vout']})")
