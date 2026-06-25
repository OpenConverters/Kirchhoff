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

Two flags, kept DISTINCT: "converged" = at least one operating point simulated without diverging; "regulated"
= Vout actually reached target within tol. A realism gate should require `regulated` — `converged` alone can
be an off-target point. Known limits (both report regulated=False / a caveat, never a wrong number silently):
  * Some real resonant/high-power decks can't reach target because the needed operating region DIVERGES in
    ngspice (e.g. an SRC whose >gain frequencies all diverge, or a 400->48 V DAB) -> regulated=False at the
    closest reachable point.
  * Dual-output topologies (isolated_buck / isolated_buck_boost): the generated deck loads only the PRIMARY
    rail, so the reported efficiency is primary-rail-only (not the full converter). Regulation of the primary
    Vout is still valid. Loading the secondary is a deck-generation fix (TasAssembler), out of this tool's scope.

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


def _parse_deck(deck):
    """Pull what the operating-point measurement needs: input DC source, the output load (kind + value), and
    the largest bulk capacitance in the deck (for an RC-based steady-state settle). We use the LARGEST cap
    rather than a node-name match because the inter-stage flattening renames the output cap's nodes; the bulk
    output cap dominates the tiny device Coss, and over-estimating the settle is safe (under-estimating is the
    bug that measures a transient)."""
    vin = vin_name = None
    load_kind = load_val = None          # ("R", ohms) | ("I", amps) | ("P", watts)
    cmax = 0.0
    for ln in deck.splitlines():
        m = re.match(r"^(V\w+)\s+\S+\s+\S+\s+DC\s+([-\d.eE+]+)", ln)
        if m and vin is None:            # the input DC source (gate drives are PULSE, not DC)
            vin_name, vin = m.group(1), float(m.group(2))
        m = re.match(r"^Rload\s+(\S+)\s+(\S+)\s+([-\d.eE+]+)", ln)
        if m:
            load_kind, load_val = "R", float(m.group(3))
        m = re.match(r"^Iload\s+\S+\s+\S+\s+DC\s+([-\d.eE+]+)", ln)
        if m:
            load_kind, load_val = "I", abs(float(m.group(1)))
        m = re.match(r"^Bload\b.*?=\s*([-\d.eE+]+)\s*/", ln)   # constant-power: I = P / max(V,Vk)
        if m:
            load_kind, load_val = "P", float(m.group(1))
        m = re.match(r"^C\w+\s+\S+\s+\S+\s+([-\d.eE+]+)", ln)
        if m:
            cmax = max(cmax, float(m.group(1)))
    return vin_name, vin, load_kind, load_val, cmax


def _simulate(tas, fidelity, tag):
    """Render + run the deck; return (vout_avg, pin, pout) measured at steady state, or None on
    non-convergence. fsw is read from the (possibly just-modified) stimulus so frequency control settles
    correctly; the settle is RC-based (max of 400 periods and 10 output-RC time constants), matching the C++
    harness, so a large-Cout / high-R output is actually settled — not measured mid-transient."""
    deck = PyKirchhoff.tas_to_ngspice(tas, fidelity)
    vin_name, vin, load_kind, load_val, cmax = _parse_deck(deck)
    fsw = _design_fsw(tas)
    period = 1.0 / fsw
    rc = (load_val if load_kind == "R" else 1.0) * cmax    # output-RC time constant (largest bulk cap)
    settle = max(400.0 * period, 10.0 * rc)
    tstep = period / 200.0
    deck = re.sub(r"\.tran\s+\S+\s+\S+\s+\S+\s+\S+",
                  f".tran {tstep:.12g} {settle:.12g} 0 {tstep:.12g}", deck)
    cpos = deck.rfind("\n.control")
    if cpos != -1:
        deck = deck[:cpos]
    # Instantaneous output power as a behavioural source, then meas its AVERAGE — this is the true average load
    # power (avg of v*i_load), correct under ripple, and load-agnostic. (ngspice `meas rms` is unreliable here.)
    if load_kind == "R":
        pexpr = f"v(Vout)*v(Vout)/{load_val:.10g}"
    elif load_kind == "I":
        pexpr = f"v(Vout)*{load_val:.10g}"
    elif load_kind == "P":
        pexpr = f"{load_val:.10g}"
    else:
        pexpr = None
    frm, to = settle - period, settle
    ctrl = ["run",
            f"meas tran vout avg v(Vout) from={frm:.12g} to={to:.12g}",
            f"meas tran iin avg i({vin_name}) from={frm:.12g} to={to:.12g}"]
    if pexpr is not None:
        deck += f"\nBpout n_pout_meas 0 V = {pexpr}\n"
        ctrl.append(f"meas tran pout avg v(n_pout_meas) from={frm:.12g} to={to:.12g}")
    deck += "\n.control\n" + "\n".join(ctrl) + "\nprint vout iin\n.endc\n.end\n"
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
        m = re.search(rf"\b{name}\s*=\s*([-\d.eE+]+)", text)
        return float(m.group(1)) if m else None
    vout, iin, pout_m = grab("vout"), grab("iin"), grab("pout")
    if vout is None or iin is None or vin is None:
        return None
    pin = abs(vin * iin)
    if pout_m is not None:
        pout = abs(pout_m)
    else:
        pout = float("nan")
    return vout, pin, pout


def simulate_regulated(tas, target_vout, topology, fidelity=None, tol=0.01, max_iter=24):
    """Bisect the topology's control variable until the simulated Vout hits target_vout, then report the
    regulated operating point + efficiency.

    Returns a dict. Two distinct flags — DON'T conflate them:
      "converged": at least one operating point simulated without diverging (a deck-health signal).
      "regulated": Vout actually reached target within `tol` (|Vout-target| <= tol*target). This is the one a
                   realism gate should require — a converged-but-off-target search is NOT a valid operating point.
    """
    if fidelity is None:
        fidelity = {"origin": "DATASHEET"}
    ctrl = _TOPO_CONTROL.get(topology)
    if ctrl is None:
        raise ValueError(f"no control-variable mapping for topology '{topology}'")
    field, bracket = _CONTROL[ctrl]
    fsw = _design_fsw(tas)
    # Frequency control: stay in the ABOVE-resonance region where Vout is MONOTONIC in frequency (below
    # resonance the gain turns over). The design fsw is at/above resonance, so bracket from just under it up.
    lo, hi = bracket if bracket is not None else (0.8 * fsw, 2.2 * fsw)

    def sample(x):
        """Evaluate the control value x; on non-convergence jitter a few % (some operating points in a real
        resonant deck diverge sporadically) to find a nearby valid point. Returns (x_used, vout, pin, pout)."""
        for dj in (0.0, 0.03, -0.03, 0.06, -0.06, 0.10):
            xj = x * (1.0 + dj)
            _set_control(tas, field, xj)
            r = _simulate(tas, fidelity, f"{topology}_{field}")
            if r is not None:
                return (xj,) + r
        return None

    # GRID + INTERPOLATION (robust to the resonant decks' sporadic divergence — it uses only the converging
    # samples, never assumes a fixed monotonicity direction, and skips holes in the gain curve). Seed a grid,
    # then repeatedly interpolate the control value that should hit target from the two converging samples
    # that straddle it.
    N = 9
    pts = [p for p in (sample(lo + (hi - lo) * k / (N - 1)) for k in range(N)) if p is not None]
    if not pts:
        return {"converged": False, "regulated": False, "control": ctrl, "topology": topology,
                "target_vout": target_vout}

    for _ in range(max_iter - N):
        pts.sort(key=lambda p: p[0])
        cur = min(pts, key=lambda p: abs(p[1] - target_vout))
        if abs(cur[1] - target_vout) <= tol * target_vout:
            break
        nxt = None                                          # interpolate within a straddling pair
        for a, b in zip(pts, pts[1:]):
            if (a[1] - target_vout) * (b[1] - target_vout) < 0 and a[1] != b[1]:
                nxt = a[0] + (b[0] - a[0]) * (target_vout - a[1]) / (b[1] - a[1])
                break
        if nxt is None:                                     # no straddle yet: step outward past the closest end
            lo_p, hi_p = pts[0], pts[-1]
            slope_end = hi_p if abs(hi_p[1] - target_vout) < abs(lo_p[1] - target_vout) else lo_p
            step = 0.12 * (hi - lo)
            nxt = slope_end[0] + (step if slope_end is hi_p else -step)
        nxt = min(max(nxt, 0.5 * lo), 1.5 * hi)
        if any(abs(nxt - p[0]) < 1e-9 * max(1.0, abs(nxt)) for p in pts):
            break                                           # converged on the grid (no new point to try)
        r = sample(nxt)
        if r is not None:
            pts.append(r)

    bx, vout, pin, pout = min(pts, key=lambda p: abs(p[1] - target_vout))
    value = bx
    return {
        "converged": True,
        "regulated": abs(vout - target_vout) <= tol * target_vout,   # did we actually reach target?
        "topology": topology,
        "control": ctrl,
        "value": value,                          # the regulated duty / phaseDeg / frequency
        "vout": vout,
        "target_vout": target_vout,
        "vout_error": (vout - target_vout) / target_vout,
        "pin": pin,
        "pout": pout,
        "efficiency": (pout / pin) if (pin and pout == pout) else float("nan"),
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
        vout, pin, pout = ol
        print(f"open-loop  (duty={d_des:.4f}): Vout={vout:.3f} V  eff={pout/pin*100:.1f}%")
    r = simulate_regulated(copy.deepcopy(tas), target_vout=24.0, topology="boost", fidelity=fid)
    print(f"regulated  (duty={r['value']:.4f}): Vout={r['vout']:.3f} V  eff={r['efficiency']*100:.1f}%  "
          f"regulated={r['regulated']} (target {r['target_vout']})")
