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
  * Dual-output topologies (isolated_buck / isolated_buck_boost): efficiency sums BOTH rails. The secondary
    load lives INSIDE the stage subckt, so its node is reached via the hierarchical name (xinst.node) and its
    ground (a subckt port) maps to the X-line's top node. With that the flybuck reports a sane full-converter
    efficiency and the secondary delivers its rail — an earlier primary-only reading that looked like a broken
    ~25% / ~1 V secondary was purely a top-level-node measurement artifact, not a converter bug.

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
try:
    import PyOpenMagnetics as _PYOM  # authoritative magnetics models (calculate_saturation_current); optional
except Exception:  # pragma: no cover - PyOM not installed -> verdict falls back to the subcircuit's Isat
    _PYOM = None

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


def _set_control(tas, field, value, topology=None):
    """Set the control field across the pwm stimuli.

    duty / frequency: apply to every pwm leg uniformly.
    phaseDeg: a phase-shifted bridge has a FIXED reference leg (phases 0 / 180) and a LAGGING leg whose two
        switches sit at φ and φ+180. We hold the reference leg and move only the lagging leg, so `value` is the
        bridge phase-shift φ — not a blanket overwrite (which would collapse the bridge).
    fsbb: the 4-switch buck-boost is a COORDINATED modulation, not uniform duty — the charge switches (Q1/Q4,
        designed at phase 0) run at duty D=value while the discharge switches (Q2/Q3) run the complementary
        (1−D)−2·dt at phase (D+dt)·360 (dt = the per-leg dead-time, stashed from the design by
        _stash_fsbb_modulation). A blanket duty overwrite drives all four legs the same → the bridge collapses
        and Vout never reaches target. Charge legs keep phase 0 throughout, so phase==0 vs ≠0 stays a stable
        partition across the bisection."""
    if topology == "fsbb" and field == "dutyCycle":
        dt = float(tas.get("_fsbbDeadFraction", 0.0))
        for st in tas.get("simulation", {}).get("stimulus", []):
            wf = st.get("waveform", {})
            if wf.get("type") != "pwm":
                continue
            if float(wf.get("phaseDeg", 0.0)) == 0.0:                  # charge leg (Q1/Q4)
                wf["dutyCycle"] = value
            else:                                                       # discharge leg (Q2/Q3)
                wf["dutyCycle"] = max(0.0, (1.0 - value) - 2.0 * dt)
                wf["phaseDeg"] = (value + dt) * 360.0
        return
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


def _stash_fsbb_modulation(tas):
    """Recover the fsbb per-leg dead-time fraction from the PRISTINE designed stimulus (before the bisection
    overwrites any duty) and stash it on the tas, so _set_control can re-derive the coordinated 4-switch
    modulation from the single control D. The discharge leg was designed at phase (D0+dt)·360 and the charge
    leg at D0, so dt = (discharge_phase/360) − D0."""
    stims = [st for st in tas.get("simulation", {}).get("stimulus", [])
             if st.get("waveform", {}).get("type") == "pwm"]
    charge = [st for st in stims if float(st["waveform"].get("phaseDeg", 0.0)) == 0.0]
    disch = [st for st in stims if float(st["waveform"].get("phaseDeg", 0.0)) != 0.0]
    if charge and disch:
        d0 = float(charge[0]["waveform"].get("dutyCycle", 0.5))
        ph0 = float(disch[0]["waveform"].get("phaseDeg", 0.0)) / 360.0
        tas["_fsbbDeadFraction"] = max(0.0, ph0 - d0)


_GND = {"0", "gnd", "GND", "Gnd"}


def _parse_deck(deck):
    """Pull what the operating-point measurement needs: the input DC source, EVERY output load, and the
    largest bulk capacitance. A load is identified structurally as an element from an output node (name
    contains 'out') to ground — this catches the primary `Rload Vout 0` AND a multi-output topology's internal
    secondary load (e.g. `RRsec vout_sec gnd`), while excluding an output-cap ESR (which goes to the cap node).

    The assembler renders every stage as its OWN `.subckt` instantiated by an `X` line, so a load that lives
    INSIDE a subckt (the flybuck secondary) has nodes that aren't visible at the top level. We resolve each
    node to a top-level-measurable reference: a subckt PORT maps to the X-line's top node (e.g. gnd->0), an
    internal node becomes the hierarchical `xinst.node`. Returns (vin_name, vin, loads, cmax) with loads a
    list of (out_ref, gnd_ref, kind, value) where the refs are ready to drop into v(...)."""
    lines = deck.splitlines()
    # subckt name -> port list; and which subckt (if any) each line sits in
    subckt_ports, line_sub, cur = {}, [], None
    for ln in lines:
        ms = re.match(r"^\.subckt\s+(\S+)\s+(.*)", ln, re.I)
        me = re.match(r"^\.ends", ln, re.I)
        line_sub.append(cur)
        if ms:
            cur = ms.group(1); subckt_ports[cur] = ms.group(2).split()
        elif me:
            cur = None
    # subckt name -> (hierarchical prefix, {port: top-level node}) from its X instantiation
    inst = {}
    for ln in lines:
        mx = re.match(r"^([Xx]\S+)\s+(.*)", ln)
        if mx:
            toks = mx.group(2).split()
            sub, tops = toks[-1], toks[:-1]
            if sub in subckt_ports and len(tops) == len(subckt_ports[sub]):
                inst[sub] = (mx.group(1).lower(), dict(zip(subckt_ports[sub], tops)))

    def resolve(node, sub):
        if sub is None:
            return node                                   # already a top-level node
        prefix, pmap = inst.get(sub, ("", {}))
        return pmap.get(node, f"{prefix}.{node}")         # port -> top node, else hierarchical internal node

    vin = vin_name = None
    loads = []
    cmax = 0.0
    for i, ln in enumerate(lines):
        sub = line_sub[i]
        m = re.match(r"^(V\w+)\s+\S+\s+\S+\s+DC\s+([-\d.eE+]+)", ln)
        if m and sub is None and vin is None:             # the top-level input DC source
            vin_name, vin = m.group(1), float(m.group(2))
        m = re.match(r"^(R\w+)\s+(\S+)\s+(\S+)\s+([-\d.eE+]+)", ln)
        # A LOAD is a resistor from an output node to ground. Exclude by name the non-load resistors that can
        # also sit near an output node (an output-cap ESR, a numerical snubber/loop-breaker, a bias/bleeder),
        # so a stray one is never silently summed as delivered power.
        if m and not re.search(r"esr|sense|rsn|dcr|bias|bleed", m.group(1), re.I):
            n1, n2, rv = m.group(2), m.group(3), float(m.group(4))
            if re.search("out", n1, re.I) and n2 in _GND:
                loads.append((resolve(n1, sub), resolve(n2, sub), "R", rv))
            elif re.search("out", n2, re.I) and n1 in _GND:
                loads.append((resolve(n2, sub), resolve(n1, sub), "R", rv))
        m = re.match(r"^Iload\s+(\S+)\s+(\S+)\s+DC\s+([-\d.eE+]+)", ln)
        if m:
            loads.append((resolve(m.group(1), sub), resolve(m.group(2), sub), "I", abs(float(m.group(3)))))
        m = re.match(r"^Bload\s+(\S+)\s+(\S+)\b.*?=\s*([-\d.eE+]+)\s*/", ln)   # constant-power
        if m:
            loads.append((resolve(m.group(1), sub), resolve(m.group(2), sub), "P", float(m.group(3))))
        m = re.match(r"^(C\w+)\s+\S+\s+\S+\s+([-\d.eE+]+)", ln)
        # cmax is the OUTPUT bulk cap (sets the settle RC). Exclude model-internal
        # caps by name: the saturating-inductor flux integrator (Cflux_*) is a 1 F
        # numerical integrator on an internal flux node, NOT a power capacitor —
        # counting it makes rc (and the .tran settle) explode by orders of magnitude.
        if m and not re.search(r"flux", m.group(1), re.I):
            cmax = max(cmax, float(m.group(2)))
    return vin_name, vin, loads, cmax


def _simulate(tas, fidelity, tag):
    """Render + run the deck; return (vout_avg, pin, pout, steady) at steady state, or None on
    non-convergence. fsw is read from the (possibly just-modified) stimulus so frequency control settles
    correctly; the settle is RC-based (max of 400 periods, 10 output-RC), so a slow/high-R output is actually
    settled. pout sums EVERY load (so a dual-output converter reports full-converter, not primary-only, power).
    `steady` compares Vout over two windows (one ~24 periods before the end, one at the end) to flag a deck
    that is still drifting / in a slow limit cycle rather than truly settled."""
    deck = PyKirchhoff.tas_to_ngspice(tas, fidelity)
    vin_name, vin, loads, cmax = _parse_deck(deck)
    fsw = _design_fsw(tas)
    period = 1.0 / fsw
    r_primary = next((v for n, ref, k, v in loads if k == "R" and n.lower() == "vout"), None)
    rc = (r_primary if r_primary else 1.0) * cmax          # output-RC time constant (largest bulk cap)
    settle = max(400.0 * period, 10.0 * rc)
    tstep = period / 200.0
    deck = re.sub(r"\.tran\s+\S+\s+\S+\s+\S+\s+\S+",
                  f".tran {tstep:.12g} {settle:.12g} 0 {tstep:.12g}", deck)
    cpos = deck.rfind("\n.control")
    if cpos != -1:
        deck = deck[:cpos]
    # Instantaneous total output power as a behavioural source (sum over loads), then meas its AVERAGE — the
    # true average load power (avg of v*i_load), correct under ripple and load-agnostic. (`meas rms` is broken.)
    # An isolated rail (flybuck secondary) returns to its OWN node, not the primary ground 0, and floats vs 0,
    # so the load power must use the DIFFERENTIAL voltage across the load (v(out)-v(ref)), not node-to-0.
    def vd(node, ref):
        return f"v({node})" if ref == "0" else f"(v({node})-v({ref}))"
    terms = []
    for node, ref, kind, val in loads:
        d = vd(node, ref)
        if kind == "R":
            terms.append(f"{d}*{d}/{val:.10g}")
        elif kind == "I":
            terms.append(f"{d}*{val:.10g}")
        elif kind == "P":
            terms.append(f"{val:.10g}")
    pexpr = " + ".join(terms) if terms else None
    frm, to = settle - period, settle
    w0 = settle - 100.0 * period                                    # last ~100 periods, for the steady check
    ctrl = ["run",
            f"meas tran vout avg v(Vout) from={frm:.12g} to={to:.12g}",
            f"meas tran vmax max v(Vout) from={w0:.12g} to={to:.12g}",
            f"meas tran vmin min v(Vout) from={w0:.12g} to={to:.12g}",
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
    vout, vmax, vmin, iin, pout_m = grab("vout"), grab("vmax"), grab("vmin"), grab("iin"), grab("pout")
    if vout is None or iin is None or vin is None:
        return None
    pin = abs(vin * iin)
    pout = abs(pout_m) if pout_m is not None else float("nan")
    # relative peak-to-peak of Vout over the last ~100 periods: just switching ripple when settled, large when
    # still drifting / in a slow limit cycle. The caller turns this into the steady/not-steady decision so the
    # threshold stays a tunable parameter (not a magic number buried here).
    ripple = 0.0 if (vmax is None or vmin is None or abs(vout) < 1e-9) else (vmax - vmin) / abs(vout)
    return vout, pin, pout, ripple


# ── magnetic-saturation verdict ───────────────────────────────────────────────────────────────────────
# An MKF_MODEL core models saturation as Lmag = L0/(1+(I/Isat)^2). When the design's peak winding current
# exceeds the core's Isat, the magnetizing inductance collapses at the operating point and the transient
# aborts ('timestep too small'). That is a magnetic-DESIGN failure (core too small / ungapped for the
# current), NOT a solver problem — so report it as an explicit verdict (Isat from the MKF subcircuit, peak
# current from the design's winding excitation) instead of opaque divergence. (HS ABT #33.)
_NG_SUFFIX = {"meg": 1e6, "t": 1e12, "g": 1e9, "k": 1e3,
              "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12, "f": 1e-15}


def _ng_value(s):
    """Parse an ngspice numeric literal (optional engineering suffix, e.g. '160.3u', '411.617m', '0.41')."""
    m = re.match(r"\s*([-+]?[0-9.]+(?:[eE][-+]?[0-9]+)?)\s*(meg|[tgkmunpf])?", s, re.I)
    if not m:
        return None
    v = float(m.group(1))
    if m.group(2):
        v *= _NG_SUFFIX[m.group(2).lower()]
    return v


def _walk_components(node):
    """Yield every component dict anywhere under a TAS topology (stages -> circuits -> components, nested)."""
    if isinstance(node, dict):
        for c in node.get("components", []) or []:
            yield c
        for v in node.values():
            yield from _walk_components(v)
    elif isinstance(node, list):
        for v in node:
            yield from _walk_components(v)


def _winding_excitations(data):
    """Per-winding processed current excitation the design imposes on a magnetic, ordered by winding."""
    for op in data.get("inputs", {}).get("operatingPoints", []) or []:
        excs = op.get("excitationsPerWinding", []) or []
        if excs:
            return [e.get("current", {}).get("processed", {}) for e in excs]
    return []


_ISAT_MISMATCH_FRAC = 0.5   # |exported - model| / model above this = a real exporter/model disagreement
                            # (the ABT #33 bug was ~N-fold; temperature/rounding scatter is well under 50%)


def _magnetic_temperature(data):
    """Ambient temperature from the magnetic's operating point (deg C); 25 if unspecified."""
    for op in data.get("inputs", {}).get("operatingPoints", []) or []:
        t = (op.get("conditions") or {}).get("ambientTemperature")
        if t is not None:
            return float(t)
    return 25.0


def _datasheet_isat(magnetic):
    """Datasheet-rated peak saturation current from manufacturerInfo.datasheetInfo (the electrical entries'
    saturationCurrentPeak), or None. Used only when the core/coil architecture is absent, so the physical
    model can't run."""
    found = []

    def _walk(o):
        if isinstance(o, dict):
            v = o.get("saturationCurrentPeak")
            if isinstance(v, (int, float)) and v > 0:
                found.append(float(v))
            for x in o.values():
                _walk(x)
        elif isinstance(o, list):
            for x in o:
                _walk(x)

    _walk((magnetic.get("manufacturerInfo") or {}).get("datasheetInfo"))
    return min(found) if found else None


def _authoritative_isat(magnetic, temperature):
    """The authoritative saturation current and its source. Precedence (the house rule): use the PHYSICAL
    MODEL (Magnetic::calculate_saturation_current — the SAME one Heaviside's realism gate and the fixed MKF
    exporter use) when the core/coil ARCHITECTURE is present; only when the architecture is missing fall back
    to the DATASHEET rating (datasheetInfo.electrical[].saturationCurrentPeak). NEVER the subcircuit's emitted
    value (that's what we cross-check against) and NEVER a re-derived formula (the ABT #33 lesson). Returns
    (Isat, source) with source in {'model','datasheet'}, or (None, None)."""
    if not isinstance(magnetic, dict):
        return None, None
    if magnetic.get("core") and magnetic.get("coil") and _PYOM is not None:
        try:
            return abs(float(_PYOM.calculate_saturation_current(magnetic, temperature))), "model"
        except Exception:
            pass
    ds = _datasheet_isat(magnetic)
    return (ds, "datasheet") if ds is not None else (None, None)


def saturation_findings(tas):
    """Per single-winding MKF_MODEL inductor, judge saturation from the AUTHORITATIVE calculate_saturation_
    current (NOT the subcircuit's emitted Isat) and CROSS-CHECK the two. Two finding kinds, both meaning 'no
    valid regulated operating point':
      kind='saturated'              : DC operating current exceeds the authoritative Isat — the core really is
                                      too small (Lmag collapses all cycle). A genuine magnetic-design failure.
      kind='exporter_isat_mismatch' : the subcircuit's emitted Isat disagrees with calculate_saturation_current
                                      beyond tolerance — the deck will collapse SPURIOUSLY but the core is fine.
                                      An MKF exporter bug, NOT a magnetic-design failure. (This is the check
                                      that would have caught ABT #33 automatically instead of blaming the core.)
    The authoritative Isat is the PHYSICAL MODEL when the core/coil architecture is present, else the DATASHEET
    rating (datasheetInfo.saturationCurrentPeak); the subcircuit's emitted Isat is NEVER the authority, only the
    cross-check target. No finding when neither model nor datasheet can supply Isat. Single-winding only: there
    the winding current IS the magnetizing current; a transformer's load currents cancel in the core (skipped)."""
    out = []
    for comp in _walk_components(tas.get("topology", {})):
        data = comp.get("data", {})
        mag = data.get("magnetic")
        if not isinstance(mag, dict):
            continue
        text = (mag.get("modelOutputs") or {}).get("spiceSubcircuit", {}).get("text")
        if not text:
            continue
        excs = _winding_excitations(data)
        if len(excs) != 1:                               # single-winding inductors only (see docstring)
            continue
        iop = excs[0].get("offset")                      # DC operating current (saturates Lmag all cycle)
        if iop is None:
            continue
        iop = abs(float(iop))
        sub_isats = [v for v in (_ng_value(x) for x in
                     re.findall(r"_Isat\s*=\s*([0-9.eE+\-]+\s*[a-zA-Z]*)", text)) if v and v > 0]
        isat_subckt = min(sub_isats) if sub_isats else None     # the exporter's emitted value -> cross-check ONLY
        isat, isat_source = _authoritative_isat(mag, _magnetic_temperature(data))
        if isat is None:                                 # no architecture AND no datasheet -> can't judge, no claim
            continue
        name = comp.get("name", "?")
        pk = excs[0].get("peak")
        peak = abs(float(pk)) if pk is not None else None
        if iop > isat:                                   # genuine saturation, by the authoritative Isat
            out.append({"kind": "saturated", "component": name, "operating_current": iop, "peak_current": peak,
                        "isat": isat, "isat_source": isat_source, "ratio": iop / isat})
        elif (isat_subckt is not None
              and abs(isat_subckt - isat) > _ISAT_MISMATCH_FRAC * isat):
            out.append({"kind": "exporter_isat_mismatch", "component": name, "operating_current": iop,
                        "isat_authoritative": isat, "isat_authoritative_source": isat_source,
                        "isat_subcircuit": isat_subckt, "ratio": isat_subckt / isat})
    return out


def simulate_regulated(tas, target_vout, topology, fidelity=None, tol=0.01, max_iter=24,
                       steady_ripple_frac=0.10):
    """Bisect the topology's control variable until the simulated Vout hits target_vout, then report the
    regulated operating point + efficiency.

    Tunable (no magic numbers): `tol` = Vout-on-target band; `steady_ripple_frac` = max Vout peak-to-peak (as a
    fraction of Vout) still counted as steady-state.

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
    if topology == "fsbb":
        _stash_fsbb_modulation(tas)   # capture dead-time before the bisection overwrites duties
    # A magnetic that can't give a valid regulated point (the deck collapses across the whole bracket): either
    # a genuinely saturated core, OR a sound core whose exported Isat disagrees with calculate_saturation_
    # current (an exporter bug that collapses the deck spuriously). Surface the right verdict instead of an
    # opaque converged=False, and don't grind the bisect through decks that all abort (HS ABT #33).
    issues = saturation_findings(tas)
    if issues:
        saturated = [s for s in issues if s["kind"] == "saturated"]
        if saturated:
            w = max(saturated, key=lambda s: s["ratio"])
            reason = (f"magnetic saturated at operating current: {w['component']} operating "
                      f"{w['operating_current']:.3g} A > Isat {w['isat']:.3g} A ({w['ratio']:.1f}x, Isat from "
                      f"{w['isat_source']}) — Lmag collapses, so no control setting yields a valid operating "
                      f"point. Magnetic-design failure (core too small / ungapped for the current).")
        else:
            m = issues[0]   # exporter_isat_mismatch
            reason = (f"exported saturation current disagrees with calculate_saturation_current: "
                      f"{m['component']} subcircuit Isat {m['isat_subcircuit']:.3g} A vs model "
                      f"{m['isat_authoritative']:.3g} A ({m['ratio']:.2f}x) — the MKF saturable-L collapses "
                      f"SPURIOUSLY, but the core is NOT saturated (operating {m['operating_current']:.3g} A < "
                      f"{m['isat_authoritative']:.3g} A). MKF exporter bug, not a magnetic-design failure — fix "
                      f"the exporter's Isat.")
        return {
            "converged": False, "regulated": False, "steady_state": False,
            "magnetic_issues": issues, "saturated": saturated,
            "topology": topology, "control": ctrl, "target_vout": target_vout, "reason": reason,
        }
    fsw = _design_fsw(tas)
    # Frequency control: stay in the ABOVE-resonance region where Vout is MONOTONIC in frequency (below
    # resonance the gain turns over). The design fsw is at/above resonance, so bracket from just under it up.
    lo, hi = bracket if bracket is not None else (0.8 * fsw, 2.2 * fsw)

    def sample(x):
        """Evaluate the control value x; on non-convergence jitter a few % (some operating points in a real
        resonant deck diverge sporadically) to find a nearby valid point. Returns (x_used, vout, pin, pout)."""
        for dj in (0.0, 0.03, -0.03, 0.06, -0.06, 0.10):
            xj = x * (1.0 + dj)
            _set_control(tas, field, xj, topology)
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

    bx, vout, pin, pout, ripple = min(pts, key=lambda p: abs(p[1] - target_vout))
    steady = ripple <= steady_ripple_frac
    return {
        "converged": True,
        "regulated": abs(vout - target_vout) <= tol * target_vout,   # did we actually reach target?
        "steady_state": bool(steady),            # False -> Vout still drifting/oscillating; treat eff with care
        "topology": topology,
        "control": ctrl,
        "value": bx,                             # the regulated duty / phaseDeg / frequency
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
        vout, pin, pout, _steady = ol
        print(f"open-loop  (duty={d_des:.4f}): Vout={vout:.3f} V  eff={pout/pin*100:.1f}%")
    r = simulate_regulated(copy.deepcopy(tas), target_vout=24.0, topology="boost", fidelity=fid)
    print(f"regulated  (duty={r['value']:.4f}): Vout={r['vout']:.3f} V  eff={r['efficiency']*100:.1f}%  "
          f"regulated={r['regulated']} (target {r['target_vout']})")
