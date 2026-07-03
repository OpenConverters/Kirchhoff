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
except ImportError:  # pragma: no cover - PyOM not installed -> verdict falls back to the subcircuit's Isat
    _PYOM = None   # narrow to ImportError: an ABI/other error must surface, not silently downgrade authority

# control variable per topology: (stimulus waveform field, search bracket). The Vout-vs-control direction is
# auto-detected (sampling the bracket ends), so we never hardcode a possibly-wrong monotonicity.
_CONTROL = {
    "duty":      ("dutyCycle", (0.04, 0.96)),   # buck family
    "phase":     ("phase",     (15.0, 170.0)),  # psfb/dab: phase-shift of the lagging leg, ~0..180 deg
                                                 # (waveform field is "phase" — matches TasAssembler + the KH designs)
    "frequency": ("frequency", None),           # resonant: bracket derived from the design fsw
    # pshb: the 3-level NPC stack is NOT a phase-shifted bridge — its output is set by the OUTER pair's
    # (S1/S4) power-transfer width (duty = D/2 of the period), with the inner pair (S2/S3) held ~50%
    # complementary. The control is that outer DUTY, bounded below the inner ~0.5 (above it the outer
    # switches overlap the inner and the NPC mis-commutates). abt #66.
    "pshbDuty":  ("dutyCycle", (0.08, 0.49)),
}
_TOPO_CONTROL = {
    **{t: "duty" for t in ("boost", "buck", "flyback", "forward", "two_switch_forward", "sepic", "cuk",
                           "zeta", "ahb", "acf", "fsbb", "weinberg", "isolated_buck", "isolated_buck_boost",
                           "push_pull")},
    **{t: "phase" for t in ("psfb", "dab")},
    "pshb": "pshbDuty",   # 3-level NPC: control the outer-pair width, not a leg phase (abt #66)
    **{t: "frequency" for t in ("llc", "src", "cllc", "clllc")},
}

# AC-input topologies whose closed-loop controller is rendered INTO the deck (a designed PI voltage loop +
# inner current loop, no open-loop stimulus). They REGULATE THEMSELVES, so there is no control variable to
# bisect — the operating point is "run the deck once and measure". Their two-timescale dynamics (line vs
# switching) need LINE-cycle measurement windowing, not the switching-period windowing the DC path uses.
_SELF_REGULATED = {"pfc", "vienna"}


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
    fsbb / ahb: a COORDINATED-COMPLEMENTARY modulation, not uniform duty — the charge switch(es) (designed
        at phase 0) run at duty D=value while the complementary switch(es) run (1−D)−2·dt at phase (D+dt)·360
        (dt = the per-leg dead-time, stashed from the design by _stash_fsbb_modulation). A blanket duty
        overwrite drives BOTH halves at the same duty → for the fsbb the bridge collapses; for the asymmetric
        half-bridge (ahb) the two complementary switches OVERLAP and SHOOT THROUGH (vin→gnd) the moment the
        bisected duty exceeds the design duty, so the input current runs away. The charge leg keeps phase 0
        throughout, so phase==0 vs ≠0 stays a stable partition across the bisection."""
    if topology == "pshb" and field == "dutyCycle":
        # 3-level NPC half-bridge: drive ONLY the OUTER power-transfer pair (S1/S4) at the bisected width;
        # the inner pair (S2/S3) stays at its designed ~50% complementary duty/phase. A blanket overwrite
        # (the generic loop below) would set the inner pair's duty too, collapsing the NPC's neutral-point
        # commutation — which is exactly why mapping pshb to a leg "phase" gave an inverted, ~5%-efficient,
        # un-regulating response (abt #66). The outer pair was stashed from the pristine deck.
        outer = set(tas.get("_pshbOuter", []))
        for st in tas.get("simulation", {}).get("stimulus", []):
            wf = st.get("waveform", {})
            if wf.get("type") == "pwm" and st.get("component") in outer:
                wf["dutyCycle"] = value
        return
    if topology in ("fsbb", "ahb", "buck", "boost") and field == "dutyCycle":
        # fsbb/ahb: coordinated 4-/2-switch complementary modulation. SYNCHRONOUS buck/boost (abt #67): the
        # phase-0 main switch (buck high-side Q1 / boost low-side Q1) runs at the bisected duty D=value while
        # the sync rectifier Q2 (phase (D+dt)·360 ≠ 0; buck low-side / boost high-side) runs the COMPLEMENT
        # (1−D)−2·dt — a blanket uniform overwrite would drive Q2 at the SAME duty as Q1, leaving the body
        # diode to conduct the rest of the rectifier period (worse than the diode it replaced). A DIODE
        # buck/boost has only the single phase-0 switch, so this branch sets just its duty — identical to
        # the generic loop (no behaviour change for the default).
        dt = float(tas.get("_fsbbDeadFraction", 0.0))
        for st in tas.get("simulation", {}).get("stimulus", []):
            wf = st.get("waveform", {})
            if wf.get("type") != "pwm":
                continue
            if float(wf.get("phase", 0.0)) == 0.0:                     # charge leg (Q1/Q4) / buck high-side
                wf["dutyCycle"] = value
            else:                                                       # discharge leg (Q2/Q3) / buck sync FET
                wf["dutyCycle"] = max(0.0, (1.0 - value) - 2.0 * dt)
                wf["phase"] = (value + dt) * 360.0
        return
    for st in tas.get("simulation", {}).get("stimulus", []):
        wf = st.get("waveform", {})
        if wf.get("type") != "pwm":
            continue
        if field == "phase":
            p = float(wf.get("phase", 0.0))
            if p in (0.0, 180.0):
                continue                                   # reference leg — fixed
            wf["phase"] = value if p < 180.0 else (value + 180.0)   # lagging leg: φ and φ+180
        else:
            wf[field] = value


def _stash_pshb_modulation(tas):
    """Record the PSHB 3-level-NPC OUTER pair from the pristine designed stimulus (before the bisection
    overwrites duties). The inner pair runs ~50% complementary (duty ≈ 0.5−dt); the outer power-transfer
    pair is designed NARROWER (duty = D/2), so the two pwm switches with the smallest duty ARE the outer
    pair whose width the regulator controls (abt #66)."""
    stims = [st for st in tas.get("simulation", {}).get("stimulus", [])
             if st.get("waveform", {}).get("type") == "pwm" and st.get("component")]
    if len(stims) >= 4:
        outer = sorted(stims, key=lambda st: float(st["waveform"].get("dutyCycle", 0.5)))[:2]
        tas["_pshbOuter"] = [st.get("component") for st in outer]


def _stash_fsbb_modulation(tas):
    """Recover the fsbb per-leg dead-time fraction from the PRISTINE designed stimulus (before the bisection
    overwrites any duty) and stash it on the tas, so _set_control can re-derive the coordinated 4-switch
    modulation from the single control D. The discharge leg was designed at phase (D0+dt)·360 and the charge
    leg at D0, so dt = (discharge_phase/360) − D0."""
    stims = [st for st in tas.get("simulation", {}).get("stimulus", [])
             if st.get("waveform", {}).get("type") == "pwm"]
    charge = [st for st in stims if float(st["waveform"].get("phase", 0.0)) == 0.0]
    disch = [st for st in stims if float(st["waveform"].get("phase", 0.0)) != 0.0]
    if charge and disch:
        d0 = float(charge[0]["waveform"].get("dutyCycle", 0.5))
        ph0 = float(disch[0]["waveform"].get("phase", 0.0)) / 360.0
        tas["_fsbbDeadFraction"] = max(0.0, ph0 - d0)


_GND = {"0", "gnd", "GND", "Gnd"}


def _sanitize(s):
    """ngspice identifier-safe form of a TAS name — MUST mirror TasAssembler::sanitize / ngnodes::sanitize
    (alnum/'_' kept, '+'->'p', '-'->'n', everything else '_'), so a reconstructed element/node token equals
    the deck's."""
    out = []
    for c in s:
        if c.isalnum() or c == "_":
            out.append(c)
        elif c == "+":
            out.append("p")
        elif c == "-":
            out.append("n")
        else:
            out.append("_")
    return "".join(out)


def _deck_hierarchy(deck):
    """Parse the assembler's per-stage `.subckt`/`X` structure ONCE and return (lines, line_sub, inst,
    resolve). `line_sub[i]` is the subckt name that line i sits in (None at top level); `inst[sub]` is
    (hierarchical_prefix, {port: top_level_node}) from that subckt's X instantiation; `resolve(node, sub)`
    maps a subckt-local node to a top-level-measurable reference (a port -> the X-line's top node, an internal
    net -> the hierarchical `xinst.node`). This is the shared node-naming rule both _parse_deck (loads) and the
    per-component excitation attachment reuse, so there is exactly one copy of the convention."""
    lines = deck.splitlines()
    subckt_ports, line_sub, cur = {}, [], None
    for ln in lines:
        ms = re.match(r"^\.subckt\s+(\S+)\s+(.*)", ln, re.I)
        me = re.match(r"^\.ends", ln, re.I)
        line_sub.append(cur)
        if ms:
            cur = ms.group(1)
            subckt_ports[cur] = ms.group(2).split()
        elif me:
            cur = None
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

    return lines, line_sub, inst, resolve


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
    lines, line_sub, inst, resolve = _deck_hierarchy(deck)

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
        m = re.match(r"^Iload\d*\s+(\S+)\s+(\S+)\s+DC\s+([-\d.eE+]+)", ln)   # \d*: multi-output suffix
        if m:
            loads.append((resolve(m.group(1), sub), resolve(m.group(2), sub), "I", abs(float(m.group(3)))))
        m = re.match(r"^Bload\d*\s+(\S+)\s+(\S+)\b.*?=\s*([-\d.eE+]+)\s*/", ln)   # constant-power, \d*: suffix
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


# ── per-component SIMULATED excitation attachment ─────────────────────────────────────────────────────────
# After the regulated operating point is found, re-run the deck ONCE and attach the SIMULATED waveform (one
# operating point, summarised peak/rms/avg/pp) to every PASSIVE component, so the realism gate + BOM read REAL
# per-component stresses instead of only the magnetic's design excitation.
#
# What we emit (the reliable subset — every quantity is directly MEASURED, never a placeholder):
#   * VOLTAGE for every passive (capacitor / resistor / inductor): `.meas tran RMS|PP|MAX|MIN|AVG v(n1,n2)` —
#     fully reliable for any node pair.
#   * CURRENT for INDUCTORS/magnetic windings only: the winding current is continuous (no displacement spikes),
#     so `.meas` on the savecurrents device vector `@l.<xinst>.<elem>[i]` gives a trustworthy RMS/AVG/peak.
# What we deliberately OMIT (per the no-silent-wrong-value rule):
#   * capacitor / resistor CURRENT. The instantaneous device current through a cap/resistor is dominated by
#     switching-edge DISPLACEMENT spikes (measured e.g. 682 A peak-to-peak on a boost output cap whose real
#     ripple RMS is ~4.8 A): the RMS/AVG are actually sane, but the PEAK/PP are numerical artefacts. Rather than
#     emit a half-trustworthy current (a valid processedWaveform needs a peak), we emit NO current for these —
#     never a 0 or a placeholder. (A series 0 V current sense per passive, or a rawfile parse, would give a
#     clean full current; left as future work — see abt #35 path (b).)
_LETTER_OF = {"magnetic": "L", "capacitor": "C", "resistor": "R"}


def _passive_elements(deck):
    """Every top-level/subckt PASSIVE element line (R/L/C) with its two nodes resolved to top-level-measurable
    references and its savecurrents device-current vector. Returns [{name, letter, body, n1, n2, dev_i}], where
    `name` is the full ngspice element token (e.g. 'LL1_pri'), `body` is that minus the type letter (the token
    the assembler built from the TAS component name, e.g. 'L1_pri'), and `dev_i` is the `@<type>.<xinst>.<elem>[i]`
    (hierarchical) or `@<elem>[i]` (top-level) savecurrents vector for its current."""
    lines, line_sub, inst, resolve = _deck_hierarchy(deck)
    out = []
    for i, ln in enumerate(lines):
        m = re.match(r"^([RLCrlc])(\w*)\s+(\S+)\s+(\S+)\s+", ln)
        if not m:
            continue
        letter = m.group(1).upper()
        body = m.group(2)
        name = m.group(1) + body
        sub = line_sub[i]
        r1, r2 = resolve(m.group(3), sub), resolve(m.group(4), sub)
        if r1 in _GND and r2 not in _GND:                 # orient so a ground reference is the second node
            r1, r2 = r2, r1
        if sub is not None:
            prefix = inst.get(sub, ("", {}))[0]
            dev_i = f"@{letter.lower()}.{prefix}.{name.lower()}[i]"
        else:
            dev_i = f"@{name.lower()}[i]"
        out.append({"name": name, "letter": letter, "body": body, "n1": r1, "n2": r2, "dev_i": dev_i})
    return out


def _passive_components(tas):
    """Every PASSIVE TAS component (magnetic / capacitor / resistor) as {name: (component_dict, type_letter)}."""
    out = {}
    for comp in _walk_components(tas.get("topology", {})):
        data = comp.get("data")
        name = comp.get("name")
        if not isinstance(data, dict) or not name:
            continue
        disc = next((k for k in ("magnetic", "capacitor", "resistor") if k in data), None)
        if disc is not None:
            out[name] = (comp, _LETTER_OF[disc])
    return out


def _own_element(elem, passives):
    """The passive TAS component name that owns a deck element, or None. An element belongs to component C iff
    their type letters match and the element body equals sanitize(C) exactly or begins with sanitize(C)+'_' (a
    multi-atom/multi-winding suffix). The LONGEST matching name wins, so 'C1' never steals 'C10'. Elements with
    no owner (a semiconductor's parasitic Coss/ESR atom, the external load resistor) are left unmeasured."""
    cands = [n for n, (_c, ltr) in passives.items()
             if ltr == elem["letter"] and (_sanitize(n) == elem["body"]
                                            or elem["body"].startswith(_sanitize(n) + "_"))]
    return max(cands, key=lambda n: len(_sanitize(n))) if cands else None


def _processed(stats):
    """A schema-valid processedWaveform from measured (rms, pp, max, min, avg). label='custom' (a real measured
    waveform), offset/average = the measured average, peak = the measured maximum ABSOLUTE value, peakToPeak =
    the measured pp; positive/negativePeak only when their sign constraint holds."""
    vmax, vmin = stats["max"], stats["min"]
    p = {"label": "custom", "offset": stats["avg"], "average": stats["avg"],
         "rms": abs(stats["rms"]), "peak": max(abs(vmax), abs(vmin)), "peakToPeak": abs(stats["pp"])}
    if vmax >= 0.0:
        p["positivePeak"] = vmax
    if vmin <= 0.0:
        p["negativePeak"] = vmin
    return p


def attach_simulated_excitations(tas, fidelity, tag="attach"):
    """Re-run the (regulated) deck once and attach a SIMULATED operating point to every passive component, in
    place on `tas`. The control field must already be set to the regulated value. Returns {component_name: op}
    for the operating points attached (also useful for verification). Raises if a MATCHED passive element's
    reliable voltage (or an inductor's current) can't be measured — that is a reconstruction bug to surface, not
    to paper over."""
    deck = PyKirchhoff.tas_to_ngspice(tas, fidelity)
    _vin_name, _vin, loads, cmax = _parse_deck(deck)
    fsw = _design_fsw(tas)
    period = 1.0 / fsw
    r_primary = next((v for n, ref, k, v in loads if k == "R" and n.lower() == "vout"), None)
    rc = (r_primary if r_primary else 1.0) * cmax
    settle = max(400.0 * period, 10.0 * rc)
    tstep = period / 200.0
    deck = re.sub(r"\.tran\s+\S+\s+\S+\s+\S+\s+\S+",
                  f".tran {tstep:.12g} {settle:.12g} 0 {tstep:.12g}", deck)
    cpos = deck.rfind("\n.control")
    if cpos != -1:
        deck = deck[:cpos]
    if "savecurrents" not in deck:            # inductor current reads the savecurrents device vector @l[i]
        deck += "\n.options savecurrents\n"

    passives = _passive_components(tas)
    owned = [(e, _own_element(e, passives)) for e in _passive_elements(deck)]
    owned = [(e, o) for e, o in owned if o is not None]
    if not owned:
        return {}
    frm, to = settle - period, settle
    ctrl = ["run"]
    # `meas ... v(a,b)` FAILS when a node is subckt-internal (hierarchical) — ngspice only accepts a single
    # saved vector there. So materialise each component's terminal-voltage DIFFERENTIAL with a `let` first
    # (v(n1)-v(n2), or v(n1) to ground), then meas that vector. Inductor current reads the savecurrents device
    # vector directly (that form does accept the hierarchical @l.<xinst>.<elem>[i]).
    for i, (e, _o) in enumerate(owned):
        vexpr = f"v({e['n1']})" if e["n2"] in _GND else f"v({e['n1']}) - v({e['n2']})"
        ctrl.append(f"let m_{i}_vd = {vexpr}")
        for suf, typ in (("vrms", "rms"), ("vpp", "pp"), ("vmax", "max"), ("vmin", "min"), ("vavg", "avg")):
            ctrl.append(f"meas tran m_{i}_{suf} {typ} m_{i}_vd from={frm:.12g} to={to:.12g}")
        if e["letter"] == "L":                # inductor current: continuous, no displacement spikes -> reliable
            for suf, typ in (("irms", "rms"), ("ipp", "pp"), ("imax", "max"), ("imin", "min"), ("iavg", "avg")):
                ctrl.append(f"meas tran m_{i}_{suf} {typ} {e['dev_i']} from={frm:.12g} to={to:.12g}")
    deck += "\n.control\n" + "\n".join(ctrl) + "\n.endc\n.end\n"
    with tempfile.NamedTemporaryFile("w", suffix=f"_{tag}.cir", delete=False) as f:
        f.write(deck)
        path = f.name
    try:
        out = subprocess.run(["ngspice", "-b", path], capture_output=True, text=True, timeout=300)
        text = out.stdout + out.stderr
    finally:
        os.unlink(path)

    def grab(name):
        m = re.search(rf"\b{name}\s*=\s*([-\d.eE+]+)", text)
        return float(m.group(1)) if m else None

    # Group measured voltage (all) + current (inductors) per element, raising on any missing MATCHED quantity.
    per_comp = {}
    for i, (e, owner) in enumerate(owned):
        v = {k: grab(f"m_{i}_v{k}") for k in ("rms", "pp", "max", "min", "avg")}
        if any(val is None for val in v.values()):
            raise ValueError(f"attach_simulated_excitations: no simulated voltage for {e['name']} "
                             f"(v across {e['n1']},{e['n2']}) — node reconstruction bug")
        cur = None
        if e["letter"] == "L":
            cur = {k: grab(f"m_{i}_i{k}") for k in ("rms", "pp", "max", "min", "avg")}
            if any(val is None for val in cur.values()):
                raise ValueError(f"attach_simulated_excitations: no simulated current for inductor "
                                 f"{e['name']} ({e['dev_i']}) — savecurrents/device-name bug")
        per_comp.setdefault(owner, []).append((e, v, cur))

    attached = {}
    for owner, elems in per_comp.items():
        comp, letter = passives[owner]
        data = comp["data"]
        data.setdefault("inputs", {})
        prior = data["inputs"].get("operatingPoints", []) or []
        temp = 25.0
        for op in prior:                      # reuse the design operating point's ambient temperature
            t = (op.get("conditions") or {}).get("ambientTemperature")
            if isinstance(t, (int, float)):
                temp = float(t)
                break
        if letter == "L":                     # magnetic: mirror excitationsPerWinding (one per winding element)
            windings = []
            for e, v, cur in elems:
                we = {"frequency": fsw, "voltage": {"processed": _processed(v)}}
                if cur is not None:
                    we["current"] = {"processed": _processed(cur)}
                windings.append(we)
            op = {"name": "simulated", "conditions": {"ambientTemperature": temp},
                  "excitationsPerWinding": windings}
        else:                                 # two-terminal (capacitor / resistor): a single excitation
            e, v, _cur = max(elems, key=lambda t: _sanitize(owner) == t[0]["body"])
            op = {"name": "simulated", "conditions": {"ambientTemperature": temp},
                  "excitation": {"frequency": fsw, "voltage": {"processed": _processed(v)}}}
        # Simulated op-point lands at operatingPoints[0] (what the realism gate / BOM read); any DESIGN op-points
        # are preserved after it. Re-attachment replaces a prior 'simulated' op rather than stacking duplicates.
        data["inputs"]["operatingPoints"] = [op] + [o for o in prior if o.get("name") != "simulated"]
        attached[owner] = op
    return attached


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
    # CRITICAL: prefix every meas RESULT name with `m_`. ngspice is case-insensitive and a `meas`
    # result becomes a (scalar) vector in the plot — so naming a measurement `vout` SHADOWS the node
    # vector `v(Vout)`. Every measurement AFTER it that reads `v(Vout)` (vmax/vmin and each vld*) then
    # operates on that 1-point scalar and returns 0 with a collapsed window (`to=0`), silently zeroing
    # pout/efficiency and forcing vmax==vmin (ripple 0 -> always "steady"). The `m_` namespace can't
    # collide with a circuit node, so each measurement reads the true transient vector. (abt #54.)
    ctrl = ["run",
            f"meas tran m_vout avg v(Vout) from={frm:.12g} to={to:.12g}",
            f"meas tran m_vmax max v(Vout) from={w0:.12g} to={to:.12g}",
            f"meas tran m_vmin min v(Vout) from={w0:.12g} to={to:.12g}",
            f"meas tran m_iin avg i({vin_name}) from={frm:.12g} to={to:.12g}"]
    # Output power: measure each load's AVERAGE terminal voltage and form the power in PYTHON below —
    # do NOT inject a behavioural power probe into the circuit. A B-source with a v*v expression (even
    # a current source into a 1 ohm dummy) perturbs the MNA Jacobian and collapses the GLOBAL timestep
    # on stiff resonant decks (the LLC half-bridge body diode aborts with "timestep too small" at
    # t~1e-11 the moment Bpout is present, abt #54). A pure measurement leaves the solve untouched.
    for i, (node, ref, _kind, _val) in enumerate(loads):
        vexpr = f"v({node})" if ref == "0" else f"v({node},{ref})"
        ctrl.append(f"meas tran m_vld{i} avg {vexpr} from={frm:.12g} to={to:.12g}")
    deck += "\n.control\n" + "\n".join(ctrl) + "\nprint v(Vout) i(" + vin_name + ")\n.endc\n.end\n"
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
    vout, vmax, vmin, iin = grab("m_vout"), grab("m_vmax"), grab("m_vmin"), grab("m_iin")
    if vout is None or iin is None or vin is None:
        return None
    pin = abs(vin * iin)
    # Sum the output power from each load's measured average terminal voltage (no in-circuit probe).
    # R: V^2/R, I: V*I, P: constant. avg(V)^2 ~= avg(V^2) at the low-ripple regulated point we report.
    pout = 0.0
    have_pout = bool(loads)
    for i, (_node, _ref, kind, val) in enumerate(loads):
        vld = grab(f"m_vld{i}")
        if vld is None:
            have_pout = False
            break
        if kind == "R":
            pout += vld * vld / val
        elif kind == "I":
            pout += abs(vld * val)
        elif kind == "P":
            pout += val
    pout = pout if have_pout else float("nan")
    # relative peak-to-peak of Vout over the last ~100 periods: just switching ripple when settled, large when
    # still drifting / in a slow limit cycle. The caller turns this into the steady/not-steady decision so the
    # threshold stays a tunable parameter (not a magic number buried here).
    ripple = 0.0 if (vmax is None or vmin is None or abs(vout) < 1e-9) else (vmax - vmin) / abs(vout)
    return vout, pin, pout, ripple


def _line_frequency(tas):
    """Line (mains) frequency [Hz] from the AC-input designRequirements, or None for a DC-input deck."""
    dr = (tas.get("inputs", {}) or {}).get("designRequirements", {}) or {}
    lf = dr.get("lineFrequency")
    if isinstance(lf, dict):
        for k in ("nominal", "minimum", "maximum"):
            if isinstance(lf.get(k), (int, float)):
                return float(lf[k])
    return float(lf) if isinstance(lf, (int, float)) else None


def _ac_sources(deck):
    """Every AC line source `Vxxx n1 n2 SIN(...)`: one for a single-phase PFC, three for a 3-phase Vienna.
    Returns [(name, n1, n2), ...]."""
    out = []
    for ln in deck.splitlines():
        m = re.match(r"^(V\w+)\s+(\S+)\s+(\S+)\s+SIN\(", ln, re.I)
        if m:
            out.append((m.group(1), m.group(2), m.group(3)))
    return out


def _output_loads(deck):
    """Output load resistor(s) of a self-regulating converter — resistors whose name contains 'load' (the KH
    builders + the top-level assembler name the output load 'Rload'). Returns [(n1, n2, R), ...]. Measured
    DIFFERENTIALLY, so a single-ended output (PFC: `Rload Vout 0`) AND a split DC bus (Vienna: `RRload busP
    busN`) both work without hardcoding a `Vout` node."""
    out = []
    for ln in deck.splitlines():
        m = re.match(r"^(R\w*[Ll]oad\w*)\s+(\S+)\s+(\S+)\s+([-\d.eE+]+)", ln)
        if m:
            out.append((m.group(2), m.group(3), float(m.group(4))))
    return out


def _vdiff(n1, n2):
    """ngspice voltage across (n1, n2): single-ended to ground, else the node-pair difference."""
    return f"v({n1})" if n2 in _GND else f"(v({n1})-v({n2}))"


def _simulate_self_regulated(tas, fidelity, line_freq, tag):
    """Operating point of a SELF-REGULATING AC-input converter (PFC / Vienna). Its closed-loop controller is
    rendered INTO the deck, so there is no control variable to bisect — run the deck over whole LINE cycles
    (the bus is precharged + UIC, so it settles in a handful) and measure the regulated bus, the REAL input
    power (Σ over phases of v_line·i_line — balanced 3-phase is ~ripple-free, single-phase carries the 2×line
    ripple), the output power and the power factor. Returns (vout, pin, pout, ripple, pf) at steady state, or
    None on non-convergence. Output is read DIFFERENTIALLY across the load (handles Vienna's split bus). Uses
    LINE-cycle windowing (the DC path's switching-period window would mis-average the line ripple)."""
    deck = PyKirchhoff.tas_to_ngspice(tas, fidelity)
    sources = _ac_sources(deck)
    loads = _output_loads(deck)
    if not sources or not loads:
        return None
    period = 1.0 / line_freq
    mtran = re.match(r"(?s).*?\.tran\s+(\S+)\s+(\S+)\s+\S+\s+\S+", deck)
    tstep = float(mtran.group(1)) if mtran else period / 4000.0
    # Settle: the 2×-line bus ripple + the (slow) voltage loop, but the precharge means a few line cycles
    # suffice; never run fewer cycles than the deck's own designed stopTime asked for.
    settle = max(6.0 * period, float(mtran.group(2)) if mtran else 0.0)
    deck = re.sub(r"\.tran\s+\S+\s+\S+\s+\S+\s+\S+(\s+uic)?",
                  f".tran {tstep:.12g} {settle:.12g} 0 {tstep:.12g} uic", deck)   # AC decks start from precharge
    cpos = deck.rfind("\n.control")
    if cpos != -1:
        deck = deck[:cpos]
    frm, to = settle - period, settle               # the last WHOLE line cycle (averages the line ripple)
    w0 = settle - 2.0 * period
    pin_terms = " + ".join(f"{_vdiff(n1, n2)}*(-i({nm}))" for nm, n1, n2 in sources)
    # `meas avg/rms/...` needs a VECTOR, not a parenthesised expression — so materialise every differential
    # (the split-bus output, each phase voltage, the input power) with a `let` first, then meas the vector.
    lets = [f"let m_pinst = {pin_terms}", f"let m_vbus = {_vdiff(loads[0][0], loads[0][1])}"]
    lets += [f"let m_vsrc{i} = {_vdiff(n1, n2)}" for i, (_nm, n1, n2) in enumerate(sources)]
    lets += [f"let m_vldv{i} = {_vdiff(n1, n2)}" for i, (n1, n2, _R) in enumerate(loads)]
    ctrl = ["run", *lets,
            f"meas tran m_pin avg m_pinst from={frm:.12g} to={to:.12g}",
            f"meas tran m_vout avg m_vbus from={frm:.12g} to={to:.12g}",
            f"meas tran m_vmax max m_vbus from={w0:.12g} to={to:.12g}",
            f"meas tran m_vmin min m_vbus from={w0:.12g} to={to:.12g}"]
    for i, (nm, _n1, _n2) in enumerate(sources):
        ctrl.append(f"meas tran m_vr{i} rms m_vsrc{i} from={frm:.12g} to={to:.12g}")
        ctrl.append(f"meas tran m_ir{i} rms i({nm}) from={frm:.12g} to={to:.12g}")
    for i in range(len(loads)):
        ctrl.append(f"meas tran m_vld{i} avg m_vldv{i} from={frm:.12g} to={to:.12g}")
    deck += "\n.control\n" + "\n".join(ctrl) + "\n.endc\n.end\n"
    with tempfile.NamedTemporaryFile("w", suffix=f"_{tag}.cir", delete=False) as f:
        f.write(deck)
        path = f.name
    try:
        out = subprocess.run(["ngspice", "-b", path], capture_output=True, text=True, timeout=600)
        text = out.stdout + out.stderr
    finally:
        os.unlink(path)
    if "Timestep too small" in text or "simulation(s) aborted" in text:
        return None

    def grab(name):
        m = re.search(rf"\b{name}\s*=\s*([-\d.eE+]+)", text)
        return float(m.group(1)) if m else None
    vout, vmax, vmin = grab("m_vout"), grab("m_vmax"), grab("m_vmin")
    pin = grab("m_pin")
    if vout is None or pin is None:
        return None
    pin = abs(pin)
    pout = 0.0
    have_pout = bool(loads)
    for i, (_n1, _n2, R) in enumerate(loads):
        vld = grab(f"m_vld{i}")
        if vld is None:
            have_pout = False
            break
        pout += vld * vld / R
    pout = pout if have_pout else float("nan")
    app = 0.0                                        # apparent power Σ vrms·irms over the phases
    have_app = True
    for i in range(len(sources)):
        vr, ir = grab(f"m_vr{i}"), grab(f"m_ir{i}")
        if vr is None or ir is None:
            have_app = False
            break
        app += abs(vr * ir)
    ripple = 0.0 if (vmax is None or vmin is None or abs(vout) < 1e-9) else (vmax - vmin) / abs(vout)
    pf = (pin / app) if (have_app and app) else float("nan")
    return vout, pin, pout, ripple, pf


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
        # Architecture present + model available: the MODEL is authoritative. A failure here is a real error
        # (malformed architecture / ABI break) to SURFACE — not something to silently downgrade to the
        # datasheet rating (that would mask exactly the bug the house rule says to throw on).
        return abs(float(_PYOM.calculate_saturation_current(magnetic, temperature))), "model"
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
    if ctrl is None and topology not in _SELF_REGULATED:
        raise ValueError(f"no control-variable mapping for topology '{topology}'")
    if topology not in _SELF_REGULATED:
        field, bracket = _CONTROL[ctrl]
    if topology in ("fsbb", "ahb", "buck", "boost"):
        _stash_fsbb_modulation(tas)   # capture dead-time before the bisection overwrites duties (complementary;
                                       # buck/boost = synchronous-rectifier variant, no-op for the diode default)
    elif topology == "pshb":
        _stash_pshb_modulation(tas)   # capture the NPC outer pair before the bisection overwrites duties (abt #66)
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
    # SELF-REGULATING AC-input topologies (PFC / Vienna): no control variable — the closed-loop controller
    # is in the deck. Run it ONCE over whole line cycles and report the regulated operating point.
    if topology in _SELF_REGULATED:
        line_freq = _line_frequency(tas)
        if line_freq is None:
            raise ValueError(f"self-regulated topology '{topology}' needs designRequirements.lineFrequency")
        r = _simulate_self_regulated(tas, fidelity, line_freq, topology)
        tmag = abs(float(target_vout)) or 1.0
        if r is None:
            return {"converged": False, "regulated": False, "steady_state": False, "topology": topology,
                    "control": "self", "target_vout": target_vout}
        vout, pin, pout, ripple, pf = r
        # Self-regulating deck: no control to set — attach the simulated per-component excitation as-is (abt #35).
        component_excitations = attach_simulated_excitations(tas, fidelity, f"{topology}_attach")
        return {
            "converged": True,
            "regulated": abs(abs(vout) - tmag) <= tol * tmag,
            "steady_state": ripple <= steady_ripple_frac,
            "topology": topology, "control": "self", "value": None,
            "vout": vout, "target_vout": target_vout, "vout_error": (abs(vout) - tmag) / tmag,
            "pin": pin, "pout": pout,
            "efficiency": (pout / pin) if (pin and pout == pout) else float("nan"),
            "power_factor": pf,
            "component_excitations": component_excitations,
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
    # Regulate on the output MAGNITUDE so INVERTING topologies (cuk, inverting buck-boost) work:
    # their Vout is negative, so bisecting signed Vout toward a positive target drives the control to
    # zero (chasing an unreachable +target). |Vout| is monotonic in the control for both polarities,
    # so we hit |Vout| = |target| and report the SIGNED Vout (the gate sees the true negative rail).
    tmag = abs(float(target_vout)) or 1.0
    # Seed grid. For a FREQUENCY-controlled resonant deck the gain is NON-monotonic: it PEAKS just
    # below the design fsw (~0.85-0.95·fr) and falls monotonically above. A uniform grid over
    # [0.8,2.2]·fsw spends most of its points on the flat falling tail and samples the sub-resonance
    # BOOST region only twice (0.8, 0.975·fsw), MISSING the peak where the extra gain that covers
    # real losses lives (abt #62: e.g. cllc's 0.90·fr boost branch reaches 12.4 V while the regulator
    # would otherwise stall at 0.975·fr ≈ 11.9 V). Concentrate the grid below ~1.1·fsw — fine enough
    # to land on and straddle the peak — and cover the falling tail sparsely. Other controls
    # (duty/phase) are monotonic, so a uniform grid is fine for them.
    if ctrl == "frequency":
        dense_hi = min(hi, 1.1 * fsw)
        grid = [lo + (dense_hi - lo) * k / 6 for k in range(7)]               # 7 pts across [lo, 1.1·fsw]
        grid += [dense_hi + (hi - dense_hi) * (k + 1) / 4 for k in range(4)]  # 4 pts across (1.1·fsw, hi]
    else:
        grid = [lo + (hi - lo) * k / 8 for k in range(9)]                     # uniform 9-pt grid
    N = len(grid)
    pts = [p for p in (sample(x) for x in grid) if p is not None]
    if not pts:
        return {"converged": False, "regulated": False, "control": ctrl, "topology": topology,
                "target_vout": target_vout}

    for _ in range(max_iter - N):
        pts.sort(key=lambda p: p[0])
        cur = min(pts, key=lambda p: abs(abs(p[1]) - tmag))
        if abs(abs(cur[1]) - tmag) <= tol * tmag:
            break
        nxt = None                                          # interpolate within a straddling pair (in |Vout|)
        for a, b in zip(pts, pts[1:]):
            if (abs(a[1]) - tmag) * (abs(b[1]) - tmag) < 0 and a[1] != b[1]:
                nxt = a[0] + (b[0] - a[0]) * (tmag - abs(a[1])) / (abs(b[1]) - abs(a[1]))
                break
        if nxt is None:                                     # no straddle yet: step outward past the closest end
            lo_p, hi_p = pts[0], pts[-1]
            slope_end = hi_p if abs(abs(hi_p[1]) - tmag) < abs(abs(lo_p[1]) - tmag) else lo_p
            step = 0.12 * (hi - lo)
            nxt = slope_end[0] + (step if slope_end is hi_p else -step)
        nxt = min(max(nxt, 0.5 * lo), 1.5 * hi)
        if any(abs(nxt - p[0]) < 1e-9 * max(1.0, abs(nxt)) for p in pts):
            break                                           # converged on the grid (no new point to try)
        r = sample(nxt)
        if r is not None:
            pts.append(r)

    # Pick the reported operating point. By default the closest-to-target sample. But a FREQUENCY-controlled
    # resonant gain curve PEAKS at fr: two frequencies give the same Vout, and the far-below-resonance branch
    # carries large circulating current (poor efficiency). So among the samples that ALREADY meet target
    # within tol, prefer the MOST EFFICIENT one (highest pout/pin) — that is the near-/above-resonance branch.
    # (Other controls are monotonic, so their on-target set collapses to one point; leave them unchanged.)
    on_target = [p for p in pts if abs(abs(p[1]) - tmag) <= tol * tmag]
    if ctrl == "frequency" and on_target:
        def _eff(p):
            _x, _v, _pin, _pout, _r = p
            return (_pout / _pin) if (_pin and _pout == _pout and _pout > 0) else -1.0
        bx, vout, pin, pout, ripple = max(on_target, key=_eff)
    else:
        bx, vout, pin, pout, ripple = min(pts, key=lambda p: abs(abs(p[1]) - tmag))
    steady = ripple <= steady_ripple_frac
    # Attach the SIMULATED per-component excitation at the regulated operating point (control set to bx), so the
    # realism gate + BOM read REAL per-component stresses, not just the magnetic's design excitation (abt #35).
    _set_control(tas, field, bx, topology)
    component_excitations = attach_simulated_excitations(tas, fidelity, f"{topology}_attach")
    return {
        "converged": True,
        # Regulated on |Vout| so inverting topologies count; report the SIGNED Vout below.
        "regulated": abs(abs(vout) - tmag) <= tol * tmag,   # did we actually reach target magnitude?
        "steady_state": bool(steady),            # False -> Vout still drifting/oscillating; treat eff with care
        "topology": topology,
        "control": ctrl,
        "value": bx,                             # the regulated duty / phaseDeg / frequency
        "vout": vout,
        "target_vout": target_vout,
        "vout_error": (abs(vout) - tmag) / tmag,
        "pin": pin,
        "pout": pout,
        "efficiency": (pout / pin) if (pin and pout == pout) else float("nan"),
        "component_excitations": component_excitations,
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
