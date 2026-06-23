#!/usr/bin/env python3
"""Independent averaged-model oracle for the AC-input topologies (PFC, Vienna).

This is a SECOND SOURCE OF TRUTH for the closed-loop operating point, deliberately built to be
independent of ngspice: a different language (Python), a different solver (an explicit fixed-step
large-signal integrator), and a different model class (cycle-AVERAGED state equations, not a switched
netlist). The DC topologies in Kirchhoff are graded against MKF; the AC topologies had no external
oracle and graded their own expected numbers. This closes that gap — the C++ capability tests feed the
design parameters here, integrate the averaged plant + the SAME designed control law, and assert that
ngspice's switched simulation matches this independent prediction (bus trajectory, drawn power, energy
balance). A netlist/wiring/sense-placement bug, or a control law that ngspice realises differently from
the design intent, shows up as a divergence between the two.

The control law is re-derived here from the design spec (kref, kp, ki, kv, ...), NOT copied from the C++
emission — re-implementing it in a second language is the point (it catches transcription errors on
either side). The formulas mirror src/Pfc.cpp / src/Vienna.cpp; keep them in sync.

Usage:  python3 averaged_model.py {pfc|vienna}  < params.json   ->  result.json on stdout
"""
import json
import sys
import math


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def predict_pfc(p):
    """Averaged single-phase boost PFC with the designed PI voltage loop.

    The inner hysteretic current loop is fast relative to the outer loop, so it is modelled as an ideal
    conductance emulator Ge = (1-gv)/Rsense (the cycle-averaged input power is Vrms^2 * Ge). The outer
    PI loop and the bus-cap power balance are integrated explicitly:
        gv   = clamp(iv, ivLo, ivHi) + kp * voutScaled,   voutScaled = kv * Vbus
        Pin  = Vrms^2 * (1 - gv) / Rsense
        dVbus/dt = (Pin - Vbus^2/Rload) / (C * Vbus)
        div/dt   = ki * (voutScaled - vref)         [conditional-integration anti-windup at the rails]
    """
    Vrms, Vtarget = p["inputVoltageRms"], p["outputVoltage"]
    Rsense, C, Rload = p["senseResistance"], p["outputCapacitance"], p["loadResistance"]
    kref, kp, ki, kv = p["referenceGain"], p["proportionalGain"], p["integralGain"], p["outputDividerGain"]
    tstop, t0, t1 = p["tstop"], p["windowStart"], p["windowEnd"]
    vprecharge = p.get("precharge", Vtarget)

    vref = kv * Vtarget
    g0 = 1.0 - kref
    ivInit = g0 - kp * vref
    gvLo, gvHi = 1.0 - 4.0 * kref, 1.0 - 0.1 * kref
    ivLo, ivHi = gvLo - kp * vref, gvHi - kp * vref

    dt = 1.0e-6
    n = int(round(tstop / dt))
    vbus, iv = vprecharge, ivInit
    acc_v = acc_p = 0.0
    cnt = 0
    for k in range(n):
        t = k * dt
        voutScaled = kv * vbus
        iv_c = _clamp(iv, ivLo, ivHi)
        gv = iv_c + kp * voutScaled
        pin = Vrms * Vrms * (1.0 - gv) / Rsense
        dvbus = (pin - vbus * vbus / Rload) / (C * vbus)
        div = ki * (voutScaled - vref)
        # conditional-integration anti-windup: stop integrating further into a saturated rail
        if (iv >= ivHi and div > 0.0) or (iv <= ivLo and div < 0.0):
            div = 0.0
        vbus += dt * dvbus
        iv += dt * div
        if t0 <= t <= t1:
            acc_v += vbus
            acc_p += pin
            cnt += 1
    vbus_avg = acc_v / cnt
    pin_avg = acc_p / cnt
    return {
        "vbus_pred": vbus_avg,
        "pin_pred": pin_avg,
        "pout_pred": vbus_avg * vbus_avg / Rload,   # energy-balance reference (lossless)
        "pf_pred": _ripple_pf(p.get("rippleFraction", 0.30)),
    }


def _ripple_pf(ripple_fraction):
    """Power-factor CEILING set by the hysteretic switching ripple (the raw, unfiltered measurement).

    The line current is the resistor-emulated fundamental (unity displacement) plus a triangular hysteretic
    ripple of fixed band dIL = rippleFraction*iPeak. Fundamental RMS I1 = iPeak/sqrt(2); triangular ripple
    RMS = dIL/(2*sqrt(3)). So Iripple/I1 = rippleFraction/sqrt(6) and
        PF = I1/Irms = 1/sqrt(1 + (rippleFraction/sqrt(6))^2).
    The MEASURED raw PF cannot exceed this ceiling (extra zero-crossing/cusp distortion only lowers it), so
    the tests use this as a model-derived bound instead of a hand-picked PF number.
    """
    r = ripple_fraction / math.sqrt(6.0)
    return 1.0 / math.sqrt(1.0 + r * r)


def predict_vienna(p):
    """Averaged three-phase Vienna rectifier bus.

    There is no outer voltage loop: the per-phase current shaping emulates a conductance sized (via kref)
    to draw the design power, so the cycle-averaged input power is Pin = Pdesign and the full bus (two
    series rail caps -> C/2) follows the power balance dVbus/dt = (Pin - Vbus^2/Rload)/((C/2)*Vbus). The
    independent checks the C++ test leans on are (1) this predicted bus vs ngspice, (2) energy balance
    Pin == Vbus^2/Rload, and (3) the boost bound Vbus > line-to-line peak.
    """
    Vrms, Vtarget = p["inputVoltageRms"], p["outputVoltage"]
    C, Rload, Pdesign = p["busCapacitance"], p["loadResistance"], p["outputPower"]
    tstop, t0, t1 = p["tstop"], p["windowStart"], p["windowEnd"]
    vprecharge = p.get("precharge", Vtarget)

    Ctot = 0.5 * C            # two rail caps in series across the full bus
    dt = 1.0e-6
    n = int(round(tstop / dt))
    vbus = vprecharge
    acc_v = 0.0
    cnt = 0
    for k in range(n):
        t = k * dt
        pin = Pdesign         # current shaping emulates exactly the design conductance
        dvbus = (pin - vbus * vbus / Rload) / (Ctot * vbus)
        vbus += dt * dvbus
        if t0 <= t <= t1:
            acc_v += vbus
            cnt += 1
    vbus_avg = acc_v / cnt
    vll_peak = Vrms * math.sqrt(3.0) * math.sqrt(2.0)
    return {
        "vbus_pred": vbus_avg,
        "pin_pred": Pdesign,
        "pout_pred": vbus_avg * vbus_avg / Rload,
        "vll_peak": vll_peak,
        "pf_pred": _ripple_pf(p.get("rippleFraction", 0.30)),
    }


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("pfc", "vienna"):
        sys.stderr.write("usage: averaged_model.py {pfc|vienna} < params.json\n")
        return 2
    params = json.load(sys.stdin)
    result = predict_pfc(params) if sys.argv[1] == "pfc" else predict_vienna(params)
    json.dump(result, sys.stdout)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
