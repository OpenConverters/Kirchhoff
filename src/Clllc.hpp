#pragma once

// Kirchhoff::Clllc — CLLLC bidirectional symmetric resonant converter (the full five-element symmetric
// tank Cr1+Lr1+Lm+Lr2+Cr2), demonstrating CLOSED-LOOP synchronous rectification expressed entirely in
// CIAS. This is the first port whose secondary rectifier is CURRENT-AWARE rather than open-loop: the
// four SR switches are driven by a control stage that senses the secondary tank-current direction and
// gates the rectifying diagonal, so the SR follows the body-diode (true rectifier) behaviour instead of
// imposing Vin/n.
//
// Architecture (the point of this port):
//   • POWER stage  — HV full bridge (open-loop PWM) + Cr1/Lr1 + transformer + Lr2/Cr2 + a small
//                    current-sense resistor + the four SR switches (with body diodes) + Cout. It
//                    EXPOSES the four SR gates and the two sense nodes as ports.
//   • CONTROL stage — a separate, SWAPPABLE CIAS brick: two AAS comparators read the sense nodes and
//                    drive the two SR diagonals. Wired to the power stage by interStageConnections.
// Both stages are CIAS bricks (CIAS carries the control too), so the whole thing stays portable to any
// circuit simulator. Swap the control stage for a different one (open-loop PWM, a different SR scheme)
// without touching the power topology.
//
// We diverge from MKF here BY DESIGN: a clean ideal SR rectifies the tank (Vout follows the resonant
// gain, ~300 V on this operating point) rather than reproducing MKF's specific solver-timed SR (187 V).

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct ClllcDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double turnsRatio;                 // n = HV / LV bus
    double primaryResonantInductance;  // Lr1
    double primaryResonantCapacitance; // Cr1
    double magnetizingInductance;      // Lm = k·Lr1
    double secondaryResonantInductance;  // Lr2 = Lr1/n² (symmetric)
    double secondaryResonantCapacitance; // Cr2 = n²·Cr1 (symmetric)
    double resonantFrequency;          // fr (= fsw)
    double switchDuty;                 // primary per-switch on-fraction
    double loadResistance;
    double outputCapacitance;
    bool reverse;                      // config.powerFlowDirection == "reverse": Vout side sources, HV
                                       // (Vin) side receives — bidirectional CLLLC (V2G / OBC), ABT #85.
    nlohmann::json config;
};

/** Design a symmetric CLLLC (full bridge primary, current-aware synchronous-rectifier secondary). */
ClllcDesign design_clllc(const nlohmann::json& tasInputs);
/** Assemble a CLLLC design into a TWO-stage TAS document: a power stage + a swappable SR control stage. */
nlohmann::json build_clllc_tas(const ClllcDesign& d);

} // namespace Kirchhoff
