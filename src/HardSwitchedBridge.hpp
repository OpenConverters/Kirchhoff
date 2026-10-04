#pragma once

// Kirchhoff::HardSwitchedBridge — the symmetric (hard-switched) PWM half bridge (ABT #1434) and full bridge
// (ABT #1435). The bridge applies +Vb to the transformer primary for D·T, nothing for (½−D)·T, −Vb for D·T
// and nothing again (per-switch duty D < ½); Vb = Vin/2 for the split-cap half bridge, Vin for the full
// bridge. The secondary is rectified by a centre-tapped (default), full-bridge or current-doubler rectifier
// (Rectifier.hpp) into an LC output filter. No MKF model exists: the design is validated against the exact
// piecewise-linear solver analytical_hard_switched_bridge and against ngspice.
//
// Half bridge: two series capacitors split the bus and return the primary to the midpoint, so they also block
// any DC in the primary (no flux walking from a duty mismatch: the midpoint shifts instead). Each carries a
// balancing resistor that pins the midpoint at Vin/2 against capacitor leakage mismatch.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"
#include "Rectifier.hpp"

namespace Kirchhoff {

struct HardSwitchedBridgeDesign {
    bool fullBridge = false;
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double diodeDrop;              // per diode, at the operating rectifier current
    double maxDutyCycle;           // per-switch duty at Vin_min (< 0.5; the rest of each half period is dead)
    double dutyCycle;              // per-switch duty at the operating point
    double bridgeVoltage;          // Vb at the operating point (Vin/2 or Vin)
    double turnsRatio;             // n = Np : Ns (CT: primary : one half-winding)
    double magnetizingInductance;  // Lm, a minimum: an energy-transfer transformer
    double outputInductance;       // Lo (each of the two for the current doubler)
    double splitCapacitance;       // half bridge: each of the two bus-splitting capacitors
    double balancingResistance;    // half bridge: the resistor across each split capacitor
    double loadResistance;
    double outputCapacitance;
    RectifierType rectifierType = RectifierType::CenterTapped;
    nlohmann::json config;
};

/**
 * @brief Design a hard-switched (symmetric PWM) half bridge.
 *
 * config: rectifierType (centerTapped | fullBridge | currentDoubler), maxDutyCycle (0.45),
 * inductorRippleRatio (0.3), magnetizingCurrentFraction (0.2: n·ΔIm/2 over the per-inductor current),
 * splitCapacitorRipple (0.02 of Vin/2). Pinned turnsRatios[0] / magnetizingInductance are honoured.
 * Throws when the pinned values cannot regulate at Vin_min or would let a rectifier diode drop out.
 */
HardSwitchedBridgeDesign design_half_bridge(const nlohmann::json& tasInputs);
nlohmann::json build_half_bridge_tas(const HardSwitchedBridgeDesign& d);

/** @brief Design a hard-switched (symmetric PWM) full bridge. Same config as design_half_bridge. */
HardSwitchedBridgeDesign design_full_bridge(const nlohmann::json& tasInputs);
nlohmann::json build_full_bridge_tas(const HardSwitchedBridgeDesign& d);

} // namespace Kirchhoff
