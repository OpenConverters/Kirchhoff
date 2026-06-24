#pragma once

// Kirchhoff::TwoSwitchForward — two-switch forward (isolated). Two switches (high + low side, driven
// together) put Vin across the transformer primary during ON; two clamp diodes reset the core during
// OFF (no separate demag winding). Secondary feeds a buck-like output stage. Port of MKF TwoSwitchForward.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct TwoSwitchForwardDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double turnsRatio;             // n = primary:secondary
    double dutyCycle;              // operating duty at nominal Vin
    double magnetizingInductance;
    double outputInductance;
    double loadResistance;
    double outputCapacitance;
    nlohmann::json config;
};

/**
 * @brief Design a two-switch forward converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
TwoSwitchForwardDesign design_two_switch_forward(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a two-switch forward converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_two_switch_forward_tas(const TwoSwitchForwardDesign& d);

} // namespace Kirchhoff
