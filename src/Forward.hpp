#pragma once

// Kirchhoff::Forward — single-switch forward (isolated). Vin -> switch -> transformer primary; a 1:1
// demagnetization (reset) winding + demag diode resets the core each cycle; the secondary feeds a
// buck-like output stage (forward diode + freewheel diode + output inductor + cap). Faithful port of
// MKF SingleSwitchForward. The 3-winding transformer is ONE magnetic component (turnsRatios = [1, n]).

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct ForwardDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double turnsRatio;             // n = primary:secondary
    double dutyCycle;              // operating duty at nominal Vin = n*(Vout+Vd)/Vin
    double magnetizingInductance;  // Lm (primary; demag is 1:1)
    double outputInductance;       // Lout (the buck-like output filter inductor)
    double loadResistance;
    double outputCapacitance;
    nlohmann::json config;
};

/**
 * @brief Design a single-switch forward converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
ForwardDesign design_forward(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a single-switch forward converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_forward_tas(const ForwardDesign& d);

} // namespace Kirchhoff
