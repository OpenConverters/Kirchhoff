#pragma once

// Kirchhoff::Buck — non-isolated step-down: Vin -> high-side switch -> sw node -> L -> Vout, with a
// freewheeling diode (sw->GND) and output filter. Vout = Vin*D. Faithful port of MKF Buck; reuses the
// family to_cias generators + the generic Kirchhoff::tas_to_ngspice assembler.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct BuckDesign {
    double inputVoltage, outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double inputVoltageMin, inputVoltageMax;   // design corners
    double dutyCycle;       // D = (Vout+Vd)/((Vin+Vd)*eff)  (MKF Buck::calculate_duty_cycle)
    double inductance;      // buck inductor (H)
    double loadResistance;  // Vout^2/Pout
    double outputCapacitance;
    nlohmann::json config;
};

/**
 * @brief Design a buck converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
BuckDesign design_buck(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a buck converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_buck_tas(const BuckDesign& d);

} // namespace Kirchhoff
