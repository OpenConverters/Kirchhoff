#pragma once

// Kirchhoff::Boost — a second topology, to prove the design + generic-assembler path generalizes beyond
// the flyback. Non-isolated boost: Vin -> L -> SW; low-side switch SW->GND; diode SW->Vout; Cout/Rload.
// Vout = Vin/(1-D). Reuses the family to_cias generators + the generic Kirchhoff::tas_to_ngspice assembler.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct BoostDesign {
    double inputVoltage, outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double inputVoltageMin, inputVoltageMax;   // design corners (min V: max current; max V: max stress)
    double dutyCycle;       // D = 1 - Vin/(Vout+Vd)
    double inductance;      // boost inductor (H)
    double loadResistance;  // Vout^2/Pout
    double outputCapacitance;
};

/**
 * @brief Design a boost converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
BoostDesign design_boost(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a boost converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_boost_tas(const BoostDesign& d);

} // namespace Kirchhoff
