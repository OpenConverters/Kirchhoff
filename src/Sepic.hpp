#pragma once

// Kirchhoff::Sepic — non-isolated single-switch SEPIC (step up or down, non-inverting).
//   Vin -> L1 -> nodeA;  S1: nodeA -> gnd;  Cs: nodeA -> nodeB;  L2: gnd -> nodeB;  D1: nodeB -> Vout.
// Two SEPARATE inductors + a series coupling capacitor. Port of MKF Sepic. M(D) = D/(1-D).

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct SepicDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double dutyCycle;            // D = (Vout+Vd)/(Vin*eff + Vout + Vd)  (operating, nominal Vin)
    double inductanceL1;         // input inductor (H)
    double inductanceL2;         // second inductor (H)
    double couplingCapacitance;  // Cs (F)
    double loadResistance;
    double outputCapacitance;
    bool synchronousRectifier = false;   // config.rectifier=="synchronous": swap D1 for a low-side sync MOSFET
    double deadFraction = 0.01;          // dead-time fraction of the period for the complementary drive
    nlohmann::json config;
};

/**
 * @brief Design a SEPIC converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
SepicDesign design_sepic(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a SEPIC converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_sepic_tas(const SepicDesign& d);

} // namespace Kirchhoff
