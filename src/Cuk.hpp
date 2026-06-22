#pragma once

// Kirchhoff::Cuk — non-isolated single-switch Cuk (INVERTING: Vout < 0). Capacitive energy transfer.
//   Vin -> L1 -> nodeA;  S1: nodeA -> gnd;  C1: nodeA -> nodeB;  D1: nodeB -> gnd (anode at nodeB);
//   L2: nodeB -> Vout(neg);  Cout: Vout -> gnd.  Port of MKF Cuk (V1 non-isolated). M(D) = -D/(1-D).

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct CukDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltageMag;     // |Vo| (output is negative)
    double outputPower, switchingFrequency, efficiency, diodeDrop;
    double dutyCycle;            // D = (|Vo|+Vd)/(Vin*eff + |Vo| + Vd)
    double inductanceL1;         // input inductor (H)
    double inductanceL2;         // output inductor (H)
    double couplingCapacitance;  // C1 (F), holds ~Vin+|Vo|
    double loadResistance;       // |Vo|^2 / Pout
    double outputCapacitance;
};

/**
 * @brief Design a Cuk converter (inverting output) for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
CukDesign design_cuk(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a Cuk converter (inverting output) design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_cuk_tas(const CukDesign& d);

} // namespace Kirchhoff
