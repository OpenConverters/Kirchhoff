#pragma once

// Kirchhoff::Zeta — non-isolated single-switch Zeta (step up/down, non-inverting, HIGH-SIDE switch).
//   Vin -> S1 -> node_SW;  L1: node_SW -> gnd;  Cc: node_SW <-> node_X;  D1: gnd -> node_X (catch);
//   L2: node_X -> Vout;  Cout: Vout -> gnd.  Port of MKF Zeta. M(D) = D/(1-D).

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct ZetaDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double dutyCycle;            // D = (Vo+Vd)/(Vin*eff + Vo + Vd)
    double inductanceL1;         // magnetizing inductor (node_SW -> gnd)
    double inductanceL2;         // output inductor (node_X -> Vout)
    double couplingCapacitance;  // Cc (F), holds ~Vo
    double loadResistance;
    double outputCapacitance;
    bool synchronousRectifier = false;   // config.rectifier=="synchronous": swap catch diode D1 for a MOSFET
    double deadFraction = 0.01;
    bool coupledInductor = false;        // config.coupledInductor: L1+L2 on one 1:1 coupled core (ABT #89)
    double couplingCoefficient = 0.999;
    nlohmann::json config;
};

/**
 * @brief Design a Zeta converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
ZetaDesign design_zeta(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a Zeta converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_zeta_tas(const ZetaDesign& d);

} // namespace Kirchhoff
