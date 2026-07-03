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
    bool synchronousRectifier = false;   // config.rectifier=="synchronous": swap catch diode D1 for a MOSFET
    double deadFraction = 0.01;
    bool coupledInductor = false;        // config.coupledInductor: L1+L2 on one 1:1 coupled core (ABT #89)
    double couplingCoefficient = 0.999;
    // Isolated Ćuk (V3, ABT #90): a transformer across the coupling capacitor — the single C1 becomes a
    // PRIMARY coupling cap (C1) + transformer + SECONDARY coupling cap (C1b), and the output is referred
    // through the turns ratio n = Ns/Np. Gives galvanic isolation + a step-up/down beyond the D/(1-D) range.
    bool isolated = false;
    double turnsRatio = 1.0;                  // n = Ns/Np (config.turnsRatio; also honours pinned turnsRatios[0])
    double secondaryCouplingCapacitance = 0;  // C1b (F), on the secondary side
    double magnetizingInductance = 0;         // transformer Lm (H)
    // Bidirectional Ćuk (V5, ABT #90): reverse power flow (config.powerFlowDirection=="reverse") — the Vout
    // side sources and the Vin side receives. Requires the synchronous rectifier so current can flow both
    // ways. Same source/load-swap mechanism as the CLLC bidirectional (ABT #85).
    bool reverse = false;
    nlohmann::json config;
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
