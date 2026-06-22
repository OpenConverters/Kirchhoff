#pragma once

// Kirchhoff::Cllc — CLLC bidirectional resonant converter (21st topology). Like the LLC but RESONANT on
// BOTH sides: a primary series tank (Cr1 + Lr1) and a secondary series tank (Lr2 + Cr2) flank the
// transformer magnetizing Lm. Both bridges are ACTIVE — a primary full bridge plus a secondary active
// synchronous rectifier (8 switches total, DAB-like), driven at the same frequency for forward power
// flow. Symmetric design (Lr2 = Lr1/n², Cr2 = n²·Cr1). Port of MKF Cllc (Infineon AN methodology).
//
// New piece vs LLC/SRC: real active switches on BOTH sides (no ideal-source abstraction, no rectifier
// diodes) AND a second resonant tank. Because the secondary uses an active synchronous rectifier, the
// converter CANNOT cold-start into a 0 V output (the gated SR shorts the secondary to the zero rail),
// and the series resonant caps make the DC operating point singular — so the deck needs initial
// conditions: it precharges the output node and runs the transient with use-initial-conditions (UIC).
// This is expressed via the TAS simulation.initialConditions field (the assembler realises it as
// .ic + uic). Since the bridges are real switches with RON pinned to MKF's, the resonant family's
// ideal-source caveat does NOT apply here — CLLC matches within the tighter 2% band.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct CllcDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double turnsRatio;                 // n = Vin_nom / Vout
    double primaryResonantInductance;  // Lr1
    double primaryResonantCapacitance; // Cr1
    double magnetizingInductance;      // Lm = k·Lr1
    double secondaryResonantInductance;  // Lr2 = Lr1/n² (symmetric)
    double secondaryResonantCapacitance; // Cr2 = n²·Cr1 (symmetric)
    double resonantFrequency;          // fr (= fsw: operated at resonance)
    double switchDuty;                 // per-switch on-fraction (~0.47, complementary with dead time)
    double loadResistance;
    double outputCapacitance;
};

/**
 * @brief Design a symmetric CLLC bidirectional resonant converter (full bridge both sides).
 * @param tasInputs Single-output spec (designRequirements + operatingPoints[0]). Quality factor 0.3 and
 *        the k = Lm/Lr1 = 4.45 inductance ratio match MKF's Cllc defaults; the converter is
 *        designed/operated at resonance (fr = the operating-point switching frequency).
 * @return A design struct (turns ratio, both tanks Lr1/Cr1/Lr2/Cr2, Lm, load, output cap).
 */
CllcDesign design_cllc(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a CLLC design into a full TAS topology document (with simulation.initialConditions).
 * @param d A design returned by design_cllc().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_cllc_tas(const CllcDesign& d);

} // namespace Kirchhoff
