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
#include <string>
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
    double resonantFrequency;          // fr = designRequirements.switchingFrequency: the tank resonance
    double operatingFrequency;         // the frequency both bridges are driven at (the operating point's): fr when
                                       // the turns ratio gives the output there, else the frequency in the switching
                                       // band where the FHA tank gain delivers it (ABT #1503)
    double requiredGain;               // tank gain the delivered rail needs (forward n·Vout/(η·Vin); reverse
                                       // Vin/(η·n·Vout)); an engine-sized ratio has 1/gainHeadroom
    double switchDuty;                 // per-switch on-fraction (~0.47, complementary with dead time)
    double loadResistance;
    double outputCapacitance;
    // Reverse power flow (config.powerFlowDirection == "reverse", ABT #85). CLLC is a bidirectional
    // (dual-active-bridge) resonant converter — its whole purpose (V2G / on-board chargers). Forward: the
    // Vin-side full bridge drives, the Vout-side SR rectifies, power flows Vin->Vout. Reverse: the Vout side
    // sources power and the Vin side receives, power flows Vout->Vin. The tank is symmetric and both bridges
    // are already actively gated, so reverse is the SAME cell with the source/load swapped: the deck sources
    // the LV (Vout) bus and delivers to (and precharges) the HV (Vin) bus.
    bool reverse;
    nlohmann::json config;
};

/**
 * @brief The operating frequency of a two-sided (CLLC / CLLLC) resonant tank: the frequency at which the FHA tank
 *        gain (analytical::cllc_fha_tank) delivers the output with turns ratio n — ABT #1503. The tank resonance
 *        fr is kept when the gain there already meets the requirement; otherwise the frequency is solved inside the
 *        switching band (config minSwitchingFrequency/maxSwitchingFrequency, or their aliases resonantBandMin/
 *        resonantBandMax; an unstated end is bounded by the tank's physics, see solve_cllc_operating_frequency).
 *        Reverse power flow drives the Vout-side tank (Lr2/Cr2) with Lm/n² and delivers Vin through 1/n.
 * @param who "design_cllc" / "design_clllc" (message prefix); ratioOrigin describes n ("pinned", ...).
 * @throws std::invalid_argument naming the required gain, the reachable gain range, and the turns ratio that
 *         would deliver the output at resonance, when no frequency in the band delivers it.
 */
struct TwoSidedOperatingPoint { double operatingFrequency, requiredGain; };
TwoSidedOperatingPoint two_sided_resonant_operating_point(
    const std::string& who, const std::string& ratioOrigin, const nlohmann::json& config, bool reverse,
    double turnsRatio, double efficiency, double inputVoltage, double outputVoltage, double outputPower,
    double resonantFrequency, double magnetizingInductance, double primaryResonantInductance,
    double primaryResonantCapacitance, double secondaryResonantInductance, double secondaryResonantCapacitance);

/**
 * @brief Design a symmetric CLLC bidirectional resonant converter (full bridge both sides).
 * @param tasInputs Single-output spec (designRequirements + operatingPoints[0]). Quality factor 0.3 and
 *        the k = Lm/Lr1 = 4.45 inductance ratio match MKF's Cllc defaults; the tank is designed to resonate at
 *        fr = designRequirements.switchingFrequency and driven at operatingFrequency (above fr for the default
 *        1.08 gain headroom; see two_sided_resonant_operating_point).
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
