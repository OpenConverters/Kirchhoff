#pragma once

// Kirchhoff::IsolatedBuck — Isolated Buck / "Flybuck" (16th topology). A synchronous buck whose filter
// inductor is a COUPLED inductor (transformer): the primary winding is the buck inductor and produces
// the regulated, non-isolated primary rail V_pri = D·Vin; each secondary winding rectifies the
// mirror-imaged voltage during the switch-OFF interval, giving a loosely cross-regulated ISOLATED
// output V_sec = V_pri/N − Vd (cheap isolated bias rails). Port of MKF IsolatedBuck.
//
// Two switches: a high-side buck switch S1 (vin->sw) and a low-side synchronous rectifier S2 (sw->gnd)
// driven COMPLEMENTARY to S1. The coupled inductor's secondary feeds a flyback-style rectifier diode +
// output cap. This port compares the PRIMARY buck rail (the harness's output[0]); the isolated
// secondary is present as an internal loaded rail (it loads the coupled inductor exactly as in MKF).
//
// New piece vs the plain Buck: the coupled inductor (2-winding magnetic with the buck inductor as the
// primary) + a flyback secondary rectifier, and the complementary synchronous low-side switch.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

// One isolated secondary rail (multi-output flybuck, ABT #86). Each rail is its OWN coupled secondary
// winding (turnsRatio N_k = Np/Ns_k), flyback rectifier + output cap. secondaries[0] reproduces the scalar
// secondary* fields byte-for-byte; extra entries add further coupled secondary windings.
struct IsolatedBuckSecondaryLeg {
    double voltage, power, turnsRatio, loadResistance, capacitance;
};

struct IsolatedBuckDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double primaryVoltage, primaryPower;      // primary buck rail (output[0]) — the compared output
    double secondaryVoltage, secondaryPower;  // FIRST isolated secondary rail (output[1]) — == secondaries[0]
    double switchingFrequency, efficiency;
    double dutyCycle;                 // D = V_pri/(Vin·η)
    double turnsRatio;                // N = V_pri/V_sec (first secondary; == secondaries[0].turnsRatio)
    double magnetizingInductance;     // Lmag (the coupled buck inductor)
    double loadResistance;            // primary load (synthesized at the output port)
    double secondaryLoadResistance;   // first-secondary load (== secondaries[0].loadResistance)
    double outputCapacitance;
    nlohmann::json config;         // primary Cpri (sets the settling RC)
    double secondaryCapacitance;      // first-secondary Cout (== secondaries[0].capacitance)
    // Every declared isolated secondary (output[1..]). Size == outputs.size()-1 (>=1). With exactly one
    // entry the deck keeps the legacy single-secondary-INTERNAL topology byte-for-byte; with more, every
    // secondary is exposed on its own external vout<i> port (the assembler synthesizes each rail's load).
    std::vector<IsolatedBuckSecondaryLeg> secondaries;
};

/**
 * @brief Design an isolated-buck (Flybuck) converter.
 * @param tasInputs Spec with TWO OR MORE outputs: designRequirements.outputs[0]=primary buck rail,
 *        outputs[1..]=isolated secondaries; operatingPoints[0].outputs[k].power give the loads.
 * @return A design struct (duty, turns ratio, coupled-inductor magnetizing inductance, both loads).
 */
IsolatedBuckDesign design_isolated_buck(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an isolated-buck design into a full TAS topology document.
 * @param d A design returned by design_isolated_buck().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_isolated_buck_tas(const IsolatedBuckDesign& d);

} // namespace Kirchhoff
