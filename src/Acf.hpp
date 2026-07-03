#pragma once

// Kirchhoff::Acf — Active-Clamp Forward (isolated, 12th topology). A single-switch forward whose
// transformer reset is done by an ACTIVE CLAMP (auxiliary switch + clamp capacitor) instead of a
// third demagnetizing winding. Main switch Q1 (vin->sw) drives a 2-winding transformer; the clamp
// switch Sc (vin->clamp_node) + clamp cap Cc (clamp_node->sw) recycle the magnetizing energy during
// the OFF interval (V_clamp = D*Vin/(1-D)). Secondary is a forward output stage (forward diode +
// freewheel diode + Lout + Cout). Port of MKF ActiveClampForward.
//
// Gain Vo = D*Vin/n (forward), single output diode -> like the forward, no diode compensation needed
// (MKF reference designs with diodeVoltageDrop=0; both decks have the same ideal-ish diode). Clamp
// switch is driven complementary to the main switch (assembler complementary drive).

#include <nlohmann/json.hpp>
#include <vector>
#include "Fidelity.hpp"

namespace Kirchhoff {

// One isolated output rail (multi-output active-clamp forward, ABT #86). outputs[0] mirrors the scalars.
struct AcfOutputLeg {
    double voltage, power, turnsRatio, diodeDrop, outputInductance, outputCapacitance, loadResistance;
};

struct AcfDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double dutyCycle;              // D = main-switch duty (operating point, fixed)
    double deadFraction;          // dead time fraction between main & clamp switches
    double turnsRatio;            // n = Np:Ns (main output)
    double magnetizingInductance; // Lm
    double clampCapacitance;      // Cc
    double outputInductance;      // Lo (main output)
    double loadResistance;
    double outputCapacitance;
    std::vector<AcfOutputLeg> outputs;   // >=1 entry; [0] duplicates the scalars above
    nlohmann::json config;
};

/**
 * @brief Design an active-clamp forward converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
AcfDesign design_acf(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an active-clamp forward converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_acf_tas(const AcfDesign& d);

} // namespace Kirchhoff
