#pragma once

// Kirchhoff::PushPull — center-tapped push-pull (isolated). Center-tapped primary (two half-windings
// from the Vin center tap), two LOW-SIDE switches driven 180 deg apart, center-tapped secondary with
// two rectifier diodes (full-wave) feeding a buck-like output (Lout + Cout). Port of MKF PushPull.
// ONE 4-winding magnetic (Lpri_top, Lpri_bot, Lsec_top, Lsec_bot), turnsRatios = [1, N, N].

#include <nlohmann/json.hpp>
#include <vector>
#include "Fidelity.hpp"

namespace Kirchhoff {

// outputs[1..] are additional isolated secondaries (multi-output push-pull, ABT #86). Each rail is its
// OWN center-tapped secondary pair (turnsRatio N_i = primary-half:secondary-half), rectifier + Lout_i/Cout_i.
struct PushPullOutputLeg {
    double voltage, power, turnsRatio, diodeDrop, outputInductance, outputCapacitance, loadResistance;
};

struct PushPullDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency, diodeDrop;
    double maxDutyCycle;            // per-switch max duty (D < 0.5); MKF default 0.48
    double turnsRatio;             // N = primary-half : secondary-half (main output)
    double dutyCycle;              // operating per-switch duty at nominal Vin
    double magnetizingInductance;  // Lm per half-winding
    double outputInductance;       // Lout (main output)
    double loadResistance;
    double outputCapacitance;
    std::vector<PushPullOutputLeg> outputs;   // >=1 entry; [0] duplicates the scalars above
    nlohmann::json config;
};

/**
 * @brief Design a push-pull converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
PushPullDesign design_push_pull(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a push-pull converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_push_pull_tas(const PushPullDesign& d);

} // namespace Kirchhoff
