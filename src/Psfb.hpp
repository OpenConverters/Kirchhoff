#pragma once

// Kirchhoff::Psfb — Phase-Shifted Full Bridge (isolated, 10th topology, first phase-shift-modulated
// bridge). Two legs of two switches each (QA/QB leading, QC/QD lagging) driven at ~50% duty; the
// lagging leg is phase-shifted relative to the leading leg, and that phase shift sets the effective
// duty Deff = phi/180 (and hence the power transfer). A series resonant/leakage inductor Lr sits
// between the bridge diagonal and the transformer primary. Secondary is rectified by a FULL-BRIDGE
// rectifier (4 diodes) feeding a buck-like output (Lout + Cout). Port of MKF Psfb.
//
// Control variable is the leg-to-leg PHASE (not duty) — the new piece vs push-pull. Uses the
// assembler's phaseDeg stimulus support + anti-parallel body diodes + snubber caps for the ideal
// bridge's dead-time convergence (the bridge template).
//
// FULL_BRIDGE rectifier is used (not center-tapped): MKF's center-tapped PSFB deck is a fake CT
// (one full winding with the "center tap" pinned at one end) -> effectively half-wave -> delivers
// ~half the designed Vout. The full-bridge rectifier deck is correct. (Surfaced to the user.)

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct PsfbDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double commandedDuty;          // D_cmd = phi/180 commanded effective duty (0.7)
    double phaseDeg;               // lagging-leg phase shift in degrees (= 180*D_cmd, tuned)
    double switchDuty;             // per-switch on-fraction (~0.5 minus dead time)
    double effectiveDuty;          // Deff = D_cmd - duty-cycle loss (design point)
    double turnsRatio;             // n = Np:Ns (full secondary)
    double seriesInductance;       // Lr (leakage + external resonant)
    double magnetizingInductance;  // Lm
    double outputInductance;       // Lo
    double loadResistance;
    double outputCapacitance;
    nlohmann::json config;         // tasInputs["config"] — user overrides for otherwise-derived values
};

/**
 * @brief Design a phase-shifted full-bridge converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
PsfbDesign design_psfb(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a phase-shifted full-bridge converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_psfb_tas(const PsfbDesign& d);

} // namespace Kirchhoff
