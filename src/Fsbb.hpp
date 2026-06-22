#pragma once

// Kirchhoff::Fsbb — Four-Switch Buck-Boost (non-inverting / H-bridge buck-boost, 13th topology).
// Non-isolated, single inductor between two synchronous half-bridges: a buck leg (Q1 HS vin->sw1,
// Q2 LS sw1->gnd) and a boost leg (Q3 HS sw2->vout, Q4 LS sw2->gnd). Operated in the BUCK_BOOST
// transition region (SIMULTANEOUS mode): Q1+Q4 ON during D charge the inductor from Vin (V_L=+Vin);
// Q2+Q3 ON during (1-D) discharge it to the output (V_L=-Vout). Volt-second balance => M = D/(1-D),
// so Vo = Vin*D/(1-D) and D = Vo/(Vin+Vo). Port of MKF FourSwitchBuckBoost (Mode 1).
//
// All four devices are synchronous MOSFETs (no rectifier diodes) -> high efficiency (~0.98). Reuses the
// bridge template: anti-parallel body diodes + snubber caps at sw1/sw2 carry the inductor current and
// tame dV/dt during the per-leg dead time. No transformer (non-isolated) -> no coupling/rectifier
// subtleties; lowest convergence risk of the bridge family.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct FsbbDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double dutyCycle;       // D = Vo/(Vin+Vo) (buck-boost simultaneous gain)
    double deadFraction;    // per-leg dead time as a fraction of the period
    double inductance;      // single inductor L (worst-case of buck@Vinmax / boost@Vinmin)
    double loadResistance;
    double outputCapacitance;
};

/**
 * @brief Design a four-switch buck-boost converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
FsbbDesign design_fsbb(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a four-switch buck-boost converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_fsbb_tas(const FsbbDesign& d);

} // namespace Kirchhoff
