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
    std::string region;     // operating region at nominal Vin: "buck" | "boost" | "buckBoost" (MKF Region)
    double regionDuty;      // the region's PWM duty (buck D=Vo/Vin·η; boost D=1−Vin·η/Vo; bb D=Vo/(Vin+Vo))
    double deadFraction;    // per-leg dead time as a fraction of the period
    double inductance;      // single inductor L (worst-case of buck@Vinmax / boost@Vinmin)
    double loadResistance;
    double outputCapacitance;
    // Transition-band (buckBoost region) sub-mode (ABT #94). "splitPwm" (MKF default, LM5176/LT8390):
    // the buck and boost legs run at DIFFERENT, phase-shifted duties → lower inductor ripple.
    // "simultaneous": all four switches commute together (D=Vo/(Vin+Vo)). Only affects the buckBoost region.
    std::string transitionMode;   // "splitPwm" | "simultaneous"
    double splitRatio;            // κ ∈ (0,1]: split-PWM boost-LS charge duty as a fraction of D (κ→1 ⇒ simultaneous)
    double splitBoostDuty;        // t1 = κ·D — boost-leg low-side (Q4) charge duty (splitPwm/buckBoost only)
    double splitBuckDuty;         // t2 = Vo·(1−t1)/Vin — buck-leg high-side (Q1) duty (splitPwm/buckBoost only)
    // Bidirectional (ABT #94): config.powerFlowDirection="reverse" makes Vout the source and delivers to Vin
    // (mirror of the Ćuk/CLLC pattern). All four devices are already synchronous MOSFETs, so the H-bridge
    // conducts both ways; only the source/load direction, the duty target, and the gate mapping flip.
    bool reverse;
    // Interleaved multi-phase (ABT #94): config.phaseCount = N (2,3,…) builds N phase-shifted 4-switch
    // buck-boost legs sharing the input/output bus, interleaved by 360/N degrees. Each leg carries Iout/N,
    // so its inductor is sized for Pout/N (⇒ N× the single-phase L). The staggered legs cancel most of the
    // net input/output ripple. N=1 (the default) is the ordinary single-phase converter (byte-identical).
    int phaseCount;
    nlohmann::json config;
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
