#pragma once

// Kirchhoff::Ahb — Asymmetric Half-Bridge (isolated, 11th topology). Single leg of two switches
// Q1 (high-side) / Q2 (low-side) driven with COMPLEMENTARY duty D / (1-D). A DC-blocking capacitor Cb
// sits in series with the transformer primary (vin -> Cb -> primary -> sw); in steady state V_Cb =
// (1-D)*Vin and the primary swings asymmetrically +(1-D)*Vin during D*T and -D*Vin during (1-D)*T.
// Secondary is rectified by a FULL-BRIDGE rectifier feeding a buck-like output (Lout + Cout). Port of
// MKF AsymmetricHalfBridge (Imbertson & Mohan 1993).
//
// Conversion ratio (CT/FB): Vo = 2*D*(1-D)*Vin/n — NON-monotonic, peaks at D=0.5 (Vo,max=Vin/2n).
// Control variable is the duty D (here a fixed operating point, D=0.45). Reuses the PSFB template:
// body diodes + snubber caps at floatable nodes, leakage-aware K, n compensates the rectifier 2*Vd.
//
// FULL_BRIDGE rectifier (not center-tapped): MKF's AHB center-tapped deck fails to converge here, and
// the full-bridge variant is correct (Vout 11.80 V for 48->12V/24W). (CT-secondary deck issue noted.)

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct AhbDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double dutyCycle;              // D = Q1 on-fraction (operating point)
    double deadFraction;          // dead time as a fraction of the period
    double turnsRatio;            // n = Np:Ns (full secondary)
    double magnetizingInductance; // Lm
    double dcBlockingCapacitance; // Cb
    double outputInductance;      // Lo
    double loadResistance;
    double outputCapacitance;
    nlohmann::json config;     // Cout (output filter)
};

/**
 * @brief Design an asymmetric half-bridge converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
AhbDesign design_ahb(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an asymmetric half-bridge converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_ahb_tas(const AhbDesign& d);

} // namespace Kirchhoff
