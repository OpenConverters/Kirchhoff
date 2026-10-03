#pragma once

// Kirchhoff::InvertingBuckBoost — the single-switch, single-inductor inverting buck-boost (ABT #1433), the
// textbook parent of Ćuk / SEPIC / Zeta / FSBB.
//   Q1: Vin -> sw (high side);  L1: sw -> gnd;  D1: anode at Vout (negative), cathode at sw;  Cout: Vout -> gnd.
//   On: v_L = +Vin, L charges from the input.  Off: L discharges through D1 into the output, so the output
//   rail is NEGATIVE: Vo = −Vin·D/(1−D) in CCM. The output is handled like Ćuk's: the spec may state the
//   setpoint either signed or as a magnitude; the TAS carries the negative rail.
// Conduction mode is config.mode = "ccm" (default; L sized for a ripple ratio) or "dcm" (L sized below the
// critical inductance at Vin,min, so the inductor runs dry every period across the whole input range).
// config.rectifier = "synchronous" replaces D1 with a MOSFET Q2 (+ body diode D2) driven complementary to Q1.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct InvertingBuckBoostDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltageMag;   // |Vo| (the rail itself is negative)
    double outputPower, switchingFrequency, efficiency, diodeDrop;
    double dutyCycle;          // CCM: (|Vo|+Vd)/(Vin·η + |Vo|+Vd);  DCM: tOn·fsw from the energy balance
    double inductance;         // L1 (H)
    double criticalInductance; // CCM/DCM boundary at Vin,min, full load (H)
    double loadResistance;     // |Vo|² / Pout
    double outputCapacitance;  // Cout (F)
    bool dcm = false;          // config.mode == "dcm"
    bool synchronousRectifier = false;
    double deadFraction = 0.01;
    nlohmann::json config;
};

/**
 * @brief Design an inverting buck-boost (negative output) for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage, signed or magnitude) + operatingPoints[0]
 *        (inputVoltage, outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return The sized design (duty cycle, inductance, output capacitance, load).
 */
InvertingBuckBoostDesign design_inverting_buck_boost(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an inverting buck-boost design into a full TAS topology document.
 * @param d A design returned by design_inverting_buck_boost().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_inverting_buck_boost_tas(const InvertingBuckBoostDesign& d);

} // namespace Kirchhoff
