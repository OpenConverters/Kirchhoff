#pragma once

// Kirchhoff::IsolatedBuckBoost — Isolated Buck-Boost / inverting "Fly-Buck-Boost" (17th topology). A
// flyback-style converter: a single high-side switch S1 charges the coupled-inductor primary from Vin
// during D, and during (1−D) the energy splits between an INVERTING non-isolated primary rail
// (V_pri = −Vin·D/(1−D), i.e. a buck-boost output, node vpri_out < 0) via a primary diode AND one or
// more isolated secondary rails (flyback-rectified). Port of MKF IsolatedBuckBoost.
//
// Like the Flybuck, this compares the PRIMARY rail (the harness's output[0]); here that rail is
// NEGATIVE (inverting), compared on magnitude exactly like the Ćuk port. The isolated secondary is
// present internally, flyback-rectified off the coupled inductor and loaded by an explicit resistor.
//
// New piece vs IsolatedBuck: the primary is an INVERTING buck-boost rail (Lpri tied to gnd, primary
// diode Dpri returning to the negative cap) rather than a buck rail, and the single switch is
// flyback-modulated (no synchronous low side).

#include <nlohmann/json.hpp>
#include <vector>
#include "Fidelity.hpp"

namespace Kirchhoff {

// One isolated secondary rail (multi-output, ABT #86). Each maps to designRequirements.outputs[1+i]:
// its own secondary winding (turnsRatio N_i = V_pri/(V_sec_i+Vd)), flyback rectifier, and output cap.
// secondaries[0] reproduces the single-output secondary scalars below byte-for-byte.
struct IsolatedBuckBoostSecondaryLeg {
    double voltage, power;            // V_sec_i, P_sec_i
    double turnsRatio;                // N_i = V_pri / (V_sec_i + Vd)
    double loadResistance;            // V_sec_i^2 / P_sec_i
    double capacitance;               // C_out_i
};

struct IsolatedBuckBoostDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double primaryVoltage, primaryPower;      // |V_pri| (output[0], inverting) — the compared output
    double secondaryVoltage, secondaryPower;  // isolated secondary rail (output[1]) — internal
    double switchingFrequency, efficiency;
    double dutyCycle;                 // D = V_pri/(Vin·η + V_pri)
    double turnsRatio;                // N = V_pri/(V_sec + Vd)
    double magnetizingInductance;     // Lmag (the coupled flyback inductor)
    double loadResistance;            // primary load (synthesized at the output port)
    double secondaryLoadResistance;   // secondary load (explicit internal resistor)
    double outputCapacitance;
    nlohmann::json config;         // primary Cpri (sets the settling RC)
    double secondaryCapacitance;      // secondary Cout
    // >=1 entry; secondaries[0] duplicates the secondary* scalars above. Additional entries (ABT #86)
    // are extra isolated rails exposed on external vout<i> ports.
    std::vector<IsolatedBuckBoostSecondaryLeg> secondaries;
};

/**
 * @brief Design an isolated buck-boost (inverting Fly-Buck-Boost) converter.
 * @param tasInputs Spec with TWO outputs: designRequirements.outputs[0]=|primary| inverting rail,
 *        outputs[1]=isolated secondary; operatingPoints[0].outputs[0/1].power give the loads.
 * @return A design struct (flyback duty, turns ratio, coupled-inductor inductance, both loads).
 */
IsolatedBuckBoostDesign design_isolated_buck_boost(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an isolated buck-boost design into a full TAS topology document.
 * @param d A design returned by design_isolated_buck_boost().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_isolated_buck_boost_tas(const IsolatedBuckBoostDesign& d);

} // namespace Kirchhoff
