#pragma once

// Kirchhoff::Flyback — design a flyback converter from TAS-style inputs, assemble it as a CIAS atom-brick
// (using the per-family to_cias generators), and emit a runnable ngspice deck (via the CIAS
// converter + a synthesized testbench).
//
// This is the P3 (assembly) + P4 (design) + P5 (deck) critical-path integration for the ideal
// flyback. The design math is a faithful-but-minimal CCM port of MKF's Flyback (turns ratio, duty,
// magnetizing inductance, load); the full CCM/DCM/QRM/BMO port is Phase 4.

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "Fidelity.hpp"

namespace Kirchhoff {

// One isolated output rail (multi-output flyback, ABT #86). outputs[0] mirrors the top-level scalar
// fields (the main/regulated rail whose voltage sets the duty); outputs[1..] are additional secondaries.
// Every secondary sees the same magnetizing flux, so each rail's turns ratio n_i = Np/Ns_i is scaled so
// that n_i·(Vout_i+Vd_i) equals the shared reflected voltage Vor — i.e. each output regulates to its own
// Vout at the common duty. Each rail carries its own rectifier, output capacitor and load.
struct FlybackOutputLeg {
    double voltage, power, turnsRatio, diodeDrop, outputCapacitance, loadResistance;
};

struct FlybackDesign {
    double inputVoltage;       // V (operating point Vin)
    double inputVoltageMin;    // V (design corner: max current)
    double inputVoltageMax;    // V (design corner: max voltage stress)
    double outputVoltage;      // V
    double outputPower;        // W
    double switchingFrequency; // Hz
    double efficiency;         // 0..1
    double diodeDrop;          // V
    double isolationVoltage;   // V RMS required input-output isolation (0 = none)

    // designed parameters
    double turnsRatio;         // n = Np/Ns
    double dutyCycle;          // D at the operating point
    double magnetizingInductance; // Lp (H)
    double loadResistance;     // R = Vout^2 / Pout
    double outputCapacitance;
    nlohmann::json config;  // F
    double inputCapacitance;   // F
    std::vector<FlybackOutputLeg> outputs;   // >=1 entry; [0] duplicates the scalars above (ABT #86)
};

// Design from TAS-style inputs.designRequirements + a chosen operating point.
/**
 * @brief Design a flyback converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct with the sized component values (turns ratio / inductances /
 *         capacitances / duty cycle / load resistance / output capacitance).
 */
FlybackDesign design_flyback(const nlohmann::json& tasInputs);

// Build the flyback as a CIAS atom-brick (components are ideal atoms from the family to_cias
// generators; ports VIN/GND/VOUT). fidelity selects ideal vs real per component.
nlohmann::json build_flyback_brick(const FlybackDesign& d, const PEAS::Fidelity& fidelity);

// Emit a complete, runnable ngspice deck: subcircuit cards (via CIAS) + Vin source + gate PULSE
// (open-loop at the designed duty) + .tran + .control to print Vout.
std::string emit_flyback_ngspice(const FlybackDesign& d, const PEAS::Fidelity& fidelity);

// Build a full TAS topology document for the flyback: topology.stages[] (inverter/transformer/
// rectifier/filter) each instantiating a CIAS brick of PEAS parts, wired by interStageConnections[],
// plus inputs + simulation.stimulus. Consumed by the generic Kirchhoff::tas_to_ngspice assembler — this
// is the "design -> TAS-of-CIAS" output, decoupled from assembly.
/**
 * @brief Assemble a flyback converter design into a full TAS topology document.
 * @param d A design returned by the matching design_*() function.
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_flyback_tas(const FlybackDesign& d);

} // namespace Kirchhoff
