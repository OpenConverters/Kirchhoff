#pragma once

// Kirchhoff::Dab — Dual Active Bridge (isolated bidirectional DC-DC, 15th topology). Two full
// H-bridges connected through a series inductance and a high-frequency transformer:
//
//     [primary bridge QA..QD] -- Lr -- [T1 Np:Ns] -- [secondary bridge QE..QH] -- Cout/Rload
//
// Both bridges are ACTIVELY driven square waves; the secondary bridge is phase-shifted by D3 (the
// inter-bridge "outer" shift) relative to the primary. Power flows P = N·V1·V2·D3·(π-|D3|)/(2π²·Fs·L)
// (Single-Phase-Shift / SPS modulation, D1=D2=0). Unlike a phase-shift full bridge with a passive
// rectifier, the secondary here is a SYNCHRONOUS active bridge — the body diodes alone would clamp
// Vout near Vin/N (a DC transformer); the active phase-shift is what limits/controls the transferred
// power and sets the operating-point Vout. Port of MKF Dab.
//
// New piece vs PSFB: the second active bridge driven at phase D3 (not a diode rectifier), and no
// output inductor — Vout is set by the power-transfer balance, so it is loss-sensitive. To reproduce
// MKF's settled Vout the deck mirrors MKF's per-switch 100 Ω ∥ 100 pF snubbers on all eight switches
// (MKF's convergence/damping snubbers, which also set the conduction droop).

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct DabDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double turnsRatio;             // N = V1_nom : V2_nom (primary-to-secondary)
    double phaseShiftDeg;          // D3 — inter-bridge outer shift in degrees (drives power)
    double switchDuty;             // per-switch on-fraction (~0.5 minus dead time)
    double seriesInductance;       // L (leakage + external resonant inductor Lr)
    double magnetizingInductance;  // Lm
    double loadResistance;
    double outputCapacitance;
};

/**
 * @brief Design a dual-active-bridge converter for the given specification.
 *
 * @param tasInputs The converter spec — designRequirements (efficiency, inputVoltage,
 *        switchingFrequency, outputs[].voltage) + operatingPoints[0] (inputVoltage,
 *        outputs[0].power). See Kirchhoff.hpp for the full input format.
 * @return A design struct (turns ratio N, outer phase shift D3, series + magnetizing inductances,
 *         load resistance, output capacitance) — sized to match MKF's SPS DAB design.
 */
DabDesign design_dab(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a dual-active-bridge design into a full TAS topology document.
 * @param d A design returned by design_dab().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_dab_tas(const DabDesign& d);

} // namespace Kirchhoff
