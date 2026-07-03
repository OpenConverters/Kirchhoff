#pragma once

// Kirchhoff::Src — Series Resonant Converter (20th topology). Like the LLC but the tank is a TWO-element
// series Lr+Cr only (no parallel resonant Lm branch); the transformer magnetizing inductance is made
// large (≈10·Lr) so it does not participate in the resonance. A half-bridge drives the Lr-Cr tank into
// the transformer + center-tapped rectifier. Gain M ≤ 1 (step-down only); operated at fsw = fr where
// the series resonance gives unity tank gain. Port of MKF Src (Series Resonant; Steigerwald 1988).
//
// Reuses the LLC half-bridge + tank + CT-rectifier structure (same fidelity caveat: MKF abstracts the
// bridge to an ideal ±Vbus/2 source + near-ideal diode, Kirchhoff builds the real switching bridge +
// N=1 diode, so the resonant family is compared at a documented 3% tolerance). The only design
// difference vs LLC is the tank: Q=2 (vs 0.4), an explicit fr, and a large non-resonant Lm.

#include <nlohmann/json.hpp>
#include <vector>
#include "Fidelity.hpp"
#include "Rectifier.hpp"

namespace Kirchhoff {

// One isolated output rail (multi-output SRC, ABT #86). outputs[0] mirrors the top-level scalar fields
// (the main/measured rail); outputs[1..] are additional isolated secondaries. Each carries its own turns
// ratio n_i (sized so n_i·(Vout_i+Vd_i) matches the shared bridge drive → all rails reflect to the same
// primary clamp and conduct together), rectifier diode drop, output cap, load, and (currentDoubler only)
// output inductor. The shared Lr–Cr tank is sized against the SUM of the reflected loads (parallel Rac).
struct SrcOutputLeg {
    double voltage, power, turnsRatio, diodeDrop, loadResistance, outputCapacitance, outputInductance;
};

struct SrcDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double turnsRatio;                // n = (k_bridge·Vin_nom) / Vout (per CT half-winding)
    double resonantInductance;        // Lr
    double resonantCapacitance;       // Cr
    double magnetizingInductance;     // Lm ≈ 10·Lr (large, non-resonant)
    double resonantFrequency;         // fr (= fsw here: operate at series resonance)
    double switchDuty;                // per-switch on-fraction (~0.45, complementary with dead time)
    double loadResistance;
    double outputCapacitance;
    RectifierType rectifierType = RectifierType::CenterTapped;  // CT default (matches MKF Src + the fixture)
    double outputInductance;          // CURRENT_DOUBLER only: each of the two output inductors
    bool fullBridge = false;          // false = split-cap half-bridge (±Vin/2, MKF default); true = 4-MOSFET
                                      // full-bridge primary (±Vin, bridge factor 1.0) — config.bridgeType (ABT #91)
    std::vector<SrcOutputLeg> outputs;   // >=1 entry; [0] duplicates the scalars above (ABT #86)
    nlohmann::json config;
};

/**
 * @brief Design a half-bridge series-resonant converter (center-tapped rectifier).
 * @param tasInputs Single-output spec (designRequirements + operatingPoints[0]). Quality factor 2.0
 *        and the half-bridge match MKF's Src defaults; the converter is designed/operated at series
 *        resonance (fr = the operating-point switching frequency).
 * @return A design struct (turns ratio, Lr, Cr, large Lm, resonant frequency, load, output cap).
 */
SrcDesign design_src(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an SRC design into a full TAS topology document.
 * @param d A design returned by design_src().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_src_tas(const SrcDesign& d);

} // namespace Kirchhoff
