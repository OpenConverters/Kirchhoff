#pragma once

// Kirchhoff::Llc — LLC resonant converter (19th topology, first resonant tank). A half-bridge drives a
// series Lr-Cr resonant tank in series with the transformer magnetizing inductance Lm (the "LLC" =
// Lr + Lm + Cr); a center-tapped full-wave rectifier feeds the output. The voltage gain is set by the
// switching frequency relative to the tank resonance fr = 1/(2π√(Lr·Cr)); the converter is operated at
// a fixed fsw (here below fr, gain > 1). Port of MKF Llc (half-bridge, center-tapped rectifier).
//
// New piece vs the PWM bridges: a RESONANT tank (explicit Lr + Cr + Lm) instead of a duty/phase-set
// square wave — the output follows the tank's frequency response, not a duty ratio.
//
// Fidelity note (surfaced): MKF's LLC deck abstracts the half-bridge to an IDEAL ±Vbus/2 bipolar
// voltage source (no switches) and uses a near-ideal rectifier diode (N=0.01, ~0 V drop). Kirchhoff
// builds the REAL converter — a split-cap half-bridge of actual switches (with ZVS body diodes) plus
// standard N=1 rectifier diodes. The two agree to ~2–2.5 % on Vout (the real switches' ZVS transitions
// + the ~0.9 V diode drop vs MKF's ideal source + ~0 V diode). LLC.h itself notes ±10 % vs bench is
// normal, so the resonant family is compared at a documented 3 % tolerance.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"
#include "Rectifier.hpp"

namespace Kirchhoff {

struct LlcDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double turnsRatio;                // n = (k_bridge·Vin_nom) / Vout (per CT half-winding; doubled for VD)
    double resonantInductance;        // Lr
    double resonantCapacitance;       // Cr
    double magnetizingInductance;     // Lm = Ln·Lr
    double resonantFrequency;         // fr = √(fmin·fmax)
    double switchDuty;                // per-switch on-fraction (~0.45, complementary with dead time)
    double loadResistance;
    double outputCapacitance;
    RectifierType rectifierType = RectifierType::CenterTapped;  // CT default (matches MKF Llc + the fixture)
    double outputInductance;          // CURRENT_DOUBLER only: each of the two output inductors Lo1/Lo2
    nlohmann::json config;
};

/**
 * @brief Design a half-bridge LLC resonant converter (center-tapped rectifier).
 * @param tasInputs Single-output spec (designRequirements + operatingPoints[0]). The resonant design
 *        knobs (quality factor 0.4, inductance ratio 5, fr from an 80–200 kHz band, half-bridge) match
 *        MKF's Llc defaults; the operating-point switching frequency sets where on the gain curve the
 *        converter runs.
 * @return A design struct (turns ratio, Lr, Cr, Lm, resonant frequency, load, output cap).
 */
LlcDesign design_llc(const nlohmann::json& tasInputs);
/**
 * @brief Assemble an LLC design into a full TAS topology document.
 * @param d A design returned by design_llc().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_llc_tas(const LlcDesign& d);

} // namespace Kirchhoff
