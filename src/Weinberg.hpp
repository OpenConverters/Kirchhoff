#pragma once

// Kirchhoff::Weinberg — Weinberg converter (18th topology). A current-fed, push-pull-derivative,
// isolated, boost-capable DC-DC converter (ESA Weinberg 1974). An input coupled inductor L1 (1:1, two
// windings) current-feeds a center-tapped push-pull primary (2 switches Q1/Q2, 180° apart, overlapping
// when D>0.5); a 4-winding main transformer (CT primary + CT secondary) drives a center-tapped
// full-wave rectifier into the output cap. Port of MKF Weinberg (V1 "classic", CT-FW diodes).
//
// Config-gated variants (ABT #88): `config.variant = "bridge"` swaps the 2-switch push-pull primary
// for a 4-switch H-bridge (diagonal PWM; halves the primary switch voltage); `config.synchronousRectifier
// = true` swaps the two center-tapped freewheel diodes for SR MOSFETs (complementary drive + body diode).
//
// New pieces vs push-pull: the current-fed INPUT coupled inductor L1 (the converter's defining
// feature), and the boost-capable conversion ratio M = 1/(2·n·(1−D)) in the D>0.5 regime. Like MKF's
// V1 the energy-recovery D3 diodes are omitted (RC snubbers tame the leakage spike instead).
//
// Magnetics are modelled the way MKF's SPICE deck is: the CT main transformer is one coupled magnetic
// with turnsRatios=[1, n, n] (winding0=primary half a, winding1=primary half b at ratio 1, windings
// 2/3 = the two secondary halves at ratio n), opposite-dot wound; the input coupled inductor is a
// separate 1:1 two-winding magnetic. Per-winding leakage is folded into the coupling (K=0.9999), as
// MKF's explicit Llk is only ~0.1% of each winding.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

// Primary-drive variant (ABT #88; MKF Weinberg `variant` enum). CLASSIC = the 2-switch current-fed
// center-tapped push-pull (V1); BRIDGE = a 4-switch H-bridge primary (V2) that drives the two primary
// halves in series (diagonal PWM + boost-overlap freewheel). The bridge halves the primary switch
// blocking voltage (n·Vout per device vs the push-pull's 2·n·Vout) — used on high-bus spacecraft rails.
enum class WeinbergVariant { Classic, Bridge };

struct WeinbergDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double turnsRatio;                // n = Np_total : Ns_total (combined)
    double dutyCycle;                 // D at nominal Vin (boost regime, D>0.5)
    double switchDuty;                // per-switch on-fraction (= D; the two switches overlap for D>0.5)
    double inputInductance;           // L1 (each winding of the input coupled inductor)
    double magnetizingInductance;     // Lpri_half (each primary-half winding of the main transformer)
    double loadResistance;
    double outputCapacitance;
    WeinbergVariant variant;          // primary drive: classic push-pull (default) or H-bridge (ABT #88)
    bool synchronousRectifier;        // secondary CT-FW: real diodes (default) or SR MOSFETs (ABT #88)
    double deadFraction;              // SR / bridge complementary-drive dead band (fraction of the period)
    nlohmann::json config;
};

/**
 * @brief Design a Weinberg converter (current-fed push-pull, boost regime).
 * @param tasInputs Single-output spec (designRequirements + operatingPoints[0]).
 * @return A design struct (turns ratio, boost-regime duty, input coupled-inductor L1, main-transformer
 *         magnetizing inductance, load, output cap) — sized to match MKF's Weinberg design.
 */
WeinbergDesign design_weinberg(const nlohmann::json& tasInputs);
/**
 * @brief Assemble a Weinberg design into a full TAS topology document.
 * @param d A design returned by design_weinberg().
 * @return A TAS document (JSON); pass it to Kirchhoff::tas_to_ngspice() for a runnable deck.
 */
nlohmann::json build_weinberg_tas(const WeinbergDesign& d);

} // namespace Kirchhoff
