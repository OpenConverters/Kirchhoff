#pragma once

// Kirchhoff::TwoSwitchFlyback — the double-ended flyback (ABT #1436).
//   Q1 (high side): Vin -> primary start;  Q2 (low side): primary end -> GND; both gated together.
//   Dc1: GND -> primary start (anode at GND);  Dc2: primary end -> Vin (cathode at Vin).
// At turn-off the primary's stored leakage energy commutates through Dc1/Dc2 back into Vin, so each switch
// is clamped at Vin — half the stress of the single-switch flyback's Vin + Vor + spike — at the price of a
// second (high-side) switch. The clamp only stays passive while the reflected voltage is BELOW Vin: if
// Vor = n·(Vo+Vd) reached Vin, Dc1/Dc2 would conduct during the reset and return the stored energy to the
// input instead of the output. So the design is the ordinary flyback's (same transformer, same CCM/DCM/BCM
// sizing, multi-output) with the duty cap tightened to keep Vor below Vin,min, and it THROWS when a pinned
// turns ratio (or a duty cap >= 0.5) puts Vor at or above Vin,min.

#include <nlohmann/json.hpp>
#include "Flyback.hpp"

namespace Kirchhoff {

using TwoSwitchFlybackDesign = FlybackDesign;

/**
 * @brief Design a two-switch flyback: the flyback design with Vor held below Vin,min.
 * @param tasInputs The converter spec (as for design_flyback). config.maxDutyCycle defaults to 0.45 and
 *        must be < 0.5; config.mode is ccm (default), dcm or bcm (quasi-resonant valley switching is not
 *        modelled for the clamped two-switch cell and is refused).
 */
TwoSwitchFlybackDesign design_two_switch_flyback(const nlohmann::json& tasInputs);

/**
 * @brief Assemble a two-switch flyback design into a full TAS topology document.
 */
nlohmann::json build_two_switch_flyback_tas(const TwoSwitchFlybackDesign& d);

} // namespace Kirchhoff
