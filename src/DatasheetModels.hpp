#pragma once

// Kirchhoff::derive_datasheet_models — turn a spec-designed TAS (whose parts carry only design
// REQUIREMENTS, so every device renders ideal) into one whose semiconductors carry a datasheet-shaped
// model built FROM THOSE REQUIREMENTS. The point is a more realistic simulation: the real on-resistance
// and forward drop the design targeted, instead of the ideal switch's 0.01 Ω / default diode.
//
// It populates each MOSFET/diode's manufacturerInfo.datasheetInfo.electrical from the values the design
// already computed (maximumOnResistance → onResistance, ratedDrainSourceVoltage → drainSourceVoltage,
// maximumForwardVoltage → forwardVoltage, …). It does NOT invent data the design doesn't have: output
// capacitance, junction capacitance and the body-diode drop are OMITTED (a real sourced part carries
// those; a requirements-derived model cannot, and per the no-fallback rule we don't fabricate them). So
// the result is a real-CONDUCTION model — honest, and a strict improvement over ideal for loss/stress —
// not a claim of a specific catalog part. Magnetics are left untouched (their real path is the MKF
// MagneticAdviser core export, a separate integration).
//
// Once realized, tas_to_ngspice / component_waveforms / extract_operating_point with a DATASHEET fidelity
// render these devices as their real equivalent circuits (SAS mosfet/diode real leaves).

#include <nlohmann/json.hpp>

namespace Kirchhoff {

// Returns a copy of `tas` with a requirements-derived datasheet model added to every MOSFET/diode that
// does not already carry one (an already-bound real part is left as-is). Idempotent.
nlohmann::json derive_datasheet_models(const nlohmann::json& tas);

}  // namespace Kirchhoff
