#pragma once

// Kirchhoff::component_waveforms — per-component time-domain waveforms for EVERY non-magnetic power
// component (switches, diodes, capacitors, resistors) of an assembled TAS, from a single ngspice run.
//
// Magnetics already have per-winding waveforms via topology_waveforms / extract_operating_point; this is
// the complement that finally exposes the switch V_DS, the diode current, the output-cap ripple, etc. —
// the data the frontend needs to plot when a user clicks any part.
//
// How it works: the deck is generated with `.options savecurrents`, so ngspice records every device's
// terminal current as an @<letter>.<inst>.<dev>[<key>] vector, and every node voltage as a named vector.
// This walks the same TAS the assembler serialises, reconstructs each component's device-current token and
// its terminal nodes (deterministically — the naming is a pure function of stage/ref/kind), runs the sim
// once, and resamples ONE switching period (the window, see below) onto the shared 128-point grid with
// WaveformProcessor stats. A component whose reconstructed current vector is absent from the run (a stripped numerical
// snubber, a control-IC seed, a real-model device not yet mapped) is simply omitted — the result reports
// what the simulation actually contains, never an invented waveform.

#include <string>
#include <nlohmann/json.hpp>

#include "FidelityJson.hpp"   // PEAS::Fidelity

namespace Kirchhoff {

// Run the assembled TAS through libngspice once and return, per non-magnetic power component:
//   { ref, stage, kind, voltage:{label,waveform,processed}, current:{waveform,processed} }
// as { "engine":"ngspice", "referencePeriod":<s>, "window":{...}, "components":[ ... ] }.
//
// `window` picks WHICH switching period of the run every component (and rail) is read over:
//   {"at":"auto"} (or {})  DC input: the last period of the run. AC input (PFC, Vienna): the period at the
//                          inductor-current peak of the last line cycle (as "inductorPeak" on the main magnetic)
//                          — the run ends on a whole number of line cycles, i.e. at a line zero crossing, where
//                          the last period shows no switching ripple worth the name.
//   {"at":"end"}           the last switching period of the run.
//   {"at":"inductorPeak", "magnetic":"<name>"}   AC input only: the switching period centred where the
//                          one-switching-period moving average of that magnetic's primary winding current
//                          has its largest magnitude in the last line cycle. "magnetic" omitted = the main one.
//   {"at":"linePhase", "linePhaseDegrees":<deg>} AC input only: the switching period centred at that phase of
//                          the line (phase-A source angle, 0..360) in the last line cycle.
// The resolved window is echoed back as "window": {at, start, end [s, absolute run time], linePeriod,
// linePhaseDegrees (of the window centre), magnetic}. An unknown mode, a line mode on a DC converter, or a
// magnetic not in the TAS THROWS.
// THROWS std::runtime_error if libngspice is unavailable or the run fails (no silent fallback).
nlohmann::json component_waveforms(const nlohmann::json& tas, const PEAS::Fidelity& fidelity,
                                   const nlohmann::json& window = nlohmann::json::object());

}  // namespace Kirchhoff
