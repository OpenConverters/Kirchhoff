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
// once, and resamples the LAST switching period onto the shared 128-point grid with WaveformProcessor
// stats. A component whose reconstructed current vector is absent from the run (a stripped numerical
// snubber, a control-IC seed, a real-model device not yet mapped) is simply omitted — the result reports
// what the simulation actually contains, never an invented waveform.

#include <string>
#include <nlohmann/json.hpp>

#include "FidelityJson.hpp"   // PEAS::Fidelity

namespace Kirchhoff {

// Run the assembled TAS through libngspice once and return, per non-magnetic power component:
//   { ref, stage, kind, voltage:{label,waveform,processed}, current:{waveform,processed} }
// as { "engine":"ngspice", "referencePeriod":<s>, "components":[ ... ] }.
// THROWS std::runtime_error if libngspice is unavailable or the run fails (no silent fallback).
nlohmann::json component_waveforms(const nlohmann::json& tas, const PEAS::Fidelity& fidelity);

}  // namespace Kirchhoff
