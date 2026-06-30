#pragma once

// Kirchhoff::analytical_operating_point — the ANALYTICAL run engine (MKF-migration item 3,
// docs/MKF_MIGRATION.md). It predicts a converter's operating point WITHOUT spawning ngspice: it reads
// the closed-form quantities that build_*_tas already derives (output voltage/current/power, per-winding
// peak/rms/avg current and voltage stresses) straight from the assembled TAS document.
//
// Why it exists: (a) speed — no simulator process per candidate, so design sweeps run orders of
// magnitude faster; (b) it runs anywhere (pure arithmetic on the TAS JSON — no libngspice, WASM-clean),
// which serves the in-browser P5 goal; (c) it is the lighter alternative to a WASM ngspice. It is an
// IDEAL-coupling estimate (the design-predicted operating point), validated to agree with the SPICE
// deck — not a replacement for the SPICE run or the real-magnetic co-sim.
//
// Default run engine stays SPICE; ANALYTICAL is opt-in (the caller picks which to use).

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "Settings.hpp"   // Kirchhoff::RunEngine (config home is Settings)

namespace Kirchhoff {

// One magnetic winding's predicted stresses (the design-time closed-form excitation).
struct WindingStress {
    std::string component;   // magnetic component name (e.g. "T1", "Lout")
    int winding = 0;         // winding index within that magnetic (0 = primary)
    double currentPeak = 0, currentRms = 0, currentAverage = 0;
    double voltagePeak = 0, voltageRms = 0;
};

// The simulator-free predicted operating point.
struct AnalyticalOperatingPoint {
    double outputVoltage = 0;       // design-predicted output (V)
    double outputCurrent = 0;       // outputPower / outputVoltage (A)
    double outputPower = 0;         // W
    double inputPower = 0;          // outputPower / efficiency (W)
    double efficiency = 0;          // 0..1
    double switchingFrequency = 0;  // Hz
    std::vector<WindingStress> windings;
};

// Predict the operating point from an assembled TAS document (the output of any build_*_tas).
// THROWS std::runtime_error if the required fields are missing — no silent fallback.
AnalyticalOperatingPoint analytical_operating_point(const nlohmann::json& tas);

} // namespace Kirchhoff
