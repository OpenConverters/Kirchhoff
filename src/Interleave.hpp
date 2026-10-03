#pragma once

// Kirchhoff::interleave_stage — N-phase interleaving as a transform of a single-phase TAS (ABT #1437).
//
// A converter interleaves by running N identical power cells off the same input and into the same output,
// their switching staggered by 360°/N. Rather than re-author every builder for N phases, a topology designs
// ONE phase (at Pout/N), assembles it with its ordinary single-phase builder, and hands the result here:
//   • every component of `stage` is replicated N times as <name>_<k> (k = 1..N);
//   • a connection that touches one of `sharedPorts` (the input, return and output rails) becomes ONE net
//     holding every phase's endpoints; every other connection (switch nodes, gate nets, internal nodes) is
//     replicated per phase as <net>_<k>;
//   • a non-shared port (a gate input) is replicated as <port>_<k>;
//   • every stimulus on `stage` is replicated per phase with its component renamed and its phase advanced by
//     360·(k−1)/N degrees.
// It THROWS on anything it cannot replicate faithfully (an inter-stage connection or an initial condition
// that references a per-phase port or component), rather than wiring a phase to the wrong net.

#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

namespace Kirchhoff {

nlohmann::json interleave_stage(const nlohmann::json& tas, const std::string& stage, int phaseCount,
                                const std::set<std::string>& sharedPorts);

// The capacitor that sums N staggered copies of one phase's current (the buck output capacitor, fed by the
// inductor currents; the boost output capacitor, fed by the diode currents). `time`/`current` describe ONE
// phase over one switching period (piecewise linear; a repeated time stamp is a step). Returns the peak-to-
// peak charge the capacitor must absorb, ΔQ, and the RMS of the AC current it carries. N = 1 reproduces the
// single-phase result (ΔQ = ΔI·T/8 for a triangle).
struct InterleavedRipple { double chargePeakToPeak; double acCurrentRms; };
InterleavedRipple interleaved_capacitor_ripple(const std::vector<double>& time, const std::vector<double>& current,
                                               double period, int phaseCount);

// config.phaseCount as a validated whole number in [1, 6] (the interleaving range FSBB, PFC and Vienna use).
int phase_count(const nlohmann::json& config, const char* who);

} // namespace Kirchhoff
