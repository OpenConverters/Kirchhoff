#pragma once

// Kirchhoff::NgspiceRunner — run an ngspice deck IN-PROCESS via libngspice (the shared-library API),
// instead of shelling out to the `ngspice -b` CLI. This is the native half of MKF-migration item 4
// (docs/MKF_MIGRATION.md): the same libngspice path the in-browser WASM run (P5) will reuse.
//
// Build gating: the implementation is compiled only when ENABLE_NGSPICE is defined (the CMake option
// finds libngspice + <ngspice/sharedspice.h>). When OFF, available() returns false and run() throws —
// callers fall back to the CLI. No silent no-op (per the no-fallbacks rule).
//
// Scope: this runs a magnetics-free converter deck (the kind Kirchhoff::tas_to_ngspice emits) and
// returns the raw transient vectors. The real-magnetic co-simulation (MKF's simulate_magnetic_circuit /
// extract_operating_point(Magnetic)) deliberately stays in MKF and is NOT ported here.

#include <string>
#include <vector>
#include <map>
#include <optional>

namespace Kirchhoff {

struct NgspiceRunResult {
    bool success = false;
    std::string error;
    std::vector<double> time;                                 // transient time vector
    std::map<std::string, std::vector<double>> vectors;       // RAW ngspice vector name -> samples
                                                              // (look up via average(), which canonicalizes)

    // Time-average of a vector over [from, to] — the in-process equivalent of `meas tran <x> AVG ...`.
    // Trapezoidal integration over the captured samples in the window, divided by the window length.
    // `name` is matched case-insensitively, with or without a "v(...)"/"i(...)" wrapper and any plot
    // prefix (e.g. "tran1.vout"). Returns nullopt if the vector or window has no samples.
    std::optional<double> average(const std::string& name, double from, double to) const;

    // Drop every sample with t < tStart from `time` and all `vectors`. NEEDED because the
    // shared-library data callback streams EVERY computed timepoint — a `.tran step stop tstart`
    // directive does NOT trim what this runner captures (tstart only gates ngspice's own stored
    // plot), so any deck that relies on a settle window must call this after the run.
    void drop_samples_before(double tStart);
};

// True iff Kirchhoff was built with libngspice (ENABLE_NGSPICE). When false, run_in_process throws.
bool ngspice_in_process_available();

// Run an ngspice deck string in-process. Any trailing `.control … .endc` block is stripped (this runner
// issues `run` itself and reads the vectors back through the shared-library data callback), so the deck
// only needs the circuit + `.options`/`.ic`/`.tran` analysis + `.end`. `timeoutSeconds` bounds the wait
// for the background simulation thread. Throws std::runtime_error if libngspice is unavailable.
NgspiceRunResult run_ngspice_in_process(const std::string& deck, double timeoutSeconds = 300.0);

} // namespace Kirchhoff
