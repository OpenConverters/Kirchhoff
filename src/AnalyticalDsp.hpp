#pragma once

// Kirchhoff::analytical — DSP foundation for the ported MKF analytical converter solver (Phase 1 of the
// analytical-run-engine port, docs/MKF_MIGRATION.md). These are the topology-agnostic signal-processing
// kernels the per-topology process_operating_points functions build on: FFT, harmonic analysis,
// waveform synthesis, and processed-data (RMS/peak/avg/THD) extraction. Ported faithfully from MKF's
// processors/Inputs.cpp so the analytical results match MKF's. Magnetics-independent (pure arithmetic
// + the MAS waveform types), exactly the ideal-coupling domain Kirchhoff targets.

#include "MAS.hpp"   // MAS::Waveform, MAS::Harmonics (the operating-point graph, available since Phase 0)

#include <complex>
#include <vector>

namespace Kirchhoff {
namespace analytical {

// In-place radix-2 Cooley-Tukey FFT (ported from MKF Inputs.cpp::fft). x.size() must be a power of 2.
void fft(std::vector<std::complex<double>>& x);

// Harmonic amplitudes + frequencies of one sampled period of a waveform, via FFT (ported from MKF
// Inputs.cpp::calculate_harmonics_data). amplitudes[0] is the DC term; amplitudes[k>0] is the
// single-sided magnitude 2·|X_k|/N; frequencies[k] = frequency·k. The waveform's data length must be a
// power of 2. (MKF's import-trim branch is omitted — analytically-synthesized waveforms are never
// "imported", so that path never runs.)
MAS::Harmonics calculate_harmonics_data(const MAS::Waveform& waveform, double frequency);

} // namespace analytical
} // namespace Kirchhoff
