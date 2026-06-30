// Validate the ported analytical DSP foundation (src/AnalyticalDsp.cpp) — Phase 1 of the MKF analytical
// solver port. Checks FFT correctness and harmonic extraction against known signals.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "AnalyticalDsp.hpp"

#include <cmath>
#include <complex>
constexpr double kPi = 3.14159265358979323846;
#include <vector>

using Kirchhoff::analytical::fft;
using Kirchhoff::analytical::calculate_harmonics_data;

TEST_CASE("fft of a pure cosine has a single non-DC bin", "[analytical][dsp]") {
    const size_t N = 64;
    const int k = 5;  // 5th harmonic
    std::vector<std::complex<double>> x(N);
    for (size_t n = 0; n < N; ++n)
        x[n] = std::cos(2.0 * kPi * k * n / N);
    fft(x);
    // Bins k and N-k carry N/2 each; all others ~0.
    for (size_t i = 1; i < N / 2; ++i) {
        double mag = std::abs(x[i]);
        if (i == static_cast<size_t>(k)) CHECK(mag == Catch::Approx(N / 2.0).margin(1e-9));
        else                             CHECK(mag == Catch::Approx(0.0).margin(1e-9));
    }
    CHECK(std::abs(x[0]) == Catch::Approx(0.0).margin(1e-9));  // no DC
}

TEST_CASE("calculate_harmonics_data recovers a sinusoid's fundamental + DC", "[analytical][dsp]") {
    const size_t N = 128;
    const double fsw = 100000.0;
    const double dc = 3.0, amp = 2.0;  // 3 V offset, 2 V peak fundamental
    MAS::Waveform w;
    std::vector<double> data(N);
    for (size_t n = 0; n < N; ++n)
        data[n] = dc + amp * std::sin(2.0 * kPi * n / N);
    w.set_data(data);

    MAS::Harmonics h = calculate_harmonics_data(w, fsw);
    REQUIRE(h.get_amplitudes().size() == N / 2);
    REQUIRE(h.get_frequencies().size() == N / 2);
    // amplitudes[0] = DC; amplitudes[1] = single-sided fundamental magnitude = peak.
    CHECK(h.get_amplitudes()[0] == Catch::Approx(dc).margin(1e-9));
    CHECK(h.get_amplitudes()[1] == Catch::Approx(amp).margin(1e-9));
    for (size_t i = 2; i < N / 2; ++i)
        CHECK(h.get_amplitudes()[i] == Catch::Approx(0.0).margin(1e-9));
    // frequencies are k·fsw.
    CHECK(h.get_frequencies()[0] == Catch::Approx(0.0));
    CHECK(h.get_frequencies()[1] == Catch::Approx(fsw));
    CHECK(h.get_frequencies()[2] == Catch::Approx(2 * fsw));
}

TEST_CASE("calculate_harmonics_data rejects non-power-of-2 data", "[analytical][dsp]") {
    MAS::Waveform w;
    w.set_data(std::vector<double>(100, 0.0));
    CHECK_THROWS(calculate_harmonics_data(w, 100000.0));
}
