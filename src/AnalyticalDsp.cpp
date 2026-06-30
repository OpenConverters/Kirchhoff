#include "AnalyticalDsp.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace Kirchhoff {
namespace analytical {

namespace { constexpr double kPi = 3.14159265358979323846264338327950288; }  // C++17: no std::numbers

// Ported verbatim from MKF processors/Inputs.cpp::fft (in-place radix-2 Cooley-Tukey).
void fft(std::vector<std::complex<double>>& x) {
    unsigned int N = static_cast<unsigned int>(x.size()), k = N, n;
    if (N == 0) return;
    double thetaT = kPi / N;
    std::complex<double> phiT = std::complex<double>(cos(thetaT), -sin(thetaT)), T;
    while (k > 1) {
        n = k;
        k >>= 1;
        phiT = phiT * phiT;
        T = 1.0L;
        for (unsigned int l = 0; l < k; l++) {
            for (long a = l; a < N; a += n) {
                long b = a + k;
                std::complex<double> t = x[a] - x[b];
                x[a] += x[b];
                x[b] = t * T;
            }
            T *= phiT;
        }
    }
    // Bit-reversal decimation.
    unsigned int m = static_cast<unsigned int>(log2(N));
    for (unsigned int a = 0; a < N; a++) {
        unsigned int b = a;
        b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
        b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
        b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
        b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
        b = ((b >> 16) | (b << 16)) >> (32 - m);
        if (b > a) std::swap(x[a], x[b]);
    }
}

// Ported from MKF processors/Inputs.cpp::calculate_harmonics_data (the synthesized-waveform path; the
// is_waveform_imported trim branch is omitted — see header).
MAS::Harmonics calculate_harmonics_data(const MAS::Waveform& waveform, double frequency) {
    MAS::Harmonics harmonics;

    std::vector<std::complex<double>> data;
    data.reserve(waveform.get_data().size());
    for (double v : waveform.get_data()) data.emplace_back(v);

    if (data.size() > 0 && ((data.size() & (data.size() - 1)) != 0))
        throw std::invalid_argument("calculate_harmonics_data: data size is not a power of 2: " +
                                    std::to_string(data.size()));
    fft(data);

    const double N = static_cast<double>(data.size());
    harmonics.get_mutable_amplitudes().push_back(std::abs(data[0] / N));         // DC
    for (size_t i = 1; i < data.size() / 2; ++i)
        harmonics.get_mutable_amplitudes().push_back(std::abs(2.0 * data[i] / N)); // single-sided
    for (size_t i = 0; i < data.size() / 2; ++i)
        harmonics.get_mutable_frequencies().push_back(frequency * static_cast<double>(i));

    return harmonics;
}

} // namespace analytical
} // namespace Kirchhoff
