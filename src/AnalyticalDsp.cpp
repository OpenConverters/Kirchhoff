#include "AnalyticalDsp.hpp"

#include <algorithm>
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

// Ported from MKF processors/Inputs.cpp::create_waveform. Each WaveformLabel produces piecewise-linear
// (data, time) control points for one period. (The skew rotation path is deferred — see header.)
MAS::Waveform create_waveform(MAS::WaveformLabel label, double peakToPeak, double frequency,
                              double dutyCycle, double offset, double deadTime,
                              double skew, double phase, size_t numberOfPoints) {
    using L = MAS::WaveformLabel;
    MAS::Waveform waveform;
    std::vector<double> data, time;
    const double period = 1.0 / frequency;

    switch (label) {
        case L::TRIANGULAR: {
            double max = peakToPeak / 2 + offset, min = -peakToPeak / 2 + offset, dc = dutyCycle * period;
            data = {min, max, min}; time = {0, dc, period}; break;
        }
        case L::TRIANGULAR_WITH_DEADTIME: {
            double max = peakToPeak / 2 + offset, min = -peakToPeak / 2 + offset, dc = dutyCycle * period;
            data = {min, max, min, 0}; time = {0, dc, period - deadTime, period}; break;
        }
        case L::UNIPOLAR_TRIANGULAR: {
            double max = peakToPeak + offset, min = offset, dc = dutyCycle * period;
            data = {min, max, min, min}; time = {0, dc, dc, period}; break;
        }
        case L::RECTANGULAR: {
            double max = peakToPeak * (1 - dutyCycle) + offset, min = -peakToPeak * dutyCycle + offset, dc = dutyCycle * period;
            data = {min, max, max, min, min}; time = {0, 0, dc, dc, period}; break;
        }
        case L::RECTANGULAR_WITH_DEADTIME: {
            double max = peakToPeak * (1 - dutyCycle) + offset, min = -peakToPeak * dutyCycle + offset, dc = dutyCycle * period;
            data = {0, max, max, min, min, 0, 0}; time = {0, 0, dc, dc, period - deadTime, period - deadTime, period}; break;
        }
        case L::SECONDARY_RECTANGULAR: {
            double max = -peakToPeak * (1 - dutyCycle) + offset, min = peakToPeak * dutyCycle + offset, dc = dutyCycle * period;
            data = {min, max, max, min, min}; time = {0, 0, dc, dc, period}; break;
        }
        case L::SECONDARY_RECTANGULAR_WITH_DEADTIME: {
            double max = -peakToPeak * (1 - dutyCycle) + offset, min = peakToPeak * dutyCycle + offset, dc = dutyCycle * period;
            data = {0, max, max, min, min, 0, 0}; time = {0, 0, dc, dc, period - deadTime, period - deadTime, period}; break;
        }
        case L::UNIPOLAR_RECTANGULAR: {
            double max = peakToPeak + offset, min = offset, dc = std::min(0.5, dutyCycle) * period;
            data = {min, max, max, min, min}; time = {0, 0, dc, dc, period}; break;
        }
        case L::BIPOLAR_RECTANGULAR: {
            double max = +peakToPeak / 2, min = -peakToPeak / 2, dc = dutyCycle * period;
            data = {0, 0, max, max, 0, 0, min, min, 0, 0};
            time = {0, 0.25 * period - dc / 2, 0.25 * period - dc / 2, 0.25 * period + dc / 2, 0.25 * period + dc / 2,
                    0.75 * period - dc / 2, 0.75 * period - dc / 2, 0.75 * period + dc / 2, 0.75 * period + dc / 2, period};
            break;
        }
        case L::BIPOLAR_TRIANGULAR: {
            double max = +peakToPeak / 2, min = -peakToPeak / 2, dc = std::min(0.5, dutyCycle) * period;
            data = {min, min, max, max, min, min};
            time = {0, 0.25 * period - dc / 2, 0.25 * period + dc / 2, 0.75 * period - dc / 2, 0.75 * period + dc / 2, period};
            break;
        }
        case L::FLYBACK_PRIMARY: {
            double max = peakToPeak + offset, min = offset, dc = dutyCycle * period;
            data = {0, min, max, 0, 0}; time = {0, 0, dc, dc, period}; break;
        }
        case L::FLYBACK_SECONDARY: {
            double max = peakToPeak + offset, min = offset, dc = dutyCycle * period;
            data = {0, 0, max, min, 0}; time = {0, dc, dc, period, period}; break;
        }
        case L::FLYBACK_SECONDARY_WITH_DEADTIME: {
            double max = peakToPeak + offset, min = offset, dc = dutyCycle * period;
            data = {0, 0, max, min, 0, 0}; time = {0, dc, dc, period - deadTime, period - deadTime, period}; break;
        }
        case L::SINUSOIDAL: {
            const size_t pts = (numberOfPoints < 2) ? 2 : numberOfPoints;
            for (size_t i = 0; i < pts; ++i) {
                double angle = i * 2 * kPi / (pts - 1);
                time.push_back(i * period / (pts - 1));
                data.push_back((std::sin(angle + phase) * peakToPeak / 2) + offset);
            }
            break;
        }
        default:
            throw std::invalid_argument("create_waveform: unsupported WaveformLabel (CUSTOM / "
                                        "RECTANGULAR_DCM must be built by the topology directly)");
    }

    waveform.set_ancillary_label(label);
    waveform.set_data(data);
    waveform.set_time(time);

    if (skew != 0.0)
        throw std::invalid_argument("create_waveform: skew != 0 requires calculate_sampled_waveform "
                                    "(analytical solver port Phase 1.3); not yet available");
    return waveform;
}

} // namespace analytical
} // namespace Kirchhoff
