#include "AnalyticalDsp.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace Kirchhoff {
namespace analytical {

namespace {
constexpr double kPi = 3.14159265358979323846264338327950288;  // C++17: no std::numbers

double lerp_(double a, double b, double t) { return a + t * (b - a); }  // C++17: no std::lerp

std::vector<double> linear_spaced_array(double start, double end, size_t n) {
    std::vector<double> v;
    if (n == 0) return v;
    if (n == 1) { v.push_back(start); return v; }
    const double step = (end - start) / static_cast<double>(n - 1);
    v.reserve(n);
    for (size_t i = 0; i < n; ++i) v.push_back(start + step * static_cast<double>(i));
    return v;
}

double round_float(double x, int decimals) {
    const double f = std::pow(10.0, decimals);
    return std::round(x * f) / f;
}

size_t round_up_to_power_of_2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}
} // namespace

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

// Ported from MKF processors/Inputs.cpp::calculate_sampled_waveform.
MAS::Waveform calculate_sampled_waveform(const MAS::Waveform& waveform, double frequency, size_t numberPoints) {
    const std::vector<double>& data = waveform.get_data();
    if (data.size() < 2)
        throw std::invalid_argument("calculate_sampled_waveform: waveform needs >= 2 data points");

    std::vector<double> time;
    if (!waveform.get_time()) {  // equidistant
        if (!std::isfinite(frequency) || frequency < 1.0)
            throw std::invalid_argument("calculate_sampled_waveform: invalid frequency");
        time = linear_spaced_array(0, 1.0 / round_float(frequency, 9), data.size());
    } else {
        time = waveform.get_time().value();
        const double period = time.back() - time.front();
        if (period <= 0) throw std::invalid_argument("calculate_sampled_waveform: non-positive period");
        frequency = 1.0 / period;
    }

    size_t pts = numberPoints;
    if (data.size() > pts)  // never down-sample below the control points
        pts = ((data.size() & (data.size() - 1)) == 0) ? data.size() : round_up_to_power_of_2(data.size());

    std::vector<double> sampledTime = linear_spaced_array(0, 1.0 / round_float(frequency, 9), pts + 1);
    std::vector<double> sampledData;
    sampledData.reserve(pts);

    for (size_t i = 0; i < pts; ++i) {
        bool found = false;
        for (size_t j = 0; j + 1 < data.size(); ++j) {
            if (time[j + 1] == time[j]) {                    // zero-length segment (repeated time point)
                if (sampledTime[i] == time[j]) { sampledData.push_back(data[j]); found = true; break; }
                continue;
            }
            if (time[j] <= sampledTime[i] && sampledTime[i] <= time[j + 1]) {
                double p = (sampledTime[i] - time[j]) / (time[j + 1] - time[j]);
                sampledData.push_back(lerp_(data[j], data[j + 1], p));
                found = true; break;
            }
        }
        if (!found) throw std::invalid_argument("calculate_sampled_waveform: unsampled point " + std::to_string(i));
    }
    sampledTime.pop_back();

    MAS::Waveform out;
    out.set_data(sampledData);
    out.set_time(sampledTime);
    if (waveform.get_ancillary_label()) out.set_ancillary_label(waveform.get_ancillary_label().value());
    return out;
}

// Ported from MKF processors/Inputs.cpp::calculate_processed_data (the synthesized-waveform advanced
// path). Stats are computed directly from the resampled data + harmonics; the shape label is read from
// the waveform (we synthesized it) rather than guessed.
MAS::ProcessedWaveform calculate_processed_data(const MAS::Waveform& waveform, double frequency) {
    MAS::Waveform sampled = calculate_sampled_waveform(waveform, frequency);
    MAS::Harmonics harmonics = calculate_harmonics_data(sampled, frequency);
    const std::vector<double>& d = sampled.get_data();
    if (d.empty()) throw std::invalid_argument("calculate_processed_data: empty sampled waveform");

    double sum = 0, sumsq = 0, mx = d[0], mn = d[0];
    for (double v : d) { sum += v; sumsq += v * v; mx = std::max(mx, v); mn = std::min(mn, v); }
    const double avg = sum / static_cast<double>(d.size());

    MAS::ProcessedWaveform p;
    if (waveform.get_ancillary_label()) p.set_label(waveform.get_ancillary_label().value());
    p.set_average(avg);
    p.set_offset(avg);                                  // DC component == mean for a periodic waveform
    p.set_positive_peak(mx);
    p.set_negative_peak(mn);
    p.set_peak(std::max(std::abs(mx), std::abs(mn)));
    double pp = mx - mn;
    auto lbl = waveform.get_ancillary_label();
    if (lbl && (*lbl == MAS::WaveformLabel::FLYBACK_PRIMARY || *lbl == MAS::WaveformLabel::FLYBACK_SECONDARY))
        pp -= avg;                                      // MKF: the 0-floor overstates the ramp pk-pk
    p.set_peak_to_peak(pp);
    p.set_rms(std::sqrt(sumsq / static_cast<double>(d.size())));

    // Effective (RMS-weighted harmonic) frequencies + THD, from the harmonic spectrum.
    const auto& A = harmonics.get_amplitudes();
    const auto& F = harmonics.get_frequencies();
    auto eff_freq = [&](size_t startIdx) {
        double num = 0, den = 0;
        for (size_t i = startIdx; i < A.size(); ++i) { double a2 = A[i] * A[i]; num += a2 * F[i] * F[i]; den += a2; }
        return den > 0 ? std::sqrt(num / den) : 0.0;
    };
    p.set_effective_frequency(eff_freq(0));
    p.set_ac_effective_frequency(eff_freq(1));
    if (A.size() > 1 && A[1] > 0) {
        double num = 0;
        for (size_t i = 2; i < A.size(); ++i) num += A[i] * A[i];
        p.set_thd(std::sqrt(num) / A[1]);
    } else {
        p.set_thd(0.0);
    }
    return p;
}

// Ported from MKF converter_models/Topology.cpp::complete_excitation.
MAS::OperatingPointExcitation complete_excitation(const MAS::Waveform& currentWaveform,
                                                  const MAS::Waveform& voltageWaveform,
                                                  double switchingFrequency, const std::string& name) {
    if (switchingFrequency <= 0 || !std::isfinite(switchingFrequency))
        throw std::invalid_argument("complete_excitation: invalid switchingFrequency");

    auto build = [&](const MAS::Waveform& w) {
        MAS::SignalDescriptor s;
        MAS::Waveform sampled = calculate_sampled_waveform(w, switchingFrequency);
        s.set_waveform(sampled);                                            // store the power-of-2 waveform
        s.set_processed(calculate_processed_data(w, switchingFrequency));
        s.set_harmonics(calculate_harmonics_data(sampled, switchingFrequency));
        return s;
    };

    MAS::OperatingPointExcitation excitation;
    excitation.set_frequency(switchingFrequency);
    excitation.set_current(build(currentWaveform));
    excitation.set_voltage(build(voltageWaveform));
    excitation.set_name(name);
    return excitation;
}

} // namespace analytical
} // namespace Kirchhoff
