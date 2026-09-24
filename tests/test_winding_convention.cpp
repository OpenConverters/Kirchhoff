// Guard test for the MAS excitation convention (2026-09-24) on every multi-winding converter model.
//
// The convention every analytical operating point must satisfy:
//   * Reference winding r = the first winding with isolationSide "primary" (index 0 in every solver here).
//   * VOLTAGES in the common dot reference: v_k(t) = N_k dPhi/dt for ONE shared core flux, so every winding
//     voltage is in phase with the reference winding (positive together while the flux rises).
//   * CURRENTS: primary-side windings (incl. a demagnetisation/reset winding) PASSIVE (positive = into the
//     dotted terminal); every other winding SOURCE (positive = out of the dotted terminal).
//   * Hence i_m(t) = (sum_primary N_k i_k(t) - sum_other N_k i_k(t)) / N_r and v_r(t) = L_m di_m/dt.
//
// For each topology the test builds a representative design with a pinned L_m and pinned turns ratios and
// checks on the EMITTED operating point:
//   (a) every winding's fundamental voltage phasor lies within 15 deg of the reference winding's;
//   (b) Faraday at the fundamental: |V_r1| / (2 pi f0 |I_m1|) = L_m within 5 %, with I_m from the
//       ampere-turn identity above; V_r1 must also LEAD I_m1 by 90 deg (+-15 deg). Models of an IDEAL
//       transformer (no L_m: series-resonant converter, current transformer) instead require |I_m1| to be
//       below 1 % of the reference winding's |I_1|;
//   (c) the DC current each output draws from its windings equals its load current within 2 %:
//       sum over the output's unipolar windings of |<i_k>|, or, for bipolar (full-bridge / active-bridge)
//       windings, the power-equivalent current sum <v_k i_k> / V_out.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ConverterAnalytical.hpp"
#include "PwmBridgeSolver.h"
#include "Cllc.hpp"
#include "Clllc.hpp"
#include "ConverterExtract.hpp"
#include "Flyback.hpp"
#include "Weinberg.hpp"
#include "NgspiceRunner.hpp"
#include "KirchhoffApi.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

namespace AN = Kirchhoff::analytical;

namespace {

// One winding of the model under test, in emitted order.
// How a winding's contribution to its output's DC current is read (check (c)):
//   MEAN      |<i_k>|        — the winding charges the output capacitor directly (flyback-type, the halves of a
//                              centre-tapped diode rectifier without an output inductor);
//   RECTIFIED <|i_k|>        — a diode full bridge straight onto the output capacitor (resonant converters);
//   POWER     <v_k i_k>/Vout — the winding feeds an output INDUCTOR or an active bridge (forward family, push-
//                              pull, phase-shifted / asymmetric bridges, DAB): the load current then also flows
//                              through the freewheel path, so only the delivered power is a winding quantity.
enum class Metric { MEAN, RECTIFIED, POWER };

struct WindingSpec {
    double turnsToReference;   // N_k / N_r
    bool primarySide;          // isolationSide "primary" (passive current) vs other (source current)
    int output;                // index into CaseSpec::outputs, or -1 when the winding feeds no checked output
    Metric metric;
};

struct OutputSpec {
    double voltage;            // V_out (power-equivalent metric)
    double current;            // load current
};

struct CaseSpec {
    std::string name;
    MAS::OperatingPoint op;
    std::vector<WindingSpec> windings;
    std::vector<OutputSpec> outputs;
    double frequency;
    std::optional<double> magnetizingInductance;   // nullopt: ideal transformer model
};

struct CaseResult {
    double maxPhaseDeviationDeg = 0.0;
    double faradayRatio = std::numeric_limits<double>::quiet_NaN();     // |V_r1|/(w|I_m1|) / L_m
    double faradayPhaseDeg = std::numeric_limits<double>::quiet_NaN();  // arg(V_r1) - arg(I_m1)
    double idealMagnetizingFraction = std::numeric_limits<double>::quiet_NaN();  // |I_m1| / |I_r1|
    double maxAverageErrorPct = 0.0;
};

constexpr int kSamples = 8192;

// Piecewise-linear evaluation of a MAS waveform on a uniform grid over one period. A repeated time stamp is a
// step: the sample AT the step takes the value after it. A waveform without time is uniformly sampled.
std::vector<double> sample(const MAS::Waveform& w, double period) {
    const auto& data = w.get_data();
    if (data.size() < 2) throw std::runtime_error("sample: waveform has fewer than 2 points");
    std::vector<double> time;
    if (w.get_time()) {
        time = *w.get_time();
        if (time.size() != data.size()) throw std::runtime_error("sample: time/data length mismatch");
    } else {
        time.resize(data.size());
        for (size_t k = 0; k < data.size(); ++k) time[k] = period * static_cast<double>(k) / data.size();
    }
    std::vector<double> out(kSamples);
    size_t seg = 0;
    for (int j = 0; j < kSamples; ++j) {
        const double t = period * static_cast<double>(j) / kSamples;
        while (seg + 1 < time.size() && time[seg + 1] <= t) ++seg;
        if (seg + 1 >= time.size()) { out[j] = data.back(); continue; }
        const double t0 = time[seg], t1 = time[seg + 1];
        out[j] = (t1 > t0) ? data[seg] + (data[seg + 1] - data[seg]) * (t - t0) / (t1 - t0) : data[seg + 1];
    }
    return out;
}

std::complex<double> fundamental(const std::vector<double>& x) {
    std::complex<double> acc(0.0, 0.0);
    for (int j = 0; j < kSamples; ++j) {
        const double th = 2.0 * M_PI * static_cast<double>(j) / kSamples;
        acc += x[j] * std::complex<double>(std::cos(th), -std::sin(th));
    }
    return acc * (2.0 / kSamples);
}

double mean(const std::vector<double>& x) {
    double s = 0.0;
    for (double v : x) s += v;
    return s / static_cast<double>(x.size());
}

double wrap_deg(double a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

CaseResult evaluate(const CaseSpec& c) {
    const auto& exc = c.op.get_excitations_per_winding();
    REQUIRE(exc.size() == c.windings.size());
    const double period = 1.0 / c.frequency;
    std::vector<std::vector<double>> v(exc.size()), i(exc.size());
    for (size_t k = 0; k < exc.size(); ++k) {
        REQUIRE(exc[k].get_voltage());
        REQUIRE(exc[k].get_current());
        REQUIRE(exc[k].get_voltage()->get_waveform());
        REQUIRE(exc[k].get_current()->get_waveform());
        v[k] = sample(*exc[k].get_voltage()->get_waveform(), period);
        i[k] = sample(*exc[k].get_current()->get_waveform(), period);
    }

    if (const char* dump = std::getenv("KH_CONVENTION_DUMP"); dump && c.name == dump) {
        for (size_t k = 0; k < exc.size(); ++k) {
            for (const char* side : {"i", "v"}) {
                const MAS::Waveform w = side[0] == 'i' ? *exc[k].get_current()->get_waveform() : *exc[k].get_voltage()->get_waveform();
                std::printf("W%zu %s n=%zu:", k, side, w.get_data().size());
                for (size_t q = 0; q < w.get_data().size(); q += 4)
                    std::printf(" (%.3g,%.4g)", w.get_time() ? (*w.get_time())[q] : -1.0, w.get_data()[q]);
                std::printf("\n");
            }
        }
    }
    CaseResult r;
    const std::complex<double> V1r = fundamental(v[0]);
    for (size_t k = 1; k < exc.size(); ++k) {
        const std::complex<double> V1k = fundamental(v[k]);
        const double dev = std::abs(wrap_deg((std::arg(V1k) - std::arg(V1r)) * 180.0 / M_PI));
        r.maxPhaseDeviationDeg = std::max(r.maxPhaseDeviationDeg, dev);
    }

    std::vector<double> im(kSamples, 0.0);
    for (size_t k = 0; k < exc.size(); ++k) {
        const double sgn = c.windings[k].primarySide ? 1.0 : -1.0;
        for (int j = 0; j < kSamples; ++j) im[j] += sgn * c.windings[k].turnsToReference * i[k][j];
    }
    const std::complex<double> Im1 = fundamental(im);
    if (c.magnetizingInductance) {
        r.faradayRatio = std::abs(V1r) / (2.0 * M_PI * c.frequency * std::abs(Im1)) / *c.magnetizingInductance;
        r.faradayPhaseDeg = wrap_deg((std::arg(V1r) - std::arg(Im1)) * 180.0 / M_PI);
    } else {
        r.idealMagnetizingFraction = std::abs(Im1) / std::abs(fundamental(i[0]));
    }

    for (size_t o = 0; o < c.outputs.size(); ++o) {
        double delivered = 0.0;
        for (size_t k = 0; k < exc.size(); ++k) {
            if (c.windings[k].output != static_cast<int>(o)) continue;
            if (c.windings[k].metric == Metric::POWER) {
                std::vector<double> p(kSamples);
                for (int j = 0; j < kSamples; ++j) p[j] = v[k][j] * i[k][j];
                delivered += mean(p) / c.outputs[o].voltage;
            } else if (c.windings[k].metric == Metric::RECTIFIED) {
                std::vector<double> a(kSamples);
                for (int j = 0; j < kSamples; ++j) a[j] = std::abs(i[k][j]);
                delivered += mean(a);
            } else {
                delivered += std::abs(mean(i[k]));
            }
        }
        const double err = 100.0 * std::abs(delivered - c.outputs[o].current) / c.outputs[o].current;
        r.maxAverageErrorPct = std::max(r.maxAverageErrorPct, err);
    }
    return r;
}

void check(const CaseSpec& c) {
    const CaseResult r = evaluate(c);
    std::printf("[convention] %-28s phase-dev %7.2f deg | Faraday %8.4f (V-Im %7.1f deg) | ideal Im/I %8.4f | "
                "avg-err %7.2f %%\n",
                c.name.c_str(), r.maxPhaseDeviationDeg, r.faradayRatio, r.faradayPhaseDeg,
                r.idealMagnetizingFraction, r.maxAverageErrorPct);
    INFO(c.name);
    CHECK(r.maxPhaseDeviationDeg <= 15.0);
    if (c.magnetizingInductance) {
        CHECK(std::abs(r.faradayRatio - 1.0) <= 0.05);
        CHECK(std::abs(r.faradayPhaseDeg - 90.0) <= 15.0);
    } else {
        CHECK(r.idealMagnetizingFraction <= 0.01);
    }
    CHECK(r.maxAverageErrorPct <= 2.0);
}

}  // namespace

TEST_CASE("convention: flyback (CCM and DCM)", "[convention]") {
    for (double L : {200e-6, 20e-6}) {
        CaseSpec c;
        c.name = L > 100e-6 ? "flyback CCM" : "flyback DCM";
        c.frequency = 100e3;
        c.magnetizingInductance = L;
        c.op = AN::analytical_flyback(48.0, {12.0}, {2.0}, {2.0}, c.frequency, L, 0.0, 1.0);
        c.windings = {{1.0, true, -1, Metric::MEAN}, {0.5, false, 0, Metric::MEAN}};
        c.outputs = {{12.0, 2.0}};
        check(c);
    }
}

TEST_CASE("convention: flyback, two outputs", "[convention]") {
    CaseSpec c;
    c.name = "flyback 2-output";
    c.frequency = 100e3;
    c.magnetizingInductance = 200e-6;
    c.op = AN::analytical_flyback(48.0, {12.0, 5.0}, {2.0, 1.0}, {2.0, 4.8}, c.frequency, 200e-6, 0.0, 1.0);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {0.5, false, 0, Metric::MEAN}, {1.0 / 4.8, false, 1, Metric::MEAN}};
    c.outputs = {{12.0, 2.0}, {5.0, 1.0}};
    check(c);
}

TEST_CASE("convention: isolated buck, non-unity turns ratio and two secondaries", "[convention]") {
    CaseSpec c;
    c.name = "isolatedBuck 2-sec";
    c.frequency = 200e3;
    c.magnetizingInductance = 47e-6;
    c.op = AN::analytical_isolated_buck(48.0, 12.0, 0.5, std::vector<double>{6.0, 24.0},
                                        std::vector<double>{1.0, 0.25}, std::vector<double>{2.0, 0.5},
                                        c.frequency, 47e-6, 0.0, 1.0);
    c.windings = {{1.0, true, 0, Metric::MEAN}, {0.5, false, 1, Metric::MEAN}, {2.0, false, 2, Metric::MEAN}};
    c.outputs = {{12.0, 0.5}, {6.0, 1.0}, {24.0, 0.25}};
    check(c);
}

TEST_CASE("convention: single-switch forward", "[convention]") {
    CaseSpec c;
    c.name = "singleSwitchForward";
    c.frequency = 100e3;
    c.magnetizingInductance = 1e-3;
    c.op = AN::analytical_forward(48.0, {5.0}, {10.0}, {1.0, 4.0}, c.frequency, 1e-3, 20e-6, 0.2, 0.0);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0, true, -1, Metric::MEAN}, {0.25, false, 0, Metric::POWER}};
    c.outputs = {{5.0, 10.0}};
    check(c);
}

TEST_CASE("convention: two-switch forward", "[convention]") {
    CaseSpec c;
    c.name = "twoSwitchForward";
    c.frequency = 100e3;
    c.magnetizingInductance = 1e-3;
    c.op = AN::analytical_two_switch_forward(48.0, {5.0}, {10.0}, {4.0}, c.frequency, 1e-3, 20e-6, 0.2, 0.0);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {0.25, false, 0, Metric::POWER}};
    c.outputs = {{5.0, 10.0}};
    check(c);
}

TEST_CASE("convention: active-clamp forward", "[convention]") {
    CaseSpec c;
    c.name = "activeClampForward";
    c.frequency = 100e3;
    c.magnetizingInductance = 1e-3;
    c.op = AN::analytical_active_clamp_forward(48.0, {5.0}, {10.0}, {4.0}, c.frequency, 1e-3, 20e-6, 0.2, 0.0);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {0.25, false, 0, Metric::POWER}};
    c.outputs = {{5.0, 10.0}};
    check(c);
}

TEST_CASE("convention: push-pull", "[convention]") {
    CaseSpec c;
    c.name = "pushPull";
    c.frequency = 100e3;
    c.magnetizingInductance = 1e-3;
    c.op = AN::analytical_push_pull(48.0, std::vector<double>{5.0}, std::vector<double>{10.0}, std::vector<double>{4.0}, c.frequency, 1e-3, 20e-6, 0.2, 0.0);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0, true, -1, Metric::MEAN},
                  {0.25, false, 0, Metric::POWER}, {0.25, false, 0, Metric::POWER}};
    c.outputs = {{5.0, 10.0}};
    check(c);
}

TEST_CASE("convention: isolated buck (fly-buck)", "[convention]") {
    CaseSpec c;
    c.name = "isolatedBuck";
    c.frequency = 200e3;
    c.magnetizingInductance = 47e-6;
    c.op = AN::analytical_isolated_buck(48.0, 12.0, 0.5, std::vector<double>{12.0}, std::vector<double>{0.5},
                                        std::vector<double>{1.0}, c.frequency, 47e-6, 0.0, 1.0);
    c.windings = {{1.0, true, 0, Metric::MEAN}, {1.0, false, 1, Metric::MEAN}};
    c.outputs = {{12.0, 0.5}, {12.0, 0.5}};
    check(c);
}

TEST_CASE("convention: isolated buck-boost (fly-buck-boost)", "[convention]") {
    CaseSpec c;
    c.name = "isolatedBuckBoost";
    c.frequency = 200e3;
    c.magnetizingInductance = 47e-6;
    c.op = AN::analytical_isolated_buck_boost(24.0, 12.0, 0.5, std::vector<double>{12.0, 5.0},
                                              std::vector<double>{0.5, 1.0}, c.frequency, 47e-6,
                                              std::vector<double>{1.0, 2.4}, 0.0, 1.0);
    // The primary winding carries the input (switch) current as well, so only the isolated rails are checked.
    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0, false, 0, Metric::MEAN}, {1.0 / 2.4, false, 1, Metric::MEAN}};
    c.outputs = {{12.0, 0.5}, {5.0, 1.0}};
    check(c);
}

// Output voltage the phase-shifted bridge delivers at this phase shift: Vo = Deff·Vbus/n (the solver takes Vo
// as an input and does not close the loop, so the representative design picks the consistent one).
static double psb_output_voltage(double Vbus, double n, double phaseDeg, double Lr, double Io, double Fs) {
    namespace PBS = OpenMagnetics::PwmBridgeSolver;
    const double dcl = PBS::compute_duty_cycle_loss(Vbus, Lr, Io, n, Fs);
    return PBS::compute_effective_duty_cycle(phaseDeg / 180.0, dcl) * Vbus / n;
}

TEST_CASE("convention: phase-shifted full bridge (CT and FB)", "[convention]") {
    for (auto rect : {AN::SrcRectifier::CENTER_TAPPED, AN::SrcRectifier::FULL_BRIDGE}) {
        CaseSpec c;
        const bool ct = rect == AN::SrcRectifier::CENTER_TAPPED;
        c.name = ct ? "psfb CT" : "psfb FB";
        c.frequency = 100e3;
        c.magnetizingInductance = 2e-3;
        const double n = 16.0, Lr = 5e-6, Io = 20.0;
        const double Vo = psb_output_voltage(400.0, n, 120.0, Lr, Io, c.frequency);
        c.op = AN::analytical_psfb(400.0, {Vo}, {Io}, {n}, c.frequency, 2e-3, Lr, 10e-6, 120.0, 0.0, rect);
        if (ct) c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}, {1.0 / n, false, 0, Metric::POWER}};
        else    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}};
        c.outputs = {{Vo, Io}};
        check(c);
    }
}

TEST_CASE("convention: phase-shifted half bridge", "[convention]") {
    CaseSpec c;
    c.name = "pshb CT";
    c.frequency = 100e3;
    c.magnetizingInductance = 2e-3;
    const double n = 8.0, Lr = 5e-6, Io = 20.0;
    const double Vo = psb_output_voltage(200.0, n, 120.0, Lr, Io, c.frequency);
    c.op = AN::analytical_pshb(400.0, {Vo}, {Io}, {n}, c.frequency, 2e-3, Lr, 10e-6, 120.0, 0.0,
                               AN::SrcRectifier::CENTER_TAPPED);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}, {1.0 / n, false, 0, Metric::POWER}};
    c.outputs = {{Vo, Io}};
    check(c);
}

TEST_CASE("convention: asymmetric half bridge (CT, FB, multi-output)", "[convention]") {
    const double D = 0.35, n = 10.0;
    const double Vo = 2.0 * D * (1.0 - D) * 400.0 / n;   // volt-second-consistent output voltage
    {
        CaseSpec c;
        c.name = "ahb CT";
        c.frequency = 100e3;
        c.magnetizingInductance = 1e-3;
        c.op = AN::analytical_asymmetric_half_bridge(400.0, {Vo}, {10.0}, {n}, c.frequency, 1e-3, D, 0.2, 0.0,
                                                     AN::SrcRectifier::CENTER_TAPPED);
        c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}, {1.0 / n, false, 0, Metric::POWER}};
        c.outputs = {{Vo, 10.0}};
        check(c);
    }
    {
        CaseSpec c;
        c.name = "ahb FB";
        c.frequency = 100e3;
        c.magnetizingInductance = 1e-3;
        c.op = AN::analytical_asymmetric_half_bridge(400.0, {Vo}, {10.0}, {n}, c.frequency, 1e-3, D, 0.2, 0.0,
                                                     AN::SrcRectifier::FULL_BRIDGE);
        c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}};
        c.outputs = {{Vo, 10.0}};
        check(c);
    }
    {
        CaseSpec c;
        c.name = "ahb 2-output";
        c.frequency = 100e3;
        c.magnetizingInductance = 1e-3;
        c.op = AN::analytical_asymmetric_half_bridge(400.0, {Vo, Vo / 2}, {10.0, 4.0}, {n, 2 * n}, c.frequency,
                                                     1e-3, D, 0.2, 0.0, AN::SrcRectifier::FULL_BRIDGE);
        c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}, {1.0 / (2 * n), false, 1, Metric::POWER}};
        c.outputs = {{Vo, 10.0}, {Vo / 2, 4.0}};
        check(c);
    }
}

TEST_CASE("convention: dual active bridge", "[convention]") {
    CaseSpec c;
    c.name = "dab";
    c.frequency = 100e3;
    c.magnetizingInductance = 1e-3;
    const double n = 400.0 / 48.0, P = 480.0, D3 = 30.0 * M_PI / 180.0;
    const double Lr = AN::dab_series_inductance_for_power(400.0, 48.0, n, D3, 0.0, 0.0, c.frequency, P);
    c.op = AN::analytical_dab(400.0, {48.0}, {10.0}, {n}, c.frequency, 1e-3, Lr, 0.0, 0.0, 30.0);
    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::POWER}};
    c.outputs = {{48.0, 10.0}};
    check(c);
}

TEST_CASE("convention: series resonant converter (ideal transformer)", "[convention]") {
    for (auto rect : {AN::SrcRectifier::FULL_BRIDGE, AN::SrcRectifier::CENTER_TAPPED}) {
        const bool ct = rect == AN::SrcRectifier::CENTER_TAPPED;
        CaseSpec c;
        c.name = ct ? "src CT" : "src FB";
        const double Lr = 50e-6, Cr = 50e-9, n = 400.0 / 48.0;
        c.frequency = 1.1 / (2.0 * M_PI * std::sqrt(Lr * Cr));
        c.op = AN::analytical_src(400.0, {48.0}, {5.0}, {n}, c.frequency, Lr, Cr, 1.0, rect);
        if (ct) c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}};
        else    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::RECTIFIED}};
        c.outputs = {{48.0, 5.0}};
        check(c);
    }
}

TEST_CASE("convention: LLC (center-tapped and full-bridge rectifier)", "[convention]") {
    for (auto rect : {AN::SrcRectifier::CENTER_TAPPED, AN::SrcRectifier::FULL_BRIDGE}) {
        const bool ct = rect == AN::SrcRectifier::CENTER_TAPPED;
        CaseSpec c;
        c.name = ct ? "llc CT" : "llc FB";
        const double Ls = 50e-6, Cr = 50e-9, Lm = 300e-6, n = 0.5 * 400.0 / 12.0;
        c.frequency = 1.0 / (2.0 * M_PI * std::sqrt(Ls * Cr));
        c.magnetizingInductance = Lm;
        c.op = AN::analytical_llc(400.0, {12.0}, {20.0}, {n}, c.frequency, Lm, Ls, Cr, 0.5, rect);
        if (ct) c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}};
        else    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::RECTIFIED}};
        c.outputs = {{12.0, 20.0}};
        check(c);
    }
}

TEST_CASE("convention: CLLC and CLLLC", "[convention]") {
    const double n = 400.0 / 48.0, Lr1 = 50e-6, Cr1 = 50e-9, Lm = 300e-6;
    const double Lr2 = Lr1 / (n * n), Cr2 = Cr1 * n * n;
    const double fr = 1.0 / (2.0 * M_PI * std::sqrt(Lr1 * Cr1));
    for (int variant = 0; variant < 3; ++variant) {
        CaseSpec c;
        c.frequency = fr;
        c.magnetizingInductance = Lm;
        c.outputs = {{48.0, 10.0}};
        if (variant == 0) {
            c.name = "cllc FB";
            c.op = AN::analytical_cllc(400.0, {48.0}, {10.0}, {n}, fr, Lm, Lr1, Cr1, Lr2, Cr2, 1.0,
                                       AN::SrcRectifier::FULL_BRIDGE);
            c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::RECTIFIED}};
        } else if (variant == 1) {
            c.name = "cllc CT";
            c.op = AN::analytical_cllc(400.0, {48.0}, {10.0}, {n}, fr, Lm, Lr1, Cr1, Lr2, Cr2, 1.0,
                                       AN::SrcRectifier::CENTER_TAPPED);
            c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}};
        } else {
            c.name = "clllc FB";
            c.op = AN::analytical_clllc(400.0, {48.0}, {10.0}, {n}, fr, Lm, Lr1, Cr1, Lr2, Cr2, 1.0,
                                        AN::SrcRectifier::FULL_BRIDGE);
            c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0 / n, false, 0, Metric::RECTIFIED}};
        }
        check(c);
    }
}

TEST_CASE("convention: current transformer (ideal transformer)", "[convention]") {
    CaseSpec c;
    c.name = "currentTransformer";
    c.frequency = 50e3;
    c.op = AN::analytical_current_transformer(MAS::WaveformLabel::SINUSOIDAL, 10.0, c.frequency, 0.01, 10.0,
                                              0.0, 0.5, 0.0);
    // turnsRatio = Np/Ns = 0.01 -> N_s/N_p = 100; the burden is a resistor, so the output check is n/a.
    c.windings = {{1.0, true, -1, Metric::MEAN}, {100.0, false, -1, Metric::MEAN}};
    check(c);
}

TEST_CASE("convention: Weinberg transformer T1", "[convention]") {
    // analytical_weinberg emits [L1 a, L1 b, T1 primary a, T1 primary b, T1 secondary a, T1 secondary b]; T1
    // (windings 2..5) is the multi-winding transformer. Its model has no magnetizing inductance (ideal T1).
    // D = 0.75 (M = Vo/Vin = 1/(2n(1−D)) -> n = 2·Vin/Vo): every switching edge (T/4, T/2, 3T/4)
    // then falls on the 128-point uniform grid complete_excitation stores the waveform on, so the check measures
    // the model rather than where the resampling puts an off-grid step (an edge between samples shifts a
    // winding average by up to one sample, ~5 % on an 18-sample conduction window).
    const double n = 2.0 * 28.0 / 50.0;
    const MAS::OperatingPoint all = AN::analytical_weinberg(28.0, 50.0, 2.0, 100e3, 100e-6, n, 0.0, 1.0);
    const auto& exc = all.get_excitations_per_winding();
    REQUIRE(exc.size() == 6);
    CaseSpec c;
    c.name = "weinberg T1";
    c.frequency = 100e3;
    c.op.get_mutable_excitations_per_winding().assign(exc.begin() + 2, exc.end());
    c.windings = {{1.0, true, -1, Metric::MEAN}, {1.0, true, -1, Metric::MEAN},
                  {1.0 / n, false, 0, Metric::MEAN}, {1.0 / n, false, 0, Metric::MEAN}};
    c.outputs = {{50.0, 2.0}};
    check(c);

    // L1: the 1:1 input coupled inductor (both windings primary-side, same sense) — Faraday with L1.
    CaseSpec l1;
    l1.name = "weinberg L1";
    l1.frequency = 200e3;   // L1 is excited at 2·fsw (two overlaps per period); evaluate its fundamental there
    l1.magnetizingInductance = 100e-6;
    l1.op.get_mutable_excitations_per_winding().assign(exc.begin(), exc.begin() + 2);
    l1.windings = {{1.0, true, -1, Metric::MEAN}, {1.0, true, -1, Metric::MEAN}};
    check(l1);
}

// Reverse power flow (the Vout side drives): the builder swaps the solver's driver/receiver windings back onto
// the physical windings; the captured T1 operating point must still be in the MAS convention (primary-side
// winding PASSIVE even though it now delivers power).
namespace {
nlohmann::json cllc_inputs(const std::string& direction) {
    nlohmann::json d;
    d["designRequirements"]["efficiency"] = 1.0;
    d["designRequirements"]["inputVoltage"]["nominal"] = 400.0;
    d["designRequirements"]["inputVoltage"]["minimum"] = 380.0;
    d["designRequirements"]["inputVoltage"]["maximum"] = 420.0;
    d["designRequirements"]["switchingFrequency"]["nominal"] = 100000.0;
    nlohmann::json o; o["name"] = "out"; o["voltage"]["nominal"] = 48.0;
    d["designRequirements"]["outputs"] = nlohmann::json::array({o});
    nlohmann::json op; op["inputVoltage"] = 400.0;
    nlohmann::json oo; oo["power"] = 480.0;
    op["outputs"] = nlohmann::json::array({oo});
    d["operatingPoints"] = nlohmann::json::array({op});
    d["config"]["powerFlowDirection"] = direction;
    return d;
}

template <class Design>
void check_bidirectional_t1(const std::string& name, const Design& d, bool reverse) {
    const auto& reg = AN::captured_operating_points();
    auto it = std::find_if(reg.begin(), reg.end(), [](const auto& kv) { return kv.first == "T1"; });
    REQUIRE(it != reg.end());
    CaseSpec c;
    c.name = name;
    c.op = it->second.template get<MAS::OperatingPoint>();
    c.frequency = d.switchingFrequency;
    c.magnetizingInductance = d.magnetizingInductance;
    const double n = d.turnsRatio;
    if (reverse) {
        // The delivered load sits on the Vin (primary) side.
        c.windings = {{1.0, true, 0, Metric::RECTIFIED}, {1.0 / n, false, -1, Metric::RECTIFIED}};
        c.outputs = {{d.inputVoltage, d.outputPower / d.inputVoltage}};
    } else {
        c.windings = {{1.0, true, -1, Metric::RECTIFIED}, {1.0 / n, false, 0, Metric::RECTIFIED}};
        c.outputs = {{d.outputVoltage, d.outputPower / d.outputVoltage}};
    }
    check(c);
}
}  // namespace

TEST_CASE("convention: CLLC and CLLLC builders, forward and reverse power flow", "[convention]") {
    for (const std::string dir : {"forward", "reverse"}) {
        {
            const Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(cllc_inputs(dir));
            AN::clear_captured_operating_points();
            (void)Kirchhoff::build_cllc_tas(d);
            check_bidirectional_t1("cllc build " + dir, d, dir == "reverse");
        }
        {
            const Kirchhoff::ClllcDesign d = Kirchhoff::design_clllc(cllc_inputs(dir));
            AN::clear_captured_operating_points();
            (void)Kirchhoff::build_clllc_tas(d);
            check_bidirectional_t1("clllc build " + dir, d, dir == "reverse");
        }
    }
}

// The SIMULATED operating point (extract_operating_point, NGSPICE) follows the same convention: V(start) -
// V(end) is the dot reference, and the secondary branch current is negated into the SOURCE convention. A
// flyback secondary therefore averages -Iout, in phase with the primary voltage.
TEST_CASE("convention: simulated flyback operating point", "[convention][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        FAIL("libngspice is not linked into this build; the simulated-convention guard cannot run");
    }
    nlohmann::json spec;
    spec["designRequirements"]["efficiency"] = 1.0;
    spec["designRequirements"]["inputVoltage"] = {{"minimum", 43.2}, {"nominal", 48.0}, {"maximum", 52.8}};
    spec["designRequirements"]["switchingFrequency"]["nominal"] = 100000.0;
    spec["designRequirements"]["outputs"] = nlohmann::json::array({{{"name", "out"}, {"voltage", {{"nominal", 12.0}}}}});
    spec["operatingPoints"] = nlohmann::json::array({{{"inputVoltage", 48.0},
                                                      {"outputs", nlohmann::json::array({{{"power", 30.0}}})}}});
    const nlohmann::json tas = Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(spec));
    const MAS::OperatingPoint op = Kirchhoff::extract_operating_point(tas, Kirchhoff::ExtractEngine::NGSPICE);
    const auto& exc = op.get_excitations_per_winding();
    REQUIRE(exc.size() == 2);
    const double f = exc[0].get_frequency();
    const auto ip = sample(*exc[0].get_current()->get_waveform(), 1.0 / f);
    const auto is = sample(*exc[1].get_current()->get_waveform(), 1.0 / f);
    const auto vp = sample(*exc[0].get_voltage()->get_waveform(), 1.0 / f);
    const auto vs = sample(*exc[1].get_voltage()->get_waveform(), 1.0 / f);
    const double phaseDev = std::abs(wrap_deg((std::arg(fundamental(vs)) - std::arg(fundamental(vp))) * 180.0 / M_PI));
    const double secondaryAverage = mean(is);
    std::printf("[convention] %-28s phase-dev %7.2f deg | primary <i> %7.3f A | secondary <i> %7.3f A (Iout 2.5 A)\n",
                "flyback simulated", phaseDev, mean(ip), secondaryAverage);
    CHECK(phaseDev <= 15.0);
    CHECK(mean(ip) > 0.0);                                             // passive primary: input current in
    CHECK(secondaryAverage == Catch::Approx(-2.5).epsilon(0.10));      // source secondary: -Iout
}

// Orientation against the ngspice deck: the analytical excitations the TAS embeds must use the SAME dots as the
// deck (the TAS "start" pins), or a consumer of the magnetic sees inverted windings. For every multi-winding
// magnetic of every transformer topology, each winding's analytical voltage and current must correlate
// POSITIVELY with the simulated V(start) − V(end) and (convention-signed) branch current over one period.
// Correlation, not equality: the FHA / ideal-commutation shapes differ in detail from the deck. The simulated
// waveform is clipped to 1.5x the analytical peak first, so the deck's commutation/snubber spikes (tens of amps
// for a few ns on a 0.8 A push-pull primary) cannot dominate the correlation either way.
namespace {
double correlation(const std::vector<double>& a, std::vector<double> b) {
    double peak = 0;
    for (double x : a) peak = std::max(peak, std::abs(x));
    for (auto& x : b) x = std::clamp(x, -1.5 * peak, 1.5 * peak);
    const double ma = mean(a), mb = mean(b);
    double sab = 0, saa = 0, sbb = 0;
    for (size_t j = 0; j < a.size(); ++j) {
        sab += (a[j] - ma) * (b[j] - mb); saa += (a[j] - ma) * (a[j] - ma); sbb += (b[j] - mb) * (b[j] - mb);
    }
    if (saa <= 0 || sbb <= 0) return std::numeric_limits<double>::quiet_NaN();
    return sab / std::sqrt(saa * sbb);
}
nlohmann::json orientation_spec(double vin, double vout, double pout, double fs, bool secondRail) {
    nlohmann::json d;
    d["designRequirements"]["efficiency"] = 1.0;
    d["designRequirements"]["inputVoltage"] = {{"nominal", vin}, {"minimum", vin * 0.95}, {"maximum", vin * 1.05}};
    d["designRequirements"]["switchingFrequency"]["nominal"] = fs;
    nlohmann::json o; o["name"] = "out"; o["voltage"]["nominal"] = vout;
    d["designRequirements"]["outputs"] = nlohmann::json::array({o});
    nlohmann::json op; op["inputVoltage"] = vin; nlohmann::json oo; oo["power"] = pout;
    op["outputs"] = nlohmann::json::array({oo});
    if (secondRail) {   // flybuck / fly-buck-boost: the isolated rail mirrors the primary rail
        nlohmann::json o2; o2["name"] = "vsec"; o2["voltage"]["nominal"] = vout;
        d["designRequirements"]["outputs"].push_back(o2);
        op["outputs"].push_back(oo);
    }
    d["operatingPoints"] = nlohmann::json::array({op});
    d["simStimulusFsw"] = nlohmann::json::array({fs});
    return d;
}
}  // namespace

TEST_CASE("convention: analytical winding orientation matches the ngspice deck", "[convention][ngspice][orientation]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        FAIL("libngspice is not linked into this build; the orientation cross-check cannot run");
    }
    struct Point { const char* topo; double vin, vout, pout, fs; bool secondRail; };
    const std::vector<Point> points = {
        {"flyback", 48.0, 12.0, 30.0, 100e3, false},         {"forward", 48.0, 5.0, 50.0, 200e3, false},
        {"two_switch_forward", 48.0, 12.0, 96.0, 250e3, false}, {"push_pull", 12.0, 5.0, 5.0, 200e3, false},
        {"acf", 48.0, 12.0, 192.0, 250e3, false},             {"ahb", 100.0, 12.0, 192.0, 100e3, false},
        {"psfb", 400.0, 24.0, 1200.0, 100e3, false},          {"pshb", 400.0, 24.0, 1200.0, 100e3, false},
        {"dab", 400.0, 48.0, 1920.0, 80e3, false},            {"llc", 400.0, 24.0, 240.0, 100e3, false},
        {"src", 400.0, 48.0, 480.0, 110e3, false},            {"cllc", 400.0, 48.0, 480.0, 200e3, false},
        {"clllc", 400.0, 48.0, 480.0, 200e3, false},          {"isolated_buck", 48.0, 5.0, 2.5, 200e3, true},
        {"isolated_buck_boost", 24.0, 12.0, 12.0, 100e3, true}, {"weinberg", 50.0, 150.0, 1500.0, 50e3, false},
    };
    for (const auto& p : points) {
        const std::string out = Kirchhoff::api::design_tas(p.topo, orientation_spec(p.vin, p.vout, p.pout, p.fs,
                                                                                     p.secondRail).dump());
        INFO(p.topo << ": " << out.substr(0, 300));
        REQUIRE(out.rfind("Exception", 0) != 0);
        const nlohmann::json tas = nlohmann::json::parse(out);
        for (const auto& mag : Kirchhoff::topology_waveforms(tas)) {
            const MAS::OperatingPoint a = Kirchhoff::extract_operating_point(tas, Kirchhoff::ExtractEngine::ANALYTICAL, mag.name);
            if (a.get_excitations_per_winding().size() < 2) continue;
            const MAS::OperatingPoint s = Kirchhoff::extract_operating_point(tas, Kirchhoff::ExtractEngine::NGSPICE, mag.name);
            REQUIRE(a.get_excitations_per_winding().size() == s.get_excitations_per_winding().size());
            for (size_t w = 0; w < a.get_excitations_per_winding().size(); ++w) {
                const auto& ea = a.get_excitations_per_winding()[w];
                const auto& es = s.get_excitations_per_winding()[w];
                const double T = 1.0 / ea.get_frequency();
                const double cv = correlation(sample(*ea.get_voltage()->get_waveform(), T), sample(*es.get_voltage()->get_waveform(), T));
                const double ci = correlation(sample(*ea.get_current()->get_waveform(), T), sample(*es.get_current()->get_waveform(), T));
                std::printf("[orientation] %-20s %-4s w%zu  corr(v) %6.3f  corr(i) %6.3f\n", p.topo, mag.name.c_str(), w, cv, ci);
                if (const char* dump = std::getenv("KH_ORIENT_DUMP"); dump && std::string(p.topo) == dump) {
                    const auto ia = sample(*ea.get_current()->get_waveform(), T), is = sample(*es.get_current()->get_waveform(), T);
                    for (int j = 0; j < kSamples; j += kSamples / 32) std::printf("   j=%5d i a/s %8.3f %8.3f\n", j, ia[j], is[j]);
                }
                INFO(p.topo << " " << mag.name << " winding " << w << ": corr(v) = " << cv << ", corr(i) = " << ci);
                CHECK(cv > 0.3);
                CHECK(ci > 0.3);
            }
        }
    }
}
