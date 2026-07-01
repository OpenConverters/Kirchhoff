// NRMSE gate — cross-validate the ConverterAnalytical waveform solvers against the ngspice
// steady-state waveforms, per winding, via a PHASE-INVARIANT normalized RMS error. Resonant
// converters have no simple closed form, so SPICE is the ground truth; this gate is what lets us
// trust the FHA/TDA resonant solvers (SRC now, LLC next). Skipped (surfaced, not silently passed)
// when Kirchhoff is built without libngspice.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "ConverterAnalytical.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

json spec_for(double vin, double vout, double power, double fsw, double eta = 1.0) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name", "out"}, {"voltage", {{"nominal", vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}} });
    return s;
}

// Resample a (possibly non-uniform, multi-period) ngspice waveform onto N uniform points over the LAST
// steady-state period [tstop - T, tstop]. Linear interpolation.
std::vector<double> resample_last_period(const std::vector<double>& time, const std::vector<double>& val,
                                         double period, int N) {
    std::vector<double> out(N, 0.0);
    if (time.size() < 2) return out;
    const double tEnd = time.back();
    const double tBeg = tEnd - period;
    size_t j = 0;
    for (int k = 0; k < N; ++k) {
        double t = tBeg + (period * k) / N;
        while (j + 1 < time.size() && time[j + 1] < t) ++j;
        if (j + 1 >= time.size()) { out[k] = val.back(); continue; }
        double t0 = time[j], t1 = time[j + 1];
        double f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
        out[k] = val[j] + f * (val[j + 1] - val[j]);
    }
    return out;
}

// Sample a closed analytical waveform (time in [0, period], last point == first) onto N uniform points.
std::vector<double> resample_analytical(const std::vector<double>& time, const std::vector<double>& val,
                                        double period, int N) {
    std::vector<double> out(N, 0.0);
    if (time.size() < 2) return out;
    size_t j = 0;
    for (int k = 0; k < N; ++k) {
        double t = (period * k) / N;
        while (j + 1 < time.size() && time[j + 1] < t) ++j;
        if (j + 1 >= time.size()) { out[k] = val.back(); continue; }
        double t0 = time[j], t1 = time[j + 1];
        double f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
        out[k] = val[j] + f * (val[j + 1] - val[j]);
    }
    return out;
}

// Phase-invariant NRMSE between two N-sample periodic sequences: minimize RMS(a_shift - b) over all
// circular shifts of a, normalized by the RMS of b (the ngspice reference). Both are assumed to cover
// exactly one period. Returns the minimum normalized RMS error (0 = identical shape+scale).
double phase_invariant_nrmse(const std::vector<double>& a, const std::vector<double>& b) {
    const int N = static_cast<int>(b.size());
    if (N == 0 || a.size() != b.size()) return std::numeric_limits<double>::infinity();
    double sumSq = 0.0;
    for (double x : b) sumSq += x * x;
    double rmsB = std::sqrt(sumSq / N);
    if (rmsB < 1e-12) rmsB = 1.0;   // reference is ~0 → normalize by 1 (absolute)
    double best = std::numeric_limits<double>::infinity();
    for (int s = 0; s < N; ++s) {
        double acc = 0.0;
        for (int k = 0; k < N; ++k) {
            double d = a[(k + s) % N] - b[k];
            acc += d * d;
        }
        double nrmse = std::sqrt(acc / N) / rmsB;
        if (nrmse < best) best = nrmse;
    }
    return best;
}

}  // namespace

// Run the analytical SRC solver and the ngspice deck for the same design; compare the resonant tank
// current (analytical "Primary" winding vs the Lr branch current) via the phase-invariant NRMSE.
TEST_CASE("NRMSE gate: SRC tank current — analytical vs ngspice", "[nrmse][src]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping NRMSE gate");
        return;
    }
    json spec = spec_for(400, 48, 480, 100000);
    Kirchhoff::SrcDesign d = Kirchhoff::design_src(spec);
    const double iout = d.outputPower / d.outputVoltage;

    // --- analytical (SRC is a half-bridge drive with a center-tapped rectifier) ---
    MAS::OperatingPoint op = Kirchhoff::analytical::analytical_src(
        d.inputVoltage, {d.outputVoltage}, {iout}, {d.turnsRatio},
        d.switchingFrequency, d.resonantInductance, d.resonantCapacitance,
        0.5, Kirchhoff::analytical::SrcRectifier::CENTER_TAPPED);
    REQUIRE(op.get_excitations_per_winding().size() >= 1);
    auto pri = op.get_excitations_per_winding()[0].get_current();
    REQUIRE(pri.has_value());
    REQUIRE(pri->get_waveform().has_value());
    const std::vector<double> aVal = pri->get_waveform()->get_data();
    REQUIRE(aVal.size() >= 2);
    // The waveform covers exactly one switching period. Use its explicit time vector when present,
    // else synthesize a uniform grid [0, period] (the stored waveform may carry data only).
    std::vector<double> aTime;
    if (pri->get_waveform()->get_time().has_value() &&
        pri->get_waveform()->get_time()->size() == aVal.size()) {
        aTime = pri->get_waveform()->get_time().value();
    } else {
        const double period0 = 1.0 / d.switchingFrequency;
        aTime.resize(aVal.size());
        for (size_t i = 0; i < aVal.size(); ++i)
            aTime[i] = period0 * static_cast<double>(i) / static_cast<double>(aVal.size() - 1);
    }

    // --- ngspice ---
    json tas = Kirchhoff::build_src_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);
    const std::string tankKey = "l.xsrccell.llr_pri#branch";
    REQUIRE(r.vectors.count(tankKey) == 1);
    const std::vector<double>& sVal = r.vectors.at(tankKey);

    const int N = 256;
    const double period = 1.0 / d.switchingFrequency;
    std::vector<double> aRes = resample_analytical(aTime, aVal, period, N);
    std::vector<double> sRes = resample_last_period(r.time, sVal, period, N);

    double nrmse = phase_invariant_nrmse(aRes, sRes);
    std::cerr << "[NRMSE] SRC tank current = " << nrmse
              << "  (analytical Ipk=" << *std::max_element(aRes.begin(), aRes.end())
              << ", ngspice Ipk=" << *std::max_element(sRes.begin(), sRes.end()) << ")\n";
    // Observed ~0.15: the FHA sinusoid tracks the real SPICE tank-current shape well at resonance
    // (the residual is the harmonic content FHA omits + the ~14% peak headroom). Gate at 0.25 — a ~1.7x
    // margin that still catches a grossly wrong shape/scale (a regression or a mis-parameterized tank).
    CHECK(nrmse < 0.25);
}
