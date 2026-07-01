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

// Real fidelity gate for the (now FHA) LLC solver: drive analytical + ngspice from the SAME design at the
// deck operating point (fr), read the ACTUAL settled Vout from SPICE, and compare the resonant tank
// current (analytical "Primary" = i_Ls vs ngspice l.xllccell.llr_pri) via the phase-invariant NRMSE. FHA
// is load-aware (Rac ∝ n²·Rload) and NOT singular at fr, so this is a genuine analytical-vs-SPICE check.
TEST_CASE("NRMSE gate: LLC tank current — analytical vs ngspice", "[nrmse][llc]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping NRMSE gate");
        return;
    }
    json spec = spec_for(400, 12, 120, 100000);   // Telecom-120W reference
    Kirchhoff::LlcDesign d = Kirchhoff::design_llc(spec);
    const double fdrive = d.resonantFrequency;    // the deck runs at fr

    // --- ngspice first: settle the real circuit and read the actual operating point ---
    json tas = Kirchhoff::build_llc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);
    const double period = 1.0 / fdrive;
    const double vout = r.average("v(vout)", r.time.back() - period, r.time.back()).value_or(0.0);
    REQUIRE(vout > 0.5);
    const double rload = d.outputVoltage / (d.outputPower / d.outputVoltage);
    const double iout = vout / rload;

    // --- analytical (half-bridge LLC, center-tapped) at the SAME operating point ---
    MAS::OperatingPoint op = Kirchhoff::analytical::analytical_llc(
        d.inputVoltage, {vout}, {iout}, {d.turnsRatio},
        fdrive, d.magnetizingInductance, d.resonantInductance, d.resonantCapacitance,
        0.5, Kirchhoff::analytical::SrcRectifier::CENTER_TAPPED);
    REQUIRE(op.get_excitations_per_winding().size() >= 1);
    auto pri = op.get_excitations_per_winding()[0].get_current();
    REQUIRE(pri.has_value());
    REQUIRE(pri->get_waveform().has_value());
    const std::vector<double> aVal = pri->get_waveform()->get_data();
    REQUIRE(aVal.size() >= 2);
    std::vector<double> aTime(aVal.size());
    for (size_t i = 0; i < aVal.size(); ++i)
        aTime[i] = period * static_cast<double>(i) / static_cast<double>(aVal.size() - 1);

    const std::string tankKey = "l.xllccell.llr_pri#branch";
    REQUIRE(r.vectors.count(tankKey) == 1);
    const std::vector<double>& sVal = r.vectors.at(tankKey);

    const int N = 256;
    std::vector<double> aRes = resample_analytical(aTime, aVal, period, N);
    std::vector<double> sRes = resample_last_period(r.time, sVal, period, N);
    double nrmse = phase_invariant_nrmse(aRes, sRes);
    std::cerr << "[NRMSE] LLC tank current = " << nrmse << "  (Vout=" << vout << " Iout=" << iout
              << ", analytical Ipk=" << *std::max_element(aRes.begin(), aRes.end())
              << ", ngspice Ipk=" << *std::max_element(sRes.begin(), sRes.end()) << ")\n";
    // Load-aware FHA: the primary tank current tracks SPICE tightly (observed ~0.03). Gate at 0.15 (SRC's
    // level) — a real fidelity gate, not a characterization catch.
    CHECK(nrmse < 0.15);
    // The secondary winding-pair must deliver the DC output current (sum of the CT half-winding averages).
    double secSum = 0.0;
    for (size_t w = 1; w < op.get_excitations_per_winding().size(); ++w)
        secSum += *op.get_excitations_per_winding()[w].get_current()->get_processed()->get_average();
    CHECK(secSum == Catch::Approx(iout).margin(0.1 * iout));
}

// Real fidelity gate for the (now FHA) CLLC solver: drive analytical + ngspice from the same design at the
// deck operating point (fr), read the settled Vout from SPICE, and compare the primary resonant tank
// current (analytical "Primary" = i_Lr1 vs ngspice l.xcllccell.llr1_pri) via the phase-invariant NRMSE.
// FHA is load-aware (Rac ∝ n²·Rload) and not singular at fr — a genuine analytical-vs-SPICE fidelity check.
TEST_CASE("NRMSE gate: CLLC tank current — analytical vs ngspice", "[nrmse][cllc]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping NRMSE gate");
        return;
    }
    json spec = spec_for(400, 48, 480, 100000);
    Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(spec);
    const double fdrive = d.resonantFrequency;

    json tas = Kirchhoff::build_cllc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);
    const double period = 1.0 / fdrive;
    const double vout = r.average("v(vout)", r.time.back() - period, r.time.back()).value_or(0.0);
    REQUIRE(vout > 0.5);
    const double rload = d.outputVoltage / (d.outputPower / d.outputVoltage);
    const double iout = vout / rload;

    MAS::OperatingPoint op = Kirchhoff::analytical::analytical_cllc(
        d.inputVoltage, {vout}, {iout}, {d.turnsRatio},
        fdrive, d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
        d.secondaryResonantInductance, d.secondaryResonantCapacitance, 1.0);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    auto pri = op.get_excitations_per_winding()[0].get_current();
    REQUIRE(pri.has_value());
    REQUIRE(pri->get_waveform().has_value());
    const std::vector<double> aVal = pri->get_waveform()->get_data();
    REQUIRE(aVal.size() >= 2);
    std::vector<double> aTime(aVal.size());
    for (size_t i = 0; i < aVal.size(); ++i)
        aTime[i] = period * static_cast<double>(i) / static_cast<double>(aVal.size() - 1);

    const std::string tankKey = "l.xcllccell.llr1_pri#branch";
    REQUIRE(r.vectors.count(tankKey) == 1);
    const std::vector<double>& sVal = r.vectors.at(tankKey);

    const int N = 256;
    std::vector<double> aRes = resample_analytical(aTime, aVal, period, N);
    std::vector<double> sRes = resample_last_period(r.time, sVal, period, N);
    double nrmse = phase_invariant_nrmse(aRes, sRes);
    std::cerr << "[NRMSE] CLLC tank current = " << nrmse << "  (Vout=" << vout << " Iout=" << iout
              << ", analytical Ipk=" << *std::max_element(aRes.begin(), aRes.end())
              << ", ngspice Ipk=" << *std::max_element(sRes.begin(), sRes.end()) << ")\n";
    // Load-aware FHA of the two-sided CLLC tank: the primary tank current tracks SPICE within ~25%
    // (observed ~0.25; the residual is the fundamental-only approximation + the full-bridge harmonics +
    // the lossless overestimate). Gate at 0.35 — a genuine fidelity gate that catches the 5x load-blind
    // TDA regression, not a characterization catch.
    CHECK(nrmse < 0.35);
    // The full-bridge secondary winding is bipolar (zero mean); its rectified average delivers the DC output.
    const auto& secW = op.get_excitations_per_winding().back();
    const std::vector<double> secData = secW.get_current()->get_waveform()->get_data();
    double rectAvg = 0.0;
    for (double x : secData) rectAvg += std::abs(x);
    rectAvg /= secData.size();
    CHECK(rectAvg == Catch::Approx(iout).margin(0.12 * iout));
}

// Real fidelity gate for the (now FHA) CLLLC solver. build_clllc_tas emits a deck with method=gear that
// stalls after ~40 points on the CLLLC netlist (a deck-integration issue, NOT the solver); method=trap
// converges the full 400 periods, so we swap it here. Drive analytical + ngspice from the same design at
// fr, read the settled Vout, compare the primary tank current (analytical "Primary" = i_Lr1 vs ngspice
// l.xclllcpower.llr1_pri) via the phase-invariant NRMSE. FHA is load-aware and not singular at fr.
TEST_CASE("NRMSE gate: CLLLC tank current — analytical vs ngspice", "[nrmse][clllc]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping NRMSE gate");
        return;
    }
    json spec = spec_for(400, 48, 480, 100000);
    Kirchhoff::ClllcDesign d = Kirchhoff::design_clllc(spec);
    const double fdrive = d.resonantFrequency;

    json tas = Kirchhoff::build_clllc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    deck = std::regex_replace(deck, std::regex("method=gear"), "method=trap");   // converge the CLLLC netlist
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);
    REQUIRE(r.time.size() > 1000);   // guard against the gear-method 40-point stall
    const double period = 1.0 / fdrive;
    const double vout = r.average("v(vout)", r.time.back() - period, r.time.back()).value_or(0.0);
    REQUIRE(vout > 0.5);
    const double rload = d.outputVoltage / (d.outputPower / d.outputVoltage);
    const double iout = vout / rload;

    MAS::OperatingPoint op = Kirchhoff::analytical::analytical_clllc(
        d.inputVoltage, {vout}, {iout}, {d.turnsRatio},
        fdrive, d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
        d.secondaryResonantInductance, d.secondaryResonantCapacitance, 1.0);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    auto pri = op.get_excitations_per_winding()[0].get_current();
    REQUIRE(pri.has_value());
    REQUIRE(pri->get_waveform().has_value());
    const std::vector<double> aVal = pri->get_waveform()->get_data();
    REQUIRE(aVal.size() >= 2);
    std::vector<double> aTime(aVal.size());
    for (size_t i = 0; i < aVal.size(); ++i)
        aTime[i] = period * static_cast<double>(i) / static_cast<double>(aVal.size() - 1);

    const std::string tankKey = "l.xclllcpower.llr1_pri#branch";
    REQUIRE(r.vectors.count(tankKey) == 1);
    const std::vector<double>& sVal = r.vectors.at(tankKey);

    const int N = 256;
    std::vector<double> aRes = resample_analytical(aTime, aVal, period, N);
    std::vector<double> sRes = resample_last_period(r.time, sVal, period, N);
    double nrmse = phase_invariant_nrmse(aRes, sRes);
    std::cerr << "[NRMSE] CLLLC tank current = " << nrmse << "  (Vout=" << vout << " Iout=" << iout
              << ", analytical Ipk=" << *std::max_element(aRes.begin(), aRes.end())
              << ", ngspice Ipk=" << *std::max_element(sRes.begin(), sRes.end()) << ")\n";
    // Observed ~0.025 — the load-aware FHA tracks the symmetric CLLLC tank tightly. Gate at 0.15.
    CHECK(nrmse < 0.15);
    // Full-bridge secondary is bipolar; its rectified average delivers the DC output.
    const std::vector<double> secData = op.get_excitations_per_winding().back().get_current()->get_waveform()->get_data();
    double rectAvg = 0.0;
    for (double x : secData) rectAvg += std::abs(x);
    rectAvg /= secData.size();
    CHECK(rectAvg == Catch::Approx(iout).margin(0.12 * iout));
}

// Resonant SECONDARY-winding fidelity vs SPICE, measured on the SETTLED last period (an earlier audit
// wrongly used rms over the whole transient, which reads artificially low). LLC/CLLLC secondaries match
// SPICE tightly; CLLC is looser because its design runs off-unity-gain (n*Vout=370 != Vin=400), the
// regime where the single-sinusoid FHA is weakest (CLLLC, identical formula but n*Vout=Vin, is exact).
TEST_CASE("NRMSE gate: resonant secondary winding rms vs ngspice", "[nrmse][secondary]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice"); return; }
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    auto settledRms = [](const std::vector<double>& t, const std::vector<double>& v, double T) {
        const int M = 512; double te = t.back(), tb = te - T; size_t j = 0; double q = 0;
        for (int k = 0; k < M; ++k) { double tt = tb + T*k/M; while (j+1<t.size() && t[j+1]<tt) ++j;
            double f = (t[j+1]-t[j]>0)?(tt-t[j])/(t[j+1]-t[j]):0; double x = v[j]+f*(v[j+1]-v[j]); q += x*x; }
        return std::sqrt(q/M);
    };
    auto anaRms = [](const MAS::OperatingPoint& op, size_t w) {
        return *op.get_excitations_per_winding()[w].get_current()->get_processed()->get_rms();
    };
    // LLC (center-tapped: analytical "Secondary 0 Half 1" vs spice lt1_sec1)
    {
        auto d = Kirchhoff::design_llc(spec_for(400,12,120,100000)); double f = d.resonantFrequency, T = 1.0/f;
        auto r = Kirchhoff::run_ngspice_in_process(Kirchhoff::tas_to_ngspice(Kirchhoff::build_llc_tas(d), ideal));
        REQUIRE(r.success);
        double vo = r.average("v(vout)", r.time.back()-T, r.time.back()).value_or(0);
        double io = vo/(d.outputVoltage/(d.outputPower/d.outputVoltage));
        auto op = Kirchhoff::analytical::analytical_llc(d.inputVoltage,{vo},{io},{d.turnsRatio},f,d.magnetizingInductance,d.resonantInductance,d.resonantCapacitance,0.5,Kirchhoff::analytical::SrcRectifier::CENTER_TAPPED);
        double ratio = anaRms(op,1) / settledRms(r.time, r.vectors.at("l.xllccell.lt1_sec1#branch"), T);
        std::cerr << "[SEC] LLC secondary rms ratio = " << ratio << "\n";
        CHECK(ratio == Catch::Approx(1.0).margin(0.12));
    }
    // CLLLC (full-bridge: analytical "Secondary 0" vs spice lt1_sec1; trap deck)
    {
        auto d = Kirchhoff::design_clllc(spec_for(400,48,480,100000)); double f = d.resonantFrequency, T = 1.0/f;
        auto deck = std::regex_replace(Kirchhoff::tas_to_ngspice(Kirchhoff::build_clllc_tas(d), ideal), std::regex("method=gear"), "method=trap");
        auto r = Kirchhoff::run_ngspice_in_process(deck); REQUIRE(r.success);
        double vo = r.average("v(vout)", r.time.back()-T, r.time.back()).value_or(0);
        double io = vo/(d.outputVoltage/(d.outputPower/d.outputVoltage));
        auto op = Kirchhoff::analytical::analytical_clllc(d.inputVoltage,{vo},{io},{d.turnsRatio},f,d.magnetizingInductance,d.primaryResonantInductance,d.primaryResonantCapacitance,d.secondaryResonantInductance,d.secondaryResonantCapacitance,1.0);
        double ratio = anaRms(op,1) / settledRms(r.time, r.vectors.at("l.xclllcpower.lt1_sec1#branch"), T);
        std::cerr << "[SEC] CLLLC secondary rms ratio = " << ratio << "\n";
        CHECK(ratio == Catch::Approx(1.0).margin(0.12));
    }
    // CLLC (off-unity design → looser; bounded, not tight)
    {
        auto d = Kirchhoff::design_cllc(spec_for(400,48,480,100000)); double f = d.resonantFrequency, T = 1.0/f;
        auto r = Kirchhoff::run_ngspice_in_process(Kirchhoff::tas_to_ngspice(Kirchhoff::build_cllc_tas(d), ideal)); REQUIRE(r.success);
        double vo = r.average("v(vout)", r.time.back()-T, r.time.back()).value_or(0);
        double io = vo/(d.outputVoltage/(d.outputPower/d.outputVoltage));
        auto op = Kirchhoff::analytical::analytical_cllc(d.inputVoltage,{vo},{io},{d.turnsRatio},f,d.magnetizingInductance,d.primaryResonantInductance,d.primaryResonantCapacitance,d.secondaryResonantInductance,d.secondaryResonantCapacitance,1.0);
        double ratio = anaRms(op,1) / settledRms(r.time, r.vectors.at("l.xcllccell.lt1_sec1#branch"), T);
        std::cerr << "[SEC] CLLC secondary rms ratio = " << ratio << "  (off-unity FHA limit)\n";
        CHECK(ratio == Catch::Approx(1.0).margin(0.25));
    }
}
