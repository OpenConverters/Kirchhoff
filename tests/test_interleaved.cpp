// Interleaved (multiphase) buck and boost (ABT #1437): config.phaseCount = N replicates the switching cell N
// times on shared rails with gates staggered by 360/N. Checked here: the TAS transform itself, the ripple-
// cancellation sizing of Cout, and — in ngspice — that the N phases share the load equally, are staggered by
// T/N, follow the per-phase analytical waveform, and that the output ripple lands within its sizing target.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include "Buck.hpp"
#include "Boost.hpp"
#include "Interleave.hpp"
#include "ConverterAnalytical.hpp"
#include "TasAssembler.hpp"
#include "NgspiceRunner.hpp"
#include "Fidelity.hpp"
#include "ComponentRequirements.hpp"

using nlohmann::json;
using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

namespace {

json spec(double vin, double vout, double power, double fsw, json config = json::object()) {
    json s;
    s["designRequirements"]["efficiency"] = 1.0;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({{{"name", "out"}, {"voltage", {{"nominal", vout}}}}});
    s["operatingPoints"] = json::array({{{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}}});
    if (!config.empty()) s["config"] = config;
    return s;
}

const json& stage_named(const json& tas, const std::string& name) {
    for (const auto& st : tas.at("topology").at("stages")) if (st.at("name") == name) return st;
    throw std::runtime_error("no stage " + name);
}

const json& connection_with_port(const json& brick, const std::string& port) {
    for (const auto& c : brick.at("connections"))
        for (const auto& e : c.at("endpoints"))
            if (e.contains("port") && !e.contains("component") && e.at("port") == port) return c;
    throw std::runtime_error("no connection on port " + port);
}

std::string num(double v) { std::ostringstream o; o.precision(10); o << v; return o.str(); }

std::vector<double> resample(const std::vector<double>& time, const std::vector<double>& val,
                             double tBeg, double period, int N) {
    std::vector<double> out(N, 0.0);
    size_t j = 0;
    for (int k = 0; k < N; ++k) {
        const double t = tBeg + period * k / N;
        while (j + 1 < time.size() && time[j + 1] < t) ++j;
        if (j + 1 >= time.size()) { out[k] = val.back(); continue; }
        const double f = (time[j + 1] > time[j]) ? (t - time[j]) / (time[j + 1] - time[j]) : 0.0;
        out[k] = val[j] + f * (val[j + 1] - val[j]);
    }
    return out;
}

double phase_invariant_nrmse(const std::vector<double>& a, const std::vector<double>& b) {
    const int N = static_cast<int>(b.size());
    double ss = 0;
    for (double x : b) ss += x * x;
    const double rmsB = std::sqrt(ss / N);
    double best = std::numeric_limits<double>::infinity();
    for (int s = 0; s < N; ++s) {
        double acc = 0;
        for (int k = 0; k < N; ++k) { const double d = a[(k + s) % N] - b[k]; acc += d * d; }
        best = std::min(best, std::sqrt(acc / N) / rmsB);
    }
    return best;
}

double mean(const std::vector<double>& v) { double m = 0; for (double x : v) m += x; return m / v.size(); }

// Circular lag (in samples) that best aligns b onto a.
int best_lag(const std::vector<double>& a, const std::vector<double>& b) {
    const int N = static_cast<int>(a.size());
    const double ma = mean(a), mb = mean(b);
    int best = 0; double bestC = -std::numeric_limits<double>::infinity();
    for (int s = 0; s < N; ++s) {
        double c = 0;
        for (int k = 0; k < N; ++k) c += (a[k] - ma) * (b[(k + s) % N] - mb);
        if (c > bestC) { bestC = c; best = s; }
    }
    return best;
}

struct Sim {
    double vout, voutPkPk;
    std::vector<std::vector<double>> phaseCurrents;   // last period, per phase, resampled
};

Sim simulate(const json& tas, int N, double fsw, const std::string& inductorBase) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    const double period = 1.0 / fsw, tstop = 3000.0 * period, tstep = period / 400.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    INFO("ngspice error: " << r.error);
    REQUIRE(r.success);
    Sim s{};
    const auto vo = r.average("v(Vout)", tstop - 40.0 * period, tstop);
    REQUIRE(vo.has_value());
    s.vout = *vo;
    // The runner stores vectors under ngspice's own spelling; match "vout" the way average() does.
    const std::vector<double>* vp = nullptr;
    for (const auto& kv : r.vectors) {
        std::string k = kv.first;
        std::transform(k.begin(), k.end(), k.begin(), ::tolower);
        if (k == "vout" || k == "v(vout)" || (k.size() > 5 && k.compare(k.size() - 5, 5, ".vout") == 0)) vp = &kv.second;
    }
    INFO("no vout vector in the ngspice result");
    REQUIRE(vp != nullptr);
    const auto& v = *vp;
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (size_t i = 0; i < r.time.size(); ++i)
        if (r.time[i] >= tstop - 10.0 * period) { lo = std::min(lo, v[i]); hi = std::max(hi, v[i]); }
    s.voutPkPk = hi - lo;
    for (int k = 1; k <= N; ++k) {
        std::string key;
        const std::string want = N == 1 ? "l" + inductorBase : "l" + inductorBase + "_" + std::to_string(k);
        for (const auto& kv : r.vectors)
            if (kv.first.find(want) != std::string::npos && kv.first.find("#branch") != std::string::npos) key = kv.first;
        INFO("no branch-current vector for " << want);
        REQUIRE(!key.empty());
        s.phaseCurrents.push_back(resample(r.time, r.vectors.at(key), tstop - period, period, 400));
    }
    return s;
}

} // namespace

TEST_CASE("interleave: N cells on shared rails, gates staggered by 360/N", "[interleaved][transform]") {
    const json tas = Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec(24, 5, 60, 300e3, {{"phaseCount", 3}})));
    const json& brick = stage_named(tas, "switchingCell").at("circuit");
    std::vector<std::string> names;
    for (const auto& c : brick.at("components")) names.push_back(c.at("name"));
    for (const char* n : {"Q1_1", "Q1_2", "Q1_3", "L1_1", "L1_2", "L1_3", "D1_1", "D1_3"})
        CHECK(std::find(names.begin(), names.end(), n) != names.end());
    CHECK(names.size() == 9);
    // The input rail is ONE net holding all three high-side drains.
    CHECK(connection_with_port(brick, "vin").at("endpoints").size() == 4);
    CHECK(connection_with_port(brick, "vout").at("endpoints").size() == 4);
    // Each phase has its own switch node and its own gate port.
    for (int k = 1; k <= 3; ++k) CHECK_NOTHROW(connection_with_port(brick, "gate_" + std::to_string(k)));
    CHECK_THROWS(connection_with_port(brick, "gate"));
    std::vector<double> phases;
    for (const auto& st : tas.at("simulation").at("stimulus")) phases.push_back(st.at("waveform").value("phase", 0.0));
    REQUIRE(phases.size() == 3);
    CHECK_THAT(phases[0], WithinAbs(0, 1e-9));
    CHECK_THAT(phases[1], WithinAbs(120, 1e-9));
    CHECK_THAT(phases[2], WithinAbs(240, 1e-9));
    // The converter-level operating point still states the WHOLE output power.
    CHECK(tas["inputs"]["operatingPoints"][0]["outputs"][0]["power"].get<double>() == 60.0);

    // The synchronous rectifier's complementary gate keeps its own offset and gains the stagger.
    const json sync = Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec(24, 5, 60, 300e3, {{"phaseCount", 2}, {"rectifier", "synchronous"}})));
    CHECK(sync.at("simulation").at("stimulus").size() == 4);

    CHECK_THROWS(Kirchhoff::design_buck(spec(24, 5, 60, 300e3, {{"phaseCount", 7}})));
    CHECK_THROWS(Kirchhoff::design_boost(spec(12, 24, 60, 300e3, {{"phaseCount", 1.5}})));
    // N = 1 is the ordinary single-phase converter, byte for byte.
    CHECK(Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec(24, 5, 60, 300e3, {{"phaseCount", 1}})))
          == Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec(24, 5, 60, 300e3))));
}

TEST_CASE("interleave: capacitor ripple of N staggered triangles", "[interleaved][ripple]") {
    const double T = 1e-5, dI = 2.0, D = 0.3;
    const std::vector<double> t{0, D * T, T}, i{-dI / 2, dI / 2, -dI / 2};   // a zero-mean triangle
    // N = 1 is the textbook buck result ΔQ = ΔI·T/8.
    CHECK_THAT(Kirchhoff::interleaved_capacitor_ripple(t, i, T, 1).chargePeakToPeak, WithinRel(dI * T / 8, 0.005));
    // At D = 1/N the staggered triangles sum to a constant: no ripple left.
    const std::vector<double> t2{0, 0.5 * T, T};
    CHECK(Kirchhoff::interleaved_capacitor_ripple(t2, i, T, 2).chargePeakToPeak < 1e-3 * dI * T / 8);
    // Interleaving never increases the ripple.
    CHECK(Kirchhoff::interleaved_capacitor_ripple(t, i, T, 3).chargePeakToPeak
          < Kirchhoff::interleaved_capacitor_ripple(t, i, T, 1).chargePeakToPeak);
}

TEST_CASE("interleaved buck/boost in ngspice: equal sharing, T/N stagger, per-phase shape, output ripple",
          "[interleaved][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    struct Case { const char* name; bool boost; int N; json config; double vin, vout, power, fsw; };
    double boostBaselineNrmse = -1;   // set by the N = 1 boost case, which runs first
    for (const Case& c : {Case{"boost N=1 (baseline)", true, 1, json::object(), 12, 24, 48, 200e3},
                          Case{"buck N=2", false, 2, {{"phaseCount", 2}}, 24, 5, 60, 300e3},
                          Case{"buck N=3", false, 3, {{"phaseCount", 3}}, 24, 5, 60, 300e3},
                          Case{"buck N=2 sync", false, 2, {{"phaseCount", 2}, {"rectifier", "synchronous"}}, 24, 5, 60, 300e3},
                          Case{"boost N=2", true, 2, {{"phaseCount", 2}}, 12, 24, 96, 200e3},
                          Case{"boost N=3", true, 3, {{"phaseCount", 3}}, 12, 30, 120, 200e3}}) {
        {   // one pass over every case (not DYNAMIC_SECTIONs: the boost baseline must survive into the N > 1 cases)
            INFO("case: " << c.name);
            const json sp = spec(c.vin, c.vout, c.power, c.fsw, c.config);
            json tas; double L = 0, vd = 0, cout = 0;
            if (c.boost) { const auto d = Kirchhoff::design_boost(sp); tas = Kirchhoff::build_boost_tas(d); L = d.inductance;
                           vd = Kirchhoff::req::analytical_rectifier_drop(d.config); cout = d.outputCapacitance; }
            else         { const auto d = Kirchhoff::design_buck(sp);  tas = Kirchhoff::build_buck_tas(d);  L = d.inductance; vd = d.diodeDrop; cout = d.outputCapacitance; }
            const Sim s = simulate(tas, c.N, c.fsw, "l1");
            const double iPhase = c.power / c.vout / c.N;
            std::cerr << "[interleaved " << c.name << "] Vout=" << s.vout << " ripple=" << s.voutPkPk
                      << " Cout=" << cout << " phase means:";
            for (const auto& p : s.phaseCurrents) std::cerr << " " << mean(p);
            std::cerr << "\n";

            CHECK_THAT(s.vout, WithinRel(c.vout, 0.03));
            // Equal sharing: identical open-loop cells, so each carries its 1/N.
            const double avg0 = mean(s.phaseCurrents[0]);
            for (const auto& p : s.phaseCurrents) CHECK_THAT(mean(p), WithinRel(avg0, 0.03));
            // Stagger: phase k lags phase 1 by (k−1)·T/N (400 samples per period).
            for (int k = 1; k < c.N; ++k) {
                const int lag = best_lag(s.phaseCurrents[0], s.phaseCurrents[k]);
                const int want = 400 * k / c.N;   // phase k+1 lags phase 1 by k·T/N
                CHECK(std::min(std::abs(lag - want), 400 - std::abs(lag - want)) <= 4);
            }
            // Per-phase waveform against the analytical single-phase solver at Iout/N.
            const MAS::OperatingPoint op = c.boost
                ? Kirchhoff::analytical::analytical_boost(c.vin, c.vout, iPhase, c.fsw, L, vd, 1.0)
                : Kirchhoff::analytical::analytical_buck(c.vin, c.vout, iPhase, c.fsw, L, vd, 1.0);
            const auto [t, i] = Kirchhoff::analytical::winding_current_samples(op, 0, 1.0 / c.fsw);
            const double nrmse = phase_invariant_nrmse(resample(t, i, 0.0, 1.0 / c.fsw, 400), s.phaseCurrents[0]);
            std::cerr << "    phase-1 NRMSE vs analytical = " << nrmse << "\n";
            // The single-phase boost already scores 0.032 here (measured 2026-10-03: the solver's own
            // residual against its deck, not an interleaving effect), so the bound is 0.04, and what this
            // gate asserts for N > 1 is that a phase is as faithful as the single-phase converter.
            CHECK(nrmse < 0.04);
            if (c.boost && c.N == 1) boostBaselineNrmse = nrmse;
            if (c.boost && c.N > 1) {
                REQUIRE(boostBaselineNrmse >= 0);
                CHECK(nrmse <= boostBaselineNrmse + 0.005);
            }
            // Cout was sized for 1 % of Vout from the summed currents; allow the ESR-free sim 50 % margin.
            CHECK(s.voutPkPk < 1.5 * 0.01 * c.vout);
        }
    }
}
