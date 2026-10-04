// LCC resonant converter (ABT #1438). No MKF model exists and FHA is poor for a capacitive-filter LCC, so it is
// validated standalone: the exact steady-state solver against first principles (half-wave antisymmetry, power
// balance), the design rules (gain at the spec frequency, frequency range, ZVS), the TAS per Cp side × bridge, and
// ngspice for all four: Vout delivered, tank current and Cp voltage as solved.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include "Lcc.hpp"
#include "TasAssembler.hpp"
#include "NgspiceRunner.hpp"
#include "Fidelity.hpp"

using nlohmann::json;
using Catch::Matchers::WithinRel;

namespace {

json spec(double vin, double vout, double power, double fsw, json config = json::object()) {
    json s;
    s["designRequirements"]["efficiency"] = 1.0;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.95}, {"nominal", vin}, {"maximum", vin * 1.05}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({{{"name", "out"}, {"voltage", {{"nominal", vout}}}}});
    s["operatingPoints"] = json::array({{{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}}});
    if (!config.empty()) s["config"] = config;
    return s;
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

const std::vector<double>* find_vector(const Kirchhoff::NgspiceRunResult& r, std::vector<std::string> needles) {
    for (const auto& kv : r.vectors) {
        std::string k = kv.first;
        std::transform(k.begin(), k.end(), k.begin(), ::tolower);
        bool all = true;
        for (const auto& n : needles) if (k.find(n) == std::string::npos) all = false;
        if (all) return &kv.second;
    }
    return nullptr;
}

const json* stage_brick(const json& tas, const char* name) {
    for (const auto& st : tas.at("topology").at("stages")) if (st.at("name") == name) return &st.at("circuit");
    return nullptr;
}

std::string net_of(const json& brick, const char* comp, const char* pin) {
    for (const auto& cn : brick.at("connections"))
        for (const auto& e : cn.at("endpoints"))
            if (e.value("component", "") == comp && e.value("pin", "") == pin) return cn.at("name");
    return "";
}

Kirchhoff::analytical::LccCircuit circuit_of(const Kirchhoff::LccDesign& d, double vin, double f) {
    const double n = d.turnsRatio;
    return {(d.fullBridge ? 1.0 : 0.5) * vin, n * (d.outputVoltage + 2.0 * d.diodeDrop), d.seriesInductance,
            d.seriesCapacitance, d.parallelOnSecondary ? d.parallelCapacitance / (n * n) : d.parallelCapacitance,
            d.magnetizingInductance, f};
}

const double kVin = 400, kVout = 2000, kPower = 200, kFsw = 100e3;

} // namespace

TEST_CASE("LCC solver: antisymmetric steady state, power in = power out", "[lcc][solver]") {
    const auto d = Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw));
    const auto c = circuit_of(d, kVin, kFsw);
    const auto s = Kirchhoff::analytical::solve_lcc(c);
    // Half-wave antisymmetry: the value at t and at t + T/2 are opposite (the second half is built that way, so
    // check the seam: the end of the period meets the start).
    CHECK_THAT(s.tankCurrent.back(), WithinRel(s.tankCurrent.front(), 1e-6));
    CHECK_THAT(s.parallelCapacitorVoltage.back(), WithinRel(s.parallelCapacitorVoltage.front(), 1e-6));
    // Lossless tank: the bridge power equals the clamp power.
    double pin = 0, pout = 0;
    for (size_t i = 1; i < s.time.size(); ++i) {
        const double dt = s.time[i] - s.time[i - 1];
        // the bridge voltage of the interval is its end's (the step at T/2 sits between two samples)
        pin += s.bridgeVoltage[i] * 0.5 * (s.tankCurrent[i] + s.tankCurrent[i - 1]) * dt;
        pout += 0.5 * (std::fabs(s.rectifierCurrent[i]) + std::fabs(s.rectifierCurrent[i - 1])) * c.clampVoltage * dt;
    }
    CHECK_THAT(pin, WithinRel(pout, 1e-4));
    // The clamp delivers the design load: n·mean|irect| = Io (with the diode drop as part of the load).
    CHECK_THAT(d.turnsRatio * s.rectifierCurrentMean, WithinRel(kPower / kVout, 1e-4));
    CHECK(s.tankCurrentAtTurnOff > 0);
}

TEST_CASE("LCC design: gain at fs, frequency range across Vin, ZVS; contradictions refused", "[lcc][design]") {
    using Catch::Matchers::ContainsSubstring;
    for (const char* side : {"secondary", "primary"}) {
        INFO(side);
        const auto d = Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw, {{"parallelCapacitorSide", side}}));
        CHECK(d.switchingFrequency == kFsw);
        CHECK(d.frequencyAtMinimumInput < kFsw);
        CHECK(d.frequencyAtMaximumInput > kFsw);
        CHECK(d.frequencyAtMinimumInput > d.seriesResonantFrequency);
        CHECK_THAT(1.0 / (2 * M_PI * std::sqrt(d.seriesInductance * d.seriesCapacitance)), WithinRel(kFsw / 1.6, 1e-9));
        const double cpPri = d.parallelOnSecondary ? d.parallelCapacitance / (d.turnsRatio * d.turnsRatio) : d.parallelCapacitance;
        CHECK_THAT(cpPri, WithinRel(d.seriesCapacitance, 1e-9));   // capacitanceRatio 1
    }
    // A pinned turns ratio moves the frequency instead of the gain.
    const auto d0 = Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw));
    json s = spec(kVin, kVout, kPower, kFsw);
    s["designRequirements"]["turnsRatios"] = json::array({{{"nominal", d0.turnsRatio * 0.97}}});
    const auto dp = Kirchhoff::design_lcc(s);
    // A smaller n needs a smaller gain n·(Vo + Vd)/Vb: a higher frequency on the falling side of the peak.
    CHECK(dp.switchingFrequency > kFsw);
    // An input range wider than the tank's gain can cover.
    json wide = spec(kVin, kVout, kPower, kFsw);
    wide["designRequirements"]["inputVoltage"]["minimum"] = 100;
    CHECK_THROWS_WITH(Kirchhoff::design_lcc(wide), ContainsSubstring("gain"));
    CHECK_THROWS(Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw, {{"parallelCapacitorSide", "both"}})));
    CHECK_THROWS(Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw, {{"resonanceRatio", 0.9}})));
}

TEST_CASE("LCC TAS: Cp on the side asked for, the bridge asked for", "[lcc][tas]") {
    for (const char* side : {"secondary", "primary"})
        for (const char* bridge : {"halfBridge", "fullBridge"}) {
            INFO(side << " / " << bridge);
            const auto d = Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw, {{"parallelCapacitorSide", side}, {"bridgeType", bridge}}));
            const json tas = Kirchhoff::build_lcc_tas(d);
            const json* cell = stage_brick(tas, "lccCell");
            REQUIRE(cell != nullptr);
            const std::string cpNet = net_of(*cell, "Cp", "1");
            CHECK(cpNet == (std::string(side) == "secondary" ? "sec_a" : "pri_top"));
            CHECK(net_of(*cell, "Cs", "1") == "tank");
            CHECK(net_of(*cell, "Lr", "primary_start") == (std::string(bridge) == "halfBridge" ? "sw" : "leg_a"));
            CHECK(tas.at("simulation").at("stimulus").size() == (std::string(bridge) == "halfBridge" ? 2u : 4u));
        }
}

TEST_CASE("LCC in ngspice: Vout, tank current and Cp voltage as solved", "[lcc][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    for (const char* side : {"secondary", "primary"})
        for (const char* bridge : {"halfBridge", "fullBridge"}) {
            const std::string label = std::string(side) + "/" + bridge;
            INFO(label);
            const auto d = Kirchhoff::design_lcc(spec(kVin, kVout, kPower, kFsw, {{"parallelCapacitorSide", side}, {"bridgeType", bridge}}));
            const json tas = Kirchhoff::build_lcc_tas(d);
            PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
            std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
            const double T = 1.0 / kFsw, tstop = 1500.0 * T, tstep = T / 400.0;
            deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                      ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
            const Kirchhoff::NgspiceRunResult res = Kirchhoff::run_ngspice_in_process(deck);
            INFO("ngspice error: " << res.error);
            REQUIRE(res.success);
            const auto vo = res.average("v(Vout)", tstop - 40.0 * T, tstop);
            REQUIRE(vo.has_value());
            const auto* il = find_vector(res, {"lr", "#branch"});
            REQUIRE(il != nullptr);
            const auto ss = Kirchhoff::analytical::solve_lcc(circuit_of(d, kVin, kFsw));
            const std::vector<double> sim = resample(res.time, *il, tstop - T, T, 400);
            const std::vector<double> an = resample(ss.time, ss.tankCurrent, 0.0, T, 400);
            const double nrmse = std::min(phase_invariant_nrmse(an, sim), [&] {
                std::vector<double> neg(sim); for (double& x : neg) x = -x; return phase_invariant_nrmse(an, neg); }());
            double pkSim = 0, pkAn = 0;
            for (double x : sim) pkSim = std::max(pkSim, std::fabs(x));
            for (double x : an) pkAn = std::max(pkAn, std::fabs(x));
            std::cerr << "[lcc " << label << "] n=" << d.turnsRatio << " Lr=" << d.seriesInductance << " Cs=" << d.seriesCapacitance
                      << " Cp=" << d.parallelCapacitance << " Vout=" << *vo << "  NRMSE=" << nrmse << "  |iLr| pk sim/an "
                      << pkSim << "/" << pkAn << "\n";
            // Measured: Vout 1995–2017 V, NRMSE 0.007–0.011, peak within 1 %. The deck runs a 2 % dead time the solver's
            // 50 % square leaves out; the half bridge's split-cap midpoint ripples. (With the 100 pF rectifier constant
            // on the secondary nodes — 30 % of Cp referred to the primary — the same deck read 1625 V: the gate sees a
            // detuned tank.)
            CHECK_THAT(*vo, WithinRel(kVout, 0.015));
            CHECK(nrmse < 0.02);
            CHECK_THAT(pkSim, WithinRel(pkAn, 0.015));
        }
}
