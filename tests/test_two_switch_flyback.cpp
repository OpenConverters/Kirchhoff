// Two-switch (double-ended) flyback (ABT #1436). No MKF model exists, so it is validated standalone: the
// design keeps Vor below Vin,min, the TAS swaps only the primary switch stage, and in ngspice the output is
// delivered, the switched nodes stay clamped inside the rails, and the primary current is the single-switch
// flyback's (same transformer, same duty).
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
#include "TwoSwitchFlyback.hpp"
#include "Flyback.hpp"
#include "ConverterAnalytical.hpp"
#include "TasAssembler.hpp"
#include "NgspiceRunner.hpp"
#include "Fidelity.hpp"

using nlohmann::json;
using Catch::Matchers::WithinRel;

namespace {

json spec(double vin, double vout, double power, double fsw, json config = json::object()) {
    json s;
    s["designRequirements"]["efficiency"] = 1.0;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.85}, {"nominal", vin}, {"maximum", vin * 1.15}};
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

struct Sim { double vout, swMax, swMin, nrmse; std::vector<double> primary; };

// Settle the deck; average Vout; extremes of a named node over the last 20 periods; primary-current NRMSE
// against the analytical flyback at the design's operating point.
Sim simulate(const json& tas, const Kirchhoff::FlybackDesign& d, const std::string& swNode) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    const double period = 1.0 / d.switchingFrequency, tstop = 2500.0 * period, tstep = period / 400.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    INFO("ngspice error: " << r.error);
    REQUIRE(r.success);
    Sim s{};
    const auto vo = r.average("v(Vout)", tstop - 40.0 * period, tstop);
    REQUIRE(vo.has_value());
    s.vout = *vo;
    s.swMax = -std::numeric_limits<double>::infinity(); s.swMin = -s.swMax;
    if (!swNode.empty()) {
        const auto* v = find_vector(r, {swNode});
        INFO("no vector for node " << swNode);
        REQUIRE(v != nullptr);
        for (size_t i = 0; i < r.time.size(); ++i)
            if (r.time[i] >= tstop - 20.0 * period) { s.swMax = std::max(s.swMax, (*v)[i]); s.swMin = std::min(s.swMin, (*v)[i]); }
    }
    const auto* ip = find_vector(r, {"t1", "pri", "#branch"});
    INFO("no T1 primary branch-current vector");
    REQUIRE(ip != nullptr);
    const double iout = d.outputPower / d.outputVoltage;
    const MAS::OperatingPoint op = Kirchhoff::analytical::analytical_flyback(
        d.inputVoltage, {d.outputVoltage}, {iout}, {d.turnsRatio}, d.switchingFrequency, d.magnetizingInductance,
        d.diodeDrop, d.efficiency, 0.4, 0.0);
    const auto [t, i] = Kirchhoff::analytical::winding_current_samples(op, 0, period);
    s.primary = resample(r.time, *ip, tstop - period, period, 400);
    s.nrmse = phase_invariant_nrmse(resample(t, i, 0.0, period, 400), s.primary);
    return s;
}

} // namespace

TEST_CASE("two-switch flyback: Vor held below Vin,min; contradictions refused", "[tsfly][design]") {
    const auto d = Kirchhoff::design_two_switch_flyback(spec(120, 12, 24, 100e3));
    CHECK(d.turnsRatio * (d.outputVoltage + d.diodeDrop) < d.inputVoltageMin);
    // Same transformer as the ordinary flyback with the same duty cap: only the primary switch differs.
    const auto f = Kirchhoff::design_flyback(spec(120, 12, 24, 100e3, {{"maxDutyCycle", 0.45}}));
    CHECK_THAT(d.magnetizingInductance, WithinRel(f.magnetizingInductance, 1e-12));
    CHECK_THAT(d.turnsRatio, WithinRel(f.turnsRatio, 1e-12));

    CHECK_THROWS(Kirchhoff::design_two_switch_flyback(spec(120, 12, 24, 100e3, {{"maxDutyCycle", 0.5}})));
    CHECK_THROWS(Kirchhoff::design_two_switch_flyback(spec(120, 12, 24, 100e3, {{"mode", "qrm"}})));
    json pinned = spec(120, 12, 24, 100e3);
    pinned["designRequirements"]["turnsRatios"] = json::array({{{"nominal", 12.0}}});   // Vor ≈ 151 V > 102 V
    CHECK_THROWS(Kirchhoff::design_two_switch_flyback(pinned));
}

TEST_CASE("two-switch flyback: the TAS swaps only the primary switch stage", "[tsfly][tas]") {
    const json tas = Kirchhoff::build_two_switch_flyback_tas(Kirchhoff::design_two_switch_flyback(spec(120, 12, 24, 100e3)));
    std::vector<std::string> names;
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.at("name") == "inverter")
            for (const auto& c : st.at("circuit").at("components")) names.push_back(c.at("name"));
    for (const char* n : {"Q1", "Q2", "Dc1", "Dc2", "Cin", "Csn1", "Csn2", "Cclmp", "Rclmp"})
        CHECK(std::find(names.begin(), names.end(), n) != names.end());
    // The RC damper sits across the primary (hi to sw), not from the input rail as in the single-switch stage.
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.at("name") == "inverter")
            for (const auto& cn : st.at("circuit").at("connections")) {
                bool cclmp = false, rclmp = false;
                for (const auto& e : cn.at("endpoints")) {
                    if (e.value("component", "") == "Cclmp" && e.value("pin", "") == "1") cclmp = true;
                    if (e.value("component", "") == "Rclmp" && e.value("pin", "") == "2") rclmp = true;
                }
                if (cclmp) CHECK(cn.at("name") == "hi");
                if (rclmp) CHECK(cn.at("name") == "sw");
            }
    CHECK(tas.at("simulation").at("stimulus").size() == 2);
    bool priHi = false;
    for (const auto& isc : tas.at("topology").at("interStageConnections")) if (isc.at("name") == "pri_hi") priHi = true;
    CHECK(priHi);
}

TEST_CASE("two-switch flyback in ngspice: delivers Vout, clamps at the rails, primary current is the flyback's",
          "[tsfly][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    struct Case { const char* name; double vin, vout, power, fsw; json config; };
    for (const Case& c : {Case{"120V ccm", 120, 12, 24, 100e3, json::object()},
                          Case{"375V ccm", 375, 12, 60, 65e3, json::object()},
                          Case{"120V dcm", 120, 12, 24, 100e3, {{"mode", "dcm"}}}}) {
        INFO("case: " << c.name);
        const json sp = spec(c.vin, c.vout, c.power, c.fsw, c.config);
        const auto d = Kirchhoff::design_two_switch_flyback(sp);
        const Sim s = simulate(Kirchhoff::build_two_switch_flyback_tas(d), d, "sw_node");
        // Baseline: the single-switch flyback with the same transformer, scored the same way.
        json fsp = sp; fsp["config"]["maxDutyCycle"] = 0.45;
        const auto fd = Kirchhoff::design_flyback(fsp);
        const Sim fs = simulate(Kirchhoff::build_flyback_tas(fd), fd, "");
        auto peak = [](const std::vector<double>& v) { return *std::max_element(v.begin(), v.end()); };
        // On-time ramp: peak minus the current the switch turns on into (first sample past the turn-on edge;
        // zero in DCM). Vin·D·T/Lm — the transformer and the duty, nothing else.
        auto ramp = [&](const std::vector<double>& v) {
            const double pk = peak(v);
            std::size_t k = 0;
            while (k < v.size() && v[k] < 0.02 * pk) ++k;
            REQUIRE(k + 1 < v.size());
            return pk - v[k + 1];
        };
        const double dPeak = std::abs(peak(s.primary) - peak(fs.primary)) / peak(fs.primary);
        const double dRamp = std::abs(ramp(s.primary) - ramp(fs.primary)) / ramp(fs.primary);
        std::cerr << "[tsfly " << c.name << "] Vout=" << s.vout << " sw node " << s.swMin << ".." << s.swMax
                  << " (Vin " << c.vin << ")  primary peak/ramp vs single-switch: " << dPeak << " / " << dRamp
                  << "  (NRMSE vs analytical " << s.nrmse << ", single-switch " << fs.nrmse << ")\n";
        CHECK_THAT(s.vout, WithinRel(c.vout, 0.03));
        // The low-side switch node never rises above Vin (Dc2 clamps it) nor below ground by more than a drop.
        CHECK(s.swMax < c.vin * 1.02 + 1.0);
        CHECK(s.swMin > -1.5);
        // The primary current is the flyback's: the same transformer at the same duty, so the single-switch deck's
        // peak and on-time ramp within 5 %. (A phase-invariant NRMSE between the two decks is dominated by where
        // the 400-point grid cuts the turn-off edge — one sample of misalignment alone scores ~0.1 — and the
        // flyback's own analytical waveform scores 0.19–0.44 in both decks, so neither is the measure. The mean
        // is not either: it carries the power, and the two decks' dampers dissipate differently.)
        CHECK(dPeak < 0.05);
        CHECK(dRamp < 0.05);
    }
}
