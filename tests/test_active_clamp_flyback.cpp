// Active-clamp flyback (ABT #1432). No MKF model exists, so it is validated standalone: the design rules of
// each drive (complementary: Lk designed as the resonant partner of Cc; NC-ACF: Lk a parasitic, pulse sized
// for the clamp voltage), the TAS of each clamp side, and in ngspice for all four variants: the output is
// delivered, the primary current follows the steady-state solver's waveform, the clamp peaks where the
// solver says, and Q1 turns on at zero voltage.
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
#include "ActiveClampFlyback.hpp"
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

Kirchhoff::analytical::AcfFlybackCircuit circuit(const Kirchhoff::ActiveClampFlybackDesign& d, double vin) {
    Kirchhoff::analytical::AcfFlybackCircuit c{};
    c.inputVoltage = vin; c.outputVoltage = d.flyback.outputVoltage; c.diodeDrop = d.flyback.diodeDrop;
    c.turnsRatio = d.flyback.turnsRatio; c.magnetizingInductance = d.magnetizingInductance;
    c.leakageInductance = d.leakageInductance; c.clampCapacitance = d.clampCapacitance;
    c.switchNodeCapacitance = d.switchNodeCapacitance; c.switchingFrequency = d.flyback.switchingFrequency;
    c.deadTime = d.deadTime; c.complementary = (d.mode == "complementary"); c.clampPulse = d.clampPulse;
    return c;
}

const json* stage_brick(const json& tas, const char* name) {
    for (const auto& st : tas.at("topology").at("stages")) if (st.at("name") == name) return &st.at("circuit");
    return nullptr;
}

// Name of the brick net a component pin is on.
std::string net_of(const json& brick, const char* comp, const char* pin) {
    for (const auto& cn : brick.at("connections"))
        for (const auto& e : cn.at("endpoints"))
            if (e.value("component", "") == comp && e.value("pin", "") == pin) return cn.at("name");
    return "";
}

} // namespace

TEST_CASE("active-clamp flyback, complementary: Lk designed with Cc; ZVS at every corner", "[acfly][design]") {
    const auto d = Kirchhoff::design_active_clamp_flyback(spec(200, 20, 65, 150e3));
    REQUIRE(d.mode == "complementary");
    REQUIRE(d.clampSide == "high");
    const double T = 1.0 / 150e3, vr = d.flyback.turnsRatio * (20 + d.flyback.diodeDrop);
    const double dMin = vr / (d.flyback.inputVoltageMin + vr);
    // π·√(Lk·Cc) = the off-time at Vin,min.
    CHECK_THAT(M_PI * std::sqrt(d.leakageInductance * d.clampCapacitance), WithinRel((1 - dMin) * T, 1e-9));
    CHECK_THAT(d.primaryInductance, WithinRel(d.magnetizingInductance + d.leakageInductance, 1e-12));
    CHECK(d.clampPulse == 0.0);
    for (double vin : {d.flyback.inputVoltageMin, d.flyback.inputVoltage, d.flyback.inputVoltageMax}) {
        const auto ss = Kirchhoff::analytical::solve_active_clamp_flyback(circuit(d, vin), 65.0 / 20);
        INFO("Vin " << vin);
        CHECK(ss.switchNodeVoltageMidDeadTime <= 1e-6 * vin);           // ZVS, with half the dead time to spare
        CHECK_THAT(ss.outputCurrent, WithinRel(65.0 / 20, 1e-5));
    }
    // The leakage costs duty: commutating the primary current through Lk takes part of each edge, so the solved
    // duty exceeds the leakage-free volt-second duty Vr/(Vin + Vr), with Vr reflected by the cantilever ratio k·n.
    const double vrEff = vr * std::sqrt(d.magnetizingInductance / d.primaryInductance);
    CHECK(d.dutyCycle > vrEff / (200 + vrEff));
}

TEST_CASE("active-clamp flyback, NC: Lk a parasitic, the pulse buys ZVS; contradictions refused", "[acfly][design]") {
    const auto d = Kirchhoff::design_active_clamp_flyback(spec(200, 20, 65, 150e3, {{"mode", "nonComplementary"}}));
    CHECK_THAT(d.leakageInductance / d.primaryInductance, WithinRel(0.02, 1e-9));
    REQUIRE(d.clampPulse > 0.0);
    const double vrEff = d.flyback.turnsRatio * std::sqrt(d.magnetizingInductance / d.primaryInductance)
                         * (20 + d.flyback.diodeDrop);
    bool someCornerNeedsTheFullPulse = false;
    for (double vin : {d.flyback.inputVoltageMin, d.flyback.inputVoltageMax}) {
        INFO("Vin " << vin);
        const auto ss = Kirchhoff::analytical::solve_active_clamp_flyback(circuit(d, vin), 65.0 / 20);
        CHECK(ss.switchNodeVoltageMidDeadTime <= 1e-6 * vin);             // ZVS at the designed pulse, with margin
        // The clamp holds above the level the rectifier needs to take the current through Lk (Vr·Lp/Lm).
        CHECK(*std::min_element(ss.clampVoltage.begin(), ss.clampVoltage.end())
              > vrEff * d.primaryInductance / d.magnetizingInductance);
        // The design is the shortest such pulse: 5 % shorter loses ZVS at the corner that set it.
        auto c = circuit(d, vin);
        c.clampPulse = 0.95 * d.clampPulse;
        if (Kirchhoff::analytical::solve_active_clamp_flyback(c, 65.0 / 20).switchNodeVoltageMidDeadTime > 1e-6 * vin)
            someCornerNeedsTheFullPulse = true;
    }
    CHECK(someCornerNeedsTheFullPulse);

    CHECK_THROWS(Kirchhoff::design_active_clamp_flyback(spec(200, 20, 65, 150e3, {{"mode", "resonant"}})));
    CHECK_THROWS(Kirchhoff::design_active_clamp_flyback(spec(200, 20, 65, 150e3, {{"clampSide", "middle"}})));
    CHECK_THROWS(Kirchhoff::design_active_clamp_flyback(spec(200, 20, 65, 150e3, {{"clampOvershoot", 0.0}})));
    json two = spec(200, 20, 65, 150e3);
    two["designRequirements"]["outputs"].push_back({{"name", "aux"}, {"voltage", {{"nominal", 12}}}});
    two["operatingPoints"][0]["outputs"].push_back({{"power", 2}});
    CHECK_THROWS(Kirchhoff::design_active_clamp_flyback(two));
}

TEST_CASE("active-clamp flyback TAS: the clamp leg per side, two drives, Lk as a requirement", "[acfly][tas]") {
    for (const char* mode : {"complementary", "nonComplementary"})
        for (const char* side : {"high", "low"}) {
            INFO(mode << " / " << side);
            const json tas = Kirchhoff::build_active_clamp_flyback_tas(Kirchhoff::design_active_clamp_flyback(
                spec(200, 20, 65, 150e3, {{"mode", mode}, {"clampSide", side}})));
            const json* inv = stage_brick(tas, "inverter");
            REQUIRE(inv != nullptr);
            for (const char* n : {"Q1", "Dq1", "Qc", "Dqc", "Cc", "Csw", "Cin"})
                CHECK(net_of(*inv, n, n[0] == 'Q' ? "drain" : n[0] == 'D' ? "anode" : "1") != "");
            CHECK(net_of(*inv, "Q1", "drain") == "sw");
            CHECK(net_of(*inv, "Dqc", "anode") == net_of(*inv, "Qc", "source"));     // body diode, source→drain
            CHECK(net_of(*inv, "Dqc", "cathode") == net_of(*inv, "Qc", "drain"));
            if (std::string(side) == "high") {
                CHECK(net_of(*inv, "Cc", "1") == "dc_pos");                          // Cc from Vin…
                CHECK(net_of(*inv, "Qc", "source") == "sw");                         // …through Qc to the drain
            } else {
                CHECK(net_of(*inv, "Cc", "1") == "sw");                              // Cc from the drain…
                CHECK(net_of(*inv, "Qc", "drain") == "dc_neg");                      // …through Qc to GND
            }
            CHECK(tas.at("simulation").at("stimulus").size() == 2);
            const json* xf = stage_brick(tas, "transformer");
            REQUIRE(xf != nullptr);
            const json& lk = xf->at("components").at(0).at("data").at("inputs").at("designRequirements")
                                 .at("leakageInductance").at(0);
            CHECK(lk.contains(std::string(mode) == "complementary" ? "nominal" : "maximum"));
        }
}

TEST_CASE("active-clamp flyback in ngspice: Vout, primary current, clamp peak and ZVS as solved", "[acfly][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    for (const char* mode : {"complementary", "nonComplementary"})
        for (const char* side : {"high", "low"}) {
            INFO(mode << " / " << side);
            const auto d = Kirchhoff::design_active_clamp_flyback(
                spec(200, 20, 65, 150e3, {{"mode", mode}, {"clampSide", side}}));
            const json tas = Kirchhoff::build_active_clamp_flyback_tas(d);
            PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
            std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
            const double T = 1.0 / 150e3, tstop = 3000.0 * T, tstep = T / 400.0;
            deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                      ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
            const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
            INFO("ngspice error: " << r.error);
            REQUIRE(r.success);
            const auto vo = r.average("v(Vout)", tstop - 40.0 * T, tstop);
            REQUIRE(vo.has_value());
            const auto* ip = find_vector(r, {"t1", "pri", "#branch"});
            const auto* vsw = find_vector(r, {"sw_node"});
            REQUIRE(ip != nullptr);
            REQUIRE(vsw != nullptr);
            const auto ss = Kirchhoff::analytical::solve_active_clamp_flyback(circuit(d, 200), 65.0 / 20);
            const std::vector<double> sim = resample(r.time, *ip, tstop - T, T, 400);
            const std::vector<double> an = resample(ss.time, ss.primaryCurrent, 0.0, T, 400);
            const double nrmse = phase_invariant_nrmse(an, sim);
            const double pkSim = *std::max_element(sim.begin(), sim.end());
            const double pkAn = *std::max_element(an.begin(), an.end());
            // Drain voltage just before Q1 turns on: the last 30 % of the dead time after the clamp gate drops, in
            // each of the final 20 periods (the node slews in Csw·V/I ≈ 40 ns of the 100 ns).
            double vOn = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < r.time.size(); ++i) {
                const double ph = std::fmod(r.time[i], T) / T;
                if (r.time[i] > tstop - 20.0 * T && ph > 1.0 - 0.3 * d.deadTime / T) vOn = std::max(vOn, (*vsw)[i]);
            }
            const double vswMax = [&] { double m = 0; for (size_t i = 0; i < r.time.size(); ++i)
                if (r.time[i] > tstop - 20.0 * T) m = std::max(m, (*vsw)[i]); return m; }();
            const double vswMaxAn = *std::max_element(ss.switchNodeVoltage.begin(), ss.switchNodeVoltage.end());
            std::cerr << "[acfly " << mode << "/" << side << "] Vout=" << *vo << "  NRMSE=" << nrmse << "  peak sim/an "
                      << pkSim << "/" << pkAn << "  drain before turn-on " << vOn << " V  drain max sim/an " << vswMax
                      << "/" << vswMaxAn << "\n";
            CHECK_THAT(*vo, WithinRel(20.0, 0.03));
            // Complementary: 0.027 measured. NC: 0.094 measured — the idle valley ring, which ngspice damps in
            // its diodes and the lossless analytical ring does not; the on-ramp and the reset match as closely.
            CHECK(nrmse < (std::string(mode) == "complementary" ? 0.04 : 0.12));
            CHECK_THAT(pkSim, WithinRel(pkAn, 0.05));
            CHECK(vOn < 0.05 * 200);                                   // ZVS: the drain is down before Q1 turns on
            CHECK_THAT(vswMax, WithinRel(vswMaxAn, 0.05));
        }
}
