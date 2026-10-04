// Hard-switched (symmetric PWM) half bridge (ABT #1434) and full bridge (ABT #1435). No MKF model exists, so they
// are validated standalone: the design rules (duty at Vin_min, a diode never drops out in the dead time, the
// split capacitors), the TAS of each bridge × rectifier, and in ngspice for all six: the output is delivered, the
// primary current follows the exact piecewise-linear solver, and the half bridge's midpoint sits at Vin/2.
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
#include "HardSwitchedBridge.hpp"
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
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
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

bool has_component(const json& brick, const char* name) {
    for (const auto& c : brick.at("components")) if (c.at("name") == name) return true;
    return false;
}

Kirchhoff::analytical::SrcRectifier solver_rect(const std::string& r) {
    using R = Kirchhoff::analytical::SrcRectifier;
    return r == "centerTapped" ? R::CENTER_TAPPED : r == "fullBridge" ? R::FULL_BRIDGE : R::CURRENT_DOUBLER;
}

struct Case { bool full; double vin, vout, power, fsw; };
const Case kHalf{false, 400.0, 24.0, 240.0, 100e3};
const Case kFull{true, 400.0, 48.0, 960.0, 100e3};

Kirchhoff::HardSwitchedBridgeDesign design(const Case& c, json config) {
    const json s = spec(c.vin, c.vout, c.power, c.fsw, config);
    return c.full ? Kirchhoff::design_full_bridge(s) : Kirchhoff::design_half_bridge(s);
}
json build(const Kirchhoff::HardSwitchedBridgeDesign& d) {
    return d.fullBridge ? Kirchhoff::build_full_bridge_tas(d) : Kirchhoff::build_half_bridge_tas(d);
}

} // namespace

TEST_CASE("hard-switched bridge design: duty at Vin_min, Lm against the freewheel, split capacitors", "[hsbridge][design]") {
    for (const Case& c : {kHalf, kFull})
        for (const char* r : {"centerTapped", "fullBridge", "currentDoubler"}) {
            INFO((c.full ? "full" : "half") << " / " << r);
            const auto d = design(c, {{"rectifierType", r}});
            const double k = c.full ? 1.0 : 0.5, g = std::string(r) == "currentDoubler" ? 1.0 : 2.0;
            const double vd = (std::string(r) == "fullBridge" ? 2.0 : 1.0) * d.diodeDrop;
            // The turns ratio uses the whole duty at Vin_min (2-dp floor) and the operating duty follows from it.
            const double dMin = d.turnsRatio * (c.vout + vd) / (g * k * c.vin * 0.9);
            CHECK(dMin <= 0.45 + 1e-12);
            CHECK(dMin > 0.45 - 0.01 * 0.45);
            CHECK_THAT(d.dutyCycle, WithinRel(d.turnsRatio * (c.vout + vd) / (g * k * c.vin), 1e-12));
            CHECK_THAT(d.bridgeVoltage, WithinRel(k * c.vin, 1e-12));
            // The exact solver accepts the design at both corners (a diode never drops out, CCM throughout).
            const double io = c.power / c.vout;
            for (double vin : {0.9 * c.vin, 1.1 * c.vin})
                CHECK_NOTHROW(Kirchhoff::analytical::analytical_hard_switched_bridge(
                    vin, k, c.vout, io, d.turnsRatio, c.fsw, d.magnetizingInductance, d.outputInductance, d.diodeDrop,
                    solver_rect(r)));
            if (!c.full) {
                CHECK(d.splitCapacitance > 0);
                CHECK(d.balancingResistance > 0);
            }
        }
}

TEST_CASE("hard-switched bridge design: contradictions are refused", "[hsbridge][design]") {
    using Catch::Matchers::ContainsSubstring;
    // A pinned turns ratio that needs more duty than maxDutyCycle at Vin_min.
    json s = spec(400, 24, 240, 100e3);
    s["designRequirements"]["turnsRatios"] = json::array({{{"nominal", 8.0}}});
    CHECK_THROWS_WITH(Kirchhoff::design_half_bridge(s), ContainsSubstring("above maxDutyCycle"));
    // A magnetizing inductance so small that its current outruns the freewheeling inductor current.
    json s2 = spec(400, 24, 240, 100e3);
    s2["designRequirements"]["magnetizingInductance"] = {{"nominal", 20e-6}};
    CHECK_THROWS_WITH(Kirchhoff::design_half_bridge(s2), ContainsSubstring("drop out"));
    CHECK_THROWS_WITH(Kirchhoff::design_full_bridge(spec(400, 48, 960, 100e3, {{"rectifierType", "voltageDoubler"}})),
                      ContainsSubstring("voltageDoubler"));
    CHECK_THROWS(Kirchhoff::design_half_bridge(spec(400, 24, 240, 100e3, {{"maxDutyCycle", 0.5}})));
    const auto hb = Kirchhoff::design_half_bridge(spec(400, 24, 240, 100e3));
    CHECK_THROWS(Kirchhoff::build_full_bridge_tas(hb));
}

TEST_CASE("hard-switched bridge TAS: the bridge, the split capacitors and each rectifier", "[hsbridge][tas]") {
    for (const Case& c : {kHalf, kFull})
        for (const char* r : {"centerTapped", "fullBridge", "currentDoubler"}) {
            INFO((c.full ? "full" : "half") << " / " << r);
            const json tas = build(design(c, {{"rectifierType", r}}));
            const json* cell = stage_brick(tas, c.full ? "fullBridgeCell" : "halfBridgeCell");
            REQUIRE(cell != nullptr);
            for (const char* q : {"Q1", "Q2", "Dq1", "Dq2", "T1", "Crc_pri", "Rrc_pri"}) CHECK(has_component(*cell, q));
            CHECK(has_component(*cell, "Q3") == c.full);
            CHECK(has_component(*cell, "CsHi") == !c.full);
            CHECK(has_component(*cell, "RbLo") == !c.full);
            CHECK(has_component(*cell, "Dr3") == (std::string(r) == "fullBridge"));
            CHECK(has_component(*cell, "Lo2") == (std::string(r) == "currentDoubler"));
            const auto& stim = tas.at("simulation").at("stimulus");
            CHECK(stim.size() == (c.full ? 4u : 2u));
            for (const auto& st : stim) {
                const std::string sw = st.at("component");
                const double ph = st.at("waveform").at("phase");
                CHECK(ph == ((sw == "Q1" || sw == "Q4") ? 0.0 : 180.0));
            }
            // The T1 excitations come from the exact solver: one per winding.
            const json* t1 = nullptr;
            for (const auto& comp : cell->at("components")) if (comp.at("name") == "T1") t1 = &comp;
            REQUIRE(t1 != nullptr);
            const auto& ops = t1->at("data").at("inputs").at("operatingPoints");
            REQUIRE(!ops.empty());
            CHECK(ops.at(0).at("excitationsPerWinding").size() == (std::string(r) == "centerTapped" ? 3u : 2u));
        }
}

TEST_CASE("hard-switched bridge in ngspice: Vout, primary current as solved, midpoint at Vin/2", "[hsbridge][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    for (const Case& c : {kHalf, kFull})
        for (const char* r : {"centerTapped", "fullBridge", "currentDoubler"}) {
            const std::string label = std::string(c.full ? "full" : "half") + "/" + r;
            INFO(label);
            json cfgx = {{"rectifierType", r}};
            const auto d = design(c, cfgx);
            const json tas = build(d);
            PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
            std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
            const double T = 1.0 / c.fsw, tstop = 1500.0 * T, tstep = T / 400.0;
            deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                      ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
            const Kirchhoff::NgspiceRunResult res = Kirchhoff::run_ngspice_in_process(deck);
            INFO("ngspice error: " << res.error);
            REQUIRE(res.success);
            const auto vo = res.average("v(Vout)", tstop - 40.0 * T, tstop);
            REQUIRE(vo.has_value());
            const auto* ip = find_vector(res, {"t1", "pri", "#branch"});
            REQUIRE(ip != nullptr);
            const double io = c.power / c.vout;
            const auto op = Kirchhoff::analytical::analytical_hard_switched_bridge(
                c.vin, c.full ? 1.0 : 0.5, c.vout, io, d.turnsRatio, c.fsw, d.magnetizingInductance,
                d.outputInductance, d.diodeDrop, solver_rect(r));
            const auto anSamples = Kirchhoff::analytical::winding_current_samples(op, 0, T);
            const std::vector<double> sim = resample(res.time, *ip, tstop - T, T, 400);
            const std::vector<double> an = resample(anSamples.first, anSamples.second, 0.0, T, 400);
            // The deck's branch current enters the primary's start pin; the solver's is in the same dot reference.
            const double nrmse = std::min(phase_invariant_nrmse(an, sim), [&] {
                std::vector<double> neg(sim); for (double& x : neg) x = -x; return phase_invariant_nrmse(an, neg); }());
            const double pkSim = *std::max_element(sim.begin(), sim.end(), [](double a, double b) { return std::fabs(a) < std::fabs(b); });
            const double pkAn = *std::max_element(an.begin(), an.end());
            std::cerr << "[hsbridge " << label << "] n=" << d.turnsRatio << " D=" << d.dutyCycle << " Vout=" << *vo
                      << "  NRMSE=" << nrmse << "  |peak| sim/an " << std::fabs(pkSim) << "/" << pkAn << "\n";
            CHECK_THAT(*vo, WithinRel(c.vout, 0.03));
            // 0.046–0.067 measured. What is left is the step edges: the primary current steps to zero at every
            // turn-off, the embedded (harmonic-trimmed) waveform rounds each step over ~T/128, and ngspice resets the
            // leakage through the primary RC in ~25 ns. The ramps themselves are checked point by point below.
            CHECK(nrmse < 0.08);
            CHECK_THAT(std::fabs(pkSim), WithinRel(pkAn, 0.03));
            // The deck starts Q1 at phase 0 and the window starts on a period boundary, so sample k is phase k/400.
            // Mid power interval: magnetizing plus reflected inductor current, as solved. Mid dead time: the bridge
            // is open and the rectifier carries everything, so the primary current is zero.
            const int kMid = static_cast<int>(std::lround(0.5 * d.dutyCycle * 400));
            const int kDead = static_cast<int>(std::lround(0.5 * (d.dutyCycle + 0.5) * 400));
            const double sgn = (sim[kMid] * an[kMid] < 0) ? -1.0 : 1.0;
            CHECK_THAT(sgn * sim[kMid], WithinRel(an[kMid], 0.02));
            CHECK_THAT(sgn * sim[kMid + 200], WithinRel(an[kMid + 200], 0.02));
            CHECK(std::fabs(sim[kDead]) < 0.02 * pkAn);
            CHECK(std::fabs(sim[kDead + 200]) < 0.02 * pkAn);
            if (!c.full) {
                const auto vmid = res.average("v(mid)", tstop - 40.0 * T, tstop);
                REQUIRE(vmid.has_value());
                std::cerr << "    midpoint " << *vmid << " V\n";
                CHECK_THAT(*vmid, WithinRel(0.5 * c.vin, 0.01));
            }
        }
}
