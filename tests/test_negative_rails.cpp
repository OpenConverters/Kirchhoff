// Negative output rails (ABT #904) — flyback, forward, two-switch forward, push-pull.
//
// A rail asked for as -12 V must come out of ngspice at -12 V. The engine designs every rail from its
// MAGNITUDE (a negative rail is magnetically identical to its positive twin — same turns ratio, same
// volt-seconds, same winding V and I); the sign only decides which of the rectifier's two output
// terminals is called the rail and which is called secondary ground. Nothing is reversed — no diode,
// no winding — so the emitted sub-circuit is the one that was designed, it just sits below ground.
// These tests pin both halves of that claim: the magnetics must NOT move, and the rail must.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <regex>
#include <string>
#include "Flyback.hpp"
#include "Forward.hpp"
#include "TwoSwitchForward.hpp"
#include "PushPull.hpp"
#include "Acf.hpp"
#include "Llc.hpp"
#include "Src.hpp"
#include "Dab.hpp"
#include "Cllc.hpp"
#include "Clllc.hpp"
#include "TasAssembler.hpp"
#include "NgspiceRunner.hpp"
#include "Fidelity.hpp"

using nlohmann::json;
using Catch::Matchers::WithinRel;

namespace {

// A flyback spec whose rails are given verbatim (signed) as {voltage, current}.
json spec(const std::vector<std::pair<double, double>>& rails,
          double vin = 120, double fsw = 100000) {
    json s;
    s["designRequirements"]["efficiency"] = 0.88;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array();
    json ops = json::array();
    for (size_t i = 0; i < rails.size(); ++i) {
        const std::string name = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        s["designRequirements"]["outputs"].push_back(
            {{"name", name}, {"voltage", {{"nominal", rails[i].first}}}, {"regulation", "voltage"}});
        ops.push_back({{"name", name}, {"power", std::fabs(rails[i].first) * rails[i].second}});
    }
    s["operatingPoints"] = json::array({{{"inputVoltage", vin}, {"outputs", ops}}});
    return s;
}

std::string num(double v) { std::ostringstream o; o.precision(10); o << v; return o.str(); }

// Settle the deck to steady state and average each rail node over the last ~40 periods.
std::vector<double> simulate_rails(const json& tas, size_t nRails, double fsw) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    const double period = 1.0 / fsw;
    const double tstop = 3000.0 * period;
    const double tstep = period / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    INFO("ngspice error: " << r.error);
    REQUIRE(r.success);
    std::vector<double> out;
    for (size_t i = 0; i < nRails; ++i) {
        const std::string node = (i == 0) ? "v(Vout)" : "v(Vout" + std::to_string(i + 1) + ")";
        auto v = r.average(node, tstop - 40.0 * period, tstop);
        REQUIRE(v.has_value());
        out.push_back(*v);
    }
    return out;
}

} // namespace

TEST_CASE("flyback: a negative rail designs identically to its positive twin", "[flyback][negative]") {
    const auto pos = Kirchhoff::design_flyback(spec({{12.0, 2.0}}));
    const auto neg = Kirchhoff::design_flyback(spec({{-12.0, 2.0}}));

    CHECK(pos.outputs[0].polarity == 1);
    CHECK(neg.outputs[0].polarity == -1);

    // The design is stored as a magnitude, and every derived magnetic quantity must be untouched.
    CHECK_THAT(neg.outputs[0].voltage, WithinRel(pos.outputs[0].voltage, 1e-12));
    CHECK_THAT(neg.turnsRatio, WithinRel(pos.turnsRatio, 1e-12));
    CHECK_THAT(neg.magnetizingInductance, WithinRel(pos.magnetizingInductance, 1e-12));
    CHECK_THAT(neg.dutyCycle, WithinRel(pos.dutyCycle, 1e-12));
    CHECK_THAT(neg.loadResistance, WithinRel(pos.loadResistance, 1e-12));
    CHECK_THAT(neg.outputs[0].outputCapacitance, WithinRel(pos.outputs[0].outputCapacitance, 1e-12));
}

TEST_CASE("flyback: the TAS states the rail's real sign and keeps the rectifier as designed",
          "[flyback][negative]") {
    const json tas = Kirchhoff::build_flyback_tas(
        Kirchhoff::design_flyback(spec({{12.0, 2.0}, {-12.0, 1.0}})));

    // The requirement the assembler reads must say where each rail actually sits.
    const json& outs = tas["inputs"]["designRequirements"]["outputs"];
    CHECK(outs[0]["voltage"]["nominal"].get<double>() > 0);
    CHECK(outs[1]["voltage"]["nominal"].get<double>() < 0);

    // SPICE diode syntax is `Dname anode cathode model`, and the rectifier subckt's port order is
    // (ac_in, dc_out). Positive rail: anode on the winding. Negative rail: reversed.
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    INFO(deck);
    // Both rails keep an identically-wired rectifier — the polarity lives in which terminal the
    // assembler calls the rail, not in a reversed component.
    CHECK(deck.find("DD1 ac_in dc_out") != std::string::npos);
    CHECK(deck.find("DD2 ac_in dc_out") != std::string::npos);
}

TEST_CASE("flyback: ngspice puts a -12 V rail at -12 V", "[flyback][negative][ngspice]") {
    const double fsw = 100000;
    const auto rails = simulate_rails(
        Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(spec({{-12.0, 2.0}}, 120, fsw))), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 6.0);     // a real rail, not a numerical dribble
    CHECK(std::fabs(rails[0]) < 24.0);    // open-loop at the designed duty, so allow generous headroom
}

TEST_CASE("flyback: a mixed +5 / +12 / -12 design puts each rail on its own side of ground",
          "[flyback][negative][ngspice]") {
    const double fsw = 100000;
    const auto rails = simulate_rails(
        Kirchhoff::build_flyback_tas(
            Kirchhoff::design_flyback(spec({{5.0, 2.0}, {12.0, 1.0}, {-12.0, 1.0}}, 120, fsw))), 3, fsw);
    INFO("rails = " << rails[0] << ", " << rails[1] << ", " << rails[2]);
    CHECK(rails[0] > 0.0);
    CHECK(rails[1] > 0.0);
    CHECK(rails[2] < 0.0);
    // The two 12 V rails share a magnitude, so the negative one must mirror the positive one closely.
    CHECK_THAT(std::fabs(rails[2]), WithinRel(rails[1], 0.25));
}


// ---------------------------------------------------------------------------
// The same mirror applies to every multi-output isolated topology whose rectifier is diodes: reverse
// each diode in the rail's path and swap that secondary's ends (ABT #904). These pin the forward
// family (two diodes + output inductor per rail) and push-pull (centre-tapped full wave).
// ---------------------------------------------------------------------------

namespace {

// A buck-derived isolated spec (forward family / push-pull): lower Vin, generous duty headroom.
json bd_spec(const std::vector<std::pair<double, double>>& rails, double vin = 48, double fsw = 100000) {
    json s;
    s["designRequirements"]["efficiency"] = 0.9;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array();
    json ops = json::array();
    for (size_t i = 0; i < rails.size(); ++i) {
        const std::string name = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        s["designRequirements"]["outputs"].push_back(
            {{"name", name}, {"voltage", {{"nominal", rails[i].first}}}, {"regulation", "voltage"}});
        ops.push_back({{"name", name}, {"power", std::fabs(rails[i].first) * rails[i].second}});
    }
    s["operatingPoints"] = json::array({{{"inputVoltage", vin}, {"outputs", ops}}});
    return s;
}

} // namespace


TEST_CASE("forward: a negative rail designs identically and simulates below ground",
          "[forward][negative][ngspice]") {
    const double fsw = 100000;
    const auto pos = Kirchhoff::design_forward(bd_spec({{12.0, 2.0}}, 48, fsw));
    const auto neg = Kirchhoff::design_forward(bd_spec({{-12.0, 2.0}}, 48, fsw));
    CHECK(neg.outputs[0].polarity == -1);
    CHECK_THAT(neg.turnsRatio, WithinRel(pos.turnsRatio, 1e-12));
    CHECK_THAT(neg.outputs[0].outputInductance, WithinRel(pos.outputs[0].outputInductance, 1e-12));

    const auto rails = simulate_rails(Kirchhoff::build_forward_tas(neg), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 6.0);
    CHECK(std::fabs(rails[0]) < 24.0);
}

TEST_CASE("two_switch_forward: a negative rail simulates below ground",
          "[two_switch_forward][negative][ngspice]") {
    const double fsw = 100000;
    const auto neg = Kirchhoff::design_two_switch_forward(bd_spec({{-12.0, 2.0}}, 48, fsw));
    CHECK(neg.outputs[0].polarity == -1);
    const auto rails = simulate_rails(Kirchhoff::build_two_switch_forward_tas(neg), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 6.0);
    CHECK(std::fabs(rails[0]) < 24.0);
}

TEST_CASE("push_pull: a negative rail simulates below ground", "[push_pull][negative][ngspice]") {
    const double fsw = 100000;
    const auto neg = Kirchhoff::design_push_pull(bd_spec({{-12.0, 2.0}}, 48, fsw));
    CHECK(neg.outputs[0].polarity == -1);
    const auto rails = simulate_rails(Kirchhoff::build_push_pull_tas(neg), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 6.0);
    CHECK(std::fabs(rails[0]) < 24.0);
}

TEST_CASE("forward: a mixed +5 / -12 design puts each rail on its own side of ground",
          "[forward][negative][ngspice]") {
    const double fsw = 100000;
    const auto rails = simulate_rails(
        Kirchhoff::build_forward_tas(Kirchhoff::design_forward(bd_spec({{5.0, 3.0}, {-12.0, 1.0}}, 48, fsw))),
        2, fsw);
    INFO("rails = " << rails[0] << ", " << rails[1]);
    CHECK(rails[0] > 0.0);
    CHECK(rails[1] < 0.0);
}


TEST_CASE("acf: a negative rail simulates below ground", "[acf][negative][ngspice]") {
    const double fsw = 100000;
    const auto pos = Kirchhoff::design_acf(bd_spec({{12.0, 2.0}}, 48, fsw));
    const auto neg = Kirchhoff::design_acf(bd_spec({{-12.0, 2.0}}, 48, fsw));
    CHECK(neg.outputs[0].polarity == -1);
    CHECK_THAT(neg.turnsRatio, WithinRel(pos.turnsRatio, 1e-12));

    const auto rails = simulate_rails(Kirchhoff::build_acf_tas(neg), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 6.0);
    CHECK(std::fabs(rails[0]) < 24.0);
}


// Resonant converters: a higher input rail and a resonant tank, so give them their own spec point.
namespace {
json res_spec(const std::vector<std::pair<double, double>>& rails, double vin = 400, double fsw = 100000) {
    return bd_spec(rails, vin, fsw);
}
} // namespace

TEST_CASE("llc: a negative rail simulates below ground", "[llc][negative][ngspice]") {
    const double fsw = 100000;
    const auto pos = Kirchhoff::design_llc(res_spec({{12.0, 2.0}}, 400, fsw));
    const auto neg = Kirchhoff::design_llc(res_spec({{-12.0, 2.0}}, 400, fsw));
    CHECK(neg.outputs[0].polarity == -1);
    CHECK_THAT(neg.turnsRatio, WithinRel(pos.turnsRatio, 1e-12));

    const auto rails = simulate_rails(Kirchhoff::build_llc_tas(neg), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 4.0);
}

TEST_CASE("src: a negative rail simulates below ground", "[src][negative][ngspice]") {
    const double fsw = 100000;
    const auto neg = Kirchhoff::design_src(res_spec({{-12.0, 2.0}}, 400, fsw));
    CHECK(neg.outputs[0].polarity == -1);
    const auto rails = simulate_rails(Kirchhoff::build_src_tas(neg), 1, fsw);
    INFO("Vout = " << rails[0]);
    CHECK(rails[0] < 0.0);
    CHECK(std::fabs(rails[0]) > 4.0);
}




TEST_CASE("active-bridge topologies refuse a negative rail instead of silently inverting it",
          "[negative][guard]") {
    // dab / cllc / clllc drive their secondary with an ACTIVE BRIDGE: polarity there comes from the
    // gate pattern, not device orientation, so neither the relabel nor the diode-reversal mirror
    // applies. They must throw rather than quietly design a POSITIVE rail (ABT #904).
    CHECK_THROWS_AS(Kirchhoff::design_dab(res_spec({{-12.0, 2.0}}, 400, 100000)), std::invalid_argument);
    CHECK_THROWS_AS(Kirchhoff::design_cllc(res_spec({{-12.0, 2.0}}, 400, 100000)), std::invalid_argument);
    CHECK_THROWS_AS(Kirchhoff::design_clllc(res_spec({{-12.0, 2.0}}, 400, 100000)), std::invalid_argument);
    // …and still design normally when asked for a positive one.
    CHECK_NOTHROW(Kirchhoff::design_dab(res_spec({{12.0, 2.0}}, 400, 100000)));
}
