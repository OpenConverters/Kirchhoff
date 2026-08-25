// Negative output rails on the flyback (ABT #904).
//
// A rail asked for as -12 V must come out of ngspice at -12 V. The engine designs every rail from its
// MAGNITUDE (a negative rail is magnetically identical to its positive twin — same turns ratio, same
// volt-seconds, same winding V and I); the sign only mirrors the output side about ground: the
// secondary's two ends swap which one feeds the rectifier and which one returns, and the diode is
// reversed. These tests pin both halves of that claim: the magnetics must NOT move, and the rail must.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <regex>
#include <string>
#include "Flyback.hpp"
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

TEST_CASE("flyback: the TAS states the rail's real sign and reverses its rectifier",
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
    CHECK(deck.find("DD1 ac_in dc_out") != std::string::npos);   // positive rail: forward
    CHECK(deck.find("DD2 dc_out ac_in") != std::string::npos);   // negative rail: reversed
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
