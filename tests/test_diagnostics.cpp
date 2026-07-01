// Diagnostics — Kirchhoff::diagnostics(tas), the TAS-derived replacement for MKF's per-model
// "<name>Diagnostics" objects. Topology-agnostic: computed component values (magnetizing/resonant
// inductance, turns ratios, resonant/output caps) + per-operating-point per-winding stresses + CCM flag,
// in MKF's flat + operatingPoints[] shape.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"

#include <string>

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

}  // namespace

TEST_CASE("diagnostics: LLC surfaces the resonant tank + per-OP stresses", "[diagnostics][llc]") {
    json d = Kirchhoff::diagnostics(Kirchhoff::build_llc_tas(Kirchhoff::design_llc(spec_for(400, 12, 120, 100000))));

    // computed{}: the cross-topology "computed*" tank values
    REQUIRE(d.contains("computed"));
    const json& c = d.at("computed");
    CHECK(c.at("magnetizingInductance").get<double>() > 0.0);
    CHECK(c.at("resonantCapacitance").get<double>() > 0.0);       // from the role="resonant" cap
    CHECK(c.at("turnsRatio").get<double>() > 0.0);
    REQUIRE(c.contains("extraInductors"));                        // resonant Lr as its own magnetic
    CHECK(c.at("extraInductors").size() == 1);
    CHECK(c.at("extraInductors").at(0).at("inductance").get<double>() > 0.0);

    // magnetics[]: main transformer (3 windings) + Lr (1 winding)
    REQUIRE(d.at("magnetics").size() == 2);
    size_t mains = 0, threeWinding = 0;
    for (const auto& m : d.at("magnetics")) {
        if (m.at("isMain").get<bool>()) { ++mains; }
        if (m.at("windings").get<size_t>() == 3) ++threeWinding;
    }
    CHECK(mains == 1);
    CHECK(threeWinding == 1);

    // capacitors[]: at least one role="resonant"
    bool resonantCap = false;
    for (const auto& cap : d.at("capacitors"))
        if (cap.value("role", std::string{}) == "resonant") resonantCap = true;
    CHECK(resonantCap);

    // operatingPoints[]: per-winding current stress present
    REQUIRE(d.at("operatingPoints").size() >= 1);
    const json& op0 = d.at("operatingPoints").at(0);
    REQUIRE(op0.at("windings").size() >= 1);
    CHECK(op0.at("windings").at(0).at("current_peak").get<double>() > 0.0);
    CHECK(op0.at("windings").at(0).at("current_rms").get<double>() > 0.0);
}

TEST_CASE("diagnostics: buck is CCM with a single inductor", "[diagnostics][buck]") {
    json d = Kirchhoff::diagnostics(Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(24, 12, 60, 100000))));
    REQUIRE(d.at("magnetics").size() == 1);
    CHECK(d.at("magnetics").at(0).at("isMain").get<bool>());
    // a 60 W / 12 V buck runs deep in CCM (5 A load >> ripple/2)
    REQUIRE(d.contains("isCcm"));
    CHECK(d.at("isCcm").get<bool>());
    CHECK(d.at("primaryPeakCurrent").get<double>() > 0.0);
    CHECK(d.at("switchingFrequency").get<double>() == Catch::Approx(100000).epsilon(0.01));
}

TEST_CASE("diagnostics: empty TAS throws (no silent fallback)", "[diagnostics][errors]") {
    CHECK_THROWS(Kirchhoff::diagnostics(json::object()));
}
