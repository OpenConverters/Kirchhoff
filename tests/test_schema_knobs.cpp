// The MAS topology-schema fields Kirchhoff honours natively (2026-09-24): each one either changes the design
// as the model parameter it names, or is a constraint that throws with a specific message. The PyOpenMagnetics
// adapter maps the schema fields onto these config / designRequirements keys; this suite pins the Kirchhoff
// side. Tag [schema].

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "Ahb.hpp"
#include "Buck.hpp"
#include "Cllc.hpp"
#include "Cmc.hpp"
#include "ComponentRequirements.hpp"
#include "Dmc.hpp"
#include "Flyback.hpp"
#include "Forward.hpp"
#include "Llc.hpp"
#include "Psfb.hpp"
#include "PushPull.hpp"

#include <nlohmann/json.hpp>
#include <string>

using nlohmann::json;
using Catch::Matchers::ContainsSubstring;

namespace {
json spec(double vin, double vout, double power, double fsw) {
    json s;
    s["designRequirements"]["efficiency"] = 1.0;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.95}, {"nominal", vin}, {"maximum", vin * 1.05}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({{{"name", "out"}, {"voltage", {{"nominal", vout}}}}});
    s["operatingPoints"] = json::array({{{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}}});
    return s;
}
// The component `name` of a TAS (null when absent).
json component(const json& tas, const std::string& name) {
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.contains("circuit") && st.at("circuit").contains("components"))
            for (const auto& c : st.at("circuit").at("components"))
                if (c.value("name", std::string()) == name) return c;
    return nullptr;
}
}  // namespace

TEST_CASE("schema: diodeVoltageDrop is a fixed rectifier drop, DIDEAL only without it", "[schema]") {
    json s = spec(48, 12, 30, 100e3);
    const auto dideal = Kirchhoff::design_flyback(s);
    s["config"]["diodeVoltageDrop"] = 2.0;
    const auto fixed = Kirchhoff::design_flyback(s);
    CHECK(fixed.diodeDrop == Catch::Approx(2.0));
    CHECK(dideal.diodeDrop != Catch::Approx(2.0));
    CHECK(fixed.dutyCycle > dideal.dutyCycle);   // Vor = n(Vo + Vd) grows with the stated drop
    s["config"]["diodeVoltageDrop"] = -0.1;
    CHECK_THROWS_WITH(Kirchhoff::design_flyback(s), ContainsSubstring("diodeVoltageDrop"));
}

TEST_CASE("schema: maximumSwitchCurrent / maximumDrainSourceVoltage are enforced", "[schema]") {
    json fwd = spec(48, 5, 50, 200e3);
    fwd["config"]["maximumSwitchCurrent"] = 0.5;
    CHECK_THROWS_WITH(Kirchhoff::build_forward_tas(Kirchhoff::design_forward(fwd)),
                      ContainsSubstring("exceeds maximumSwitchCurrent"));
    fwd["config"]["maximumSwitchCurrent"] = 50.0;
    CHECK_NOTHROW(Kirchhoff::build_forward_tas(Kirchhoff::design_forward(fwd)));

    json fb = spec(48, 12, 30, 100e3);
    fb["config"]["maximumDrainSourceVoltage"] = 60.0;
    CHECK_THROWS_WITH(Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(fb)),
                      ContainsSubstring("exceeds maximumDrainSourceVoltage"));
    json pp = spec(48, 12, 60, 100e3);
    pp["config"]["maximumDrainSourceVoltage"] = 80.0;   // push-pull switches block 2·Vin_max
    CHECK_THROWS_WITH(Kirchhoff::build_push_pull_tas(Kirchhoff::design_push_pull(pp)),
                      ContainsSubstring("exceeds maximumDrainSourceVoltage"));
}

TEST_CASE("schema: a switch-current cap and a ripple ratio are both honoured (larger inductance)", "[schema]") {
    json s = spec(12, 5, 10, 200e3);
    s["config"]["maximumSwitchCurrent"] = 3.0;
    const double capOnly = Kirchhoff::design_buck(s).inductance;
    s["config"]["rippleRatio"] = 0.1;   // tighter than the cap allows
    const double both = Kirchhoff::design_buck(s).inductance;
    CHECK(both > capOnly);
}

TEST_CASE("schema: integrated / leakage series inductors are folded into the transformer", "[schema]") {
    json llc = spec(400, 24, 240, 100e3);
    llc["config"]["integratedResonantInductor"] = true;
    const auto d = Kirchhoff::design_llc(llc);
    const json tas = Kirchhoff::build_llc_tas(d);
    CHECK(component(tas, "Lr").is_null());
    const json t1 = component(tas, "T1");
    REQUIRE(!t1.is_null());
    const json leak = t1.at("data").at("inputs").at("designRequirements").at("leakageInductance");
    for (const auto& l : leak) CHECK(l.at("nominal").get<double>() == Catch::Approx(d.resonantInductance));

    json psfb = spec(400, 24, 1200, 100e3);
    psfb["config"]["useLeakageInductance"] = true;
    const auto p = Kirchhoff::design_psfb(psfb);
    const json ptas = Kirchhoff::build_psfb_tas(p);
    CHECK(component(ptas, "Lr").is_null());
    CHECK(component(ptas, "T1").at("data").at("inputs").at("designRequirements").at("leakageInductance").at(0)
              .at("nominal").get<double>() == Catch::Approx(p.seriesInductance));
}

TEST_CASE("schema: CLLC asymmetric tank ratios and the switching-frequency band", "[schema]") {
    json s = spec(400, 48, 480, 200e3);
    s["config"]["resonantInductorRatio"] = 0.95;
    s["config"]["resonantCapacitorRatio"] = 1.05;
    const auto d = Kirchhoff::design_cllc(s);
    const double n = d.turnsRatio;
    CHECK(d.secondaryResonantInductance == Catch::Approx(0.95 * d.primaryResonantInductance / (n * n)));
    CHECK(d.secondaryResonantCapacitance == Catch::Approx(1.05 * n * n * d.primaryResonantCapacitance));
    s["config"]["symmetricDesign"] = true;   // contradicts the ratios
    CHECK_THROWS_WITH(Kirchhoff::design_cllc(s), ContainsSubstring("symmetricDesign"));
    json band = spec(400, 48, 480, 200e3);
    band["config"]["minSwitchingFrequency"] = 210e3;
    CHECK_THROWS_WITH(Kirchhoff::design_cllc(band), ContainsSubstring("outside the switching-frequency band"));
}

TEST_CASE("schema: AHB input-voltage step adds a transient operating point, explicit Lo/Cb honoured", "[schema]") {
    json s = spec(100, 12, 192, 100e3);
    s["config"]["operatingDutyCycle"] = 0.4;
    s["config"]["inputVoltageStepRange"] = 20.0;
    s["config"]["outputInductance"] = 22e-6;
    s["config"]["dcBlockingCapacitance"] = 3.3e-6;
    const auto d = Kirchhoff::design_ahb(s);
    CHECK(d.outputInductance == Catch::Approx(22e-6));
    CHECK(d.dcBlockingCapacitance == Catch::Approx(3.3e-6));
    const json t1 = component(Kirchhoff::build_ahb_tas(d), "T1");
    const json ops = t1.at("data").at("inputs").at("operatingPoints");
    REQUIRE(ops.size() == 2);
    const double off0 = ops.at(0).at("excitationsPerWinding").at(0).at("current").at("processed").at("offset").get<double>();
    const double off1 = ops.at(1).at("excitationsPerWinding").at(0).at("current").at("processed").at("offset").get<double>();
    CHECK(off1 - off0 == Catch::Approx(0.4 * 20.0 / 100e3 / (2.0 * d.magnetizingInductance)));
    s["config"]["maximumDutyCycle"] = 0.3;
    CHECK_THROWS_WITH(Kirchhoff::design_ahb(s), ContainsSubstring("maximumDutyCycle"));
}

TEST_CASE("schema: EMI chokes read the MAS impedance point, attenuation targets and leakage limit", "[schema]") {
    json cmc = {{"operatingVoltage", {{"nominal", 230}}}, {"operatingCurrent", 6.0}, {"lineFrequency", 50.0},
                {"ambientTemperature", 25.0},
                {"minimumImpedance", json::array({{{"frequency", 150e3}, {"impedance", {{"magnitude", 1000.0}, {"phase", 60.0}}}}})},
                {"maximumLeakageInductance", 5e-6}};
    const MAS::Inputs in = Kirchhoff::build_cmc_inputs(Kirchhoff::design_cmc(cmc));
    const auto mi = in.get_design_requirements().get_minimum_impedance();
    REQUIRE(mi);
    CHECK(mi->at(0).get_impedance().get_magnitude() == Catch::Approx(1000.0));
    REQUIRE(mi->at(0).get_impedance().get_phase());
    CHECK(*mi->at(0).get_impedance().get_phase() == Catch::Approx(60.0));
    const auto leak = in.get_design_requirements().get_leakage_inductance();
    REQUIRE(leak);
    CHECK(*leak->at(0).get_maximum() == Catch::Approx(5e-6));
    json gap = cmc;
    gap["maximumDcResistance"] = 0.05;
    CHECK_THROWS_WITH(Kirchhoff::design_cmc(gap), ContainsSubstring("schema gap"));

    json dmc = {{"inputVoltage", {{"nominal", 230}}}, {"operatingCurrent", 6.0}, {"lineFrequency", 50.0},
                {"ambientTemperature", 25.0}, {"switchingFrequency", 100e3}, {"filterCapacitance", 1e-6},
                {"minimumInductance", 10e-6}};
    const double lMin = Kirchhoff::design_dmc(dmc).computedInductance;
    dmc["targetAttenuation"] = json::array({{{"frequency", 150e3}, {"attenuation", 60.0}}});
    CHECK(Kirchhoff::design_dmc(dmc).computedInductance > lMin);
    dmc.erase("filterCapacitance");
    CHECK_THROWS_WITH(Kirchhoff::design_dmc(dmc), ContainsSubstring("filterCapacitance"));
}
