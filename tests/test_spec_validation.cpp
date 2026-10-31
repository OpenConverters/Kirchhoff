// Requirements that describe no converter must be REFUSED, not designed around (ABT #747, #752).
//
// The engine used to validate in places and not in others. fs = 0, Vout = 0 and Vout > Vin were rejected
// with specific exceptions, while efficiency = 1.5, Vin,min > Vin,nom, Pout = 0 and Vin,min = 0 sailed
// through and produced designs whose parts were then presented as fact — a 0 F output capacitor on a
// Pout = 0 buck, a 0 H transformer with a turns ratio of zero on a Vin,min = 0 forward. Separately, the
// single-output cells (buck, boost, sepic, cuk, zeta, fsbb) designed outputs[0] and DISCARDED any further
// output without a word, handing back a one-output converter for a two-output request.
//
// Both classes are checked here through the api layer, because that is the choke point every caller goes
// through — the WASM bundle the web app runs, the pybind module, the MCP servers.
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "KirchhoffApi.hpp"

using json = nlohmann::json;

namespace {

// A minimal, valid buck spec — the shape the web app's buildSpec emits.
json buck_spec() {
    return json{
        {"designRequirements", {
            {"efficiency", 0.92},
            {"inputType", "dc"},
            {"inputVoltage", {{"nominal", 48.0}}},
            {"switchingFrequency", {{"nominal", 250e3}}},
            {"outputs", json::array({ json{{"name", "out"}, {"voltage", {{"nominal", 12.0}}}, {"regulation", "voltage"}} })},
        }},
        {"operatingPoints", json::array({ json{
            {"name", "full_load"}, {"inputVoltage", 48.0}, {"ambientTemperature", 25.0},
            {"outputs", json::array({ json{{"name", "out"}, {"power", 60.0}} })}} })},
        {"config", {{"rectifier", "diode"}}},
    };
}

bool refused(const json& spec, const std::string& topology = "buck") {
    const std::string out = Kirchhoff::api::design_tas_full(topology, spec.dump());
    return out.rfind("Exception", 0) == 0;
}

std::string why(const json& spec, const std::string& topology = "buck") {
    return Kirchhoff::api::design_tas_full(topology, spec.dump()).substr(0, 200);
}

}  // namespace

TEST_CASE("a well-formed spec still designs", "[validation]") {
    REQUIRE_FALSE(refused(buck_spec()));
}

TEST_CASE("impossible requirements are refused, not designed around", "[validation]") {
    SECTION("efficiency above unity") {
        json s = buck_spec();
        s["designRequirements"]["efficiency"] = 1.5;
        INFO(why(s));
        REQUIRE(refused(s));
    }
    SECTION("efficiency of zero") {
        json s = buck_spec();
        s["designRequirements"]["efficiency"] = 0.0;
        REQUIRE(refused(s));
    }
    SECTION("an input range that starts at zero volts") {
        json s = buck_spec();
        s["designRequirements"]["inputVoltage"]["minimum"] = 0.0;
        INFO(why(s));
        REQUIRE(refused(s));
    }
    SECTION("an input minimum above the nominal") {
        json s = buck_spec();
        s["designRequirements"]["inputVoltage"]["minimum"] = 200.0;
        INFO(why(s));
        REQUIRE(refused(s));
    }
    SECTION("an input maximum below the nominal") {
        json s = buck_spec();
        s["designRequirements"]["inputVoltage"]["maximum"] = 12.0;
        REQUIRE(refused(s));
    }
    SECTION("zero output power") {
        json s = buck_spec();
        s["operatingPoints"][0]["outputs"][0]["power"] = 0.0;
        INFO(why(s));
        REQUIRE(refused(s));
    }
    SECTION("negative output power") {
        json s = buck_spec();
        s["operatingPoints"][0]["outputs"][0]["power"] = -60.0;
        REQUIRE(refused(s));
    }
    SECTION("a zero-volt output rail") {
        json s = buck_spec();
        s["designRequirements"]["outputs"][0]["voltage"]["nominal"] = 0.0;
        REQUIRE(refused(s));
    }
    SECTION("a negative rail is NOT refused — inverting topologies ask for one") {
        json s = buck_spec();
        s["designRequirements"]["outputs"][0]["voltage"]["nominal"] = -12.0;
        s["operatingPoints"][0]["outputs"][0]["power"] = 30.0;
        // cuk inverts; it must be free to design this (whether it converges is its own business).
        const std::string out = Kirchhoff::api::design_tas_full("cuk", s.dump());
        REQUIRE(out.find("voltage must be non-zero") == std::string::npos);
    }
}

TEST_CASE("a single-output cell refuses a multi-output request instead of dropping it", "[validation]") {
    for (const char* topology : {"buck", "boost", "sepic", "cuk", "zeta", "fsbb"}) {
        json s = buck_spec();
        s.erase("config");
        s["designRequirements"]["outputs"].push_back(
            json{{"name", "out2"}, {"voltage", {{"nominal", 5.0}}}, {"regulation", "voltage"}});
        s["operatingPoints"][0]["outputs"].push_back(json{{"name", "out2"}, {"power", 20.0}});
        INFO(topology << ": " << why(s, topology));
        REQUIRE(refused(s, topology));
        // ...and it says what it cannot do, rather than failing somewhere downstream by accident.
        REQUIRE(why(s, topology).find("single output rail") != std::string::npos);
    }
}

namespace {

// A one-output DC spec for an isolated cell: Vin, one rail Vout delivering P, switching at fsw.
json dc_spec(double vin, double vout, double power, double fsw) {
    return json{
        {"designRequirements", {
            {"efficiency", 1.0},
            {"inputType", "dc"},
            {"inputVoltage", {{"nominal", vin}, {"minimum", 0.95 * vin}, {"maximum", 1.05 * vin}}},
            {"switchingFrequency", {{"nominal", fsw}}},
            {"outputs", json::array({ json{{"name", "out"}, {"voltage", {{"nominal", vout}}}, {"regulation", "voltage"}} })},
        }},
        {"operatingPoints", json::array({ json{
            {"name", "full_load"}, {"inputVoltage", vin}, {"ambientTemperature", 25.0},
            {"outputs", json::array({ json{{"name", "out"}, {"power", power}} })}} })},
    };
}

// A one-output AC-input (PFC-type) spec: line RMS voltage, line frequency, DC bus Vout delivering P.
json ac_spec(const std::string& inputType, double vrms, double vout, double power, double fsw) {
    return json{
        {"designRequirements", {
            {"efficiency", 1.0},
            {"inputType", inputType},
            {"inputVoltage", {{"nominal", vrms}}},
            {"lineFrequency", {{"nominal", 50.0}}},
            {"switchingFrequency", {{"nominal", fsw}}},
            {"outputs", json::array({ json{{"name", "out"}, {"voltage", {{"nominal", vout}}}} })},
        }},
        {"operatingPoints", json::array({ json{
            {"inputVoltage", vrms}, {"outputs", json::array({ json{{"name", "out"}, {"power", power}} })}} })},
    };
}

}  // namespace

// psfb, pshb, ahb, weinberg, cllc, clllc, pfc and vienna design outputs[0] only. They used to drop any further
// output without a word (the caller got a one-output converter back for a two-output request); they must refuse
// it and say so. The one-output spec of each is checked to design first, so the refusal is the extra rail's.
TEST_CASE("single-rail topologies refuse a second output instead of dropping it", "[validation][single-output]") {
    struct Case { const char* topology; const char* label; json spec; };
    const std::vector<Case> cases = {
        {"psfb",     "phase-shifted full bridge", dc_spec(400.0, 12.0, 600.0, 100e3)},
        {"pshb",     "phase-shifted half bridge", dc_spec(400.0, 12.0, 600.0, 100e3)},
        {"ahb",      "asymmetric half bridge",    dc_spec(100.0, 12.0, 192.0, 100e3)},
        {"weinberg", "Weinberg",                  dc_spec(42.0, 100.0, 500.0, 100e3)},
        {"cllc",     "CLLC",                      dc_spec(400.0, 48.0, 480.0, 200e3)},
        {"clllc",    "CLLLC",                     dc_spec(400.0, 48.0, 480.0, 200e3)},
        {"pfc",      "PFC",                       ac_spec("acSinglePhase", 230.0, 400.0, 500.0, 100e3)},
        {"vienna",   "Vienna rectifier",          ac_spec("acThreePhase", 230.0, 700.0, 5000.0, 20e3)},
    };
    for (const auto& c : cases) {
        INFO(c.topology << " one output: " << why(c.spec, c.topology));
        REQUIRE_FALSE(refused(c.spec, c.topology));

        json two = c.spec;
        two["designRequirements"]["outputs"].push_back(
            json{{"name", "out2"}, {"voltage", {{"nominal", 5.0}}}, {"regulation", "voltage"}});
        two["operatingPoints"][0]["outputs"].push_back(json{{"name", "out2"}, {"power", 20.0}});
        const std::string expected = std::string(c.label) + " supports one output; the spec has 2";
        INFO(c.topology << " two outputs: " << why(two, c.topology));
        REQUIRE(why(two, c.topology).find(expected) != std::string::npos);

        // The operating point alone carrying a second rail is the same request and is refused the same way.
        json opOnly = c.spec;
        opOnly["operatingPoints"][0]["outputs"].push_back(json{{"name", "out2"}, {"power", 20.0}});
        INFO(c.topology << " operating point with two outputs: " << why(opOnly, c.topology));
        REQUIRE(why(opOnly, c.topology).find(expected) != std::string::npos);
    }
}
