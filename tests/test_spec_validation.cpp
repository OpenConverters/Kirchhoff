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
