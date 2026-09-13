// End-to-end: design a TAS -> Kelvin select_components (real TAS DB) -> bind_part -> the bound
// component reads as a real DATASHEET part (not the "requirements-derived" fabrication), and a
// re-select defers it as already bound. Requires the TAS data dir (KELVIN_TAS_DATA_DIR env or the
// KELVIN_TAS_DATA_DIR compile default), READABLE from this runtime; skipped with a clear message
// if the dir is unset or unreachable (e.g. a WASM/node sandbox with no filesystem access — the
// real coverage then comes from the native build of this test).
#include <cstdlib>
#include <fstream>
#include <functional>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include "KirchhoffApi.hpp"

using json = nlohmann::json;
namespace kapi = Kirchhoff::api;

namespace {
std::string data_dir() {
    if (const char* e = std::getenv("KELVIN_TAS_DATA_DIR")) return e;
#ifdef KELVIN_TAS_DATA_DIR
    return KELVIN_TAS_DATA_DIR;
#else
    return "";
#endif
}
bool is_exception(const std::string& s) { return s.rfind("Exception:", 0) == 0; }

const char* kFlybackSpec = R"({
    "designRequirements": { "efficiency": 0.88,
        "inputVoltage": { "nominal": 48.0 }, "switchingFrequency": { "nominal": 100000 },
        "outputs": [ { "name": "12V", "voltage": { "nominal": 12.0 } } ] },
    "operatingPoints": [ { "inputVoltage": 48.0, "outputs": [ { "power": 24.0 } ] } ]
})";
}  // namespace

TEST_CASE("kelvin sourcing: design -> select_components -> bind_part -> DATASHEET", "[kelvin]") {
    std::string dir = data_dir();
    if (dir.empty()) {
        WARN("KELVIN_TAS_DATA_DIR not set — skipping real-DB sourcing test");
        return;
    }
    // The dir is set at compile time, but this test needs to READ the multi-GB TAS
    // catalogue off disk. In a WASM/node sandbox the path exists on the host yet is
    // not reachable through the module's filesystem, so the engine reports "catalogue
    // file does not exist". Detect that and skip (the native build has real FS access
    // and provides the actual coverage) rather than fail on a missing precondition.
    if (!std::ifstream(dir + "/controllers.ndjson").good()) {
        WARN("TAS data dir '" << dir << "' not readable from this runtime (no filesystem "
             "access, e.g. WASM/node) — skipping real-DB sourcing test");
        return;
    }

    std::string tas = kapi::design_tas("flyback", kFlybackSpec);
    REQUIRE_FALSE(is_exception(tas));

    std::string resStr = kapi::select_components(tas, dir, "", R"({"topology":"flyback"})");
    REQUIRE_FALSE(is_exception(resStr));
    json res = json::parse(resStr);
    REQUIRE(res.contains("components"));

    // Find a filled component that carries a candidate envelope (mosfet/diode/capacitor/resistor).
    json target;
    for (const auto& c : res.at("components")) {
        if (c.value("filled", false) && c.contains("selection") &&
            !c.at("selection").at("candidates").empty()) {
            target = c;
            break;
        }
    }
    REQUIRE_FALSE(target.is_null());
    INFO("bound component: " << target.at("ref") << " mpn=" << target.value("mpn", std::string()));

    std::string ref = target.at("ref").get<std::string>();
    json envelope = target.at("selection").at("candidates")[0].at("envelope");
    std::string mpn = target.value("mpn", std::string());
    REQUIRE_FALSE(mpn.empty());

    std::string boundStr = kapi::bind_part(tas, ref, envelope.dump());
    REQUIRE_FALSE(is_exception(boundStr));
    json bound = json::parse(boundStr);

    // The bound component's family slot now carries a real manufacturerInfo (deep-search).
    std::function<bool(const json&)> has_real_mpn = [&](const json& node) -> bool {
        if (node.is_object()) {
            if (node.contains("manufacturerInfo")) return true;
            for (auto it = node.begin(); it != node.end(); ++it)
                if (has_real_mpn(it.value())) return true;
        } else if (node.is_array()) {
            for (const auto& e : node) if (has_real_mpn(e)) return true;
        }
        return false;
    };
    bool found_bound = false;
    for (const auto& stage : bound.at("topology").at("stages"))
        for (const auto& comp : stage.at("circuit").at("components"))
            if (comp.value("name", std::string()) == ref)
                found_bound = has_real_mpn(comp.at("data"));
    REQUIRE(found_bound);

    // realize_tas leaves the bound part alone (no "requirements-derived" over a real MPN).
    std::string realizedStr = kapi::realize_tas(boundStr);
    REQUIRE_FALSE(is_exception(realizedStr));
    REQUIRE((realizedStr.find("requirements-derived") == std::string::npos ||
             realizedStr.find(mpn) != std::string::npos));

    // A re-select now defers the bound component.
    std::string res2Str = kapi::select_components(boundStr, dir, "", R"({"topology":"flyback"})");
    json res2 = json::parse(res2Str);
    for (const auto& c : res2.at("components"))
        if (c.at("ref") == ref) {
            REQUIRE(c.value("filled", true) == false);
            REQUIRE(c.value("deferred", std::string()) == "already bound");
        }
}

// The production report (2026-09-13): picking Würth 750811248 from Kelvin for the default flyback's
// transformer threw "MAS DATASHEET: no transformer/coupledInductor entry in datasheetInfo.electrical
// of 'transformer' (1 configuration(s) present)". The catalogue had filed that 40:10:10 flyback
// transformer as a single-winding 300 uH inductor. Rebuilt from the Midcom Smart Transformer Selector
// workbook, it carries two wirings, and the 1-secondary flyback must bind and simulate on its datasheet.
TEST_CASE("kelvin sourcing: the Midcom flyback transformer 750811248 binds and simulates in a flyback",
          "[kelvin][magnetic]") {
    std::string dir = data_dir();
    if (dir.empty() || !std::ifstream(dir + "/magnetics.ndjson").good()) {
        WARN("TAS magnetics catalogue not readable — skipping");
        return;
    }
    std::string record;
    {
        std::ifstream f(dir + "/magnetics.ndjson");
        for (std::string line; std::getline(f, line);)
            if (line.find("\"reference\":\"750811248\"") != std::string::npos) { record = line; break; }
    }
    REQUIRE_FALSE(record.empty());
    const json rec = json::parse(record);
    const json& electrical = rec.at("magnetic").at("manufacturerInfo").at("datasheetInfo").at("electrical");
    REQUIRE(electrical.size() == 2);
    CHECK(electrical[0].at("subtype") == "transformer");

    // The web GUI's default flyback: 36-60 V in, one 12 V / 24 W output.
    const char* spec = R"({
        "designRequirements": { "efficiency": 0.88,
            "inputVoltage": { "minimum": 36.0, "nominal": 48.0, "maximum": 60.0 },
            "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "out", "voltage": { "nominal": 12.0 } } ] },
        "operatingPoints": [ { "inputVoltage": 48.0, "outputs": [ { "power": 24.0 } ] } ]
    })";
    std::string tas = kapi::design_tas("flyback", spec);
    REQUIRE_FALSE(is_exception(tas));
    const json tasDoc = json::parse(tas);  // named: a range-for over json::parse(...).at(...) dangles
    std::string ref;
    for (const auto& stage : tasDoc.at("topology").at("stages"))
        for (const auto& comp : stage.at("circuit").at("components"))
            if (comp.contains("data") && comp.at("data").is_object() && comp.at("data").contains("magnetic"))
                ref = comp.at("name").get<std::string>();
    REQUIRE_FALSE(ref.empty());

    std::string bound = kapi::bind_part(tas, ref, record);
    REQUIRE_FALSE(is_exception(bound));
    std::string waves = kapi::component_waveforms(bound, R"({"origin":"REQUIREMENTS"})");
    INFO(waves.substr(0, 400));
    CHECK_FALSE(is_exception(waves));

    // Counter-check: the record as it was catalogued before the rebuild still fails, loudly.
    const json old = json::parse(R"({"magnetic":{"manufacturerInfo":{"name":"Würth Elektronik",
        "reference":"750811248","status":"production","datasheetInfo":{"electrical":[{"subtype":"inductor",
        "inductance":{"nominal":0.0003},"dcResistance":{"nominal":0.155,"maximum":0.025},
        "saturationCurrentPeak":3.5}]}}}})");
    std::string oldBound = kapi::bind_part(tas, ref, old.dump());
    REQUIRE_FALSE(is_exception(oldBound));
    std::string oldWaves = kapi::component_waveforms(oldBound, R"({"origin":"REQUIREMENTS"})");
    CHECK(is_exception(oldWaves));
    CHECK(oldWaves.find("no transformer/coupledInductor entry") != std::string::npos);
}
