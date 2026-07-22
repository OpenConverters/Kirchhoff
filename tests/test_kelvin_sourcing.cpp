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
