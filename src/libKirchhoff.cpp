// libKirchhoff — the Emscripten/embind WASM surface for Kirchhoff, mirroring MKF's WebLibMKF converter
// API so the OpenMagnetics Wizard frontend can drive KH the same way it drove MKF's converter_models.
//
// Convention (identical to WebLibMKF): every function takes JSON as a std::string and returns JSON (or a
// raw ngspice netlist) as a std::string. On error the returned string starts with "Exception: " (the JS
// side already checks `result.startsWith("Exception")`). This keeps the ABI a flat string<->string map —
// no embind value_object churn, no MAS C++ types crossing the boundary.
//
// KH is topology-AGNOSTIC once a TAS exists, so the surface is:
//   * design_<topo>_tas(spec)                 -> TAS document           (24 per-topology entry points)
//   * process_converter(topo, spec, engine)   -> {inputs, diagnostics, extraComponents, tas}  (one-shot)
//   * generate_ngspice_circuit(tas, fidelity) -> ngspice deck           (generic)
//   * extract_operating_point / topology_waveforms / diagnostics / main_magnetic_inputs /
//     extra_components_inputs(tas)            -> JSON                    (generic, the extract surface)
//
// Build: emscripten only (guarded by __EMSCRIPTEN__). CMake target `libKirchhoff` (KIRCHHOFF_BUILD_WASM).

#include <emscripten/bind.h>

#include "Kirchhoff.hpp"          // design_<topo>/build_<topo>_tas + tas_to_ngspice/ltspice + extract surface
#include "ConverterExtract.hpp"   // extract_operating_point / topology_waveforms / diagnostics / shims
#include "FidelityJson.hpp"       // PEAS::fidelity_from_json
#include "Clllc.hpp"              // topologies not in the Kirchhoff.hpp umbrella
#include "Pfc.hpp"
#include "Vienna.hpp"

#include <functional>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

namespace {

// Run a JSON-string-in / JSON-string-out body, funnelling any exception into the "Exception: ..." string
// the JS side expects (so a bad spec surfaces as a message, never a WASM trap).
template <class F>
std::string guarded(F&& body) {
    try {
        return std::forward<F>(body)();
    } catch (const std::exception& e) {
        return std::string("Exception: ") + e.what();
    } catch (...) {
        return std::string("Exception: unknown error");
    }
}

// --- the topology dispatch table: name -> design_<topo>(spec) then build_<topo>_tas(design) ------------
// One row per supported topology. This is the string-keyed dispatcher WebLibMKF's process_converter had
// and KH lacked; it turns KH's 24 typed (design_X, build_X_tas) pairs into a single by-name entry point.
using TasBuilder = std::function<json(const json&)>;

const std::unordered_map<std::string, TasBuilder>& tas_builders() {
    static const std::unordered_map<std::string, TasBuilder> table = {
#define KH_ROW(name) { #name, [](const json& s) { return Kirchhoff::build_##name##_tas(Kirchhoff::design_##name(s)); } }
        KH_ROW(flyback), KH_ROW(boost), KH_ROW(buck), KH_ROW(forward), KH_ROW(two_switch_forward),
        KH_ROW(sepic), KH_ROW(cuk), KH_ROW(zeta), KH_ROW(push_pull), KH_ROW(psfb), KH_ROW(ahb),
        KH_ROW(acf), KH_ROW(fsbb), KH_ROW(llc), KH_ROW(cllc), KH_ROW(clllc), KH_ROW(src), KH_ROW(dab),
        KH_ROW(isolated_buck), KH_ROW(isolated_buck_boost), KH_ROW(weinberg), KH_ROW(pfc),
        KH_ROW(vienna), KH_ROW(pshb),
#undef KH_ROW
    };
    return table;
}

json build_tas_for(const std::string& topology, const json& spec) {
    auto it = tas_builders().find(topology);
    if (it == tas_builders().end())
        throw std::runtime_error("process_converter: unknown topology '" + topology + "'");
    return it->second(spec);
}

Kirchhoff::ExtractEngine engine_from(const std::string& e) {
    if (e == "ngspice" || e == "NGSPICE") return Kirchhoff::ExtractEngine::NGSPICE;
    if (e == "analytical" || e == "ANALYTICAL" || e.empty()) return Kirchhoff::ExtractEngine::ANALYTICAL;
    throw std::runtime_error("extract engine must be 'analytical' or 'ngspice', got '" + e + "'");
}

// ---- generic TAS surface (the extract trio replacement + diagnostics + legacy shims) ------------------

std::string design_tas(std::string topology, std::string spec) {
    return guarded([&] { return build_tas_for(topology, json::parse(spec)).dump(); });
}

std::string generate_ngspice_circuit(std::string tas, std::string fidelity) {
    return guarded([&] {
        return Kirchhoff::tas_to_ngspice(json::parse(tas), PEAS::fidelity_from_json(json::parse(fidelity)));
    });
}

std::string generate_ltspice_circuit(std::string tas, std::string fidelity) {
    return guarded([&] {
        return Kirchhoff::tas_to_ltspice(json::parse(tas), PEAS::fidelity_from_json(json::parse(fidelity)));
    });
}

std::string extract_operating_point(std::string tas, std::string engine, std::string magneticName) {
    return guarded([&] {
        MAS::OperatingPoint op = Kirchhoff::extract_operating_point(json::parse(tas), engine_from(engine), magneticName);
        json j = op; return j.dump();
    });
}

std::string topology_waveforms(std::string tas) {
    return guarded([&] {
        json out = json::array();
        for (auto& m : Kirchhoff::topology_waveforms(json::parse(tas))) {
            json mi = m.inputs;
            out.push_back(json{{"name", m.name}, {"isMain", m.isMain}, {"inputs", std::move(mi)}});
        }
        return out.dump();
    });
}

std::string diagnostics(std::string tas) {
    return guarded([&] { return Kirchhoff::diagnostics(json::parse(tas)).dump(); });
}

std::string main_magnetic_inputs(std::string tas) {
    return guarded([&] {
        MAS::Inputs in = Kirchhoff::main_magnetic_inputs(json::parse(tas));
        json j = in; return j.dump();
    });
}

std::string extra_components_inputs(std::string tas) {
    return guarded([&] { return Kirchhoff::extra_components_inputs(json::parse(tas)).dump(); });
}

// ---- the one-shot the Wizard calls: spec -> everything MKF's calculate_*/simulate_* returned -----------
// Mirrors WebLibMKF's process_converter(topology, json, useNgspice): assemble the TAS, then return the
// adviser-ready main-magnetic Inputs, the diagnostics, the extra components, and the TAS itself. `engine`
// selects the operating-point source ("analytical" | "ngspice") for the returned Inputs' operating points.
std::string process_converter(std::string topology, std::string spec, std::string engine) {
    return guarded([&] {
        json tas = build_tas_for(topology, json::parse(spec));
        MAS::Inputs mainInputs = Kirchhoff::main_magnetic_inputs(tas);
        // engine-selected operating point for the main magnetic
        MAS::OperatingPoint op = Kirchhoff::extract_operating_point(tas, engine_from(engine));
        json inputsJson = mainInputs;
        json opJson = op;
        return json{
            {"topology", topology},
            {"inputs", std::move(inputsJson)},
            {"operatingPoint", std::move(opJson)},
            {"diagnostics", Kirchhoff::diagnostics(tas)},
            {"extraComponents", Kirchhoff::extra_components_inputs(tas)},
            {"tas", std::move(tas)},
        }.dump();
    });
}

}  // namespace

EMSCRIPTEN_BINDINGS(kirchhoff) {
    namespace em = emscripten;
    // per-topology design entry point (topology passed as an arg — the whole 24-row table in one binding)
    em::function("design_tas", &design_tas);
    // generic assemble -> deck
    em::function("generate_ngspice_circuit", &generate_ngspice_circuit);
    em::function("generate_ltspice_circuit", &generate_ltspice_circuit);
    // the extract surface (MKF simulate_and_extract trio replacement) + diagnostics + legacy shims
    em::function("extract_operating_point", &extract_operating_point);
    em::function("topology_waveforms", &topology_waveforms);
    em::function("diagnostics", &diagnostics);
    em::function("main_magnetic_inputs", &main_magnetic_inputs);
    em::function("extra_components_inputs", &extra_components_inputs);
    // the one-shot Wizard entry point
    em::function("process_converter", &process_converter);
}
