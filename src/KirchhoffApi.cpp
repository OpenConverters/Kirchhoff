#include "KirchhoffApi.hpp"

#include "Kirchhoff.hpp"          // design_<topo>/build_<topo>_tas + tas_to_ngspice/ltspice + extract surface
#include "ConverterExtract.hpp"   // extract_operating_point / topology_waveforms / diagnostics / main_magnetic_inputs
#include "ConverterAnalytical.hpp" // captured_operating_points — full-waveform registry filled during builds
#include "ComponentWaveforms.hpp" // component_waveforms — per-component V/I from one ngspice run
#include "DatasheetModels.hpp"    // derive_datasheet_models — realize real-conduction semiconductors
#include "FidelityJson.hpp"       // PEAS::fidelity_from_json
#include "Clllc.hpp"              // topologies not in the Kirchhoff.hpp umbrella
#include "Pfc.hpp"
#include "Vienna.hpp"
#include "Cmc.hpp"                // common-mode choke — a component designer (no TAS), not a topology row
#include "Dmc.hpp"                // differential-mode choke — component designer + LC propose
#include "CurrentTransformer.hpp" // current transformer — component designer (burden-resistor sensing)
#include "JsonUtil.hpp"           // strip_nulls — schema-valid serialization of typed MAS objects

#include <algorithm>
#include <functional>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

namespace Kirchhoff {
namespace api {

namespace {

// Funnel any exception into the "Exception: ..." string the callers expect (a bad spec surfaces as a
// message, never a crash / thrown exception crossing the ABI).
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

// The topology dispatch table: name -> design_<topo>(spec) then build_<topo>_tas(design). Turns KH's 24
// typed (design_X, build_X_tas) pairs into a single by-name entry point (WebLibMKF's process_converter had
// this; the typed KH core did not).
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

// The spec's operating ambient temperature [°C]: operatingPoints[0].ambientTemperature, else the documented
// emitted default of 25 (SPEC §2). The 24 converter builders each stamp a hardcoded 25 into the TAS and the
// registry; the api layer resolves the real value here and threads it through in one place (apply_ambient +
// restamp_captured_ambient) rather than editing every builder.
double spec_ambient(const json& spec) {
    if (spec.contains("operatingPoints") && spec.at("operatingPoints").is_array()
        && !spec.at("operatingPoints").empty()) {
        const json& op0 = spec.at("operatingPoints").at(0);
        if (op0.contains("ambientTemperature") && op0.at("ambientTemperature").is_number())
            return op0.at("ambientTemperature").get<double>();
    }
    return 25.0;
}

// Overwrite every ambient the builders stamped uniformly at 25 °C with the spec's real ambient: the TAS-level
// operating points AND every magnetic component's operatingPoints[].conditions (what MKF's adviser designs
// the core against). Only touches magnetics that already carry an ambient, so a component designer's own
// ambient is never clobbered.
void apply_ambient(json& tas, double ambientC) {
    if (tas.contains("inputs") && tas.at("inputs").contains("operatingPoints"))
        for (auto& op : tas.at("inputs").at("operatingPoints"))
            if (op.contains("ambientTemperature")) op["ambientTemperature"] = ambientC;
    if (!tas.contains("topology") || !tas.at("topology").contains("stages")) return;
    for (auto& st : tas.at("topology").at("stages")) {
        if (!st.contains("circuit") || !st.at("circuit").is_object()
            || !st.at("circuit").contains("components")) continue;
        for (auto& c : st.at("circuit").at("components")) {
            if (!c.contains("data") || !c.at("data").is_object()) continue;
            json& data = c.at("data");
            if (!data.contains("magnetic")) continue;
            if (!data.contains("inputs") || !data.at("inputs").is_object()
                || !data.at("inputs").contains("operatingPoints")) continue;
            for (auto& op : data.at("inputs").at("operatingPoints"))
                if (op.contains("conditions") && op.at("conditions").contains("ambientTemperature"))
                    op.at("conditions").at("ambientTemperature") = ambientC;
        }
    }
}

json build_tas_for(const std::string& topology, const json& spec) {
    auto it = tas_builders().find(topology);
    if (it == tas_builders().end())
        throw std::runtime_error("Kirchhoff::api: unknown topology '" + topology + "'");
    // Every build starts with an empty full-waveform registry; the builders' named
    // excitations_processed(op, component) calls fill it as they bake the TAS.
    Kirchhoff::analytical::clear_captured_operating_points();
    json tas = it->second(spec);
    // Thread the spec's ambient (default 25 °C) through the artifacts the builders stamped at 25: the TAS
    // operating points, every magnetic's operating-point conditions, and the captured full-waveform registry.
    const double ambientC = spec_ambient(spec);
    apply_ambient(tas, ambientC);
    Kirchhoff::analytical::restamp_captured_ambient(ambientC);
    return tas;
}

// The registry as a json object {"<component>": <full MAS::OperatingPoint>}. Read it immediately after
// build_tas_for — the next build clears it.
json captured_waveforms_json() {
    json out = json::object();
    for (const auto& [component, opJson] : Kirchhoff::analytical::captured_operating_points())
        out[component] = opJson;
    return out;
}

Kirchhoff::ExtractEngine engine_from(const std::string& e) {
    if (e == "ngspice" || e == "NGSPICE") return Kirchhoff::ExtractEngine::NGSPICE;
    if (e == "analytical" || e == "ANALYTICAL" || e.empty()) return Kirchhoff::ExtractEngine::ANALYTICAL;
    throw std::runtime_error("extract engine must be 'analytical' or 'ngspice', got '" + e + "'");
}

// Envelope a designed magnetic COMPONENT (CMC / DMC): the schema-clean Inputs under "inputs" + its
// diagnostics under the component's documented key ("cmcDiagnostics" / "dmcDiagnostics"). The Inputs is
// stripped of the quicktype null members so every emitted object stays schema-valid; the diagnostics
// live OUTSIDE it (never polluting the schema type). Legacy shims spread `inputs` at the root and attach
// the diagnostics sibling themselves — one shape for every component designer.
std::string component_result(const MAS::Inputs& inputs, const char* diagKey, json diagnostics) {
    json inputsJson = Kirchhoff::strip_nulls(json(inputs));
    return json{{"inputs", std::move(inputsJson)}, {diagKey, std::move(diagnostics)}}.dump();
}

}  // namespace

std::string design_tas(const std::string& topology, const std::string& spec) {
    return guarded([&] { return build_tas_for(topology, json::parse(spec)).dump(); });
}

std::string design_tas_full(const std::string& topology, const std::string& spec) {
    return guarded([&] {
        json tas = build_tas_for(topology, json::parse(spec));
        return json{
            {"tas", std::move(tas)},
            {"analyticalWaveforms", captured_waveforms_json()},
        }.dump();
    });
}

std::string generate_ngspice_circuit(const std::string& tas, const std::string& fidelity) {
    return guarded([&] {
        return Kirchhoff::tas_to_ngspice(json::parse(tas), PEAS::fidelity_from_json(json::parse(fidelity)));
    });
}

std::string generate_ltspice_circuit(const std::string& tas, const std::string& fidelity) {
    return guarded([&] {
        return Kirchhoff::tas_to_ltspice(json::parse(tas), PEAS::fidelity_from_json(json::parse(fidelity)));
    });
}

std::string simulate_ngspice(const std::string& tas, const std::string& fidelity) {
    return guarded([&] {
        if (!Kirchhoff::ngspice_in_process_available())
            return json{{"success", false}, {"error", "Kirchhoff built without libngspice"}}.dump();
        const std::string deck = Kirchhoff::tas_to_ngspice(json::parse(tas),
                                                           PEAS::fidelity_from_json(json::parse(fidelity)));
        Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
        json out;
        out["success"] = r.success;
        out["error"] = r.error;
        if (r.success && !r.time.empty()) {
            out["tStart"] = r.time.front();
            out["tEnd"] = r.time.back();
            out["points"] = r.time.size();
            json vecs = json::object();
            for (const auto& kv : r.vectors) {
                const auto& v = kv.second;
                if (v.empty()) continue;
                double sum = 0, mn = v.front(), mx = v.front();
                for (double x : v) { sum += x; mn = std::min(mn, x); mx = std::max(mx, x); }
                vecs[kv.first] = json{{"average", sum / v.size()}, {"min", mn}, {"max", mx}, {"last", v.back()}};
            }
            out["vectors"] = std::move(vecs);
        }
        return out.dump();
    });
}

std::string extract_operating_point(const std::string& tas, const std::string& engine,
                                    const std::string& magneticName) {
    return guarded([&] {
        MAS::OperatingPoint op = Kirchhoff::extract_operating_point(json::parse(tas), engine_from(engine),
                                                                    magneticName);
        json j = op; return j.dump();
    });
}

std::string topology_waveforms(const std::string& tas) {
    return guarded([&] {
        json out = json::array();
        for (auto& m : Kirchhoff::topology_waveforms(json::parse(tas))) {
            json mi = m.inputs;
            out.push_back(json{{"name", m.name}, {"isMain", m.isMain}, {"inputs", std::move(mi)}});
        }
        return out.dump();
    });
}

std::string component_waveforms(const std::string& tas, const std::string& fidelity) {
    return guarded([&] {
        // Like simulate_ngspice, report the no-libngspice case as data (success:false), not an Exception
        // string, so the frontend can branch to "run analytical / rebuild" instead of erroring.
        if (!Kirchhoff::ngspice_in_process_available())
            return json{{"success", false}, {"error", "Kirchhoff built without libngspice"}}.dump();
        return Kirchhoff::component_waveforms(json::parse(tas),
                                              PEAS::fidelity_from_json(json::parse(fidelity))).dump();
    });
}

std::string realize_tas(const std::string& tas) {
    return guarded([&] { return Kirchhoff::derive_datasheet_models(json::parse(tas)).dump(); });
}

std::string diagnostics(const std::string& tas) {
    return guarded([&] { return Kirchhoff::diagnostics(json::parse(tas)).dump(); });
}

std::string main_magnetic_inputs(const std::string& tas) {
    return guarded([&] {
        MAS::Inputs in = Kirchhoff::main_magnetic_inputs(json::parse(tas));
        json j = in; return j.dump();
    });
}

std::string design_magnetic_inputs(const std::string& topology, const std::string& spec) {
    return guarded([&] {
        // The COMPONENT particular cases: a CMC / DMC / current transformer is not a converter — there is
        // no TAS to build; the Inputs come straight from the component designer. Everything else is a
        // switching topology (design_tas + main_magnetic_inputs).
        MAS::Inputs in = [&]() -> MAS::Inputs {
            if (topology == "common_mode_choke" || topology == "cmc" || topology == "commonModeChoke")
                return Kirchhoff::build_cmc_inputs(Kirchhoff::design_cmc(json::parse(spec)));
            if (topology == "differential_mode_choke" || topology == "dmc" || topology == "differentialModeChoke")
                return Kirchhoff::build_dmc_inputs(Kirchhoff::design_dmc(json::parse(spec)));
            if (topology == "current_transformer" || topology == "currentTransformer")
                return Kirchhoff::design_current_transformer(json::parse(spec));
            return Kirchhoff::main_magnetic_inputs(build_tas_for(topology, json::parse(spec)));
        }();
        return Kirchhoff::strip_nulls(json(in)).dump();
    });
}

std::string design_cmc(const std::string& spec) {
    return guarded([&] {
        Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(json::parse(spec));
        json diag{
            {"computedInductance", d.computedInductance},
            {"dominantFrequency", d.dominantFrequency},
            {"dominantImpedance", d.dominantImpedance},
        };
        return component_result(Kirchhoff::build_cmc_inputs(d), "cmcDiagnostics", std::move(diag));
    });
}

std::string design_dmc(const std::string& spec) {
    return guarded([&] {
        Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(json::parse(spec));
        json diag{
            {"computedInductance", d.computedInductance},
            {"computedMinFrequency", d.computedMinFrequency},
            {"computedMaxFrequency", d.computedMaxFrequency},
            {"impedanceAtMinFrequency", d.computedImpedanceAtMinFreq},
            {"numberWindings", d.numberOfWindings},
        };
        return component_result(Kirchhoff::build_dmc_inputs(d), "dmcDiagnostics", std::move(diag));
    });
}

std::string propose_dmc_design(const std::string& spec) {
    return guarded([&] { return Kirchhoff::propose_dmc_design(json::parse(spec)).dump(); });
}

std::string design_current_transformer(const std::string& spec) {
    return guarded([&] {
        return Kirchhoff::strip_nulls(json(Kirchhoff::design_current_transformer(json::parse(spec)))).dump();
    });
}

std::string simulate_cmc_ideal_waveforms(const std::string& spec, double inductance,
                                         double parasiticCapPf, double dvdtVPerNs) {
    return guarded([&] {
        const json specJson = json::parse(spec);
        Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(specJson);
        // Legacy wizard contract: the aux spec may carry the waveform-window controls
        // (ConverterWizardBase injects them); honor them instead of pinning the defaults.
        const int numberOfPeriods = specJson.value("numberOfPeriods", 2);
        const int numberOfSteadyStatePeriods = specJson.value("numberOfSteadyStatePeriods", 10);
        return Kirchhoff::simulate_cmc_ideal_waveforms(d, inductance, parasiticCapPf, dvdtVPerNs,
                                                       numberOfPeriods, numberOfSteadyStatePeriods).dump();
    });
}

std::string simulate_cmc_lisn_waveforms(const std::string& spec, double inductance) {
    return guarded([&] {
        Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(json::parse(spec));
        return Kirchhoff::simulate_cmc_lisn_waveforms(d, inductance).dump();
    });
}

std::string simulate_dmc_waveforms(const std::string& spec, double inductance, double capacitance) {
    return guarded([&] {
        Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(json::parse(spec));
        return Kirchhoff::simulate_dmc_waveforms(d, inductance, capacitance).dump();
    });
}

std::string verify_dmc_attenuation(const std::string& spec, double inductance, double capacitance) {
    return guarded([&] {
        Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(json::parse(spec));
        return Kirchhoff::verify_dmc_attenuation(d, inductance, capacitance).dump();
    });
}

std::string process_converter(const std::string& topology, const std::string& spec, const std::string& engine) {
    return guarded([&] {
        json tas = build_tas_for(topology, json::parse(spec));
        json analyticalWaveforms = captured_waveforms_json();
        MAS::Inputs mainInputs = Kirchhoff::main_magnetic_inputs(tas);
        MAS::OperatingPoint op = Kirchhoff::extract_operating_point(tas, engine_from(engine));
        json inputsJson = mainInputs;
        json opJson = op;
        // The full TAS is returned, so every extra component (output inductor, resonant Lr/Cr, output Co)
        // is already present as its own stage — no separate extra-components extraction needed in HS.
        return json{
            {"topology", topology},
            {"inputs", std::move(inputsJson)},
            {"operatingPoint", std::move(opJson)},
            {"diagnostics", Kirchhoff::diagnostics(tas)},
            // Full analytical waveforms per magnetic (see design_tas_full) — out-of-band from the TAS.
            {"analyticalWaveforms", std::move(analyticalWaveforms)},
            {"tas", std::move(tas)},
        }.dump();
    });
}

}  // namespace api
}  // namespace Kirchhoff
