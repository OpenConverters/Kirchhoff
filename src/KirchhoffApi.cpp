#include "KirchhoffApi.hpp"

#include "Kirchhoff.hpp"          // design_<topo>/build_<topo>_tas + tas_to_ngspice/ltspice + extract surface
#include "ConverterExtract.hpp"   // extract_operating_point / topology_waveforms / diagnostics / main_magnetic_inputs
#include "FidelityJson.hpp"       // PEAS::fidelity_from_json
#include "Clllc.hpp"              // topologies not in the Kirchhoff.hpp umbrella
#include "Pfc.hpp"
#include "Vienna.hpp"

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

json build_tas_for(const std::string& topology, const json& spec) {
    auto it = tas_builders().find(topology);
    if (it == tas_builders().end())
        throw std::runtime_error("Kirchhoff::api: unknown topology '" + topology + "'");
    return it->second(spec);
}

Kirchhoff::ExtractEngine engine_from(const std::string& e) {
    if (e == "ngspice" || e == "NGSPICE") return Kirchhoff::ExtractEngine::NGSPICE;
    if (e == "analytical" || e == "ANALYTICAL" || e.empty()) return Kirchhoff::ExtractEngine::ANALYTICAL;
    throw std::runtime_error("extract engine must be 'analytical' or 'ngspice', got '" + e + "'");
}

}  // namespace

std::string design_tas(const std::string& topology, const std::string& spec) {
    return guarded([&] { return build_tas_for(topology, json::parse(spec)).dump(); });
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

std::string diagnostics(const std::string& tas) {
    return guarded([&] { return Kirchhoff::diagnostics(json::parse(tas)).dump(); });
}

std::string main_magnetic_inputs(const std::string& tas) {
    return guarded([&] {
        MAS::Inputs in = Kirchhoff::main_magnetic_inputs(json::parse(tas));
        json j = in; return j.dump();
    });
}

std::string process_converter(const std::string& topology, const std::string& spec, const std::string& engine) {
    return guarded([&] {
        json tas = build_tas_for(topology, json::parse(spec));
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
            {"tas", std::move(tas)},
        }.dump();
    });
}

}  // namespace api
}  // namespace Kirchhoff
