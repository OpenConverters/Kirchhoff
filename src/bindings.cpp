// PyKirchhoff — pybind11 module exposing the Kirchhoff orchestrator (design + generic TAS->deck assembly).
#include <pybind11/pybind11.h>
#include <pybind11_json/pybind11_json.hpp>
#include "Kirchhoff.hpp"     // umbrella: most design_<topo>/build_<topo>_tas + the TAS assembler
#include "FidelityJson.hpp"
// Topologies whose headers are not (yet) pulled into the Kirchhoff.hpp umbrella:
#include "Clllc.hpp"
#include "Pfc.hpp"
#include "Vienna.hpp"
#include "NgspiceRunner.hpp" // in-process libngspice runner (run_ngspice_console)
#include "KirchhoffApi.hpp" // string/JSON facade (run_ngspice_ac)
#include "Cmc.hpp"           // common-mode choke — component designer (MAS::Inputs, no TAS)
#include "Dmc.hpp"           // differential-mode choke — component designer + LC propose
#include "CurrentTransformer.hpp"  // current transformer — component designer
#include "ConverterExtract.hpp"  // main_magnetic_inputs — the adviser's MAS::Inputs from a TAS
#include "JsonUtil.hpp"      // strip_nulls — schema-valid serialization of typed MAS objects
#include "KirchhoffApi.hpp"  // api::select_components / api::bind_part (Kelvin sourcing facade)
#include "CrossRef.hpp"      // kelvin::crossref — deterministic substitute ranker (no LLM)

namespace py = pybind11;
using json = nlohmann::json;

// Bind design_<name>_tas(spec) -> TAS document, mirroring the C++
// build_<name>_tas(design_<name>(spec)) pipeline. One line per topology.
#define BIND_DESIGN(name)                                                                     \
    m.def("design_" #name "_tas",                                                             \
          [](const json& tasInputs) {                                                         \
              return Kirchhoff::build_##name##_tas(Kirchhoff::design_##name(tasInputs));       \
          },                                                                                   \
          py::arg("tas_inputs"),                                                              \
          "Design a " #name " for the given spec and return its full TAS topology document "  \
          "(dict). Pass the result to tas_to_ngspice() to get a runnable deck.")

PYBIND11_MODULE(PyKirchhoff, m) {
    m.doc() = R"pbdoc(
PyKirchhoff — design and simulate power converters from a high-level spec.

Workflow (JSON in, JSON / text out):

    import PyKirchhoff, subprocess
    spec = {
        "designRequirements": {
            "efficiency": 1.0,
            "inputVoltage": {"minimum": 45.6, "nominal": 48, "maximum": 50.4},
            "switchingFrequency": {"nominal": 100000},
            "outputs": [{"name": "out", "voltage": {"nominal": 12}}],
        },
        "operatingPoints": [{"inputVoltage": 48, "outputs": [{"power": 24}]}],
    }
    tas  = PyKirchhoff.design_flyback_tas(spec)                  # 1+2: design & assemble
    deck = PyKirchhoff.tas_to_ngspice(tas, {"origin": "REQUIREMENTS"})  # 3: ngspice deck
    open("flyback.cir", "w").write(deck)
    subprocess.run(["ngspice", "-b", "flyback.cir"])            # prints the measured Vout

All quantities are SI units (V, A, W, Hz). The deck is self-contained: it runs
the transient analysis and measures the output, so `ngspice -b deck.cir` prints Vout.
There is one design_<topology>_tas(spec) per supported topology (see below); the
assembly/simulate steps (tas_to_ngspice / tas_to_ltspice) are topology-agnostic.
)pbdoc";

    // --- design_<topology>_tas: every topology with a design + build_tas pair ---
    BIND_DESIGN(flyback);
    BIND_DESIGN(boost);
    BIND_DESIGN(buck);
    BIND_DESIGN(forward);
    BIND_DESIGN(two_switch_forward);
    BIND_DESIGN(sepic);
    BIND_DESIGN(cuk);
    BIND_DESIGN(zeta);
    BIND_DESIGN(push_pull);
    BIND_DESIGN(psfb);
    BIND_DESIGN(ahb);
    BIND_DESIGN(acf);
    BIND_DESIGN(fsbb);
    BIND_DESIGN(llc);
    BIND_DESIGN(cllc);
    BIND_DESIGN(clllc);
    BIND_DESIGN(src);
    BIND_DESIGN(dab);
    BIND_DESIGN(isolated_buck);
    BIND_DESIGN(isolated_buck_boost);
    BIND_DESIGN(weinberg);
    BIND_DESIGN(pfc);
    BIND_DESIGN(vienna);
    BIND_DESIGN(pshb);

    // --- generic, topology-agnostic assemble -> deck (two SPICE dialects) ---
    m.def("tas_to_ngspice",
          [](const json& tas, const json& fidelity) {
              return Kirchhoff::tas_to_ngspice(tas, PEAS::fidelity_from_json(fidelity));
          },
          py::arg("tas"), py::arg("fidelity"),
          "Assemble any TAS topology document into a runnable ngspice deck (string).\n"
          "fidelity selects the component models, e.g. {\"origin\": \"REQUIREMENTS\"} for an\n"
          "ideal-component deck (other origins: \"DATASHEET\", \"MKF_MODEL\").");

    m.def("run_ngspice_console",
          [](const std::string& deck, double timeout) {
              return Kirchhoff::run_ngspice_console(deck, timeout);
          },
          py::arg("deck"), py::arg("timeout") = 600.0,
          "Run an ngspice deck IN-PROCESS via the integrated libngspice, executing its\n"
          "`.control … .endc` block (run/meas/wrdata), and return the captured console\n"
          "output. The in-process replacement for `ngspice -b <deck>` — no external\n"
          "binary. Parse `.meas` results from the returned text.");

    m.def("run_ngspice_ac",
          [](const std::string& deck) { return json::parse(Kirchhoff::api::run_ngspice_ac(deck)); },
          py::arg("deck"),
          "Run a RAW .ac deck IN-PROCESS via the integrated libngspice and return the\n"
          "complex sweep: {success, error, frequenciesHz, vectors:{name:{re,im}}}.\n"
          "Cross-engine verification entry (Hertz ABT #299).");

    // --- the adviser's magnetic inputs from an assembled TAS: MAS::Inputs (designRequirements +
    //     operatingPoints) for the main magnetic. Feed to PyOpenMagnetics.calculate_advised_magnetics[_fast]
    //     to get a designed core+coil. This is the KH-native replacement for MKF's deleted
    //     design_magnetics_from_converter (design_<topo>_tas -> main_magnetic_inputs -> adviser). ---
    m.def("main_magnetic_inputs",
          [](const json& tas, const std::string& magnetic) {
              return Kirchhoff::strip_nulls(json(Kirchhoff::main_magnetic_inputs(tas, magnetic)));
          },
          py::arg("tas"), py::arg("magnetic") = std::string(""),
          "Extract a magnetic's MAS Inputs (designRequirements + operatingPoints) from an assembled TAS\n"
          "document (any design_<topo>_tas result). `magnetic` names which one (component name / BOM ref);\n"
          "'' picks the main magnetic (most windings). Multi-magnetic topologies (LLC/CLLC/CLLLC,\n"
          "SEPIC/Cuk/Zeta, PSFB/DAB) expose each magnetic separately. Pass the result to\n"
          "PyOpenMagnetics.calculate_advised_magnetics_fast(inputs, N, mode) to design a core+coil.");

    // --- per-magnetic FULL-waveform inputs (real ≥2-point time-domain samples), one entry per magnetic.
    //     main_magnetic_inputs carries processed stats only (waveform: null); for a 'custom'-shaped
    //     excitation (QRM valley-switching, resonant/bridge transformer legs) the adviser needs the real
    //     samples spliced in. This is the source the web BOM enriches from (analyticalWaveforms). Parity
    //     with the embind topology_waveforms. ---
    m.def("topology_waveforms",
          [](const json& tas) {
              json out = json::array();
              for (auto& m : Kirchhoff::topology_waveforms(tas))
                  out.push_back(json{{"name", m.name}, {"isMain", m.isMain},
                                     {"inputs", Kirchhoff::strip_nulls(json(m.inputs))}});
              return out;
          },
          py::arg("tas"),
          "Full-waveform MAS Inputs for EVERY magnetic in an assembled TAS: a list of\n"
          "{name, isMain, inputs} with real time-domain waveforms (unlike main_magnetic_inputs, which\n"
          "carries processed stats only). Use inputs.operatingPoints[].excitationsPerWinding to enrich a\n"
          "'custom'-shaped excitation before advising.");

    // --- common-mode choke: a COMPONENT designer, not a topology (no TAS document) ---
    m.def("design_cmc_inputs",
          [](const json& spec) {
              Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(spec);
              json inputs = Kirchhoff::strip_nulls(json(Kirchhoff::build_cmc_inputs(d)));
              json diag;
              diag["computedInductance"] = d.computedInductance;
              diag["dominantFrequency"] = d.dominantFrequency;
              diag["dominantImpedance"] = d.dominantImpedance;
              return json{{"inputs", std::move(inputs)}, {"cmcDiagnostics", std::move(diag)}};
          },
          py::arg("spec"),
          "Design a common-mode choke from the wizard spec (operatingVoltage, operatingCurrent,\n"
          "lineFrequency, ambientTemperature, and one of: minimumImpedance[], targetInsertionLoss[],\n"
          "parasiticCap_pF+dvdt_V_ns, or desiredInductance). Returns {'inputs': <MAS Inputs dict\n"
          "(designRequirements + CM operating point) for the MagneticAdviser>, 'cmcDiagnostics':\n"
          "{computedInductance, dominantFrequency, dominantImpedance}}.");

    // --- differential-mode choke: component designer + LC "help me" sizing ---
    m.def("design_dmc_inputs",
          [](const json& spec) {
              Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(spec);
              json inputs = Kirchhoff::strip_nulls(json(Kirchhoff::build_dmc_inputs(d)));
              json diag;
              diag["computedInductance"] = d.computedInductance;
              diag["computedMinFrequency"] = d.computedMinFrequency;
              diag["computedMaxFrequency"] = d.computedMaxFrequency;
              diag["impedanceAtMinFrequency"] = d.computedImpedanceAtMinFreq;
              diag["numberWindings"] = d.numberOfWindings;
              return json{{"inputs", std::move(inputs)}, {"dmcDiagnostics", std::move(diag)}};
          },
          py::arg("spec"),
          "Design a differential-mode choke from the wizard spec (configuration, inputVoltage,\n"
          "operatingCurrent, lineFrequency, switchingFrequency?, ambientTemperature, and one of\n"
          "minimumImpedance[] / minimumInductance). Returns {'inputs': <MAS Inputs dict>,\n"
          "'dmcDiagnostics': {computedInductance, computedMin/MaxFrequency, impedanceAtMinFrequency,\n"
          "numberWindings}}.");
    m.def("propose_dmc_design", &Kirchhoff::propose_dmc_design, py::arg("spec"),
          "DMC 'help me with the design' LC sizing → {inductance, capacitance, cutoffFrequency,\n"
          "targetAttenuation_dB, peakCurrent, ...}. Re-call design_dmc_inputs with the proposed\n"
          "inductance as minimumInductance.");

    // --- current transformer: burden-resistor sensing 2-winding transformer ---
    m.def("design_current_transformer_inputs",
          [](const json& spec) {
              return Kirchhoff::strip_nulls(json(Kirchhoff::design_current_transformer(spec)));
          },
          py::arg("spec"),
          "Design a current transformer from the spec (waveformLabel, maximumPrimaryCurrentPeak,\n"
          "frequency, turnsRatio, burdenResistor, ambientTemperature, +optional secondaryDcResistance/\n"
          "dutyCycle/diodeVoltageDrop). Returns the MAS Inputs dict (2-winding transformer + sensing op).");

    // --- CMC EMI/waveform ngspice sims (require an ngspice-enabled build) ---
    m.def("simulate_cmc_ideal_waveforms",
          [](const json& spec, double inductance, double parasiticCap_pF, double dvdt_V_ns) {
              return Kirchhoff::simulate_cmc_ideal_waveforms(
                  Kirchhoff::design_cmc(spec), inductance, parasiticCap_pF, dvdt_V_ns);
          },
          py::arg("spec"), py::arg("inductance"), py::arg("parasitic_capacitance_pF") = 10.0,
          py::arg("dvdt_V_per_ns") = 50.0,
          "Per-winding CM ideal-waveform sim → {success, inputs:{operatingPoints}, converterWaveforms:[],\n"
          "cmcDiagnostics}. success:false (not a throw) when built without libngspice.");
    m.def("simulate_cmc_lisn_waveforms",
          [](const json& spec, double inductance) {
              return Kirchhoff::simulate_cmc_lisn_waveforms(Kirchhoff::design_cmc(spec), inductance);
          },
          py::arg("spec"), py::arg("inductance"),
          "CISPR LISN sweep over the spec impedance frequencies → {success, converterWaveforms:[{frequency,\n"
          "time, inputVoltage, windingCurrents, lisnVoltage, commonModeAttenuation, commonModeImpedance,\n"
          "theoreticalImpedance}]}.");
    m.def("simulate_dmc_waveforms",
          [](const json& spec, double inductance, double capacitance) {
              return Kirchhoff::simulate_dmc_waveforms(Kirchhoff::design_dmc(spec), inductance, capacitance);
          },
          py::arg("spec"), py::arg("inductance"), py::arg("capacitance") = 0.0,
          "DMC LC low-pass sim over the test frequencies (one SHARED filter cap: capacitance arg, else\n"
          "spec filterCapacitance, else fc = fsw/10 auto-size) → {success, converterWaveforms:[{frequency,\n"
          "time, inputVoltage, outputVoltage, inductorCurrent, dmAttenuation}], failedFrequencies?}.");
    m.def("verify_dmc_attenuation",
          [](const json& spec, double inductance, double capacitance) {
              return Kirchhoff::verify_dmc_attenuation(Kirchhoff::design_dmc(spec), inductance, capacitance);
          },
          py::arg("spec"), py::arg("inductance"), py::arg("capacitance") = 0.0,
          "Verify a DMC + filter cap meets the required attenuation (same filter as the sim) →\n"
          "[{frequency, requiredAttenuation, measuredAttenuation|None, theoreticalAttenuation, simulated,\n"
          "passed, message}]. capacitance 0 = auto-size from spec filterCapacitance / fsw.");

    m.def("tas_to_ltspice",
          [](const json& tas, const json& fidelity) {
              return Kirchhoff::tas_to_ltspice(tas, PEAS::fidelity_from_json(fidelity));
          },
          py::arg("tas"), py::arg("fidelity"),
          "Same assembly rendered in the LTspice dialect (a second SPICE backend).");

    // --- Kelvin component sourcing (real parts from the TAS DB, via the shared selector) ---
    auto unwrap = [](const std::string& r) -> json {
        if (r.rfind("Exception:", 0) == 0) throw std::runtime_error(r);
        return json::parse(r);
    };
    m.def("select_components",
          [unwrap](const json& tas, const std::string& data_dir, const std::string& cache_dir,
                   const json& options) {
              return unwrap(Kirchhoff::api::select_components(
                  tas.dump(), data_dir, cache_dir, options.is_null() ? "" : options.dump()));
          },
          py::arg("tas"), py::arg("data_dir"), py::arg("cache_dir") = std::string(),
          py::arg("options") = json::object(),
          "Kelvin sourcing: ranked candidate list per fillable component seed. Returns\n"
          "{components:[{ref,family,kind?,filled,mpn?|deferred?|error?,selection?}]}.");
    m.def("bind_part",
          [unwrap](const json& tas, const std::string& ref, const json& envelope) {
              return unwrap(Kirchhoff::api::bind_part(tas.dump(), ref, envelope.dump()));
          },
          py::arg("tas"), py::arg("ref"), py::arg("envelope"),
          "Stamp a chosen candidate envelope into a component (DATASHEET fidelity).");

    // --- parity with the embind surface the web app drives (web/src/kh.js) ---
    // These were reachable only from WASM, so a Python consumer (the MCP server)
    // could not offer the flows the browser already has. All four are thin
    // wrappers over the same KirchhoffApi facade embind calls.
    m.def("process_converter",
          [unwrap](const std::string& topology, const json& spec, const std::string& engine) {
              return unwrap(Kirchhoff::api::process_converter(topology, spec.dump(), engine));
          },
          py::arg("topology"), py::arg("spec"), py::arg("engine") = std::string("analytical"),
          "One-shot design: spec -> {topology, inputs, operatingPoint, diagnostics, tas}.\n"
          "Topology-dispatching entry point — the generic form of design_<topo>_tas.");

    m.def("simulate_ngspice",
          [unwrap](const json& tas, const json& fidelity) {
              return unwrap(Kirchhoff::api::simulate_ngspice(tas.dump(), fidelity.dump()));
          },
          py::arg("tas"), py::arg("fidelity") = json{{"origin", "REQUIREMENTS"}},
          "Assemble and run a TAS through the IN-PROCESS libngspice, returning the parsed\n"
          "transient result rather than raw console text.");

    m.def("component_waveforms",
          [unwrap](const json& tas, const json& fidelity) {
              return unwrap(Kirchhoff::api::component_waveforms(tas.dump(), fidelity.dump()));
          },
          py::arg("tas"), py::arg("fidelity") = json{{"origin", "REQUIREMENTS"}},
          "Per-component V/I (switches, diodes, caps, resistors) from ONE ngspice run →\n"
          "{engine, referencePeriod, components:[...]}.");

    m.def("realize_tas",
          [unwrap](const json& tas) { return unwrap(Kirchhoff::api::realize_tas(tas.dump())); },
          py::arg("tas"),
          "Add requirements-derived datasheet models (real Rds(on)/Vf) to every semiconductor\n"
          "so a DATASHEET-fidelity deck renders real-conduction devices. Returns the new TAS.");

    m.def("extract_operating_point",
          [unwrap](const json& tas, const std::string& engine, const std::string& magnetic,
                   const json& fidelity) {
              return unwrap(Kirchhoff::api::extract_operating_point(tas.dump(), engine, magnetic,
                                                                    fidelity.dump()));
          },
          py::arg("tas"), py::arg("engine") = std::string("analytical"),
          py::arg("magnetic") = std::string(""),
          py::arg("fidelity") = json{{"origin", "REQUIREMENTS"}},
          "A magnetic's MAS operating point, computed analytically or from an ngspice run.\n"
          "engine in {'analytical','ngspice'}; magnetic '' picks the main one.");

    m.def("diagnostics", [unwrap](const json& tas) {
              return unwrap(Kirchhoff::api::diagnostics(tas.dump()));
          },
          py::arg("tas"), "Design diagnostics for an assembled TAS.");

    // Kelvin's per-COMPONENT selector (the entry point web/src/kh.js drives):
    // load a family's prebuilt index shard once, then rank against one
    // component's design requirements with converter context. Distinct from
    // select_components, which walks a whole TAS off the raw NDJSON catalogue.
    m.def("kelvin_load_shard",
          [unwrap](const std::string& family, const py::bytes& shard) {
              return unwrap(Kirchhoff::api::kelvin_load_shard(family, std::string(shard)));
          },
          py::arg("family"), py::arg("shard_bytes"),
          "Load a prebuilt .kidx index shard for a component family →\n"
          "{family, rowCount, buildId}. Required before kelvin_select on that family.");
    m.def("kelvin_select",
          [unwrap](const std::string& category, const json& requirements, const json& options) {
              return unwrap(Kirchhoff::api::kelvin_select(category, requirements.dump(),
                                                          options.dump()));
          },
          py::arg("category"), py::arg("requirements"), py::arg("options") = json::object(),
          "Rank one family's parts against a component's design requirements →\n"
          "SelectionResult, or {error:'NoCandidates', rejections, ...}. Candidates carry\n"
          "the record's byte span, not the full envelope.");

    m.def("design_magnetic_inputs",
          [unwrap](const std::string& topology, const json& spec) {
              return unwrap(Kirchhoff::api::design_magnetic_inputs(topology, spec.dump()));
          },
          py::arg("topology"), py::arg("spec"),
          "spec -> the magnetic's MAS Inputs for ANY topology, WITHOUT carrying a TAS —\n"
          "the entry point the OpenMagnetics wizards consume.");

    m.def("design_tas_full",
          [unwrap](const std::string& topology, const json& spec) {
              return unwrap(Kirchhoff::api::design_tas_full(topology, spec.dump()));
          },
          py::arg("topology"), py::arg("spec"),
          "Design and assemble in one call, returning the full document set.");

    m.def("determine_pfc_mode",
          [unwrap](const json& spec, double inductance) {
              return unwrap(Kirchhoff::api::determine_pfc_mode(spec.dump(), inductance));
          },
          py::arg("spec"), py::arg("inductance"),
          "Conduction mode (CCM/BCM/DCM) a PFC stage runs in for a given boost inductor.");

    // Kelvin's deterministic cross-reference ranker (no LLM) — header-only, and
    // Kelvin's own pybind module self-disables inside the Kirchhoff build, so it
    // is surfaced here rather than left WASM-only.
    m.def("cross_reference",
          [](const std::string& category, const json& original, const json& candidates,
             const json& options) {
              return kelvin::crossref::cross_reference_json(category, original, candidates, options);
          },
          py::arg("category"), py::arg("original"), py::arg("candidates"),
          py::arg("options") = json::object(),
          "Scored drop-in substitutes for an original part, ranked best-first →\n"
          "{category, original_verified, candidates:[{mpn, status, penalty, params, ...}]}.\n"
          "options: {original_verified: bool, max_results: int}.");
}

#undef BIND_DESIGN
