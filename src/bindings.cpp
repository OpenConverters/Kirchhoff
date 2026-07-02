// PyKirchhoff — pybind11 module exposing the Kirchhoff orchestrator (design + generic TAS->deck assembly).
#include <pybind11/pybind11.h>
#include <pybind11_json/pybind11_json.hpp>
#include "Kirchhoff.hpp"     // umbrella: most design_<topo>/build_<topo>_tas + the TAS assembler
#include "FidelityJson.hpp"
// Topologies whose headers are not (yet) pulled into the Kirchhoff.hpp umbrella:
#include "Clllc.hpp"
#include "Pfc.hpp"
#include "Vienna.hpp"
#include "Cmc.hpp"           // common-mode choke — component designer (MAS::Inputs, no TAS)
#include "Dmc.hpp"           // differential-mode choke — component designer + LC propose
#include "CurrentTransformer.hpp"  // current transformer — component designer
#include "JsonUtil.hpp"      // strip_nulls — schema-valid serialization of typed MAS objects

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
}

#undef BIND_DESIGN
