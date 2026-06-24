// PyKirchhoff — pybind11 module exposing the Kirchhoff orchestrator (design + generic TAS->deck assembly).
#include <pybind11/pybind11.h>
#include <pybind11_json/pybind11_json.hpp>
#include "Kirchhoff.hpp"     // umbrella: most design_<topo>/build_<topo>_tas + the TAS assembler
#include "FidelityJson.hpp"
// Topologies whose headers are not (yet) pulled into the Kirchhoff.hpp umbrella:
#include "Clllc.hpp"
#include "Pfc.hpp"
#include "Vienna.hpp"

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

    m.def("tas_to_ltspice",
          [](const json& tas, const json& fidelity) {
              return Kirchhoff::tas_to_ltspice(tas, PEAS::fidelity_from_json(fidelity));
          },
          py::arg("tas"), py::arg("fidelity"),
          "Same assembly rendered in the LTspice dialect (a second SPICE backend).");
}

#undef BIND_DESIGN
