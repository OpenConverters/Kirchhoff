// PyKirchhoff — pybind11 module exposing the Kirchhoff orchestrator (design + generic TAS->ngspice assembly).
#include <pybind11/pybind11.h>
#include <pybind11_json/pybind11_json.hpp>
#include "Flyback.hpp"
#include "Boost.hpp"
#include "TasAssembler.hpp"
#include "FidelityJson.hpp"

namespace py = pybind11;
using json = nlohmann::json;

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
)pbdoc";

    m.def("design_flyback_tas",
          [](const json& tasInputs) { return Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(tasInputs)); },
          py::arg("tas_inputs"),
          "Design a flyback for the given spec and return its full TAS topology document (dict).\n"
          "Pass the result to tas_to_ngspice() to get a runnable ngspice deck.");

    m.def("design_boost_tas",
          [](const json& tasInputs) { return Kirchhoff::build_boost_tas(Kirchhoff::design_boost(tasInputs)); },
          py::arg("tas_inputs"),
          "Design a boost for the given spec and return its full TAS topology document (dict).\n"
          "Pass the result to tas_to_ngspice() to get a runnable ngspice deck.");

    m.def("tas_to_ngspice",
          [](const json& tas, const json& fidelity) {
              return Kirchhoff::tas_to_ngspice(tas, PEAS::fidelity_from_json(fidelity));
          },
          py::arg("tas"), py::arg("fidelity"),
          "Assemble any TAS topology document into a runnable ngspice deck (string).\n"
          "fidelity selects the component models, e.g. {\"origin\": \"REQUIREMENTS\"} for an\n"
          "ideal-component deck (other origins: \"DATASHEET\", \"MKF_MODEL\").");
}
