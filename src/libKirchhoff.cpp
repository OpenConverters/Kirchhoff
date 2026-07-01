// libKirchhoff — the Emscripten/embind WASM surface for Kirchhoff. A THIN binding layer: every function
// is Kirchhoff::api::* (the shared string-in/string-out JSON facade in KirchhoffApi.hpp). The same facade
// is what MKF / WebLibMKF link natively, so the browser module and the native consumers expose the exact
// same methods with the exact same JSON contract — one API, two front-ends.
//
// Build: emscripten only. CMake target `libKirchhoff` (KIRCHHOFF_BUILD_WASM). ngspice is linked into this
// module (ENABLE_NGSPICE=ON) so the browser can both design AND simulate.

#include <emscripten/bind.h>
#include "KirchhoffApi.hpp"

EMSCRIPTEN_BINDINGS(kirchhoff) {
    namespace em = emscripten;
    namespace api = Kirchhoff::api;
    // per-topology design entry point (topology passed as an arg — the whole 24-row table in one binding)
    em::function("design_tas", &api::design_tas);
    // generic assemble -> deck, and run the deck (ngspice-in-wasm)
    em::function("generate_ngspice_circuit", &api::generate_ngspice_circuit);
    em::function("generate_ltspice_circuit", &api::generate_ltspice_circuit);
    em::function("simulate_ngspice", &api::simulate_ngspice);
    // the extract surface (MKF simulate_and_extract trio replacement) + diagnostics + the adviser's Inputs
    em::function("extract_operating_point", &api::extract_operating_point);
    em::function("topology_waveforms", &api::topology_waveforms);
    em::function("diagnostics", &api::diagnostics);
    em::function("main_magnetic_inputs", &api::main_magnetic_inputs);
    // the one-shot Wizard entry point
    em::function("process_converter", &api::process_converter);
}
