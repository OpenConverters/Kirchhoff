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
    // design + the FULL analytical waveforms (per-magnetic MAS::OperatingPoint, out-of-band from the TAS)
    em::function("design_tas_full", &api::design_tas_full);
    // generic assemble -> deck, and run the deck (ngspice-in-wasm)
    em::function("generate_ngspice_circuit", &api::generate_ngspice_circuit);
    em::function("generate_ltspice_circuit", &api::generate_ltspice_circuit);
    em::function("simulate_ngspice", &api::simulate_ngspice);
    // the extract surface (MKF simulate_and_extract trio replacement) + diagnostics + the adviser's Inputs
    em::function("extract_operating_point", &api::extract_operating_point);
    em::function("topology_waveforms", &api::topology_waveforms);
    // per-component V/I (switches, diodes, caps, resistors) from one ngspice run
    em::function("component_waveforms", &api::component_waveforms);
    // realize requirements-derived datasheet models (real conduction) onto a spec-designed TAS
    em::function("realize_tas", &api::realize_tas);
    // Kelvin component sourcing: candidate lists per seed + bind a chosen part (DATASHEET fidelity).
    // In the browser, dataDir/cacheDir are the Emscripten FS paths the worker mounts shards into.
    em::function("select_components", &api::select_components);
    em::function("bind_part", &api::bind_part);
    em::function("diagnostics", &api::diagnostics);
    em::function("main_magnetic_inputs", &api::main_magnetic_inputs);
    // the one-shot Wizard entry point
    em::function("process_converter", &api::process_converter);
    // spec -> magnetic MAS::Inputs for any topology (incl. "common_mode_choke"), no TAS round-trip
    em::function("design_magnetic_inputs", &api::design_magnetic_inputs);
    // the CMC one-shot (calculate_cmc_inputs replacement): {"inputs", "cmcDiagnostics"}
    em::function("design_cmc", &api::design_cmc);
    // the DMC one-shot (calculate_dmc_inputs) + its "help me" LC sizing (propose_dmc_design)
    em::function("design_dmc", &api::design_dmc);
    em::function("propose_dmc_design", &api::propose_dmc_design);
    // the current-transformer one-shot (process_current_transformer replacement)
    em::function("design_current_transformer", &api::design_current_transformer);
    // CMC EMI/waveform sims (simulate_cmc_ideal_waveforms + simulate_cmc_lisn_waveforms)
    em::function("simulate_cmc_ideal_waveforms", &api::simulate_cmc_ideal_waveforms);
    em::function("simulate_cmc_lisn_waveforms", &api::simulate_cmc_lisn_waveforms);
    // DMC EMI/attenuation sims (simulate_dmc_waveforms + verify_dmc_attenuation)
    em::function("simulate_dmc_waveforms", &api::simulate_dmc_waveforms);
    em::function("verify_dmc_attenuation", &api::verify_dmc_attenuation);
}
