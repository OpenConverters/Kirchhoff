// WASM smoke test for the in-process ngspice runner: compiled with emcc against the emscripten-built
// libngspice and run under node. Proves the libngspice shared-library API works in WebAssembly (the P5
// in-browser-run goal). NgspiceRunner.cpp is self-contained (only sharedspice.h + std), so this links
// just it — no CIAS/MAS deps. Build/run via scripts/build_wasm_ngspice_smoke.sh.

#include "NgspiceRunner.hpp"
#include <cstdio>

int main() {
    // 10 V source, R=1k, C=1u -> v(out) settles to 10 V after ~10 RC.
    const std::string deck =
        "Vsrc in 0 DC 10\n"
        "R1 in out 1k\n"
        "C1 out 0 1u\n"
        ".options reltol=1e-4\n"
        ".tran 1u 10m 0\n"
        ".end\n";

    if (!Kirchhoff::ngspice_in_process_available()) {
        std::printf("FAIL: built without ENABLE_NGSPICE\n");
        return 1;
    }
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    if (!r.success) {
        std::printf("FAIL: run error: %s (time samples=%zu)\n", r.error.c_str(), r.time.size());
        return 2;
    }
    auto v = r.average("v(out)", 9.5e-3, 10e-3);
    std::printf("WASM in-process ngspice: RC v(out) = %f V (expect ~10.0), %zu time samples\n",
                v ? *v : -1.0, r.time.size());
    return (v && *v > 9.9 && *v < 10.1) ? 0 : 3;
}
