// Node smoke test for the libKirchhoff WASM module (the WebLibMKF converter-surface replacement).
// Build:  source ~/emsdk/emsdk_env.sh && emcmake cmake -S . -B build-wasm-kh -G Ninja \
//         -DENABLE_NGSPICE=OFF -DKIRCHHOFF_BUILD_PYBIND=OFF && cmake --build build-wasm-kh --target libKirchhoff
// Run:    node tests/wasm/test_libkirchhoff.mjs   (expects build-wasm-kh/kirchhoff.js next to build dir)
// Exercises design_tas / diagnostics / extract_operating_point / main_magnetic_inputs /
// extra_components_inputs / generate_ngspice_circuit / process_converter + the Exception-string path.

import createKirchhoffModule from '../../build-wasm-kh/kirchhoff.js';

const spec = JSON.stringify({
  designRequirements: {
    efficiency: 1.0,
    inputVoltage: { minimum: 360, nominal: 400, maximum: 440 },
    switchingFrequency: { nominal: 100000 },
    outputs: [{ name: "out", voltage: { nominal: 12 } }],
  },
  operatingPoints: [{ inputVoltage: 400, outputs: [{ power: 120 }] }],
});

const M = await createKirchhoffModule();
let fails = 0;
const check = (name, cond) => { console.log(`${cond ? "PASS" : "FAIL"}  ${name}`); if (!cond) fails++; };

// 1. design_tas -> TAS
const tas = M.design_tas("llc", spec);
check("design_tas(llc) returns TAS json", tas.startsWith("{") && tas.includes("stages"));
check("design_tas not an Exception", !tas.startsWith("Exception"));

// 2. diagnostics
const diag = JSON.parse(M.diagnostics(tas));
check("diagnostics has computed.resonantCapacitance", diag.computed?.resonantCapacitance > 0);
check("diagnostics has magnetics[]", Array.isArray(diag.magnetics) && diag.magnetics.length === 2);

// 3. extract_operating_point (analytical)
const op = JSON.parse(M.extract_operating_point(tas, "analytical", ""));
check("extract op has 3 windings", op.excitationsPerWinding?.length === 3);

// 4. main_magnetic_inputs
const mi = JSON.parse(M.main_magnetic_inputs(tas));
check("main_magnetic_inputs has designRequirements", !!mi.designRequirements);

// 5. extra_components_inputs
const extra = JSON.parse(M.extra_components_inputs(tas));
check("extra_components has Lr + caps", extra.some(e => e.name === "Lr") && extra.some(e => e.componentType === "capacitor"));

// 6. generate_ngspice_circuit
const deck = M.generate_ngspice_circuit(tas, JSON.stringify({ origin: "REQUIREMENTS" }));
check("ngspice deck is a netlist", deck.includes(".tran") || deck.includes(".param") || deck.includes("\n"));
check("deck not an Exception", !deck.startsWith("Exception"));

// 7. process_converter one-shot
const pc = JSON.parse(M.process_converter("llc", spec, "analytical"));
check("process_converter has inputs+diagnostics+extraComponents+tas",
      !!pc.inputs && !!pc.diagnostics && Array.isArray(pc.extraComponents) && !!pc.tas);

// 8. error path surfaces as Exception string
const bad = M.design_tas("not_a_topology", spec);
check("unknown topology -> Exception string", bad.startsWith("Exception"));

console.log(fails === 0 ? "\nALL WASM CHECKS PASSED" : `\n${fails} WASM CHECK(S) FAILED`);
process.exit(fails === 0 ? 0 : 1);
