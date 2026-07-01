// Node smoke test for the NGSPICE-ENABLED webKirchhoff wasm module — proves KH designs AND simulates a
// converter with ngspice in-browser (the same libngspice the frontend loads). Requires the ngspice build:
//   emcmake cmake -S . -B build-wasm-ng -G Ninja -DKIRCHHOFF_BUILD_PYBIND=OFF -DENABLE_NGSPICE=ON \
//     -DNGSPICE_LIB=<...>/ngspice/install/lib/libngspice.so.0.0.14 \
//     -DNGSPICE_INCLUDE_DIR=<...>/ngspice/install/include
//   cmake --build build-wasm-ng --target libKirchhoff
// Needs the ngspice stack/memory sizing in the link flags (STACK_SIZE=64MB, INITIAL_MEMORY=128MB) or the
// sim overflows the default 64 KB stack -> "memory access out of bounds".
import createKirchhoffModule from '../../build-wasm-ng/kirchhoff.js';

const spec = JSON.stringify({
  designRequirements: {
    efficiency: 1.0,
    inputVoltage: { minimum: 21.6, nominal: 24, maximum: 26.4 },
    switchingFrequency: { nominal: 100000 },
    outputs: [{ name: 'out', voltage: { nominal: 12 } }],
  },
  operatingPoints: [{ inputVoltage: 24, outputs: [{ power: 60 }] }],
});

const M = await createKirchhoffModule();
let fails = 0;
const check = (name, cond) => { console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}`); if (!cond) fails++; };

const tas = M.design_tas('buck', spec);
check('design_tas(buck) ok', tas.startsWith('{'));

// simulate_ngspice: the whole point — run the deck in the browser's ngspice.
const sim = JSON.parse(M.simulate_ngspice(tas, JSON.stringify({ origin: 'REQUIREMENTS' })));
check('simulate_ngspice success', sim.success === true);
check('simulate produced a transient', sim.points > 1000);
check('simulate has per-vector summaries', sim.vectors && Object.keys(sim.vectors).length > 0);

// extract_operating_point with the ngspice engine (rebuilds winding currents from the sim).
const op = JSON.parse(M.extract_operating_point(tas, 'ngspice', ''));
check('extract(ngspice) has windings', (op.excitationsPerWinding || []).length >= 1);

console.log(fails === 0 ? '\nALL NGSPICE-WASM CHECKS PASSED' : `\n${fails} FAILED`);
process.exit(fails === 0 ? 0 : 1);
