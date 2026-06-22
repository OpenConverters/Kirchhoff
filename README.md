# Kirchhoff

Design and simulate **any** power-converter topology using **only** the PSMA-org Agnostic-Structure
classes (TAS, CIAS, PEAS, MAS, SAS, RAS, CAS), C++-first (compiles to native + Python via pybind11 +
WASM via emscripten), reusing the math/exporters from MKF.

Approved design & roadmap: `/home/alf/.claude/plans/i-want-everything-in-enchanted-fox.md`.

## Pipeline (proven end-to-end for the ideal Flyback)

```
TAS inputs ──► Kirchhoff::design_flyback ──► CIAS atom-brick ──► CIAS→ngspice ──► ngspice run ──► Vout
            (n, D, Lp, Rload, Cout)   (to_cias per part)   (R/C/L/K/S/D)
```

`flyback_demo` designs a 48 V → 12 V / 24 W / 100 kHz flyback and simulates it: **Vout ≈ 11.91 V**.

## Layout

- **Per-family converters** (each an independent C++ lib, tri-target): `RAS/`, `CAS/`, `SAS/`,
  `MAS/kirchhoff/` under `/home/alf/PSMA/`. Each exposes `<x>_to_cias(peas, fidelity) → CIAS leaf`,
  generates its typed `<X>.hpp` from the JSON schema via **quicktype**, and ships native + `Py<X>`
  (pybind11) + `lib<X>` (emscripten/embind) targets.
- **CIAS converter** (`PSMA/CIAS/`): `CiasToNgspiceConverter` turns a CIAS atom-brick into ngspice
  `.subckt`/cards (LTspice/PSIM/Simba/NL5 to follow).
- **Shared** (`PSMA/PEAS/src/`): `Fidelity.hpp` (the compulsory `{origin, allowStoredModelParams,
  curveFit}` directive), `Dimension.hpp` (`resolve_dimensional_values`, MKF semantics), `FidelityJson.hpp`.
- **Kirchhoff** (`OpenConverters/Kirchhoff/`): `src/Flyback.{hpp,cpp}` (design + assembly + deck emission),
  `examples/flyback_demo.cpp`.

## Build & run

```bash
# one family (native + pybind)
cmake -S /home/alf/PSMA/RAS -B /home/alf/PSMA/RAS/build -G Ninja -DPython_EXECUTABLE=$(which python3)
cmake --build /home/alf/PSMA/RAS/build -j4 && /home/alf/PSMA/RAS/build/ras_tests

# WASM target for a family
source /home/alf/emsdk/emsdk_env.sh
emcmake cmake -S /home/alf/PSMA/RAS -B /home/alf/PSMA/RAS/build-wasm -G Ninja
cmake --build /home/alf/PSMA/RAS/build-wasm -j4    # -> libRAS.wasm.{js,wasm}

# end-to-end flyback (links all family + CIAS libs, runs ngspice)
cmake -S /home/alf/OpenConverters/Kirchhoff -B build -G Ninja
cmake --build build -j4 --target flyback_demo && ./build/flyback_demo
```

Requirements: cmake≥3.15, ninja, a C++17 compiler, `quicktype` (Node) on PATH, `ngspice`, and (for
WASM) emsdk at `/home/alf/emsdk`. All builds use Ninja with `-j4`.

## MKF equivalence test (`tests/test_mkf_equivalence.cpp`)

The contract: **for given inputs, a Kirchhoff topology's design+simulation must reproduce MKF's own
design+simulation with ideal components**, within tolerance. MKF is the reference.

- `tests/reference/gen_mkf_reference.cpp` links MKF (`libMKF.so`) and runs MKF's own
  `process_design_requirements()` + ideal-component ngspice sim for each topology, capturing the
  design params, the ideal deck text, and the settled simulator outputs (Vout/Iout/Pin/Pout/η) into
  `tests/reference/<topology>.mkf.json`. Regenerate with `bash tests/reference/build_and_run.sh`
  (needs MKF built at `$MKF_ROOT/build`) whenever MKF's converter models change.
- `test_mkf_equivalence` then (1) re-runs MKF's stored deck through the same `ngspice` as a
  reproducibility guard, and (2) runs Kirchhoff's own `design_* → build_*_tas → tas_to_ngspice` pipeline on
  the same inputs and asserts Vout/Iout/η agree with MKF within 2 % / 2 % / 3 %.

To make Kirchhoff match MKF, Kirchhoff's **ideal device models are pinned to MKF's** (SAS ideal switch
`RON=0.01`, ideal diode `Vf=0.8334 V → IS=1e-14`; CIAS `SW VH=0.5/ROFF=1e6`, diode `RS=1e-6`,
transformer `K=0.9999`) and each topology's design math is a faithful port of MKF's
`process_design_requirements()`.

**13 topologies** are MKF-equivalence-verified: flyback, boost, buck, forward, two-switch-forward,
SEPIC, Cuk, Zeta, push-pull, **phase-shifted full bridge (PSFB)** (first phase-shift-modulated bridge —
4 switches, leg-to-leg phase, series resonant Lr, full-bridge rectifier), **asymmetric half-bridge
(AHB)** (2-switch complementary-duty isolated bridge with a DC-blocking cap; gain 2·D·(1−D)·Vin/n), and
**active-clamp forward (ACF)** (forward with active-clamp reset: aux switch + clamp cap instead of a
demag winding), and **four-switch buck-boost (4SBB)** (non-isolated H-bridge buck-boost, single
inductor, four synchronous switches; buck-boost transition region, M=D/(1−D)).

Current agreement: **boost** Vout/Iout 0.2 %, η 0.2 %; **flyback** Vout/Iout 1.9 %, η 2.5 %; **PSFB**
Vout/Iout ~0.9 % (efficiency directional — Kirchhoff's ideal switches beat MKF's lossy rectifier diodes).

> **Note (surfaced, not papered over):** MKF's own PtP reference tests settle 400 switching periods.
> For a 100 µF/24 Ω boost output that is only ~1.7·RC — the captured Vout/η is a *transient*, not
> steady state. Both the reference generator and the Kirchhoff side here run to ≥10·RC so they compare at
> true steady state (where they agree to <0.2 %). New topologies: add them to the generator + the test
> and regenerate fixtures.

## Status (see the plan for the full roadmap)

- ✅ **P0** reset + toolchain.
- ✅ **P1** per-family `to_cias` (RAS/CAS/SAS/MAS), ideal + datasheet, tri-target (native+pybind+WASM),
  ~40 unit checks green. (Real parasitics + MAS DATASHEET/MKF_MODEL origins = P4.)
- ✅ **P2** CIAS atom-brick → ngspice (`.subckt`/cards), tested.
- 🟡 **P3** assembly: done for the flyback in `Kirchhoff/src`; a generic TAS-document walker (any topology)
  in `TAS/src` is pending.
- 🟡 **P4** design: minimal CCM Flyback ported; full CCM/DCM/QRM port + Buck/Boost/LLC/… pending.
- 🟡 **P5** run: native ngspice ✅ (Vout=11.91 V); WASM libngspice (in-browser run) pending.
- ⬜ **P6** results extraction into PEAS/MAS `outputs`.
- ⬜ **P7** CIAS/Kirchhoff pybind+WASM, standalone wheels, git submodules, the other 4 export formats,
  validation/CI/docs.
