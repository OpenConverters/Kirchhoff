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

The PSMA-org dependency repos (PEAS/RAS/CAS/SAS/MAS/CIAS + the schema repos
TAS/AAS/CONAS/CTAS used by the validation test) are git **submodules** under
`deps/`. Clone with them, then build:

```bash
git clone --recurse-submodules git@github.com:OpenConverters/Kirchhoff.git
# (or, in an existing clone)  git submodule update --init

cd Kirchhoff
cmake -S . -B build -G Ninja
cmake --build build -j4
ctest --test-dir build            # 21-topology MKF-equivalence gate + schema checks

# a single end-to-end demo:
cmake --build build -j4 --target flyback_demo && ./build/flyback_demo
```

The build pulls each family lib from `deps/<repo>` (override the location with
`-DPSMA_ROOT=<dir>` if you keep the repos elsewhere). The MKF reference-fixture
generator under `tests/reference/` additionally links `libMKF.so` from an
external OpenMagnetics/MKF checkout (`MKF_ROOT`); it is a one-off, not part of
the normal build.

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

**21 topologies** are MKF-equivalence-verified: flyback, boost, buck, forward, two-switch-forward,
SEPIC, Cuk, Zeta, push-pull, **phase-shifted full bridge (PSFB)** (first phase-shift-modulated bridge —
4 switches, leg-to-leg phase, series resonant Lr, full-bridge rectifier), **asymmetric half-bridge
(AHB)** (2-switch complementary-duty isolated bridge with a DC-blocking cap; gain 2·D·(1−D)·Vin/n),
**active-clamp forward (ACF)** (forward with active-clamp reset: aux switch + clamp cap instead of a
demag winding), **four-switch buck-boost (4SBB)** (non-isolated H-bridge buck-boost, single
inductor, four synchronous switches; buck-boost transition region, M=D/(1−D)), **phase-shifted
half-bridge (PSHB)** (3-level NPC leg with split caps + clamp diodes, phase-shift control, bus=Vin/2),
**dual active bridge (DAB)** (isolated bidirectional, two actively-driven full bridges coupled by a
series inductor + transformer; SPS inter-bridge phase D3 sets the transferred power — no output
inductor, so Vout floats to the power-transfer balance), and **isolated buck / Flybuck** (a synchronous
buck whose filter inductor is a coupled inductor — regulated non-isolated primary rail V_pri=D·Vin plus
a flyback-rectified isolated secondary bias rail; the asserted output is the primary, the secondary
loads the coupled inductor internally), and **isolated buck-boost** (inverting Fly-Buck-Boost: a
flyback-style single switch whose non-isolated primary rail is an inverting buck-boost output
V_pri=−Vin·D/(1−D), compared on magnitude like Ćuk, plus an isolated flyback secondary), and
**Weinberg** (current-fed, push-pull-derivative, boost-capable isolated converter: an input coupled
inductor L1 current-feeds a center-tapped push-pull primary; a 4-winding CT transformer drives a
center-tapped full-wave rectifier; boost regime M=1/(2·n·(1−D))), and **LLC resonant** (half-bridge
driving a series Lr-Cr tank in series with the transformer magnetizing Lm; gain set by fsw vs the tank
resonance fr; CT rectifier). The resonant family is compared at a documented **3 %** tolerance: MKF
abstracts the half-bridge to an ideal ±Vbus/2 source + near-ideal diode, while Kirchhoff builds the
real split-cap switching half-bridge — they agree to ~2–2.5 %. The family includes **LLC** (Lr-Cr-Lm) and **SRC** (Lr-Cr series only, operated at resonance). **CLLC** (bidirectional, active bridges on
*both* sides + dual resonant tanks Cr1-Lr1 / Lr2-Cr2) is the exception: its bridges are real switches so
it matches at the tight 2 %, but its series resonant caps make the cold DC operating point singular and
its active synchronous rectifier cannot start into a 0 V output — so it uses the optional
`simulation.initialConditions` field (the assembler precharges the output node and runs the transient
with use-initial-conditions / UIC). Initial-condition support lives entirely in the Kirchhoff TAS
assembler; CIAS, which only emits component cards, is untouched.

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
  ~40 unit checks green. (MAS DATASHEET origin: ABT #170; MKF_MODEL subcircuit hoist: test_real_magnetic;
  remaining real parasitics — e.g. magnetic winding Rdc — still open.)
- ✅ **P2** CIAS atom-brick → ngspice (`.subckt`/cards), tested.
- 🟡 **P3** assembly: done for the flyback in `Kirchhoff/src`; a generic TAS-document walker (any topology)
  in `TAS/src` is pending.
- 🟡 **P4** design: minimal CCM Flyback ported; full CCM/DCM/QRM port + Buck/Boost/LLC/… pending.
- 🟡 **P5** run: native ngspice ✅ (Vout=11.91 V); WASM libngspice (in-browser run) pending.
- ⬜ **P6** results extraction into PEAS/MAS `outputs`.
- ⬜ **P7** CIAS/Kirchhoff pybind+WASM, standalone wheels, git submodules, the other 4 export formats,
  validation/CI/docs.
