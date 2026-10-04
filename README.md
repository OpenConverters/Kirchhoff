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
scripts/run_catch2_tests.sh build   # every Catch2 binary: MKF-equivalence gate, schema checks, …
                                  # (invokes the binaries DIRECTLY, never ctest — see ABT #670)

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

Topologies MKF never modelled have no MKF reference to be equivalent to, and are verified **standalone**:
their own closed form against the ngspice deck. The **inverting buck-boost** (single switch, single
inductor, negative rail Vo = −Vin·D/(1−D); CCM or DCM, diode or synchronous rectifier) is checked in
`test_inverting_buck_boost` — inductor-current NRMSE ≤ 0.006 against the analytical waveform, ripple within
5 %, rail within 5 % of −|Vo| — and delivers its spec at three operating points in `test_requirements`.
**Interleaved buck / boost** (`config.phaseCount` 2–6) is the single-phase cell replicated by
`interleave_stage` with the drives staggered 360°/N; `test_interleaved` checks every phase's inductor current
against the single-phase analytical waveform, the stagger, and the output-capacitor ripple against the
summed-phase charge. The **flying-capacitor multilevel (FCML) buck / boost** (`config.levels` 3–6) stacks N−1
complementary pairs with N−2 flying capacitors under phase-shifted PWM, so the inductor sees a Vtop/(N−1) step at
(N−1)·fs; `analytical_fcml` is its exact piecewise-linear inductor waveform. `test_fcml` runs buck and boost at 3
and 4 levels in ngspice: Vout within 0.5 % of the lossless output less the switches' on-resistance drop, inductor
ripple shape NRMSE ≤ 0.06 and peak-to-peak within 8 %, every flying capacitor within 3 % of k·Vtop/(N−1). The buck's
flying capacitors settle themselves; a boost's have no natural balancing (its inductor faces the stiff source), so
its deck switches the per-pair duty trims a balancing controller settles on and starts on that balanced orbit
(`fcml_balanced_orbit`: the exact periodic steady state with finite capacitors and load).
The **two-switch flyback** (clamp diodes return the leakage energy to Vin, so each
switch sees only Vin) is checked in `test_two_switch_flyback` against the single-switch flyback deck with the
same transformer: Vout within 3 %, switch nodes inside the rails, primary peak and on-time ramp within 5 %.
The **active-clamp flyback** has its own steady-state solver (closed-form segments — linear ramps, the Lk–Cc
clamp resonance, the Lk/Lp–Csw ring of a floating drain — with the periodic state and duty solved together).
Complementary drive: Lk is designed as Cc's resonant partner (π·√(Lk·Cc) = off-time at Vin,min) and emitted as
a nominal leakage requirement. NC-ACF: Lk is a capped parasitic and one clamp pulse before turn-on buys ZVS.
`test_active_clamp_flyback` runs all four drive × clamp-side variants in ngspice: Vout within 3 %, primary-current
NRMSE 0.028 (complementary) / 0.074 (NC), drain at the body-diode drop before Q1 turns on (ZVS).
The hard-switched **half bridge** (split-capacitor bus, bleeders across each half) and **full bridge** drive the
primary at ±Vb for D·T per half period with D < ½ and leave it clamped at zero while every rectifier diode
freewheels; `analytical_hard_switched_bridge` is their exact piecewise-linear steady state for a centre-tapped,
full-bridge or current-doubler rectifier. `test_hard_switched_bridge` runs all six bridge × rectifier pairs in
ngspice: Vout within 0.7 %, primary current within 2 % of the solver mid power interval, zero in the dead time,
peak within 1.5 %, and the half bridge's midpoint at Vin/2 within 0.03 %.
The **LCC** (series Lr–Cs, parallel Cp across the transformer — on the secondary by default, where an HV winding's
capacitance is the parallel capacitor — capacitive output filter) is solved on its exact steady state (`solve_lcc`:
piecewise-linear tank with the rectifier clamping Cp, half-wave antisymmetric, Newton on the period map), since FHA
is poor for a capacitive-filter LCC. The turns ratio comes from the exact gain at the spec frequency, the frequency
range across the input range from the same solver, and every corner is checked inductive (ZVS). `test_lcc` runs
both Cp sides × both bridges in ngspice: 400 V → 2 kV within 0.85 %, tank-current NRMSE 0.007–0.011. The rectifier's
junction capacitance is part of that tank: `config.rectifierJunctionCapacitance` (per diode) is folded out of the
discrete Cp part — with 30 pF diodes the folded design delivers 2017 V, the unfolded one 1765 V.

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
