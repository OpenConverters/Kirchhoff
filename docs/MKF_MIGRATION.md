# MKF → Kirchhoff converter-model migration

**Goal:** Kirchhoff (KH) becomes the single, exportable, magnetics-independent home for all
power-converter models. OpenMagnetics/MKF drops its `src/converter_models/` and reverts to being a
pure magnetics library that *consumes* KH.

This doc inventories what lives in MKF today and recommends what moves, what stays, and the one
architectural decision that makes the cut clean.

## Where things stand

MKF's `MAS::Topology` enum has **27 entries**. **24 are true power converters** — and KH already
re-implements all 24 (faithful ports, 21 MKF-equivalence-gated). The other **3 are passive magnetic
components**, labelled as such in MKF's own `FUTURE_TOPOLOGIES.md`:

| MKF `converter_models/` entry | Nature | In KH? | Verdict |
|---|---|---|---|
| 24 power converters (Buck…Vienna) | power conversion | ✅ all 24 | KH is/becomes the owner |
| `CommonModeChoke` | EMI choke (magnetic) | ❌ | **stays in MKF** |
| `DifferentialModeChoke` | EMI choke (magnetic) | ❌ | **stays in MKF** |
| `CurrentTransformer` | sense magnetic | ❌ | **stays in MKF** |

`DifferentialModeChoke` and `CurrentTransformer` don't even inherit MKF's `Topology` base — they're
magnetics characterization, not converters. They should **not** move to KH.

But "KH already has the 24" understates MKF's maturity. MKF's models are "DAB-quality": each carries
3–5 **datasheet reference designs**, **NRMSE-vs-SPICE** gates, **ZVS diagnostics**, and — crucially —
**rectifier-topology variants** (CT / FB / CD / VD = centre-tapped / full-bridge / current-doubler /
voltage-doubler) for the bridge and resonant families. KH currently implements one rectifier per
topology. So the migration is a **consolidation**, not a copy: bring MKF's modeling *richness* into
KH's cleaner *exportable* architecture.

(Conversely, KH already surpasses MKF in places: KH's Vienna is a real 3-phase netlist where MKF's is
a single-phase peak-of-line model — "full 3-phase netlist deferred"; and KH expresses PFC/Vienna/CLLLC
closed-loop control in CIAS. Those KH advances do not regress.)

## What should MOVE to Kirchhoff

Ranked by value × time-sensitivity:

1. **Reference-design corpus + PtP/NRMSE validation harness** *(highest priority — time-sensitive)*.
   `tests/Test*ReferenceDesignsPtp.cpp`, `PtpHelpers.h`, `ConverterPortChecks.h`,
   `NgspiceTestHelpers.h`, `tests/testData/*.json` (real datasheet operating points: LM5160/LM5180
   Fly-Buck, TIDA-00709, Infineon REF-DAB11K, KIT 20 kW CLLC, …). **This is KH's future validation
   oracle.** Today KH's equivalence gate depends on MKF *generating* reference fixtures; the moment
   MKF drops `converter_models/`, that generator disappears. KH must absorb these reference designs as
   its own standalone golden corpus **before** MKF deletes them.
2. **Rectifier-topology variants (CT / FB / CD / VD).** ✅ **DONE (2026-06-30).** All five
   rectifier-aware topologies now select the secondary variant from `config.rectifierType` (no schema
   change), via a shared `src/Rectifier.hpp` (`RectifierType` enum + parser) and `cfg::get_str`:
   - **LLC** — CT (default) / FB / CD / VD
   - **SRC** — CT (default) / FB / CD  (no VD)
   - **PSFB / PSHB / AHB** — FB (default) / CT / CD  (KH's CT is a *real* two-half-winding centre tap,
     not MKF's "fake CT"; AHB's MAS fourth variant is AHB_FLYBACK, not VD)
   Each variant is **simulation-validated** (no MKF fixture exists for non-default rectifiers; 8 new
   `[rectifier]` tests). Defaults keep the existing decks byte-identical so the MKF fixtures are
   unchanged. Engineering notes captured for the rest of the migration: the split-cap resonant decks are
   ngspice element-order-sensitive (the CT branch must reproduce the validated card order); the
   current-doubler needs a loop-breaker R (winding+Lo1+Lo2 is an all-inductor loop), a multi-period
   measurement window (its LC filter rings below fsw), and a halved turns ratio (delivers ~½ Vsec).
3. **Multi-simulator exporters.** KH's README promises "LTspice/PSIM/Simba/NL5 to follow"; delivering
   these into CIAS is the "simulate in general, any simulator" goal. **Status (2026-06-30): designed,
   not yet built — gated on validation tooling (a deliberate decision, not papered over).**

   - **Template to mirror:** `CIAS::CiasToNgspiceConverter::to_cards` (`deps/CIAS/src/CiasCircuitConverter.cpp`)
     — build a `(component,pin)→node` map from `circuit.connections` (a net exposed at a port takes the
     port name, else the connection's local name), then dispatch on the PEAS discriminator
     (`resistor` / `capacitor` / `semiconductor`{mosfet,diode} / `magnetic` / `analog`{comparator,
     multiplier,summer,integrator}), pulling values with the throwing `nominal_at` (no fallbacks).
   - **Already present:** ngspice + LTspice, both as `SpiceDialect` values of the same emitter (the only
     card-level delta is the behavioural ternary `(c)?(a):(b)` vs `if(c,a,b)`). So the *SPICE-text* half
     of the README promise is effectively met.
   - **The hard part (PLECS / Simba / NL5):** none of these use shared net names. PLECS is a
     brace-delimited schematic, Simba is JSON with GUID device IDs, NL5 is XML with **integer** nodes —
     all three reference components by **per-instance terminal index + synthesized XY geometry** (PLECS
     and Simba also need routed wire/connector points). So a CIAS shared-net `connections` model must be
     *lowered* to terminal pairs + an **auto-layout pass** (assign coordinates; route wires). They cannot
     be a third `SpiceDialect` value; each needs its own `CiasTo<Format>Converter` class and a parallel
     `tas_to_<format>` assembler path (the SPICE deck-wrapper — sources, `.tran`/`.options`/`.control` —
     is SPICE-shaped and must be re-expressed per format).
   - **Why not shipped overnight:** (a) **no validation tooling** — PLECS/Simba/NL5 are not installed, so
     an exporter can only be checked *structurally* (element + node-incidence histogram, like
     `tests/test_ltspice_backend.cpp`), NOT confirmed to actually open/run in the target tool; and (b) the
     **analog control blocks** (comparator/multiplier/integrator) and the **magnetic coupling (K)** have
     non-obvious, tool-specific representations that cannot be verified blind. Shipping guessed-at,
     tool-unvalidated format code to a shared submodule `main` would violate the verify-before-push /
     no-papering-over rules. **Recommended order when a tool is available: NL5 first** (integer nodes,
     XML string-templated, closest to the netlist model → highest structural confidence), then Simba
     (structured `ordered_json`), then PLECS (hardest — full geometry + base64 init blocks). Effort
     ranking: LTspice ≪ NL5 < Simba ≲ PLECS.
   - **Recommended architecture (low-risk groundwork, independent of the tools):** factor the shared
     net-resolution + `nominal_at` value-extraction out of `CiasToNgspiceConverter` into a reusable
     internal `CiasCircuitWalk` header, so every backend shares one correct, tested net model and only
     supplies a per-component emission table. This refactor is behavior-preserving (guarded by the
     existing ngspice/ltspice/cias tests) and can land before any new format. **Open decision for the
     user (see end):** build NL5 now with structural-only validation, or wait for a tool + the layout pass.
4. **`NgspiceRunner` (shared-library/libngspice path) + results parser.** ✅ **Native half DONE
   (2026-06-30).** KH now has `src/NgspiceRunner.{hpp,cpp}` — an in-process libngspice runner
   (`run_ngspice_in_process(deck)`) using the `<ngspice/sharedspice.h>` API (`ngSpice_Init` with a
   per-run `userData` capture struct, `ngSpice_Circ` → `run` → data-callback vector capture →
   `remcirc`), with `NgspiceRunResult::average()` reproducing `meas tran AVG` by trapezoidal
   integration. CMake gates it behind `ENABLE_NGSPICE` (auto-ON when libngspice + the header are found;
   the stub throws when OFF — no silent no-op). Validated: `test_ngspice_runner` runs an RC deck and a
   boost deck in-process and matches the `ngspice -b` CLI exactly (boost Vout 23.9595 V both ways).
   The real-magnetic co-sim (`simulate_magnetic_circuit`, `extract_operating_point(Magnetic)`) is NOT
   ported — it stays in MKF (Decision 1). **WASM half (P5) pending** — see below.

   **WASM (in-browser) ngspice — ✅ VALIDATED end-to-end (2026-06-30).** ngspice 45.2 compiles to
   WebAssembly and `Kirchhoff::run_ngspice_in_process` runs a deck through it under node. Reproducible:
   `scripts/build_ngspice_wasm.sh` (emscripten 3.1.51, `emconfigure --with-ngshared` + the four ngspice
   source patches — accept `.wasm`, guard `main()`, drop `getrusage`/init-file read under
   `__EMSCRIPTEN__` — and `-fwasm-exceptions -sSUPPORT_LONGJMP=wasm`) builds `libngspice.so.0.0.14` (a
   WASM static archive); `scripts/run_wasm_ngspice_smoke.sh` emcc-compiles `tests/wasm_ngspice_smoke.cpp`
   + `src/NgspiceRunner.cpp` against it and runs under node → **RC v(out) = 10.000000 V, 10008 time
   samples, exit 0.** The runner carries the two WASM workarounds: `set no_mem_check` (no `/proc/meminfo`
   under WASM, harmless on native) and treating a non-empty time vector as completion (the sync WASM
   build's `run` returns without firing the bg-thread callback) — the runner's wait loop handles the
   latter via `ngSpice_running()` + the captured time vector. Remaining productionization (not blocking):
   a CMake `EMSCRIPTEN`-gated `ExternalProject_Add(ngspice …)` + an embind module that exposes the full
   `design → tas_to_ngspice → run` pipeline to JS (today the WASM run is proven via the standalone smoke;
   the scripts are the recipe to fold into CMake). The ANALYTICAL engine (item 3) remains the lighter
   no-libngspice browser path.
5. **Magnetics-independent analytical kernels.** `PwmBridgeSolver.h` (piecewise sub-interval tank
   solver for DAB/PSFB/PSHB), `PfcControllerDesign.*`, `PfcControllerSubcircuits.h` (ideal opamp /
   comparator `.subckt` primitives). Consolidate with KH's own re-derivations.
6. **Design-knowledge docs.** `CONVERTER_MODELS_GOLDEN_GUIDE.md` (the DAB-quality spec), per-topology
   `*_PLAN.md`, and `FUTURE_TOPOLOGIES.md` — reconcile the latter with `TOPOLOGY_ROADMAP.md` (they
   agree on the open gaps: totem-pole PFC is flagged "verify totem-pole coverage"; full 3-phase Vienna;
   rectifier variants).

## What should STAY in MKF

1. **The 3 passive magnetic components** (CMC / DMC / Current Transformer) — magnetics, not conversion.
2. **All magnetics math** — `physical_models/MagnetizingInductance`, `LeakageInductance`,
   `WindingOhmicLosses`; `constructive_models/Magnetic`/`Core`/`Coil`; and the magnetic→subcircuit
   exporter (`CircuitSimulatorNgspice::export_magnetic_as_subcircuit`, saturating B-source, fracpole
   AC-resistance ladder). Per the standing rule *all magnetics math lives in MKF* — KH must never
   re-implement these.
3. **`MagneticAdviser::get_advised_magnetic_from_converter`** and the real-magnetic co-simulation path
   (`NgspiceRunner::simulate_magnetic_circuit`). These stay on the magnetics side — but their
   dependency **inverts** (see below).

## The architectural crux — keep the dependency one-directional

The converter↔magnetics coupling is narrow and one-directional: converters call a small set of
magnetics services (`calculate_inductance_from_number_turns_and_gapping`,
`calculate_leakage_inductance_all_windings`, `simulate_magnetic_circuit`); magnetics never calls
converters except through the adviser. To avoid a **circular dependency** after the split:

- **Kirchhoff stays magnetics-independent.** It already is — KH's normal build does not link
  `libMKF`; only the optional reference-fixture generator does. KH emits ideal/datasheet decks and
  exposes the magnetic as a port/subcircuit placeholder. It must **not** absorb the
  `MagnetizingInductance`/`LeakageInductance`/`Magnetic` call sites.
- **MKF's adviser consumes KH.** `get_advised_magnetic_from_converter` imports KH (for converter
  design params + the converter netlist) and MKF magnetics (for Lmag/Lk + the magnetic subcircuit),
  and stitches them for the real-magnetic co-simulation.

Result: **KH depends on nothing in MKF; MKF's adviser depends on KH.** Then MKF deletes
`src/converter_models/` (except the 3 passive magnetic components) and its converter tests.

Shared, owned by neither: the **`MAS::Topology` enum + generated `MAS::<Topology>` structs** stay in
the MAS schema repo (the cross-repo contract). Both sides consume MAS; that's already how KH works.

## Suggested sequencing

- **Phase 0 (now, before MKF changes anything):** freeze MKF's converter PtP reference designs +
  datasheet `testData` as KH's golden corpus. Without this, KH loses its validation oracle the day
  MKF drops converters.
- **Phase 1:** port the rectifier variants (CT/FB/CD/VD) and the multi-simulator exporters +
  libngspice runner into KH/CIAS.
- **Phase 2:** invert the adviser seam — point `get_advised_magnetic_from_converter` at KH.
- **Phase 3:** MKF deletes `src/converter_models/` (keep CMC/DMC/CT) + converter tests; reverts to
  pure magnetics.

## Decisions (resolved 2026-06-29)

1. **Real-magnetic operating-point path stays in MKF's adviser.** KH stays magnetics-free; the
   real-magnetic co-sim (`simulate_magnetic_circuit`, `MagnetizingInductance`, `LeakageInductance`)
   is not ported. Confirmed.
2. **Port all rectifier variants now** (CT / FB / CD / VD), not just KH's current gaps. Confirmed.
3. **CMC / DMC / Current-Transformer stay in MKF** — they are magnetic components, never come to KH.
   Confirmed.

## Also moving: the analytical operating-point engine (default stays SPICE)

MKF carries, per topology, an **analytical** `process_operating_points()` (closed-form /
piecewise-linear waveforms) plus shared analytical kernels — `PwmBridgeSolver.h` (DAB/PSFB/PSHB
sub-interval tank solver), the Clllc 4-state ODE solver, the Dab analytical-modulation modes — all
NRMSE-gated against SPICE. The **ideal-coupling (K=1) analytical path is magnetics-independent**
(only the `process_operating_points(Magnetic)` path needs a real core, and that stays in MKF), so it
is cleanly portable.

Port it as a **selectable run engine** on the extraction step, default unchanged:

- `SPICE` (default) — current behaviour, highest fidelity, ideal/datasheet decks.
- `ANALYTICAL` (opt-in) — fast, simulator-free operating points/waveforms (ideal magnetics).
- (`MKF real-magnetic co-sim` remains the third tier, on the MKF side.)

Why it's worth porting: (a) **speed** — no ngspice spawn per candidate, so design sweeps/optimization
loops get orders-of-magnitude faster; (b) **runs anywhere** — pure C++/WASM, no libngspice needed,
which directly serves KH's in-browser P5 goal; (c) **self-validation** — the analytical-vs-SPICE
NRMSE check becomes intrinsic to KH, cutting KH's dependence on MKF as the reference oracle (synergy
with Phase 0). Caveat: analytical = ideal magnetics; it is an estimate, not a replacement for the
SPICE deck or the real-magnetic co-sim.

## Topology taxonomy enum: MAS → PEAS (schema RFC, not a KH blocker)

**Directive:** KH must use the canonical typed enum **`MAS::Topology` (→ `PEAS::Topology` after the
move)** — not the ad-hoc topology *string* it carries in `designRequirements` today. The string is a
smell: KH re-implements MKF's converters, so it should share MKF's typed taxonomy (generated from the
schema, like the `MAS::<Topology>` structs are via quicktype), giving one source of truth for "which
converter" across KH, MKF and the advisers.

The converter-type enum currently lives in the **magnetics** schema (MAS). Once converters belong to
KH (power electronics), a converter taxonomy living in MAS is a layering inversion — it conceptually
belongs in **PEAS** (the power-electronics schema). Plan:

- **Step 1 (KH, no schema change):** replace KH's topology string with the generated `MAS::Topology`
  enum — KH becomes a typed consumer of the canonical taxonomy now, same source of truth as MKF.
- **Step 2 (schema RFC):** move the **taxonomy enum** MAS → PEAS; KH and MKF switch to `PEAS::Topology`.
  Governance-controlled, breaking change (every MAS consumer; generated `MAS::<Topology>` structs;
  `MasMigration` string maps) — must go through the MAS/PEAS RFC/committee process; do **not** edit
  schema files unilaterally.
- Granularity: only the **taxonomy enum** moves to PEAS; the per-topology **magnetic-design data
  structs** (`MAS::Flyback`, turns/gapping, …) legitimately stay in MAS. Confirm PEAS vs a
  TAS/converter schema as the home with the schema owners.

Recommendation: do **Step 1 now** (KH adopts `MAS::Topology`), schedule **Step 2** via RFC timed with
the migration (ownership of "what a converter topology is" shifts to the power-electronics side then).

### Status (2026-06-30) — the schema move was ALREADY DONE; KH now uses `PEAS::Topology`

Investigation finding: **the taxonomy enum already lives in the PEAS schema**, not MAS. It is defined
once at `PEAS/schemas/utils.json#/$defs/topology` (verified: `flybackConverter` has **zero** hits in
either repo's `MAS/schemas/`, one hit in `PEAS/schemas/utils.json`); the `$def`'s own description says
*"PEAS HOSTS this shared vocabulary … MAS OWNS its content."* MAS's `designRequirements` only `$ref`s it
transitively via `designRequirementsBase`. So **no schema edit is needed** (and none was made — the
governance rule is respected). Both KH and MKF already generate their `MAS::Topology` C++ enum with
`PEAS/schemas/utils.json` in the quicktype `-S` graph, i.e. they were *already* consuming the
PEAS-owned vocabulary; only the C++ namespace *label* was `MAS::`.

- **KH (done, `<commit>`):** `PEAS_Topology.hpp` is now generated at build time from the canonical
  `$def` (`scripts/gen_peas_topology.sh` → quicktype `--namespace PEAS`, wired as a CMake custom
  command like MAS.hpp). `src/Topology.hpp` repoints `using Topology = PEAS::Topology`. The ~24
  converter `.cpp` and `ComponentRequirements.hpp` are unchanged (they use the `Kirchhoff::Topology`
  alias + JSON only — verified no `get_topology()` struct coupling). Identical values + serialization
  → byte-identical assembled JSON; full suite unchanged (134/32).
- **MKF (no change — deliberately):** MKF is **already** consuming the PEAS-owned enum (its generated
  `MAS::Topology` values come from `PEAS/schemas/utils.json`). A C++ repoint of MKF to `PEAS::Topology`
  is **NOT done**: ~67 files use `MAS::Topology`, and ~14 are *bound to the generated struct field*
  (`DesignRequirements::get_topology()/set_topology()` are typed `MAS::Topology` by quicktype and can't
  be repointed without regenerating MAS.hpp itself). Introducing a second, identical-but-distinct
  `PEAS::Topology` alongside `MAS::Topology` across 30+ `switch`/`map` sites is exactly the
  dual-enum / total-switch hazard the numeric-correctness guardrails warn about, for **zero functional
  gain** (values + JSON are identical). Recommended: leave MKF on the `MAS::`-labelled enum (it already
  IS the PEAS vocabulary); revisit only if/when MKF drops converters and the MAS.hpp generation itself
  is changed. **Flagged as a decision** — say the word to force the MKF C++ repoint despite the risk.

## Dependency versions (checked 2026-06-29)

- **PEAS** — working copy & KH's recorded gitlink both at `02c49d1` = `origin/main` tip → **on latest
  PEAS main**. An unmerged `feat/peas-family-consolidation` (`e4672df`) is ahead of main.
- **CIAS** — working copy at `91cb391` = `origin/main` tip → **on latest CIAS main**, but KH's
  *committed* gitlink still points at older `d91068e` (shows as `M deps/CIAS`); the bump is
  **uncommitted** — `git add deps/CIAS` to make it durable. CIAS's remote default `HEAD` points at
  `feat/peas-family-consolidation` (`734adb7`), ahead of main.
- **Open question:** both repos have a `feat/peas-family-consolidation` branch ahead of `main`, and
  CIAS's *default* branch is that feature branch. Decide whether KH tracks `main` (stable) or the
  consolidation line (active). This may overlap with the enum→PEAS move above.

## Migration progress log (2026-06-30, overnight)

Done + on `main` (all tested):
- **Item 1 — MAS::Topology typed-enum adoption (Step 1)** ✅ `25e397d`. `src/Topology.hpp`;
  `finalize_control_seeds` takes the enum; 24 call sites converted; byte-identical JSON; suite 134/32.
- **Item 3 — ANALYTICAL run engine** ✅ `9370f3a`. `src/Analytical.{hpp,cpp}`; simulator-free
  operating-point prediction from the TAS doc; `test_analytical` 162 assertions / 15 topologies +
  analytical-vs-SPICE cross-check.
- **Item 4 — native in-process libngspice runner** ✅ `9370f3a`. `src/NgspiceRunner.{hpp,cpp}`;
  ENABLE_NGSPICE-gated; validated equal to the CLI.

Open decisions for the user (collected; not blocking):
1. **Item 2 multi-sim exporters** — build **NL5** now with structural-only validation (element +
   node-incidence histogram; cannot confirm it opens in NL5 — not installed), or wait for the tool +
   the auto-layout pass? PLECS/Simba are heavier still. Recommended: do the behavior-preserving
   `CiasCircuitWalk` refactor first regardless, then NL5 when a tool is available.
2. **Item 4 WASM ngspice (P5)** — proceed with the emscripten libngspice `ExternalProject` (needs
   `emsdk_env.sh` sourced, the fragile ngspice-45.2 patch script, unvalidatable here), or rely on the
   **ANALYTICAL engine** (item 3, already WASM-clean) for the in-browser path and defer WASM ngspice?
3. **Enum → PEAS (Step 2)** — schedule the governed MAS→PEAS taxonomy RFC; until then `Topology.hpp`
   stays `using Topology = MAS::Topology`.
4. **Selectable-engine API surface** — want a single `operating_point(tas, RunEngine)` dispatcher
   (ANALYTICAL → `analytical_operating_point`; SPICE → deck + `run_ngspice_in_process`/CLI), or keep
   the two entry points separate as now?
5. **deps/CIAS gitlink + `feat/peas-family-consolidation`** — track `main` or the consolidation line
   (carried over from above).
