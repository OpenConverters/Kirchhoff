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
3. **Multi-simulator exporters.** `processors/CircuitSimulator{Ltspice,Plecs,Simba,Nl5}.cpp` +
   `CircuitSimulatorExporterHelpers.h`. KH's README already promises "LTspice/PSIM/Simba/NL5 to
   follow"; moving these into CIAS/KH delivers the "simulate in general, any simulator" goal.
4. **`NgspiceRunner` (shared-library/libngspice path) + results parser.** `processors/NgspiceRunner.*`.
   KH currently shells out via the deck's `.control` block; MKF's runner has the libngspice
   shared-lib API KH's P5 roadmap needs for in-browser WASM runs.
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
