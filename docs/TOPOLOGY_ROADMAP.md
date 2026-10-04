# Topology roadmap — implemented coverage & remaining gaps

Kirchhoff is the home for the OpenConverters/PSMA **converter models**: each topology's
design math + TAS assembly + runnable SPICE deck, exportable and usable independently of any
magnetics design. (Historically these models lived in OpenMagnetics/MKF; they are migrating
here as MKF returns to being a pure magnetics library — see `docs/MKF_MIGRATION.md`.)

## Implemented today (29 topologies)

| Family | Topologies | Verification |
|---|---|---|
| Non-isolated DC-DC | Buck, Boost, Ćuk, SEPIC, Zeta, FSBB (4-switch buck-boost) | MKF-equivalence |
| Non-isolated DC-DC | Inverting buck-boost (CCM/DCM, diode/synchronous) | standalone (closed form vs ngspice) |
| Isolated single/two-switch | Flyback, Forward, Two-switch forward, Active-clamp forward, Push-pull, AHB, Isolated-buck (Flybuck), Isolated-buck-boost, Weinberg | MKF-equivalence |
| Isolated single/two-switch | Two-switch flyback (CCM/DCM/BCM; clamp diodes return the leakage energy to Vin) | standalone (vs the single-switch flyback deck) |
| Isolated single/two-switch | Active-clamp flyback (complementary / NC-ACF drive × high-/low-side clamp; ZVS) | standalone (steady-state solver vs ngspice) |
| Isolated bridge / phase-shift | PSFB, PSHB (3-level NPC), DAB (bidirectional) | MKF-equivalence |
| Isolated bridge (hard-switched) | Half bridge (split caps), Full bridge — symmetric PWM, CT/FB/CD rectifier | standalone (exact piecewise-linear solver vs ngspice) |
| Resonant | LLC, SRC, CLLC, CLLLC | LLC/SRC/CLLC vs MKF; CLLLC standalone |
| AC-input PFC | PFC (1-φ boost), Vienna (3-φ 3-level) | standalone |

21 are gated by the MKF-equivalence suite; CLLLC/PFC/Vienna diverge from MKF by design
(AC input and/or closed-loop control expressed in CIAS) and are validated standalone, as is every
topology MKF never modelled (the inverting buck-boost onwards).

## Variants already implemented (formerly listed as gaps)

- **Rectifier variants** for the isolated families — center-tapped / full-bridge / current-doubler /
  voltage-doubler (`Rectifier.hpp`), incl. current-doubler on PSFB/full-bridge (ABT #83).
- **Flyback conduction modes** — CCM / DCM / BCM / QRM valley switching (ABT #80).
- **LLC/SRC full-bridge primary** + above/below-resonance operation (ABT #91).
- **PFC variants** — totem-pole, interleaved, SEPIC, Ćuk; DCM / CrM / transition modes (ABT #92).
- **Interleaving** — FSBB (ABT #94), Vienna (ABT #93), PFC (ABT #92), and buck / boost (2–6 phases,
  ABT #1437) via `config.phaseCount`; buck/boost by the generic `interleave_stage` transform (`Interleave.hpp`).
- **AHB flyback** — the asymmetric-half-bridge flyback rectifier variant of AHB (ABT #87). Not the
  same circuit as the active-clamp flyback below (clamp cap in series with the primary, not across it).
- **CLLC/CLLLC bidirectional** reverse power flow (ABT #85).

## Remaining gaps — prioritized (each tracked in ABT)

### Tier 2 — high-value modern variants
- **LCC resonant** (ABT #1438) — series-parallel tank, HV outputs where winding capacitance is Cp.

### Tier 3 — multilevel
- **Flying-capacitor multilevel (FCML) buck/boost** (ABT #1439).

### Tier 4 — scope expansion (decided 2026-10-03: in scope)
- **DC-AC inverters** (ABT #1440) — 1-φ full-bridge SPWM + LC filter first, then 3-φ 2-level VSI, then NPC
  (PEAS topology values singlePhaseInverter / threePhaseInverter).
- **DAB variants** (ABT #1664) — modulation (SPS/EPS/DPS/TPS, already in the engine) as a selectable
  variant, dual-active half-bridge, three-phase DAB.

### PSFB practical refinements (not new topologies)
- ZVS-aware Lr sizing + ZVS load boundary (ABT #1428), primary clamp diodes (ABT #1429),
  DC-blocking cap + flux-walk test (ABT #1430), current-transformer flux / reset / placement (ABT #1431).

### Order being worked (2026-10-03)
Interleaved buck/boost → two-switch flyback → active-clamp flyback → hard-switched half-bridge →
hard-switched full-bridge (all done) → LCC → FCML → inverters (in scope, decided 2026-10-03). Tracked per ABT ticket.
