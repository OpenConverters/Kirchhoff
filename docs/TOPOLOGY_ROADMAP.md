# Topology roadmap — implemented coverage & remaining gaps

Kirchhoff is the home for the OpenConverters/PSMA **converter models**: each topology's
design math + TAS assembly + runnable SPICE deck, exportable and usable independently of any
magnetics design. (Historically these models lived in OpenMagnetics/MKF; they are migrating
here as MKF returns to being a pure magnetics library — see `docs/MKF_MIGRATION.md`.)

## Implemented today (24 topologies)

| Family | Topologies | Verification |
|---|---|---|
| Non-isolated DC-DC | Buck, Boost, Ćuk, SEPIC, Zeta, FSBB (4-switch buck-boost) | MKF-equivalence |
| Isolated single/two-switch | Flyback, Forward, Two-switch forward, Active-clamp forward, Push-pull, AHB, Isolated-buck (Flybuck), Isolated-buck-boost, Weinberg | MKF-equivalence |
| Isolated bridge / phase-shift | PSFB, PSHB (3-level NPC), DAB (bidirectional) | MKF-equivalence |
| Resonant | LLC, SRC, CLLC, CLLLC | LLC/SRC/CLLC vs MKF; CLLLC standalone |
| AC-input PFC | PFC (1-φ boost), Vienna (3-φ 3-level) | standalone |

21 are gated by the MKF-equivalence suite; CLLLC/PFC/Vienna diverge from MKF by design
(AC input and/or closed-loop control expressed in CIAS) and are validated standalone.

## Variants already implemented (formerly listed as gaps)

- **Rectifier variants** for the isolated families — center-tapped / full-bridge / current-doubler /
  voltage-doubler (`Rectifier.hpp`), incl. current-doubler on PSFB/full-bridge (ABT #83).
- **Flyback conduction modes** — CCM / DCM / BCM / QRM valley switching (ABT #80).
- **LLC/SRC full-bridge primary** + above/below-resonance operation (ABT #91).
- **PFC variants** — totem-pole, interleaved, SEPIC, Ćuk; DCM / CrM / transition modes (ABT #92).
- **Interleaving** — FSBB (ABT #94), Vienna (ABT #93), PFC (ABT #92) via `config.phaseCount`.
- **AHB flyback** — the asymmetric-half-bridge flyback rectifier variant of AHB (ABT #87). Not the
  same circuit as the active-clamp flyback below (clamp cap in series with the primary, not across it).
- **CLLC/CLLLC bidirectional** reverse power flow (ABT #85).

## Remaining gaps — prioritized (each tracked in ABT)

### Tier 1 — fundamental gaps
- **Inverting buck-boost** (ABT #1433) — textbook parent of Ćuk/SEPIC/Zeta/FSBB. Cheapest add.
- **Symmetric (hard-switched) half-bridge PWM** (ABT #1434) — split-cap two-switch isolated converter.
- **Hard-switched full-bridge PWM** (ABT #1435) — the plain bridge PSFB and AHB are variants of.
- **Two-switch (double-ended) flyback** (ABT #1436) — clamp diodes return leakage energy to Vin.

### Tier 2 — high-value modern variants
- **Active-clamp flyback** (ABT #1432) — clamp switch + clamp cap across the primary (high-/low-side),
  ZVS, complementary / non-complementary modes. Dominant USB-PD / GaN adapter topology.
- **LCC resonant** (ABT #1438) — series-parallel tank, HV outputs where winding capacitance is Cp.

### Tier 3 — interleaving & multilevel
- **Interleaved (multiphase) buck / boost** (ABT #1437) — reuse `config.phaseCount`.
- **Flying-capacitor multilevel (FCML) buck/boost** (ABT #1439).

### Tier 4 — scope expansion (deliberate yes/no, not drift)
- **DC-AC inverters** (ABT #1440) — 1-φ full-bridge, 3-φ 2-level VSI, NPC. Decide explicitly whether
  "any power-converter topology" includes inverters.

### PSFB practical refinements (not new topologies)
- ZVS-aware Lr sizing + ZVS load boundary (ABT #1428), primary clamp diodes (ABT #1429),
  DC-blocking cap + flux-walk test (ABT #1430), current-transformer flux / reset / placement (ABT #1431).

### Suggested next three
1. Active-clamp flyback (highest real-world relevance).
2. Inverting buck-boost (trivial; closes the textbook hole).
3. Hard-switched half-bridge + full-bridge (the PWM-bridge family the phase-shift variants imply).
