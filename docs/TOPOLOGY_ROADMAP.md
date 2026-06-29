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

## Remaining gaps — prioritized

### Tier 1 — fundamental gaps (conspicuous absences)
- **Inverting buck-boost** — the textbook single-switch non-isolated buck-boost. We have its
  derivatives (Ćuk/SEPIC/Zeta/FSBB) but not the parent. Cheapest add; closes a textbook hole.
- **Symmetric (hard-switched) half-bridge** — split-cap two-switch isolated PWM converter.
- **Hard-switched full-bridge (PWM)** — the non-phase-shifted full bridge. We have PSFB
  (phase-shifted) and AHB (asymmetric) but not the plain PWM bridges those are variants of.

### Tier 2 — high-value modern variants
- **Active-clamp flyback (ACF flyback)** — distinct from the active-clamp *forward* we have;
  the dominant USB-PD / GaN adapter topology. Reuses the ACF clamp logic.
- **Totem-pole (bridgeless) PFC** — modern GaN PFC; successor to the boost PFC, companion to Vienna.
- **Full-bridge LLC** (wide-range) — complement to the half-bridge LLC.
- **DCM / QRM (valley-switching) flyback modes** — Flyback is CCM-only today (P4 in the plan).

### Tier 3 — secondary-side & interleaving variants (cheap, practical)
- **Current-doubler rectifier** option for PSFB / full-bridge (standard at low-V / high-I).
- **Interleaved boost / interleaved buck (multiphase)** — VRM-relevant; mostly a multiplicity
  wrapper over existing stages.
- **Flying-capacitor multilevel (FCML) buck/boost** — non-isolated multilevel (we have 3-level
  only on the isolated/AC side: PSHB, Vienna).

### Tier 4 — scope expansion (deliberate yes/no, not drift)
- **DC-AC inverters** — 1-φ full-bridge, 3-φ 2-level VSI, NPC inverter. Everything today is
  DC-DC or AC-DC (rectifier); the whole inversion quadrant is empty. Large effort; decide
  explicitly whether "any power-converter topology" includes inverters.

### Suggested next three
1. Inverting buck-boost (trivial; closes the textbook hole).
2. Hard-switched half-bridge + full-bridge (unlocks the PWM-bridge family the phase-shift variants imply).
3. Active-clamp flyback (highest real-world relevance).
