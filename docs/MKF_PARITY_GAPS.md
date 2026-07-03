# MKF converter_models → Kirchhoff variant-parity audit

Scope: every VARIANT/MODE the historical MKF `converter_models` supported (rectifier types,
modulation modes, region/configuration enums, power-flow direction, CCM/DCM/QRM/BMO,
multi-output, LISN/attenuation sims), and whether Kirchhoff covers it in **both** the
analytical model and the ngspice deck.

Method: the MKF tree at the commit before deletion (`MKF 3e0261fd^:src/converter_models/`)
was extracted and diffed against KH `src/*.cpp` + `ConverterAnalytical.cpp` + `docs/SPEC.md`,
one audit per topology group. **Topology coverage is complete** (all 28 MKF models exist in
KH); this document tracks the remaining **variant** gaps.

Status legend: ✅ parity/superset · ⚠️ partial · ❌ missing. "both" = missing in analytical AND ngspice.

---

## Executive summary — priority order

**P0 — wrong output today (structural collapse):**
- **FSBB** emits a single always-on SIMULTANEOUS 4-switch scheme for every operating point; MKF
  classifies BUCK / BOOST / BUCK_BOOST regions and defaults to SPLIT_PWM (LM5176/LT8390). Buck- and
  boost-dominant points get the wrong gate drive; the ported `BUCK_BOOST_AUTO` analytical branch is dead code.

**P1 — real variants/modes MKF had, KH lacks:**
- **Multi-output** (N secondaries): Flyback, Forward, TwoSwitchForward, ACF, PushPull, IsolatedBuck,
  IsolatedBuckBoost, DAB, LLC, SRC — all single-output in KH. Analytical kernels for Forward/TSF/ACF/SRC
  already loop N; PushPull is an analytical *regression* (kernel is single-output).
- **Weinberg `variant = BRIDGE`** (4-switch H-bridge primary) + **synchronousRectifier** — both missing.
- **AHB `AHB_FLYBACK` rectifier** — MKF's 4th AHB variant (active-clamp flyback), full analytical + deck; KH absent.
- **CLLC / CLLLC bidirectional** (reverse power flow) — their raison d'être; KH forward-only both engines.
- **Sepic/Cuk/Zeta coupled-inductor** (mutual K, zero-input-ripple) — KH emits independent L1/L2.
- **Isolated Ćuk (V3)** + **bidirectional Ćuk (V5)** — KH has only the non-isolated diode Ćuk.
- **Flyback QRM (valley) + BMO (boundary)** switching-frequency modes — no code path in KH.
- **PFC non-boost variants** (totem-pole, interleaved, SEPIC/Ćuk PFC) + **non-CCM modes** (DCM/CrM/Transition).
- **LLC/SRC full-bridge primary**; **LLC above/below-resonance** operation (KH pins drive to `fr`).
- **Vienna `phaseCount>1`** interleaving + `peakOfLinePlusSectors` sampling.

**P1-done (this sweep):**
- ✅ DAB SPS/EPS/DPS/TPS modulation (analytical + deck).
- ✅ Sepic + Zeta synchronous rectifier.

**P2 — sizing/diagnostic parity (no waveform/topology impact):**
- `maximumDutyCycle` configurable gate (most solvers hardcode 0.95; Boost doesn't even guard < 1).
- `maximumSwitchCurrent` alternative inductance sizing (Buck/Boost/Zeta/FSBB/forward-family/isolated).
- Multiple operating points (KH consumes `operatingPoints[0]` only; worst-corner still captured separately).
- Advanced pinned-tank entry for SRC/CLLC/CLLLC (Lr/Cr direct); DAB `useLeakageInductance` + pinned Lr.
- CMC/DMC "sim → built MAS OperatingPoint" wrappers (raw waveform arrays are already returned).

**NOT gaps (MKF never implemented — do NOT "fix"):**
- `VOLTAGE_DOUBLER` rectifier for AHB / PSFB / PSHB / SRC — not in `AhbRectifierType`/`BRectifierType`/
  `SrcRectifierType`. Only **LLC** has it (`LlcRectifierType`), and KH covers it. KH's `voltageDoubler`
  throws in AHB/PSFB/PSHB/SRC are correct-by-parity.
- `CENTER_TAPPED_DIODE` outside SRC; rectifier-type enums for CLLC/CLLLC (DAB-architecture, none exist).
- SRC below-resonance analytical throw (MKF throws too); LLC/SRC reverse/precharge/synchronous (absent in MKF).
- Components (CMC/DMC/CT): **at parity** — all windings/configs/spec-modes/waveform-labels ported, LISN +
  attenuation sims ported. **SPEC.md §6 "not yet ported" notes are STALE.**

---

## Per-topology detail

### Non-isolated DC-DC
| Topology | Gap | Where | MKF ref | Prio |
|---|---|---|---|---|
| FSBB | BUCK/BOOST/BUCK_BOOST region classification + SIMULTANEOUS/**SPLIT_PWM** | both | FSBB.cpp:20-347,799-889; FSBB.h:154,276 | P0 |
| FSBB | bidirectional, phaseCount (interleaved), outputVoltageRippleRatio→Co | both | FSBB.h:326,334; FSBB.cpp:239,374,493 | P1/P2 |
| Sepic | coupled-inductor (mutual K) | both | Sepic.cpp:301-315,590-600 | P1 |
| Sepic | synchronous rectifier | — | Sepic.cpp:494,567-578 | ✅ done (00b872f) |
| Cuk | isolated (V3: transformer+turnsRatio), bidirectional (V5), coupled-inductor (V2), synchronous (V4) | both | Cuk.cpp:182,199-320,549,815-1015 | P1 |
| Zeta | coupled-inductor (mutual K) | both | Zeta.cpp:303-313,603-612 | P1 |
| Zeta | synchronous rectifier | — | Zeta.cpp:503,570-591 | ✅ done (00b872f) |
| Buck/Boost | maximumSwitchCurrent sizing, configurable maxDuty (Boost unguarded < 1), multi-OP | mixed | Buck.cpp:176; Boost.cpp:179 | P2 |

### Flyback family
| Topology | Gap | Where | MKF ref | Prio |
|---|---|---|---|---|
| Flyback | multi-output (N secondaries) | both | Flyback.cpp:288-342,464-506 | P1 |
| Flyback | QRM (valley) + BMO (boundary) Fs modes | both | Flyback.cpp:30,75-116 | P1 |
| Flyback | DCM duty in emitted deck (deck is CCM-only; analytical is DCM-aware) | ngspice | Flyback.cpp:386-394 | P1 |
| Flyback | maximumDrainSourceVoltage turns-ratio branch; AdvancedFlyback per-OP duty/deadtime | analytical | Flyback.cpp:1111-1131,1313-1374 | P2 |
| IsolatedBuck / IsolatedBuckBoost | >1 isolated secondary; maximumSwitchCurrent; maxDuty gate | both | IsolatedBuck.cpp:165-253; IsolatedBuckBoost.cpp:119-149 | P1/P2 |

### Forward family
| Topology | Gap | Where | MKF ref | Prio |
|---|---|---|---|---|
| Weinberg | **variant = BRIDGE** (4-switch H-bridge primary) | both | Weinberg.cpp:212-214,657-830,982-984 | P1 |
| Weinberg | synchronousRectifier (SR MOSFETs vs D_pos/D_neg) | both | Weinberg.cpp:725,898-912 | P1 |
| PushPull | multi-output aux secondaries (**analytical regression** — MKF had it) | both | PushPull.cpp:108-116,368-448,760-857 | P1 |
| Forward/TSF/ACF | multi-output (analytical kernels loop N; drivers single-output) | ngspice+driver | SingleSwitchForward.cpp:196-205; TwoSwitchForward.cpp:246; ActiveClampForward.cpp:355 | P1 |
| Forward-family | maximumSwitchCurrent inductance floor; multi-OP | analytical | per-file process_design_requirements | P2 |

### Bridge / phase-shift
| Topology | Gap | Where | MKF ref | Prio |
|---|---|---|---|---|
| DAB | SPS/EPS/DPS/TPS modulation | — | Dab.cpp:250-268,948-1110 | ✅ done (abab8d4) |
| DAB | bidirectional (signed D3), multi-output, useLeakageInductance, pinned Lr | mixed | Dab.cpp:728-731,858-922,1555-1569 | P1/P2 |
| AHB | **AHB_FLYBACK rectifier** (active-clamp flyback) | both | AsymmetricHalfBridge.cpp:232-256,1765-1870 | P1 |
| PSFB | (all MKF rectifier types covered) | — | — | ✅ |
| PSHB | CURRENT_DOUBLER in the analytical solver (deck OK; analytical uses inline approx) | analytical | PhaseShiftedHalfBridge.cpp:365,570-583 | P2 |

### Resonant
| Topology | Gap | Where | MKF ref | Prio |
|---|---|---|---|---|
| LLC | FULL_BRIDGE primary; multi-output; above/below-resonance operation | both | Llc.cpp:26,1198,1474 | P1 |
| LLC | integratedResonantInductor; BEHAVIORAL_PULSE fast sim | ngspice | Llc.cpp:1437,1467 | P2 |
| SRC | FULL_BRIDGE + FULL_BRIDGE_PHASE_SHIFT bridge type; multi-output; pinned Lr/Cr | both | Src.cpp:52-54,495,771-786 | P1/P2 |
| CLLC | **REVERSE / bidirectional**; pinned full tank; half-bridge analytical | both | Cllc.cpp:1143-1152,1852-1919 | P1 |
| CLLLC | **REVERSE / bidirectional**; region classification; half-bridge; Cr precharge | both | Clllc.cpp:632-668,1063-1135 | P1 |

### AC input
| Topology | Gap | Where | MKF ref | Prio |
|---|---|---|---|---|
| PFC | TOTEM_POLE, INTERLEAVED_BOOST, SEPIC-PFC, CUK-PFC variants | both | PowerFactorCorrection.cpp:433,436,239,1434 | P1 |
| PFC | DCM / CrM / TRANSITION mode inductance sizing | both | PowerFactorCorrection.cpp:441-452 | P1 |
| PFC | average-current-mode (UC3854) switching controller (KH uses hysteretic) | ngspice | generate_ngspice_switching_circuit:1067 | P2 |
| Vienna | phaseCount>1 interleaving; peakOfLinePlusSectors (6-sector DPWM) | analytical | Vienna.cpp:420,488-522,646 | P1/P2 |
| Vienna | Vienna-II/switchType/SR switch-current closed forms | analytical (diagnostic) | Vienna.cpp:145-207 | P2 |

### Component designers — ✅ at parity
CMC (2/3/4 windings, 4 spec modes, LISN sim), DMC (4 configurations, both spec modes, attenuation-verify
sim, propose LC), CT (3 waveform labels) — all ported (several MKF bugs fixed along the way). Only the
"sim → built MAS OperatingPoint" convenience wrappers are unported (raw arrays are returned). **Action:
update the stale SPEC.md §6 "not yet ported" notes.**
