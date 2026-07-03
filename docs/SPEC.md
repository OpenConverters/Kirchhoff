# Kirchhoff spec contract — every wizard payload the engine accepts

This is the authoritative reference for building the JSON `spec` that Kirchhoff's string API consumes,
so a frontend / shim can translate a wizard's `buildParams()` output **without reading the C++**. It
covers the 24 switching topologies (`process_converter` / `design_tas` / `design_magnetic_inputs`) and
the 3 magnetic-component designers (CMC / DMC / current transformer).

Canonical reference implementation of a converter spec: **`web/src/topologies.js::buildSpec`** — mirror
it. The per-topology readers live in `src/<Topo>.cpp::design_<topo>()`; the component designers in
`src/{Cmc,Dmc,CurrentTransformer}.cpp`.

---

## 1. Two payload shapes

There are exactly two:

| Shape | Entry points | Used by |
|---|---|---|
| **Converter spec** (`{designRequirements, operatingPoints, config?}`) | `design_tas(topo, spec)`, `process_converter(topo, spec, engine)`, `design_magnetic_inputs(topo, spec)` | the 24 switching topologies |
| **Component spec** (flat) | `design_cmc(spec)`, `design_dmc(spec)`, `propose_dmc_design(spec)`, `design_current_transformer(spec)`, `design_magnetic_inputs("cmc"/"dmc"/"current_transformer", spec)` | CMC, DMC, current transformer |

**Units are SI everywhere**: V, A, W, Hz, Ω, H, F, °C, and degrees for phase. No exceptions.

### Error contract
Every `api::` entry returns a JSON string. On failure the string **starts with `"Exception: "`** followed
by the message (the callers check that prefix — no exception crosses the ABI). The PyOM/legacy shims wrap
this as `{"error": "..."}`. There is no structured `{field, message}` form yet (see §7).

### `dimensionWithTolerance` and resolution
A dimensional field is `{minimum?, nominal?, maximum?}` (any subset) **or** a bare number. The engine
collapses it with `resolve_dimensional_values(j, preferred)`:
- **NOMINAL** (default): `nominal` → else `(minimum+maximum)/2` → else `maximum` → else `minimum`; **throws** if none present.
- **MAXIMUM**: `maximum` → else `nominal` → else `minimum`.
- **MINIMUM**: `minimum` → else `nominal` → else `maximum`.

> **Corner loss warning:** every converter resolves `inputVoltage` at **both** MAXIMUM and MINIMUM to
> size the inductor and the device ratings. A **scalar** `inputVoltage` makes `min == nominal == max`, so
> the worst-case corners collapse and ratings are computed at the nominal only. Always send the
> `{minimum, nominal, maximum}` triplet for DC converters when you have the range.

---

## 2. The converter spec envelope

```json
{
  "designRequirements": {
    "efficiency": 0.9,                                  // optional, per-topology default (see tables)
    "inputType": "dc",                                  // "dc" | "acSinglePhase" | "acThreePhase"
    "inputVoltage": {"minimum": 36, "nominal": 48, "maximum": 60},   // dimensional (see corner warning)
    "switchingFrequency": {"nominal": 100000},          // dimensional; resolved NOMINAL
    "outputs": [
      {"name": "out", "voltage": {"nominal": 12}, "regulation": "voltage"}
    ],
    "isolationVoltage": 1500,                           // optional; Flyback only threads it to the magnetic
    "lineFrequency": {"nominal": 50}                    // required iff inputType != "dc"
  },
  "operatingPoints": [
    {"name": "full_load", "inputVoltage": 48, "ambientTemperature": 25,
     "outputs": [{"name": "out", "power": 24}]}
  ],
  "config": { "...": "optional design knobs, see §4" }
}
```

**Field resolution (all converters):**
- `designRequirements.outputs[i].voltage` — dimensional, resolved **NOMINAL**. Required.
- `designRequirements.switchingFrequency` — dimensional, resolved **NOMINAL**. Required.
- `designRequirements.inputVoltage` — dimensional, resolved **MAXIMUM + MINIMUM** (and NOMINAL as the
  operating-point fallback). Required.
- `designRequirements.efficiency` — scalar, optional, per-topology default.
- `operatingPoints[0].inputVoltage` — **raw scalar** (`.get<double>()`, no dimensional resolution).
- `operatingPoints[0].outputs[i].power` — **raw scalar**.
- `operatingPoints[0].ambientTemperature` — scalar, default `25.0` if absent. Threads through to the TAS
  operating points, **every magnetic's operating-point `conditions.ambientTemperature`** (what MKF's adviser
  designs the core against), and the captured `analyticalWaveforms` registry — so a non-25 °C ambient is
  honored, not just echoed.

**operatingPoints fallback:** when `operatingPoints` is absent/empty, the design value comes from
`designRequirements` instead: `inputVoltage → nominal(dr.inputVoltage)`, `power → nominal(dr.outputs[i].power)`.
So you may send `outputs[i].power` inside `designRequirements.outputs[i]` if you omit `operatingPoints`.
**Exceptions that HARD-REQUIRE `operatingPoints[0]` (no fallback):** `isolated_buck`, `isolated_buck_boost`,
`weinberg` — these dereference `operatingPoints[0]` unconditionally.

**Multi-output:** `isolated_buck` / `isolated_buck_boost` require exactly **2** outputs (primary rail +
isolated secondary). `forward`, `two_switch_forward`, `acf` support **N isolated secondaries** (ABT #86):
each `designRequirements.outputs[i]` gets its own duty-derived turns ratio `n_i = Vin_min·D_max/(Vout_i+Vd_i)`,
a secondary winding on the shared transformer (isolationSides ordinal per rail: `secondary`, `tertiary`, …),
a rectifier + output filter (`Lout_i`/`Cout_i`), and its own external output port (`vout`, `vout2`, …) whose
load the assembler synthesizes from `operatingPoints[0].outputs[i]`. The main rail (output 0) is single-output
byte-identical; extra rails carry their `Cout_i` inside the switching cell. (`two_switch_forward` adds a tagged
numerical clamp-node snubber only when >1 output, for the coupled-winding LC-ring convergence.) All other
converters use `outputs[0]` only.

---

## 3. "I know the design I want" mode (pinning)

The wizards' design-level toggle is supported, but via **MAS designRequirements keys**, not the legacy
`desired*` names. The shim must translate legacy `desiredInductance` / `desiredTurnsRatios` /
`desiredMagnetizingInductance` into these:

| Pin | Keys read (first present wins) | Resolution | Which topologies honor it |
|---|---|---|---|
| Magnetizing / main inductance | `designRequirements.magnetizingInductance` \| `desiredInductance` \| `inductance` | NOMINAL | buck, boost, fsbb, flyback, isolated_buck, isolated_buck_boost, forward, two_switch_forward, acf, push_pull, weinberg, ahb, psfb, pshb, dab, llc, src, cllc, clllc |
| Primary:secondary turns ratio | `designRequirements.turnsRatios[idx]` | NOMINAL | flyback(0), forward(**1**), two_switch_forward(0), acf(0), push_pull(**1**), weinberg(**1**), ahb(0), psfb(0), pshb(0), dab(0), isolated_buck(0), isolated_buck_boost(0), llc(0), src(0), cllc(0), clllc(0) |

Notes:
- **`inductance` / `magnetizingInductance` accept a number or `{nominal}`.** They pin the magnetizing
  inductance (LLC/CLLC/CLLLC additionally re-size the resonant tank `Lr = Lm/Ln`, `Cr = 1/((2πfr)²Lr)`;
  SRC keeps Lm out of resonance so pinning it does NOT re-size the tank).
- **turnsRatios index differs**: `forward`, `push_pull`, `weinberg` read index **1** (index 0 is the
  1:1 demag/second-primary winding); everyone else reads index **0**.
- **Not pinnable anywhere:** series/resonant inductance `Lr` (computed from Q/Ln or the phase shift),
  DAB/PSFB/PSHB `seriesInductance`, split/DC-blocking caps. `sepic`, `cuk`, `zeta`, `pfc`, `vienna`
  support **no** pinning at all (L is always computed).
- LLC-only explicit tank pins exist: `desiredResonantInductance`/`resonantInductance` +
  `desiredResonantCapacitance`/`resonantCapacitance` override Lr/Cr verbatim (applied last).

---

## 4. `config` — optional design knobs

`spec.config` is an object of overrides; omit any to take the principled default. All are numbers except
the two string knobs. Global (all topologies) knobs:

| key | default | meaning |
|---|---|---|
| `tranStopTime` | `0.004` (DC), `0.06` pfc, `0.04` vienna, `0.006` flyback | transient window [s] |
| `tranMaxTimeStep` | `5e-8` (DC), `2e-7` pfc, `5e-7` vienna | transient max step [s] |
| `vDerate` | `0.8` | device voltage derating (rating = stress / derate); per-class `vDerateMosfet`/`vDerateDiode`/`vDerateCapacitor` fall back to this |
| `rdsOnLossFraction` | `0.01` | switch Rds(on) loss budget as fraction of rated power |
| `rippleRatio` / `inductorRippleRatio` | `0.3`–`0.4` (per topology) | inductor ΔI/I |
| `outputRippleFraction` / `outputCapRipple` | `0.01` | output-cap ripple fraction |

Notable per-topology knobs (see the tables for the full list): `rectifier` (buck/boost: `"diode"` |
`"synchronous"`), `rectifierType` (ahb/psfb/pshb/llc/src: `"fullBridge"` | `"centerTapped"` |
`"currentDoubler"` | `"voltageDoubler"` — **SRC and voltageDoubler on ahb/psfb/pshb throw**; **ahb** also
accepts `"ahbFlyback"` — an active-clamp flyback: energy-storage transformer, single flyback diode, no
output inductor, transfer Vo·n = Vin·D, ABT #87),
`bridgeType` (llc/src: `"halfBridge"` | `"fullBridge"` — full = 4-MOSFET primary driving the tank at ±Vin,
bridge factor 1.0, vs the half-bridge's split-cap ±Vin/2, 0.5; **any other value throws**, ABT #91),
`deadTimeFraction`, `commandedDuty`, `operatingDutyCycle`, `maxDutyCycle`, `dabPhaseShiftDeg`,
`qualityFactor`, `inductanceRatio`, `gainHeadroom`, `driveAtSwitchingFrequency`.

---

## 5. Per-topology tables

Legend: **req** = required, **opt** = optional (default in parens). `outputs[0].voltage`,
`switchingFrequency`, `inputVoltage` are req everywhere and resolved as in §2 — omitted from the rows
below. Every converter also emits one `control_stage` and a `full_load` operating point; that is engine
output, not input.

### Non-isolated DC-DC

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **buck** | 0.9 | inductance | `rectifier`("diode"), `deadTimeFraction`(0.01), `rippleRatio`(0.4) | L sized at Vin_MAX; `rectifier:"synchronous"` adds a low-side FET; step-down only |
| **boost** | 0.9 | inductance | `rectifier`("diode"), `rippleRatio`(0.4) | **THROWS if dutyCycle ≤ 0** (needs Vout > Vin/η); L at Vin_MAX |
| **sepic** | 0.9 | — | `l1RippleRatio`(0.4), `l2RippleRatio`(0.30), `couplingCapRipple`(0.05), `rectifier`("diode"), `coupledInductor`(false), `couplingCoefficient`(0.999) | non-inverting up/down; no pinning |
| **cuk** | 0.9 | turnsRatios[0] (isolated) | `l1RippleRatio`(0.4), `l2RippleRatio`(0.30), `diodeSnubberCap`(1e-9), `snubberRes`(100), `rectifier`("diode"), `coupledInductor`(false), `couplingCoefficient`(0.999), `isolated`(false), `turnsRatio`(1.0) | **INVERTING** — emits `outputs[0].voltage.nominal` negative; send positive magnitude. `isolated:true` inserts a transformer across the coupling cap (see below) |
| **zeta** | 0.9 | — | `l1RippleRatio`(0.4), `l2RippleRatio`(0.30), `couplingCapRipple`(0.05), `rectifier`("diode"), `coupledInductor`(false), `couplingCoefficient`(0.999) | non-inverting up/down |

**Isolated Ćuk (`config.isolated`, ABT #90).** `isolated: true` inserts a 2-winding transformer across the
coupling capacitor: the single C1 becomes a **primary** coupling cap (C1) + transformer + **secondary**
coupling cap (C1b), and the output is referred through `turnsRatio` = n = Np/Ns (KH convention, so the
secondary is the primary /n; also honours a pinned `designRequirements.turnsRatios[0]`). This adds galvanic
isolation and a step-up/step-down beyond the D/(1-D) range — the design sizes the duty against the
primary-referred output `|Vo|·n` so the transformer restores `|Vo|` on the secondary. Works with
`rectifier: "synchronous"`. Mutually exclusive with `coupledInductor`.

**Bidirectional Ćuk (`config.powerFlowDirection="reverse"`, ABT #90 V5).** Reverse power flow: the −|Vo| rail
sources power and the Vin rail receives it (same key as the CLLC bidirectional). Requires
`rectifier: "synchronous"` so the rectifier branch can carry reverse current, and swaps the main/rectifier
roles of the two switches. The inverting single-switch cell means the open-loop operating point is
suboptimal (a closed loop trims the duty), so reverse delivers genuine power Vout→Vin with energy conserved
rather than a pinned magnitude. Not combinable with `isolated` yet (throws).

**SEPIC / Ćuk / Zeta coupled inductor (`config.coupledInductor`, ABT #89).** By default L1 and L2 are two
independent single-winding magnetics. With `coupledInductor: true` they share ONE core as a single
2-winding magnetic (1:1) with mutual coupling `couplingCoefficient` (default 0.999, must be in (0,1)) — the
classic zero-input-ripple design (TI SLYT411). The TAS emits it as one magnetic with `turnsRatios: [1]` and
a `leakageInductance` of `L1·(1-k²)`, so the ngspice path renders the pairwise coupling `K` exactly as it
does for a transformer. Coupling steers the ripple between windings without changing the DC transfer; the
inverting Ćuk uses the opposite winding-dot orientation so the coupling does not shift its operating point.
| **fsbb** (4-switch buck-boost) | 0.9 | inductance | `deadTimeFraction`(0.01), `inductorRippleRatio`(0.4), `outputCapacitance`(100e-6), `fsbbTransitionBand`(0.10), `transitionMode`(**splitPwm**), `fsbbSplitRatio`(0.5), `powerFlowDirection`(forward) | region-aware gate drive (buck@Vin>Vo / boost@Vo>Vin / buck-boost band); L = worst of buck@Vin_max / boost@Vin_min, unity fallback if Vin_min==Vin_max==Vo; `phaseCount>1` (interleaved) throws (not implemented) |

**FSBB transition sub-mode + bidirectional (ABT #94).** In the buck-boost transition band (`fsbbTransitionBand`,
±10% of Vo around Vin) the sub-mode selects the gate scheme. `transitionMode: "splitPwm"` (MKF default,
LM5176/LT8390) runs the buck and boost legs at DIFFERENT, phase-shifted duties: the boost leg charges the
inductor for `t1 = fsbbSplitRatio·D` (κ=0.5 default), then a mild `(Vin−Vo)` freewheel interval, then
discharge, with `t2 = Vo·(1−t1)/Vin` volt-second-balanced so Vout stays on target. Because the strong +Vin
charge is shortened, the inductor-current ripple is **strictly lower** than `"simultaneous"` (all four
switches commuting together) at the same L — measured ~2.2 A pk-pk vs ~4.4 A at 12→12 V/24 W/100 kHz.
`κ→1` collapses split-PWM back to simultaneous. `powerFlowDirection: "reverse"` makes the Vout rail source
power and delivers to Vin (the synchronous H-bridge conducts both ways — the two legs swap source/delivered
roles; the output filter cap moves to the delivered Vin rail); open-loop it asserts genuine reverse delivery +
energy conservation, not a pinned setpoint. Interleaved multi-phase (`phaseCount>1`) is not yet implemented
and is rejected loudly.

### Flyback & isolated buck

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **flyback** | **0.88** | inductance, turnsRatios[0] | `tranStopTime`(**0.006**) | single output; `isolationVoltage` threaded to the magnetic (reinforced insulation) iff > 0; Lm gapped (nominal+0.1 tol) |
| **isolated_buck** (Fly-Buck) | 1.0 | inductance, turnsRatios[0] | `inductorRippleRatio`(0.4) | **needs 2 outputs** (primary + isolated); **operatingPoints[0] mandatory**; no isolationVoltage threading |
| **isolated_buck_boost** | 1.0 | inductance, turnsRatios[0] | `inductorRippleRatio`(0.4) | **needs 2 outputs**; **operatingPoints[0] mandatory**; inverting primary rail (send magnitude) |

### Forward family

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **forward** (single-switch) | 0.9 | inductance, **turnsRatios[1+i]** | `maxDutyCycle`(0.5), `inductorRippleRatio`(0.4) | 3-winding (demag+secondary), isolationSides {primary,primary,secondary,…}; second magnetic = output inductor; **multi-output** (N secondaries, ABT #86) |
| **two_switch_forward** | 0.9 | inductance, turnsRatios[i] | `maxDutyCycle`(0.5), `inductorRippleRatio`(0.4) | 2-winding; each switch blocks only Vin_max; **multi-output** (N secondaries, ABT #86; adds a tagged clamp-node snubber when >1 output) |
| **acf** (active-clamp forward) | 0.9 | inductance, turnsRatios[i] | `operatingDutyCycle`(0.45), `deadTimeFraction`(0.01), `nodeSnubberCap`(2.2e-9) | synchronous rectifiers (MOSFETs, not diodes); active clamp resets core; **multi-output** (N secondaries, ABT #86) |
| **push_pull** | 0.9 | inductance, **turnsRatios[1]** | `maxDutyCycle`(0.48), `outputCapacitance`(100e-6) | center-tapped primary; secondary ratios emitted as **{maximum}** ceilings |
| **weinberg** | **1.0** | inductance, **turnsRatios[1]** | `variant`("classic"), `synchronousRectifier`(false), `boostDutyTarget`(0.55), `l1RippleRatio`(0.30), `transformerCoupling`(0.999), `bridgeTurnsScale`(0.5), `deadTimeFraction`(0.02) | current-fed push-pull; **operatingPoints[0] mandatory**; boost regime (D>0.5); `variant="bridge"` = 4-switch H-bridge primary (diagonal PWM, halves primary switch Vds, `bridgeTurnsScale` rescales the shared transformer); `synchronousRectifier=true` swaps the CT-FW diodes for SR MOSFETs + body diodes (ABT #88) |

### Bridge & phase-shift

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **ahb** (asymmetric half-bridge) | 0.9 | inductance, turnsRatios[0] | `operatingDutyCycle`(0.30), `deadTimeFraction`(0.01), `rectifierType`("fullBridge") | Lm emitted as **{minimum}** (ungapped); `voltageDoubler` throws |
| **psfb** (phase-shifted full bridge) | 0.9 | inductance, turnsRatios[0] | `commandedDuty`(0.7), `switchDutyFraction`(0.48), `rectifierType`("fullBridge") | Lr computed (not pinnable); Lm {minimum}; `voltageDoubler` throws |
| **pshb** (phase-shifted half bridge) | 0.9 | inductance, turnsRatios[0] | `commandedDuty`(0.7), `magnetizingCurrentFraction`(0.3), `rectifierType`("fullBridge") | 3-level NPC; ratings on **half** bus Vin/2; sets `config.nodeShuntCap=1e-9` internally |
| **dab** (dual active bridge) | 0.9 | inductance, turnsRatios[0] | `dabPhaseShiftDeg`(25.0), `switchDutyFraction`(0.499) | **SPS only** (see below); Lr from phase shift; 8 driven switches |

**DAB modulation — SPS / EPS / DPS / TPS.** The DAB accepts all three phase shifts (degrees):

| config key | symbol | meaning | range | default |
|---|---|---|---|---|
| `dabPhaseShiftDeg` | D3 | outer inter-bridge shift — drives power & its direction | `(0, 180)` (band 0–90) | 25 |
| `dabInnerPhaseShift1Deg` | D1 | primary intra-bridge shift | `[0, 90)` | 0 |
| `dabInnerPhaseShift2Deg` | D2 | secondary intra-bridge shift | `[0, 90)` | 0 (or D1 if `dabModulationType:"DPS"`) |
| `dabModulationType` | — | `SPS`\|`EPS`\|`DPS`\|`TPS` — optional hint; for `DPS` an unset D2 mirrors D1 | — | — |

Modulation is defined by which shifts are non-zero: **SPS** D1=D2=0, **EPS** one inner shift, **DPS** D1=D2,
**TPS** D1≠D2. **Sizing:** SPS uses the exact closed form `Lr = N·V1·V2·D3·(π−|D3|)/(2π²·Fs·P)`; any inner
shift switches to sizing Lr numerically from the general power model (same tank kernel the waveforms use) so
the design still delivers rated power at the chosen `(D1, D2, D3)`. Out-of-range shifts throw a clear
`design_dab: …` error. **Common mistake:** passing a per-unit shift ×180 (e.g. `0.3·180 = 54`) — send the
angle directly. **Sim caveat:** the analytical design + waveforms are correct at any power, but the *ideal*
ngspice deck for a non-SPS modulation only converges up to ~100 W at 400→24 (the zero-voltage plateaus
commutate the ideal rectifier stiffly); SPS converges at full power. This limits only the in-browser
"Simulated" view for high-power EPS/DPS/TPS, not the design.

### Resonant

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **llc** | 1.0 | inductance(+tank), turnsRatios[0], resonant Lr/Cr | `rectifierType`("centerTapped"), `bridgeType`("halfBridge"), `driveAtSwitchingFrequency`(false), `resonantBandMin`(80e3)/`Max`(200e3), `qualityFactor`(0.4), `inductanceRatio`(5.0) | tank at `fr=√(fmin·fmax)`; **stimulus runs at fr, not switchingFrequency, unless `driveAtSwitchingFrequency`** (ABT #91); `bridgeType="fullBridge"` = 4-MOSFET primary at ±Vin (factor 1.0); leakage emitted |
| **src** (series resonant) | 1.0 | turnsRatios[0], inductance(no tank re-size) | `gainHeadroom`(1.08), `qualityFactor`(0.8), `inductanceRatio`(10.0), `rectifierType`("centerTapped"), `bridgeType`("halfBridge") | `fr = switchingFrequency`; step-down only; `bridgeType="fullBridge"` = 4-MOSFET primary at ±Vin (factor 1.0, ABT #91); **voltageDoubler throws** |
| **cllc** | 1.0 | inductance(+tank), turnsRatios[0] | `gainHeadroom`(1.08), `qualityFactor`(0.3), `inductanceRatio`(4.45), `powerFlowDirection`("forward") | full-bridge both sides; active SR; precharges the delivered bus; **bidirectional** (see below) |
| **clllc** | 1.0 | inductance(+tank), turnsRatios[0] | `qualityFactor`(0.4), `inductanceRatio`(6.0), `senseResistance`(0.01), `powerFlowDirection`("forward") | CLLC + discrete secondary Lr; adds an SR-control stage; **bidirectional** (see below) |

**CLLC / CLLLC power-flow direction (`config.powerFlowDirection`, ABT #85).** These are dual-active-bridge
resonant converters whose defining feature is **bidirectional** power flow (V2G / on-board chargers).
`"forward"` (default) drives the Vin (HV) full bridge and rectifies on the Vout (LV) side — power flows
Vin→Vout. `"reverse"` makes the Vout side source power and the Vin side receive it — power flows Vout→Vin.
The tank/turns-ratio sizing is identical (symmetric bidirectional design) and both bridges are already
actively gated, so reverse is the same cell with the source/load swapped: the DC source drives the LV rail,
the HV rail carries the delivered load and is precharged, and the embedded transformer excitations are
reflected about the tank (driver = the LV winding). The reverse open-loop gain at `fr` differs from forward
(the reflected-load Q differs); a closed-loop regulator trims frequency to hit the target on the driven side.

**LLC / SRC full-bridge primary + LLC off-resonance (`config.bridgeType` / `driveAtSwitchingFrequency`, ABT
#91).** Both LLC and SRC default to a split-cap **half-bridge** primary (tank driven at ±Vin/2, bridge factor
0.5). `bridgeType="fullBridge"` emits a **4-MOSFET** primary (Q1..Q4, diagonal pairs (Q1,Q4)/(Q2,Q3) on the
two gate nets) that drives the tank at ±Vin (factor 1.0); the embedded FHA excitation and the turns-ratio
sizing both switch to 1.0, so for the same turns ratio the full bridge delivers ~2× the output, and with the
default (doubled) turns ratio it still meets spec. Separately, LLC pins its drive to the tank resonance `fr`
by default (M(fn=1)=1 → exactly spec, the KH improvement over MKF's off-resonance rectifier-test config);
`driveAtSwitchingFrequency=true` instead drives at the requested `switchingFrequency` and embeds the FHA gain
at that frequency, so above/below-resonance operating points are produced (below `fr` boosts, M>1; above
`fr` bucks, M<1). SRC already operates at series resonance (`fr = switchingFrequency`), so it has no such flag.

### PFC (AC input) — `inputVoltage` is LINE RMS, emitted `{nominal}` only

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **pfc** (boost PFC 1-φ) | 1.0 | — | `currentRippleFraction`(0.30), `outputCapacitance`(220e-6), `senseResistance`(0.1), `mode`("ccm"), `topologyVariant`("boost"), `numberOfPhases`(2, interleaved) | `inputType:"acSinglePhase"`; **`inputVoltage` = single-phase line RMS**, `lineFrequency` req; reads only `power` from operatingPoints; closed-loop (no open stimulus) |
| **vienna** (3-φ) | 1.0 | — | `busCapacitance`(470e-6), `balanceModulation`(4.0), `senseResistance`(0.1), `numberOfChannels`/`phaseCount`(1), `samplingStrategy`(`"fullLineCycle"`) | `inputType:"acThreePhase"`; **`inputVoltage` = per-phase line RMS**; `outputs[0].voltage` = full bus; switches block half bus, diodes full bus. `numberOfChannels`>1 interleaves each phase across N parallel channel inductors (analytical current split by 1/N; deck unchanged); `samplingStrategy` ∈ `fullLineCycle`\|`peakOfLineOnly`\|`peakOfLinePlusSectors` (last adds the six DPWM sector operating points) |

**PFC/Vienna spec differences from DC converters:** `inputVoltage`/`lineFrequency`/`switchingFrequency`
are emitted as `{nominal}` only (no min/max triplet); `operatingPoints[0]` supplies only
`outputs[0].power` (no `inputVoltage`).

**PFC conduction mode & topology variant (ABT #92):** `config.mode` ∈ {`ccm`(default), `dcm`, `crm`,
`transition`} drives the boost-inductor sizing (MKF `calculate_inductance_ccm`/`_dcm`/`_crcm`; `crm` and
`transition` share the boundary formula). The hysteretic current band follows the sized inductor's actual
peak ripple, so CCM is byte-identical to the original. `config.topologyVariant` ∈ {`boost`(default),
`interleaved`}: **interleaved** is N (=`numberOfPhases`, 2 or 3) phase-shifted boost legs sharing one bridge
and bus cap — each leg carries 1/N of the line current, so its inductor is N× larger and the per-phase
reference gain is 1/N (the plant gain K0 scales by N). **totemPole** is NOT yet supported (the analytical
bipolar inductor excitation IS ported — `analytical_pfc(bipolar=true)`: true bipolar sine, zero mean — but
the bridgeless bipolar closed-loop deck is not); `sepic`/`cuk` (buck-boost class) are NOT supported. Every
unsupported variant/mode THROWS a specific exception (no silent boost/CCM fallback). NOTE: PFC decks (both
boost and interleaved) use CIAS analog control blocks and do not fully validate under `tests/test_tas_schema.py`
(a `oneOf` stage-shape mismatch that predates this work and is identical between boost and interleaved) —
that harness covers only the DC converters.

---

## 6. Component designers (flat spec)

These are **not converters** — no TAS, no stages. Each returns a MAS `Inputs` (designRequirements +
operating point) for MKF's `MagneticAdviser`, plus a diagnostics sibling.

### 6.1 Common-mode choke — `design_cmc(spec)` → `{inputs, cmcDiagnostics}`

```json
{
  "operatingVoltage": 230,        // dimensional; mains voltage [V]
  "operatingCurrent": 6,          // line current [A]         (req, > 0)
  "lineFrequency": 50,            // [Hz]                     (req, > 0)
  "ambientTemperature": 25,       // [°C]                     (req)
  "lineImpedance": 50,            // LISN Ω/line              (opt, default 50)
  "numberOfWindings": 2,          // 2|3|4                    (opt, default 2)
  // ── exactly one spec mode ──
  "minimumImpedance": [{"frequency": 150000, "impedance": 1000}],     // OR
  "targetInsertionLoss": [{"frequency": 150000, "insertionLoss": 20}],// OR (dB → Z=Zline·(10^(IL/20)-1))
  "parasiticCap_pF": 100, "dvdt_V_ns": 5,                             // noise-estimation mode
  "safetyMargin_dB": 6, "regulatoryStandard": "CISPR 32 Class B",     // (noise mode; standard req'd valid)
  // ── OR advanced "I know the design" ──
  "desiredInductance": 0.002, "designFrequency": 150000
}
```
Required L = `max over points of Z/(2πf)`; advanced mode pins L (`{nominal}`) and excites at
`designFrequency`. `cmcDiagnostics = {computedInductance, dominantFrequency, dominantImpedance}`.
`regulatoryStandard` ∈ {`CISPR 32 Class A/B`, `FCC Part 15 Class A/B`} — **unknown value throws** (no
silent default). designRequirements: `application=interferenceSuppression`,
`subApplication=commonModeNoiseFiltering`, (N-1)×1:1 turns ratios, all-primary isolation, Lm `{minimum}`
(or `{nominal}` advanced), the impedance points. **topology field deliberately unset** (MKF parity).

### 6.2 Differential-mode choke — `design_dmc(spec)` → `{inputs, dmcDiagnostics}`; `propose_dmc_design(spec)` → LC sizing

```json
{
  "configuration": "singlePhase",   // singlePhase|singlePhaseBalanced|threePhase|threePhaseWithNeutral (→ 1/2/3/4 windings)
  "inputVoltage": 230,              // dimensional [V]        (req)
  "operatingCurrent": 8,            // [A]                    (req, > 0)
  "lineFrequency": 50,              // [Hz]                   (req, > 0)
  "switchingFrequency": 100000,     // ripple freq [Hz]       (opt)
  "ambientTemperature": 40,         // [°C]                   (req)
  "peakCurrent": 12,                // [A]                    (opt; else operatingCurrent×1.2 in the excitation)
  // ── exactly one inductance target ──
  "minimumImpedance": [{"frequency": 150000, "impedance": 800}],   // OR
  "minimumInductance": 0.0012,                                     // advanced / post-propose
  "filterCapacitance": 1e-6                                        // (propose only, opt)
}
```
`design_dmc` requires an inductance target (impedance **or** minimumInductance) — **throws otherwise**.
L_min is taken at the **lowest** impedance-spec frequency (MKF rule; differs from CMC's max-L rule).
`dmcDiagnostics = {computedInductance, computedMinFrequency, computedMaxFrequency, impedanceAtMinFrequency,
numberWindings}`. designRequirements: `subApplication=differentialModeNoiseFiltering`, **topology set**
to `differentialModeChoke` (MKF DMC did set it, unlike CMC).

**Help-mode flow:** `propose_dmc_design(spec)` (no inductance target) → `{inductance, capacitance,
cutoffFrequency, targetAttenuation_dB, peakCurrent, energyStorage_mJ, configuration, numberOfWindings}` →
re-call `design_dmc` with the returned `inductance` as `minimumInductance`. The ngspice
attenuation-verify **is** ported — `verify_dmc_attenuation` (`DmcSim.cpp`) runs the LC deck and compares
required vs theoretical vs measured attenuation (honestly reports `simulated:false` when libngspice is absent).

### 6.3 Current transformer — `design_current_transformer(spec)` → MAS `Inputs` (no diagnostics)

```json
{
  "waveformLabel": "sinusoidal",       // sinusoidal|unipolarRectangular|unipolarTriangular (others throw)
  "maximumPrimaryCurrentPeak": 20,     // [A]                (req)
  "frequency": 100000,                 // [Hz]               (req)
  "turnsRatio": 0.01,                  // Np/Ns; I_sec = I_pri × turnsRatio   (req, > 0)
  "burdenResistor": 10,                // [Ω]                (req)
  "ambientTemperature": 25,            // [°C]               (req)
  "secondaryDcResistance": 0.5,        // [Ω]                (opt, default 0)
  "dutyCycle": 0.5,                    // (opt, default 0.5)
  "diodeVoltageDrop": 0.0              // [V] (opt, default 0)
}
```
A real 2-winding transformer: designRequirements has `turnsRatios[0]={nominal: round(turnsRatio,2)}`, Lm
`{minimum: 1e-6}` floor, isolationSides {primary, secondary}, `topology=currentTransformer`. Secondary
current = primary × turnsRatio (ampere-turn balance), secondary voltage = I_sec·(R_burden + R_dc) + V_diode.

### 6.4 `design_magnetic_inputs(topology, spec)` — the topology-agnostic entry

Returns the **bare** main-magnetic `MAS::Inputs` (no diagnostics wrapper) for any topology. For the 24
switching topologies it is `design_tas` + `main_magnetic_inputs` in one call. It additionally accepts the
component aliases: `"common_mode_choke"`/`"cmc"`, `"differential_mode_choke"`/`"dmc"`,
`"current_transformer"` — routing to the component designers above (their bare `inputs`).

---

## 6.5 Per-node component waveforms — `component_waveforms(tas, fidelity)`

The per-*magnetic* waveforms come from the build registry (`process_converter().analyticalWaveforms`,
keyed by magnetic component name). For the **non-magnetic** parts (switches, diodes, caps, resistors) the
engine offers `component_waveforms(tasJson, fidelityJson)`: **one ngspice run** over the assembled TAS that
returns every such component's terminal current and headline voltage, resampled onto the same grid as the
winding waveforms. Input is a full **TAS** (the output of `design_tas`), not a spec; `fidelity` is a PEAS
`Fidelity` JSON (e.g. `{"origin":"requirements"}` for the ideal-switch deck).

### Output shape

```json
{
  "engine": "ngspice",
  "referencePeriod": 1e-5,                 // one switching period [s]; every waveform.time spans [0, referencePeriod)
  "components": [
    {
      "ref": "S1",                          // component name in the TAS brick
      "stage": "buckCell",                  // the stage (brick) it lives in
      "kind": "mosfet",                     // mosfet | diode | capacitor | resistor
      "current": {
        "waveform":  { "data": [ /* 128 pts */ ], "time": [ /* 128 pts, 0..period */ ] },
        "processed": { "label": "custom", "peak": 8.1, "rms": 4.4, "offset": 2.0, "peakToPeak": 9.0, "dutyCycle": 0.4 }
      },
      "voltage": {                          // present only when BOTH terminal nodes resolve to sim vectors
        "waveform":  { "data": [ ... ], "time": [ ... ] },
        "processed": { "label": "custom", "peak": 60.0, "rms": 38.0, "offset": 24.0, "peakToPeak": 60.0 },
        "label": "V_DS"                     // V_DS (mosfet) | V_AK (diode) | V (2-terminal passive)
      }
    }
  ]
}
```

- **`current`** is the device's own conduction branch (savecurrents): `i` through switches/caps/resistors,
  `id` through diodes. Both sides carry a `waveform` (128-pt resampled last period) **and** `processed`
  stats; `harmonics` are omitted to keep the payload small.
- **`voltage`** is the node-difference across the headline terminals: mosfet `drain − source` (`V_DS`),
  diode `anode − cathode` (`V_AK`), 2-terminal passive `pin1 − pin2` (`V`). It is **emitted only when both
  terminal nodes are resolvable sim vectors** — a component whose voltage node can't be found gets
  `current` but no `voltage` (honest omission, never a fabricated trace).
- A component whose current branch isn't in the deck (e.g. a stripped/real-model device the ideal deck
  doesn't simulate) is **omitted entirely** rather than emitted with zeros.

### Node naming (how a pin maps to an ngspice vector)

Each stage becomes a subcircuit instance `x<stage>` (lowercased, `sanitize`d — non-alphanumerics → `_`,
`+`→`p`, `-`→`n`). A brick net resolves to a top-level ngspice node as follows:

| Net exposed on… | ngspice node |
|---|---|
| a stage **port** wired into an inter-stage group | the group name (an `externalPort` group containing "gnd" collapses to `"0"`) |
| a stage **port** not grouped | `"<stage>__<port>"` |
| an **internal** brick net (no port) | `"x<stage>.<net>"` (hierarchical) |

Voltages are looked up by that node name (ground `"0"` is identically zero). This mirrors
`TasAssembler::node_for_stage_port` / `sanitize` exactly — the same convention `extract_operating_point`
uses to reconstruct winding node voltages, so **every** waveform in the app (winding, node, component)
shares one time grid and one node-naming rule.

---

## 7. Open items (frontend-facing)

- **Converter-node waveform overlays** (old `simulate_<topo>_ideal_waveforms.converterWaveforms[]`):
  the per-*magnetic* waveforms are available via `process_converter().analyticalWaveforms` and
  `topology_waveforms`; per-*node* switch/rectifier V/I are available from `component_waveforms(tas,
  fidelity)` (one ngspice run → every non-magnetic component's V/I). CMC/DMC LISN & ideal-waveform sims
  **are** ported (`CmcSim.cpp` `simulate_cmc_ideal_waveforms`/`simulate_cmc_lisn_waveforms`, `DmcSim.cpp`
  `simulate_dmc_waveforms`/`verify_dmc_attenuation`) and wired through the API; the only unported piece is
  wrapping the LISN/LC sim output as a full MAS `OperatingPoint` (the raw per-frequency waveform arrays are
  returned today).
- **Structured errors:** not yet — errors are the `"Exception: <message>"` string. A `{field, message}`
  form would let wizards highlight the offending input; schedule if needed.
```
