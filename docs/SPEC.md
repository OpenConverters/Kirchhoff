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
- `operatingPoints[0].ambientTemperature` — scalar; emitted default `25.0` if absent.

**operatingPoints fallback:** when `operatingPoints` is absent/empty, the design value comes from
`designRequirements` instead: `inputVoltage → nominal(dr.inputVoltage)`, `power → nominal(dr.outputs[i].power)`.
So you may send `outputs[i].power` inside `designRequirements.outputs[i]` if you omit `operatingPoints`.
**Exceptions that HARD-REQUIRE `operatingPoints[0]` (no fallback):** `isolated_buck`, `isolated_buck_boost`,
`weinberg` — these dereference `operatingPoints[0]` unconditionally.

**Multi-output:** `isolated_buck` / `isolated_buck_boost` require exactly **2** outputs (primary rail +
isolated secondary). All other converters use `outputs[0]` only.

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
`"currentDoubler"` | `"voltageDoubler"` — **SRC and voltageDoubler on ahb/psfb/pshb throw**),
`deadTimeFraction`, `commandedDuty`, `operatingDutyCycle`, `maxDutyCycle`, `dabPhaseShiftDeg`,
`qualityFactor`, `inductanceRatio`, `gainHeadroom`.

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
| **sepic** | 0.9 | — | `l1RippleRatio`(0.4), `l2RippleRatio`(0.30), `couplingCapRipple`(0.05) | non-inverting up/down; no pinning |
| **cuk** | 0.9 | — | `l1RippleRatio`(0.4), `l2RippleRatio`(0.30), `diodeSnubberCap`(1e-9), `snubberRes`(100) | **INVERTING** — emits `outputs[0].voltage.nominal` negative; send positive magnitude |
| **zeta** | 0.9 | — | `l1RippleRatio`(0.4), `l2RippleRatio`(0.30), `couplingCapRipple`(0.05) | non-inverting up/down |
| **fsbb** (4-switch buck-boost) | 0.9 | inductance | `deadTimeFraction`(0.01), `inductorRippleRatio`(0.4), `outputCapacitance`(100e-6) | L = worst of buck@Vin_max / boost@Vin_min; unity fallback if Vin_min==Vin_max==Vo |

### Flyback & isolated buck

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **flyback** | **0.88** | inductance, turnsRatios[0] | `tranStopTime`(**0.006**) | single output; `isolationVoltage` threaded to the magnetic (reinforced insulation) iff > 0; Lm gapped (nominal+0.1 tol) |
| **isolated_buck** (Fly-Buck) | 1.0 | inductance, turnsRatios[0] | `inductorRippleRatio`(0.4) | **needs 2 outputs** (primary + isolated); **operatingPoints[0] mandatory**; no isolationVoltage threading |
| **isolated_buck_boost** | 1.0 | inductance, turnsRatios[0] | `inductorRippleRatio`(0.4) | **needs 2 outputs**; **operatingPoints[0] mandatory**; inverting primary rail (send magnitude) |

### Forward family

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **forward** (single-switch) | 0.9 | inductance, **turnsRatios[1]** | `maxDutyCycle`(0.5), `inductorRippleRatio`(0.4) | 3-winding (demag+secondary), isolationSides {primary,primary,secondary}; second magnetic = output inductor |
| **two_switch_forward** | 0.9 | inductance, turnsRatios[0] | `maxDutyCycle`(0.5), `inductorRippleRatio`(0.4) | 2-winding; each switch blocks only Vin_max |
| **acf** (active-clamp forward) | 0.9 | inductance, turnsRatios[0] | `operatingDutyCycle`(0.45), `deadTimeFraction`(0.01), `nodeSnubberCap`(2.2e-9) | synchronous rectifiers (MOSFETs, not diodes); active clamp resets core |
| **push_pull** | 0.9 | inductance, **turnsRatios[1]** | `maxDutyCycle`(0.48), `outputCapacitance`(100e-6) | center-tapped primary; secondary ratios emitted as **{maximum}** ceilings |
| **weinberg** | **1.0** | inductance, **turnsRatios[1]** | `boostDutyTarget`(0.55), `l1RippleRatio`(0.30), `transformerCoupling`(0.999) | current-fed push-pull; **operatingPoints[0] mandatory**; boost regime (D>0.5) |

### Bridge & phase-shift

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **ahb** (asymmetric half-bridge) | 0.9 | inductance, turnsRatios[0] | `operatingDutyCycle`(0.30), `deadTimeFraction`(0.01), `rectifierType`("fullBridge") | Lm emitted as **{minimum}** (ungapped); `voltageDoubler` throws |
| **psfb** (phase-shifted full bridge) | 0.9 | inductance, turnsRatios[0] | `commandedDuty`(0.7), `switchDutyFraction`(0.48), `rectifierType`("fullBridge") | Lr computed (not pinnable); Lm {minimum}; `voltageDoubler` throws |
| **pshb** (phase-shifted half bridge) | 0.9 | inductance, turnsRatios[0] | `commandedDuty`(0.7), `magnetizingCurrentFraction`(0.3), `rectifierType`("fullBridge") | 3-level NPC; ratings on **half** bus Vin/2; sets `config.nodeShuntCap=1e-9` internally |
| **dab** (dual active bridge) | 0.9 | inductance, turnsRatios[0] | `dabPhaseShiftDeg`(25.0), `switchDutyFraction`(0.499) | **SPS only** (see below); Lr from phase shift; 8 driven switches |

**DAB modulation (`dabPhaseShiftDeg`).** Kirchhoff implements **Single-Phase-Shift (SPS)** only: the inner
bridge shifts D1 = D2 = 0, and `dabPhaseShiftDeg` is the **outer inter-bridge shift D3, in degrees**. It
drives both the power transfer and the series-inductance sizing
`Lr = N·V1·V2·D3·(π − |D3|) / (2π²·Fs·P)` (with D3 in radians). Valid range is `(0, 180)` degrees — the
formula gives `Lr ≤ 0` outside it — and the controllable SPS band is ~**0–90°**; the default 25° matches
MKF's controllability-margin choice. **Common mistake:** passing a per-unit shift ×180 (e.g. `0.3·180 =
54`) — send the angle directly. Out-of-range values now throw a clear `design_dab: dabPhaseShiftDeg …`
error instead of producing a negative Lr. There is **no** `modulationType`/`innerPhaseShift`/`D1`/`D2`
config yet (EPS/DPS/TPS): the solver `analytical_dab` carries D1/D2 slots but they are wired to 0, so any
wizard EPS/DPS/TPS knobs are decorative until those are exposed.

### Resonant

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **llc** | 1.0 | inductance(+tank), turnsRatios[0], resonant Lr/Cr | `rectifierType`("centerTapped"), `resonantBandMin`(80e3)/`Max`(200e3), `qualityFactor`(0.4), `inductanceRatio`(5.0) | tank at `fr=√(fmin·fmax)`; **stimulus runs at fr, not switchingFrequency**; leakage emitted |
| **src** (series resonant) | 1.0 | turnsRatios[0], inductance(no tank re-size) | `gainHeadroom`(1.08), `qualityFactor`(0.8), `inductanceRatio`(10.0), `rectifierType`("centerTapped") | `fr = switchingFrequency`; step-down only; **voltageDoubler throws** |
| **cllc** | 1.0 | inductance(+tank), turnsRatios[0] | `gainHeadroom`(1.08), `qualityFactor`(0.3), `inductanceRatio`(4.45) | full-bridge both sides; active SR; precharges Vout |
| **clllc** | 1.0 | inductance(+tank), turnsRatios[0] | `qualityFactor`(0.4), `inductanceRatio`(6.0), `senseResistance`(0.01) | CLLC + discrete secondary Lr; adds an SR-control stage |

### PFC (AC input) — `inputVoltage` is LINE RMS, emitted `{nominal}` only

| topology | efficiency default | pinning | key config (default) | quirks |
|---|---|---|---|---|
| **pfc** (boost PFC 1-φ) | 1.0 | — | `currentRippleFraction`(0.30), `outputCapacitance`(220e-6), `senseResistance`(0.1) | `inputType:"acSinglePhase"`; **`inputVoltage` = single-phase line RMS**, `lineFrequency` req; reads only `power` from operatingPoints; closed-loop (no open stimulus) |
| **vienna** (3-φ) | 1.0 | — | `busCapacitance`(470e-6), `balanceModulation`(4.0), `senseResistance`(0.1) | `inputType:"acThreePhase"`; **`inputVoltage` = per-phase line RMS**; `outputs[0].voltage` = full bus; switches block half bus, diodes full bus |

**PFC/Vienna spec differences from DC converters:** `inputVoltage`/`lineFrequency`/`switchingFrequency`
are emitted as `{nominal}` only (no min/max triplet); `operatingPoints[0]` supplies only
`outputs[0].power` (no `inputVoltage`).

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
re-call `design_dmc` with the returned `inductance` as `minimumInductance`. (`propose` is analytical only —
the ngspice attenuation-verify is not yet ported.)

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
  are not yet ported (design path works; the EMI-spectrum view stays empty until then).
- **Structured errors:** not yet — errors are the `"Exception: <message>"` string. A `{field, message}`
  form would let wizards highlight the offending input; schedule if needed.
```
