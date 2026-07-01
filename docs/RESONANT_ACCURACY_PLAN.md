# Resonant converter analytical-accuracy plan (CLLC, and LLC/CLLLC/SRC)

## Status quo

The resonant solvers (`analytical_src/llc/cllc/clllc` in `src/ConverterAnalytical.cpp`) are
**load-aware First-Harmonic Approximation (FHA)** — they replaced MKF's faithfully-ported but
**load-blind, lossless** Nielsen/4-state TDA (which emitted a tank current independent of `Iout` and
~5× the SPICE value; see commits `fdfd615`/`90a878e`/`5837cd4`). FHA is SPICE-validated by the
`[nrmse]` gates in `tests/test_nrmse_gate.cpp`:

| topology | primary NRMSE vs SPICE | secondary rms ratio | operating point |
|---|---|---|---|
| SRC | 0.15 | — | at resonance |
| LLC | **0.027** | 0.96 | ~unity gain (fr) |
| CLLLC | **0.024** | 0.997 | unity gain (n·Vout = Vin) |
| CLLC | **0.25** | 0.85 | **off-unity** (n·Vout = 370 ≠ Vin = 400, Lm/Lr = 4.45) |

## Root cause (established — literature + empirical)

FHA models the whole tank current as ONE sinusoid at fsw. It is **exact at resonance / unity gain**
and **degrades off-resonance and when magnetizing current is a large fraction** (low Lm/Lr). CLLC is
the only one whose *reference design* runs off-unity, so it is the only one that shows the error — but
**the error is latent in LLC/CLLLC too**: they are only tested at unity gain. Run LLC below resonance
(a normal high-gain LLC mode) and FHA drifts the same way.

Evidence it is a method limit, not a bug:
- CLLLC uses the **identical FHA formula** as CLLC and is exact (0.024) — the only difference is CLLLC's
  design sits at unity (n·Vout = Vin, Lm/Lr = 6).
- Literature is unanimous: FHA has "significant discrepancies" for CLLC off-resonance; accurate methods
  are **state-plane analysis** and **time-domain analysis (TDA)** / generalized mode analysis. A key
  FHA error source is the **secondary rectifier going DCM below resonance** (extra gain FHA can't see).
- A quick "improved FHA" attempt (i_load sinusoid + i_magnetizing triangle) made CLLC **worse** (0.266),
  and neither the total-fundamental (ILs) nor the load-branch (ILoad) amplitude is universally correct
  (ILs matches at unity, ILoad off-unity). No clean single-parameter fix exists.

## Would fixing CLLC also improve LLC/CLLLC?

**Yes** — a correct time-domain model is a *single unified engine* for all resonant families, accurate
across the full operating range:
- At their current unity-gain test points, LLC/CLLLC are already tight; a TDA keeps them tight.
- **Off-resonance (below/above fr), LLC/CLLLC currently have the same latent ~25% FHA error CLLC shows.**
  A correct TDA removes it there too. So the win is *robustness across operating points*, not just CLLC.
- It also lets us collapse four per-topology FHA solvers into one validated resonant engine.

## Two options

### Option A — Improved FHA (closed-form; ~1–2 days; target ~10% on CLLC)
Add the published FHA corrections: the **rectifier-DCM-below-resonance gain correction** and a proper
magnetizing decomposition, calibrated with SPICE in the loop. Lower risk, still an approximation, and
does NOT help the above-resonance CLLC point (which is CCM). Marginal.

### Option B — Correct time-domain resonant model (the real fix; ~1–2 weeks; accurate everywhere)
A proper resonant-tank steady-state solver. This is the same *family* as MKF's TDA but fixes its two
fatal flaws:
1. **Load coupling.** The old TDA modeled the secondary as a fixed ±Vo voltage clamp, so the tank
   current was independent of `Iout` (the load-blindness). Instead model the secondary as a **rectifier
   feeding the output cap + load**: the secondary conducts when forward-biased, delivers current to the
   output, and its cycle-average must equal `Iout`. This is the crux fix.
2. **Finite Q.** Add representative tank/switch ESR so the periodic steady state is well-posed at
   resonance (the lossless tank is singular there — the old TDA blew up; MKF's CLLLC already added
   1 mΩ ESR for exactly this reason but never coupled the load).
Plus **DCM detection** for below-resonance secondary conduction.

Implementation sketch:
1. State per family (SRC: iLr,vCr; LLC: iLr,iLm,vCr; CLLC/CLLLC: iLr1,iLr2,vCr1,vCr2), inductor ESR.
2. Secondary boundary = rectifier + Cout + Rload: diode-conduction events clamp the winding to ±Vo and
   source current to the output; magnetizing (iLm = iLr1 − iLr2/n) circulates and does NOT reach the load.
3. Piecewise linear/exponential propagation per sub-interval with **event detection** for diode on/off
   and switch transitions.
4. Half-wave-antisymmetric periodic steady state (x(π) = −x(0)) via the affine-propagator `(M+I)x0 = −g`
   (now non-singular from ESR, and load-coupled). Newton/iterate on Vo self-consistency if needed.
5. Per-winding waveforms → `WaveformProcessor` → `MAS::OperatingPoint` (unchanged downstream).
6. Keep the FHA solver as the fast seed / fallback.

## Validation (either option) — extend the gate before touching the solver
The `[nrmse]` gate infrastructure already drives analytical + ngspice from the same design and compares
the settled tank current. **Extend it to test EACH resonant topology at MULTIPLE operating points**
(below / at / above resonance × light / heavy load) — this exposes the latent off-resonance error in
LLC/CLLLC and is the acceptance test for the fix (target < 0.10 everywhere). The load-scaling regression
guard (`[analytical][scaling]`) is already in place.

## Option B — implementation progress + debugging findings (next-session starting points)

A first LLC time-domain propagator was prototyped (state [iLr, iLm, vCr], ESR = Zr/40 on Lr, RK4 with a
diode-mode machine: PWR_POS/PWR_NEG clamp the primary to ±Vo, FREEWHEEL locks iLr=iLm below resonance;
half-wave-antisymmetric steady state x(Thalf) = −x(0) via Newton with a numeric 3×3 Jacobian). It runs
but is **not yet correct** — two concrete problems to solve first:

1. **Singular steady-state solve AT fr.** With ESR = Zr/40 (Q≈40) the Newton still diverges at exactly fr
   (residual ~29, hits the iteration cap, iLr → −88 A). The resonance singularity is only *partly* tamed
   by that ESR. Fixes to try: (a) larger/again physically-motivated ESR (Q≈8–15 from real tank+switch
   loss), (b) a homotopy/continuation seed (solve just off fr, walk in), (c) solve in a basis that
   removes the antisymmetric null direction, (d) Levenberg–Marquardt damping instead of raw Newton.
2. **~4× over-prediction of the tank amplitude OFF resonance** (where the Newton DOES converge cleanly,
   resid ~1e-11): at 0.85·fr the model gives iLr ≈ 5.4 A vs SPICE 1.25 A, with vCr swinging ±556 V — the
   tank rings too high. This is a physical-model error, not a solver error: the ±Vo clamp is not
   extracting/damping enough energy (the delivered power / load coupling is under-represented), so the
   reactive amplitude builds up. Re-derive the powering-interval energy balance and confirm the drive
   (Vbus = k_bridge·Vin = 200 for the half-bridge — matches the working FHA) and the Vo clamp sign/timing
   against a single SPICE waveform overlay BEFORE trusting the grid. Overlay iLr, iLm, vCr and vprimary
   from SPICE vs the model at one operating point to localize the discrepancy.

The lesson holds: get ONE operating point matching SPICE by direct waveform overlay first, then the grid —
do not tune thresholds to hide a 4× error.

### Update — deeper findings (the naive TDA is a dead end; the correct architecture found; a fidelity floor)

Iterated the LLC model further with SPICE waveform overlay at 0.85·fr:

1. **The antisymmetric ±Vo-clamp TDA (MKF's family) is fundamentally broken and cannot be tweaked into
   correctness.** With an ideal ±Vo voltage clamp the tank has NO load damping: the reactive current
   builds unbounded (iLr ≈ 6 A vs SPICE 1.25 A; "delivered" 584 W vs 120 W load). An outer charge-balance
   loop on Vout *diverges* — the delivered current *increases* with Vout (wrong feedback sign). The clamp
   model is a dead end.
2. **The correct architecture is TIME-MARCHING the full 4-state circuit** [iLr, iLm, vCr, vCout] to steady
   state: the rectifier clamps the primary to ±n·vCout when a diode conducts, the rectified secondary
   |iD|·n charges the output cap Cout, and the load draws vCout/Rload. Marching ~80–400 cycles from rest,
   it CONVERGES STABLY (no singularity, no divergence) and gives the RIGHT amplitude/gain: with a
   reflected rectifier drop Vd≈1.4 V and tank Q≈8–12, Vout ≈ 12.2 V (SPICE 12.0) and iLr peak ≈ 1.33 A
   (SPICE 1.25). This fixes BOTH MKF failures — the resonance blow-up and the load-blindness.
3. **BUT off-resonance SHAPE fidelity plateaus at ~0.30 NRMSE — the same as FHA — and is NOT improvable by
   Q, Vd, or freewheel/mode logic.** At 0.85·fr the SPICE iLr carries harmonics from the real transformer
   leakage (K=0.999), the split resonant caps, and the actual diode conduction, which neither a pure-
   sinusoid model (FHA, 0.29) nor an idealized-clamp time-march (0.30) reproduces. This is a **reduced-
   order-model fidelity floor**, not a bug: reaching < 0.10 off-resonance requires modeling that circuit
   detail — i.e. converging toward the full SPICE netlist, which defeats the point of a fast analytical
   solver.

**Revised conclusion.** A fast reduced-order resonant model — whether FHA or a simplified time-domain sim —
has a ~0.15–0.30 shape floor OFF resonance vs the full SPICE circuit. FHA is *exact at/near resonance*
(the design point: LLC 0.027, CLLLC 0.024) and the time-march offers stability + correct amplitude but no
off-resonance shape gain over FHA. So the practical options are: (a) keep FHA (accurate where these
converters actually operate; off-resonance documented) — CURRENT STATE; (b) ship the time-march 4-state
model where amplitude/gain across the range matters more than exact shape (stable, load-correct, but
~2–3× slower and no better on shape); (c) for lab-grade off-resonance waveforms, use the ngspice path the
harness already runs. Getting a *fast* model under 0.10 everywhere is a genuine research problem
(state-plane / generalized-mode analysis with explicit harmonic + rectifier-DCM modeling), not a tuning
exercise. The validation harness (< 0.10 target, grid-wide) remains the correct acceptance test for any
future attempt.

## Recommendation
- If only CLLC-at-its-point matters → Option A.
- If KH's resonant models must be trustworthy across their **full operating range** (the right bar for an
  exportable converter-model library, and needed before retiring MKF's converter_models) → **Option B**.
  It is the same effort tier as the (broken) TDA it replaces, but done correctly and SPICE-gated at
  multiple operating points from the start.
