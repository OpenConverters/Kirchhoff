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

## Recommendation
- If only CLLC-at-its-point matters → Option A.
- If KH's resonant models must be trustworthy across their **full operating range** (the right bar for an
  exportable converter-model library, and needed before retiring MKF's converter_models) → **Option B**.
  It is the same effort tier as the (broken) TDA it replaces, but done correctly and SPICE-gated at
  multiple operating points from the start.
