#pragma once

// Kirchhoff::analytical — per-topology analytical operating-point solvers (Phase 2+ of the MKF
// analytical-converter-solver port). Each function is the ideal-coupling (magnetics-free) `process_
// operating_points` for one topology: given the operating point (Vin/Vout/Iout/fsw) and the magnetic
// scalars (inductance / turns ratio), it computes the per-winding current + voltage WAVEFORMS in closed
// form and runs them through the DSP foundation (AnalyticalDsp.hpp) to produce a MAS::OperatingPoint
// with full waveforms, harmonics, and processed stresses — a genuine simulator-free prediction (not the
// target-echo of Analytical.hpp). Ported faithfully from MKF's converter_models.

#include "MAS.hpp"

#include <limits>
#include <vector>

namespace Kirchhoff {
namespace analytical {

// Buck (synchronous/diode, CCM + DCM). Returns an OperatingPoint with one winding excitation (the output
// inductor): TRIANGULAR current (average = outputCurrent, ripple from the inductance) and RECTANGULAR
// voltage in CCM; the DCM branch recomputes tOn and emits the discontinuous waveforms. Ported from MKF
// Buck::process_operating_points_for_input_voltage. Throws if the required duty cycle >= 1.
MAS::OperatingPoint analytical_buck(double inputVoltage, double outputVoltage, double outputCurrent,
                                    double switchingFrequency, double inductance,
                                    double diodeVoltageDrop = 0.0, double efficiency = 1.0);

// Boost (CCM + DCM). One winding excitation (the input inductor): TRIANGULAR current whose AVERAGE is the
// input current Iout·(Vout+Vd)/Vin (not the load current), ripple from the inductance; RECTANGULAR
// voltage (Vin during on, Vin-Vout-Vd during off). Ported from MKF Boost::process_operating_points_for_
// input_voltage. Throws if the duty (1 - Vin·η/(Vout+Vd)) is <= 0 (Vin above Vout) or >= 1.
MAS::OperatingPoint analytical_boost(double inputVoltage, double outputVoltage, double outputCurrent,
                                     double switchingFrequency, double inductance,
                                     double diodeVoltageDrop = 0.0, double efficiency = 1.0);

// ── Phase 3: the PWM converter family ───────────────────────────────────────
// Each function below is the ideal-coupling `process_operating_points_for_input_voltage`
// for one topology, parameterized by plain scalars/vectors (no MKF Topology / Magnetic
// objects): the magnetizing/output inductance and turns ratio(s) that MKF reads from a
// Magnetic are passed as scalar params, and `switchingFrequency` is always a param (the
// mode-derived `resolve_switching_frequency` is NOT ported). Multi-winding topologies push
// one excitation per winding. All DSP comes from WaveformProcessor.

// Flyback (CCM + DCM via critical inductance, multi-secondary). Pushes Primary
// (FLYBACK_PRIMARY current) + one Secondary i (FLYBACK_SECONDARY current) per output. The
// per-winding mode (CCM RECTANGULAR vs DCM *_WITH_DEADTIME) follows the sign of the primary
// current offset; the secondary rectangular-voltage duty switches to the volt-second value in
// DCM. `currentRippleRatio` defaults to NaN → derived from L. Ported from MKF Flyback.cpp:123
// (process_operating_points_for_input_voltage). Throws if the duty cycle > 1.
MAS::OperatingPoint analytical_flyback(double inputVoltage,
                                       const std::vector<double>& outputVoltages,
                                       const std::vector<double>& outputCurrents,
                                       const std::vector<double>& turnsRatios,
                                       double switchingFrequency, double inductance,
                                       double diodeVoltageDrop = 0.0, double efficiency = 1.0,
                                       double currentRippleRatio = std::numeric_limits<double>::quiet_NaN());

// Single-switch forward (CCM + DCM). `turnsRatios` is [demag(=1), sec0, sec1, …]; the
// magnetizing inductance and the output-filter inductance (for the DCM boundary) are scalar
// params. Pushes Primary, Demagnetization winding, and one Secondary i per output. Ported from
// MKF SingleSwitchForward.cpp:43. Throws if t_on > T/2 (wrong topology configuration).
MAS::OperatingPoint analytical_forward(double inputVoltage,
                                       const std::vector<double>& outputVoltages,
                                       const std::vector<double>& outputCurrents,
                                       const std::vector<double>& turnsRatios,
                                       double switchingFrequency, double inductance,
                                       double mainOutputInductance, double currentRippleRatio,
                                       double diodeVoltageDrop = 0.0);

// Two-switch forward (CCM + DCM). `turnsRatios` is [sec0, sec1, …] (no demag winding). Pushes
// "First primary" + one Secondary i per output. Ported from MKF TwoSwitchForward.cpp:41. Throws
// if t_on > T/2.
MAS::OperatingPoint analytical_two_switch_forward(double inputVoltage,
                                                  const std::vector<double>& outputVoltages,
                                                  const std::vector<double>& outputCurrents,
                                                  const std::vector<double>& turnsRatios,
                                                  double switchingFrequency, double inductance,
                                                  double mainOutputInductance, double currentRippleRatio,
                                                  double diodeVoltageDrop = 0.0);

// Push-pull, single main output (CCM + DCM). `turnsRatio` is the main-secondary ratio
// (turnsRatios[1] in MKF); `inductance` is magnetizing, `outputInductance` the output filter L
// (DCM boundary). Pushes Primary Half 1, Primary Half 2, Secondary 0 Half 1, Secondary 0 Half 2
// (four center-tapped windings). Ported from MKF PushPull.cpp:71. Throws if t_on > T/2.
MAS::OperatingPoint analytical_push_pull(double inputVoltage, double outputVoltage,
                                         double outputCurrent, double switchingFrequency,
                                         double turnsRatio, double inductance,
                                         double outputInductance, double currentRippleRatio,
                                         double diodeVoltageDrop = 0.0);

// Weinberg (CT-forward + input coupled-inductor). `inductance` is the L1 coupled-inductor
// (magnetizing) value; `turnsRatio` the combined Pri:Sec ratio. Pushes Primary + Secondary
// (bipolar-rectangular voltages; the primary current is a regime-dependent CUSTOM ramp:
// buck/boundary pulse for D ≤ 0.5, 4-segment overlap shape for the boost regime D > 0.5).
// Ported from MKF Weinberg.cpp:182. Throws on singular gain / D over the maximum.
MAS::OperatingPoint analytical_weinberg(double inputVoltage, double outputVoltage,
                                        double outputCurrent, double switchingFrequency,
                                        double inductance, double turnsRatio,
                                        double diodeVoltageDrop = 0.0, double efficiency = 1.0);

// SEPIC (single L1 winding). One "Primary" excitation: TRIANGULAR L1 current around
// IL1avg = Iout·D/((1−D)·η), RECTANGULAR voltage (±Vin/−Vo, pp = Vin+Vo). Ported from MKF
// Sepic.cpp:97. Throws if the duty cycle exceeds the maximum.
MAS::OperatingPoint analytical_sepic(double inputVoltage, double outputVoltage,
                                     double outputCurrent, double switchingFrequency,
                                     double inductanceL1, double diodeVoltageDrop = 0.0,
                                     double efficiency = 1.0);

// Cuk (non-isolated, single L1 winding). Like SEPIC but the L1 voltage swing is VC1 = Vin/(1−D).
// Ported from MKF Cuk.cpp:174 (the V1/V2 non-isolated path). Throws if the duty exceeds the max.
MAS::OperatingPoint analytical_cuk(double inputVoltage, double outputVoltage,
                                   double outputCurrent, double switchingFrequency,
                                   double inductanceL1, double diodeVoltageDrop = 0.0,
                                   double efficiency = 1.0);

// Zeta (single L1 winding). Same primary excitation form as SEPIC. Ported from MKF Zeta.cpp:100.
// Throws if the duty exceeds the maximum.
MAS::OperatingPoint analytical_zeta(double inputVoltage, double outputVoltage,
                                    double outputCurrent, double switchingFrequency,
                                    double inductanceL1, double diodeVoltageDrop = 0.0,
                                    double efficiency = 1.0);

// Four-switch buck-boost (single-phase, non-bidirectional). One "Inductor" excitation. BUCK
// region (Vo < Vin): D = Vo/(Vin·η), iL_avg = Iout, pp_voltage = Vin. BOOST region (Vo > Vin):
// D = 1 − Vin·η/Vo, iL_avg = Iout/(1−D), pp_voltage = Vo. Ported from MKF FourSwitchBuckBoost.cpp
// (process_operating_point_for_excitation, BUCK/BOOST branches). The mixed BUCK_BOOST transition
// region is NOT ported (needs transition_mode + min on/off-time params). Throws if Vo == Vin
// (transition region) or the region duty is out of range.
MAS::OperatingPoint analytical_fsbb(double inputVoltage, double outputVoltage,
                                    double outputCurrent, double switchingFrequency,
                                    double inductance, double efficiency = 1.0);

// Isolated buck (fly-buck). Two outputs: the primary buck rail (Vpri, Ipri) and one isolated
// secondary (Vsec, Isec). `inductance` is the primary/magnetizing inductance, `turnsRatio` the
// Np/Ns ratio. Pushes Primary (piecewise CUSTOM current) + Secondary. Note: as in MKF the
// secondary *winding voltage* is set by the primary rail Vpri/n, so `secondaryOutputVoltage`
// only defines the second output's presence. Ported from MKF IsolatedBuck.cpp:41. Throws if the
// duty ≥ 1.
MAS::OperatingPoint analytical_isolated_buck(double inputVoltage, double primaryOutputVoltage,
                                             double primaryOutputCurrent, double secondaryOutputVoltage,
                                             double secondaryOutputCurrent, double switchingFrequency,
                                             double inductance, double turnsRatio,
                                             double diodeVoltageDrop = 0.0, double efficiency = 1.0);

// Isolated buck-boost (fly-buck-boost). Two outputs (primary rail + isolated secondary).
// `inductance` is primary/magnetizing, `turnsRatio` Np/Ns. Pushes Primary (TRIANGULAR current
// around (Ipri + Isec/n)/(1−D)) + Secondary (FLYBACK_PRIMARY current). Ported from MKF
// IsolatedBuckBoost.cpp:47. Throws if the duty ≥ 1.
MAS::OperatingPoint analytical_isolated_buck_boost(double inputVoltage, double primaryOutputVoltage,
                                                   double primaryOutputCurrent, double secondaryOutputVoltage,
                                                   double secondaryOutputCurrent, double switchingFrequency,
                                                   double inductance, double turnsRatio,
                                                   double diodeVoltageDrop = 0.0, double efficiency = 1.0);

// Active-clamp forward (CCM + DCM). `turnsRatios` is [sec0, sec1, …] (no separate demag winding — the
// active clamp resets the core during 1−D, so the primary sees +Vin during t1 and −Vclamp during t2,
// Vclamp = D/(1−D)·Vin → volt-second balanced). `inductance` is the magnetizing inductance,
// `mainOutputInductance` the main output-filter L (DCM boundary), `dutyCycle` the forward-switch
// max/operating duty (MKF get_maximum_duty_cycle, default 0.45) that shapes the secondary waveforms.
// Pushes "First primary" + one Secondary i per output. Ported from MKF ActiveClampForward.cpp:41. Throws
// if t1 > T/2 or the clamp voltage is undefined (duty == 1).
MAS::OperatingPoint analytical_active_clamp_forward(double inputVoltage,
                                                    const std::vector<double>& outputVoltages,
                                                    const std::vector<double>& outputCurrents,
                                                    const std::vector<double>& turnsRatios,
                                                    double switchingFrequency, double inductance,
                                                    double mainOutputInductance, double currentRippleRatio,
                                                    double dutyCycle = 0.45, double diodeVoltageDrop = 0.0);

// ── Phase 4: the phase-shifted bridge family ────────────────────────────────
// Each function reuses the shared header-only kernel OpenMagnetics::PwmBridgeSolver
// (duty-cycle loss + three-segment sub-interval breakdown) exactly as the MKF bridge
// solvers do. The rectifier is fixed to CENTER_TAPPED (the MKF default). The ZVS /
// ngspice *diagnostic* outputs MKF also produces do not change any winding waveform
// and are omitted; the duty-cycle-loss kernel result (which DOES shape the waveforms)
// is kept. The effective duty cycle is derived from `phaseShiftDegrees` (D_cmd =
// |phi|/180); a non-positive phase shift throws (no default duty is substituted).

// Phase-shifted full bridge (CCM, center-tapped). The primary swings ±Vin; pushes
// Primary + a center-tapped pair "Secondary i a"/"Secondary i b" per output (each
// half-winding carries the reflected output-inductor current on alternate half-cycles
// and reverse-blocks on the other). `seriesInductance` is the resonant/leakage Lr
// (must be > 0 — feeds the Sabate duty-cycle loss and the freewheel decay);
// `outputInductance` Lo sets the output-inductor ripple ΔILo = Vo·(1−Deff)/(Fs·Lo)
// (Lo ≤ 0 ⇒ zero ripple). Ported from MKF PhaseShiftedFullBridge.cpp:359. Throws on
// non-positive Vin / turns ratio / Fs / Lm / Lr / phase shift.
MAS::OperatingPoint analytical_psfb(double inputVoltage,
                                    const std::vector<double>& outputVoltages,
                                    const std::vector<double>& outputCurrents,
                                    const std::vector<double>& turnsRatios,
                                    double switchingFrequency, double magnetizingInductance,
                                    double seriesInductance, double outputInductance,
                                    double phaseShiftDegrees, double diodeVoltageDrop = 0.0);

// Phase-shifted half bridge (3-level NPC, CCM, center-tapped). Identical sub-interval
// model to the PSFB but the primary swings ±Vin/2 (BRIDGE_VOLTAGE_FACTOR = 0.5) and the
// freewheel decay is MOSFET-RON-limited rather than diode-limited. Same winding set
// (Primary + center-tapped pairs) and same parameters as `analytical_psfb`. Ported from
// MKF PhaseShiftedHalfBridge.cpp:318. Throws on the same non-positive conditions.
MAS::OperatingPoint analytical_pshb(double inputVoltage,
                                    const std::vector<double>& outputVoltages,
                                    const std::vector<double>& outputCurrents,
                                    const std::vector<double>& turnsRatios,
                                    double switchingFrequency, double magnetizingInductance,
                                    double seriesInductance, double outputInductance,
                                    double phaseShiftDegrees, double diodeVoltageDrop = 0.0);

// Asymmetric half bridge (CCM, center-tapped). Complementary-duty half bridge with a
// series DC-blocking cap Cb. Pushes Primary + center-tapped "Secondary 0a"/"Secondary 0b"
// (single output), or per-output load-share "Secondary k" (multi-output). Key AHB physics:
// the Cb forces mean(i_pri)=0, so the magnetizing current carries a DC bias
// mean(i_Lm)=(Io/n)(1−2D); the primary voltage is +(1−D)Vin during D·Tsw and −D·Vin during
// (1−D)·Tsw (volt-second balanced). `dutyCycle` D ∈ (0,1) is explicit; `currentRippleRatio`
// sizes the output inductor via compute_lo_min (the actual ΔILo is smaller, set by the
// AHB secondary-voltage geometry). Ported from MKF AsymmetricHalfBridge.cpp:460 (the
// CENTER_TAPPED standard path; AHB-Flyback/V5 and FB/CD rectifiers are NOT ported).
// Throws if D ∉ (0,1), any turns ratio ≤ 0, Vin/Vo/Fs/Lm ≤ 0, or vector sizes mismatch.
MAS::OperatingPoint analytical_asymmetric_half_bridge(double inputVoltage,
                                                      const std::vector<double>& outputVoltages,
                                                      const std::vector<double>& outputCurrents,
                                                      const std::vector<double>& turnsRatios,
                                                      double switchingFrequency, double magnetizingInductance,
                                                      double dutyCycle, double currentRippleRatio,
                                                      double diodeVoltageDrop = 0.0);

// Dual Active Bridge (DAB) — triple-phase-shift (SPS/EPS/DPS/TPS). Both sides are full bridges
// driving a series-inductor (Lr) tank through the transformer; bidirectional isolated power flow.
// Phase shifts in DEGREES: D1/D2 are the intra-bridge (primary/secondary leg) shifts, D3 the
// inter-bridge shift that sets the transferred power and its DIRECTION (sign of D3). `seriesInductance`
// is the tank Lr, `magnetizingInductance` Lm. Both bridge voltages are piecewise-constant, so the tank
// current iL(θ) and magnetizing current Im(θ) are exactly piecewise-linear, with iL(0)/Im(0) fixed by
// half-wave antisymmetry x(π)=−x(0). Pushes Primary (BIPOLAR_RECTANGULAR Vab + tank current iL+Im) +
// one Secondary i per output (load-share projection of the tank current — exact for a matched single
// output, approximate for multi-output, as in MKF). Ported from MKF Dab.cpp:701 with its closed-form
// 8-segment sub-interval propagator (Dab.cpp:156-349). Throws on non-positive Fs / Lm / series inductance.
MAS::OperatingPoint analytical_dab(double inputVoltage,
                                   const std::vector<double>& outputVoltages,
                                   const std::vector<double>& outputCurrents,
                                   const std::vector<double>& turnsRatios,
                                   double switchingFrequency, double magnetizingInductance,
                                   double seriesInductance,
                                   double innerPhaseShiftD1Degrees = 0.0,
                                   double innerPhaseShiftD2Degrees = 0.0,
                                   double outerPhaseShiftD3Degrees = 0.0);

// ── Phase 5: resonant converter family (FHA) ────────────────────────────────

// Secondary rectifier topology for the resonant converters.
enum class SrcRectifier { FULL_BRIDGE, CENTER_TAPPED };

// Series Resonant Converter (SRC) via First-Harmonic Approximation (FHA), ABOVE-RESONANCE only
// (Λ = fsw/fr ≥ 1; the capacitive/hard-switching region throws, matching MKF Phase 2). The series
// Lr-Cr tank is driven by the bridge square wave; FHA reduces it to a sinusoidal divider:
// Rac = (8/π²)·n²·Rload, Zin = √(Rac² + X²), X = ωLr − 1/(ωCr), ILr_pk = (4/π)·k_bridge·Vin / Zin,
// φ = atan2(X, Rac). `bridgeVoltageFactor` k_bridge = 1.0 (full bridge) or 0.5 (half bridge). Pushes
// Primary (sinusoidal tank current + square bridge voltage) + per output either one full-bridge
// Secondary i or two center-tapped "Secondary i Half {1,2}" half-windings. Ported from MKF
// Src.cpp:338. Throws on Λ < 1, non-positive fsw / Lr / Cr / turns ratio.
MAS::OperatingPoint analytical_src(double inputVoltage,
                                   const std::vector<double>& outputVoltages,
                                   const std::vector<double>& outputCurrents,
                                   const std::vector<double>& turnsRatios,
                                   double switchingFrequency,
                                   double resonantInductance, double resonantCapacitance,
                                   double bridgeVoltageFactor = 1.0,
                                   SrcRectifier rectifier = SrcRectifier::FULL_BRIDGE);

// LLC resonant converter via the Runo Nielsen TIME-DOMAIN analysis (TDA), NOT FHA. A half-bridge drives a
// series Lr-Cr tank in series with the transformer magnetizing inductance Lm; the tank state
// x = [I_Ls, I_Lm, V_Cr] evolves through three linear sub-states (A_POS / A_NEG secondary-conducting,
// B_FW freewheeling) whose event-driven closed-form segments compose one half cycle. Steady state is the
// half-wave antisymmetry x(Thalf) = −x(0), enforced by a multi-start damped-Newton solve (with LIP
// singularity perturbation + physical-bound sanity fallback) on the 3-vector residual. Pushes Primary
// (tank current I_Ls + topology-dependent voltage: VLm for a separate Lr, or Vi−VCr for an integrated Lr)
// + per output either two center-tapped "Secondary i Half {1,2}" half-windings or one full-bridge
// "Secondary i". Secondary current = (I_Ls − I_Lm)·share·n (the transferred diode current). Ported from
// MKF converter_models/Llc.cpp (LlcStateVector/propagate_substate/find_next_event/propagate_half_cycle/
// solve_steady_state/sample_segments :315-783, process_operating_point_for_input_voltage :786). MKF's
// ZVS/LIP diagnostics and the VOLTAGE_DOUBLER/CURRENT_DOUBLER rectifiers (no SrcRectifier equivalent) are
// omitted; the tank solver is transcribed exactly. `bridgeVoltageFactor` k_bridge = 0.5 (half bridge,
// the LLC convention) or 1.0 (full bridge). Throws on non-positive fsw / Lm / Ls / Cr / turns ratio.
MAS::OperatingPoint analytical_llc(double inputVoltage,
                                   const std::vector<double>& outputVoltages,
                                   const std::vector<double>& outputCurrents,
                                   const std::vector<double>& turnsRatios,
                                   double switchingFrequency,
                                   double magnetizingInductance,
                                   double seriesResonantInductance,
                                   double resonantCapacitance,
                                   double bridgeVoltageFactor = 0.5,
                                   SrcRectifier rectifier = SrcRectifier::CENTER_TAPPED,
                                   bool integratedResonantInductor = false);

// CLLC bidirectional resonant converter via the 4-state TIME-DOMAIN analysis (Sun et al. 2020 IEEE TPEL
// 35(4):3491–3505). Unlike the LLC (single Cr resonance) the CLLC resonates on BOTH sides: a primary
// series tank (Cr1+Lr1) and a secondary series tank (Lr2+Cr2) flank the magnetizing Lm, so the steady
// state needs a 4-vector x = [i_Lr1, i_Lm, v_Cr1, v_Cr2_pri] (both cap voltages are independent), not the
// LLC's 3-vector. The secondary tank is referred to the primary (Lr2_pri = Lr2·n², Cr2_pri = Cr2/n²); the
// ODE is a block-antidiagonal 4×4 linear system propagated in closed form via a 2×2 eigendecomposition,
// with the same event-driven P_POS/P_NEG/F sub-states as the LLC. Steady state is the half-wave
// antisymmetry x(Thalf) = −x(0), solved by a multi-start damped-Picard iteration (physically-motivated
// cap-voltage seeds, per-seed sanity bounds, best-residual kept). Pushes Primary (tank current i_Lr1 +
// magnetizing voltage VLm) + one full-bridge "Secondary 0" (I_sec = n·(i_Lr1 − i_Lm), V_sec = VLm/n) —
// exactly MKF's winding set. `rectifier` = CENTER_TAPPED instead splits the secondary into two polarity
// half-windings (family-consistent with analytical_llc/_src; MKF CLLC itself is single-winding full-wave).
// `bridgeVoltageFactor` k_bridge = 1.0 (full bridge, the CLLC default) or 0.5 (half bridge). Ported from
// MKF converter_models/Cllc.cpp: the cllc4_* machinery (:735-1081) + process_operating_point_for_input_
// voltage (:1087, forward branch). MKF's ZVS / mode-classification diagnostics, the extra-component (Cr/Lr)
// waveforms, the dead 3-state collapsed path (is_asymmetric is always true), and REVERSE power flow are
// omitted (KH has no power-flow-direction input; design_cllc is forward-only). Throws on non-positive fsw /
// Lm / turns ratio / any tank value, and on an infeasible conversion gain n·Vout/(k·Vin) ∉ [0.5, 3.0]
// (MKF's M_req guard) or a non-converging solve (mirroring MKF's own guards — no fabricated defaults).
MAS::OperatingPoint analytical_cllc(double inputVoltage,
                                    const std::vector<double>& outputVoltages,
                                    const std::vector<double>& outputCurrents,
                                    const std::vector<double>& turnsRatios,
                                    double switchingFrequency,
                                    double magnetizingInductance,
                                    double primaryResonantInductance,
                                    double primaryResonantCapacitance,
                                    double secondaryResonantInductance,
                                    double secondaryResonantCapacitance,
                                    double bridgeVoltageFactor = 1.0,
                                    SrcRectifier rectifier = SrcRectifier::FULL_BRIDGE);

// CLLLC bidirectional resonant converter via the 4-state RK4 AFFINE-PROPAGATOR TDA (a symmetric CLLC-with-
// two-caps: both the primary Cr1-Lr1 tank AND the secondary Lr2-Cr2 tank flank the magnetizing Lm). The
// steady state is the 4-vector x = [i_Lr1, i_Lr2, v_Cr1, v_Cr2] (i_Lm derived as i_Lr1 − i_Lr2/n by
// ampere-turn balance; Lr2/Cr2 kept as ACTUAL secondary-side values with n entering the coupled-inductance
// matrix explicitly). Unlike the LLC/CLLC event-driven closed-form solvers, MKF's CLLLC integrates the
// linear tank ODE with RK4: because the half-period propagator P is affine (P(x0) = M·x0 + g), it measures
// g and the four columns of M in five RK4 passes then solves the half-wave-antisymmetry (M+I)·x0 = −g by
// 4×4 Gaussian elimination — an exact (modulo RK4 truncation) steady state in six passes, no Newton/Picard.
// A 1 mΩ ESR on Lr1/Lr2 keeps (M+I) non-singular even for a lossless tank at exact resonance. Pushes
// Primary (tank current i_Lr1 + magnetizing voltage v_pri = Lm·di_Lm/dt) + one full-bridge "Secondary 0"
// (current = i_Lr2, voltage = v_pri/n) — exactly MKF's winding set. `rectifier` = CENTER_TAPPED instead
// splits the secondary current into two polarity half-windings (family-consistent with analytical_llc/
// _cllc/_src). `bridgeVoltageFactor` sets BOTH bridges (1.0 full bridge, the CLLLC default; 0.5 half
// bridge). Ported from MKF converter_models/Clllc.cpp: the Clllc* RK4 machinery (:356-494) +
// process_operating_point_for_input_voltage (:606, forward branch). MKF's ZVS / mode / current-sharing
// diagnostics, the extra-component (Cr/Lr) waveforms, and REVERSE power flow (KH has no power-flow-direction
// input; design_clllc is forward-only) are omitted. Throws on non-positive fsw / Lm / turns ratio / any tank
// value / HV or LV bus voltage (mirroring MKF's guards — no fabricated defaults).
MAS::OperatingPoint analytical_clllc(double inputVoltage,
                                     const std::vector<double>& outputVoltages,
                                     const std::vector<double>& outputCurrents,
                                     const std::vector<double>& turnsRatios,
                                     double switchingFrequency,
                                     double magnetizingInductance,
                                     double primaryResonantInductance,
                                     double primaryResonantCapacitance,
                                     double secondaryResonantInductance,
                                     double secondaryResonantCapacitance,
                                     double bridgeVoltageFactor = 1.0,
                                     SrcRectifier rectifier = SrcRectifier::FULL_BRIDGE);

// ── Phase 6: three-phase AC-input PFC (Vienna rectifier) ─────────────────────
// Vienna rectifier — a 3-phase, 3-level unidirectional boost PFC. The magnetic is the PER-PHASE boost
// inductor; each of the three legs boosts its phase (line-to-neutral) voltage onto the split ±Vdc/2 DC
// bus. This solver ports MKF Vienna::process_operating_point_for_input_voltage (converter_models/
// Vienna.cpp:556) plus the helpers it calls: compute_phase_peak_voltage (:105), compute_modulation_index
// (:111), compute_line_peak_current (:117), and build_line_cycle_waveform (:282). MKF's diagnostic-only
// members (last*/computed* switch/diode RMS/avg stresses via compute_switch_rms etc.) are OMITTED — they
// shape no winding excitation. MKF's phaseCount>1 interleaving and the peakOfLinePlusSectors strategy
// (a different helper, emit_switching_period_op_at_line_angle) are OMITTED to match KH's single-channel-
// per-phase ViennaDesign; N_ch is fixed at 1 → exactly THREE windings ("Phase A/B/C").
//
// Parameterization mirrors KH's ViennaDesign (src/Vienna.hpp): `linePhaseVoltageRms` is the per-phase
// line-to-NEUTRAL RMS (KH `inputVoltageRms`), so V_phase_peak = √2·linePhaseVoltageRms — identical to
// MKF's compute_phase_peak_voltage(V_LL) = √2·V_LL/√3 once V_LL = √3·linePhaseVoltageRms. `outputDcVoltage`
// is the FULL bus Vdc (split ±Vdc/2); `outputPower` is the total 3-phase output power (MKF's P = Vout·Iout).
//
// Two shaping modes, both faithful to MKF's branches:
//   fullLineCycle = true  (DEFAULT): each winding carries the full 50/60 Hz line cycle (MKF
//     build_line_cycle_waveform) — a bipolar full-sine current envelope of amplitude I_pk, shifted ±120°
//     per phase, with the per-angle switching-ripple triangle superimposed. complete_excitation runs at
//     the LINE frequency. This is the "rectified-sine envelope with HF ripple" KH's build_vienna_tas uses.
//   fullLineCycle = false (MKF peakOfLineOnly default): the switching-period snapshot at peak-of-line —
//     TRIANGULAR current about I_pk (ΔI_pp = V_phase_peak·(1−M)·Tsw/L) + RECTANGULAR ±(V_phase_peak /
//     V_phase_peak−Vdc/2) voltage, at the SWITCHING frequency.
//
// THROWS (mirroring MKF's guards; no fabricated defaults) on: non-positive linePhaseVoltageRms /
// outputDcVoltage / outputPower / switchingFrequency / boostInductance / lineFrequency; efficiency ∉ (0,1];
// powerFactor ∉ (0,1]; over-modulation M = V_phase_peak/(Vdc/2) > 1; and (fullLineCycle only) Fsw ≤ F_line.
MAS::OperatingPoint analytical_vienna(double linePhaseVoltageRms,
                                      double outputDcVoltage,
                                      double outputPower,
                                      double lineFrequency,
                                      double switchingFrequency,
                                      double boostInductance,
                                      double efficiency = 1.0,
                                      double powerFactor = 1.0,
                                      bool fullLineCycle = true);

} // namespace analytical
} // namespace Kirchhoff
