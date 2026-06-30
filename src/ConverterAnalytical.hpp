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

} // namespace analytical
} // namespace Kirchhoff
