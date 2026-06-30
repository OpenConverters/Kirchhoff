#pragma once

// Kirchhoff::analytical — per-topology analytical operating-point solvers (Phase 2+ of the MKF
// analytical-converter-solver port). Each function is the ideal-coupling (magnetics-free) `process_
// operating_points` for one topology: given the operating point (Vin/Vout/Iout/fsw) and the magnetic
// scalars (inductance / turns ratio), it computes the per-winding current + voltage WAVEFORMS in closed
// form and runs them through the DSP foundation (AnalyticalDsp.hpp) to produce a MAS::OperatingPoint
// with full waveforms, harmonics, and processed stresses — a genuine simulator-free prediction (not the
// target-echo of Analytical.hpp). Ported faithfully from MKF's converter_models.

#include "MAS.hpp"

namespace Kirchhoff {
namespace analytical {

// Buck (synchronous/diode, CCM + DCM). Returns an OperatingPoint with one winding excitation (the output
// inductor): TRIANGULAR current (average = outputCurrent, ripple from the inductance) and RECTANGULAR
// voltage in CCM; the DCM branch recomputes tOn and emits the discontinuous waveforms. Ported from MKF
// Buck::process_operating_points_for_input_voltage. Throws if the required duty cycle >= 1.
MAS::OperatingPoint analytical_buck(double inputVoltage, double outputVoltage, double outputCurrent,
                                    double switchingFrequency, double inductance,
                                    double diodeVoltageDrop = 0.0, double efficiency = 1.0);

} // namespace analytical
} // namespace Kirchhoff
