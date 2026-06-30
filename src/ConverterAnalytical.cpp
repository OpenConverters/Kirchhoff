#include "ConverterAnalytical.hpp"
#include "processors/WaveformProcessor.h"   // the shared DSP (MKF), reused — not re-implemented

#include <cmath>
#include <stdexcept>
#include <vector>

namespace Kirchhoff {
namespace analytical {

// The converter solvers compute only the topology PHYSICS (the per-winding waveform parameters); the
// waveform construction, harmonics, and processed stresses come from OpenMagnetics::WaveformProcessor.
using WP = OpenMagnetics::WaveformProcessor;

// Ported from MKF converter_models/Buck.cpp (calculate_duty_cycle + process_operating_points_for_input_voltage).
MAS::OperatingPoint analytical_buck(double inputVoltage, double outputVoltage, double outputCurrent,
                                    double switchingFrequency, double inductance,
                                    double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;

    double dutyCycle = (outputVoltage + diodeVoltageDrop) / ((inputVoltage + diodeVoltageDrop) * efficiency);
    if (dutyCycle >= 1.0)
        throw std::invalid_argument("analytical_buck: required duty cycle >= 1 (Vout too close to Vin)");

    const double period = 1.0 / switchingFrequency;
    double tOn = dutyCycle / switchingFrequency;
    double primaryCurrentPeakToPeak = (inputVoltage - outputVoltage) * tOn / inductance;
    const double minimumCurrent = outputCurrent - primaryCurrentPeakToPeak / 2.0;
    const double primaryVoltageMinimum = -outputVoltage - diodeVoltageDrop;
    const double primaryVoltageMaximum = inputVoltage - outputVoltage;
    const double primaryVoltagePeakToPeak = primaryVoltageMaximum - primaryVoltageMinimum;

    MAS::Waveform currentWaveform, voltageWaveform;
    if (minimumCurrent >= 0) {  // CCM
        currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, primaryCurrentPeakToPeak, switchingFrequency,
                                          dutyCycle, outputCurrent);
        voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeakToPeak, switchingFrequency,
                                          dutyCycle, 0.0);
    } else {  // DCM — recompute the conduction interval (the CCM duty would overrun the period)
        tOn = std::sqrt(2 * outputCurrent * inductance * (outputVoltage + diodeVoltageDrop) /
                        (switchingFrequency * (inputVoltage - outputVoltage) * (inputVoltage + diodeVoltageDrop)));
        const double tOff = tOn * ((inputVoltage + diodeVoltageDrop) / (outputVoltage + diodeVoltageDrop) - 1);
        const double deadTime = period - tOn - tOff;
        primaryCurrentPeakToPeak = (inputVoltage - outputVoltage) * tOn / inductance;
        const double iAvg = primaryCurrentPeakToPeak / 2.0;       // area balance: avg = ΔIL/2
        const double dcmDutyCycle = tOn * switchingFrequency;
        currentWaveform = WP::create_waveform(Lbl::TRIANGULAR_WITH_DEADTIME, primaryCurrentPeakToPeak,
                                          switchingFrequency, dcmDutyCycle, iAvg, deadTime);
        // The DCM voltage needs explicit levels/times (a single duty can't express tOn + the level split).
        voltageWaveform.set_data(std::vector<double>{primaryVoltageMaximum, primaryVoltageMaximum,
                                                     primaryVoltageMinimum, primaryVoltageMinimum, 0, 0});
        voltageWaveform.set_time(std::vector<double>{0, tOn, tOn, tOn + tOff, tOn + tOff, period});
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
    }

    MAS::OperatingPoint operatingPoint;
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    return operatingPoint;
}

// Ported from MKF converter_models/Boost.cpp.
MAS::OperatingPoint analytical_boost(double inputVoltage, double outputVoltage, double outputCurrent,
                                     double switchingFrequency, double inductance,
                                     double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;

    double dutyCycle = 1.0 - inputVoltage * efficiency / (outputVoltage + diodeVoltageDrop);
    if (dutyCycle >= 1.0)
        throw std::invalid_argument("analytical_boost: required duty cycle >= 1");
    if (dutyCycle <= 0.0)
        throw std::invalid_argument("analytical_boost: duty cycle <= 0 (input voltage above output)");

    const double period = 1.0 / switchingFrequency;
    double tOn = dutyCycle / switchingFrequency;
    double primaryCurrentPeakToPeak = inputVoltage * tOn / inductance;
    const double primaryCurrentAverage = outputCurrent * (outputVoltage + diodeVoltageDrop) / inputVoltage; // input I
    const double primaryCurrentMinimum = primaryCurrentAverage - primaryCurrentPeakToPeak / 2.0;
    const double primaryVoltageMinimum = inputVoltage - outputVoltage - diodeVoltageDrop;
    const double primaryVoltageMaximum = inputVoltage;
    const double primaryVoltagePeakToPeak = primaryVoltageMaximum - primaryVoltageMinimum;

    MAS::Waveform currentWaveform, voltageWaveform;
    if (primaryCurrentMinimum >= 0) {  // CCM
        currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, primaryCurrentPeakToPeak, switchingFrequency,
                                              dutyCycle, primaryCurrentAverage);
        voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeakToPeak, switchingFrequency,
                                              dutyCycle, 0.0);
    } else {  // DCM
        tOn = std::sqrt(2 * outputCurrent * inductance * (outputVoltage + diodeVoltageDrop - inputVoltage) /
                        (switchingFrequency * inputVoltage * inputVoltage));
        const double tOff = tOn * ((outputVoltage + diodeVoltageDrop) / (outputVoltage + diodeVoltageDrop - inputVoltage) - 1);
        const double deadTime = period - tOn - tOff;
        primaryCurrentPeakToPeak = inputVoltage * tOn / inductance;
        const double iAvg = primaryCurrentPeakToPeak / 2.0;
        const double dcmDutyCycle = tOn * switchingFrequency;
        currentWaveform = WP::create_waveform(Lbl::TRIANGULAR_WITH_DEADTIME, primaryCurrentPeakToPeak,
                                              switchingFrequency, dcmDutyCycle, iAvg, deadTime);
        voltageWaveform.set_data(std::vector<double>{primaryVoltageMaximum, primaryVoltageMaximum,
                                                     primaryVoltageMinimum, primaryVoltageMinimum, 0, 0});
        voltageWaveform.set_time(std::vector<double>{0, tOn, tOn, tOn + tOff, tOn + tOff, period});
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
    }

    MAS::OperatingPoint operatingPoint;
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    return operatingPoint;
}

} // namespace analytical
} // namespace Kirchhoff
