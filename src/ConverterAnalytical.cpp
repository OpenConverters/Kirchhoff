#include "ConverterAnalytical.hpp"
#include "processors/WaveformProcessor.h"   // the shared DSP (MKF), reused — not re-implemented
#include "converter_models/PwmBridgeSolver.h"  // shared phase-shifted-bridge kernel (MKF), reused — not re-implemented

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
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

// ─────────────────────────────────────────────────────────────────────────────
// Phase 3: the PWM converter family. Each body ports one MKF
// process_operating_points_for_input_voltage faithfully (file:line cited per fn).
// ─────────────────────────────────────────────────────────────────────────────

// Flyback::get_total_input_power(currents, voltages, eff, Vd) — the reflected-power helper.
static double flyback_total_input_power(const std::vector<double>& outputCurrents,
                                        const std::vector<double>& outputVoltages,
                                        double efficiency, double diodeVoltageDrop) {
    double totalPower = 0;
    for (size_t i = 0; i < outputCurrents.size(); ++i)
        totalPower += outputCurrents[i] * (outputVoltages[i] + diodeVoltageDrop);
    return totalPower / efficiency;
}

// Ported from MKF converter_models/Flyback.cpp:123 (process_operating_points_for_input_voltage).
MAS::OperatingPoint analytical_flyback(double inputVoltage,
                                       const std::vector<double>& outputVoltages,
                                       const std::vector<double>& outputCurrents,
                                       const std::vector<double>& turnsRatios,
                                       double switchingFrequency, double inductance,
                                       double diodeVoltageDrop, double efficiency,
                                       double currentRippleRatio) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_flyback: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");

    const double deadTime = 0.0;
    double maximumReflectedOutputVoltage = 0;
    for (size_t i = 0; i < outputVoltages.size(); ++i)
        maximumReflectedOutputVoltage = std::max(maximumReflectedOutputVoltage,
                                                 (outputVoltages[i] + diodeVoltageDrop) * turnsRatios[i]);
    double primaryVoltagePeaktoPeak = inputVoltage + maximumReflectedOutputVoltage;

    double totalOutputPower = flyback_total_input_power(outputCurrents, outputVoltages, 1.0, 0.0);
    double maximumEffectiveLoadCurrent = totalOutputPower / outputVoltages[0];
    double maximumEffectiveLoadCurrentReflected = maximumEffectiveLoadCurrent / turnsRatios[0];
    double totalInputPower = flyback_total_input_power(outputCurrents, outputVoltages, efficiency, 0.0);
    double averageInputCurrent = totalInputPower / inputVoltage;

    double dCcm = averageInputCurrent / (averageInputCurrent + maximumEffectiveLoadCurrentReflected);

    // CCM/DCM boundary: critical primary inductance (Basso 2nd ed. p.747).
    double mainOutputVoltageWithDiode = outputVoltages[0] + diodeVoltageDrop;
    double aux = mainOutputVoltageWithDiode * turnsRatios[0];
    double criticalInductance = (totalOutputPower > 0)
        ? efficiency * inputVoltage * inputVoltage * aux * aux
            / (2.0 * totalOutputPower * switchingFrequency
               * (inputVoltage + aux) * (aux + efficiency * inputVoltage))
        : 0.0;
    bool isDcm = inductance < criticalInductance;

    double dutyCycle = isDcm
        ? std::sqrt(2.0 * totalInputPower * inductance * switchingFrequency) / inputVoltage  // DCM energy balance
        : dCcm;
    if (dutyCycle > 1)
        throw std::invalid_argument("analytical_flyback: dutyCycle cannot be larger than one: " +
                                    std::to_string(dutyCycle));

    double centerSecondaryCurrentRampLumped = maximumEffectiveLoadCurrent / (1 - dutyCycle);
    double centerPrimaryCurrentRamp = centerSecondaryCurrentRampLumped / turnsRatios[0];

    double primaryCurrentAverage = centerPrimaryCurrentRamp;
    double ripple;
    if (std::isnan(currentRippleRatio)) {
        double primaryCurrentPeakToPeak = inputVoltage * dutyCycle / switchingFrequency / inductance;
        ripple = primaryCurrentPeakToPeak / centerPrimaryCurrentRamp;
    } else {
        ripple = currentRippleRatio;
    }
    double primaryCurrentPeakToPeak = centerPrimaryCurrentRamp * ripple * 2;
    double primaryCurrentOffset = std::max(0.0, primaryCurrentAverage - primaryCurrentPeakToPeak / 2);

    // mode follows the primary current offset (CCM iff the ramp does not touch zero).
    bool modeIsCcm = (primaryCurrentOffset > 0);

    MAS::OperatingPoint operatingPoint;
    // Primary
    {
        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_PRIMARY, primaryCurrentPeakToPeak,
                                                            switchingFrequency, dutyCycle, primaryCurrentOffset, deadTime);
        MAS::Waveform voltageWaveform = modeIsCcm
            ? WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeaktoPeak, switchingFrequency, dutyCycle, 0, deadTime)
            : WP::create_waveform(Lbl::RECTANGULAR_WITH_DEADTIME, primaryVoltagePeaktoPeak, switchingFrequency, dutyCycle, 0, deadTime);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    }
    // Secondaries
    for (size_t i = 0; i < turnsRatios.size(); ++i) {
        double secondaryPower = outputCurrents[i] * (outputVoltages[i] + 0.0);
        double powerDivider = secondaryPower / totalOutputPower;
        double minimumSecondaryVoltage = -inputVoltage / turnsRatios[i];
        double maximumSecondaryVoltage = outputVoltages[i] + diodeVoltageDrop;
        double secondaryVoltagePeaktoPeak = maximumSecondaryVoltage - minimumSecondaryVoltage;
        double secondaryCurrentAverage = centerPrimaryCurrentRamp * turnsRatios[i] * powerDivider;
        double secondaryCurrentPeaktoPeak = secondaryCurrentAverage * ripple * 2;
        double secondaryCurrentOffset = std::max(0.0, secondaryCurrentAverage - secondaryCurrentPeaktoPeak / 2);

        double secondaryVoltageDuty = dutyCycle;
        if (isDcm) {
            double secondaryAux = maximumSecondaryVoltage * turnsRatios[i];
            secondaryVoltageDuty = secondaryAux / (inputVoltage + secondaryAux);
        }

        MAS::Waveform currentWaveform, voltageWaveform;
        if (modeIsCcm) {
            voltageWaveform = WP::create_waveform(Lbl::SECONDARY_RECTANGULAR, secondaryVoltagePeaktoPeak, switchingFrequency, secondaryVoltageDuty, 0, deadTime);
            currentWaveform = WP::create_waveform(Lbl::FLYBACK_SECONDARY, secondaryCurrentPeaktoPeak, switchingFrequency, dutyCycle, secondaryCurrentOffset, deadTime);
        } else {
            voltageWaveform = WP::create_waveform(Lbl::SECONDARY_RECTANGULAR_WITH_DEADTIME, secondaryVoltagePeaktoPeak, switchingFrequency, secondaryVoltageDuty, 0, deadTime);
            // DCM: the rectifier conducts only for t_sec (< toff), the reflected secondary current ramping
            // from its peak down to zero. secondaryCurrentPeaktoPeak/2 is the physical reflected peak
            // (= N_i·powerDivider_i·I_pri_pk). Confine the falling ramp to t_sec via a derived dead time so the
            // full-period average equals the load current Iout_i: <i_sec> = (I_pk/2)·(t_sec·fsw) = Iout_i.
            // NOTE: MKF's analytical Flyback.cpp leaves deadTime=0 here and the secondary integrates to
            // (I_pk/2)·(1−D) ≠ Iout (a latent bug; MKF only checks secondary-avg in its ngspice path). KH
            // owns the converter solvers now and enforces the correct physics, matching the ngspice waveform.
            const double secondaryPeakCurrent = secondaryCurrentPeaktoPeak / 2.0;
            const double period = 1.0 / switchingFrequency;
            const double secondaryConductionTime = 2.0 * outputCurrents[i] / (secondaryPeakCurrent * switchingFrequency);
            const double secondaryDeadTime = period - dutyCycle * period - secondaryConductionTime;
            currentWaveform = WP::create_waveform(Lbl::FLYBACK_SECONDARY_WITH_DEADTIME, secondaryPeakCurrent, switchingFrequency, dutyCycle, 0.0, secondaryDeadTime);
        }
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Secondary " + std::to_string(i)));
    }
    return operatingPoint;
}

// Ported from MKF converter_models/SingleSwitchForward.cpp:43.
MAS::OperatingPoint analytical_forward(double inputVoltage,
                                       const std::vector<double>& outputVoltages,
                                       const std::vector<double>& outputCurrents,
                                       const std::vector<double>& turnsRatios,
                                       double switchingFrequency, double inductance,
                                       double mainOutputInductance, double currentRippleRatio,
                                       double diodeVoltageDrop) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        turnsRatios.size() != outputVoltages.size() + 1)
        throw std::invalid_argument("analytical_forward: need turnsRatios = [demag, sec0, …] "
                                    "(one demag entry + one per output)");

    double mainOutputCurrent = outputCurrents[0];
    double mainOutputVoltage = outputVoltages[0];
    double mainSecondaryTurnsRatio = turnsRatios[1];

    const double period = 1.0 / switchingFrequency;
    double t1 = period * (mainOutputVoltage + diodeVoltageDrop) / (inputVoltage / mainSecondaryTurnsRatio);
    if (t1 > period / 2)
        throw std::invalid_argument("analytical_forward: T1 cannot be larger than period/2, wrong topology configuration");

    double magnetizationCurrent = inputVoltage * t1 / inductance;
    double minimumPrimaryCurrent = -magnetizationCurrent / 2;
    double maximumPrimaryCurrent = magnetizationCurrent / 2;

    std::vector<double> minimumSecondaryCurrents, maximumSecondaryCurrents;
    for (size_t i = 0; i < outputVoltages.size(); ++i) {
        double outputCurrentRipple = currentRippleRatio * outputCurrents[i];
        double minSec = outputCurrents[i] - outputCurrentRipple / 2;
        double maxSec = outputCurrents[i] + outputCurrentRipple / 2;
        minimumSecondaryCurrents.push_back(minSec);
        maximumSecondaryCurrents.push_back(maxSec);
        size_t turnsRatioSecondaryIndex = 1 + i;  // skip the demagnetization winding
        minimumPrimaryCurrent += minSec / turnsRatios[turnsRatioSecondaryIndex];
        maximumPrimaryCurrent += maxSec / turnsRatios[turnsRatioSecondaryIndex];
    }

    if (minimumPrimaryCurrent < 0) {  // DCM
        double sqrtArg = 2 * mainOutputCurrent * mainOutputInductance * (mainOutputVoltage + diodeVoltageDrop) /
                         (switchingFrequency * (inputVoltage / mainSecondaryTurnsRatio - diodeVoltageDrop - mainOutputVoltage) *
                          (inputVoltage / mainSecondaryTurnsRatio));
        if (sqrtArg < 0)
            throw std::invalid_argument("analytical_forward: negative value under sqrt in DCM t1 calculation");
        t1 = std::sqrt(sqrtArg);
        if (t1 > period / 2)
            throw std::invalid_argument("analytical_forward: T1 cannot be larger than period/2, wrong topology configuration");
        minimumPrimaryCurrent = 0;
        maximumPrimaryCurrent = magnetizationCurrent;
        for (size_t i = 0; i < outputVoltages.size(); ++i) {
            double outputCurrentRipple = currentRippleRatio * outputCurrents[i];
            minimumSecondaryCurrents[i] = 0;
            maximumSecondaryCurrents[i] = outputCurrentRipple;
            size_t turnsRatioSecondaryIndex = 1 + i;
            minimumPrimaryCurrent += minimumSecondaryCurrents[i] / turnsRatios[turnsRatioSecondaryIndex];
            maximumPrimaryCurrent += maximumSecondaryCurrents[i] / turnsRatios[turnsRatioSecondaryIndex];
        }
    }

    double td = t1;  // demagnetization time equals on-time for Nt = Np
    double deadTime = period - t1 - td;
    double actualDutyCycle = t1 / period;

    MAS::OperatingPoint operatingPoint;
    // Primary — explicit +Vin / −Vin / 0 voltage levels.
    {
        double primaryCurrentPeakToPeak = maximumPrimaryCurrent - minimumPrimaryCurrent;
        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_PRIMARY, primaryCurrentPeakToPeak,
                                                            switchingFrequency, actualDutyCycle, minimumPrimaryCurrent, deadTime);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_data(std::vector<double>{0, inputVoltage, inputVoltage, -inputVoltage, -inputVoltage, 0, 0});
        voltageWaveform.set_time(std::vector<double>{0, 0, t1, t1, t1 + td, t1 + td, period});
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    }
    // Demagnetization winding — inverted voltage polarity.
    {
        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_SECONDARY_WITH_DEADTIME, magnetizationCurrent,
                                                            switchingFrequency, actualDutyCycle, minimumPrimaryCurrent, deadTime);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_data(std::vector<double>{0, -inputVoltage, -inputVoltage, inputVoltage, inputVoltage, 0, 0});
        voltageWaveform.set_time(std::vector<double>{0, 0, t1, t1, t1 + td, t1 + td, period});
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Demagnetization winding"));
    }
    // Secondaries
    for (size_t i = 0; i < outputVoltages.size(); ++i) {
        double secondaryCurrentPeakToPeak = maximumSecondaryCurrents[i] - minimumSecondaryCurrents[i];
        size_t turnsRatioSecondaryIndex = 1 + i;
        double minimumSecondaryVoltage = -(inputVoltage + diodeVoltageDrop) / turnsRatios[turnsRatioSecondaryIndex];
        double maximumSecondaryVoltage = inputVoltage / turnsRatios[turnsRatioSecondaryIndex];
        double secondaryVoltagePeakToPeak = maximumSecondaryVoltage - minimumSecondaryVoltage;
        double secondaryVoltageOffset = maximumSecondaryVoltage + minimumSecondaryVoltage;

        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_PRIMARY, secondaryCurrentPeakToPeak,
                                                            switchingFrequency, actualDutyCycle, minimumSecondaryCurrents[i], 0);
        MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR_WITH_DEADTIME, secondaryVoltagePeakToPeak,
                                                            switchingFrequency, actualDutyCycle, secondaryVoltageOffset, deadTime);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Secondary " + std::to_string(i)));
    }
    return operatingPoint;
}

// Ported from MKF converter_models/TwoSwitchForward.cpp:41.
MAS::OperatingPoint analytical_two_switch_forward(double inputVoltage,
                                                  const std::vector<double>& outputVoltages,
                                                  const std::vector<double>& outputCurrents,
                                                  const std::vector<double>& turnsRatios,
                                                  double switchingFrequency, double inductance,
                                                  double mainOutputInductance, double currentRippleRatio,
                                                  double diodeVoltageDrop) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        turnsRatios.size() != outputVoltages.size())
        throw std::invalid_argument("analytical_two_switch_forward: need one turnsRatio per output");

    double mainOutputCurrent = outputCurrents[0];
    double mainOutputVoltage = outputVoltages[0];
    double mainSecondaryTurnsRatio = turnsRatios[0];

    const double period = 1.0 / switchingFrequency;
    double t1 = period * (mainOutputVoltage + diodeVoltageDrop) / (inputVoltage / mainSecondaryTurnsRatio);
    if (t1 > period / 2)
        throw std::invalid_argument("analytical_two_switch_forward: T1 cannot be larger than period/2, wrong topology configuration");

    double magnetizationCurrent = inputVoltage * t1 / inductance;
    double minimumPrimaryCurrent = -magnetizationCurrent / 2;
    double maximumPrimaryCurrent = magnetizationCurrent / 2;

    std::vector<double> minimumSecondaryCurrents, maximumSecondaryCurrents;
    for (size_t i = 0; i < outputVoltages.size(); ++i) {
        double outputCurrentRipple = currentRippleRatio * outputCurrents[i];
        double minSec = outputCurrents[i] - outputCurrentRipple / 2;
        double maxSec = outputCurrents[i] + outputCurrentRipple / 2;
        minimumSecondaryCurrents.push_back(minSec);
        maximumSecondaryCurrents.push_back(maxSec);
        minimumPrimaryCurrent += minSec / turnsRatios[i];
        maximumPrimaryCurrent += maxSec / turnsRatios[i];
    }

    if (minimumPrimaryCurrent < 0) {  // DCM
        double sqrtArg = 2 * mainOutputCurrent * mainOutputInductance * (mainOutputVoltage + diodeVoltageDrop) /
                         (switchingFrequency * (inputVoltage / mainSecondaryTurnsRatio - diodeVoltageDrop - mainOutputVoltage) *
                          (inputVoltage / mainSecondaryTurnsRatio));
        if (sqrtArg < 0)
            throw std::invalid_argument("analytical_two_switch_forward: negative value under sqrt in DCM t1 calculation");
        t1 = std::sqrt(sqrtArg);
        if (t1 > period / 2)
            throw std::invalid_argument("analytical_two_switch_forward: T1 cannot be larger than period/2, wrong topology configuration");
        minimumPrimaryCurrent = 0;
        maximumPrimaryCurrent = magnetizationCurrent;
        for (size_t i = 0; i < outputVoltages.size(); ++i) {
            double outputCurrentRipple = currentRippleRatio * outputCurrents[i];
            minimumSecondaryCurrents[i] = 0;
            maximumSecondaryCurrents[i] = outputCurrentRipple;
            minimumPrimaryCurrent += minimumSecondaryCurrents[i] / turnsRatios[i];
            maximumPrimaryCurrent += maximumSecondaryCurrents[i] / turnsRatios[i];
        }
    }

    double minimumPrimarySideTransformerVoltage = -inputVoltage - 2 * diodeVoltageDrop;
    double maximumPrimarySideTransformerVoltage = inputVoltage;
    double td = t1;
    double deadTime = period - t1 - td;

    MAS::OperatingPoint operatingPoint;
    // Primary (single combined winding)
    {
        MAS::Waveform currentWaveform, voltageWaveform;
        if (minimumPrimaryCurrent > 0) {  // CCM
            currentWaveform.set_data(std::vector<double>{0, minimumPrimaryCurrent, maximumPrimaryCurrent, magnetizationCurrent, 0, 0, 0});
            currentWaveform.set_time(std::vector<double>{0, 0, t1, t1, t1 + td, period, period});
            currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        } else {  // DCM
            currentWaveform.set_data(std::vector<double>{minimumPrimaryCurrent, maximumPrimaryCurrent, 0, 0});
            currentWaveform.set_time(std::vector<double>{0, t1, t1, period});
            currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        }
        voltageWaveform.set_data(std::vector<double>{0, maximumPrimarySideTransformerVoltage, maximumPrimarySideTransformerVoltage,
                                                     minimumPrimarySideTransformerVoltage, minimumPrimarySideTransformerVoltage, 0, 0});
        voltageWaveform.set_time(std::vector<double>{0, 0, t1, t1, t1 + td, t1 + td, period});
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "First primary"));
    }
    double actualDutyCycle = t1 / period;
    for (size_t i = 0; i < outputVoltages.size(); ++i) {
        double secondaryCurrentPeakToPeak = maximumSecondaryCurrents[i] - minimumSecondaryCurrents[i];
        double minimumSecondaryVoltage = -(inputVoltage + 2 * diodeVoltageDrop) / turnsRatios[i];
        double maximumSecondaryVoltage = inputVoltage / turnsRatios[i];
        double secondaryVoltagePeakToPeak = maximumSecondaryVoltage - minimumSecondaryVoltage;
        double secondaryVoltageOffset = maximumSecondaryVoltage + minimumSecondaryVoltage;

        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_PRIMARY, secondaryCurrentPeakToPeak,
                                                            switchingFrequency, actualDutyCycle, minimumSecondaryCurrents[i], 0);
        MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR_WITH_DEADTIME, secondaryVoltagePeakToPeak,
                                                            switchingFrequency, actualDutyCycle, secondaryVoltageOffset, deadTime);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Secondary " + std::to_string(i)));
    }
    return operatingPoint;
}

// Ported from MKF converter_models/PushPull.cpp:71 (single main output: the auxiliary-secondary
// loop does not execute, so this reproduces the canonical four center-tapped windings).
MAS::OperatingPoint analytical_push_pull(double inputVoltage, double outputVoltage,
                                         double outputCurrent, double switchingFrequency,
                                         double turnsRatio, double inductance,
                                         double outputInductance, double currentRippleRatio,
                                         double diodeVoltageDrop) {
    using Lbl = MAS::WaveformLabel;
    const double mainSecondaryTurnsRatio = turnsRatio;
    const double inductorCurrentRipple = currentRippleRatio * outputCurrent;
    const double period = 1.0 / switchingFrequency;
    double t1 = period / 2 * (outputVoltage + diodeVoltageDrop) / (inputVoltage / mainSecondaryTurnsRatio);
    if (t1 > period / 2)
        throw std::invalid_argument("analytical_push_pull: T1 cannot be larger than period/2, wrong topology configuration");

    double magnetizationCurrent = inputVoltage * t1 / inductance;
    double minimumSecondaryCurrent = outputCurrent - inductorCurrentRipple / 2;
    double maximumSecondaryCurrent = outputCurrent + inductorCurrentRipple / 2;
    double minimumPrimaryCurrent = minimumSecondaryCurrent / mainSecondaryTurnsRatio - magnetizationCurrent / 2;
    double maximumPrimaryCurrent = maximumSecondaryCurrent / mainSecondaryTurnsRatio + magnetizationCurrent / 2;

    MAS::OperatingPoint operatingPoint;
    auto pushCustom = [&](const std::vector<double>& iData, const std::vector<double>& iTime,
                          const std::vector<double>& vData, const std::vector<double>& vTime,
                          const std::string& name) {
        MAS::Waveform currentWaveform, voltageWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(iData);
        currentWaveform.set_time(iTime);
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(vData);
        voltageWaveform.set_time(vTime);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, name));
    };

    if (minimumPrimaryCurrent > 0) {  // CCM
        double minPriI = minimumPrimaryCurrent, maxPriI = maximumPrimaryCurrent;
        double minPriV = -inputVoltage, maxPriV = inputVoltage;
        double minSecT1OfFET = minimumSecondaryCurrent, maxSecT1OfFET = maximumSecondaryCurrent;
        double minSecT2Other = (maximumSecondaryCurrent / mainSecondaryTurnsRatio + magnetizationCurrent / 2) / 2 * mainSecondaryTurnsRatio - inductorCurrentRipple / 2;
        double maxSecT2Other = (maximumSecondaryCurrent / mainSecondaryTurnsRatio + magnetizationCurrent / 2) / 2 * mainSecondaryTurnsRatio;
        double minSecT2OfFET = minimumSecondaryCurrent - minSecT2Other;
        double maxSecT2OfFET = maximumSecondaryCurrent - maxSecT2Other;
        double minSecV = -inputVoltage / mainSecondaryTurnsRatio, maxSecV = inputVoltage / mainSecondaryTurnsRatio;

        pushCustom({minPriI, maxPriI, 0, 0}, {0, t1, t1, period},
                   {maxPriV, maxPriV, 0, 0, minPriV, minPriV, 0, 0},
                   {0, t1, t1, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   "Primary Half 1");
        pushCustom({0, 0, minPriI, maxPriI, 0, 0}, {0, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   {minPriV, minPriV, 0, 0, maxPriV, maxPriV, 0, 0},
                   {0, t1, t1, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   "Primary Half 2");
        pushCustom({0, 0, maxSecT2Other, minSecT2Other, minSecT1OfFET, maxSecT1OfFET, maxSecT2OfFET, minSecT2OfFET},
                   {0, t1, t1, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   {minSecV, minSecV, 0, 0, maxSecV, maxSecV, 0, 0},
                   {0, t1, t1, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   "Secondary 0 Half 1");
        pushCustom({minSecT1OfFET, maxSecT1OfFET, maxSecT2OfFET, minSecT2OfFET, 0, 0, maxSecT2Other, minSecT2Other},
                   {0, t1, t1, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   {maxSecV, maxSecV, 0, 0, minSecV, minSecV, 0, 0},
                   {0, t1, t1, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period},
                   "Secondary 0 Half 2");
    } else {  // DCM
        double t1d = std::sqrt(2 * outputCurrent * outputInductance * (outputVoltage + diodeVoltageDrop) /
                               (2 * switchingFrequency * (inputVoltage / mainSecondaryTurnsRatio - diodeVoltageDrop - outputVoltage) *
                                (inputVoltage / mainSecondaryTurnsRatio)));
        double t2 = t1d * (inputVoltage / mainSecondaryTurnsRatio) / (outputVoltage + diodeVoltageDrop) - t1d;
        if (t1d + t2 > period / 2)
            throw std::invalid_argument("analytical_push_pull: T1 + T2 cannot be larger than period/2, wrong topology configuration");
        t1 = t1d;

        double maxSecCurrent = inductorCurrentRipple;
        double minPriI = 0;
        double maxPriI = inductorCurrentRipple / mainSecondaryTurnsRatio + magnetizationCurrent / 2;
        double minPriV = -inputVoltage, maxPriV = inputVoltage;

        double maxSecT1OfFET = maxSecCurrent;
        double minSecT2Other = (maxSecCurrent / mainSecondaryTurnsRatio + magnetizationCurrent / 2) / 2 * mainSecondaryTurnsRatio - inductorCurrentRipple / 2;
        double maxSecT2Other = (maxSecCurrent / mainSecondaryTurnsRatio + magnetizationCurrent / 2) / 2 * mainSecondaryTurnsRatio;
        double minSecT2OfFET = 0;
        double maxSecT2OfFET = maxSecCurrent - maxSecT2Other;
        double minSecV = -inputVoltage / mainSecondaryTurnsRatio, maxSecV = inputVoltage / mainSecondaryTurnsRatio;

        double minPriVT3 = -(outputVoltage + diodeVoltageDrop) * mainSecondaryTurnsRatio;
        double maxPriVT3 = (outputVoltage + diodeVoltageDrop) * mainSecondaryTurnsRatio;
        double maxSecT3 = maxSecT2Other - maxSecT2OfFET;
        double minSecVT3 = -outputVoltage - diodeVoltageDrop;
        double maxSecVT3 = outputVoltage + diodeVoltageDrop;

        pushCustom({minPriI, maxPriI, 0, 0}, {0, t1, t1, period},
                   {maxPriV, maxPriV, 0, 0, minPriVT3, minPriVT3, minPriV, minPriV, 0, 0, maxPriVT3, maxPriVT3},
                   {0, t1, t1, t1 + t2, t1 + t2, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period / 2 + t1 + t2, period / 2 + t1 + t2, period},
                   "Primary Half 1");
        pushCustom({0, minPriI, maxPriI, 0, 0}, {0, period / 2, period / 2 + t1, period / 2 + t1, period},
                   {minPriV, minPriV, 0, 0, maxPriVT3, maxPriVT3, maxPriV, maxPriV, 0, 0, minPriVT3, minPriVT3},
                   {0, t1, t1, t1 + t2, t1 + t2, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period / 2 + t1 + t2, period / 2 + t1 + t2, period},
                   "Primary Half 2");
        pushCustom({0, 0, maxSecT2Other, minSecT2Other, maxSecT3, 0, maxSecT1OfFET, maxSecT2OfFET, minSecT2OfFET, 0},
                   {0, t1, t1, t1 + t2, t1 + t2, period / 2, period / 2 + t1, period / 2 + t1, period / 2 + t1 + t2, period},
                   {minSecV, minSecV, 0, 0, maxSecVT3, maxSecVT3, maxSecV, maxSecV, 0, 0, minSecVT3, minSecVT3},
                   {0, t1, t1, t1 + t2, t1 + t2, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period / 2 + t1 + t2, period / 2 + t1 + t2, period},
                   "Secondary 0 Half 1");
        pushCustom({0, maxSecT1OfFET, maxSecT2OfFET, minSecT2OfFET, 0, maxSecT2Other, minSecT2Other, maxSecT3, 0},
                   {0, t1, t1, t1 + t2, period / 2 + t1, period / 2 + t1, period / 2 + t1 + t2, period / 2 + t1 + t2, period},
                   {maxSecV, maxSecV, 0, 0, minSecVT3, minSecVT3, minSecV, minSecV, 0, 0, maxSecVT3, maxSecVT3},
                   {0, t1, t1, t1 + t2, t1 + t2, period / 2, period / 2, period / 2 + t1, period / 2 + t1, period / 2 + t1 + t2, period / 2 + t1 + t2, period},
                   "Secondary 0 Half 2");
    }
    return operatingPoint;
}

// Weinberg helpers (ported from MKF Weinberg.cpp:19-112).
static int weinberg_detect_operating_regime(double dutyCycle) {
    constexpr double eps = 1e-6;
    if (dutyCycle < 0.5 - eps) return 0;   // buck-like
    if (dutyCycle > 0.5 + eps) return 2;   // boost-like
    return 1;                              // boundary
}
static double weinberg_conversion_ratio_boost(double dutyCycle, double turnsRatio) {
    if (turnsRatio <= 0.0) throw std::invalid_argument("analytical_weinberg: turnsRatio must be > 0");
    double oneMinusD = 1.0 - dutyCycle;
    if (oneMinusD <= 1e-6) throw std::invalid_argument("analytical_weinberg: D too close to 1 — singular gain");
    return 1.0 / (2.0 * turnsRatio * oneMinusD);
}
static double weinberg_conversion_ratio_buck(double dutyCycle, double turnsRatio) {
    if (turnsRatio <= 0.0) throw std::invalid_argument("analytical_weinberg: turnsRatio must be > 0");
    return 2.0 * dutyCycle / turnsRatio;
}
static double weinberg_duty_cycle(double inputVoltage, double outputVoltage, double turnsRatio,
                                  double diodeVoltageDrop, double efficiency, double maximumDutyCycle) {
    if (inputVoltage <= 0) throw std::invalid_argument("analytical_weinberg: input voltage must be > 0");
    if (outputVoltage <= 0) throw std::invalid_argument("analytical_weinberg: outputVoltage must be > 0");
    if (turnsRatio <= 0) throw std::invalid_argument("analytical_weinberg: turnsRatio must be > 0");
    double M = (outputVoltage + diodeVoltageDrop) / (inputVoltage * efficiency);
    double dBoost = 1.0 - 1.0 / (2.0 * turnsRatio * M);
    if (dBoost > 0.5) {
        if (dBoost >= maximumDutyCycle - 0.01)
            throw std::invalid_argument("analytical_weinberg: D exceeds maximumDutyCycle");
        return dBoost;
    }
    double dBuck = 0.5 * turnsRatio * M;
    if (dBuck >= 0.5) return 0.5;
    if (dBuck <= 1e-3) throw std::invalid_argument("analytical_weinberg: D <= 0.001 — converter would lose regulation");
    return dBuck;
}

// Ported from MKF converter_models/Weinberg.cpp:182 (the Primary + Secondary excitations only;
// the variant only affects diagnostic switch-stress, not the winding waveforms).
MAS::OperatingPoint analytical_weinberg(double inputVoltage, double outputVoltage,
                                        double outputCurrent, double switchingFrequency,
                                        double inductance, double turnsRatio,
                                        double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    double dutyCycle = weinberg_duty_cycle(inputVoltage, outputVoltage, turnsRatio, diodeVoltageDrop, efficiency, 0.95);
    int regime = weinberg_detect_operating_regime(dutyCycle);
    double overlap = std::max(0.0, 2.0 * dutyCycle - 1.0);
    double M = (regime == 2) ? weinberg_conversion_ratio_boost(dutyCycle, turnsRatio)
                             : weinberg_conversion_ratio_buck(dutyCycle, turnsRatio);

    double inputCurrent = (M > 0) ? outputCurrent / (efficiency * M) : 0.0;
    double inductanceL1 = inductance;
    double deltaIL1 = inductanceL1 > 0 ? (inputVoltage * std::max(overlap, dutyCycle)) / (inductanceL1 * switchingFrequency) : 0.0;

    const double period = 1.0 / switchingFrequency;
    const double iL1_low = std::max(inputCurrent - 0.5 * deltaIL1, 0.0);
    const double iL1_high = inputCurrent + 0.5 * deltaIL1;
    const double iL1_half_low = 0.5 * iL1_low;
    const double iL1_half_high = 0.5 * iL1_high;

    std::vector<double> iData, iTime;
    if (regime == 2) {  // boost: overlap1 / Q1-only / overlap2 / Q2-only
        const double tOverlap = (2.0 * dutyCycle - 1.0) * period / 2.0;
        const double t1 = tOverlap, t2 = period / 2.0, t3 = period / 2.0 + tOverlap;
        iData = {iL1_half_low, iL1_half_high, iL1_low, iL1_high, iL1_half_high, iL1_half_low, 0.0, 0.0};
        iTime = {0.0, t1, t1, t2, t2, t3, t3, period};
    } else {  // buck or boundary: pulse during Q1-on
        const double tOn = dutyCycle * period;
        iData = {iL1_low, iL1_high, 0.0, 0.0};
        iTime = {0.0, tOn, tOn, period};
    }

    MAS::OperatingPoint operatingPoint;
    {
        MAS::Waveform iPri;
        iPri.set_ancillary_label(Lbl::CUSTOM);
        iPri.set_data(iData);
        iPri.set_time(iTime);
        MAS::Waveform vPri = WP::create_waveform(Lbl::BIPOLAR_RECTANGULAR, 2.0 * inputVoltage, switchingFrequency, 0.5, 0.0, 0);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iPri, vPri, switchingFrequency, "Primary"));

        const double nInv = (turnsRatio > 1e-9) ? (1.0 / turnsRatio) : 0.0;
        MAS::Waveform iSec;
        std::vector<double> iSecData;
        iSecData.reserve(iData.size());
        for (double v : iData) iSecData.push_back(v * turnsRatio);
        iSec.set_ancillary_label(Lbl::CUSTOM);
        iSec.set_data(iSecData);
        iSec.set_time(iTime);
        MAS::Waveform vSec = WP::create_waveform(Lbl::BIPOLAR_RECTANGULAR, 2.0 * inputVoltage * nInv, switchingFrequency, 0.5, 0.0, 0);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iSec, vSec, switchingFrequency, "Secondary"));
    }
    return operatingPoint;
}

// SEPIC/Cuk/Zeta share the same CCM duty: D = (Vo+Vd)·n / (Vin·η + (Vo+Vd)·n).
static double single_inductor_duty_cycle(double inputVoltage, double outputVoltage, double diodeVoltageDrop,
                                         double efficiency, double turnsRatio, double maximumDutyCycle,
                                         const char* who) {
    if (inputVoltage <= 0) throw std::invalid_argument(std::string(who) + ": input voltage must be > 0");
    if (outputVoltage <= 0) throw std::invalid_argument(std::string(who) + ": Vo must be > 0");
    double reflectedVo = (outputVoltage + diodeVoltageDrop) * turnsRatio;
    double dutyCycle = reflectedVo / (reflectedVo + inputVoltage * efficiency);
    if (dutyCycle > maximumDutyCycle * 1.01)
        throw std::invalid_argument(std::string(who) + ": duty cycle exceeds maximumDutyCycle");
    return dutyCycle;
}

// Ported from MKF converter_models/Sepic.cpp:97.
MAS::OperatingPoint analytical_sepic(double inputVoltage, double outputVoltage,
                                     double outputCurrent, double switchingFrequency,
                                     double inductanceL1, double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    double dutyCycle = single_inductor_duty_cycle(inputVoltage, outputVoltage, diodeVoltageDrop, efficiency, 1.0, 0.95, "analytical_sepic");
    double IL1avg = outputCurrent * dutyCycle / ((1.0 - dutyCycle) * efficiency);
    double deltaIL1 = inductanceL1 > 0 ? (inputVoltage * dutyCycle) / (inductanceL1 * switchingFrequency) : 0.0;
    double primaryVoltagePeakToPeak = inputVoltage + outputVoltage;

    MAS::OperatingPoint operatingPoint;
    MAS::Waveform currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, deltaIL1, switchingFrequency, dutyCycle, IL1avg, 0);
    MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeakToPeak, switchingFrequency, dutyCycle, 0, 0);
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    return operatingPoint;
}

// Ported from MKF converter_models/Cuk.cpp:174 (V1/V2 non-isolated path).
MAS::OperatingPoint analytical_cuk(double inputVoltage, double outputVoltage,
                                   double outputCurrent, double switchingFrequency,
                                   double inductanceL1, double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    double outputVoltageMag = std::abs(outputVoltage);
    double dutyCycle = single_inductor_duty_cycle(inputVoltage, outputVoltageMag, diodeVoltageDrop, efficiency, 1.0, 0.95, "analytical_cuk");
    double IL1avg = outputCurrent * dutyCycle / ((1.0 - dutyCycle) * 1.0 * efficiency);
    double VC1 = inputVoltage / (1.0 - dutyCycle);
    double deltaIL1 = inductanceL1 > 0 ? (inputVoltage * dutyCycle) / (inductanceL1 * switchingFrequency) : 0.0;
    double primaryVoltagePeakToPeak = VC1;  // L1 sees +Vin (ON) / Vin−VC1 (OFF), pp = VC1

    MAS::OperatingPoint operatingPoint;
    MAS::Waveform currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, deltaIL1, switchingFrequency, dutyCycle, IL1avg, 0);
    MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeakToPeak, switchingFrequency, dutyCycle, 0, 0);
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    return operatingPoint;
}

// Ported from MKF converter_models/Zeta.cpp:100.
MAS::OperatingPoint analytical_zeta(double inputVoltage, double outputVoltage,
                                    double outputCurrent, double switchingFrequency,
                                    double inductanceL1, double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    double dutyCycle = single_inductor_duty_cycle(inputVoltage, outputVoltage, diodeVoltageDrop, efficiency, 1.0, 0.95, "analytical_zeta");
    double IL1avg = outputCurrent * dutyCycle / ((1.0 - dutyCycle) * efficiency);
    double deltaIL1 = inductanceL1 > 0 ? (inputVoltage * dutyCycle) / (inductanceL1 * switchingFrequency) : 0.0;
    double primaryVoltagePeakToPeak = inputVoltage + outputVoltage;

    MAS::OperatingPoint operatingPoint;
    MAS::Waveform currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, deltaIL1, switchingFrequency, dutyCycle, IL1avg, 0);
    MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeakToPeak, switchingFrequency, dutyCycle, 0, 0);
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    return operatingPoint;
}

// Ported from MKF converter_models/FourSwitchBuckBoost.cpp (BUCK + BOOST branches of
// process_operating_point_for_excitation; compute_buck_duty:56 / compute_boost_duty:76).
MAS::OperatingPoint analytical_fsbb(double inputVoltage, double outputVoltage,
                                    double outputCurrent, double switchingFrequency,
                                    double inductance, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    const double period = 1.0 / switchingFrequency;
    const double maximumDutyCycle = 0.95;

    double dutyForWaveform, dIL, iL_avg, primaryVoltagePtp;
    if (outputVoltage < inputVoltage) {  // BUCK
        double D_buck = outputVoltage / (inputVoltage * efficiency);
        if (D_buck >= 1.0) throw std::invalid_argument("analytical_fsbb: buck D >= 1");
        if (D_buck >= maximumDutyCycle - 0.01) throw std::invalid_argument("analytical_fsbb: buck D exceeds maximumDutyCycle");
        double tOn = D_buck * period;
        iL_avg = outputCurrent;
        dIL = (inputVoltage - outputVoltage) * tOn / inductance;
        primaryVoltagePtp = inputVoltage;
        dutyForWaveform = D_buck;
    } else if (outputVoltage > inputVoltage) {  // BOOST
        double D_boost = 1.0 - (inputVoltage * efficiency) / outputVoltage;
        if (D_boost >= 1.0 || D_boost <= 0.0) throw std::invalid_argument("analytical_fsbb: boost D out of range");
        if (D_boost >= maximumDutyCycle - 0.01) throw std::invalid_argument("analytical_fsbb: boost D exceeds maximumDutyCycle");
        double tOn = D_boost * period;
        iL_avg = outputCurrent / (1.0 - D_boost);
        dIL = inputVoltage * tOn / inductance;
        primaryVoltagePtp = outputVoltage;
        dutyForWaveform = D_boost;
    } else {
        throw std::invalid_argument("analytical_fsbb: Vo == Vin lands in the buck-boost transition region, not ported");
    }

    MAS::OperatingPoint operatingPoint;
    MAS::Waveform currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, dIL, switchingFrequency, dutyForWaveform, iL_avg, 0);
    MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePtp, switchingFrequency, dutyForWaveform, 0, 0);
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Inductor"));
    return operatingPoint;
}

// Ported from MKF converter_models/IsolatedBuck.cpp:41 (fly-buck; D = Vpri/Vin·η, calculate_duty_cycle:10).
MAS::OperatingPoint analytical_isolated_buck(double inputVoltage, double primaryOutputVoltage,
                                             double primaryOutputCurrent, double secondaryOutputVoltage,
                                             double secondaryOutputCurrent, double switchingFrequency,
                                             double inductance, double turnsRatio,
                                             double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    (void)secondaryOutputVoltage;  // structural (2 outputs); the flyback secondary winding voltage is set by Vpri/n.
    double dutyCycle = primaryOutputVoltage / inputVoltage * efficiency;
    if (dutyCycle >= 1) throw std::invalid_argument("analytical_isolated_buck: duty cycle must be smaller than 1");
    const double period = 1.0 / switchingFrequency;
    double tOn = dutyCycle * period;

    double totalReflectedSecondaryCurrent = secondaryOutputCurrent / turnsRatio;
    double magnetizingCurrentRipple = (inputVoltage - primaryOutputVoltage) * tOn / inductance;
    double magnetizingCurrentAverage = primaryOutputCurrent + totalReflectedSecondaryCurrent;
    double magnetizingCurrentMax = magnetizingCurrentAverage + magnetizingCurrentRipple / 2;
    double magnetizingCurrentMin = magnetizingCurrentAverage - magnetizingCurrentRipple / 2;
    double reflectedSecondaryOffsetOff = (secondaryOutputCurrent / (1.0 - dutyCycle)) / turnsRatio;

    double primaryVoltageMaximum = inputVoltage - primaryOutputVoltage;
    double primaryVoltageMinimum = -primaryOutputVoltage;
    double primaryVoltagePeaktoPeak = primaryVoltageMaximum - primaryVoltageMinimum;  // = Vin

    MAS::OperatingPoint operatingPoint;
    // Primary — piecewise (winding currents step at switch events while the magnetizing flux stays continuous).
    {
        double iPriOnStart = magnetizingCurrentMin;
        double iPriOnEnd = magnetizingCurrentMax;
        double iPriOffStart = magnetizingCurrentMax - reflectedSecondaryOffsetOff;
        double iPriOffEnd = magnetizingCurrentMin - reflectedSecondaryOffsetOff;
        MAS::Waveform currentWaveform;
        currentWaveform.set_data(std::vector<double>{iPriOnStart, iPriOnEnd, iPriOffStart, iPriOffEnd});
        currentWaveform.set_time(std::vector<double>{0.0, tOn, tOn, period});
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltagePeaktoPeak, switchingFrequency, dutyCycle, 0, 0);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    }
    // Secondary (single isolated output)
    {
        double secondaryCurrentMaximum = (1 + dutyCycle) / (1 - dutyCycle) * secondaryOutputCurrent - secondaryOutputCurrent;
        double secondaryCurrentMinimum = 0;
        double secondaryVoltageMinimum = -(inputVoltage - primaryOutputVoltage) / turnsRatio;
        double secondaryVoltageMaximum = primaryOutputVoltage / turnsRatio - diodeVoltageDrop;

        MAS::Waveform currentWaveform;
        currentWaveform.set_data(std::vector<double>{0, 0, secondaryOutputCurrent + secondaryCurrentMinimum, secondaryOutputCurrent + secondaryCurrentMaximum});
        currentWaveform.set_time(std::vector<double>{0, tOn, tOn, period});
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_data(std::vector<double>{secondaryVoltageMinimum, secondaryVoltageMinimum, secondaryVoltageMaximum, secondaryVoltageMaximum});
        voltageWaveform.set_time(std::vector<double>{0, tOn, tOn, period});
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Secondary 0"));
    }
    return operatingPoint;
}

// Ported from MKF converter_models/IsolatedBuckBoost.cpp:47 (fly-buck-boost; D = Vpri/(Vin+Vpri)·η, calculate_duty_cycle:16).
MAS::OperatingPoint analytical_isolated_buck_boost(double inputVoltage, double primaryOutputVoltage,
                                                   double primaryOutputCurrent, double secondaryOutputVoltage,
                                                   double secondaryOutputCurrent, double switchingFrequency,
                                                   double inductance, double turnsRatio,
                                                   double diodeVoltageDrop, double efficiency) {
    using Lbl = MAS::WaveformLabel;
    (void)secondaryOutputVoltage;  // structural (2 outputs); the secondary winding voltage follows Vpri.
    double dutyCycle = primaryOutputVoltage / (inputVoltage + primaryOutputVoltage) * efficiency;
    if (dutyCycle >= 1) throw std::invalid_argument("analytical_isolated_buck_boost: duty cycle must be smaller than 1");
    double tOn = dutyCycle / switchingFrequency;

    double totalReflectedSecondaryCurrent = secondaryOutputCurrent / turnsRatio;
    double primaryCurrentPeakToPeak = (inputVoltage * primaryOutputVoltage) / (inputVoltage + primaryOutputVoltage) /
                                      (switchingFrequency * inductance);
    double primaryCurrentAverage = (primaryOutputCurrent + totalReflectedSecondaryCurrent) / (1.0 - dutyCycle);
    double primaryVoltaveMaximum = inputVoltage;
    double primaryVoltaveMinimum = -(primaryOutputVoltage + diodeVoltageDrop);
    double primaryVoltavePeaktoPeak = primaryVoltaveMaximum - primaryVoltaveMinimum;

    MAS::OperatingPoint operatingPoint;
    // Primary
    {
        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, primaryCurrentPeakToPeak, switchingFrequency, dutyCycle, primaryCurrentAverage, 0);
        MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, primaryVoltavePeaktoPeak, switchingFrequency, dutyCycle, 0, 0);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Primary"));
    }
    // Secondary (single isolated output)
    {
        double secondaryCurrentMaximum = (1 + dutyCycle) / (1 - dutyCycle) * secondaryOutputCurrent - secondaryOutputCurrent;
        double secondaryCurrentPeakToPeak = secondaryCurrentMaximum - 0;
        double secondaryVoltaveMaximum = (primaryOutputVoltage + diodeVoltageDrop) / turnsRatio - diodeVoltageDrop;
        double secondaryVoltaveMinimum = -inputVoltage / turnsRatio;
        double secondaryVoltavePeaktoPeak = secondaryVoltaveMaximum - secondaryVoltaveMinimum;

        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_PRIMARY, secondaryCurrentPeakToPeak, switchingFrequency, 1.0 - dutyCycle, secondaryOutputCurrent, 0, tOn);
        MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, secondaryVoltavePeaktoPeak, switchingFrequency, 1.0 - dutyCycle, 0, 0, tOn);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Secondary 0"));
    }
    return operatingPoint;
}

// Ported from MKF converter_models/ActiveClampForward.cpp:41 (process_operating_points_for_input_voltage).
// turnsRatios = [sec0, sec1, …] (no demag winding — the active clamp resets the core during 1−D). The primary
// sees +Vin for t1 and −Vclamp for t2, with Vclamp = D/(1−D)·Vin (volt-second balanced).
MAS::OperatingPoint analytical_active_clamp_forward(double inputVoltage,
                                                    const std::vector<double>& outputVoltages,
                                                    const std::vector<double>& outputCurrents,
                                                    const std::vector<double>& turnsRatios,
                                                    double switchingFrequency, double inductance,
                                                    double mainOutputInductance, double currentRippleRatio,
                                                    double dutyCycle, double diodeVoltageDrop) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        turnsRatios.size() != outputVoltages.size())
        throw std::invalid_argument("analytical_active_clamp_forward: need one turnsRatio per output");

    const double mainOutputCurrent = outputCurrents[0];
    const double mainOutputVoltage = outputVoltages[0];
    const double mainSecondaryTurnsRatio = turnsRatios[0];
    const double period = 1.0 / switchingFrequency;

    double t1 = period * (mainOutputVoltage + diodeVoltageDrop) / (inputVoltage / mainSecondaryTurnsRatio);
    if (t1 > period / 2)
        throw std::invalid_argument("analytical_active_clamp_forward: T1 cannot be larger than period/2, wrong topology configuration");
    double t2 = period - t1;
    double deadTime = 0;

    const double magnetizationCurrent = inputVoltage * t1 / inductance;
    double minimumPrimaryCurrent = -magnetizationCurrent / 2;
    double maximumPrimaryCurrent = magnetizationCurrent / 2;

    std::vector<double> minimumSecondaryCurrents, maximumSecondaryCurrents;
    for (size_t i = 0; i < outputVoltages.size(); ++i) {
        double outputCurrentRipple = currentRippleRatio * outputCurrents[i];
        double minSec = outputCurrents[i] - outputCurrentRipple / 2;
        double maxSec = outputCurrents[i] + outputCurrentRipple / 2;
        minimumSecondaryCurrents.push_back(minSec);
        maximumSecondaryCurrents.push_back(maxSec);
        minimumPrimaryCurrent += minSec / turnsRatios[i];
        maximumPrimaryCurrent += maxSec / turnsRatios[i];
    }

    if (minimumPrimaryCurrent < 0) {  // DCM
        double sqrtArg = 2 * mainOutputCurrent * mainOutputInductance * (mainOutputVoltage + diodeVoltageDrop) /
                         (switchingFrequency * (inputVoltage / mainSecondaryTurnsRatio - diodeVoltageDrop - mainOutputVoltage) *
                          (inputVoltage / mainSecondaryTurnsRatio));
        if (sqrtArg < 0)
            throw std::invalid_argument("analytical_active_clamp_forward: negative value under sqrt in DCM t1 calculation");
        t1 = std::sqrt(sqrtArg);
        if (t1 > period / 2)
            throw std::invalid_argument("analytical_active_clamp_forward: T1 cannot be larger than period/2, wrong topology configuration");
        t2 = t1 * inputVoltage / mainSecondaryTurnsRatio / (mainOutputVoltage + diodeVoltageDrop) - t1;
        deadTime = period - t1 - t2;
        minimumPrimaryCurrent = 0;
        maximumPrimaryCurrent = magnetizationCurrent;
        for (size_t i = 0; i < outputVoltages.size(); ++i) {
            double outputCurrentRipple = currentRippleRatio * outputCurrents[i];
            minimumSecondaryCurrents[i] = 0;
            maximumSecondaryCurrents[i] = outputCurrentRipple;
            minimumPrimaryCurrent += minimumSecondaryCurrents[i] / turnsRatios[i];
            maximumPrimaryCurrent += maximumSecondaryCurrents[i] / turnsRatios[i];
        }
    }

    const double denom = 1 - t1 * switchingFrequency;
    if (std::fabs(denom) < 1e-12)
        throw std::invalid_argument("analytical_active_clamp_forward: clamp voltage undefined when duty cycle equals 1");
    const double clampVoltage = t1 * switchingFrequency / denom * inputVoltage;

    const double maxPriV = inputVoltage;
    const double minPriV = -clampVoltage;
    const double minPriIT2 = -magnetizationCurrent / 2;
    const double maxPriIT2 = magnetizationCurrent / 2;

    MAS::OperatingPoint operatingPoint;
    // Primary ("First primary") — CUSTOM: ramp up during t1 (forward), magnetizing ramp down during reset.
    {
        MAS::Waveform currentWaveform, voltageWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(std::vector<double>{minimumPrimaryCurrent, maximumPrimaryCurrent, maxPriIT2, minPriIT2});
        currentWaveform.set_time(std::vector<double>{0, t1, t1, period});
        if (minimumPrimaryCurrent > 0) {  // CCM
            voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
            voltageWaveform.set_data(std::vector<double>{maxPriV, maxPriV, minPriV, minPriV, maxPriV});
            voltageWaveform.set_time(std::vector<double>{0, t1, t1, period, period});
        } else {  // DCM
            voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
            voltageWaveform.set_data(std::vector<double>{maxPriV, maxPriV, minPriV, minPriV, 0, 0, maxPriV});
            voltageWaveform.set_time(std::vector<double>{0, t1, t1, t1 + t2, t1 + t2, period, period});
        }
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "First primary"));
    }
    // Secondaries
    for (size_t i = 0; i < outputVoltages.size(); ++i) {
        double secondaryCurrentPeakToPeak = maximumSecondaryCurrents[i] - minimumSecondaryCurrents[i];
        double minimumSecondaryVoltage = -clampVoltage / turnsRatios[i];
        double maximumSecondaryVoltage = inputVoltage / turnsRatios[i];
        double secondaryVoltagePeakToPeak = maximumSecondaryVoltage - minimumSecondaryVoltage;
        double secondaryVoltageOffset = maximumSecondaryVoltage + minimumSecondaryVoltage;
        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_PRIMARY, secondaryCurrentPeakToPeak,
                                                            switchingFrequency, dutyCycle, minimumSecondaryCurrents[i], 0);
        MAS::Waveform voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR_WITH_DEADTIME, secondaryVoltagePeakToPeak,
                                                            switchingFrequency, dutyCycle, secondaryVoltageOffset, deadTime);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency, "Secondary " + std::to_string(i)));
    }
    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 4: the phase-shifted bridge family (PSFB, 3-level PSHB, AHB). These reuse
// the shared header-only kernel OpenMagnetics::PwmBridgeSolver (duty-cycle loss +
// three-segment sub-interval breakdown) exactly as the MKF bridge solvers do.
// ─────────────────────────────────────────────────────────────────────────────

// Shared sub-interval core for the two phase-shifted bridges (PSFB and 3-level
// PSHB). `Vbus` is the primary bridge swing (Vin for PSFB, Vin/2 for PSHB);
// `freewheelTau` is the time constant of the exponential primary-current decay
// during the freewheel sub-interval (diode-dynamic-resistance-limited for PSFB,
// MOSFET-RON-limited for PSHB — the only two places the two MKF bodies differ).
// Ported from the bodies of MKF PhaseShiftedFullBridge.cpp:359 and
// PhaseShiftedHalfBridge.cpp:318, which are line-for-line identical save for the
// `Vbus` and `freewheelTau` substitutions. Pushes Primary + (per output) the two
// center-tapped half-windings "Secondary i a" / "Secondary i b". The ZVS / ngspice
// diagnostics MKF also computes do not shape any winding waveform and are omitted
// (per the port brief). Throws on non-positive Fs / Lm / Lr / duty cycle.
static MAS::OperatingPoint pwm_bridge_phase_shifted_core(
    double Vbus, double Vo, double Io, double Fs,
    double Lm, double Lr, double Lo, double D_cmd, double freewheelTau,
    const std::vector<double>& turnsRatios, const char* who) {
    using Lbl = MAS::WaveformLabel;
    namespace PBS = OpenMagnetics::PwmBridgeSolver;

    if (Fs <= 0) throw std::invalid_argument(std::string(who) + ": switching frequency must be positive");
    if (Lm <= 0) throw std::invalid_argument(std::string(who) + ": magnetizing inductance must be positive");
    if (Lr <= 0) throw std::invalid_argument(std::string(who) + ": series (resonant) inductance must be positive; "
                                             "it was not provided");
    if (D_cmd <= 0) throw std::invalid_argument(std::string(who) + ": effective duty cycle is non-positive; provide a "
                                                "positive phase shift (no default duty substituted)");

    const double n = turnsRatios[0];

    // Sabate 1990 duty-cycle loss + effective duty (kernel calls — same as MKF).
    double dcl_duty = PBS::compute_duty_cycle_loss(Vbus, Lr, Io, n, Fs);
    double Deff = PBS::compute_effective_duty_cycle(D_cmd, dcl_duty);

    const double period = 1.0 / Fs;
    const double Thalf = period / 2.0;

    // Im integrates +Vbus during BOTH commutation and active intervals (total +Vbus
    // time per half-cycle = D_cmd·Thalf), so use D_cmd.
    double Im_peak = Vbus * D_cmd / (4.0 * Fs * Lm);

    // Output-inductor ripple ΔILo = Vo·(1−Deff)/(Fs·Lo) (Deff = active fraction the
    // secondary sees). Lo ≤ 0 ⇒ zero ripple (matches MKF's (Lo>0)?…:0 guard).
    double dILo = (Lo > 0) ? Vo * (1.0 - Deff) / (Fs * Lo) : 0.0;
    double ILo_min = Io - dILo / 2.0;
    double ILo_max = Io + dILo / 2.0;

    auto segs = PBS::build_first_half_cycle(period, Vbus, D_cmd, dcl_duty);

    double t_dcl    = std::min(dcl_duty * Thalf, Thalf);
    double t_active = std::min(D_cmd * Thalf,    Thalf);
    if (t_active < t_dcl) t_active = t_dcl;
    const double Vsec_pk = Vbus / n;

    const int N_samples = 256;
    const double dt = Thalf / N_samples;
    const int totalSamples = 2 * N_samples + 1;

    std::vector<double> time_full(totalSamples), Vpri_full(totalSamples),
        Ipri_full(totalSamples), Im_full(totalSamples), ILo_full(totalSamples), VLo_full(totalSamples);

    // Antisymmetric-magnetizing initial condition: Im(0) = −Im_peak.
    const double Im0 = -Im_peak;
    std::vector<double> Im_at_seg_start(segs.size());
    double Im_running = Im0;
    for (size_t si = 0; si < segs.size(); ++si) {
        Im_at_seg_start[si] = Im_running;
        double dur = segs[si].t_end - segs[si].t_start;
        Im_running += segs[si].Vp * dur / Lm;
    }
    auto seg_index_at = [&](double t) -> size_t {
        for (size_t si = 0; si < segs.size(); ++si)
            if (t < segs[si].t_end - 1e-15) return si;
        return segs.size() - 1;
    };

    for (int k = 0; k <= N_samples; ++k) {
        double t = k * dt;
        time_full[k] = t;
        size_t si = seg_index_at(t);
        const auto& s = segs[si];
        double Vp = s.Vp;
        Vpri_full[k] = Vp;

        Im_full[k] = Im_at_seg_start[si] + Vp * (t - s.t_start) / Lm;
        if (Im_full[k] >  Im_peak) Im_full[k] =  Im_peak;
        if (Im_full[k] < -Im_peak) Im_full[k] = -Im_peak;

        // Output-inductor: trough at t_dcl, peak at t_active (exact piecewise-linear).
        double slope_neg = (Lo > 0) ? -Vo / Lo : 0.0;
        double slope_pos = (Lo > 0) ?  (Vsec_pk - Vo) / Lo : 0.0;
        double ILo_at_zero = ILo_min - slope_neg * t_dcl;
        if (t < t_dcl) {
            ILo_full[k] = ILo_at_zero + slope_neg * t;
            VLo_full[k] = -Vo;
        } else if (t < t_active) {
            ILo_full[k] = ILo_min + slope_pos * (t - t_dcl);
            VLo_full[k] = Vsec_pk - Vo;
        } else {
            ILo_full[k] = ILo_max + slope_neg * (t - t_active);
            VLo_full[k] = -Vo;
        }

        // Primary current: reflected output-inductor current + commutation ramp +
        // magnetizing; freewheel uses the exponential decay (freewheelTau).
        if (s.is_commutation && (s.t_end - s.t_start) > 1e-15) {
            double cfrac = (t - s.t_start) / (s.t_end - s.t_start);
            double ipri_start = -ILo_max / n;
            double ipri_end   =  ILo_min / n;
            Ipri_full[k] = ipri_start + (ipri_end - ipri_start) * cfrac + Im_full[k];
        } else if (s.is_active) {
            Ipri_full[k] = ILo_full[k] / n + Im_full[k];
        } else {
            double t_fw = t - t_active;
            double ipri_start_fw = ILo_max / n + Im_peak;
            double ipri_end_fw   = Im_peak;
            double f = 1.0 - std::exp(-t_fw / freewheelTau);   // 0 at t=0, →1 as t→∞
            Ipri_full[k] = ipri_start_fw + (ipri_end_fw - ipri_start_fw) * f;
        }
    }

    // Negative half-cycle by antisymmetry; the output inductor is unipolar (rectified).
    for (int k = 1; k <= N_samples; ++k) {
        time_full[N_samples + k] = Thalf + k * dt;
        Ipri_full[N_samples + k] = -Ipri_full[k];
        Vpri_full[N_samples + k] = -Vpri_full[k];
        Im_full[N_samples + k]   = -Im_full[k];
        ILo_full[N_samples + k]  = ILo_full[k];
        VLo_full[N_samples + k]  = VLo_full[k];
    }

    MAS::OperatingPoint operatingPoint;
    // ---- Primary winding excitation ----
    {
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(Ipri_full);
        currentWaveform.set_time(time_full);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(Vpri_full);
        voltageWaveform.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, Fs, "Primary"));
    }
    // ---- Secondary center-tapped half-windings (one pair per output) ----
    // Each half-winding conducts the (reflected) output-inductor current on alternate
    // half-cycles; the opposite half-cycle reverse-blocks (zero current, −Vsec_pk).
    for (size_t secIdx = 0; secIdx < turnsRatios.size(); ++secIdx) {
        double ni = turnsRatios[secIdx];
        if (ni <= 0) continue;
        double VsecPk = Vbus / ni;
        std::vector<double> v1(totalSamples), i1(totalSamples), v2(totalSamples), i2(totalSamples);
        for (int k = 0; k < totalSamples; ++k) {
            bool positive_half = (k <= N_samples);
            double vpri = Vpri_full[k];
            double iLo_k = ILo_full[k];
            if (positive_half) {
                v1[k] = (vpri > 0) ? VsecPk : (vpri < 0 ? -VsecPk : 0.0);
                i1[k] = (vpri > 0) ? iLo_k : 0.0;
                v2[k] = -v1[k]; i2[k] = 0.0;
            } else {
                v2[k] = (vpri < 0) ? VsecPk : (vpri > 0 ? -VsecPk : 0.0);
                i2[k] = (vpri < 0) ? iLo_k : 0.0;
                v1[k] = -v2[k]; i1[k] = 0.0;
            }
        }
        MAS::Waveform i1Wfm; i1Wfm.set_ancillary_label(Lbl::CUSTOM); i1Wfm.set_data(i1); i1Wfm.set_time(time_full);
        MAS::Waveform v1Wfm; v1Wfm.set_ancillary_label(Lbl::CUSTOM); v1Wfm.set_data(v1); v1Wfm.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(i1Wfm, v1Wfm, Fs, "Secondary " + std::to_string(secIdx) + "a"));
        MAS::Waveform i2Wfm; i2Wfm.set_ancillary_label(Lbl::CUSTOM); i2Wfm.set_data(i2); i2Wfm.set_time(time_full);
        MAS::Waveform v2Wfm; v2Wfm.set_ancillary_label(Lbl::CUSTOM); v2Wfm.set_data(v2); v2Wfm.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(i2Wfm, v2Wfm, Fs, "Secondary " + std::to_string(secIdx) + "b"));
    }
    return operatingPoint;
}

// Ported from MKF converter_models/PhaseShiftedFullBridge.cpp:359
// (Psfb::process_operating_point_for_input_voltage). Center-tapped rectifier.
MAS::OperatingPoint analytical_psfb(double inputVoltage,
                                    const std::vector<double>& outputVoltages,
                                    const std::vector<double>& outputCurrents,
                                    const std::vector<double>& turnsRatios,
                                    double switchingFrequency, double magnetizingInductance,
                                    double seriesInductance, double outputInductance,
                                    double phaseShiftDegrees, double diodeVoltageDrop) {
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_psfb: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    if (inputVoltage <= 0) throw std::invalid_argument("analytical_psfb: input voltage must be > 0");
    if (turnsRatios[0] <= 0) throw std::invalid_argument("analytical_psfb: primary turns ratio must be > 0");
    (void)diodeVoltageDrop;  // not used in PSFB waveform generation (only in the design-side Vo/turns-ratio)

    double D_cmd = std::abs(phaseShiftDegrees) / 180.0;   // effective commanded duty from phase shift
    // PSFB freewheel: diode-dynamic-resistance-limited primary decay, τ = Lr/(n²·R_d),
    // R_d = DIODE_RS = 5 mΩ (matches MKF's compute_diode_drop_at_current series term).
    constexpr double DIODE_RS = 0.005;
    double R_eff = turnsRatios[0] * turnsRatios[0] * DIODE_RS;
    double freewheelTau = (seriesInductance > 0 && R_eff > 0) ? seriesInductance / R_eff : 1.0;
    return pwm_bridge_phase_shifted_core(inputVoltage, outputVoltages[0], outputCurrents[0],
                                         switchingFrequency, magnetizingInductance, seriesInductance,
                                         outputInductance, D_cmd, freewheelTau, turnsRatios, "analytical_psfb");
}

// Ported from MKF converter_models/PhaseShiftedHalfBridge.cpp:318
// (Pshb::process_operating_point_for_input_voltage). 3-level NPC bridge: the primary
// swings ±Vin/2 (BRIDGE_VOLTAGE_FACTOR = 0.5). Center-tapped rectifier.
MAS::OperatingPoint analytical_pshb(double inputVoltage,
                                    const std::vector<double>& outputVoltages,
                                    const std::vector<double>& outputCurrents,
                                    const std::vector<double>& turnsRatios,
                                    double switchingFrequency, double magnetizingInductance,
                                    double seriesInductance, double outputInductance,
                                    double phaseShiftDegrees, double diodeVoltageDrop) {
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_pshb: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    if (inputVoltage <= 0) throw std::invalid_argument("analytical_pshb: input voltage must be > 0");
    if (turnsRatios[0] <= 0) throw std::invalid_argument("analytical_pshb: primary turns ratio must be > 0");
    (void)diodeVoltageDrop;  // not used in PSHB waveform generation (only in the design-side Vo/turns-ratio)

    constexpr double BRIDGE_VOLTAGE_FACTOR = 0.5;
    double Vhb = inputVoltage * BRIDGE_VOLTAGE_FACTOR;
    double D_cmd = std::abs(phaseShiftDegrees) / 180.0;
    // PSHB freewheel: NPC inner+outer switch path shorts the primary, τ = Lr/(2·RON),
    // RON = 0.01 Ω (matches MKF's SPICE SW1 model).
    constexpr double RON_PER_SWITCH = 0.01;
    double freewheelTau = (seriesInductance > 0) ? seriesInductance / (2.0 * RON_PER_SWITCH) : 1.0;
    return pwm_bridge_phase_shifted_core(Vhb, outputVoltages[0], outputCurrents[0],
                                         switchingFrequency, magnetizingInductance, seriesInductance,
                                         outputInductance, D_cmd, freewheelTau, turnsRatios, "analytical_pshb");
}

// Ported from MKF converter_models/AsymmetricHalfBridge.cpp:460
// (AsymmetricHalfBridge::process_operating_point_for_input_voltage), CENTER_TAPPED
// (the default) standard path. The dedicated AHB-Flyback (V5) storage path and the
// FULL_BRIDGE / CURRENT_DOUBLER rectifier variants are NOT ported (rectifier fixed to
// center-tapped per the port brief). The output inductance is derived from
// `currentRippleRatio` via the same compute_lo_min sizing MKF uses for raw calls
// (Lo = Vo·(1−2D(1−D))/(ripple·Io·Fs)); note that, by the AHB secondary-voltage
// geometry, the resulting *physical* output-inductor ripple is smaller than ripple·Io.
MAS::OperatingPoint analytical_asymmetric_half_bridge(double inputVoltage,
                                                      const std::vector<double>& outputVoltages,
                                                      const std::vector<double>& outputCurrents,
                                                      const std::vector<double>& turnsRatios,
                                                      double switchingFrequency, double magnetizingInductance,
                                                      double dutyCycle, double currentRippleRatio,
                                                      double diodeVoltageDrop) {
    using Lbl = MAS::WaveformLabel;
    // ---- Input validation (no defaults, no fallbacks: throw loud, mirroring MKF) ----
    if (!(inputVoltage > 0.0) || !std::isfinite(inputVoltage))
        throw std::invalid_argument("analytical_asymmetric_half_bridge: inputVoltage must be > 0");
    if (turnsRatios.empty())
        throw std::invalid_argument("analytical_asymmetric_half_bridge: turnsRatios must contain at least one entry");
    for (double tr : turnsRatios)
        if (!(tr > 0.0) || !std::isfinite(tr))
            throw std::invalid_argument("analytical_asymmetric_half_bridge: all turnsRatios must be > 0");
    if (!(magnetizingInductance > 0.0) || !std::isfinite(magnetizingInductance))
        throw std::invalid_argument("analytical_asymmetric_half_bridge: magnetizingInductance must be > 0");
    const size_t numOutputs = outputVoltages.size();
    if (numOutputs == 0 || outputCurrents.size() != numOutputs || turnsRatios.size() != numOutputs)
        throw std::invalid_argument("analytical_asymmetric_half_bridge: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double D = dutyCycle;
    if (!(D > 0.0 && D < 1.0))
        throw std::invalid_argument("analytical_asymmetric_half_bridge: dutyCycle must lie in (0, 1)");
    const double fsw = switchingFrequency;
    if (!(fsw > 0.0))
        throw std::invalid_argument("analytical_asymmetric_half_bridge: switchingFrequency must be > 0");
    (void)diodeVoltageDrop;  // not used in AHB waveform generation (only in the design-side turns-ratio)

    const double Vin = inputVoltage;
    // Multi-output: primary sees the total reflected power (project to output #0).
    const double Vo = outputVoltages[0];
    double IoEff = outputCurrents[0];
    double totalPower = 0.0;
    for (size_t k = 0; k < numOutputs; ++k) totalPower += outputVoltages[k] * outputCurrents[k];
    if (numOutputs > 1 && Vo > 0.0) IoEff = totalPower / Vo;
    const double Io = IoEff;
    const double n = turnsRatios[0];
    const double Lm = magnetizingInductance;
    const double Tsw = 1.0 / fsw;
    const double tA = D * Tsw;
    const double tC = (1.0 - D) * Tsw;

    const double Vpri_pos = +(1.0 - D) * Vin;
    const double Vpri_neg = -D * Vin;
    const double Vsec_pos = +(1.0 - D) * Vin / n;
    const double Vsec_neg = +D * Vin / n;

    const double Io_per_inductor = Io;  // center-tapped: single output inductor carries full Io

    // AHB DC magnetizing bias: the series Cb forces mean(i_pri)=0, so i_Lm carries the
    // offset mean(i_Lm) = (Io/n)·(1−2D) (Imbertson-Mohan 1993; TI SLUP223 fig.3).
    const double iLm_dc_bias = (Io_per_inductor / n) * (1.0 - 2.0 * D);
    const double dILm_pp = (1.0 - D) * Vin * D * Tsw / Lm;
    const double Im0 = iLm_dc_bias - dILm_pp / 2.0;

    // Output inductor sized from the requested ripple via compute_lo_min, then the
    // physical interval-A ripple recovered from the secondary-voltage geometry.
    const double dILo_target = std::max(currentRippleRatio * Io_per_inductor, 1e-6);
    if (!(Vo > 0.0)) throw std::invalid_argument("analytical_asymmetric_half_bridge: outputVoltage must be > 0");
    const double Lo1 = Vo * (1.0 - 2.0 * D * (1.0 - D)) / (dILo_target * fsw);   // compute_lo_min
    const double dILo1_pp = (Vsec_pos - Vo) * tA / Lo1;
    const double ILo1_min = Io_per_inductor - dILo1_pp / 2.0;
    const double ILo1_max = Io_per_inductor + dILo1_pp / 2.0;

    const int N = 128;
    std::vector<double> time, vPri, iPri, vSec_a, iSec_a, vSec_b, iSec_b;
    const size_t cap = 2 * N + 3;
    for (auto* v : {&time, &vPri, &iPri, &vSec_a, &iSec_a, &vSec_b, &iSec_b}) v->reserve(cap);

    // emit one sample to the primary + center-tapped half-winding arrays.
    auto emit = [&](double t, double v_pri_k, double i_lm_k, double i_lo1_k) {
        time.push_back(t);
        vPri.push_back(v_pri_k);
        const bool inA = (v_pri_k > 0.0);
        const bool inC = (v_pri_k < 0.0);
        iPri.push_back(inA ? (i_lo1_k / n + i_lm_k) : (inC ? (-i_lo1_k / n + i_lm_k) : i_lm_k));
        if (inA) {
            vSec_a.push_back(+Vsec_pos); iSec_a.push_back(i_lo1_k);
            vSec_b.push_back(-Vsec_pos); iSec_b.push_back(0.0);
        } else if (inC) {
            vSec_a.push_back(-Vsec_neg); iSec_a.push_back(0.0);
            vSec_b.push_back(+Vsec_neg); iSec_b.push_back(i_lo1_k);
        } else {
            vSec_a.push_back(0.0); iSec_a.push_back(0.0);
            vSec_b.push_back(0.0); iSec_b.push_back(0.0);
        }
    };

    // Interval A (Q1 ON): Lo1 charges (+slope).
    for (int k = 0; k <= N; ++k) {
        const double frac = static_cast<double>(k) / N;
        emit(frac * tA, Vpri_pos, Im0 + frac * dILm_pp, ILo1_min + frac * dILo1_pp);
    }
    // Discontinuity duplicate sample (post-jump v_pri).
    emit(tA, Vpri_neg, Im0 + dILm_pp, ILo1_max);
    // Interval C (Q2 ON): Lo1 discharges (−slope); i_Lm ramps back to Im0.
    for (int k = 1; k <= N; ++k) {
        const double frac = static_cast<double>(k) / N;
        emit(tA + frac * tC, Vpri_neg, (Im0 + dILm_pp) - frac * dILm_pp, ILo1_max - frac * dILo1_pp);
    }

    auto wfm = [](const std::vector<double>& d, const std::vector<double>& t) {
        MAS::Waveform w;
        w.set_ancillary_label(Lbl::CUSTOM);
        w.set_data(d);
        w.set_time(t);
        return w;
    };

    MAS::OperatingPoint operatingPoint;
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(wfm(iPri, time), wfm(vPri, time), fsw, "Primary"));
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(wfm(iSec_a, time), wfm(vSec_a, time), fsw, "Secondary 0a"));
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(wfm(iSec_b, time), wfm(vSec_b, time), fsw, "Secondary 0b"));

    // V6 multi-output: replace the center-tapped pair with per-output load-share
    // secondaries (share_k = Vo_k·Io_k/ΣVo·Io; i_sec_k = share_k·n_k·iPri).
    if (numOutputs > 1 && totalPower > 0.0) {
        auto& exc = operatingPoint.get_mutable_excitations_per_winding();
        exc.pop_back(); exc.pop_back();  // drop "Secondary 0a"/"0b"
        for (size_t k = 0; k < numOutputs; ++k) {
            const double Vo_k = outputVoltages[k];
            const double Io_k = outputCurrents[k];
            const double n_k = turnsRatios[k];
            if (n_k <= 0.0) continue;
            const double share = (Vo_k * Io_k) / totalPower;
            std::vector<double> iSec_k(iPri.size()), vSec_k(vPri.size());
            for (size_t i = 0; i < iPri.size(); ++i) {
                iSec_k[i] = share * n_k * iPri[i];
                vSec_k[i] = (vPri[i] > 0.0) ? +Vo_k : -Vo_k;
            }
            exc.push_back(WP::complete_excitation(wfm(iSec_k, time), wfm(vSec_k, time), fsw,
                                                  "Secondary " + std::to_string(k)));
        }
    }
    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 4: Dual Active Bridge (DAB) — triple-phase-shift (SPS/EPS/DPS/TPS).
// The closed-form 8-segment sub-interval propagator below is a faithful port of
// MKF Dab.cpp:156-349: both bridge voltages Vab(θ)/Vcd(θ) are piecewise-constant,
// so the tank current iL(θ) and magnetizing current Im(θ) are exactly piecewise-
// linear; iL(0)/Im(0) are fixed by half-wave antisymmetry x(π)=−x(0). The MKF
// ZVS/modulation-type diagnostics do not shape a winding waveform and are omitted.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

double dab_Vab_at(double theta, double V1, double D1) {
    theta = std::fmod(theta, 2.0 * M_PI);
    if (theta < 0) theta += 2.0 * M_PI;
    if (theta < D1)        return 0.0;
    if (theta < M_PI)      return  V1;
    if (theta < M_PI + D1) return 0.0;
    return -V1;
}
double dab_Vcd_at(double theta, double V2, double D2, double D3) {
    double ts = std::fmod(theta - D3, 2.0 * M_PI);
    if (ts < 0) ts += 2.0 * M_PI;
    if (ts < D2)        return 0.0;
    if (ts < M_PI)      return  V2;
    if (ts < M_PI + D2) return 0.0;
    return -V2;
}
struct DabSubInterval { double theta_start, theta_end, Vab, Vcd; };

std::vector<double> dab_boundary_angles(double D1, double D2, double D3) {
    auto wrap = [](double a) { a = std::fmod(a, 2.0 * M_PI); if (a < 0) a += 2.0 * M_PI; return a; };
    std::vector<double> raw = {0.0, wrap(D1), wrap(M_PI), wrap(M_PI + D1),
                               wrap(D3), wrap(D3 + D2), wrap(D3 + M_PI), wrap(D3 + M_PI + D2)};
    std::sort(raw.begin(), raw.end());
    std::vector<double> out;
    out.reserve(raw.size() + 1);
    constexpr double EPS = 1e-9;
    for (double a : raw) if (out.empty() || a - out.back() > EPS) out.push_back(a);
    if (out.empty() || out.back() < 2.0 * M_PI - EPS) out.push_back(2.0 * M_PI);
    return out;
}
std::vector<DabSubInterval> dab_build_subintervals(double V1, double V2, double D1, double D2, double D3) {
    auto angles = dab_boundary_angles(D1, D2, D3);
    std::vector<DabSubInterval> segs;
    segs.reserve(angles.size());
    for (size_t k = 0; k + 1 < angles.size(); ++k) {
        double a0 = angles[k], a1 = angles[k + 1], mid = 0.5 * (a0 + a1);
        segs.push_back({a0, a1, dab_Vab_at(mid, V1, D1), dab_Vcd_at(mid, V2, D2, D3)});
    }
    return segs;
}
// iL(0)/Im(0) from half-wave antisymmetry x(π) = −x(0): x(0) = −½·∫₀^π (dx/dt) dt.
void dab_initial_conditions(const std::vector<DabSubInterval>& segs, double N, double L, double Lm,
                            double Fs, double& iL0, double& Im0) {
    double inv_iL = 1.0 / (L * 2.0 * M_PI * Fs);
    double inv_Im = 1.0 / (Lm * 2.0 * M_PI * Fs);
    double sum_iL = 0.0, sum_Im = 0.0;
    for (const auto& s : segs) {
        if (s.theta_start >= M_PI) break;
        double end = std::min(s.theta_end, M_PI);
        double dtheta = end - s.theta_start;
        if (dtheta <= 0) continue;
        double vL = s.Vab - N * s.Vcd;
        sum_iL += vL * inv_iL * dtheta;
        sum_Im += s.Vab * inv_Im * dtheta;
    }
    iL0 = -0.5 * sum_iL;
    Im0 = -0.5 * sum_Im;
}
void dab_propagate_period(const std::vector<DabSubInterval>& segs, double N, double L, double Lm, double Fs,
                          double iL0, double Im0, std::vector<double>& bt, std::vector<double>& bi,
                          std::vector<double>& bm) {
    double inv_iL = 1.0 / (L * 2.0 * M_PI * Fs);
    double inv_Im = 1.0 / (Lm * 2.0 * M_PI * Fs);
    bt.clear(); bi.clear(); bm.clear();
    bt.reserve(segs.size() + 1); bi.reserve(segs.size() + 1); bm.reserve(segs.size() + 1);
    double iL = iL0, Im = Im0;
    bt.push_back(0.0); bi.push_back(iL); bm.push_back(Im);
    for (const auto& s : segs) {
        double dtheta = s.theta_end - s.theta_start;
        double vL = s.Vab - N * s.Vcd;
        iL += vL * inv_iL * dtheta;
        Im += s.Vab * inv_Im * dtheta;
        bt.push_back(s.theta_end); bi.push_back(iL); bm.push_back(Im);
    }
}
double dab_sample(double theta, const std::vector<double>& bt, const std::vector<double>& by) {
    if (bt.empty()) return 0.0;
    if (theta <= bt.front()) return by.front();
    if (theta >= bt.back())  return by.back();
    auto it = std::upper_bound(bt.begin(), bt.end(), theta);
    size_t hi = static_cast<size_t>(it - bt.begin()), lo = hi - 1;
    double t = (theta - bt[lo]) / (bt[hi] - bt[lo]);
    return by[lo] + t * (by[hi] - by[lo]);
}

}  // anonymous namespace

// Ported from MKF converter_models/Dab.cpp:701 (process_operating_point_for_input_voltage).
MAS::OperatingPoint analytical_dab(double inputVoltage,
                                   const std::vector<double>& outputVoltages,
                                   const std::vector<double>& outputCurrents,
                                   const std::vector<double>& turnsRatios,
                                   double switchingFrequency, double magnetizingInductance,
                                   double seriesInductance,
                                   double innerPhaseShiftD1Degrees,
                                   double innerPhaseShiftD2Degrees,
                                   double outerPhaseShiftD3Degrees) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_dab: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double Fs = switchingFrequency;
    if (Fs <= 0) throw std::invalid_argument("analytical_dab: switching frequency must be positive");
    const double Lm = magnetizingInductance;
    if (Lm <= 0) throw std::invalid_argument("analytical_dab: magnetizing inductance must be positive");
    const double L = seriesInductance;
    if (L <= 0) throw std::invalid_argument("analytical_dab: series inductance must be positive");

    const double V1 = inputVoltage;
    const double N_main = turnsRatios[0];
    const double V2_main = outputVoltages[0];
    const double D1_rad = innerPhaseShiftD1Degrees * M_PI / 180.0;
    const double D2_rad = innerPhaseShiftD2Degrees * M_PI / 180.0;
    const double D3_rad = outerPhaseShiftD3Degrees * M_PI / 180.0;

    const double period = 1.0 / Fs;
    const double Thalf = period / 2.0;

    auto segs = dab_build_subintervals(V1, V2_main, D1_rad, D2_rad, D3_rad);
    double iL_start = 0.0, Im_start = 0.0;
    dab_initial_conditions(segs, N_main, L, Lm, Fs, iL_start, Im_start);
    std::vector<double> bnd_theta, bnd_iL, bnd_Im;
    dab_propagate_period(segs, N_main, L, Lm, Fs, iL_start, Im_start, bnd_theta, bnd_iL, bnd_Im);

    const int N_samples = 256;
    const int totalSamples = 2 * N_samples + 1;
    const double dt = Thalf / N_samples;
    const double angle_per_time = 2.0 * M_PI * Fs;

    std::vector<double> time_full(totalSamples), iL_full(totalSamples),
        Vab_full(totalSamples), Im_full(totalSamples);
    for (int k = 0; k < totalSamples; ++k) {
        double t = k * dt;
        double theta = std::fmod(t * angle_per_time, 2.0 * M_PI);
        if (theta < 0) theta += 2.0 * M_PI;
        time_full[k] = t;
        Vab_full[k] = dab_Vab_at(theta, V1, D1_rad);
        iL_full[k]  = dab_sample(theta, bnd_theta, bnd_iL);
        Im_full[k]  = dab_sample(theta, bnd_theta, bnd_Im);
    }

    MAS::OperatingPoint operatingPoint;
    // Primary: total current = tank iL + magnetizing Im; voltage = Vab (primary bridge square wave).
    {
        std::vector<double> I_primary(totalSamples);
        for (int k = 0; k < totalSamples; ++k) I_primary[k] = iL_full[k] + Im_full[k];
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(I_primary);
        currentWaveform.set_time(time_full);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::BIPOLAR_RECTANGULAR);
        voltageWaveform.set_data(Vab_full);
        voltageWaveform.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, Fs, "Primary"));
    }
    // Secondaries: load-share projection of the tank current onto each output (matches MKF).
    double total_g = 0.0;
    for (size_t i = 0; i < turnsRatios.size(); ++i)
        if (outputVoltages[i] > 0 && outputCurrents[i] > 0) total_g += outputCurrents[i] / outputVoltages[i];
    if (total_g <= 0) total_g = 1.0;
    for (size_t secIdx = 0; secIdx < turnsRatios.size(); ++secIdx) {
        double n_i = turnsRatios[secIdx];
        if (n_i <= 0) continue;
        double V2_i = outputVoltages[secIdx];
        double Io_i = outputCurrents[secIdx];
        double share = (V2_i > 0 && Io_i > 0) ? (Io_i / V2_i) / total_g : (secIdx == 0 ? 1.0 : 0.0);
        std::vector<double> iSecData(totalSamples), vSecData(totalSamples);
        for (int k = 0; k < totalSamples; ++k) {
            iSecData[k] = n_i * iL_full[k] * share;
            double theta = time_full[k] * angle_per_time;   // Vcd_at wraps internally (matches MKF)
            vSecData[k] = dab_Vcd_at(theta, V2_i, D2_rad, D3_rad);
        }
        MAS::Waveform secCurrentWfm;
        secCurrentWfm.set_ancillary_label(Lbl::CUSTOM);
        secCurrentWfm.set_data(iSecData);
        secCurrentWfm.set_time(time_full);
        MAS::Waveform secVoltageWfm;
        secVoltageWfm.set_ancillary_label(Lbl::BIPOLAR_RECTANGULAR);
        secVoltageWfm.set_data(vSecData);
        secVoltageWfm.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(secCurrentWfm, secVoltageWfm, Fs, "Secondary " + std::to_string(secIdx)));
    }
    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 5: resonant converter family (First-Harmonic Approximation).
// ─────────────────────────────────────────────────────────────────────────────

// Ported from MKF converter_models/Src.cpp:338 (process_operating_point_for_input_voltage).
MAS::OperatingPoint analytical_src(double inputVoltage,
                                   const std::vector<double>& outputVoltages,
                                   const std::vector<double>& outputCurrents,
                                   const std::vector<double>& turnsRatios,
                                   double switchingFrequency,
                                   double resonantInductance, double resonantCapacitance,
                                   double bridgeVoltageFactor, SrcRectifier rectifier) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_src: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double fsw = switchingFrequency;
    if (fsw <= 0) throw std::invalid_argument("analytical_src: switching frequency must be > 0");
    const double Lr = resonantInductance, Cr = resonantCapacitance;
    if (Lr <= 0 || Cr <= 0) throw std::invalid_argument("analytical_src: resonant Lr/Cr must be > 0");
    const double k_bridge = bridgeVoltageFactor;

    const double fr = 1.0 / (2.0 * M_PI * std::sqrt(Lr * Cr));
    const double Lambda = fsw / fr;
    // At-resonance (Λ = 1, X = 0, φ = 0) is the canonical SRC operating point; allow a small tolerance
    // so fsw == fr (with fr recomputed from Lr·Cr rounding) is not misread as below-resonance. Genuine
    // below-resonance (capacitive / hard-switching) still throws.
    if (Lambda < 1.0 - 1e-6)
        throw std::invalid_argument("analytical_src: below-resonance operation (Lambda = " +
                                    std::to_string(Lambda) + " < 1) not supported (capacitive / hard-switching)");

    const size_t nOutputs = outputVoltages.size();
    const double n_main = turnsRatios[0];
    if (n_main <= 0) throw std::invalid_argument("analytical_src: turns ratio must be > 0");
    const double Rload = outputVoltages[0] / outputCurrents[0];
    const double Rac = (8.0 * n_main * n_main) / (M_PI * M_PI) * Rload;   // canonical FHA Rac

    const double w = 2.0 * M_PI * fsw;
    const double XLr = w * Lr;
    const double XCr = 1.0 / (w * Cr);
    const double X = XLr - XCr;                  // > 0 above resonance (inductive)
    const double Zin = std::sqrt(Rac * Rac + X * X);

    const double Vbridge_pk_fund = (4.0 / M_PI) * k_bridge * inputVoltage;
    const double ILr_pk = Vbridge_pk_fund / Zin;
    const double phi = std::atan2(X, Rac);
    const double Vbridge_lvl = k_bridge * inputVoltage;

    const int N = 256;
    const int totalSamples = N + 1;              // closed period (last == first)
    const double period = 1.0 / fsw;
    const double dt = period / N;
    std::vector<double> time_full(totalSamples), ILr_full(totalSamples), Vpri_full(totalSamples);
    for (int k = 0; k < totalSamples; ++k) {
        double t = k * dt;
        double theta = w * t;
        time_full[k] = t;
        ILr_full[k] = ILr_pk * std::sin(theta - phi);
        double thetaMod = std::fmod(theta, 2.0 * M_PI);
        if (thetaMod < 0) thetaMod += 2.0 * M_PI;
        Vpri_full[k] = (thetaMod < M_PI) ? +Vbridge_lvl : -Vbridge_lvl;
    }

    MAS::OperatingPoint operatingPoint;
    // Primary: sinusoidal tank current, square bridge voltage.
    {
        MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(ILr_full); iW.set_time(time_full);
        MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(Vpri_full); vW.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iW, vW, fsw, "Primary"));
    }

    double total_g = 0.0;
    for (size_t i = 0; i < nOutputs; ++i)
        if (outputVoltages[i] > 0 && outputCurrents[i] > 0) total_g += outputCurrents[i] / outputVoltages[i];
    if (total_g <= 0) total_g = 1.0;

    for (size_t outputIdx = 0; outputIdx < nOutputs; ++outputIdx) {
        double n_i = turnsRatios[outputIdx];
        if (n_i <= 0) n_i = 1.0;
        const double Vout_i = outputVoltages[outputIdx];
        const double Iout_i = outputCurrents[outputIdx];
        const double share = (Vout_i > 0 && Iout_i > 0) ? (Iout_i / Vout_i) / total_g
                                                        : (outputIdx == 0 ? 1.0 : 0.0);

        if (rectifier == SrcRectifier::CENTER_TAPPED) {
            for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
                std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
                for (int k = 0; k < totalSamples; ++k) {
                    double iPri = ILr_full[k];
                    double i_share = iPri * n_i * share;
                    double i_half = (halfIdx == 0) ? std::max(0.0, +i_share) : std::max(0.0, -i_share);
                    double v_half = (halfIdx == 0) ? ((iPri >= 0.0) ? +Vout_i : -Vout_i)
                                                   : ((iPri >= 0.0) ? -Vout_i : +Vout_i);
                    iSecData[k] = i_half;
                    vSecData[k] = v_half;
                }
                MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
                MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
                operatingPoint.get_mutable_excitations_per_winding().push_back(
                    WP::complete_excitation(iW, vW, fsw, "Secondary " + std::to_string(outputIdx) +
                                                         " Half " + std::to_string(halfIdx + 1)));
            }
        } else {  // FULL_BRIDGE: single secondary winding sees the full sinusoid.
            std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
            for (int k = 0; k < totalSamples; ++k) {
                iSecData[k] = ILr_full[k] * n_i * share;
                vSecData[k] = (ILr_full[k] >= 0.0) ? +Vout_i : -Vout_i;
            }
            MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
            MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(iW, vW, fsw, "Secondary " + std::to_string(outputIdx)));
        }
    }
    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// LLC resonant converter — Runo Nielsen TIME-DOMAIN analysis (event-driven sub-state propagator).
// Every helper below is transcribed FAITHFULLY from MKF converter_models/Llc.cpp:315-783 (the anonymous
// TDA machinery) — signs, indices, the Newton Jacobian, the sub-state event conditions, the multi-start
// seeds and the singularity/sanity guards are byte-for-byte the MKF numerics (MKF file:line cited per
// function). MKF's diagnostic-only pieces (classify_mode, ZVS/LIP members) are omitted — they shape no
// winding waveform.
//
// State x = [I_Ls, I_L, V_C]: resonant-inductor current, magnetizing current, resonant-cap voltage.
// Three linear sub-states cover Nielsen's six modes:
//   A_POS: +Vo diode conducting   dILs/dt=(Vi−Vc−Vo)/Ls  dIL/dt=+Vo/L   dVc/dt=ILs/C
//   A_NEG: −Vo diode conducting   dILs/dt=(Vi−Vc+Vo)/Ls  dIL/dt=−Vo/L   dVc/dt=ILs/C
//   B_FW : freewheeling (ILs≡IL)  dILs/dt=(Vi−Vc)/(Ls+L)                dVc/dt=ILs/C
// ─────────────────────────────────────────────────────────────────────────────
namespace {

enum class LlcSubState { A_POS, A_NEG, B_FW };  // MKF Llc.cpp:315

struct LlcStateVector { double iLs; double iL; double vC; };  // MKF Llc.cpp:317

struct LlcSubStateSegment {  // MKF Llc.cpp:323
    LlcSubState state;
    double t_start;
    double t_end;
    LlcStateVector x_start;
    LlcStateVector x_end;
};

// MKF Llc.cpp:333 substate_freq
void llc_substate_freq(LlcSubState s, double Ls, double L, double C, double& w, double& Z) {
    if (s == LlcSubState::B_FW) {
        double L_eff = Ls + L;  // freewheeling: Lm in series with Ls (diodes off)
        w = 1.0 / std::sqrt(L_eff * C);
        Z = std::sqrt(L_eff / C);
    } else {
        w = 1.0 / std::sqrt(Ls * C);  // power delivery: Lm clamped, only Ls and C ring
        Z = std::sqrt(Ls / C);
    }
}

// MKF Llc.cpp:356 propagate_substate — closed-form single sub-state step.
LlcStateVector llc_propagate_substate(LlcSubState s, LlcStateVector x_in, double dt, double Vi, double Vo,
                                      double Ls, double L, double C) {
    LlcStateVector out{};
    if (dt <= 0) return x_in;

    double w, Z;
    llc_substate_freq(s, Ls, L, C, w, Z);
    double cs = std::cos(w * dt);
    double sn = std::sin(w * dt);

    if (s == LlcSubState::B_FW) {
        double V_eq = Vi;  // freewheeling: I_Ls ≡ I_L, drive = Vi
        double dV = x_in.vC - V_eq;
        double iLs_new = x_in.iLs * cs - (dV / Z) * sn;
        double vC_new  = V_eq + dV * cs + x_in.iLs * Z * sn;
        out.iLs = iLs_new;
        out.iL  = iLs_new;  // tracks ILs in freewheeling
        out.vC  = vC_new;
        return out;
    }

    // Power-delivery: A_POS → V_drive = Vi−Vo, dIL/dt=+Vo/L; A_NEG → V_drive = Vi+Vo, dIL/dt=−Vo/L.
    double V_drive = (s == LlcSubState::A_POS) ? (Vi - Vo) : (Vi + Vo);
    double dIL_dt  = (s == LlcSubState::A_POS) ? (+Vo / L) : (-Vo / L);

    double dV = x_in.vC - V_drive;
    out.iLs = x_in.iLs * cs - (dV / Z) * sn;
    out.vC  = V_drive + dV * cs + x_in.iLs * Z * sn;
    out.iL  = x_in.iL + dIL_dt * dt;
    return out;
}

// MKF Llc.cpp:402 trigger_value — trigger g(dt) that ends the current sub-state.
double llc_trigger_value(LlcSubState s, LlcStateVector x_in, double dt, double Vi, double Vo,
                         double Ls, double L, double C) {
    LlcStateVector x = llc_propagate_substate(s, x_in, dt, Vi, Vo, Ls, L, C);
    if (s == LlcSubState::A_POS) return x.iL - x.iLs;   // > 0 means Id flipped negative
    if (s == LlcSubState::A_NEG) return x.iLs - x.iL;   // > 0 means Id flipped positive
    // B_FW: VLm = (L/(Ls+L))·(Vi−Vc) exits the ±Vo clamp window; return the most-positive violation.
    double VLm = (L / (Ls + L)) * (Vi - x.vC);
    double pos_violation = VLm - Vo;
    double neg_violation = -VLm - Vo;
    return std::max(pos_violation, neg_violation);
}

// MKF Llc.cpp:433 find_next_event — smallest t in (0, t_max] where the trigger crosses 0 upward.
double llc_find_next_event(LlcSubState s, LlcStateVector x_in, double t_max, double Vi, double Vo,
                           double Ls, double L, double C) {
    constexpr int COARSE_STEPS = 64;
    double dt_coarse = t_max / COARSE_STEPS;
    double prev_g = llc_trigger_value(s, x_in, 1e-12, Vi, Vo, Ls, L, C);
    if (prev_g >= 0) {
        return 0.0;  // trigger already true at t≈0 (tangent case): leave the boundary immediately
    }

    for (int k = 1; k <= COARSE_STEPS; ++k) {
        double t = k * dt_coarse;
        double g = llc_trigger_value(s, x_in, t, Vi, Vo, Ls, L, C);
        if (g >= 0 && std::isfinite(g)) {
            double lo = t - dt_coarse;
            double hi = t;
            double g_lo = prev_g;
            for (int it = 0; it < 50; ++it) {
                double mid = 0.5 * (lo + hi);
                double g_mid = llc_trigger_value(s, x_in, mid, Vi, Vo, Ls, L, C);
                if (g_mid * g_lo < 0) {
                    hi = mid;
                } else {
                    lo = mid;
                    g_lo = g_mid;
                }
                if ((hi - lo) < 1e-12) break;
            }
            return 0.5 * (lo + hi);
        }
        prev_g = g;
    }
    return t_max;  // no crossing
}

// MKF Llc.cpp:473 next_state_after_B — which secondary diode forward-biases after a B_FW transition.
LlcSubState llc_next_state_after_B(LlcStateVector x_at_event, double Vi, double Ls, double L) {
    double VLm = (L / (Ls + L)) * (Vi - x_at_event.vC);
    return (VLm > 0) ? LlcSubState::A_POS : LlcSubState::A_NEG;
}

// MKF Llc.cpp:484 initial_substate — sub-state at t=0 of the half cycle.
LlcSubState llc_initial_substate(LlcStateVector x0, double Vi, double Vo, double Ls, double L) {
    double Id = x0.iLs - x0.iL;
    if (std::abs(Id) > 1e-9) {
        return (Id > 0) ? LlcSubState::A_POS : LlcSubState::A_NEG;
    }
    double VLm = (L / (Ls + L)) * (Vi - x0.vC);
    if (VLm > Vo) return LlcSubState::A_POS;
    if (VLm < -Vo) return LlcSubState::A_NEG;
    return LlcSubState::B_FW;
}

// MKF Llc.cpp:500 propagate_half_cycle — event loop over [0, Thalf] → chain of closed-form segments.
std::vector<LlcSubStateSegment> llc_propagate_half_cycle(
    LlcStateVector x0, double Thalf, double Vi, double Vo, double Ls, double L, double C) {
    std::vector<LlcSubStateSegment> segments;
    segments.reserve(8);

    LlcSubState current = llc_initial_substate(x0, Vi, Vo, Ls, L);
    LlcStateVector x = x0;
    double t = 0.0;
    constexpr int MAX_SEGMENTS = 16;  // safety bound; LLC modes use ≤ 6

    for (int k = 0; k < MAX_SEGMENTS; ++k) {
        double remaining = Thalf - t;
        if (remaining <= 1e-15) break;

        double t_event = llc_find_next_event(current, x, remaining, Vi, Vo, Ls, L, C);
        double dt = std::min(t_event, remaining);
        if (dt < 1e-15 && k > 0) {
            // Zero-length segment: transition immediately to avoid an infinite loop.
            LlcSubState next;
            if (current == LlcSubState::B_FW) {
                next = llc_next_state_after_B(x, Vi, Ls, L);
            } else {
                next = LlcSubState::B_FW;
            }
            current = next;
            continue;
        }
        LlcStateVector x_end = llc_propagate_substate(current, x, dt, Vi, Vo, Ls, L, C);
        segments.push_back({current, t, t + dt, x, x_end});
        t += dt;
        x = x_end;

        if (t >= Thalf - 1e-15) break;

        if (current == LlcSubState::B_FW) {
            current = llc_next_state_after_B(x, Vi, Ls, L);
        } else {
            current = LlcSubState::B_FW;  // A_POS / A_NEG ended on Id = 0 → freewheeling
        }
    }

    return segments;
}

// MKF Llc.cpp:555 solve_steady_state — damped Newton on F(x0)=propagate_half(x0).end + x0 with Picard
// fallback; returns the BEST (lowest-residual) iterate seen, re-propagated for waveforms.
LlcStateVector llc_solve_steady_state(
    LlcStateVector x0_seed, double Thalf, double Vi, double Vo, double Ls, double L, double C,
    std::vector<LlcSubStateSegment>& outSegments, double& outResidual) {
    auto eval_F = [&](LlcStateVector x0) -> std::array<double, 3> {
        auto segs = llc_propagate_half_cycle(x0, Thalf, Vi, Vo, Ls, L, C);
        LlcStateVector x_end = segs.empty() ? x0 : segs.back().x_end;
        return {x_end.iLs + x0.iLs, x_end.iL + x0.iL, x_end.vC + x0.vC};
    };
    auto norm = [](const std::array<double, 3>& f) {
        return std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    };

    LlcStateVector x0 = x0_seed;
    auto F = eval_F(x0);
    double r = norm(F);

    constexpr int MAX_ITERS = 50;
    constexpr double TOL = 1e-7;

    double scale_i = std::max(1e-3, 0.01 * std::abs(x0.iLs) + 1e-3);
    double scale_v = std::max(1e-2, 0.01 * std::abs(x0.vC)  + 1e-2);

    double damping = 1.0;
    double prev_r = r;
    int stagnant = 0;

    LlcStateVector bestX0 = x0;  // track the best iterate; Newton/Picard is non-monotone
    double bestR = r;

    for (int iter = 0; iter < MAX_ITERS && r > TOL; ++iter) {
        // Jacobian by central differences.
        double J[3][3];
        LlcStateVector xp, xm;
        std::array<double, 3> Fp, Fm;

        xp = x0; xp.iLs += scale_i;
        xm = x0; xm.iLs -= scale_i;
        Fp = eval_F(xp); Fm = eval_F(xm);
        for (int i = 0; i < 3; ++i) J[i][0] = (Fp[i] - Fm[i]) / (2 * scale_i);

        xp = x0; xp.iL += scale_i;
        xm = x0; xm.iL -= scale_i;
        Fp = eval_F(xp); Fm = eval_F(xm);
        for (int i = 0; i < 3; ++i) J[i][1] = (Fp[i] - Fm[i]) / (2 * scale_i);

        xp = x0; xp.vC += scale_v;
        xm = x0; xm.vC -= scale_v;
        Fp = eval_F(xp); Fm = eval_F(xm);
        for (int i = 0; i < 3; ++i) J[i][2] = (Fp[i] - Fm[i]) / (2 * scale_v);

        // Solve J·dx = -F by Gaussian elimination with partial pivoting.
        double A[3][4];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) A[i][j] = J[i][j];
            A[i][3] = -F[i];
        }
        bool singular = false;
        for (int col = 0; col < 3; ++col) {
            int piv = col;
            double maxv = std::abs(A[col][col]);
            for (int row = col + 1; row < 3; ++row) {
                if (std::abs(A[row][col]) > maxv) {
                    maxv = std::abs(A[row][col]);
                    piv = row;
                }
            }
            if (maxv < 1e-14) { singular = true; break; }
            if (piv != col) std::swap(A[col], A[piv]);
            for (int row = col + 1; row < 3; ++row) {
                double f = A[row][col] / A[col][col];
                for (int k = col; k < 4; ++k) A[row][k] -= f * A[col][k];
            }
        }
        if (singular) break;
        double dx[3];
        for (int i = 2; i >= 0; --i) {
            double sum = A[i][3];
            for (int j = i + 1; j < 3; ++j) sum -= A[i][j] * dx[j];
            dx[i] = sum / A[i][i];
        }

        // Damped step + line search on ||F||.
        double try_d = damping;
        LlcStateVector x_new{};
        std::array<double, 3> F_new{};
        double r_new = r;
        bool accepted = false;
        for (int ls = 0; ls < 6; ++ls) {
            x_new.iLs = x0.iLs + try_d * dx[0];
            x_new.iL  = x0.iL  + try_d * dx[1];
            x_new.vC  = x0.vC  + try_d * dx[2];
            F_new = eval_F(x_new);
            r_new = norm(F_new);
            if (std::isfinite(r_new) && r_new < r) {
                accepted = true;
                break;
            }
            try_d *= 0.5;
        }
        if (!accepted) {
            // Picard fallback: x0_new = -x_end.
            auto segs = llc_propagate_half_cycle(x0, Thalf, Vi, Vo, Ls, L, C);
            if (!segs.empty()) {
                LlcStateVector x_end = segs.back().x_end;
                x_new.iLs = -x_end.iLs;
                x_new.iL  = -x_end.iL;
                x_new.vC  = -x_end.vC;
                F_new = eval_F(x_new);
                r_new = norm(F_new);
            }
        }
        x0 = x_new;
        F = F_new;
        if (r_new >= prev_r * 0.999) {
            stagnant++;
            damping *= 0.7;
        } else {
            stagnant = 0;
            damping = std::min(1.0, damping * 1.2);
        }
        prev_r = r_new;
        r = r_new;
        if (std::isfinite(r) && r < bestR) {
            bestR = r;
            bestX0 = x0;
        }
        if (stagnant >= 4) break;
    }

    outSegments = llc_propagate_half_cycle(bestX0, Thalf, Vi, Vo, Ls, L, C);
    outResidual = bestR;
    return bestX0;
}

// MKF Llc.cpp:744 sample_segments — raster the segment chain onto a uniform [0, Thalf] grid (N+1 points).
void llc_sample_segments(const std::vector<LlcSubStateSegment>& segs, double Thalf, int N,
                         double Vi, double Vo, double Ls, double L, double C,
                         std::vector<double>& ILs_pos, std::vector<double>& IL_pos,
                         std::vector<double>& Vc_pos, std::vector<double>& VL_pos) {
    double dt = Thalf / N;
    size_t segIdx = 0;
    for (int k = 0; k <= N; ++k) {
        double t = k * dt;
        if (t > Thalf) t = Thalf;
        while (segIdx + 1 < segs.size() && t > segs[segIdx].t_end + 1e-15) ++segIdx;
        if (segs.empty()) {
            ILs_pos[k] = 0; IL_pos[k] = 0; Vc_pos[k] = 0; VL_pos[k] = 0;
            continue;
        }
        const auto& seg = segs[segIdx];
        double t_local = t - seg.t_start;
        if (t_local < 0) t_local = 0;
        if (t_local > seg.t_end - seg.t_start) t_local = seg.t_end - seg.t_start;
        LlcStateVector x = llc_propagate_substate(seg.state, seg.x_start, t_local, Vi, Vo, Ls, L, C);
        ILs_pos[k] = x.iLs;
        IL_pos[k]  = x.iL;
        Vc_pos[k]  = x.vC;
        if (seg.state == LlcSubState::A_POS) {
            VL_pos[k] = +Vo;
        } else if (seg.state == LlcSubState::A_NEG) {
            VL_pos[k] = -Vo;
        } else {
            VL_pos[k] = (L / (Ls + L)) * (Vi - x.vC);  // B_FW: VLm = (L/(Ls+L))·(Vi−Vc)
        }
    }
}

}  // anonymous namespace

// Ported from MKF converter_models/Llc.cpp:786 (process_operating_point_for_input_voltage). The MKF
// member/method reads become explicit scalar params (KH's MAS has no LlcOperatingPoint): k_bridge →
// bridgeVoltageFactor, computedResonantInductance → seriesResonantInductance, computedResonantCapacitance
// → resonantCapacitance, get_integrated_resonant_inductor() → integratedResonantInductor,
// get_effective_rectifier_type() → rectifier (CENTER_TAPPED / FULL_BRIDGE). Missing/non-positive
// quantities THROW (mirroring MKF's own guards — no fabricated defaults).
MAS::OperatingPoint analytical_llc(double inputVoltage,
                                   const std::vector<double>& outputVoltages,
                                   const std::vector<double>& outputCurrents,
                                   const std::vector<double>& turnsRatios,
                                   double switchingFrequency,
                                   double magnetizingInductance,
                                   double seriesResonantInductance,
                                   double resonantCapacitance,
                                   double bridgeVoltageFactor,
                                   SrcRectifier rectifier,
                                   bool integratedResonantInductor) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_llc: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double fsw = switchingFrequency;
    if (fsw <= 0) throw std::invalid_argument("analytical_llc: switching frequency must be > 0");
    const double Ls = seriesResonantInductance;
    const double C  = resonantCapacitance;
    const double L  = magnetizingInductance;
    // MKF guards (Llc.cpp:849, :875): magnetizing inductance and resonant-tank values must be positive.
    if (L <= 0) throw std::invalid_argument("analytical_llc: magnetizing inductance must be > 0");
    if (Ls <= 0 || C <= 0) throw std::invalid_argument("analytical_llc: resonant Ls/Cr must be > 0");

    const double k_bridge = bridgeVoltageFactor;
    const bool integratedLs = integratedResonantInductor;
    const double Vi = k_bridge * inputVoltage;  // MKF Llc.cpp:813

    const size_t nOutputs = outputVoltages.size();
    const double n_main = turnsRatios[0];       // MKF get_n_for_output(0)
    if (n_main <= 0) throw std::invalid_argument("analytical_llc: turns ratio must be > 0");
    double Vo = n_main * outputVoltages[0];     // MKF Llc.cpp:838 (VD's ×0.5 is not ported)

    if (!std::isfinite(Vo) || Vo < 0)  // MKF Llc.cpp:890
        throw std::invalid_argument("analytical_llc: reflected output voltage Vo is invalid");

    const double period = 1.0 / fsw;      // MKF Llc.cpp:878
    const double Thalf = period / 2.0;
    const double Thalf_eff = Thalf;       // MKF Llc.cpp:888 (dead time not modelled)
    if (!std::isfinite(Thalf_eff) || Thalf_eff <= 0)  // MKF Llc.cpp:892
        throw std::invalid_argument("analytical_llc: half switching period is invalid");

    const int N = 256;                    // MKF Llc.cpp:905
    const double dt = Thalf_eff / N;

    // FHA-style Newton seed (MKF Llc.cpp:911-920).
    double Im_pk_est = Vo * Thalf_eff / (2.0 * L);
    if (!std::isfinite(Im_pk_est) || std::abs(Im_pk_est) > 1e6)
        Im_pk_est = std::copysign(1.0, Im_pk_est);

    double Iout_main = outputCurrents[0];
    double Iload_reflected = Iout_main / n_main;
    double Ires_est = std::max(std::abs(Im_pk_est) + std::abs(Iload_reflected),
                               std::abs(Iload_reflected) * 1.5);

    LlcStateVector x0_seed{ -Ires_est, -Im_pk_est, 0.0 };

    std::vector<LlcSubStateSegment> segments;
    double residual = 0.0;

    // LIP singularity perturbation (MKF Llc.cpp:941-945): near Vi ≈ Vo the (iLs, vC) sub-system is a 180°
    // rotation and the Newton Jacobian is rank-deficient; nudge Vi up 0.5% for the solve only.
    double Vi_solver = Vi;
    double denom_vo = std::max(std::abs(Vi), std::abs(Vo));
    if (denom_vo > 0 && std::abs(Vi - Vo) / denom_vo < 0.005) {
        Vi_solver = Vi * 1.005;
    }
    // Multi-start (MKF Llc.cpp:954-982): the half-cycle steady-state map is piecewise, so ‖F‖ has several
    // basins; seed from several physically-plausible start points and keep the lowest residual.
    double Zr = (C > 0) ? std::sqrt(Ls / C) : 0.0;
    std::vector<LlcStateVector> seeds = {
        x0_seed,
        { -0.7 * Ires_est, -Im_pk_est, 0.0 },
        { -1.4 * Ires_est, -Im_pk_est, 0.0 },
        { -Ires_est, -Im_pk_est,  Ires_est * Zr },
        { -Ires_est, -Im_pk_est, -Ires_est * Zr },
        {  Ires_est, -Im_pk_est, 0.0 },
        { -Ires_est,  Im_pk_est, 0.0 },
    };
    LlcStateVector x0 = x0_seed;
    residual = std::numeric_limits<double>::max();
    for (const auto& seed : seeds) {
        std::vector<LlcSubStateSegment> segTry;
        double rTry = 0.0;
        LlcStateVector xTry = llc_solve_steady_state(seed, Thalf_eff, Vi_solver, Vo, Ls, L, C, segTry, rTry);
        if (rTry >= 0.0 && rTry < residual) {
            residual = rTry;
            x0 = xTry;
            segments = segTry;
        }
        if (residual < 1e-3) break;  // off-LIP converges on the first seed; skip the rest
    }
    if (segments.empty()) {  // all starts bailed — fall back to the default solve
        x0 = llc_solve_steady_state(x0_seed, Thalf_eff, Vi_solver, Vo, Ls, L, C, segments, residual);
    }

    // Sanity check (MKF Llc.cpp:989-997): reject an unphysical null-space iterate → FHA closed-form seed.
    double sanity_iLs = std::max(10.0 * Ires_est, 20.0);
    double sanity_vC  = std::max(10.0 * std::abs(Vi), 10.0 * std::abs(Vo));
    if (sanity_vC < 200.0) sanity_vC = 200.0;
    if (!std::isfinite(x0.iLs) || !std::isfinite(x0.iL) || !std::isfinite(x0.vC) ||
        std::abs(x0.iLs) > sanity_iLs ||
        std::abs(x0.vC)  > sanity_vC) {
        x0 = x0_seed;
        residual = -1.0;  // flag "seed fallback"
    }
    // Re-propagate with the authoritative Vi for waveform emission (MKF Llc.cpp:1000).
    segments = llc_propagate_half_cycle(x0, Thalf_eff, Vi, Vo, Ls, L, C);

    std::vector<double> ILs_pos(N + 1, 0.0), IL_pos(N + 1, 0.0);
    std::vector<double> Vc_pos(N + 1, 0.0), VL_pos(N + 1, 0.0);
    llc_sample_segments(segments, Thalf_eff, N, Vi, Vo, Ls, L, C, ILs_pos, IL_pos, Vc_pos, VL_pos);

    double ILs0 = x0.iLs;  // MKF Llc.cpp:1038
    double IL0  = x0.iL;

    // Convergence diagnostic (MKF Llc.cpp:1052-1058): a bounded-but-imperfect residual → warn, don't throw.
    double i_thresh = std::max(0.5, 0.02 * std::abs(Iload_reflected));
    if (residual > i_thresh) {
        std::cerr << "[analytical_llc] Nielsen TDA solver did not fully converge: residual="
                  << residual << " A (Vi=" << Vi << "V, Vo=" << Vo << "V, fsw=" << fsw
                  << "Hz) — analytical waveform is bounded but may be imperfect for this op point."
                  << std::endl;
    }

    // Build full-period waveforms by half-wave antisymmetry (MKF Llc.cpp:1069-1106).
    int totalSamples = 2 * N + 1;
    std::vector<double> time_full(totalSamples);
    std::vector<double> ILs_full(totalSamples);
    std::vector<double> IL_full(totalSamples);
    std::vector<double> Vpri_full(totalSamples);  // primary voltage (topology-dependent)
    std::vector<double> VLm_full(totalSamples);   // magnetizing voltage (for the secondaries)

    for (int k = 0; k <= N; ++k) {
        time_full[k] = k * dt;
        ILs_full[k] = std::isfinite(ILs_pos[k]) ? ILs_pos[k] : ILs0;
        IL_full[k]  = std::isfinite(IL_pos[k])  ? IL_pos[k]  : IL0;

        double VLm_k = std::isfinite(VL_pos[k]) ? VL_pos[k] : 0.0;
        double Vc_k  = std::isfinite(Vc_pos[k]) ? Vc_pos[k] : 0.0;
        VLm_full[k] = VLm_k;

        if (integratedLs) {
            Vpri_full[k] = Vi - Vc_k;   // integrated Lr: bridge voltage with Cr ripple
        } else {
            Vpri_full[k] = VLm_k;       // separate Lr: magnetizing voltage only
        }
    }

    for (int k = 1; k <= N; ++k) {
        time_full[N + k] = Thalf + k * dt;
        ILs_full[N + k] = std::isfinite(ILs_pos[k]) ? -ILs_pos[k] : -ILs0;
        IL_full[N + k]  = std::isfinite(IL_pos[k])  ? -IL_pos[k]  : -IL0;

        double VLm_k = std::isfinite(VL_pos[k]) ? VL_pos[k] : 0.0;
        double Vc_k  = std::isfinite(Vc_pos[k]) ? Vc_pos[k] : 0.0;
        VLm_full[N + k] = -VLm_k;

        if (integratedLs) {
            Vpri_full[N + k] = -(Vi - Vc_k);
        } else {
            Vpri_full[N + k] = -VLm_k;
        }
    }

    MAS::OperatingPoint operatingPoint;
    // Primary excitation (MKF Llc.cpp:1154-1169): tank current I_Ls + topology-dependent voltage.
    {
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(ILs_full);
        currentWaveform.set_time(time_full);

        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(Vpri_full);
        voltageWaveform.set_time(time_full);

        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, fsw, "Primary"));
    }

    // Secondary excitations (MKF Llc.cpp:1171-1359). Id = I_Ls − I_L is the primary-referred diode
    // current (the tank current transferred to the secondaries); I_sec = Id·share·n (ampere-turn
    // balance). CT → two half-windings; FB → one bipolar winding. (VD / CD are not ported.)
    double total_g = 0.0;
    for (size_t i = 0; i < nOutputs; ++i) {
        double Vout_i = outputVoltages[i];
        double Iout_i = outputCurrents[i];
        if (Vout_i > 0 && Iout_i > 0) total_g += Iout_i / Vout_i;
    }
    if (total_g <= 0) total_g = 1.0;

    for (size_t outputIdx = 0; outputIdx < nOutputs; ++outputIdx) {
        double n_i = turnsRatios[outputIdx];
        if (n_i <= 0) n_i = 1.0;

        double Vout_i = outputVoltages[outputIdx];
        double Iout_i = outputCurrents[outputIdx];
        double share = (Vout_i > 0 && Iout_i > 0) ? (Iout_i / Vout_i) / total_g
                                                  : (outputIdx == 0 ? 1.0 : 0.0);

        if (rectifier == SrcRectifier::CENTER_TAPPED) {
            // MKF Llc.cpp:1218-1244 — two half-windings, each conducting on one polarity.
            for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
                std::vector<double> iSecData(totalSamples, 0.0);
                std::vector<double> vSecData(totalSamples, 0.0);
                for (int k = 0; k < totalSamples; ++k) {
                    double Id = ILs_full[k] - IL_full[k];
                    if (!std::isfinite(Id)) Id = 0;
                    double Id_share = Id * share;
                    double Id_half = (halfIdx == 0) ? std::max(0.0, Id_share) : std::max(0.0, -Id_share);
                    iSecData[k] = Id_half * n_i;
                    vSecData[k] = VLm_full[k] / n_i;
                    if (!std::isfinite(iSecData[k])) iSecData[k] = 0;
                    if (!std::isfinite(vSecData[k])) vSecData[k] = 0;
                }
                MAS::Waveform secCurrentWfm; secCurrentWfm.set_ancillary_label(Lbl::CUSTOM);
                secCurrentWfm.set_data(iSecData); secCurrentWfm.set_time(time_full);
                MAS::Waveform secVoltageWfm; secVoltageWfm.set_ancillary_label(Lbl::CUSTOM);
                secVoltageWfm.set_data(vSecData); secVoltageWfm.set_time(time_full);
                std::string windingName = "Secondary " + std::to_string(outputIdx)
                                        + " Half " + std::to_string(halfIdx + 1);
                operatingPoint.get_mutable_excitations_per_winding().push_back(
                    WP::complete_excitation(secCurrentWfm, secVoltageWfm, fsw, windingName));
            }
        } else {  // FULL_BRIDGE — MKF Llc.cpp:1246-1266: single bipolar secondary winding.
            std::vector<double> iSecData(totalSamples, 0.0);
            std::vector<double> vSecData(totalSamples, 0.0);
            for (int k = 0; k < totalSamples; ++k) {
                double Id = ILs_full[k] - IL_full[k];
                if (!std::isfinite(Id)) Id = 0;
                iSecData[k] = Id * share * n_i;
                vSecData[k] = VLm_full[k] / n_i;
                if (!std::isfinite(iSecData[k])) iSecData[k] = 0;
                if (!std::isfinite(vSecData[k])) vSecData[k] = 0;
            }
            MAS::Waveform secCurrentWfm; secCurrentWfm.set_ancillary_label(Lbl::CUSTOM);
            secCurrentWfm.set_data(iSecData); secCurrentWfm.set_time(time_full);
            MAS::Waveform secVoltageWfm; secVoltageWfm.set_ancillary_label(Lbl::CUSTOM);
            secVoltageWfm.set_data(vSecData); secVoltageWfm.set_time(time_full);
            std::string windingName = "Secondary " + std::to_string(outputIdx);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(secCurrentWfm, secVoltageWfm, fsw, windingName));
        }
    }

    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// CLLC bidirectional resonant converter — 4-state time-domain analysis (Sun et al. 2020 IEEE TPEL
// 35(4):3491–3505). Every helper below is transcribed FAITHFULLY from MKF converter_models/Cllc.cpp:735-
// 1081 (the cllc4_* anonymous machinery) — the 2×2 eigendecomposition propagator, the block-antidiagonal
// ODE forcing, the event conditions, the damped-Picard steady-state solve, and the multi-start seeds are
// byte-for-byte the MKF numerics (MKF file:line cited per function). MKF ALWAYS routes CLLC through this
// 4-state path (`const bool is_asymmetric = true`, Cllc.cpp:1258); its 3-state collapsed path
// (Cllc.cpp:375-710, identical to the ported LLC above) is dead code and is NOT ported.
//
// State x = [i_Lr1, i_Lm, v_Cr1, v_Cr2_pri]  (v_Cr2 referred to the primary). Three sub-states cover the
// modes (bridge = +Vi throughout the positive half cycle):
//   P_POS: Id = i_Lr1 − i_Lm > 0, +Vo rectifier conducting   P_NEG: Id < 0, −Vo conducting
//   F    : freewheel (i_Lr1 ≡ i_Lm, i_Lr2 = 0 → Cr2 frozen)
// Per-mode forcing (σ = +1 P_POS, −1 P_NEG), Δ = Lr1·Lm + Lr1·Lr2 + Lr2·Lm; equilibrium (0, 0, Vi, −σ·Vo).
// ─────────────────────────────────────────────────────────────────────────────
namespace {

enum class CllcSubState { P_POS, P_NEG, F };  // MKF Cllc.cpp:375

struct CllcStateVector { double iLs; double iL; double vC; };  // MKF Cllc.cpp:377 (collapsed 3-vector)

struct CllcSubStateSegment {  // MKF Cllc.cpp:383
    CllcSubState state;
    double t_start;
    double t_end;
    CllcStateVector x_start;
    CllcStateVector x_end;
};

struct CllcTankParams4 { double Lr1, Lm, Lr2, Cr1, Cr2; };  // MKF Cllc.cpp:735
struct CllcState4 { double iLr1, iLm, vCr1, vCr2; };          // MKF Cllc.cpp:736
struct CllcSegment4 {                                         // MKF Cllc.cpp:737
    CllcSubState state;
    double t_start, t_end;
    CllcState4 x_start, x_end;
};

// MKF Cllc.cpp:748 — solve d²x/dt² = M·x for 2-D x at time t given x(0) and dx/dt(0), via a 2×2
// eigendecomposition of M. Both eigenvalues of M are ≤ 0 for a physical (lossless) tank.
void cllc4_oscillate_2x2(const double M[2][2], double t,
                         double x0, double x1, double xd0, double xd1,
                         double& y0, double& y1) {
    double tr = M[0][0] + M[1][1];
    double det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
    double disc = (tr*tr) * 0.25 - det;
    if (disc < 0 && disc > -1e-20) disc = 0;   // numerical fuzz
    if (disc < 0) {
        throw std::runtime_error(
            "CLLC 4-state propagator: M has complex eigenvalues (disc=" + std::to_string(disc) +
            "); tank parameters may be non-physical (negative Lr/Cr).");
    }
    double sq = std::sqrt(disc);
    double mu1 = tr * 0.5 + sq;     // larger (less negative) eigenvalue
    double mu2 = tr * 0.5 - sq;
    if (mu1 > 1e-6 || mu2 > 1e-6) {
        throw std::runtime_error(
            "CLLC 4-state propagator: M has positive eigenvalue (μ1=" + std::to_string(mu1) +
            ", μ2=" + std::to_string(mu2) + "); tank is non-passive.");
    }
    double w1 = std::sqrt(std::max(-mu1, 0.0));
    double w2 = std::sqrt(std::max(-mu2, 0.0));
    double v1[2], v2[2];
    if (std::abs(M[0][1]) > 1e-30 * (std::abs(tr) + 1.0)) {
        v1[0] = M[0][1]; v1[1] = mu1 - M[0][0];
        v2[0] = M[0][1]; v2[1] = mu2 - M[0][0];
    }
    else if (std::abs(M[1][0]) > 1e-30 * (std::abs(tr) + 1.0)) {
        v1[0] = mu1 - M[1][1]; v1[1] = M[1][0];
        v2[0] = mu2 - M[1][1]; v2[1] = M[1][0];
    }
    else {
        if (std::abs(M[0][0] - mu1) < std::abs(M[0][0] - mu2)) {
            v1[0] = 1; v1[1] = 0; v2[0] = 0; v2[1] = 1;
        } else {
            v1[0] = 0; v1[1] = 1; v2[0] = 1; v2[1] = 0;
        }
    }
    double det_P = v1[0]*v2[1] - v1[1]*v2[0];
    if (std::abs(det_P) < 1e-30) {
        throw std::runtime_error(
            "CLLC 4-state propagator: degenerate eigenvector matrix "
            "(repeated eigenvalue without Jordan block handling).");
    }
    auto decomp = [&](double r0, double r1, double& a, double& b) {
        a = ( v2[1]*r0 - v2[0]*r1) / det_P;
        b = (-v1[1]*r0 + v1[0]*r1) / det_P;
    };
    double a0, b0, ad0, bd0;
    decomp(x0,  x1,  a0,  b0);
    decomp(xd0, xd1, ad0, bd0);
    auto evolve = [](double w, double t_, double a, double ad) {
        if (w < 1e-30) return a + ad * t_;     // ω→0: drift limit
        return a * std::cos(w*t_) + ad * std::sin(w*t_) / w;
    };
    double c1 = evolve(w1, t, a0, ad0);
    double c2 = evolve(w2, t, b0, bd0);
    y0 = c1*v1[0] + c2*v2[0];
    y1 = c1*v1[1] + c2*v2[1];
}

// MKF Cllc.cpp:823 — closed-form propagator for one 4-state sub-state (bridge = +Vi).
CllcState4 cllc4_propagate_substate(CllcSubState s, CllcState4 x_in,
                                    double dt, double Vi, double Vo,
                                    const CllcTankParams4& tp) {
    if (dt <= 0) return x_in;
    if (s == CllcSubState::F) {
        // Freewheel: iLr1 ≡ iLm, vCr2 frozen. Single LC oscillator: L_F = Lr1+Lm, C_F = Cr1, eq (0, Vi).
        double L_F = tp.Lr1 + tp.Lm;
        double w  = 1.0 / std::sqrt(L_F * tp.Cr1);
        double Z  = std::sqrt(L_F / tp.Cr1);
        double cs = std::cos(w*dt), sn = std::sin(w*dt);
        double iLm0 = x_in.iLm;          // = iLr1 in F mode
        double dVc  = x_in.vCr1 - Vi;
        CllcState4 out{};
        out.iLm  = iLm0 * cs - dVc / Z * sn;
        out.iLr1 = out.iLm;
        out.vCr1 = Vi + dVc * cs + iLm0 * Z * sn;
        out.vCr2 = x_in.vCr2;
        return out;
    }
    // P_POS / P_NEG: full 4-state propagation.
    double sigma = (s == CllcSubState::P_POS) ? +1.0 : -1.0;
    double Delta = tp.Lr1*tp.Lm + tp.Lr1*tp.Lr2 + tp.Lr2*tp.Lm;
    double x3_eq = Vi;
    double x4_eq = -sigma * Vo;
    double xt0 = x_in.iLr1;
    double xt1 = x_in.iLm;
    double yt0 = x_in.vCr1 - x3_eq;
    double yt1 = x_in.vCr2 - x4_eq;
    double A12[2][2] = {
        { -(tp.Lm + tp.Lr2)/Delta, -tp.Lm/Delta },
        { -tp.Lr2/Delta,             tp.Lr1/Delta }
    };
    double A21[2][2] = {
        { 1.0/tp.Cr1, 0.0 },
        { 1.0/tp.Cr2, -1.0/tp.Cr2 }
    };
    double xd0 = A12[0][0]*yt0 + A12[0][1]*yt1;
    double xd1 = A12[1][0]*yt0 + A12[1][1]*yt1;
    double yd0 = A21[0][0]*xt0 + A21[0][1]*xt1;
    double yd1 = A21[1][0]*xt0 + A21[1][1]*xt1;
    double Mt[2][2] = {
        { A12[0][0]*A21[0][0] + A12[0][1]*A21[1][0],
          A12[0][0]*A21[0][1] + A12[0][1]*A21[1][1] },
        { A12[1][0]*A21[0][0] + A12[1][1]*A21[1][0],
          A12[1][0]*A21[0][1] + A12[1][1]*A21[1][1] }
    };
    double Mb[2][2] = {
        { A21[0][0]*A12[0][0] + A21[0][1]*A12[1][0],
          A21[0][0]*A12[0][1] + A21[0][1]*A12[1][1] },
        { A21[1][0]*A12[0][0] + A21[1][1]*A12[1][0],
          A21[1][0]*A12[0][1] + A21[1][1]*A12[1][1] }
    };
    double x_new0, x_new1, y_new0, y_new1;
    cllc4_oscillate_2x2(Mt, dt, xt0, xt1, xd0, xd1, x_new0, x_new1);
    cllc4_oscillate_2x2(Mb, dt, yt0, yt1, yd0, yd1, y_new0, y_new1);
    CllcState4 out{};
    out.iLr1 = x_new0;
    out.iLm  = x_new1;
    out.vCr1 = y_new0 + x3_eq;
    out.vCr2 = y_new1 + x4_eq;
    return out;
}

// MKF Cllc.cpp:896 — trigger value at end of dt (P→F when Id reaches 0; F→P when VLm exits ±Vo).
double cllc4_trigger_value(CllcSubState s, CllcState4 x_in, double dt,
                           double Vi, double Vo, const CllcTankParams4& tp) {
    CllcState4 x = cllc4_propagate_substate(s, x_in, dt, Vi, Vo, tp);
    if (s == CllcSubState::P_POS) return x.iLm  - x.iLr1;    // rises to 0
    if (s == CllcSubState::P_NEG) return x.iLr1 - x.iLm;     // rises to 0
    double VLm = (tp.Lm / (tp.Lr1 + tp.Lm)) * (Vi - x.vCr1);
    return std::max(VLm - Vo, -VLm - Vo);
}

// MKF Cllc.cpp:905 — coarse-grid bisection event finder.
double cllc4_find_next_event(CllcSubState s, CllcState4 x_in, double t_max,
                             double Vi, double Vo, const CllcTankParams4& tp) {
    constexpr int COARSE_STEPS = 64;
    double dt_coarse = t_max / COARSE_STEPS;
    double prev_g = cllc4_trigger_value(s, x_in, 1e-12, Vi, Vo, tp);
    if (prev_g >= 0) return 0.0;
    for (int k = 1; k <= COARSE_STEPS; ++k) {
        double t = k * dt_coarse;
        double g = cllc4_trigger_value(s, x_in, t, Vi, Vo, tp);
        if (g >= 0 && std::isfinite(g)) {
            double lo = t - dt_coarse, hi = t, g_lo = prev_g;
            for (int it = 0; it < 50; ++it) {
                double mid = 0.5 * (lo + hi);
                double g_mid = cllc4_trigger_value(s, x_in, mid, Vi, Vo, tp);
                if (g_mid * g_lo < 0) { hi = mid; }
                else { lo = mid; g_lo = g_mid; }
                if ((hi - lo) < 1e-12) break;
            }
            return 0.5 * (lo + hi);
        }
        prev_g = g;
    }
    return t_max;
}

// MKF Cllc.cpp:930 — next P sub-state after a freewheel ends (sign of VLm).
CllcSubState cllc4_next_state_after_F(CllcState4 x, double Vi, const CllcTankParams4& tp) {
    double VLm = (tp.Lm / (tp.Lr1 + tp.Lm)) * (Vi - x.vCr1);
    return (VLm > 0) ? CllcSubState::P_POS : CllcSubState::P_NEG;
}

// MKF Cllc.cpp:936 — initial sub-state at t=0+ of the half cycle.
CllcSubState cllc4_initial_substate(CllcState4 x0, double Vi, double Vo, const CllcTankParams4& tp) {
    double Id = x0.iLr1 - x0.iLm;   // = iLr2_pri (secondary tank current)
    if (std::abs(Id) > 1e-9) {
        return (Id > 0) ? CllcSubState::P_POS : CllcSubState::P_NEG;
    }
    double VLm = (tp.Lm / (tp.Lr1 + tp.Lm)) * (Vi - x0.vCr1);
    if (VLm >  Vo) return CllcSubState::P_POS;
    if (VLm < -Vo) return CllcSubState::P_NEG;
    return CllcSubState::F;
}

// MKF Cllc.cpp:948 — drive the event loop over [0, Thalf] for one half cycle.
std::vector<CllcSegment4> cllc4_propagate_half_cycle(
    CllcState4 x0, double Thalf, double Vi, double Vo, const CllcTankParams4& tp) {
    std::vector<CllcSegment4> segments;
    segments.reserve(8);
    CllcSubState current = cllc4_initial_substate(x0, Vi, Vo, tp);
    CllcState4 x = x0;
    double t = 0.0;
    constexpr int MAX_SEGMENTS = 16;
    for (int k = 0; k < MAX_SEGMENTS; ++k) {
        double remaining = Thalf - t;
        if (remaining <= 1e-15) break;
        double t_event = cllc4_find_next_event(current, x, remaining, Vi, Vo, tp);
        double dt = std::min(t_event, remaining);
        if (dt < 1e-15 && k > 0) {
            current = (current == CllcSubState::F)
                      ? cllc4_next_state_after_F(x, Vi, tp)
                      : CllcSubState::F;
            continue;
        }
        CllcState4 x_end = cllc4_propagate_substate(current, x, dt, Vi, Vo, tp);
        segments.push_back({current, t, t + dt, x, x_end});
        t += dt; x = x_end;
        if (t >= Thalf - 1e-15) break;
        current = (current == CllcSubState::F)
                  ? cllc4_next_state_after_F(x, Vi, tp)
                  : CllcSubState::F;
    }
    return segments;
}

// MKF Cllc.cpp:983 — damped-Picard steady-state solve on the half-wave antisymmetry x(Thalf) = −x(0).
// NOT Newton: the 4-D finite-difference Jacobian traps on degenerate freewheel-only fixed points for
// asymmetric CLLC tanks, so MKF uses a relaxed cycle iteration x_{n+1} = α·(−x(Thalf|x_n)) + (1−α)·x_n
// (the discrete analogue of a SPICE transient). Returns the BEST (lowest-residual) iterate seen.
CllcState4 cllc4_solve_steady_state(
    CllcState4 x0_seed, double Thalf, double Vi, double Vo, const CllcTankParams4& tp,
    std::vector<CllcSegment4>& outSegments, double& outResidual) {
    auto eval_F = [&](CllcState4 x0) -> std::array<double, 4> {
        auto segs = cllc4_propagate_half_cycle(x0, Thalf, Vi, Vo, tp);
        CllcState4 xe = segs.empty() ? x0 : segs.back().x_end;
        return { xe.iLr1 + x0.iLr1, xe.iLm + x0.iLm,
                 xe.vCr1 + x0.vCr1, xe.vCr2 + x0.vCr2 };
    };
    auto norm = [](const std::array<double, 4>& f) {
        return std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2] + f[3]*f[3]);
    };
    CllcState4 x0 = x0_seed;
    constexpr int MAX_ITERS = 200;
    constexpr double TOL = 1e-6;
    const double alpha = 0.4;
    double r = std::numeric_limits<double>::infinity();
    CllcState4 bestX0 = x0;
    double bestR = r;
    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        auto segs = cllc4_propagate_half_cycle(x0, Thalf, Vi, Vo, tp);
        if (segs.empty()) break;
        CllcState4 xe = segs.back().x_end;
        CllcState4 x_target{ -xe.iLr1, -xe.iLm, -xe.vCr1, -xe.vCr2 };
        CllcState4 x_new{
            (1.0 - alpha) * x0.iLr1 + alpha * x_target.iLr1,
            (1.0 - alpha) * x0.iLm  + alpha * x_target.iLm,
            (1.0 - alpha) * x0.vCr1 + alpha * x_target.vCr1,
            (1.0 - alpha) * x0.vCr2 + alpha * x_target.vCr2
        };
        if (!std::isfinite(x_new.iLr1) || !std::isfinite(x_new.iLm) ||
            !std::isfinite(x_new.vCr1) || !std::isfinite(x_new.vCr2)) break;
        x0 = x_new;
        auto F = eval_F(x0);
        r = norm(F);
        if (std::isfinite(r) && r < bestR) {
            bestR = r;
            bestX0 = x0;
        }
        if (r < TOL) break;
    }
    outSegments = cllc4_propagate_half_cycle(bestX0, Thalf, Vi, Vo, tp);
    outResidual = bestR;
    return bestX0;
}

// MKF Cllc.cpp:1051 — sample the 4-state segment chain onto a uniform N+1-long grid over [0, Thalf].
void cllc4_sample_segments(const std::vector<CllcSegment4>& segs, double Thalf, int N,
                           double Vi, double Vo, const CllcTankParams4& tp,
                           std::vector<double>& iLr1_out, std::vector<double>& iLm_out,
                           std::vector<double>& vCr1_out, std::vector<double>& vCr2_out) {
    double dt = Thalf / N;
    size_t segIdx = 0;
    for (int k = 0; k <= N; ++k) {
        double t = std::min<double>(k * dt, Thalf);
        while (segIdx + 1 < segs.size() && t > segs[segIdx].t_end + 1e-15) ++segIdx;
        if (segs.empty()) {
            iLr1_out[k] = iLm_out[k] = vCr1_out[k] = vCr2_out[k] = 0.0;
            continue;
        }
        const auto& seg = segs[segIdx];
        double t_local = t - seg.t_start;
        if (t_local < 0) t_local = 0;
        double seg_dt = seg.t_end - seg.t_start;
        if (t_local > seg_dt) t_local = seg_dt;
        CllcState4 x = cllc4_propagate_substate(seg.state, seg.x_start, t_local, Vi, Vo, tp);
        iLr1_out[k] = x.iLr1;
        iLm_out[k]  = x.iLm;
        vCr1_out[k] = x.vCr1;
        vCr2_out[k] = x.vCr2;
    }
}

}  // anonymous namespace

// Ported from MKF converter_models/Cllc.cpp:1087 (process_operating_point_for_input_voltage — the FORWARD
// power-flow branch). The MKF member/method reads become explicit scalar params (KH's MAS has no
// CllcOperatingPoint): get_bridge_voltage_factor() → bridgeVoltageFactor, params.primaryResonantInductance
// → primaryResonantInductance, etc. Missing/non-positive quantities and an infeasible conversion gain
// THROW (mirroring MKF's own guards — no fabricated defaults). MKF's diagnostic-only pieces (ZVS margins,
// mode classification, extra-component Cr/Lr waveforms) and its REVERSE branch are omitted.
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
                                    double bridgeVoltageFactor,
                                    SrcRectifier rectifier) {
    using Lbl = MAS::WaveformLabel;
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_cllc: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double switchingFrequency_ = switchingFrequency;
    if (switchingFrequency_ <= 0)   // MKF Cllc.cpp:1100
        throw std::invalid_argument("analytical_cllc: switching frequency must be > 0");

    const double outputVoltage = outputVoltages[0];   // MKF reads output_voltages[0]/output_currents[0]
    const double outputCurrent = outputCurrents[0];
    const double n  = turnsRatios[0];                 // MKF Cllc.cpp:1105
    const double Lm = magnetizingInductance;
    if (n <= 0 || Lm <= 0)          // MKF Cllc.cpp:1107
        throw std::invalid_argument("analytical_cllc: invalid turns ratio or magnetizing inductance");

    // Collapse 5-element CLLC tank to the primary-referred quantities (MKF Cllc.cpp:1119-1129).
    const double Lr1     = primaryResonantInductance;
    const double Cr1     = primaryResonantCapacitance;
    const double Lr2_sec = secondaryResonantInductance;
    const double Cr2_sec = secondaryResonantCapacitance;
    if (Lr1 <= 0 || Cr1 <= 0 || Lr2_sec <= 0 || Cr2_sec <= 0)   // MKF Cllc.cpp:1123
        throw std::invalid_argument("analytical_cllc: resonant tank values invalid (Lr1/Cr1/Lr2/Cr2 > 0)");

    const double Lr2_pri = Lr2_sec * n * n;
    const double Cr2_pri = Cr2_sec / (n * n);
    const double Lr_eq   = Lr1 + Lr2_pri;            // used for VLm (F sub-state)

    const double k_bridge = bridgeVoltageFactor;
    // FORWARD power flow only (MKF Cllc.cpp:1149-1152). The REVERSE (isReverse) branch is not ported.
    const double Vi = k_bridge * inputVoltage;
    const double Vo = n * outputVoltage;             // reflected output (primary-referred)

    // Feasibility pre-check (MKF Cllc.cpp:1173-1192). A resonant CLLC operates near unity primary-referred
    // gain; M_req = |Vo|/|Vi| far outside [0.5, 3.0] has no bounded power-transferring steady state, so the
    // solver would only converge to a fabricated zero-power waveform. Throw instead (no-fallback rule).
    double M_req = (std::abs(Vi) > 0.0) ? std::abs(Vo) / std::abs(Vi)
                                        : std::numeric_limits<double>::infinity();
    if (M_req < 0.5 || M_req > 3.0) {
        throw std::invalid_argument(
            "analytical_cllc: operating point is infeasible for this resonant tank — the requested "
            "conversion gain n*Vout/(k*Vin) = " + std::to_string(M_req) + " is outside the range a "
            "resonant tank can supply (it operates near unity gain). Vin=" + std::to_string(inputVoltage) +
            " V, Vout=" + std::to_string(outputVoltage) + " V, n=" + std::to_string(n) + ".");
    }

    const double period = 1.0 / switchingFrequency_;   // MKF Cllc.cpp:1194
    const double Thalf  = period / 2.0;
    const double Thalf_eff = Thalf;                    // dead time not modelled (LLC convention)

    // FHA-style Newton/Picard seed (MKF Cllc.cpp:1207-1218).
    const double Im_pk_est = Vo * Thalf_eff / (2.0 * Lm);
    const double Iload_reflected = outputCurrent / n;   // FORWARD: load on secondary
    const double Ires_est = std::max(std::abs(Im_pk_est) + std::abs(Iload_reflected),
                                     std::abs(Iload_reflected) * 1.5);

    // No LIP perturbation for the 4-state path (MKF Cllc.cpp:1220-1227): the damped-Picard solver inverts
    // no Jacobian, so solve at the true Vi for a self-consistent converged state.
    const double Vi_solver = Vi;

    const int N = 200;                                  // MKF Cllc.cpp:1262
    const double dt = Thalf_eff / N;
    std::vector<double> ILs_pos(N + 1, 0.0), IL_pos(N + 1, 0.0), Vc_pos(N + 1, 0.0);

    std::vector<CllcSubStateSegment> segments;
    double residual = 0.0;

    // 4-state path (MKF Cllc.cpp:1266-1367). Lr2/Cr2 already primary-referred.
    CllcTankParams4 tp4{Lr1, Lm, Lr2_pri, Cr1, Cr2_pri};
    // Multi-start over physically-motivated seeds (MKF Cllc.cpp:1284-1294): the 4-D residual surface has
    // multiple basins; naïve zero-cap seeds drop into a degenerate freewheel-only fixed point. Seed the
    // caps near their ±extreme around the P_POS equilibria (Vi, −Vo) at a few AC amplitudes.
    const double Iload_pri = std::abs(Iload_reflected) * (M_PI / 2.0);   // pk reflected
    const double dVc1_est  = std::abs(Iload_pri) * Thalf_eff / (2.0 * Cr1);
    const double dVc2_est  = std::abs(Iload_pri) * Thalf_eff / (2.0 * Cr2_pri);
    const std::array<CllcState4, 6> seed_candidates{{
        { -Ires_est, -Im_pk_est, Vi - dVc1_est, -Vo - dVc2_est },
        { -Ires_est, -Im_pk_est, Vi + dVc1_est, -Vo + dVc2_est },
        { -Ires_est, -Im_pk_est, 0.0,           -Vo            },
        { -Ires_est, -Im_pk_est, -0.5 * Vi,     -1.5 * Vo      },
        { -Ires_est, -Im_pk_est, -Vi,           -Vo            },
        { -Ires_est, -Im_pk_est, 0.0,           0.0            }
    }};
    std::vector<CllcSegment4> segs4;
    CllcState4 x0_4{};
    double best_residual = std::numeric_limits<double>::infinity();
    for (const auto& cand : seed_candidates) {
        std::vector<CllcSegment4> segs_try;
        double res_try = 0.0;
        CllcState4 x_try = cllc4_solve_steady_state(cand, Thalf_eff, Vi_solver, Vo, tp4, segs_try, res_try);
        if (!std::isfinite(res_try) || res_try < 0) continue;
        // Reject degenerate F-only converged solutions (single freewheel, transfers no power).
        bool degenerate = (segs_try.size() == 1 && segs_try[0].state == CllcSubState::F);
        if (degenerate && res_try < 1e-6) continue;
        // Reject unphysical solutions (per-state sanity bounds).
        double sanity_i = std::max(10.0 * Ires_est, 20.0);
        double sanity_v = std::max({10.0 * std::abs(Vi), 10.0 * std::abs(Vo), 200.0});
        if (std::abs(x_try.iLr1) > sanity_i || std::abs(x_try.iLm)  > sanity_i ||
            std::abs(x_try.vCr1) > sanity_v || std::abs(x_try.vCr2) > sanity_v) continue;
        if (res_try < best_residual) {
            best_residual = res_try;
            x0_4 = x_try;
            segs4 = segs_try;
        }
    }
    residual = best_residual;
    // Re-propagate with the authoritative Vi (MKF Cllc.cpp:1330).
    segs4 = cllc4_propagate_half_cycle(x0_4, Thalf_eff, Vi, Vo, tp4);
    // Re-measure convergence on the OBSERVABLE states (MKF Cllc.cpp:1342-1348): tank current, magnetizing
    // current, and the TOTAL cap voltage vCr1+vCr2 (the cap common-mode split is an unobservable gauge).
    if (!segs4.empty() && residual >= 0.0) {
        const CllcState4& xe = segs4.back().x_end;
        double rI1 = xe.iLr1 + x0_4.iLr1;
        double rIm = xe.iLm  + x0_4.iLm;
        double rVsum = (xe.vCr1 + xe.vCr2) + (x0_4.vCr1 + x0_4.vCr2);
        residual = std::sqrt(rI1 * rI1 + rIm * rIm + rVsum * rVsum);
    }
    // Sample, then collapse vCr1+vCr2 → Vc_pos (MKF Cllc.cpp:1351-1354).
    std::vector<double> vCr1_pos(N + 1, 0.0), vCr2_pos(N + 1, 0.0);
    cllc4_sample_segments(segs4, Thalf_eff, N, Vi, Vo, tp4, ILs_pos, IL_pos, vCr1_pos, vCr2_pos);
    for (int k = 0; k <= N; ++k) Vc_pos[k] = vCr1_pos[k] + vCr2_pos[k];
    // Convert 4-state segments to the collapsed CllcSubStateSegment shape (MKF Cllc.cpp:1358-1367).
    segments.reserve(segs4.size());
    for (const auto& s4 : segs4) {
        segments.push_back({
            s4.state, s4.t_start, s4.t_end,
            CllcStateVector{s4.x_start.iLr1, s4.x_start.iLm, s4.x_start.vCr1 + s4.x_start.vCr2},
            CllcStateVector{s4.x_end.iLr1,  s4.x_end.iLm,  s4.x_end.vCr1  + s4.x_end.vCr2}
        });
    }

    // Guard the degenerate "no seed converged" case (MKF Cllc.cpp:1406-1411): throw on a non-finite
    // residual (the all-zero state would emit a fabricated zero-power waveform).
    if (residual != -1.0 && !std::isfinite(residual)) {
        throw std::runtime_error(
            "analytical_cllc: steady-state solver found no converging seed (all multi-start seeds failed); "
            "the all-zero state would emit a fabricated zero-power waveform. Vi=" + std::to_string(Vi) +
            " V, Vo=" + std::to_string(Vo) + " V.");
    }
    // Bounded-but-imperfect convergence (MKF Cllc.cpp:1417-1424): the 4-state TDA plateaus above TOL on
    // some designs (residual ~ several A). The waveform is non-degenerate and bounded → warn, don't throw.
    if (residual != -1.0) {
        double i_thresh = std::max(0.5, 0.02 * std::abs(Iload_reflected));
        if (residual > i_thresh) {
            std::cerr << "[analytical_cllc] steady-state solver did not fully converge: residual="
                      << residual << " A (Vi=" << Vi << "V, Vo=" << Vo
                      << "V) — analytical waveform is bounded but may be imperfect." << std::endl;
        }
    }

    // VLm at each sample (MKF Cllc.cpp:1427-1439): closed-form per sub-state.
    std::vector<double> VLm_pos(N + 1, 0.0);
    size_t segIdx = 0;
    for (int k = 0; k <= N; ++k) {
        double t = std::min<double>(k * dt, Thalf_eff);
        while (segIdx + 1 < segments.size() && t > segments[segIdx].t_end + 1e-15) ++segIdx;
        if (segments.empty()) { VLm_pos[k] = 0.0; continue; }
        const auto& seg = segments[segIdx];
        switch (seg.state) {
            case CllcSubState::P_POS: VLm_pos[k] = +Vo; break;
            case CllcSubState::P_NEG: VLm_pos[k] = -Vo; break;
            case CllcSubState::F: VLm_pos[k] = (Lm / (Lr_eq + Lm)) * (Vi - Vc_pos[k]); break;
        }
    }

    // Build full-period waveforms by half-wave antisymmetry (MKF Cllc.cpp:1547-1567).
    const int totalSamples = 2 * N + 1;
    std::vector<double> time_full(totalSamples);
    std::vector<double> ILs_full(totalSamples);
    std::vector<double> IL_full(totalSamples);
    std::vector<double> VLm_full(totalSamples);
    for (int k = 0; k <= N; ++k) {
        time_full[k] = k * dt;
        ILs_full[k] = std::isfinite(ILs_pos[k]) ? ILs_pos[k] : 0.0;
        IL_full[k]  = std::isfinite(IL_pos[k])  ? IL_pos[k]  : 0.0;
        VLm_full[k] = std::isfinite(VLm_pos[k]) ? VLm_pos[k] : 0.0;
    }
    for (int k = 1; k <= N; ++k) {
        time_full[N + k] = Thalf_eff + k * dt;
        ILs_full[N + k] = -ILs_full[k];
        IL_full[N + k]  = -IL_full[k];
        VLm_full[N + k] = -VLm_full[k];
    }

    MAS::OperatingPoint operatingPoint;
    // PRIMARY winding excitation (MKF Cllc.cpp:1637-1651): current = ILs (series tank current), voltage =
    // VLm (primary magnetizing voltage — what the core sees, not the bridge voltage).
    {
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(ILs_full);
        currentWaveform.set_time(time_full);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(VLm_full);
        voltageWaveform.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency_, "Primary"));
    }

    // SECONDARY winding excitation. MKF Cllc.cpp:1657-1675 emits a single full-wave secondary:
    //   I_sec = n·(ILs − IL) (the transferred rectifier current), V_sec = VLm/n.
    // FULL_BRIDGE (default) is that exact MKF winding. CENTER_TAPPED (family-consistent with
    // analytical_llc/_src; MKF CLLC itself is single-winding) splits the transferred current into two
    // polarity half-windings, mirroring the ported LLC's CT rectifier.
    if (rectifier == SrcRectifier::CENTER_TAPPED) {
        for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
            std::vector<double> iSec(totalSamples, 0.0), vSec(totalSamples, 0.0);
            for (int k = 0; k < totalSamples; ++k) {
                double Id = n * (ILs_full[k] - IL_full[k]);
                if (!std::isfinite(Id)) Id = 0.0;
                iSec[k] = (halfIdx == 0) ? std::max(0.0, Id) : std::max(0.0, -Id);
                vSec[k] = std::isfinite(VLm_full[k]) ? VLm_full[k] / n : 0.0;
            }
            MAS::Waveform currentWaveform;
            currentWaveform.set_ancillary_label(Lbl::CUSTOM);
            currentWaveform.set_data(iSec); currentWaveform.set_time(time_full);
            MAS::Waveform voltageWaveform;
            voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
            voltageWaveform.set_data(vSec); voltageWaveform.set_time(time_full);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency_,
                                        "Secondary 0 Half " + std::to_string(halfIdx + 1)));
        }
    } else {
        std::vector<double> iSec(totalSamples), vSec(totalSamples);
        for (int k = 0; k < totalSamples; ++k) {
            iSec[k] = n * (ILs_full[k] - IL_full[k]);
            vSec[k] = VLm_full[k] / n;
        }
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(iSec); currentWaveform.set_time(time_full);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(vSec); voltageWaveform.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, switchingFrequency_, "Secondary 0"));
    }

    return operatingPoint;
}

// ─── Phase 5: CLLLC bidirectional resonant converter (4-state RK4 affine-propagator TDA) ──────────────
// Ported from MKF converter_models/Clllc.cpp. Unlike the LLC/CLLC event-driven closed-form solvers, MKF's
// CLLLC integrates the 4-state linear tank ODE x = [i_Lr1, i_Lr2, v_Cr1, v_Cr2] with RK4 over the half
// period, exploiting that the half-period propagator P is AFFINE (P(x0) = M·x0 + g): it MEASURES g (= P(0))
// and the four columns of M with five RK4 passes then solves the half-wave-antisymmetric steady state
// (M+I)·x0 = −g by 4×4 Gaussian elimination — an exact (modulo RK4 truncation) steady state in six RK4
// passes, no Newton / no Picard (MKF Clllc.cpp:41-54). A 1 mΩ parasitic ESR on Lr1/Lr2 keeps M's spectral
// radius < 1 so (M+I) is non-singular even for a lossless tank at exact resonance. i_Lm is DERIVED
// algebraically as i_Lr1 − i_Lr2/n (transformer ampere-turn balance), not carried as a state; Lr2/Cr2 are
// ACTUAL secondary-side values (n enters the coupled-inductance matrix explicitly, not by primary-referral).
// This machinery is NOT byte-identical to the CLLC/LLC event-driven helpers already in this file, so
// CLLLC's own RK4 helpers are ported here.
namespace {

// MKF Clllc.cpp:356 — 4-state vector [i_Lr1, i_Lr2, v_Cr1, v_Cr2].
struct ClllcState {
    double iLr1 = 0;
    double iLr2 = 0;
    double vCr1 = 0;
    double vCr2 = 0;
};

// MKF Clllc.cpp:363 — tank + bridge parameters. Lr2/Cr2 are secondary-side; n enters the ODE explicitly.
struct ClllcParams {
    double Lr1 = 0;
    double Lr2 = 0;
    double Lm  = 0;
    double Cr1 = 0;
    double Cr2 = 0;
    double n   = 1;
    double rLr1 = 1e-3;
    double rLr2 = 1e-3;
    double k_pri = 1.0;
    double k_sec = 1.0;
};

// MKF Clllc.cpp:378 — dx/dt at state x with raw bridge voltages (scaled by k_pri / k_sec). Per the KVL
// derivation (MKF Clllc.cpp:340-351), eliminating v_pri gives the 2×2 coupled-inductance system
// M_L = [[Lr1+Lm, −Lm/n], [−Lm/n, Lr2+Lm/n²]] which is inverted in closed form here (det = |M_L|).
ClllcState clllc_dxdt(const ClllcState& x, double uHV_raw, double uLV_raw, const ClllcParams& p) {
    double uHV = p.k_pri * uHV_raw;
    double uLV = p.k_sec * uLV_raw;
    double Lm_n  = p.Lm / p.n;
    double Lm_n2 = p.Lm / (p.n * p.n);
    double det = (p.Lr1 + p.Lm) * (p.Lr2 + Lm_n2) - Lm_n * Lm_n;
    if (det <= 0)
        throw std::runtime_error("analytical_clllc: coupled-L matrix singular");

    double rhs1 = uHV - x.vCr1 - p.rLr1 * x.iLr1;
    double rhs2 = -x.vCr2 - uLV - p.rLr2 * x.iLr2;

    ClllcState d;
    d.iLr1 = ((p.Lr2 + Lm_n2) * rhs1 + Lm_n * rhs2) / det;
    d.iLr2 = (Lm_n * rhs1 + (p.Lr1 + p.Lm) * rhs2) / det;
    d.vCr1 = x.iLr1 / p.Cr1;
    d.vCr2 = x.iLr2 / p.Cr2;
    return d;
}

// MKF Clllc.cpp:399 — a + s·b.
inline ClllcState clllc_state_axpy(const ClllcState& a, const ClllcState& b, double s) {
    return ClllcState{a.iLr1 + s*b.iLr1, a.iLr2 + s*b.iLr2,
                      a.vCr1 + s*b.vCr1, a.vCr2 + s*b.vCr2};
}

// MKF Clllc.cpp:405 — single RK4 step of size dt with constant bridge inputs.
ClllcState clllc_rk4_step(const ClllcState& x, double dt, double uHV, double uLV, const ClllcParams& p) {
    ClllcState k1 = clllc_dxdt(x, uHV, uLV, p);
    ClllcState k2 = clllc_dxdt(clllc_state_axpy(x, k1, dt * 0.5), uHV, uLV, p);
    ClllcState k3 = clllc_dxdt(clllc_state_axpy(x, k2, dt * 0.5), uHV, uLV, p);
    ClllcState k4 = clllc_dxdt(clllc_state_axpy(x, k3, dt), uHV, uLV, p);
    return ClllcState{
        x.iLr1 + dt / 6.0 * (k1.iLr1 + 2*k2.iLr1 + 2*k3.iLr1 + k4.iLr1),
        x.iLr2 + dt / 6.0 * (k1.iLr2 + 2*k2.iLr2 + 2*k3.iLr2 + k4.iLr2),
        x.vCr1 + dt / 6.0 * (k1.vCr1 + 2*k2.vCr1 + 2*k3.vCr1 + k4.vCr1),
        x.vCr2 + dt / 6.0 * (k1.vCr2 + 2*k2.vCr2 + 2*k3.vCr2 + k4.vCr2),
    };
}

// MKF Clllc.cpp:422 — propagate a half period with both bridges in the +polarity. Caller produces the
// negative half by mirror symmetry (half-wave antisymmetry of the steady state).
ClllcState clllc_propagate_half(ClllcState x0, double Thalf, double VHV, double VLV,
                                const ClllcParams& p, int N) {
    double dt = Thalf / N;
    ClllcState x = x0;
    for (int k = 0; k < N; ++k) {
        x = clllc_rk4_step(x, dt, +VHV, +VLV, p);
    }
    return x;
}

// MKF Clllc.cpp:434 — 4×4 linear solve via Gaussian elimination with partial pivoting. Throws on singular.
void clllc_solve4x4(double A[4][4], double b[4], double out[4]) {
    double M[4][5];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) M[i][j] = A[i][j];
        M[i][4] = b[i];
    }
    for (int col = 0; col < 4; ++col) {
        int piv = col;
        double mx = std::abs(M[col][col]);
        for (int r = col + 1; r < 4; ++r) {
            if (std::abs(M[r][col]) > mx) { mx = std::abs(M[r][col]); piv = r; }
        }
        if (mx < 1e-30)
            throw std::runtime_error("analytical_clllc: steady-state (M+I) singular (lossless tank?)");
        if (piv != col) for (int k = 0; k < 5; ++k) std::swap(M[col][k], M[piv][k]);
        for (int r = col + 1; r < 4; ++r) {
            double f = M[r][col] / M[col][col];
            for (int k = col; k < 5; ++k) M[r][k] -= f * M[col][k];
        }
    }
    for (int i = 3; i >= 0; --i) {
        double s = M[i][4];
        for (int j = i + 1; j < 4; ++j) s -= M[i][j] * out[j];
        out[i] = s / M[i][i];
    }
}

// MKF Clllc.cpp:465 — one-shot steady state x0 such that P(x0) = −x0 (half-wave antisymmetry). P is affine:
// P(x0) = M·x0 + g. Measure g (= P(0)) and the four columns of M (= P(e_i) − g) by five RK4 passes, then
// solve (M + I)·x0 = −g.
ClllcState clllc_solve_steady_state(double Thalf, double VHV, double VLV, const ClllcParams& p, int N) {
    ClllcState zero;
    ClllcState ge = clllc_propagate_half(zero, Thalf, VHV, VLV, p, N);
    double g[4] = {ge.iLr1, ge.iLr2, ge.vCr1, ge.vCr2};

    double M[4][4];
    for (int i = 0; i < 4; ++i) {
        ClllcState ei;
        if (i == 0) ei.iLr1 = 1.0;
        else if (i == 1) ei.iLr2 = 1.0;
        else if (i == 2) ei.vCr1 = 1.0;
        else ei.vCr2 = 1.0;
        ClllcState end = clllc_propagate_half(ei, Thalf, VHV, VLV, p, N);
        M[0][i] = end.iLr1 - g[0];
        M[1][i] = end.iLr2 - g[1];
        M[2][i] = end.vCr1 - g[2];
        M[3][i] = end.vCr2 - g[3];
    }

    double A[4][4];
    double rhs[4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) A[i][j] = M[i][j] + (i == j ? 1.0 : 0.0);
        rhs[i] = -g[i];
    }
    double x0[4];
    clllc_solve4x4(A, rhs, x0);
    return ClllcState{x0[0], x0[1], x0[2], x0[3]};
}

}  // anonymous namespace

// Ported from MKF converter_models/Clllc.cpp:606 (process_operating_point_for_input_voltage — the FORWARD
// power-flow branch). MKF member/accessor reads become explicit scalar params (KH's MAS has no
// ClllcOperatingPoint): op.get_switching_frequency() → switchingFrequency, computedPrimarySeriesInductance
// → primaryResonantInductance, computedSecondarySeriesInductance → secondaryResonantInductance,
// computedMagnetizingInductance → magnetizingInductance, turnsRatios[0] → n. `bridgeVoltageFactor` sets BOTH
// k_pri and k_sec (KH exposes a single factor — the symmetric-CLLLC assumption that primary and secondary
// bridges are the same type; MKF derives them independently from bridge_type_primary/secondary). MKF's
// ZVS / mode-classification / current-sharing DIAGNOSTIC members, the extra-component (Cr/Lr) waveforms,
// the half-wave-antisymmetry residual bookkeeping, and the REVERSE power-flow branch (KH has no
// power-flow-direction input; design_clllc is forward-only) are omitted. Missing / non-positive required
// inputs THROW (mirroring MKF's own guards — no fabricated defaults).
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
                                     double bridgeVoltageFactor,
                                     SrcRectifier rectifier) {
    using Lbl = MAS::WaveformLabel;
    // MKF Clllc.cpp:613 (missing voltages/currents). KH additionally requires matched vector lengths.
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_clllc: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");

    const double fsw = switchingFrequency;
    if (!std::isfinite(fsw) || fsw <= 0)   // MKF Clllc.cpp:618
        throw std::invalid_argument("analytical_clllc: switching frequency must be > 0");

    const double n = turnsRatios[0];       // MKF Clllc.cpp:627
    if (!std::isfinite(n) || n <= 0)       // MKF Clllc.cpp:628
        throw std::invalid_argument("analytical_clllc: turns ratio must be > 0");

    // FORWARD power flow only (MKF Clllc.cpp:638-644 reverse branch omitted): the HV bridge is active and
    // the LV bridge synchronous-rectifies. outputVoltages[0] is the LV bus; inputVoltage the HV bus.
    const double Vlv = outputVoltages[0];  // MKF Clllc.cpp:638
    const double Vhv = inputVoltage;       // MKF Clllc.cpp:639
    if (Vlv <= 0 || Vhv <= 0)              // MKF Clllc.cpp:645
        throw std::invalid_argument("analytical_clllc: HV/LV bus voltages must be positive");

    // Tank parameters (MKF Clllc.cpp:648-657 forward branch). Lr2/Cr2 are the ACTUAL secondary-side values.
    ClllcParams p;
    p.Lr1 = primaryResonantInductance;
    p.Lr2 = secondaryResonantInductance;
    p.Cr1 = primaryResonantCapacitance;
    p.Cr2 = secondaryResonantCapacitance;
    // MKF Clllc.cpp:622 (computed tank must be positive) — here the tank arrives as explicit params.
    if (p.Lr1 <= 0 || p.Cr1 <= 0 || p.Lr2 <= 0 || p.Cr2 <= 0)
        throw std::invalid_argument("analytical_clllc: resonant tank values invalid (Lr1/Cr1/Lr2/Cr2 > 0)");
    p.n     = n;
    p.k_pri = bridgeVoltageFactor;   // MKF Clllc.cpp:656-657 (k_pri, k_sec; KH uses one factor for both)
    p.k_sec = bridgeVoltageFactor;
    p.Lm    = magnetizingInductance; // MKF Clllc.cpp:669
    if (!std::isfinite(p.Lm) || p.Lm <= 0)   // MKF Clllc.cpp:671
        throw std::invalid_argument("analytical_clllc: magnetizing inductance must be > 0");
    p.rLr1 = 1e-3;   // MKF Clllc.cpp:673-674 — 1 mΩ damping; prevents (M+I) singularity at exact resonance.
    p.rLr2 = 1e-3;

    const double VHV_drv = Vhv;   // MKF Clllc.cpp:676-677 (forward: no active/passive swap)
    const double VLV_drv = Vlv;

    const double Thalf = 0.5 / fsw;   // MKF Clllc.cpp:679
    constexpr int N = 512;            // MKF Clllc.cpp:680
    const double dt = Thalf / N;

    ClllcState x0 = clllc_solve_steady_state(Thalf, VHV_drv, VLV_drv, p, N);   // MKF Clllc.cpp:683

    // Re-propagate to fill the positive-half waveforms (MKF Clllc.cpp:685-701).
    std::vector<double> iLr1H(N + 1), iLr2H(N + 1), iLmH(N + 1), vCr1H(N + 1), vCr2H(N + 1);
    auto fill_at = [&](int k, const ClllcState& s) {
        iLr1H[k] = s.iLr1;
        iLr2H[k] = s.iLr2;
        iLmH[k]  = s.iLr1 - s.iLr2 / p.n;   // ampere-turn balance
        vCr1H[k] = s.vCr1;
        vCr2H[k] = s.vCr2;
    };
    ClllcState x = x0;
    fill_at(0, x);
    for (int k = 1; k <= N; ++k) {
        x = clllc_rk4_step(x, dt, +VHV_drv, +VLV_drv, p);
        fill_at(k, x);
    }

    // Build the full-period (2N+1 samples) by mirroring the negative half (MKF Clllc.cpp:710-724).
    const int total = 2 * N + 1;
    std::vector<double> tF(total), iLr1F(total), iLr2F(total), iLmF(total),
                        vCr1F(total), vCr2F(total), vPriF(total), vSecF(total);
    for (int k = 0; k <= N; ++k) {
        tF[k] = k * dt;
        iLr1F[k] = iLr1H[k]; iLr2F[k] = iLr2H[k]; iLmF[k] = iLmH[k];
        vCr1F[k] = vCr1H[k]; vCr2F[k] = vCr2H[k];
    }
    for (int k = 1; k <= N; ++k) {
        tF[N + k]    = Thalf + k * dt;
        iLr1F[N + k] = -iLr1H[k]; iLr2F[N + k] = -iLr2H[k]; iLmF[N + k] = -iLmH[k];
        vCr1F[N + k] = -vCr1H[k]; vCr2F[N + k] = -vCr2H[k];
    }

    // Recover the primary/secondary winding voltages from the ODE: v_pri = Lm·di_Lm/dt (MKF Clllc.cpp:726-735).
    for (int k = 0; k < total; ++k) {
        double uHV_full = (k <= N) ? +VHV_drv : -VHV_drv;
        double uLV_full = (k <= N) ? +VLV_drv : -VLV_drv;
        ClllcState xk{iLr1F[k], iLr2F[k], vCr1F[k], vCr2F[k]};
        ClllcState d = clllc_dxdt(xk, uHV_full, uLV_full, p);
        double diLm = d.iLr1 - d.iLr2 / p.n;
        vPriF[k] = p.Lm * diLm;
        vSecF[k] = vPriF[k] / p.n;
    }

    // ── Excitations: exactly MKF's winding set (MKF Clllc.cpp:851-866, forward frame): Primary + Secondary.
    MAS::OperatingPoint operatingPoint;
    // PRIMARY winding (MKF Clllc.cpp:852-860): current = i_Lr1 (HV-side series tank current), voltage =
    // v_pri (the primary magnetizing voltage the core sees — not the bridge voltage).
    {
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(iLr1F);
        currentWaveform.set_time(tF);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(vPriF);
        voltageWaveform.set_time(tF);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, fsw, "Primary"));
    }
    // SECONDARY winding. MKF Clllc.cpp:861-866 pushes a SINGLE full-wave "Secondary": current = i_Lr2
    // (LV-side series tank current), voltage = v_sec = v_pri/n. FULL_BRIDGE (default) reproduces that exact
    // MKF winding. CENTER_TAPPED (family-consistent with analytical_llc/_cllc/_src) instead splits i_Lr2
    // into two polarity half-windings.
    if (rectifier == SrcRectifier::CENTER_TAPPED) {
        for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
            std::vector<double> iSec(total, 0.0), vSec(total, 0.0);
            for (int k = 0; k < total; ++k) {
                double Id = std::isfinite(iLr2F[k]) ? iLr2F[k] : 0.0;
                iSec[k] = (halfIdx == 0) ? std::max(0.0, Id) : std::max(0.0, -Id);
                vSec[k] = std::isfinite(vSecF[k]) ? vSecF[k] : 0.0;
            }
            MAS::Waveform currentWaveform;
            currentWaveform.set_ancillary_label(Lbl::CUSTOM);
            currentWaveform.set_data(iSec); currentWaveform.set_time(tF);
            MAS::Waveform voltageWaveform;
            voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
            voltageWaveform.set_data(vSec); voltageWaveform.set_time(tF);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(currentWaveform, voltageWaveform, fsw,
                                        "Secondary 0 Half " + std::to_string(halfIdx + 1)));
        }
    } else {
        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(iLr2F); currentWaveform.set_time(tF);
        MAS::Waveform voltageWaveform;
        voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
        voltageWaveform.set_data(vSecF); voltageWaveform.set_time(tF);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, fsw, "Secondary 0"));
    }

    return operatingPoint;
}

} // namespace analytical
} // namespace Kirchhoff
