#include "ConverterAnalytical.hpp"
#include "processors/WaveformProcessor.h"   // the shared DSP (MKF), reused — not re-implemented

#include <algorithm>
#include <cmath>
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

} // namespace analytical
} // namespace Kirchhoff
