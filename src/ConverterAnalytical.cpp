#include "ConverterAnalytical.hpp"
#include "processors/WaveformProcessor.h"   // the shared DSP (MKF), reused — not re-implemented
#include "PwmBridgeSolver.h"  // phase-shifted-bridge kernel — RELOCATED into KH (was MKF converter_models/)

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

// --- build_<topo>_tas bridge helpers (see header) ------------------------------------------------------
namespace {
// Emit the minimal, schema-valid processed side (current/voltage) into `dst[side]` from a SignalDescriptor.
void emit_processed(nlohmann::json& dst, const char* side, const std::optional<MAS::SignalDescriptor>& sig) {
    if (!sig) return;
    auto proc = sig->get_processed();
    if (!proc) return;
    nlohmann::json p;
    p["label"]  = nlohmann::json(proc->get_label());   // WaveformLabel enum (required)
    p["offset"] = proc->get_offset();                  // double (required)
    if (proc->get_peak())         p["peak"]       = *proc->get_peak();
    if (proc->get_rms())          p["rms"]        = *proc->get_rms();
    if (proc->get_peak_to_peak()) p["peakToPeak"] = *proc->get_peak_to_peak();
    if (proc->get_duty_cycle())   p["dutyCycle"]  = *proc->get_duty_cycle();
    dst[side]["processed"] = std::move(p);
}
}  // namespace

std::vector<nlohmann::json> excitations_processed(const MAS::OperatingPoint& op) {
    std::vector<nlohmann::json> out;
    for (const auto& e : op.get_excitations_per_winding()) {
        nlohmann::json j;
        j["frequency"] = e.get_frequency();
        if (e.get_name()) j["name"] = *e.get_name();
        emit_processed(j, "current", e.get_current());
        emit_processed(j, "voltage", e.get_voltage());
        out.push_back(std::move(j));
    }
    return out;
}

namespace {
std::vector<std::pair<std::string, nlohmann::json>>& captured_registry() {
    static thread_local std::vector<std::pair<std::string, nlohmann::json>> reg;
    return reg;
}
}  // namespace

void clear_captured_operating_points() { captured_registry().clear(); }

void restamp_captured_ambient(double ambientTemperature) {
    // Overwrite the ambient the capturing overload stamped (its 25 C default) with the build's real ambient,
    // once the api layer has read it from the spec. Keeps the registry ambient consistent with the TAS.
    for (auto& kv : captured_registry())
        kv.second["conditions"]["ambientTemperature"] = ambientTemperature;
}

const std::vector<std::pair<std::string, nlohmann::json>>& captured_operating_points() {
    return captured_registry();
}

std::vector<nlohmann::json> excitations_processed(const MAS::OperatingPoint& op, const std::string& component,
                                                  double ambientTemperature) {
    // Stamp the ambient temperature onto a copy's conditions before serializing: the analytical solvers
    // build only excitations, so the default-constructed conditions block otherwise carries an
    // uninitialized (denormal-garbage) ambient temperature into the registry.
    MAS::OperatingPoint stamped = op;
    stamped.get_mutable_conditions().set_ambient_temperature(ambientTemperature);
    nlohmann::json full = stamped;   // MAS-typed serialization: waveforms + harmonics + processed, schema-shaped
    captured_registry().emplace_back(component, std::move(full));
    return excitations_processed(op);
}

namespace {
double processed_of(const MAS::SignalDescriptor* sig, const std::string& field, const char* side,
                    std::size_t w) {
    if (!sig) throw std::runtime_error("winding stress: winding " + std::to_string(w) + " has no " + side);
    auto proc = sig->get_processed();
    if (!proc) throw std::runtime_error("winding stress: winding " + std::to_string(w) + " " + side
                                        + " has no processed data");
    std::optional<double> v;
    if (field == "peak") v = proc->get_peak();
    else if (field == "rms") v = proc->get_rms();
    else if (field == "offset") v = proc->get_offset();
    else if (field == "peakToPeak") v = proc->get_peak_to_peak();
    else if (field == "dutyCycle") v = proc->get_duty_cycle();
    else throw std::runtime_error("winding stress: unknown field '" + field + "'");
    if (!v) throw std::runtime_error("winding stress: winding " + std::to_string(w) + " " + side + " has no "
                                     + field);
    return *v;
}
}  // namespace

double winding_current(const MAS::OperatingPoint& op, std::size_t w, const std::string& field) {
    const auto& excs = op.get_excitations_per_winding();
    if (w >= excs.size()) throw std::runtime_error("winding_current: winding index out of range");
    auto cur = excs[w].get_current();
    return processed_of(cur ? &*cur : nullptr, field, "current", w);
}

double winding_voltage(const MAS::OperatingPoint& op, std::size_t w, const std::string& field) {
    const auto& excs = op.get_excitations_per_winding();
    if (w >= excs.size()) throw std::runtime_error("winding_voltage: winding index out of range");
    auto vol = excs[w].get_voltage();
    return processed_of(vol ? &*vol : nullptr, field, "voltage", w);
}

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
    // CCM/DCM detection convention (shared by the buck-derived-output solvers: two-switch forward,
    // push-pull, …). minimumPrimaryCurrent is the DCM discriminant: DCM is entered when it goes negative,
    // i.e. when the reflected load current falls below the magnetizing peak (Σ minSec/n < mag/2). This is
    // NOT the textbook output-inductor DCM boundary (Io < ΔIL/2 for a fixed Lo — Erickson & Maksimović §5),
    // because the model represents the output ripple as load-PROPORTIONAL (currentRippleRatio*Io), so minSec
    // never reaches zero. That representation is EXACT at the rated operating point these solvers are called
    // at (it is how Lo was sized) and only diverges under a light-load sweep, which the solvers don't do.
    // Accurate light-load DCM would need load-INDEPENDENT ripple (ΔIL = V_L·t_on/Lo) plus a minSec<0 test —
    // a deliberate cross-solver redesign, intentionally not undertaken here.

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
    // Demagnetization winding — inverted voltage polarity. Its current is the UNIPOLAR magnetizing-reset
    // pulse (0 during the on-time, then magnetizationCurrent ramping back to 0 during the reset interval),
    // so its baseline offset is 0 — NOT minimumPrimaryCurrent, which folds in the reflected secondary LOAD
    // current that flows only in the primary/secondaries, never in the diode-clamped demag winding.
    {
        MAS::Waveform currentWaveform = WP::create_waveform(Lbl::FLYBACK_SECONDARY_WITH_DEADTIME, magnetizationCurrent,
                                                            switchingFrequency, actualDutyCycle, 0.0, deadTime);
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
    // DCM discriminant: minimumPrimaryCurrent < 0 (reflected load below the magnetizing peak). See the
    // CCM/DCM convention note in analytical_forward — load-proportional ripple, exact at the rated point.
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
                                        double diodeVoltageDrop, double efficiency,
                                        bool bridgeVariant) {
    using Lbl = MAS::WaveformLabel;
    double dutyCycle = weinberg_duty_cycle(inputVoltage, outputVoltage, turnsRatio, diodeVoltageDrop, efficiency, 0.95);
    int regime = weinberg_detect_operating_regime(dutyCycle);
    double overlap = std::max(0.0, 2.0 * dutyCycle - 1.0);
    double M = (regime == 2) ? weinberg_conversion_ratio_boost(dutyCycle, turnsRatio)
                             : weinberg_conversion_ratio_buck(dutyCycle, turnsRatio);

    // Input current from power balance: Vin·Iin = Vout·Iout/η, and M = Vout/Vin (conversion ratio), so
    // Iin = Iout·M/η. (The earlier Iout/(η·M) was inverted — ~M² too small; confirmed vs the ngspice deck,
    // which draws Iin = Pin/Vin ≈ Iout·M and splits it Iin/2 per L1 / push-pull-primary winding.)
    double inputCurrent = (M > 0) ? outputCurrent * M / efficiency : 0.0;
    double inductanceL1 = inductance;
    double deltaIL1 = inductanceL1 > 0 ? (inputVoltage * std::max(overlap, dutyCycle)) / (inductanceL1 * switchingFrequency) : 0.0;

    const double period = 1.0 / switchingFrequency;
    const double n = turnsRatio;
    const double D = dutyCycle;
    const double Iin = inputCurrent;
    const double halfDeltaIL1 = 0.5 * deltaIL1;

    // Weinberg carries TWO magnetics; this solver emits all SIX windings in a fixed order so the build can
    // slice them: [0,1] = input coupled inductor L1 (two 1:1 halves), [2,3] = T1 push-pull primary halves,
    // [4,5] = T1 push-pull secondary halves. Per-winding currents match the current-fed push-pull ngspice
    // deck: each primary/L1 half is a trapezoidal PULSE (peak Iin) over its switch-on window D·T; each
    // secondary half conducts ONLY during single-conduction (1−D)·T — during the overlap both primaries
    // drive and the center-tapped secondary voltage cancels, so no secondary current. The two halves (a/b)
    // are phase-shifted by T/2. L1's windings share sense (both +Iin/2); the T1 push-pull halves are
    // opposite-wound, so their DC offsets take OPPOSITE sign → net transformer DC-MMF ~0 (as measured).
    const double tOv  = std::max(0.0, 2.0 * D - 1.0) * period / 2.0;   // overlap duration
    const double tOnP = std::min(D, 1.0) * period;                     // primary switch-on window
    const double tSec = std::max(0.0, 1.0 - D) * period;               // secondary single-conduction window
    auto priPulse = [&](double t) -> double {
        if (t < 0.0 || t > tOnP || tOnP <= 0.0) return 0.0;
        const double lvl = Iin + halfDeltaIL1 * (2.0 * t / tOnP - 1.0);  // small ripple across the pulse
        if (tOv > 0.0 && t < tOv)        return lvl * (t / tOv);         // leading-overlap ramp
        if (tOv > 0.0 && t > tOnP - tOv) return lvl * ((tOnP - t) / tOv);
        return lvl;
    };
    auto secPulse = [&](double t) -> double {
        const double ts = t - tOv;
        if (ts < 0.0 || ts > tSec || tSec <= 0.0) return 0.0;
        const double lvl = Iin * n;
        const double edge = 0.10 * tSec;                                // short ramps (trapezoid, not boxcar)
        if (edge > 0.0 && ts < edge)        return lvl * (ts / edge);
        if (edge > 0.0 && ts > tSec - edge) return lvl * ((tSec - ts) / edge);
        return lvl;
    };
    const int N = 256;
    const double dt = period / N;
    auto wrap = [&](double t){ t = std::fmod(t, period); return t < 0 ? t + period : t; };
    std::vector<double> iTime(N + 1), iPriA(N + 1), iPriB(N + 1), iSecA(N + 1), iSecB(N + 1);
    for (int k = 0; k <= N; ++k) {
        const double t = k * dt;
        iTime[k] = t;
        iPriA[k] = priPulse(t);
        iPriB[k] = priPulse(wrap(t + 0.5 * period));   // L1 half b: same sense, phase-shifted T/2
        iSecA[k] = secPulse(t);
        iSecB[k] = secPulse(wrap(t + 0.5 * period));
    }
    std::vector<double> iPriBneg(N + 1), iSecBneg(N + 1);   // opposite-wound T1 halves
    for (int k = 0; k <= N; ++k) { iPriBneg[k] = -iPriB[k]; iSecBneg[k] = -iSecB[k]; }

    // L1 input inductor voltage: charges at +Vin for the effective fraction dEff, resets at −vL1Reset
    // (volt-second balanced). dEff = max(2D−1, D) (the boost-overlap charge fraction).
    const double dEff = std::max(2.0 * dutyCycle - 1.0, dutyCycle);
    const double vL1Reset = (dEff < 1.0) ? inputVoltage * dEff / (1.0 - dEff) : inputVoltage;
    auto customCurrent = [&](const std::vector<double>& data) {
        MAS::Waveform w; w.set_ancillary_label(Lbl::CUSTOM); w.set_data(data); w.set_time(iTime); return w; };

    MAS::OperatingPoint operatingPoint;
    auto& exc = operatingPoint.get_mutable_excitations_per_winding();
    // [0,1] L1 coupled inductor (two halves, same sense): pulse current, voltage +Vin (dEff) / −vL1Reset.
    MAS::Waveform vL1 = WP::create_waveform(Lbl::RECTANGULAR, inputVoltage + vL1Reset, switchingFrequency, dEff, 0.0, 0);
    exc.push_back(WP::complete_excitation(customCurrent(iPriA), vL1, switchingFrequency, "L1 primary a"));
    exc.push_back(WP::complete_excitation(customCurrent(iPriB), vL1, switchingFrequency, "L1 primary b"));
    // [2,3] T1 push-pull primary halves (opposite-wound → opposite DC sign), voltage ±n·Vout
    // (n = Np/Ns; the secondary is diode-clamped to ±Vout, so the primary reflects to n·Vout).
    // The bridge variant (ABT #88) drives the same two halves in SERIES from a 4-switch H-bridge; the
    // per-half reflected voltage and current are unchanged (only the switch that carries them differs),
    // so the excitations are shared and only the description records the drive.
    const char* priDrive = bridgeVariant ? "bridge" : "push-pull";
    MAS::Waveform vPri = WP::create_waveform(Lbl::BIPOLAR_RECTANGULAR, 2.0 * outputVoltage * turnsRatio, switchingFrequency, 0.5, 0.0, 0);
    exc.push_back(WP::complete_excitation(customCurrent(iPriA),    vPri, switchingFrequency, std::string("T1 ") + priDrive + " primary a"));
    exc.push_back(WP::complete_excitation(customCurrent(iPriBneg), vPri, switchingFrequency, std::string("T1 ") + priDrive + " primary b"));
    // [4,5] T1 push-pull secondary halves (opposite-wound), voltage ±Vout.
    MAS::Waveform vSec = WP::create_waveform(Lbl::BIPOLAR_RECTANGULAR, 2.0 * outputVoltage, switchingFrequency, 0.5, 0.0, 0);
    exc.push_back(WP::complete_excitation(customCurrent(iSecA),    vSec, switchingFrequency, "T1 secondary a"));
    exc.push_back(WP::complete_excitation(customCurrent(iSecBneg), vSec, switchingFrequency, "T1 secondary b"));
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
                                    double inductance, double efficiency, FsbbMode mode) {
    using Lbl = MAS::WaveformLabel;
    const double period = 1.0 / switchingFrequency;
    const double maximumDutyCycle = 0.95;
    if (inputVoltage <= 0) throw std::invalid_argument("analytical_fsbb: input voltage must be > 0");
    if (outputVoltage <= 0) throw std::invalid_argument("analytical_fsbb: output voltage must be > 0");
    if (inductance <= 0) throw std::invalid_argument("analytical_fsbb: inductance must be > 0");

    double dutyForWaveform, dIL, iL_avg, primaryVoltagePtp;
    if (mode == FsbbMode::SIMULTANEOUS) {
        // 4-switch simultaneous buck-boost (the Kirchhoff deck's mode): all four switches commute
        // each cycle. Charge phase (Q1+Q4) applies +Vin across L for D·T; discharge phase (Q2+Q3)
        // applies −Vo for (1−D)·T. Gain M = D/(1−D) = Vo/Vin ⇒ D = Vo/(Vin+Vo). Regular at Vo==Vin.
        const double D = outputVoltage / (inputVoltage + outputVoltage);
        iL_avg = outputCurrent / (1.0 - D);            // buck-boost inductor average current
        dIL = inputVoltage * (D * period) / inductance;  // charge phase: Vin across L for D·T
        primaryVoltagePtp = inputVoltage + outputVoltage;  // +Vin (D) / −Vo (1−D), volt-second balanced
        dutyForWaveform = D;
    } else if (outputVoltage < inputVoltage) {  // BUCK
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
    const std::vector<double>& turnsRatios, const char* who,
    SrcRectifier rectifier = SrcRectifier::CENTER_TAPPED) {
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
    // ---- Secondary winding(s), one set per output, per the rectifier topology ----
    // FULL_BRIDGE: ONE winding conducting on BOTH half-cycles — the diode bridge full-wave
    //   rectifies, so the winding carries the (reflected) output-inductor current with its
    //   direction reversing each half-cycle (bipolar ±ILo) at the full bipolar ±Vsec square.
    // CENTER_TAPPED: two half-windings, each conducting ILo on alternate half-cycles and
    //   reverse-blocking (zero current, −Vsec) on the other.
    for (size_t secIdx = 0; secIdx < turnsRatios.size(); ++secIdx) {
        double ni = turnsRatios[secIdx];
        if (ni <= 0) continue;
        double VsecPk = Vbus / ni;
        if (rectifier == SrcRectifier::FULL_BRIDGE) {
            std::vector<double> v(totalSamples), i(totalSamples);
            for (int k = 0; k < totalSamples; ++k) {
                double vpri = Vpri_full[k];
                double iLo_k = ILo_full[k];
                v[k] = (vpri > 0) ? VsecPk : (vpri < 0 ? -VsecPk : 0.0);
                i[k] = (vpri > 0) ? iLo_k  : (vpri < 0 ? -iLo_k  : 0.0);
            }
            MAS::Waveform iWfm; iWfm.set_ancillary_label(Lbl::CUSTOM); iWfm.set_data(i); iWfm.set_time(time_full);
            MAS::Waveform vWfm; vWfm.set_ancillary_label(Lbl::CUSTOM); vWfm.set_data(v); vWfm.set_time(time_full);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(iWfm, vWfm, Fs, "Secondary " + std::to_string(secIdx)));
            continue;
        }
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
                                    double phaseShiftDegrees, double diodeVoltageDrop,
                                    SrcRectifier rectifier) {
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
                                         outputInductance, D_cmd, freewheelTau, turnsRatios, "analytical_psfb",
                                         rectifier);
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
                                    double phaseShiftDegrees, double diodeVoltageDrop,
                                    SrcRectifier rectifier) {
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
                                         outputInductance, D_cmd, freewheelTau, turnsRatios, "analytical_pshb",
                                         rectifier);
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
                                                      double diodeVoltageDrop, SrcRectifier rectifier) {
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

    if (numOutputs > 1 && totalPower > 0.0) {
        // V6 multi-output: per-output load-share secondaries (share_k = Vo_k·Io_k/ΣVo·Io;
        // i_sec_k = share_k·n_k·iPri) — ONE winding per output regardless of rectifier.
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
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(wfm(iSec_k, time), wfm(vSec_k, time), fsw,
                                        "Secondary " + std::to_string(k)));
        }
    } else if (rectifier == SrcRectifier::FULL_BRIDGE) {
        // Full-bridge rectifier: ONE secondary winding. The bridge routes the full output-inductor
        // current through the winding at all times (2 diodes conduct), with the winding current sign
        // following the primary polarity: i_sec = sign(v_pri)·i_Lo = iSec_a − iSec_b, at the full ±Vsec
        // square (= vSec_a). AHB has NO freewheel interval (complementary duty), so the winding conducts
        // the whole period ⇒ RMS ≈ Io, and the ASYMMETRIC duty gives a real DC bias Io·(2D−1) that the
        // gapped AHB (energy-transfer) transformer carries — confirmed against the ngspice deck
        // (secondary avg = Io·(2D−1), rms ≈ Io). The zero-offset inline model understated this bias.
        std::vector<double> iSecFB(iSec_a.size()), vSecFB(vSec_a.size());
        for (size_t k = 0; k < iSec_a.size(); ++k) { iSecFB[k] = iSec_a[k] - iSec_b[k]; vSecFB[k] = vSec_a[k]; }
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(wfm(iSecFB, time), wfm(vSecFB, time), fsw, "Secondary 0"));
    } else {
        // Center-tapped rectifier: two polarity-split half-windings.
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(wfm(iSec_a, time), wfm(vSec_a, time), fsw, "Secondary 0a"));
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(wfm(iSec_b, time), wfm(vSec_b, time), fsw, "Secondary 0b"));
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

// General DAB average power transfer for ANY modulation (SPS/EPS/DPS/TPS): averages Vab·iL over the period,
// reusing the exact subinterval/propagation kernel analytical_dab emits — so this power equals the power the
// emitted waveforms carry (no separate closed form to drift out of sync). D1/D2/D3 in RADIANS. Ported from
// MKF Dab::compute_power_general.
double dab_power_transfer(double V1, double V2, double N, double D3rad, double D1rad, double D2rad,
                          double Fs, double L) {
    auto segs = dab_build_subintervals(V1, V2, D1rad, D2rad, D3rad);
    constexpr double Lm_dummy = 1.0;   // power depends on iL only; Im (needs Lm) is irrelevant here
    double iL0, Im0;
    dab_initial_conditions(segs, N, L, Lm_dummy, Fs, iL0, Im0);
    std::vector<double> bt, bi, bm;
    dab_propagate_period(segs, N, L, Lm_dummy, Fs, iL0, Im0, bt, bi, bm);
    double power = 0.0;
    for (size_t k = 0; k < segs.size(); ++k)
        power += segs[k].Vab * 0.5 * (bi[k] + bi[k + 1]) * (segs[k].theta_end - segs[k].theta_start);
    return power / (2.0 * M_PI);
}

// Series inductance L that delivers `P` at the given modulation (D1/D2/D3 in RADIANS). Power is monotonically
// decreasing in L, so geometric-bisect. Throws if P is unreachable even at minimal L (the modulation cannot
// deliver the requested power — no silent saturation, per the no-fallback rule). Used to size L for
// EPS/DPS/TPS designs so they still deliver the target power; SPS keeps its exact closed form.
double dab_series_inductance_for_power(double V1, double V2, double N, double D3rad, double D1rad,
                                       double D2rad, double Fs, double P) {
    if (P <= 0) throw std::invalid_argument("dab_series_inductance_for_power: power must be positive");
    double lo = 1e-12, hi = 1.0;   // L bounds spanning any realistic tank
    if (dab_power_transfer(V1, V2, N, D3rad, D1rad, D2rad, Fs, lo) < P)
        throw std::runtime_error("analytical_dab: requested power exceeds the achievable maximum at this "
                                 "modulation (D1/D2/D3) — cannot size the series inductance");
    for (int i = 0; i < 200; ++i) {
        double mid = std::sqrt(lo * hi);
        if (dab_power_transfer(V1, V2, N, D3rad, D1rad, D2rad, Fs, mid) > P) lo = mid; else hi = mid;
        if (hi / lo < 1.0 + 1e-10) break;
    }
    return std::sqrt(lo * hi);
}

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
// LLC resonant converter — load-aware First-Harmonic Approximation (FHA).
// REPLACES the earlier faithful port of MKF's lossless event-driven Nielsen TDA, which models an
// infinite-Q, LOAD-BLIND tank: the emitted tank current was independent of Iout (proven: identical rms
// across a 7x load sweep) and ~5x the SPICE tank current, diverging toward fr. FHA folds the load into
// the tank via the AC-equivalent resistance Rac = (8/pi^2)*n^2*Rload, so the primary tank current scales
// with load and matches SPICE (validated by the [nrmse][llc] gate, like analytical_src). Tank: series
// Ls-Cr feeding Lm in parallel with the reflected load Rac.
//   Zin = j(wLs - 1/(wCr)) + (jwLm)||Rac,   ILs_pk = (4/pi)*k_bridge*Vin / |Zin|,  phi = arg(Zin).
// The primary winding carries the resonant current ILs; the magnetizing current ILm is the Lm triangle
// (Lm is clamped to +/-Vo = +/-n*Vout by the secondary rectifier); the secondary carries n*(ILs - ILm)
// rectified. FHA is NOT singular at fr (Ls-Cr cancel, Zin -> Lm||Rac), so it is valid at/above/below fr.
// ─────────────────────────────────────────────────────────────────────────────
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
    (void)integratedResonantInductor;   // in FHA the winding-current shapes do not depend on Ls integration
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_llc: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double fsw = switchingFrequency;
    if (fsw <= 0) throw std::invalid_argument("analytical_llc: switching frequency must be > 0");
    const double Ls = seriesResonantInductance, Cr = resonantCapacitance, Lm = magnetizingInductance;
    if (Lm <= 0) throw std::invalid_argument("analytical_llc: magnetizing inductance must be > 0");
    if (Ls <= 0 || Cr <= 0) throw std::invalid_argument("analytical_llc: resonant Ls/Cr must be > 0");
    const double k_bridge = bridgeVoltageFactor;
    const double n_main = turnsRatios[0];
    if (n_main <= 0) throw std::invalid_argument("analytical_llc: turns ratio must be > 0");

    const size_t nOutputs = outputVoltages.size();
    const double Vout0 = outputVoltages[0], Iout0 = outputCurrents[0];
    if (Vout0 <= 0 || Iout0 <= 0) throw std::invalid_argument("analytical_llc: main output V/I must be > 0");
    const double Vo = n_main * Vout0;                                    // reflected output (magnetizing clamp)
    const double Rload = Vout0 / Iout0;
    const double Rac = (8.0 / (M_PI * M_PI)) * n_main * n_main * Rload;   // FHA AC-equivalent load

    const double w = 2.0 * M_PI * fsw;
    const double XLs = w * Ls, XCr = 1.0 / (w * Cr), XLm = w * Lm;
    // Lm || Rac (complex): Zp = (jXLm.Rac)/(Rac + jXLm) = Rac.XLm^2/D + j.Rac^2.XLm/D,  D = Rac^2 + XLm^2.
    const double D = Rac * Rac + XLm * XLm;
    const double Zp_re = Rac * XLm * XLm / D;
    const double Zp_im = Rac * Rac * XLm / D;
    const double Zin_re = Zp_re;
    const double Zin_im = (XLs - XCr) + Zp_im;
    const double Zin_mag = std::sqrt(Zin_re * Zin_re + Zin_im * Zin_im);
    const double phi = std::atan2(Zin_im, Zin_re);

    const double Vin_fund_pk = (4.0 / M_PI) * k_bridge * inputVoltage;
    const double ILs_pk = (Zin_mag > 0) ? Vin_fund_pk / Zin_mag : 0.0;

    // Magnetizing current: Lm sees +/-Vo square -> triangle, peak = Vo*(Thalf)/(2*Lm).
    const double Thalf = 1.0 / (2.0 * fsw);
    const double ILm_pk = Vo * Thalf / (2.0 * Lm);

    const int N = 256;
    const int totalSamples = 2 * N + 1;
    const double period = 1.0 / fsw;
    const double dt = period / (2 * N);
    std::vector<double> time_full(totalSamples), ILs_full(totalSamples), ILm_full(totalSamples),
        Vpri_full(totalSamples);
    for (int k = 0; k < totalSamples; ++k) {
        const double t = k * dt;
        const double theta = w * t;
        time_full[k] = t;
        ILs_full[k] = ILs_pk * std::sin(theta - phi);
        double thetaMod = std::fmod(theta, 2.0 * M_PI);
        if (thetaMod < 0) thetaMod += 2.0 * M_PI;
        if (thetaMod < M_PI) {                       // +Vo half: iLm ramps -ILm_pk -> +ILm_pk
            ILm_full[k] = -ILm_pk + 2.0 * ILm_pk * (thetaMod / M_PI);
            Vpri_full[k] = +Vo;
        } else {                                     // -Vo half: iLm ramps +ILm_pk -> -ILm_pk
            ILm_full[k] = ILm_pk - 2.0 * ILm_pk * ((thetaMod - M_PI) / M_PI);
            Vpri_full[k] = -Vo;
        }
    }

    MAS::OperatingPoint operatingPoint;
    // Primary: resonant tank current ILs + clamped +/-Vo transformer voltage.
    {
        MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(ILs_full); iW.set_time(time_full);
        MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(Vpri_full); vW.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iW, vW, fsw, "Primary"));
    }
    // Secondaries: rectified reflected tank current I_sec = n*(ILs - ILm)*share. CT -> two half-windings
    // (each conducts one polarity); FB -> one bipolar winding.
    double total_g = 0.0;
    for (size_t i = 0; i < nOutputs; ++i)
        if (outputVoltages[i] > 0 && outputCurrents[i] > 0) total_g += outputCurrents[i] / outputVoltages[i];
    if (total_g <= 0) total_g = 1.0;
    for (size_t outputIdx = 0; outputIdx < nOutputs; ++outputIdx) {
        double n_i = turnsRatios[outputIdx]; if (n_i <= 0) n_i = 1.0;
        const double Vout_i = outputVoltages[outputIdx], Iout_i = outputCurrents[outputIdx];
        const double share = (Vout_i > 0 && Iout_i > 0) ? (Iout_i / Vout_i) / total_g
                                                        : (outputIdx == 0 ? 1.0 : 0.0);
        // Enforce the known DC output: FHA captures the fundamental of the diode current but not its exact
        // conduction phase, so the raw rectified reflected current n*(ILs-ILm) does not integrate to Iout.
        // Scale it so its rectified (full-wave) average equals Iout_i — the boundary condition the output
        // capacitor physically imposes. (The primary tank current ILs is untouched; it is SPICE-validated.)
        double rawOutAvg = 0.0;
        for (int k = 0; k < totalSamples; ++k)
            rawOutAvg += std::abs((ILs_full[k] - ILm_full[k]) * share * n_i);
        rawOutAvg /= totalSamples;
        const double secScale = (rawOutAvg > 1e-12) ? (Iout_i / rawOutAvg) : 1.0;
        if (rectifier == SrcRectifier::CENTER_TAPPED) {
            for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
                std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
                for (int k = 0; k < totalSamples; ++k) {
                    const double Id = (ILs_full[k] - ILm_full[k]) * share;
                    const double Id_half = (halfIdx == 0) ? std::max(0.0, Id) : std::max(0.0, -Id);
                    iSecData[k] = Id_half * n_i * secScale;
                    vSecData[k] = (halfIdx == 0) ? (Vpri_full[k] >= 0 ? +Vout_i : -Vout_i)
                                                 : (Vpri_full[k] >= 0 ? -Vout_i : +Vout_i);
                }
                MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
                MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
                operatingPoint.get_mutable_excitations_per_winding().push_back(
                    WP::complete_excitation(iW, vW, fsw, "Secondary " + std::to_string(outputIdx) +
                                                         " Half " + std::to_string(halfIdx + 1)));
            }
        } else {
            std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
            for (int k = 0; k < totalSamples; ++k) {
                iSecData[k] = (ILs_full[k] - ILm_full[k]) * share * n_i * secScale;
                vSecData[k] = (Vpri_full[k] >= 0 ? +Vout_i : -Vout_i);
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
// CLLC bidirectional resonant converter — load-aware First-Harmonic Approximation (FHA).
// REPLACES the faithful port of MKF's 4-state event-driven TDA, which (like LLC) modelled a lossless,
// LOAD-BLIND tank: proven to emit a tank current independent of Iout (byte-identical rms across a 7x load
// sweep) and ~5x the SPICE tank current. FHA folds the load into the two-sided CLLC tank: primary Lr1-Cr1
// in series, then Lm in parallel with (the secondary Lr2-Cr2 reflected to the primary + the AC-equivalent
// load Rac = (8/pi^2)*n^2*Rload):
//     Zsec = Rac + j(w*Lr2*n^2 - 1/(w*Cr2/n^2)),   Zpar = (j*w*Lm) || Zsec,
//     Zin  = j(w*Lr1 - 1/(w*Cr1)) + Zpar,          ILr1_pk = (4/pi)*k_bridge*Vin / |Zin|.
// The primary winding carries the resonant current ILr1; the magnetizing current is the Lm triangle
// (clamped to +/-Vo = +/-n*Vout); the secondary carries n*(ILr1 - ILm), scaled so its rectified average
// equals the DC output Iout. Load-aware, and not singular at fr. (MKF Cllc.cpp reads → explicit params.)
// ─────────────────────────────────────────────────────────────────────────────
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
    const double fsw = switchingFrequency;
    if (fsw <= 0) throw std::invalid_argument("analytical_cllc: switching frequency must be > 0");
    const double Vout0 = outputVoltages[0], Iout0 = outputCurrents[0];
    const double n = turnsRatios[0], Lm = magnetizingInductance;
    if (n <= 0 || Lm <= 0) throw std::invalid_argument("analytical_cllc: invalid turns ratio or magnetizing inductance");
    if (Vout0 <= 0 || Iout0 <= 0) throw std::invalid_argument("analytical_cllc: main output V/I must be > 0");
    const double Lr1 = primaryResonantInductance, Cr1 = primaryResonantCapacitance;
    const double Lr2s = secondaryResonantInductance, Cr2s = secondaryResonantCapacitance;
    if (Lr1 <= 0 || Cr1 <= 0 || Lr2s <= 0 || Cr2s <= 0)
        throw std::invalid_argument("analytical_cllc: resonant tank values invalid (Lr1/Cr1/Lr2/Cr2 > 0)");

    const double k_bridge = bridgeVoltageFactor;
    const double Vo = n * Vout0;                              // reflected output (magnetizing clamp)
    const double Rload = Vout0 / Iout0;
    const double Rac = (8.0 / (M_PI * M_PI)) * n * n * Rload;
    const double Lr2p = Lr2s * n * n, Cr2p = Cr2s / (n * n);  // secondary tank reflected to primary

    const double w = 2.0 * M_PI * fsw;
    const double Xpri = w * Lr1 - 1.0 / (w * Cr1);            // primary tank reactance
    const double Xsec = w * Lr2p - 1.0 / (w * Cr2p);         // reflected secondary tank reactance
    const double XLm  = w * Lm;
    // Zsec = Rac + jXsec;  Zpar = (jXLm)||Zsec = (jXLm.Zsec)/(jXLm + Zsec).  jXLm.(Rac+jXsec) = -XLm.Xsec + j.XLm.Rac.
    const double num_re = -XLm * Xsec, num_im = XLm * Rac;
    const double den_re = Rac,          den_im = XLm + Xsec;  // (Rac) + j(XLm + Xsec)
    const double denMag2 = den_re * den_re + den_im * den_im;
    const double Zpar_re = (num_re * den_re + num_im * den_im) / denMag2;
    const double Zpar_im = (num_im * den_re - num_re * den_im) / denMag2;
    const double Zin_re = Zpar_re;
    const double Zin_im = Xpri + Zpar_im;
    const double Zin_mag = std::sqrt(Zin_re * Zin_re + Zin_im * Zin_im);
    const double phi = std::atan2(Zin_im, Zin_re);

    const double Vin_fund_pk = (4.0 / M_PI) * k_bridge * inputVoltage;
    const double ILr1_pk = (Zin_mag > 0) ? Vin_fund_pk / Zin_mag : 0.0;

    const double Thalf = 1.0 / (2.0 * fsw);
    const double ILm_pk = Vo * Thalf / (2.0 * Lm);

    const int N = 256;
    const int totalSamples = 2 * N + 1;
    const double period = 1.0 / fsw;
    const double dt = period / (2 * N);
    std::vector<double> time_full(totalSamples), ILr1_full(totalSamples), ILm_full(totalSamples),
        Vpri_full(totalSamples);
    for (int k = 0; k < totalSamples; ++k) {
        const double t = k * dt, theta = w * t;
        time_full[k] = t;
        ILr1_full[k] = ILr1_pk * std::sin(theta - phi);
        double thetaMod = std::fmod(theta, 2.0 * M_PI);
        if (thetaMod < 0) thetaMod += 2.0 * M_PI;
        if (thetaMod < M_PI) { ILm_full[k] = -ILm_pk + 2.0 * ILm_pk * (thetaMod / M_PI); Vpri_full[k] = +Vo; }
        else { ILm_full[k] = ILm_pk - 2.0 * ILm_pk * ((thetaMod - M_PI) / M_PI); Vpri_full[k] = -Vo; }
    }

    MAS::OperatingPoint operatingPoint;
    {
        MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(ILr1_full); iW.set_time(time_full);
        MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(Vpri_full); vW.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iW, vW, fsw, "Primary"));
    }
    // Secondary: n*(ILr1 - ILm), scaled so the rectified average delivers the DC output Iout0.
    double rawOutAvg = 0.0;
    for (int k = 0; k < totalSamples; ++k) rawOutAvg += std::abs((ILr1_full[k] - ILm_full[k]) * n);
    rawOutAvg /= totalSamples;
    const double secScale = (rawOutAvg > 1e-12) ? (Iout0 / rawOutAvg) : 1.0;
    if (rectifier == SrcRectifier::CENTER_TAPPED) {
        for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
            std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
            for (int k = 0; k < totalSamples; ++k) {
                const double Id = ILr1_full[k] - ILm_full[k];
                const double Id_half = (halfIdx == 0) ? std::max(0.0, Id) : std::max(0.0, -Id);
                iSecData[k] = Id_half * n * secScale;
                vSecData[k] = (halfIdx == 0) ? (Vpri_full[k] >= 0 ? +Vout0 : -Vout0)
                                             : (Vpri_full[k] >= 0 ? -Vout0 : +Vout0);
            }
            MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
            MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(iW, vW, fsw, "Secondary 0 Half " + std::to_string(halfIdx + 1)));
        }
    } else {
        std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
        for (int k = 0; k < totalSamples; ++k) {
            iSecData[k] = (ILr1_full[k] - ILm_full[k]) * n * secScale;
            vSecData[k] = (Vpri_full[k] >= 0 ? +Vout0 : -Vout0);
        }
        MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
        MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iW, vW, fsw, "Secondary 0"));
    }
    return operatingPoint;
}

// ─── Phase 5: CLLLC bidirectional resonant converter — load-aware First-Harmonic Approximation (FHA) ──
// REPLACES the faithful port of MKF's RK4 affine-propagator TDA, which (verified) is ALSO load-BLIND: it
// emitted a tank current independent of Iout (byte-identical rms across a load sweep). CLLLC has the same
// C-L-L-L-C tank as CLLC (primary Lr1-Cr1, magnetizing Lm, secondary Lr2-Cr2), so the FHA is identical to
// analytical_cllc's two-sided model: Zin = j(wLr1-1/(wCr1)) + (jwLm)||(Rac + j(wLr2*n^2 - 1/(wCr2/n^2))),
// Rac = (8/pi^2)*n^2*Rload; ILr1_pk = (4/pi)*k_bridge*Vin/|Zin|. Primary carries ILr1; secondary carries
// n*(ILr1-ILm) scaled to deliver Iout. There is no CLLLC reference design/deck, so it is validated
// structurally (load-scaling + zero-mean primary + winding counts) and inherits the CLLC formula's
// SPICE-validated fidelity ([nrmse][cllc] at 0.25). (MKF Clllc.cpp reads → explicit scalar params.)
// ─────────────────────────────────────────────────────────────────────────────
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
    if (outputVoltages.empty() || outputVoltages.size() != outputCurrents.size() ||
        outputVoltages.size() != turnsRatios.size())
        throw std::invalid_argument("analytical_clllc: outputVoltages/outputCurrents/turnsRatios "
                                    "must be non-empty and equal length");
    const double fsw = switchingFrequency;
    if (fsw <= 0) throw std::invalid_argument("analytical_clllc: switching frequency must be > 0");
    const double Vout0 = outputVoltages[0], Iout0 = outputCurrents[0];
    const double n = turnsRatios[0], Lm = magnetizingInductance;
    if (n <= 0 || Lm <= 0) throw std::invalid_argument("analytical_clllc: invalid turns ratio or magnetizing inductance");
    if (Vout0 <= 0 || Iout0 <= 0) throw std::invalid_argument("analytical_clllc: main output V/I must be > 0");
    const double Lr1 = primaryResonantInductance, Cr1 = primaryResonantCapacitance;
    const double Lr2s = secondaryResonantInductance, Cr2s = secondaryResonantCapacitance;
    if (Lr1 <= 0 || Cr1 <= 0 || Lr2s <= 0 || Cr2s <= 0)
        throw std::invalid_argument("analytical_clllc: resonant tank values invalid (Lr1/Cr1/Lr2/Cr2 > 0)");

    const double k_bridge = bridgeVoltageFactor;
    const double Vo = n * Vout0;                              // reflected output (magnetizing clamp)
    const double Rload = Vout0 / Iout0;
    const double Rac = (8.0 / (M_PI * M_PI)) * n * n * Rload;
    const double Lr2p = Lr2s * n * n, Cr2p = Cr2s / (n * n);  // secondary tank reflected to primary

    const double w = 2.0 * M_PI * fsw;
    const double Xpri = w * Lr1 - 1.0 / (w * Cr1);            // primary tank reactance
    const double Xsec = w * Lr2p - 1.0 / (w * Cr2p);         // reflected secondary tank reactance
    const double XLm  = w * Lm;
    // Zsec = Rac + jXsec;  Zpar = (jXLm)||Zsec = (jXLm.Zsec)/(jXLm + Zsec).  jXLm.(Rac+jXsec) = -XLm.Xsec + j.XLm.Rac.
    const double num_re = -XLm * Xsec, num_im = XLm * Rac;
    const double den_re = Rac,          den_im = XLm + Xsec;  // (Rac) + j(XLm + Xsec)
    const double denMag2 = den_re * den_re + den_im * den_im;
    const double Zpar_re = (num_re * den_re + num_im * den_im) / denMag2;
    const double Zpar_im = (num_im * den_re - num_re * den_im) / denMag2;
    const double Zin_re = Zpar_re;
    const double Zin_im = Xpri + Zpar_im;
    const double Zin_mag = std::sqrt(Zin_re * Zin_re + Zin_im * Zin_im);
    const double phi = std::atan2(Zin_im, Zin_re);

    const double Vin_fund_pk = (4.0 / M_PI) * k_bridge * inputVoltage;
    const double ILr1_pk = (Zin_mag > 0) ? Vin_fund_pk / Zin_mag : 0.0;

    const double Thalf = 1.0 / (2.0 * fsw);
    const double ILm_pk = Vo * Thalf / (2.0 * Lm);

    const int N = 256;
    const int totalSamples = 2 * N + 1;
    const double period = 1.0 / fsw;
    const double dt = period / (2 * N);
    std::vector<double> time_full(totalSamples), ILr1_full(totalSamples), ILm_full(totalSamples),
        Vpri_full(totalSamples);
    for (int k = 0; k < totalSamples; ++k) {
        const double t = k * dt, theta = w * t;
        time_full[k] = t;
        ILr1_full[k] = ILr1_pk * std::sin(theta - phi);
        double thetaMod = std::fmod(theta, 2.0 * M_PI);
        if (thetaMod < 0) thetaMod += 2.0 * M_PI;
        if (thetaMod < M_PI) { ILm_full[k] = -ILm_pk + 2.0 * ILm_pk * (thetaMod / M_PI); Vpri_full[k] = +Vo; }
        else { ILm_full[k] = ILm_pk - 2.0 * ILm_pk * ((thetaMod - M_PI) / M_PI); Vpri_full[k] = -Vo; }
    }

    MAS::OperatingPoint operatingPoint;
    {
        MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(ILr1_full); iW.set_time(time_full);
        MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(Vpri_full); vW.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iW, vW, fsw, "Primary"));
    }
    // Secondary: n*(ILr1 - ILm), scaled so the rectified average delivers the DC output Iout0.
    double rawOutAvg = 0.0;
    for (int k = 0; k < totalSamples; ++k) rawOutAvg += std::abs((ILr1_full[k] - ILm_full[k]) * n);
    rawOutAvg /= totalSamples;
    const double secScale = (rawOutAvg > 1e-12) ? (Iout0 / rawOutAvg) : 1.0;
    if (rectifier == SrcRectifier::CENTER_TAPPED) {
        for (size_t halfIdx = 0; halfIdx < 2; ++halfIdx) {
            std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
            for (int k = 0; k < totalSamples; ++k) {
                const double Id = ILr1_full[k] - ILm_full[k];
                const double Id_half = (halfIdx == 0) ? std::max(0.0, Id) : std::max(0.0, -Id);
                iSecData[k] = Id_half * n * secScale;
                vSecData[k] = (halfIdx == 0) ? (Vpri_full[k] >= 0 ? +Vout0 : -Vout0)
                                             : (Vpri_full[k] >= 0 ? -Vout0 : +Vout0);
            }
            MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
            MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(iW, vW, fsw, "Secondary 0 Half " + std::to_string(halfIdx + 1)));
        }
    } else {
        std::vector<double> iSecData(totalSamples, 0.0), vSecData(totalSamples, 0.0);
        for (int k = 0; k < totalSamples; ++k) {
            iSecData[k] = (ILr1_full[k] - ILm_full[k]) * n * secScale;
            vSecData[k] = (Vpri_full[k] >= 0 ? +Vout0 : -Vout0);
        }
        MAS::Waveform iW; iW.set_ancillary_label(Lbl::CUSTOM); iW.set_data(iSecData); iW.set_time(time_full);
        MAS::Waveform vW; vW.set_ancillary_label(Lbl::CUSTOM); vW.set_data(vSecData); vW.set_time(time_full);
        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(iW, vW, fsw, "Secondary 0"));
    }
    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 6: three-phase AC-input PFC — Vienna rectifier.
// Ported from MKF converter_models/Vienna.cpp:556 (process_operating_point_for_input_voltage)
// plus the helpers it calls (compute_phase_peak_voltage :105, compute_modulation_index :111,
// compute_line_peak_current :117, build_line_cycle_waveform :282). Diagnostics-only members are
// omitted (see header). Interleaving (numberOfChannels, MKF's phaseCount) splits each phase's current
// across N_ch channel inductors; peakOfLinePlusSectors adds the six DPWM sector operating points.
// ─────────────────────────────────────────────────────────────────────────────

// MKF Vienna::compute_phase_peak_voltage (Vienna.cpp:105). Expressed with the per-phase
// line-to-NEUTRAL RMS: V_phase_peak = √2·V_phase_rms. Equals MKF's √2·V_LL/√3 since V_LL = √3·V_phase_rms.
static double vienna_phase_peak_voltage(double linePhaseVoltageRms) {
    if (linePhaseVoltageRms <= 0)
        throw std::invalid_argument("analytical_vienna: linePhaseVoltageRms must be > 0");
    return std::sqrt(2.0) * linePhaseVoltageRms;
}

// MKF Vienna::compute_modulation_index (Vienna.cpp:111): M = V_phase_peak / (Vdc/2).
static double vienna_modulation_index(double vPhasePeak, double vdc) {
    if (vdc <= 0)
        throw std::invalid_argument("analytical_vienna: outputDcVoltage must be > 0");
    return vPhasePeak / (vdc / 2.0);
}

// MKF Vienna::compute_line_peak_current (Vienna.cpp:117): I_pk = √2·P / (3·V_phase_rms·η·pf).
static double vienna_line_peak_current(double power, double vPhaseRms, double eff, double pf) {
    if (power <= 0)
        throw std::invalid_argument("analytical_vienna: outputPower must be > 0");
    if (vPhaseRms <= 0)
        throw std::invalid_argument("analytical_vienna: phase RMS voltage must be > 0");
    if (eff <= 0 || eff > 1.0)
        throw std::invalid_argument("analytical_vienna: efficiency must be in (0, 1]");
    if (pf <= 0 || pf > 1.0)
        throw std::invalid_argument("analytical_vienna: powerFactor must be in (0, 1]");
    return std::sqrt(2.0) * power / (3.0 * vPhaseRms * eff * pf);
}

// MKF Vienna::LineCycleKind (Vienna.h) — selects the current vs. inductor-voltage envelope build.
enum class ViennaLineCycleKind { CURRENT, VOLTAGE };

// MKF Vienna::build_line_cycle_waveform (Vienna.cpp:282), transcribed exactly. Builds the full 50/60 Hz
// line-cycle envelope for ONE phase (shifted by phaseOffsetRad), with the per-angle switching-ripple
// triangle superimposed. numSamples default 4096 matches MKF's header default (Vienna.h:242).
static MAS::Waveform vienna_build_line_cycle_waveform(
    ViennaLineCycleKind kind,
    double iPk, double vPhasePeak, double vdc,
    double L, double fsw, double fLine,
    double phaseOffsetRad,
    size_t numSamples = 4096) {
    if (numSamples < 2)
        throw std::invalid_argument("analytical_vienna: numSamples must be >= 2");
    if (fLine <= 0)
        throw std::invalid_argument("analytical_vienna: lineFrequency must be > 0");
    if (fsw <= fLine)
        throw std::invalid_argument("analytical_vienna: switchingFrequency must be > lineFrequency");
    if (L <= 0)
        throw std::invalid_argument("analytical_vienna: boostInductance must be > 0");

    const double T_line = 1.0 / fLine;
    const double T_sw   = 1.0 / fsw;
    const double omega  = 2.0 * M_PI * fLine;
    const double Vhalf  = vdc / 2.0;

    std::vector<double> time(numSamples), data(numSamples);
    for (size_t i = 0; i < numSamples; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(numSamples - 1) * T_line;
        time[i]  = t;

        double theta    = omega * t + phaseOffsetRad;
        double sinTheta = std::sin(theta);
        double Vphase_t = vPhasePeak * sinTheta;
        double Iavg_t   = iPk * sinTheta;

        double dutyAbs = 1.0 - std::abs(Vphase_t) / Vhalf;   // per-angle boost duty (1 − |Vphase|/Vhalf)
        if (dutyAbs < 0.0) dutyAbs = 0.0;
        if (dutyAbs > 1.0) dutyAbs = 1.0;

        double dI_pp = std::abs(Vphase_t) * dutyAbs * T_sw / L;   // local switching-period ripple pk-pk

        double tri = 0.0;   // sub-sampled triangular ripple tri(2π·Fsw·t) ∈ [−1,+1]
        {
            double swPhase = std::fmod(fsw * t, 1.0);
            tri = (swPhase < 0.5) ? (4.0 * swPhase - 1.0) : (3.0 - 4.0 * swPhase);
        }

        if (kind == ViennaLineCycleKind::CURRENT) {
            data[i] = Iavg_t + 0.5 * dI_pp * tri;
        } else {
            double V_on  = Vphase_t;
            double V_off = Vphase_t - ((Vphase_t >= 0) ? Vhalf : -Vhalf);
            data[i] = (tri >= 0) ? V_on : V_off;
        }
    }

    MAS::Waveform wf;
    wf.set_data(data);
    wf.set_time(time);
    return wf;
}

// Ported from MKF converter_models/Vienna.cpp:556 (process_operating_point_for_input_voltage).
MAS::OperatingPoint analytical_vienna(double linePhaseVoltageRms,
                                      double outputDcVoltage,
                                      double outputPower,
                                      double lineFrequency,
                                      double switchingFrequency,
                                      double boostInductance,
                                      double efficiency,
                                      double powerFactor,
                                      bool fullLineCycle,
                                      int numberOfChannels,
                                      bool peakOfLinePlusSectors) {
    using Lbl = MAS::WaveformLabel;

    const double Fsw = switchingFrequency;
    if (Fsw <= 0)
        throw std::invalid_argument("analytical_vienna: switchingFrequency must be > 0");   // MKF Vienna.cpp:566
    if (numberOfChannels < 1)   // MKF Vienna.cpp:420 (phaseCount >= 1); no silent clamp to 1
        throw std::invalid_argument("analytical_vienna: numberOfChannels must be >= 1");
    const double Vdc = outputDcVoltage;

    // MKF Vienna.cpp:573-579: phase peak, modulation index, over-modulation gate.
    const double V_phase_peak = vienna_phase_peak_voltage(linePhaseVoltageRms);
    const double V_phase_rms  = V_phase_peak / std::sqrt(2.0);
    const double M            = vienna_modulation_index(V_phase_peak, Vdc);
    if (M > 1.0)
        throw std::invalid_argument(
            "analytical_vienna: modulation index M=" + std::to_string(M) +
            " > 1 (over-modulation); outputDcVoltage must exceed 2·V_phase_peak");

    // MKF Vienna.cpp:584-593: P = Vout·Iout (here supplied directly as outputPower), I_pk, L.
    const double P    = outputPower;
    const double I_pk = vienna_line_peak_current(P, V_phase_rms, efficiency, powerFactor);
    const double L    = boostInductance;
    if (L <= 0)
        throw std::invalid_argument("analytical_vienna: boostInductance must be > 0");

    // MKF Vienna.cpp:595-600: peak-of-line duty, inductor-voltage levels, switching-ripple pk-pk.
    const double duty_at_peak = 1.0 - M;
    const double V_L_on  = V_phase_peak;             // switch closed: full phase voltage across L
    const double V_L_off = V_phase_peak - Vdc / 2.0; // switch open: rectifier conducts to bus (negative)
    const double V_L_pp  = V_L_on - V_L_off;         // = Vdc/2
    const double Tsw     = 1.0 / Fsw;
    const double DeltaI_pp = V_L_on * duty_at_peak * Tsw / L;

    // MKF Vienna.cpp:655-665: three phase-inductor windings, shifted ±120°.
    static const char*  phaseNames[3]   = {"Phase A", "Phase B", "Phase C"};
    static const double phaseOffsets[3] = {0.0, -2.0 * M_PI / 3.0, +2.0 * M_PI / 3.0};

    // Interleaving (MKF Vienna.cpp:420,646,677): split the per-phase current across N_ch parallel channel
    // inductors. Each channel carries 1/N_ch of the phase's DC/envelope current (its peak scales by 1/N_ch);
    // the per-channel switching ripple ΔI_pp = V_L·D·Tsw/L is UNCHANGED (set by the per-channel inductance L,
    // not by the current the channel carries), so it is not divided down.
    const double I_pk_ch = I_pk / static_cast<double>(numberOfChannels);
    const double Vhalf   = Vdc / 2.0;

    // MKF's 6-sector DPWM operating points (Vienna.cpp:488-522,733): the line angles are the sector centres,
    // 30° + k·60° (k = 0..5). At each we take a switching-period snapshot evaluated at that angle's phase
    // voltage/current — capturing the inductor stress across the whole line cycle, not just at its peak.
    static const int kViennaSectorCount = 6;

    MAS::OperatingPoint operatingPoint;
    for (int ph = 0; ph < 3; ++ph) {
        for (int ch = 0; ch < numberOfChannels; ++ch) {
            std::string baseName = phaseNames[ph];
            if (numberOfChannels > 1) baseName += " ch" + std::to_string(ch);

            MAS::Waveform currentWaveform, voltageWaveform;
            double opFreq = Fsw;

            if (fullLineCycle) {
                // MKF Vienna.cpp:677-685: full line-cycle envelope, complete_excitation at the LINE frequency.
                currentWaveform = vienna_build_line_cycle_waveform(
                    ViennaLineCycleKind::CURRENT, I_pk_ch, V_phase_peak, Vdc, L, Fsw, lineFrequency, phaseOffsets[ph]);
                voltageWaveform = vienna_build_line_cycle_waveform(
                    ViennaLineCycleKind::VOLTAGE, I_pk_ch, V_phase_peak, Vdc, L, Fsw, lineFrequency, phaseOffsets[ph]);
                opFreq = lineFrequency;
            } else {
                // MKF Vienna.cpp:687-703: peak-of-line switching-period snapshot. RECTANGULAR voltage with
                // offset=0 yields V_on = pp·(1−D) = V_phase_peak and V_off = −pp·D = V_phase_peak − Vdc/2
                // (volt-second balanced, zero mean) — see the MKF comment at Vienna.cpp:690.
                currentWaveform = WP::create_waveform(Lbl::TRIANGULAR, DeltaI_pp, Fsw, duty_at_peak, I_pk_ch, 0);
                voltageWaveform = WP::create_waveform(Lbl::RECTANGULAR, V_L_pp, Fsw, duty_at_peak, 0.0, 0);
            }

            operatingPoint.get_mutable_excitations_per_winding().push_back(
                WP::complete_excitation(currentWaveform, voltageWaveform, opFreq, baseName));

            // MKF Vienna.cpp:488-522: the additional DPWM sector operating points. Only for the peak-of-line
            // family (fullLineCycle=false); each sector is its own switching-period snapshot at a sector-centre
            // line angle, with the duty/ripple/current re-evaluated from that angle's phase voltage.
            if (!fullLineCycle && peakOfLinePlusSectors) {
                for (int s = 0; s < kViennaSectorCount; ++s) {
                    const double lineAngle  = (M_PI / 6.0) + s * (M_PI / 3.0);   // 30° + s·60°
                    const double localAngle = lineAngle + phaseOffsets[ph];
                    const double Vphase_s   = V_phase_peak * std::sin(localAngle);
                    const double Iavg_s     = I_pk_ch * std::sin(localAngle);

                    double dutyAbs = 1.0 - std::abs(Vphase_s) / Vhalf;   // per-angle boost duty
                    if (dutyAbs < 0.0) dutyAbs = 0.0;
                    if (dutyAbs > 1.0) dutyAbs = 1.0;
                    const double dIpp_s = std::abs(Vphase_s) * dutyAbs * (1.0 / Fsw) / L;

                    MAS::Waveform curS = WP::create_waveform(Lbl::TRIANGULAR, dIpp_s, Fsw, dutyAbs, Iavg_s, 0);
                    MAS::Waveform volS = WP::create_waveform(Lbl::RECTANGULAR, V_L_pp, Fsw, dutyAbs, 0.0, 0);
                    operatingPoint.get_mutable_excitations_per_winding().push_back(
                        WP::complete_excitation(curS, volS, Fsw, baseName + " sector " + std::to_string(s)));
                }
            }
        }
    }

    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 7: single-phase AC-input PFC — boost front end.
// Ported from MKF converter_models/PowerFactorCorrection.cpp:425
// (PowerFactorCorrection::process_operating_points), the BOOST topologyVariant branch
// (bipolar=false, buckBoostClass=false). The boost-path helpers it delegates to are inlined:
//   per_phase_power (:238)          → outputPower (single phase, no interleaving split)
//   calculate_duty_cycle (:218-229) → boost ratio D = 1 − Vin/(Vout+Vd)
//   effective_diode_voltage_drop    → diodeVoltageDrop param
// See ConverterAnalytical.hpp for the full omitted/throws contract.
// ─────────────────────────────────────────────────────────────────────────────
MAS::OperatingPoint analytical_pfc(double inputVoltageRms,
                                   double outputVoltage,
                                   double outputPower,
                                   double lineFrequency,
                                   double switchingFrequency,
                                   double boostInductance,
                                   double efficiency,
                                   double diodeVoltageDrop,
                                   int numberOfPeriods,
                                   bool bipolar) {
    using Lbl = MAS::WaveformLabel;

    // Guards — mirror MKF's own (PowerFactorCorrection::run_checks :175-216,
    // calculate_inductance_ccm :263); no fabricated defaults.
    if (inputVoltageRms <= 0)
        throw std::invalid_argument("analytical_pfc: inputVoltageRms must be > 0");
    if (outputVoltage <= 0)
        throw std::invalid_argument("analytical_pfc: outputVoltage must be > 0");
    if (outputPower <= 0)
        throw std::invalid_argument("analytical_pfc: outputPower must be > 0");
    if (lineFrequency <= 0)
        throw std::invalid_argument("analytical_pfc: lineFrequency must be > 0");
    if (switchingFrequency <= 0)
        throw std::invalid_argument("analytical_pfc: switchingFrequency must be > 0");
    if (boostInductance <= 0)
        throw std::invalid_argument("analytical_pfc: boostInductance must be > 0");
    if (efficiency <= 0 || efficiency > 1.0)   // MKF run_checks :207
        throw std::invalid_argument("analytical_pfc: efficiency must be in (0, 1]");
    // Boost can only step UP: Vout must exceed the peak line voltage (MKF run_checks :194-204).
    const double vinPeakForCheck = inputVoltageRms * std::sqrt(2.0);
    if (outputVoltage + diodeVoltageDrop <= vinPeakForCheck)
        throw std::invalid_argument(
            "analytical_pfc: outputVoltage must exceed peak input voltage sqrt(2)*inputVoltageRms "
            "(a boost PFC can only step up)");

    // MKF :454-468 — worst-case at the (single) supplied RMS line voltage.
    const double vinRmsMin  = inputVoltageRms;
    const double vinPeakMin = vinRmsMin * std::sqrt(2.0);
    const double L          = boostInductance;
    const double pinAvg     = outputPower / efficiency;
    const double iinRmsAvg  = pinAvg / vinRmsMin;
    const double iLinePeak  = iinRmsAvg * std::sqrt(2.0);

    // MKF :500-513 — time grid: Tsw/4 step over `numberOfPeriods` mains periods.
    const double mainsPeriod     = 1.0 / lineFrequency;
    const double switchingPeriod = 1.0 / switchingFrequency;
    const int    actualPeriods   = (numberOfPeriods > 0) ? numberOfPeriods : 2;
    const double totalTime       = mainsPeriod * actualPeriods;
    const double timeStep        = switchingPeriod / 4.0;
    const size_t numPoints       = static_cast<size_t>(totalTime / timeStep) + 1;

    std::vector<double> currentData, voltageData, timeData;
    currentData.reserve(numPoints);
    voltageData.reserve(numPoints);
    timeData.reserve(numPoints);

    // MKF :515-586 — the boost branch (bipolar=false, buckBoostClass=false).
    for (size_t i = 0; i < numPoints; ++i) {
        const double t = i * timeStep;
        timeData.push_back(t);

        const double theta = 2.0 * M_PI * t / mainsPeriod;
        // Bridged boost → rectified |sin| (unipolar inductor current/voltage).  MKF :523-526
        // Bridgeless TOTEM-POLE (bipolar=true) → the inductor sits on the AC line with no rectifier, so it
        // sees a TRUE bipolar sine: signed current envelope and signed off-time bus polarity.  MKF :393-432
        const double vinShape   = bipolar ? std::sin(theta) : std::abs(std::sin(theta));
        double vinInst          = vinPeakMin * vinShape;
        double vinAbsInst       = vinPeakMin * std::abs(vinShape);
        // Floor |Vin| near the line zero-crossing so the boost duty stays bounded.  MKF :399-404
        if (bipolar && vinAbsInst < vinPeakMin * 0.05) {
            vinAbsInst = vinPeakMin * 0.05;
            vinInst    = std::copysign(vinAbsInst, vinShape);
        }

        // Boost duty D = 1 − |Vin|/(Vout+Vd), clipped to the physical [0, 1].  MKF :543-547
        double D = 1.0 - vinAbsInst / (outputVoltage + diodeVoltageDrop);
        if (D < 0.0) D = 0.0;
        if (D > 1.0) D = 1.0;

        const double iAvgInst = iLinePeak * vinShape;                       // signed for totem-pole  MKF :549
        const double deltaI   = vinAbsInst * D / (L * switchingFrequency);  // MKF :550

        // Integer switching-cycle phase (4 samples/period since timeStep = Tsw/4).  MKF :557-559
        constexpr size_t samplesPerSwCycle = 4;
        const double switchPhase = static_cast<double>(i % samplesPerSwCycle)
                                   / static_cast<double>(samplesPerSwCycle);

        double ripple;   // MKF :561-566
        if (switchPhase < D) {
            ripple = -deltaI / 2 + deltaI * (switchPhase / D);
        } else {
            ripple = deltaI / 2 - deltaI * ((switchPhase - D) / (1 - D));
        }
        currentData.push_back(iAvgInst + ripple);                           // MKF :568

        if (switchPhase < D) {
            voltageData.push_back(vinInst);                                 // ON: L sees +Vin.  MKF :570-572
        } else {
            // Boost-family OFF-time: inductor sees Vin − Vout − Vd. For totem-pole the bus polarity the
            // inductor sees is also signed with the half-cycle (voutSigned).  MKF :579-585.
            const double voutSigned = bipolar ? std::copysign(outputVoltage, vinShape) : outputVoltage;
            voltageData.push_back(vinInst - voutSigned - diodeVoltageDrop);
        }
    }

    // MKF :588-624 — one CUSTOM current + voltage waveform (the single boost-inductor winding),
    // completed at the LINE frequency. WP::complete_excitation supplies the DSP MKF runs inline
    // (calculate_sampled_waveform / _harmonics_data / _processed_data).
    MAS::Waveform currentWaveform;
    currentWaveform.set_ancillary_label(Lbl::CUSTOM);
    currentWaveform.set_data(currentData);
    currentWaveform.set_time(timeData);

    MAS::Waveform voltageWaveform;
    voltageWaveform.set_ancillary_label(Lbl::CUSTOM);
    voltageWaveform.set_data(voltageData);
    voltageWaveform.set_time(timeData);

    MAS::OperatingPoint operatingPoint;
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(currentWaveform, voltageWaveform, lineFrequency, "Boost inductor"));
    return operatingPoint;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 8: magnetic-COMPONENT operating-point models (CT / DMC / CMC). Each ports
// one MKF converter_models process_operating_points faithfully; the component's
// per-winding excitation is computed from explicit electrical params (no Magnetic /
// core geometry), and all DSP comes from WaveformProcessor.
// ─────────────────────────────────────────────────────────────────────────────

// Element-wise data ops used by the CT reflection chain (MKF Inputs::multiply_waveform /
// sum_waveform, Inputs.cpp:106/114 — trivial scalar transforms of the data vector, not DSP; the
// time grid is preserved so the result is still samplable).
static MAS::Waveform ca_scale_waveform(MAS::Waveform waveform, double scalar) {
    for (auto& datum : waveform.get_mutable_data()) datum *= scalar;
    return waveform;
}
static MAS::Waveform ca_offset_waveform(MAS::Waveform waveform, double scalar) {
    for (auto& datum : waveform.get_mutable_data()) datum += scalar;
    return waveform;
}

// Ported from MKF converter_models/CurrentTransformer.cpp:42 (process_operating_points(turnsRatio,
// secondaryDcResistance)).
MAS::OperatingPoint analytical_current_transformer(MAS::WaveformLabel primaryCurrentWaveformLabel,
                                                   double maximumPrimaryCurrentPeak,
                                                   double frequency,
                                                   double turnsRatio,
                                                   double burdenResistor,
                                                   double secondaryDcResistance,
                                                   double dutyCycle,
                                                   double diodeVoltageDrop) {
    using Lbl = MAS::WaveformLabel;

    // Guards: reject non-positive required inputs (no fabricated defaults). complete_excitation itself
    // also rejects frequency <= 0.
    if (maximumPrimaryCurrentPeak <= 0)
        throw std::invalid_argument("analytical_current_transformer: maximumPrimaryCurrentPeak must be > 0");
    if (frequency <= 0)
        throw std::invalid_argument("analytical_current_transformer: frequency must be > 0");
    if (turnsRatio <= 0)
        throw std::invalid_argument("analytical_current_transformer: turnsRatio must be > 0");

    // MKF CurrentTransformer.cpp:45-56 — primary-current peak-to-peak per waveform shape (and the
    // unsupported-label guard MKF throws).
    double peakToPeak;
    switch (primaryCurrentWaveformLabel) {
        case Lbl::SINUSOIDAL:
            peakToPeak = maximumPrimaryCurrentPeak * 2;
            break;
        case Lbl::UNIPOLAR_RECTANGULAR:
        case Lbl::UNIPOLAR_TRIANGULAR:
            peakToPeak = maximumPrimaryCurrentPeak;
            break;
        default:
            throw std::invalid_argument(
                "analytical_current_transformer: only SINUSOIDAL, UNIPOLAR_RECTANGULAR, "
                "UNIPOLAR_TRIANGULAR are allowed for current transformers");
    }

    // MKF :59 — the primary (sensed line) current waveform.
    MAS::Waveform primaryCurrentWaveform =
        WP::create_waveform(primaryCurrentWaveformLabel, peakToPeak, frequency, dutyCycle);

    // MKF :66-68 — secondary current = primary × turnsRatio (Ip·Np = Is·Ns); secondary voltage =
    // Is·(burden + Rsec_dc) + Vdiode. MKF derives the secondary current via reflect_waveform (2-arg,
    // Inputs.cpp:1222) which likewise multiplies the data by the ratio.
    // NOTE: the burden AND the secondary DC resistance are both OHMS in series with Is, so both scale the
    // current (i·R); only the rectifier drop is a true DC volt offset. The ported form added Rsec_dc as a
    // volt offset (ohms + volts), which inflated the DC average by i·0 → Rsec_dc regardless of current.
    MAS::Waveform secondaryCurrentWaveform = ca_scale_waveform(primaryCurrentWaveform, turnsRatio);
    MAS::Waveform secondaryVoltageWaveform =
        ca_offset_waveform(ca_scale_waveform(secondaryCurrentWaveform, burdenResistor + secondaryDcResistance),
                           diodeVoltageDrop);

    // MKF :73 — the primary winding voltage is the secondary voltage reflected back: V_pri = V_sec × turnsRatio.
    MAS::Waveform primaryVoltageWaveform = ca_scale_waveform(secondaryVoltageWaveform, turnsRatio);

    // MKF :79-90 — two winding excitations (Primary + Secondary). complete_excitation runs the same DSP
    // (sampled waveform + harmonics + processed) MKF applies to each SignalDescriptor inline (:60-78).
    MAS::OperatingPoint operatingPoint;
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(primaryCurrentWaveform, primaryVoltageWaveform, frequency, "Primary"));
    operatingPoint.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(secondaryCurrentWaveform, secondaryVoltageWaveform, frequency, "Secondary"));
    return operatingPoint;
}

// Winding/phase count for a DMC configuration. MKF DifferentialModeChoke.h:96 (get_number_of_windings).
static int dmc_number_of_windings(DmcConfiguration configuration) {
    switch (configuration) {
        case DmcConfiguration::SINGLE_PHASE:             return 1;
        case DmcConfiguration::SINGLE_PHASE_BALANCED:    return 2;
        case DmcConfiguration::THREE_PHASE:              return 3;
        case DmcConfiguration::THREE_PHASE_WITH_NEUTRAL: return 4;
    }
    return 1;
}

// Ported from MKF converter_models/DifferentialModeChoke.cpp:145 (process_operating_points) +
// resolve_peak_current (:128).
MAS::OperatingPoint analytical_differential_mode_choke(double operatingCurrent,
                                                       double inputVoltage,
                                                       double lineFrequency,
                                                       double switchingFrequency,
                                                       DmcConfiguration configuration,
                                                       double peakCurrent,
                                                       double ambientTemperature) {
    using Lbl = MAS::WaveformLabel;

    // MKF :150-151 — operating (loss) frequency is the line frequency; ripple is at the switching frequency
    // (MKF require_input throws if it is missing/non-positive).
    if (switchingFrequency <= 0)
        throw std::invalid_argument("analytical_differential_mode_choke: switchingFrequency must be > 0");
    if (lineFrequency <= 0)
        throw std::invalid_argument("analytical_differential_mode_choke: lineFrequency must be > 0");
    const double operatingFrequency = lineFrequency;
    const double rippleFrequency    = switchingFrequency;

    // MKF resolve_peak_current(0.20) (:128-143): an explicit peak wins; otherwise derive it from a positive
    // operating current with a 20% ripple assumption; otherwise there is genuinely no current info → throw.
    constexpr double kOperatingRippleFraction = 0.20;
    double resolvedPeakCurrent;
    if (std::isfinite(peakCurrent)) {
        resolvedPeakCurrent = peakCurrent;
    } else {
        if (!(operatingCurrent > 0))
            throw std::invalid_argument(
                "analytical_differential_mode_choke: neither peakCurrent nor a positive operatingCurrent "
                "was provided — cannot size the choke");
        resolvedPeakCurrent = operatingCurrent * (1.0 + kOperatingRippleFraction);
    }

    // MKF :162-165 — ripple current = peak − operating (fallback 20% if the difference is negative).
    double currentRipple = resolvedPeakCurrent - operatingCurrent;
    if (currentRipple < 0)
        currentRipple = operatingCurrent * 0.2;

    const int numWindings = dmc_number_of_windings(configuration);
    const double operatingVoltage = inputVoltage;

    // MKF :185-194 — per-winding phase angles (0 / ±120° for 3-phase; the neutral shares 0°).
    std::vector<double> phaseAngles;
    if (configuration == DmcConfiguration::SINGLE_PHASE) {
        phaseAngles = {0.0};
    } else if (configuration == DmcConfiguration::SINGLE_PHASE_BALANCED) {
        phaseAngles = {0.0, 0.0};
    } else if (configuration == DmcConfiguration::THREE_PHASE) {
        phaseAngles = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0};
    } else {  // THREE_PHASE_WITH_NEUTRAL
        phaseAngles = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0, 0.0};
    }

    // Keep the RAW per-winding current waveforms so the magnetizing current (Σ I_k) is summed on the same
    // 10000-point line-period grid MKF uses (complete_excitation stores the resampled waveform, not the raw).
    std::vector<MAS::Waveform> rawCurrentWaveforms;
    std::vector<MAS::OperatingPointExcitation> excitations;

    for (int windingIdx = 0; windingIdx < numWindings; windingIdx++) {
        const double phaseAngle = phaseAngles[windingIdx];
        const bool isNeutral =
            (configuration == DmcConfiguration::THREE_PHASE_WITH_NEUTRAL && windingIdx == 3);

        // MKF :207-231 — line-frequency sinusoid of amplitude √2·operatingCurrent (RMS→peak) + a triangular
        // switching-frequency ripple of amplitude currentRipple, over one line period (10000 points). The
        // neutral winding carries 10% of the phase amplitude.
        double currentAmplitude = operatingCurrent * std::sqrt(2.0);
        if (isNeutral) currentAmplitude *= 0.1;

        const int numPoints = 10000;
        const double period = 1.0 / operatingFrequency;
        std::vector<double> timeData(numPoints), currentData(numPoints);
        for (int i = 0; i < numPoints; i++) {
            const double t = i * period / numPoints;
            timeData[i] = t;
            currentData[i] = currentAmplitude * std::sin(2.0 * M_PI * operatingFrequency * t + phaseAngle);
            const double ripplePhase = std::fmod(t * rippleFrequency, 1.0);
            const double ripple = (ripplePhase < 0.5) ? (4.0 * ripplePhase - 1.0) : (3.0 - 4.0 * ripplePhase);
            currentData[i] += currentRipple * ripple;
        }

        MAS::Waveform currentWaveform;
        currentWaveform.set_ancillary_label(Lbl::CUSTOM);
        currentWaveform.set_data(currentData);
        currentWaveform.set_time(timeData);
        rawCurrentWaveforms.push_back(currentWaveform);

        // MKF :248-258 — small line-frequency voltage across the inductor (~5% of input; neutral 10% of that).
        double voltageAmplitude = operatingVoltage * 0.05;
        if (isNeutral) voltageAmplitude *= 0.1;
        MAS::Waveform voltageWaveform =
            WP::create_waveform(Lbl::SINUSOIDAL, voltageAmplitude, operatingFrequency, 0.5);

        // MKF :237-274 builds the current + voltage SignalDescriptors (processed + sampled + harmonics) then
        // the excitation; complete_excitation runs exactly that DSP at the line frequency.
        excitations.push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, operatingFrequency,
                                    "Winding " + std::to_string(windingIdx + 1)));
    }

    MAS::OperatingPoint operatingPoint;
    operatingPoint.set_excitations_per_winding(excitations);
    operatingPoint.get_mutable_conditions().set_ambient_temperature(ambientTemperature);

    // MKF :281-315 — magnetizing current = point-by-point sum of all winding currents (in a DMC every winding
    // drives the flux the same way, so MMF ∝ Σ I_k). Set on the first winding's excitation.
    {
        std::vector<double> sumData = rawCurrentWaveforms[0].get_data();
        const auto timeData = rawCurrentWaveforms[0].get_time().value();
        for (size_t w = 1; w < rawCurrentWaveforms.size(); ++w) {
            const auto& wData = rawCurrentWaveforms[w].get_data();
            for (size_t j = 0; j < sumData.size() && j < wData.size(); ++j)
                sumData[j] += wData[j];
        }

        MAS::Waveform magnetizingWaveform;
        magnetizingWaveform.set_ancillary_label(Lbl::CUSTOM);
        magnetizingWaveform.set_data(sumData);
        magnetizingWaveform.set_time(timeData);

        MAS::SignalDescriptor magnetizingCurrent;
        auto sampledWaveform = WP::calculate_sampled_waveform(magnetizingWaveform, operatingFrequency);
        magnetizingCurrent.set_waveform(sampledWaveform);
        magnetizingCurrent.set_harmonics(WP::calculate_harmonics_data(sampledWaveform, operatingFrequency));
        magnetizingCurrent.set_processed(WP::calculate_processed_data(magnetizingCurrent, sampledWaveform, true));

        operatingPoint.get_mutable_excitations_per_winding()[0].set_magnetizing_current(magnetizingCurrent);
    }

    return operatingPoint;
}

// CMC winding names by count. Ported from MKF CommonModeChoke::windingNames (CommonModeChoke.cpp:35).
static std::vector<std::string> cmc_winding_names(int numWindings) {
    switch (numWindings) {
        case 2:  return {"Line", "Neutral"};
        case 3:  return {"Phase A", "Phase B", "Phase C"};
        case 4:  return {"Phase A", "Phase B", "Phase C", "Neutral"};
        default: {
            std::vector<std::string> names;
            for (int i = 0; i < numWindings; ++i)
                names.push_back("Winding " + std::to_string(i + 1));
            return names;
        }
    }
}

// MKF CommonModeChoke.cpp:89-93 — V_mains → CM excitation scaling, calibrated so 230 V is a no-op
// (dV/dt ∝ V_bus ≈ √2·V_mains → I_cm scales linearly with the mains voltage; vanishes at 0 V).
static constexpr double CMC_VREF_VMAINS = 230.0;
static double cmc_excitation_scaling(double operatingVoltage) {
    if (operatingVoltage <= 0.0) return 0.0;
    return operatingVoltage / CMC_VREF_VMAINS;
}

// Ported from MKF converter_models/CommonModeChoke.cpp:327 (the scalar-arg
// process_operating_points(turnsRatios, magnetizingInductance)); the all-1:1 turnsRatios arg shapes no
// excitation and is dropped.
MAS::OperatingPoint analytical_common_mode_choke(double magnetizingInductance,
                                                 double operatingCurrent,
                                                 double operatingVoltage,
                                                 double excitationFrequency,
                                                 int numberOfWindings,
                                                 double parasiticCapacitancePf,
                                                 double dvdtVPerNs,
                                                 double ambientTemperature) {
    using Lbl = MAS::WaveformLabel;

    // Guards mirror MKF run_checks (CommonModeChoke.cpp:228-262): numberOfWindings 2-4, positive operating
    // current, positive excitation (line/dominant) frequency; plus the magnetizing inductance MKF derives
    // from the Magnetic and the operating voltage the CM scaling needs — all required, no fabricated defaults.
    if (numberOfWindings < 2 || numberOfWindings > 4)
        throw std::invalid_argument("analytical_common_mode_choke: numberOfWindings must be 2, 3, or 4");
    if (magnetizingInductance <= 0)
        throw std::invalid_argument("analytical_common_mode_choke: magnetizingInductance must be > 0");
    if (operatingCurrent <= 0)
        throw std::invalid_argument("analytical_common_mode_choke: operatingCurrent must be > 0");
    if (operatingVoltage <= 0)
        throw std::invalid_argument("analytical_common_mode_choke: operatingVoltage must be > 0");
    if (excitationFrequency <= 0)
        throw std::invalid_argument("analytical_common_mode_choke: excitationFrequency must be > 0");

    const double excFreq = excitationFrequency;

    // CM current amplitude: I_cm = C·dV/dt when both are supplied, else a representative
    // fallback, then scaled by the mains voltage (see cmc_excitation_scaling).
    //
    // The fallback is the *residual* CM current flowing through the choke in normal
    // operation — what the core actually sees after the input Y-caps shunt most of the
    // switch-node injection — NOT the raw C·dV/dt source current. A raw-injection value
    // (~100 mA ≈ 20 pF × 5 V/ns) through a high-permeability nanocrystalline CM core
    // (µ_r ~1e5) drives B far past saturation (e.g. B_peak ≈ 2.7 T on a small WE
    // nanocrystalline choke, B_sat ≈ 1.2 T), i.e. false saturation. 10 mA is a moderate
    // post-Y-cap residual that keeps a typical mains CMC in its linear region; callers
    // who want the raw-injection stress case supply parasiticCapacitancePf + dvdtVPerNs.
    double iCmPeak;
    if (parasiticCapacitancePf > 0.0 && dvdtVPerNs > 0.0)
        iCmPeak = parasiticCapacitancePf * dvdtVPerNs * 1e-3;
    else
        iCmPeak = 0.01;
    iCmPeak *= cmc_excitation_scaling(operatingVoltage);

    // MKF :359-361 — CM voltage across the CM inductance: V = L·ω·I_cm_peak.
    const double omega   = 2.0 * M_PI * excFreq;
    const double vCmPeak = magnetizingInductance * omega * iCmPeak;

    const auto names = cmc_winding_names(numberOfWindings);

    MAS::OperatingPoint operatingPoint;
    for (int w = 0; w < numberOfWindings; ++w) {
        // MKF :378-406 — every winding gets the SAME CM ripple current (peak-to-peak 2·I_cm) riding on the
        // line-current DC bias, and a CM voltage leading the current by 90° (ideal inductor V = L·dI/dt).
        MAS::Waveform currentWaveform = WP::create_waveform(
            Lbl::SINUSOIDAL, iCmPeak * 2.0, excFreq, 0.5, operatingCurrent, 0, 0, 0);
        MAS::Waveform voltageWaveform = WP::create_waveform(
            Lbl::SINUSOIDAL, vCmPeak * 2.0, excFreq, 0.5, 0.0, 0, 0, M_PI / 2.0);

        operatingPoint.get_mutable_excitations_per_winding().push_back(
            WP::complete_excitation(currentWaveform, voltageWaveform, excFreq, names[w]));
    }

    // MKF :414-418 — conditions + name.
    operatingPoint.get_mutable_conditions().set_ambient_temperature(ambientTemperature);
    operatingPoint.set_name("Nominal");
    return operatingPoint;
}

} // namespace analytical
} // namespace Kirchhoff
