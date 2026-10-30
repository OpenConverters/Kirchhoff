#pragma once

// "I know the design I want" duty pin (ABT #2134): config.forcedDutyCycle makes the main switch run at exactly
// that duty. The duty is then the COMMAND, not the result: the design no longer derives it from the requested
// output voltage, and the rail the converter delivers into the requested load (R = Vreq²/Preq, held) becomes a
// result — the same contract as config.driveAtSwitchingFrequency for the resonant converters (ABT #1539).
//
// Each topology's analytical solver already derives its switch on-time from the operating point through one
// timing function (ConverterAnalytical: forward_timing / push_pull_timing / flyback_timing /
// isolated_buck_boost_duty_cycle). The pin finds the main rail at which THAT function returns the pinned duty
// (solve_output_voltage_for_duty), so the analytical waveforms, the embedded excitations and the open-loop deck
// all run at the same duty. The other rails follow the shared primary volt-seconds of their topology.
//
// The design keeps sizing the stage from the REQUESTED rails (turns ratios, inductances, capacitors); the
// requested rails are remembered so the worst-corner (Vin_min) ratings still see the regulated converter.

#include "ConverterAnalytical.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
namespace duty_pin {

inline std::string num(double v) {
    std::ostringstream s;
    s << v;
    return s.str();
}

// Remember the rails as requested before the pin replaces them with the delivered ones.
template <class Design>
void remember_requested(Design& d) {
    d.dutyPinned = true;
    d.requestedVoltages.clear();
    d.requestedPowers.clear();
    for (const auto& leg : d.outputs) {
        d.requestedVoltages.push_back(leg.voltage);
        d.requestedPowers.push_back(leg.power);
    }
}

// Replace every rail by the voltage it delivers into its held load resistance; P = V²/R.
template <class Design>
void deliver(Design& d, const std::vector<double>& delivered, double dutyCycle, const std::string& context) {
    for (size_t i = 0; i < d.outputs.size(); ++i) {
        auto& leg = d.outputs[i];
        if (!(delivered[i] > 0.0) || !std::isfinite(delivered[i]))
            throw std::invalid_argument(context + ": at a Duty Cycle of " + num(dutyCycle) + " output " +
                                        std::to_string(i + 1) + " (turns ratio " + num(leg.turnsRatio) +
                                        ") gets no voltage above its rectifier drop. Raise the Duty Cycle.");
        leg.voltage = delivered[i];
        leg.power = delivered[i] * delivered[i] / leg.loadResistance;
    }
    d.outputVoltage = d.outputs.at(0).voltage;
    d.outputPower = d.outputs.at(0).power;
    d.dutyCycle = dutyCycle;
}

// Forward family (single-switch forward, two-switch forward, active-clamp forward). Every secondary sees the
// primary volt-seconds Vin·D: an extra rail delivers Vo_i = D·Vin/n_i − Vd_i; the main rail is the one at which
// forward_timing (CCM, or the output-inductor DCM on-time) returns D. `maximumDutyCycle` is the topology's
// hard limit (core reset), stated in `limitReason`.
template <class Design>
void pin_forward_family(Design& d, double dutyCycle, double rippleRatio, double maximumDutyCycle,
                        const std::string& limitReason, const std::string& context) {
    if (!(dutyCycle <= maximumDutyCycle))
        throw std::invalid_argument(context + ": a Duty Cycle of " + num(dutyCycle) + " is above " +
                                    num(maximumDutyCycle) + ", " + limitReason + ".");
    remember_requested(d);
    const double Vin = d.inputVoltage, fsw = d.switchingFrequency;
    const size_t nOut = d.outputs.size();
    std::vector<double> ratios, delivered(nOut);
    for (const auto& leg : d.outputs) ratios.push_back(leg.turnsRatio);
    for (size_t i = 1; i < nOut; ++i)
        delivered[i] = dutyCycle * Vin / ratios[i] - d.outputs[i].diodeDrop;
    for (size_t i = 1; i < nOut; ++i)
        if (!(delivered[i] > 0.0)) deliver(d, delivered, dutyCycle, context);   // throws, naming the rail
    const auto& main = d.outputs[0];
    auto dutyAt = [&](double vo) {
        std::vector<double> v = delivered, io(nOut);
        v[0] = vo;
        for (size_t i = 0; i < nOut; ++i) io[i] = v[i] / d.outputs[i].loadResistance;
        return analytical::forward_timing(Vin, v, io, ratios, fsw, d.magnetizingInductance, main.outputInductance,
                                          rippleRatio, main.diodeDrop).t1 * fsw;
    };
    delivered[0] = analytical::solve_output_voltage_for_duty(dutyAt, dutyCycle, Vin / ratios[0] - main.diodeDrop,
                                                             context);
    deliver(d, delivered, dutyCycle, context);
}

// Centre-tapped push-pull, per-switch duty D (< 0.5): an extra rail delivers Vo_i = 2·D·Vin/N_i − Vd_i; the main
// rail is the one at which push_pull_timing returns t1 = D·T.
template <class Design>
void pin_push_pull(Design& d, double dutyCycle, double rippleRatio, const std::string& context) {
    if (!(dutyCycle < 0.5))
        throw std::invalid_argument(context + ": a Duty Cycle of " + num(dutyCycle) + " is not possible on a "
                                    "push-pull: each switch conducts for at most half the period, so its duty "
                                    "must stay below 0.5.");
    remember_requested(d);
    const double Vin = d.inputVoltage, fsw = d.switchingFrequency;
    const size_t nOut = d.outputs.size();
    std::vector<double> ratios, delivered(nOut);
    for (const auto& leg : d.outputs) ratios.push_back(leg.turnsRatio);
    for (size_t i = 1; i < nOut; ++i)
        delivered[i] = 2.0 * dutyCycle * Vin / ratios[i] - d.outputs[i].diodeDrop;
    for (size_t i = 1; i < nOut; ++i)
        if (!(delivered[i] > 0.0)) deliver(d, delivered, dutyCycle, context);   // throws, naming the rail
    auto dutyAt = [&](double vo) {
        std::vector<double> v = delivered, io(nOut);
        v[0] = vo;
        for (size_t i = 0; i < nOut; ++i) io[i] = v[i] / d.outputs[i].loadResistance;
        return analytical::push_pull_timing(Vin, v, io, ratios, fsw, d.magnetizingInductance, d.outputInductance,
                                            rippleRatio, d.diodeDrop).t1 * fsw;
    };
    delivered[0] = analytical::solve_output_voltage_for_duty(dutyAt, dutyCycle, Vin / ratios[0] - d.diodeDrop,
                                                             context);
    deliver(d, delivered, dutyCycle, context);
}

// Flyback: every secondary clamps to the shared reflected voltage, Vo_i = n_0·(Vo_0+Vd_0)/n_i − Vd_i; the main
// rail is the one at which flyback_timing (CCM duty, or the DCM energy-balance duty) returns D. The flyback has
// no rail ceiling, so the search bound doubles from the requested rail until the model's duty reaches D.
template <class Design>
void pin_flyback(Design& d, double dutyCycle, const std::string& context) {
    remember_requested(d);
    const double Vin = d.inputVoltage, fsw = d.switchingFrequency;
    const size_t nOut = d.outputs.size();
    std::vector<double> ratios;
    for (const auto& leg : d.outputs) ratios.push_back(leg.turnsRatio);
    const double n0 = ratios[0], Vd0 = d.outputs[0].diodeDrop;
    auto rails = [&](double vo) {
        std::vector<double> v(nOut);
        v[0] = vo;
        for (size_t i = 1; i < nOut; ++i) v[i] = n0 * (vo + Vd0) / ratios[i] - d.outputs[i].diodeDrop;
        return v;
    };
    auto dutyAt = [&](double vo) {
        std::vector<double> v = rails(vo), io(nOut);
        // A rail whose winding voltage stays below its rectifier drop does not conduct: 0 V, 0 A.
        for (size_t i = 0; i < nOut; ++i) {
            v[i] = std::max(v[i], 0.0);
            io[i] = v[i] / d.outputs[i].loadResistance;
        }
        return analytical::flyback_timing(Vin, v, io, ratios, fsw, d.magnetizingInductance, d.diodeDrop,
                                          d.efficiency).dutyCycle;
    };
    double bound = d.requestedVoltages.at(0);
    int doublings = 0;
    while (!(dutyAt(bound) >= dutyCycle)) {
        if (++doublings > 60)
            throw std::invalid_argument(context + ": the flyback model never reaches a Duty Cycle of " +
                                        num(dutyCycle) + " at this input voltage. Lower the Duty Cycle.");
        bound *= 2.0;
    }
    const double vo = analytical::solve_output_voltage_for_duty(dutyAt, dutyCycle, 2.0 * bound, context);
    deliver(d, rails(vo), dutyCycle, context);
}

}  // namespace duty_pin
}  // namespace Kirchhoff
