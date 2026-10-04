#pragma once

// Kirchhoff::Fcml — flying-capacitor multilevel (FCML) buck and boost (ABT #1439), selected on the ordinary
// buck / boost by config.levels = N (2 = the plain two-level converter, unchanged; 3..6 multilevel).
//
// N−1 complementary switch pairs stack between the top rail and ground with the switch node in the middle; N−2
// flying capacitors, Cf_k across the k-th inner pair of nodes, hold k·Vtop/(N−1). Phase-shifted PWM (pair k at
// (k−1)·360°/(N−1), all at duty D) makes the switch node step between adjacent levels Vtop/(N−1) apart at
// (N−1)·fs: every switch blocks Vtop/(N−1) and the inductor sees a 1/(N−1) voltage step at (N−1) times the
// frequency — the inductance shrinks by (N−1)². The flying capacitors balance naturally under phase-shifted
// PWM with equal duties when the inductor faces a resistive load (buck); a boost's inductor faces the source, so its
// imbalance mode is undamped and in hardware a controller holds the flying capacitors — its deck starts them on the
// periodic orbit, with the duty trims a balancing controller would settle on (fcml_balanced_orbit, ABT #1681). Buck: Vtop = Vin, the pair's high switch is the main switch. Boost: Vtop = Vout, the low
// switch is.

#include <nlohmann/json.hpp>
#include <vector>
#include "ConverterAnalytical.hpp"

namespace Kirchhoff {

// config.levels as a validated whole number in [2, 6]; 2 when absent.
int fcml_levels(const nlohmann::json& config, const char* who);

namespace analytical {
// The inductor of an N-level FCML in CCM, lossless, over one switching period: D from volt-second balance
// (buck Vo/Vin, boost 1 − Vin/Vo), the switch node from the count of pairs whose high switch conducts, the
// current piecewise linear about its mean (buck Io, boost Io/(1 − D)). Emits "Primary". Throws when the inductor
// runs discontinuous (DCM is not modelled) or the conversion is impossible (buck Vo ≥ Vin, boost Vo ≤ Vin).
MAS::OperatingPoint analytical_fcml(bool boost, int levels, double inputVoltage, double outputVoltage,
                                    double outputCurrent, double switchingFrequency, double inductance);

// Volt-seconds of one inductor ripple: ΔIL·L (independent of L and the load), at one input voltage.
double fcml_ripple_volt_seconds(bool boost, int levels, double inputVoltage, double outputVoltage,
                                double switchingFrequency);
// The largest charge a flying capacitor moves in one period per ampere of inductor current, at one input voltage.
double fcml_flying_charge_per_amp(bool boost, int levels, double inputVoltage, double outputVoltage,
                                  double switchingFrequency);

// The balanced periodic steady state of the switched cell with FINITE flying capacitors, output capacitor and load
// (R = Vo²/P), lossless switches, at average duty D: per-pair main-switch duty trims (summing to zero, so the
// average duty and Vout are kept) chosen so every flying capacitor's period mean sits at k·Vtop/(N−1), and the
// state at t = 0 on that orbit. Solved by shooting one period (Newton on periodicity + the N−2 mean conditions).
// With equal duties the output's switching ripple drives the flying capacitors off their levels, and a boost has
// nothing to pull them back (its inductor faces the stiff source): the trims are what a balancing controller
// converges to (ABT #1681). Throws when Newton does not converge.
struct FcmlOrbit {
    std::vector<double> dutyTrims;               // per pair k = 1..N−1, added to D
    double inductorCurrent;                      // at t = 0
    std::vector<double> flyingCapacitorVoltages; // k = 1..N−2, at t = 0
    double outputVoltage;                        // at t = 0
};
FcmlOrbit fcml_balanced_orbit(bool boost, int levels, double inputVoltage, double dutyCycle, double loadResistance,
                              double switchingFrequency, double inductance, double flyingCapacitance,
                              double outputCapacitance);
} // namespace analytical

// What the FCML cell builder needs from a buck or boost design.
struct FcmlCell {
    bool boost = false;
    int levels = 3;
    double inputVoltage, inputVoltageMin, inputVoltageMax, outputVoltage, outputPower, switchingFrequency, efficiency;
    double dutyCycle, deadFraction, inductance, flyingCapacitance, outputCapacitance;
    nlohmann::json config;
};

nlohmann::json build_fcml_tas(const FcmlCell& c);

} // namespace Kirchhoff
