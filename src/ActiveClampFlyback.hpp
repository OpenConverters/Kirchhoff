#pragma once

// Kirchhoff::ActiveClampFlyback — the active-clamp flyback (ABT #1432).
//
//   Q1 (low side): primary end (sw) -> GND.  Clamp: switch Qc + capacitor Cc across the primary, either
//   high-side (sw -> Qc -> Cc -> Vin) or low-side (sw -> Cc -> Qc -> GND). Both legs carry an explicit
//   body diode (Dq1, Dqc): the deck's switches are ideal, and the clamp's whole action at turn-off runs
//   through Dqc until Qc's channel takes over.
//
// With the complementary drive the leakage inductance Lk is a DESIGNED element, the resonant partner of Cc,
// as Lr is in an LLC: its
// half resonant period with Cc equals the off-time at Vin,min, so the secondary current is a half-sine
// that ends as Q1 turns back on, and the resonance's amplitude is set by a clamp-overshoot budget
// (config.clampOvershoot: the clamp voltage's peak above the reflected voltage, as a fraction of it).
// Those two conditions fix both Lk and Cc. Lk goes out as the transformer's leakageInductance requirement
// (nominal: the magnetic must realise it), so the simulated transformer carries it as coupling.
//
// Two drive modes:
//   complementary    — Qc is on for the whole off-time (dead-time apart). The magnetizing current is
//                      bidirectional and is sized (Lm) to swing negative enough to discharge the switch
//                      node within the dead time: Q1 turns on at zero voltage.
//   nonComplementary — (NC-ACF) the transformer runs discontinuous; Dqc absorbs the leakage energy at
//                      turn-off and Qc is pulsed once, just before Q1 turns on, long enough to drive the
//                      magnetizing current negative enough for ZVS. Lk is a parasitic here (a maximum,
//                      config.leakageRatio of Lp) and Cc keeps the resonance rule, which makes it large.
//
// One output (the ticket's scope). A multi-output spec throws.

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "ConverterAnalytical.hpp"
#include "Flyback.hpp"

namespace Kirchhoff {

namespace analytical {

// The primary cell the steady-state solver integrates. Lm and Lk are the cantilever model of the coupled
// transformer the deck emits (primary inductance Lp = Lm + Lk, coupling k = sqrt(Lm/Lp)); its ideal
// transformer ratio is k·n, not n, and the solver uses exactly that.
struct AcfFlybackCircuit {
    double inputVoltage;          // V
    double outputVoltage;         // V (magnitude)
    double diodeDrop;             // V, output rectifier
    double turnsRatio;            // n = Np/Ns (physical)
    double magnetizingInductance; // Lm (H), cantilever magnetizing branch
    double leakageInductance;     // Lk (H)
    double clampCapacitance;      // Cc (F)
    double switchNodeCapacitance; // Csw (F): Coss + winding capacitance at the switch node; > 0
    double switchingFrequency;    // Hz
    double deadTime;              // s, between the two gates
    bool complementary;           // true: Qc on for the off-time; false: NC-ACF, one pulse before Q1 turns on
    double clampPulse;            // s, NC-ACF pulse width (ignored when complementary)
};

struct AcfFlybackSteadyState {
    double dutyCycle;             // Q1 on-time / period
    double period;
    // One period from Q1's turn-on, uniform grid (period / (N-1)).
    std::vector<double> time;
    std::vector<double> primaryCurrent;     // through Lk (= the primary winding terminal current)
    std::vector<double> primaryVoltage;     // across the primary winding terminals (Lk + Lm)
    std::vector<double> magnetizingCurrent; // Lm branch
    std::vector<double> clampVoltage;       // Vc: the clamp switch node sits at Vin + Vc (high-side clamp)
    std::vector<double> switchNodeVoltage;  // Q1's drain
    std::vector<double> secondaryCurrent;   // rectifier current (>= 0)
    std::vector<double> secondaryVoltage;   // dot convention: +Vin/(k·n) while Q1 conducts
    double outputCurrent;                   // A, average rectifier current
    double primaryCurrentAtClampTurnOff;    // A, the primary current as the clamp gate drops (< 0 discharges the node)
    double switchNodeVoltageAtTurnOn;       // V, Q1's drain as Q1 turns on: 0 means it turned on at zero voltage
    double switchNodeVoltageMidDeadTime;    // V, Q1's drain half-way through the dead time before it turns on (the
                                            //    design's ZVS margin: 0 means the node was down with time to spare)
};

/**
 * @brief Periodic steady state of the active-clamp flyback cell at a given output voltage and current.
 * The duty cycle is an unknown: it is solved together with the initial state so the period closes on itself
 * and the rectifier delivers outputCurrent. Every interval is integrated in closed form (linear ramps, the
 * Lk–Cc / (Lk+Lm)–Cc clamp resonance, or the Lk–Csw / (Lk+Lm)–Csw ring of a floating switch node); diode,
 * body-diode and node-at-rail transitions are located by bisection. Throws if no
 * steady state is found.
 */
AcfFlybackSteadyState solve_active_clamp_flyback(const AcfFlybackCircuit& circuit, double outputCurrent);

/// The steady state as MAS excitations: "Primary" then "Secondary".
MAS::OperatingPoint active_clamp_flyback_operating_point(const AcfFlybackSteadyState& s, double switchingFrequency);

} // namespace analytical

struct ActiveClampFlybackDesign {
    FlybackDesign flyback;        // transformer turns ratio, rectifier, output/input capacitors, spec corners
    std::string mode;             // "complementary" | "nonComplementary"
    std::string clampSide;        // "high" | "low"
    double magnetizingInductance; // Lm (H), cantilever magnetizing branch
    double leakageInductance;     // Lk (H), designed
    double primaryInductance;     // Lp = Lm + Lk (H), the magnetic's magnetizing-inductance requirement
    double clampCapacitance;      // Cc (F)
    double switchNodeCapacitance; // F, lumped Coss + winding capacitance at the switch node (real part)
    double deadTime;              // s
    double clampPulse;            // s (NC-ACF only; 0 when complementary)
    double zvsCurrent;            // A, negative magnetizing current that slews the switch node in the dead time
    double dutyCycle;             // at the nominal operating point (the steady-state solve)
    double clampVoltageMax;       // V, peak Vc over the Vin corners
    double switchVoltageMax;      // V, peak drain-source voltage of Q1 and Qc
};

/**
 * @brief Design an active-clamp flyback.
 * @param tasInputs As for design_flyback, one output. config: mode (complementary | nonComplementary,
 *        default complementary), clampSide (high | low, default high), maxDutyCycle (default 0.5),
 *        clampOvershoot (default 0.15), deadTime (s, default 100 ns), resonantCapacitance (the switch-node
 *        capacitance, as the flyback's QRM knob), leakageRatio (NC-ACF only: Lk's cap as a fraction of Lp,
 *        default 0.02).
 */
ActiveClampFlybackDesign design_active_clamp_flyback(const nlohmann::json& tasInputs);

nlohmann::json build_active_clamp_flyback_tas(const ActiveClampFlybackDesign& d);

} // namespace Kirchhoff
