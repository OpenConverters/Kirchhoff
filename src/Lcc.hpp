#pragma once

// Kirchhoff::Lcc — the LCC (series-parallel) resonant converter (ABT #1438). A half or full bridge drives a
// square ±Vb into a series Lr–Cs, then a parallel capacitor Cp across the transformer, a full-bridge diode
// rectifier and a capacitive output filter. Cp sits on the secondary by default — the high-voltage case, where
// the winding capacitance IS the parallel capacitor — or on the primary as a discrete part
// (config.parallelCapacitorSide). Frequency-controlled, operated above the tank's load-dependent resonance so
// the bridge commutates on the tank current (ZVS).
//
// No MKF model exists, and FHA is poor for a capacitive-filter LCC (Cp charges between rectifier conduction
// intervals), so the design is solved on the exact steady state (solve_lcc) and validated against ngspice.

#include <nlohmann/json.hpp>
#include <array>
#include <vector>
#include "Fidelity.hpp"

namespace Kirchhoff {

namespace analytical {

// The tank referred to the primary. The bridge applies +Vb for half the period and −Vb for the other (50 %,
// commutation inside the dead time ignored); the rectifier clamps the parallel capacitor at ±Vc =
// ±n·(Vo + Vd_path) while it conducts (the output capacitor taken as a stiff voltage).
struct LccCircuit {
    double busVoltage;              // Vb
    double clampVoltage;            // Vc, primary-referred
    double seriesInductance;        // Lr
    double seriesCapacitance;       // Cs
    double parallelCapacitance;     // Cp, primary-referred (a secondary Cp is Cp_sec / n²)
    double magnetizingInductance;   // Lm, across Cp
    double switchingFrequency;
};

// One period of the steady state (half-wave antisymmetric). Currents primary-referred, the rectifier current
// signed (+ while the +Vc clamp conducts).
struct LccSteadyState {
    double period;
    std::vector<double> time, bridgeVoltage, tankCurrent, seriesCapacitorVoltage, parallelCapacitorVoltage,
        magnetizingCurrent, rectifierCurrent;
    double rectifierCurrentMean;    // mean |rectifier current| over the period (primary-referred)
    double tankCurrentAtTurnOff;    // tank current as the +Vb half ends: > 0 is inductive (the node swings
                                    // onto the other rail in the dead time, ZVS)
    std::array<double, 4> initialState;   // (i, vs, vp, im) at t = 0: a warm start for a nearby solve
};

// Throws on non-positive elements, and when the periodic state does not converge. `guess` (a nearby solution's
// initialState) skips the cold settling.
LccSteadyState solve_lcc(const LccCircuit& c, const std::array<double, 4>* guess = nullptr);

// The clamp voltage a resistive load R (at the clamp, primary-referred) settles at: Vc = R·mean|irect|.
// c.clampVoltage is ignored and set in the returned state's circuit; Throws if no Vc in (0, ∞) balances.
struct LccLoadedState { double clampVoltage; LccSteadyState state; };
LccLoadedState solve_lcc_loaded(LccCircuit c, double loadResistance);

} // namespace analytical

struct LccDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, efficiency;
    double switchingFrequency;            // the drive frequency at the operating point (the spec's unless n is pinned)
    double frequencyAtMinimumInput;       // frequency control range across the input range, full load
    double frequencyAtMaximumInput;
    double seriesResonantFrequency;       // 1/(2π√(Lr·Cs))
    double diodeDrop;
    double turnsRatio;                    // n = Np/Ns
    double seriesInductance, seriesCapacitance;
    double parallelCapacitance;           // as placed (secondary: the secondary-side value)
    bool parallelOnSecondary = true;
    double magnetizingInductance;
    bool fullBridge = false;
    double splitCapacitance, balancingResistance;   // half bridge
    double deadTimeFraction;
    double loadResistance, outputCapacitance;
    nlohmann::json config;
};

/**
 * @brief Design an LCC resonant converter (capacitive output filter, full-bridge rectifier).
 *
 * config: parallelCapacitorSide (secondary | primary), bridgeType (halfBridge | fullBridge),
 * qualityFactor Q = √(Lr/Cs)/Rac (1.0), capacitanceRatio Cp/Cs (1.0), resonanceRatio fs/fr (1.6),
 * inductanceRatio Lm/Lr (10), deadTimeFraction (0.02), outputVoltageRipple (0.01).
 * The turns ratio follows from the exact gain at the spec frequency; a pinned turns ratio moves the frequency
 * instead. Throws when Vin_min needs more gain than the tank has above its peak, or a corner loses ZVS.
 */
LccDesign design_lcc(const nlohmann::json& tasInputs);
nlohmann::json build_lcc_tas(const LccDesign& d);

} // namespace Kirchhoff
