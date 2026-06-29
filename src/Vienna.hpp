#pragma once

// Kirchhoff::Vienna — three-phase, three-level Vienna rectifier (a unidirectional 3-phase boost PFC).
// The first THREE-phase AC-input topology. Per phase: a boost inductor to a node X, two rail diodes
// (X -> +bus, −bus -> X) and a BIDIRECTIONAL switch X <-> the split-bus midpoint M. M = the source
// neutral = ground (a 4-wire connection — every node stays referenced, unlike a floating star), so the
// bus is split symmetric ±Vdc/2 about ground. Switch ON clamps X to M and charges the inductor; OFF
// lets the inductor current flow to the rail it is heading for → three voltage levels (+bus, M, −bus).
//
// Control: CLOSED-LOOP, per-phase current shaping (the proper Vienna PFC). Each phase has a current
// reference ∝ its phase voltage; the bidirectional switch is gated so the inductor current tracks it →
// the input currents follow the three phase voltages → near-unity 3-phase power factor. The Vienna
// subtlety is that the gating POLARITY flips with the sign of the phase voltage; this is handled by
// gating on V(phase)·(iref − iL) > 0 (same sign as sign(v)·error). Expressed in CIAS (summer +
// multiplier + comparator analog blocks) as a swappable control stage. Midpoint balancing is left to the
// natural symmetry of the balanced 3-phase load (active balancing is a further refinement).
// See [[kirchhoff-ac-topologies]].

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct ViennaDesign {
    double inputVoltageRms;     // per-phase (line-to-neutral) RMS
    double lineFrequency;
    double outputVoltage;       // full DC bus, +bus to −bus (split ±Vdc/2 about ground)
    double outputPower;
    double switchingFrequency;
    double efficiency;
    double boostInductance;     // per-phase L
    double busCapacitance;      // each half of the split bus
    double loadResistance;
    nlohmann::json config;      // across the full bus
    double senseResistance;     // Rsense in series with each phase inductor
    double referenceGain;       // kref: i_ref voltage = kref·V(phase); emulates R = Rsense/kref
    double currentHysteresis;   // hysteresis on the gating signal V(phase)·(iref − iL·Rsense)
    double balanceGain;         // rail-balancing integral gain (kbal), derived from the balance plant
    double balanceClamp;        // ±limit on the common balancing term (anti-windup rail)
    // ── Outer VOLTAGE loop (DESIGNED PI, mirrors Pfc): drives the dynamic emulated conductance g from the
    // bus error so the bus REGULATES to target against real-part losses (a fixed kref cannot — it sags
    // below the boost-feasible region and collapses to passive rectification). g modulates each phase's
    // current reference iref = g·V(phase)/Rsense; bus low → g up → more current → bus boosts back to target.
    double outputDividerGain;   // kv: V(busScaled) = kv·(busP−busN)
    double proportionalGain;    // kp of the bus-voltage PI
    double integralGain;        // ki of the bus-voltage PI (zero placed at the load pole)
};

/** Design a three-phase Vienna rectifier (open-loop boost; see header on control status). */
ViennaDesign design_vienna(const nlohmann::json& tasInputs);
/** Assemble a Vienna design into a single-stage TAS document (3-phase AC input, split DC bus). */
nlohmann::json build_vienna_tas(const ViennaDesign& d);

} // namespace Kirchhoff
