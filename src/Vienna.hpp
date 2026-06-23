#pragma once

// Kirchhoff::Vienna — three-phase, three-level Vienna rectifier (a unidirectional 3-phase boost PFC).
// The first THREE-phase AC-input topology. Per phase: a boost inductor to a node X, two rail diodes
// (X -> +bus, −bus -> X) and a BIDIRECTIONAL switch X <-> the split-bus midpoint M. M = the source
// neutral = ground (a 4-wire connection — every node stays referenced, unlike a floating star), so the
// bus is split symmetric ±Vdc/2 about ground. Switch ON clamps X to M and charges the inductor; OFF
// lets the inductor current flow to the rail it is heading for → three voltage levels (+bus, M, −bus).
//
// Status: the POWER topology + 3-phase source assemble and simulate (open-loop fixed-duty switching
// boosts the split bus). The full closed-loop Vienna control — polarity-dependent per-phase current
// shaping (the gating polarity flips with the sign of the phase voltage) plus midpoint balancing — is a
// substantial refinement and is NOT yet implemented (it needs a difference/summer block in CIAS on top
// of the multiplier/integrator/comparator already there). See [[kirchhoff-ac-topologies]].

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
    double switchDuty;          // open-loop boost duty (per switch)
    double boostInductance;     // per-phase L
    double busCapacitance;      // each half of the split bus
    double loadResistance;      // across the full bus
};

/** Design a three-phase Vienna rectifier (open-loop boost; see header on control status). */
ViennaDesign design_vienna(const nlohmann::json& tasInputs);
/** Assemble a Vienna design into a single-stage TAS document (3-phase AC input, split DC bus). */
nlohmann::json build_vienna_tas(const ViennaDesign& d);

} // namespace Kirchhoff
