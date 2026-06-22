#pragma once

// Kirchhoff::Pshb — Phase-Shifted Half-Bridge (3-level NPC, isolated, 14th topology). A single
// neutral-point-clamped leg between Vin and ground: four stacked switches S1(vin->nH)/S2(nH->mid)/
// S3(mid->nL)/S4(nL->gnd) with two clamp diodes (mid_cap->nH, nL->mid_cap) and a split-cap midpoint
// (mid_cap = Vin/2). The leg midpoint produces a 3-level voltage (+Vin/2, 0, -Vin/2) phase-shift
// modulated against the split-cap midpoint; a series resonant Lr feeds the transformer primary, and a
// full-bridge secondary rectifier + Lout/Cout form the output. Port of MKF Pshb.
//
// Same family as PSFB but with bus = Vin/2 (so n is ~half) — reuses the PSFB rectifier/output template
// + leakage-aware K + body diodes + snubbers; the new piece is the stacked NPC leg + split caps.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct PshbDesign {
    double inputVoltage, inputVoltageMin, inputVoltageMax;
    double outputVoltage, outputPower, switchingFrequency, efficiency;
    double commandedDuty;          // D_cmd = phi/180
    double phaseDeg;               // leg phase shift (= 180*D_cmd)
    double switchDuty;             // outer-switch on-fraction (~0.5 minus dead time)
    double deadFraction;           // per-leg dead time as a fraction of the period
    double effectiveDuty;          // Deff = D_cmd - duty loss
    double turnsRatio;             // n (full secondary)
    double seriesInductance;       // Lr
    double magnetizingInductance;  // Lm
    double outputInductance;       // Lo
    double splitCapacitance;       // split-cap each (sets mid_cap = Vin/2)
    double loadResistance;
    double outputCapacitance;
};

PshbDesign design_pshb(const nlohmann::json& tasInputs);
nlohmann::json build_pshb_tas(const PshbDesign& d);

} // namespace Kirchhoff
