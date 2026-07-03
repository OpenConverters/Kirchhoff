#pragma once

// Kirchhoff::Pfc — single-phase boost Power-Factor-Correction front end. The first AC-input topology:
// a floating mains source feeds a diode bridge whose DC return is ground, then a boost stage pumps the
// rectified |Vac| up to a high-voltage DC bus.
//
// Control: CLOSED-LOOP, current-mode (the proper PFC). A current-sense resistor in the bridge return
// measures the input current; a reference proportional to the rectified line voltage (a divider off the
// rectified bus) sets the target; a HYSTERETIC comparator gates the boost switch so the inductor current
// tracks that reference. Because the reference ∝ |Vac|, the input current follows the line voltage →
// near-unity power factor (the boost emulates a resistor). This is expressed entirely in CIAS (the
// comparator is an AAS analog block, the sense/reference are R's), so it stays a swappable control stage
// and portable to any simulator — see [[control-in-cias]].
//
// The reference gain is matched to the design load so the bus settles at its target (a fixed-gain,
// power-balanced regulation). Adding an outer voltage loop (an analog multiplier scaling the reference
// by a bus-voltage error integrator) for ACTIVE regulation against load changes is the next refinement.
//
// There is no MKF reference for PFC, so it is validated standalone (near-unity power factor + boosted
// DC bus), not by the MKF-equivalence suite.

#include <nlohmann/json.hpp>
#include <string>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct PfcDesign {
    double inputVoltageRms;     // mains RMS line voltage
    double lineFrequency;       // mains frequency (Hz)
    double outputVoltage;       // boosted DC bus
    double outputPower;
    double switchingFrequency;  // TARGET average switching frequency (sizes L for the hysteretic band)
    double efficiency;
    double boostInductance;     // L (CCM, current tracks the reference)
    double outputCapacitance;   // bus cap (smooths the 2·line-frequency ripple)
    double loadResistance;
    nlohmann::json config;
    double senseResistance;     // Rsense in series with L (inductor-current sense)
    double referenceGain;       // kref: nominal current-loop gain; emulates R = Rsense/kref
    double currentHysteresis;   // comparator hysteresis on the (i_ref − i_sense) signal [V]
    double proportionalGain;    // outer voltage-loop PROPORTIONAL gain (kp), derived from the plant
    double integralGain;        // outer voltage-loop INTEGRAL gain (ki = kp·ωp), derived from the plant
    double outputDividerGain;   // kv: V(voutScaled) = kv·V(vout) (output-voltage sense)
    // ── Conduction mode + topology variant (ABT #92) ────────────────────────────────────────────────
    std::string mode;           // "ccm" | "dcm" | "crm" | "transition" — drives boostInductance sizing
    std::string topologyVariant;// "boost" | "totemPole" | "interleaved" (SEPIC/Ćuk not yet supported)
    int numberOfPhases;         // interleaved: number of phase-shifted boost legs (2 or 3); else 1
    bool bipolar;               // totem-pole: bridgeless, the inductor sees a TRUE bipolar sine (no bridge)
};

/** Design a single-phase current-mode (hysteretic) boost PFC. */
PfcDesign design_pfc(const nlohmann::json& tasInputs);
/** Assemble a PFC design into a TWO-stage TAS document: a power stage + a swappable current controller. */
nlohmann::json build_pfc_tas(const PfcDesign& d);

} // namespace Kirchhoff
