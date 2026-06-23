#pragma once

// Kirchhoff::Pfc — single-phase boost Power-Factor-Correction front end. The FIRST AC-input topology:
// a floating mains source feeds a diode bridge whose DC return is ground, then a boost stage pumps the
// rectified |Vac| up to a regulated high-voltage DC bus.
//
// Control: fixed-frequency, fixed-duty, DISCONTINUOUS conduction mode (DCM). A DCM boost has INHERENT
// power-factor correction — each switching cycle the peak inductor current is Vac(t)·D·Tsw/L, so the
// cycle-averaged input current is proportional to the instantaneous line voltage, with NO current
// controller. (CCM PFC needs an average-current controller; that would be a CTAS controller stage on
// top, the natural next step — see [[control-in-cias]].)
//
// There is no MKF reference for PFC, so this is validated standalone (regulated DC bus + power factor),
// not by the MKF-equivalence suite.

#include <nlohmann/json.hpp>
#include "Fidelity.hpp"

namespace Kirchhoff {

struct PfcDesign {
    double inputVoltageRms;     // mains RMS line voltage
    double lineFrequency;       // mains frequency (Hz)
    double outputVoltage;       // boosted DC bus (must exceed the line peak)
    double outputPower;
    double switchingFrequency;
    double efficiency;
    double switchDuty;          // fixed boost duty (chosen for DCM across the line cycle)
    double boostInductance;     // L, sized for DCM at the target power
    double outputCapacitance;   // bus cap (smooths the 2·line-frequency ripple)
    double loadResistance;
};

/** Design a single-phase DCM boost PFC. */
PfcDesign design_pfc(const nlohmann::json& tasInputs);
/** Assemble a PFC design into a single-stage TAS document (bridge + boost, AC input). */
nlohmann::json build_pfc_tas(const PfcDesign& d);

} // namespace Kirchhoff
