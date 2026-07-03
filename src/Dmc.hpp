#pragma once

// Differential-mode choke (DMC) — the requirement-synthesis half, plus the MAS::Inputs assembly over
// analytical_differential_mode_choke (the excitation half, in ConverterAnalytical) and the analytical
// LC-filter sizing (propose_dmc_design). Ported from MKF converter_models/DifferentialModeChoke.{h,cpp}
// (deleted in 3e0261fd). Shares the whole downstream core with CMC via ChokeDesign.hpp — a DMC is the
// same "EMI filter choke" pattern with subApplication=differentialModeNoiseFiltering and windings that
// each drive the flux the same way (vs a CMC where only the common mode does).
//
// Two spec modes for the inductance requirement (the wizard always supplies one):
//   * minimumImpedance[]{frequency,impedance} → L_min = Z / (2πf) at the LOWEST spec frequency
//   * minimumInductance (scalar)              → L_min directly (advanced / post-propose mode)
// propose_dmc_design synthesises an (L, C) LC-filter pair from a target attenuation when the user is in
// "help me" mode (the wizard then re-calls design_dmc with the proposed L as minimumInductance).
//
// The ngspice verification / waveform simulation (verify_attenuation, simulate_*) is NOT here — it is
// the ngspice half, alongside the CMC sims.

#include "MAS.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace Kirchhoff {

// The resolved DMC design (analog of CmcDesign). Scalars SI.
struct DmcDesign {
    double inputVoltage = 0.0;         // resolved mains voltage scalar [V]
    double operatingCurrent = 0.0;     // line (load) current [A]
    double lineFrequency = 0.0;        // [Hz]
    double switchingFrequency = 0.0;   // ripple (noise) frequency [Hz]; 0 = not supplied
    double ambientTemperature = 0.0;   // [°C]
    // Raw schema configuration string (singlePhase|singlePhaseBalanced|threePhase|threePhaseWithNeutral).
    // Kept as the plain string — NOT a MAS enum — so DMC never depends on a MAS type that the
    // Inputs-rooted MAS.hpp generation may or may not emit (CMC likewise takes numberOfWindings as a
    // plain int). Validated on parse; the winding count / analytical model are derived from it.
    std::string configuration = "singlePhase";
    int numberOfWindings = 1;          // derived from configuration (1/2/3/4)
    std::optional<double> peakCurrent; // explicit winding peak; else derived from operatingCurrent
    std::vector<MAS::ImpedanceAtFrequency> impedancePoints;  // resolved Z(f) requirements (may be empty)
    std::optional<double> minimumInductance;  // direct L bound (advanced mode)
    std::optional<double> filterCapacitance;  // the LC filter cap the design pairs with (from propose)
    // Diagnostics (the calculate_dmc_inputs `dmcDiagnostics` block).
    double computedInductance = 0.0;
    double computedMinFrequency = 0.0;
    double computedMaxFrequency = 0.0;
    double computedImpedanceAtMinFreq = 0.0;
};

// LC filter transfer function |H| ≈ 1/|1−ω²LC|; for f ≫ fc: attenuation ≈ 40·log10(f/fc). Inverse:
// fc = f / 10^(A/40), L = 1/(4π²fc²C). Public for unit tests (mirrors MKF's static).
double dmc_required_inductance(double targetAttenuationDb, double frequencyHz, double capacitanceF);

// Parse the wizard spec (flat DmcWizard payload: configuration, inputVoltage, operatingCurrent,
// lineFrequency, switchingFrequency?, ambientTemperature, minimumImpedance[]? | minimumInductance?,
// peakCurrent?, filterCapacitance?). The two inductance modes may coexist (an EMPTY minimumImpedance
// array never shadows a supplied minimumInductance; when both yield a value the stricter — larger —
// bound wins). Throws on missing required fields, non-positive operating current / line frequency /
// switching frequency / filter capacitance / impedance points / minimum inductance, and when NEITHER
// an impedance spec NOR a minimumInductance is given (no inductance target for the MagneticAdviser).
DmcDesign design_dmc(const nlohmann::json& spec);

// The MAS::Inputs the choke is designed around — the shared filter-choke designRequirements with
// subApplication=differentialModeNoiseFiltering + one operating point from analytical_differential_mode_choke.
MAS::Inputs build_dmc_inputs(const DmcDesign& d);

// "Help me with the design" LC sizing (ported from MKF propose_design, MINUS the ngspice verification
// block — that lives with the other ngspice sims). Returns {inductance, inductance_uH, capacitance,
// capacitance_nF, cutoffFrequency, cutoffFrequency_kHz, targetAttenuation_dB, peakCurrent,
// energyStorage_mJ, configuration, numberOfWindings}. The wizard reads `inductance` and re-calls
// design_dmc with it as minimumInductance.
nlohmann::json propose_dmc_design(const nlohmann::json& spec);

// ── ngspice DMC simulations (the wizard's EMI-spectrum + attenuation-check views; ngspice-gated) ──────
// Ported from MKF simulate_and_extract_waveforms + verify_attenuation (DmcSim.cpp).
// BOTH simulate the SAME LC filter: the caller's capacitance argument, else the spec's
// filterCapacitance, else the fc = fsw/10 auto-sizing (which THROWS without a switchingFrequency —
// no silent line-frequency substitute). Failed ngspice runs are surfaced, never papered over:
// simulate collects them in "failedFrequencies" (success:false when ALL fail); verify marks the row
// simulated:false with a null measuredAttenuation and judges on the theoretical value, saying so.
// simulate_dmc_waveforms: LC low-pass sim over the test frequencies → {success, converterWaveforms:
// [{frequency, time, inputVoltage, outputVoltage, inductorCurrent, dmAttenuation}], failedFrequencies?}.
// verify_dmc_attenuation: per-point {frequency, requiredAttenuation, measuredAttenuation|null,
// theoreticalAttenuation, simulated, passed, message} (required = 20·log10(Z/Rload); pass ≥ 0.9·required).
nlohmann::json simulate_dmc_waveforms(const DmcDesign& d, double inductance, double capacitance = 0.0);
nlohmann::json verify_dmc_attenuation(const DmcDesign& d, double inductance, double capacitance = 0.0);

} // namespace Kirchhoff
