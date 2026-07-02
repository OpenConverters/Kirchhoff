#pragma once

// Common-mode choke (CMC) — the requirement-synthesis half of the design, plus the MAS::Inputs
// assembly over analytical_common_mode_choke (the excitation half, in ConverterAnalytical).
//
// Ported from MKF converter_models/CommonModeChoke.{h,cpp} (deleted from MKF in 3e0261fd — MKF
// reverted to pure magnetics; Kirchhoff owns every converter/component model). The generic reactance
// math + the shared DesignRequirements skeleton live in ChokeDesign.hpp; this file carries only the
// CMC-specific spec conversions (insertion loss, switch-node noise estimation, regulatory limits) and
// the spec parser. The three specification modes all collapse to a set of (frequency, |Z|) points:
//   1. minimumImpedance[]    : direct {frequency, impedance} pairs
//   2. targetInsertionLoss[] : {frequency, insertionLoss[dB]} pairs, Z_cm = Z_line·(10^(IL/20)−1)
//   3. Estimate from noise   : parasiticCap_pF + dvdt_V_ns (+ safetyMargin_dB, regulatoryStandard)
//                              → I_cm = C·dV/dt → noise dBµV over the limit → required Z at 150 kHz
// Required CM inductance L = max over points of Z/(2πf); the advanced ("I know the design I want")
// mode pins the inductance to `desiredInductance` and excites at `designFrequency` instead.
//
// A CMC is a COMPONENT, not a converter: no TAS, no stages, no PWM stimulus. The deliverable is the
// MAS::Inputs (designRequirements + one CM operating point) handed to MKF's MagneticAdviser.

#include "MAS.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace Kirchhoff {

// ── CMC-specific spec conversions (public for unit tests, mirroring MKF's statics) ──────────────────
// Single-stage CM low-pass: IL(dB) ≈ 20·log10(Z_cm/Z_LISN + 1) → Z_cm = Z_LISN·(10^(IL/20) − 1).
double cmc_insertion_loss_to_impedance(double insertionLossDb, double lineImpedanceOhms);
// Noise-estimation mode: I_cm = C·dV/dt, V_noise = I_cm·(Z_line/2) in dBµV, attenuation needed to
// reach the regulatory limit (+ margin) → required CM impedance at the test frequency.
double cmc_noise_params_to_impedance(double parasiticCapPf,
                                     double dvdtVPerNs,
                                     double lineImpedanceOhms,
                                     double safetyMarginDb,
                                     double testFrequencyHz = 150e3,
                                     double limitDbuv = 66.0);
// Quasi-peak conducted-emissions limit at 150 kHz (dBµV) for a named standard. THROWS on an
// unrecognized name (no silent 66 dBµV fallback — the MKF behaviour this replaces hid typos).
double cmc_emissions_limit_dbuv(const std::string& standardName);

// The resolved CMC design (the analog of BuckDesign & co: design_cmc(spec) → CmcDesign →
// build_cmc_inputs). Scalars are SI; impedancePoints carries the union of all three spec modes.
struct CmcDesign {
    double operatingVoltage = 0.0;     // mains voltage, resolved scalar [V]
    double operatingCurrent = 0.0;     // line (load) current [A]
    double lineFrequency = 0.0;        // [Hz]
    double ambientTemperature = 0.0;   // [°C]
    double lineImpedance = 0.0;        // LISN reference impedance per line [Ω]
    int numberOfWindings = 2;          // 2 (L/N), 3 (3-phase), 4 (3-phase + N)
    double parasiticCapPf = 0.0;       // 0 = not supplied (drives the CM excitation amplitude)
    double dvdtVPerNs = 0.0;           // 0 = not supplied
    std::vector<MAS::ImpedanceAtFrequency> impedancePoints;  // resolved Z(f) requirements
    double computedInductance = 0.0;   // max L over impedancePoints [H]
    double dominantFrequency = 0.0;    // frequency of the hardest impedance point [Hz]
    double dominantImpedance = 0.0;    // |Z| of that point [Ω]
    std::optional<double> desiredInductance;  // advanced mode: pinned CM inductance [H]
    double designFrequency = 150e3;    // advanced mode: excitation frequency [Hz]
};

// Parse + resolve the wizard spec (the flat CmcWizard payload: operatingVoltage, operatingCurrent,
// lineFrequency, ambientTemperature, lineImpedance?, numberOfWindings?, minimumImpedance[]?,
// targetInsertionLoss[]?, parasiticCap_pF?, dvdt_V_ns?, safetyMargin_dB?, regulatoryStandard?,
// desiredInductance?, designFrequency?). Throws on missing required fields, numberOfWindings ∉
// [2,4], non-positive operating current / line frequency / impedance points, and when NO spec mode
// yields an inductance (no impedance point and no desiredInductance).
CmcDesign design_cmc(const nlohmann::json& spec);

// The MAS::Inputs the choke is designed around — the shared filter-choke designRequirements (see
// ChokeDesign.hpp) with subApplication=commonModeNoiseFiltering + one operating point from
// analytical_common_mode_choke at the dominant (or design) frequency.
MAS::Inputs build_cmc_inputs(const CmcDesign& d);

// ── ngspice CMC simulations (the wizard's EMI / waveform views; require an ngspice-enabled build) ─────
// Ported from MKF simulate_realistic_cmc + simulate_and_extract_waveforms (CmcSim.cpp). Both return
// {"success": false, "error": ...} when the build has no libngspice (callers branch on it).
//
// simulate_cmc_ideal_waveforms: per-winding CM sine (I_cm = C·dV/dt) → {success, inputs:{operatingPoints:
// [<simulated OperatingPoint>]}, converterWaveforms:[], cmcDiagnostics:{computedInductance}}.
// simulate_cmc_lisn_waveforms: CISPR LISN test over the impedance-spec frequencies → {success,
// converterWaveforms:[{frequency, time, inputVoltage, windingCurrents, lisnVoltage,
// commonModeAttenuation, commonModeImpedance, theoreticalImpedance}]}.
nlohmann::json simulate_cmc_ideal_waveforms(const CmcDesign& d, double inductance, double parasiticCapPf,
                                            double dvdtVPerNs, int numberOfPeriods = 2,
                                            int numberOfSteadyStatePeriods = 10);
nlohmann::json simulate_cmc_lisn_waveforms(const CmcDesign& d, double inductance);

} // namespace Kirchhoff
