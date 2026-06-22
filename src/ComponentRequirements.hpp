#pragma once

// ComponentRequirements — build the per-component `inputs.designRequirements` (and, for magnetics,
// `inputs.operatingPoints`) that a converter design imposes on each power-train part, detailed enough
// to source a real part. These populate the EXISTING per-family designRequirements schemas
// (SAS/CAS/MAS inputs/designRequirements.json) — no schema invention.
//
// Conventions (from standard power-supply part-selection practice; see Kirchhoff/docs/component-requirements.md):
//   * Voltage ratings: required rating = peak voltage stress / V_DERATE (80% derating, IPC-9592).
//   * Current ratings: semiconductor continuous rating sized to the PEAK device current (conservative);
//     capacitor ripple-current rating = the RMS current the cap actually carries.
//   * Stresses are evaluated at the worst-case input-voltage corner (max V for voltage, min V for current).
//
// These are REQUIREMENTS (minimum ratings / maximum allowed parasitics), not a chosen part. The
// component object itself (semiconductor/capacitor/magnetic) stays empty until a part is sourced.

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <optional>

namespace Kirchhoff {
namespace req {

using nlohmann::json;

constexpr double V_DERATE = 0.8;   // operate at <= 80% of the device voltage rating
constexpr double ESR_RIPPLE_FRACTION = 0.005;  // ESR ripple-voltage budget = 0.5% of Vout

// --- semiconductor: MOSFET main switch ---
inline json mosfet(const std::string& role, double ratedVds, double ratedId,
                   double maxRdsOn, double maxTjC) {
    json r;
    r["deviceType"] = "mosfet";
    r["role"] = role;
    r["ratedDrainSourceVoltage"] = ratedVds;
    r["ratedContinuousDrainCurrent"] = ratedId;
    r["maximumOnResistance"] = maxRdsOn;
    r["maximumJunctionTemperature"] = maxTjC;
    return r;
}

// --- semiconductor: rectifier diode (role omitted — the SAS role enum has no generic output rectifier) ---
inline json diode(double ratedVr, double ratedIf, double maxVf,
                  std::optional<double> maxTrr = std::nullopt) {
    json r;
    r["deviceType"] = "diode";
    r["ratedReverseVoltage"] = ratedVr;
    r["ratedForwardCurrent"] = ratedIf;
    r["maximumForwardVoltage"] = maxVf;
    if (maxTrr) r["maximumReverseRecoveryTime"] = *maxTrr;
    return r;
}

// --- capacitor ---
inline json capacitor(double capacitance, double ratedVoltage, double minRippleCurrentRms,
                      double maxEsr, const std::string& role) {
    json r;
    r["capacitance"]["nominal"] = capacitance;
    r["ratedVoltage"] = ratedVoltage;
    r["minimumRippleCurrent"] = minRippleCurrentRms;
    r["maximumEsr"] = maxEsr;
    r["role"] = role;
    return r;
}

// One winding's excitation: current (peak drives saturation, rms drives heating) AND voltage (the
// volt-seconds drive the flux swing / core loss). The PEAS excitation schema requires both, plus
// frequency. processed-waveform peaks are absolute magnitudes.
inline json winding_excitation(const std::string& currentLabel, double frequency,
                               double iPeak, double iRms, double iOffset, double iPkPk,
                               std::optional<double> dutyCycle,
                               double vPeak, double vRms, double vOffset, double vPkPk) {
    json ci;
    ci["label"] = currentLabel;
    ci["peak"] = iPeak; ci["rms"] = iRms; ci["offset"] = iOffset; ci["peakToPeak"] = iPkPk;
    if (dutyCycle) ci["dutyCycle"] = *dutyCycle;
    json vo;
    vo["label"] = "rectangular";
    vo["peak"] = vPeak; vo["rms"] = vRms; vo["offset"] = vOffset; vo["peakToPeak"] = vPkPk;
    if (dutyCycle) vo["dutyCycle"] = *dutyCycle;
    json e;
    e["frequency"] = frequency;
    e["current"]["processed"] = ci;
    e["voltage"]["processed"] = vo;
    return e;
}

// --- magnetic: full inputs (designRequirements + one operating point) ---
inline json magnetic_inputs(double Lm, double lmTolerance,
                            const std::vector<double>& turnsRatios,
                            const std::vector<std::string>& isolationSides,
                            std::optional<double> isolationVoltage,
                            double ambientC,
                            const std::vector<json>& excitationsPerWinding) {
    json dr;
    dr["magnetizingInductance"]["nominal"] = Lm;
    dr["magnetizingInductance"]["minimum"] = Lm * (1.0 - lmTolerance);
    dr["magnetizingInductance"]["maximum"] = Lm * (1.0 + lmTolerance);
    dr["turnsRatios"] = json::array();
    for (double n : turnsRatios) dr["turnsRatios"].push_back(json{{"nominal", n}});
    dr["isolationSides"] = isolationSides;
    if (isolationVoltage) {
        dr["insulation"]["mainSupplyVoltage"]["nominal"] = *isolationVoltage;
        dr["insulation"]["insulationType"] = "reinforced";
        dr["insulation"]["standards"] = json::array({"IEC 62368-1"});
    }
    json op;
    op["conditions"]["ambientTemperature"] = ambientC;
    op["excitationsPerWinding"] = excitationsPerWinding;
    json inputs;
    inputs["designRequirements"] = dr;
    inputs["operatingPoints"] = json::array({op});
    return inputs;
}

} // namespace req
} // namespace Kirchhoff
