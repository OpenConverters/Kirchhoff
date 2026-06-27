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
#include <cmath>
#include <algorithm>
#include "SasConverter.hpp"   // SAS::ideal_diode_drop — single source for the DIDEAL forward drop
#include "DimensionJson.hpp"      // PEAS::resolve_dimensional_values — canonical {nominal,min,max} resolver

namespace Kirchhoff {
namespace req {

using nlohmann::json;

constexpr double V_DERATE = 0.8;   // operate at <= 80% of the device voltage rating
constexpr double ESR_RIPPLE_FRACTION = 0.005;  // ESR ripple-voltage budget = 0.5% of Vout

// Forward drop of the DIDEAL rectifier the deck emits, for a converter that must COMPENSATE for it when
// sizing turns ratios etc. This DELEGATES to SAS::ideal_diode_drop — SAS owns the ideal-diode model (it
// stamps the leaf's forwardVoltage, which the CIAS emitter turns into the ngspice IS), so the design-time
// compensation here and the emitted .model share ONE definition and cannot drift. (NB: the IDEAL-fidelity
// diode; a real sourced diode carries its own datasheet Vf@I — real-rectifier compensation must use that.)
inline double dideal_diode_drop(double current) { return SAS::ideal_diode_drop(current); }

// "Design around the magnetic" (ABT #30 / della-Pollock flow): if the caller pinned the magnetizing
// inductance in the spec — any of magnetizingInductance / desiredInductance / inductance, as a number or
// {nominal} — the topology sizes the REST of the stage around THAT L instead of computing its own. Returns
// nullopt if none is provided (compute as before).
inline std::optional<double> provided_inductance(const json& designRequirements) {
    if (designRequirements.is_object())
        for (const char* k : {"magnetizingInductance", "desiredInductance", "inductance"})
            if (designRequirements.contains(k))
                return PEAS::resolve_dimensional_values(designRequirements.at(k));   // canonical resolver, not raw nominal
    return std::nullopt;
}

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

// --- semiconductor: body diode of a sourced FET (anti-parallel) ---
// A basic, SAS-valid diode requirement tagged role "bodyDiode": the fill DEFERS it (the FET it
// shadows carries the real device) rather than sourcing it as an independent rectifier. Ratings
// mirror the switch it shadows (it blocks the same bus and freewheels the same current).
inline json body_diode(double ratedVr, double ratedIf) {
    json r;
    r["deviceType"] = "diode";
    r["ratedReverseVoltage"] = ratedVr;
    r["ratedForwardCurrent"] = ratedIf;
    r["role"] = "bodyDiode";
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

// --- controller / gate driver (CTAS family): the control IC that drives the stage ---
// `category` is the CTAS controllerCategory discriminator (pwmController / llcController /
// pfcController / gateDriver / …) so the HS librarian sources the right CLASS of part for
// the topology's control mode. It lives at the TOP LEVEL of the seed's designRequirements
// (ctas/inputs/designRequirements.json requires `category` there — NOT under `function`,
// which is the PART's datasheetInfo discriminator, a different object). The selector also
// matches the converter's topology, Vin and fsw: for switching-converter categories the
// schema additionally requires `topology` + `switchingFrequency` in the seed, injected by
// finalize_control_seeds() once the assembled doc carries them.
inline json controller(const std::string& category) {
    json r;
    r["category"] = category;
    return r;
}

// The switching-converter controller categories: ctas/inputs/designRequirements.json requires
// `topology` + `switchingFrequency` in the seed for exactly these (so an ideal behavioural switch
// model can be built from the seed alone). Other categories (gateDriver, supervisor, …) need only
// `category`. Mirrors the schema's if/then enum.
inline bool is_switching_controller_category(const std::string& category) {
    return category == "pwmController" || category == "multiphaseController" ||
           category == "llcController" || category == "pfcController" ||
           category == "dualPwmController" || category == "phaseShiftController" ||
           category == "digitalController";
}

// Inject the topology + switching frequency that switching-controller seeds require, reading the
// frequency from the assembled doc's top-level designRequirements. Call once per build_*_tas after
// the stages and top-level designRequirements are set, passing the converter's peas `topology` enum
// value (e.g. "flybackConverter"). Non-switching controller seeds are left untouched.
inline void finalize_control_seeds(json& tas, const std::string& topology) {
    if (!tas.contains("topology") || !tas.at("topology").contains("stages")) return;
    const json& dreq = tas.at("inputs").at("designRequirements");
    for (auto& st : tas["topology"]["stages"]) {
        if (!st.contains("circuit") || !st.at("circuit").is_object()) continue;
        for (auto& c : st["circuit"]["components"]) {
            if (!c.contains("data") || !c.at("data").contains("controller")) continue;
            json& dr = c["data"]["inputs"]["designRequirements"];
            if (is_switching_controller_category(dr.value("category", std::string{}))) {
                dr["topology"] = topology;
                dr["switchingFrequency"] = dreq.at("switchingFrequency");
            }
        }
    }
}

// A ready-to-append physicalControl STAGE carrying one controller seed. role "control"
// => the assembler skips it in the power deck (the gate is driven by the stimulus /
// control law), but the HS librarian walks its circuit and sources the real control IC.
// Add ONE line — control_stage("<ctas-category>") — to a topology's stages array.
inline json control_stage(const std::string& category,
                          const std::string& stageName = "control",
                          const std::string& compName = "U1") {
    json comp;
    comp["name"] = compName;
    comp["data"]["controller"] = json::object();
    comp["data"]["inputs"]["designRequirements"] = controller(category);
    json brick;
    brick["name"] = stageName + "-brick";
    // A physicalControl stage must expose >=1 typed port (tas/topology.json). The controller's
    // gate-drive output is a signal-level ('control') terminal. The brick is sourced for the BOM
    // only — the power-deck assembler skips role:"control" stages — so it is NOT electrically wired
    // here (the interStageConnections to the switch gates belong to the control-circuit realisation),
    // which is why `connections` stays empty.
    json gatePort; gatePort["name"] = "gate_drive";
    gatePort["description"] = "Controller gate-drive / control-signal output.";
    brick["ports"] = json::array({gatePort});
    brick["components"] = json::array({comp});
    brick["connections"] = json::array();
    json s;
    s["name"] = stageName;
    s["role"] = "control";
    s["controlImplementation"] = "physical";
    s["circuit"] = brick;
    json pb; pb["port"] = "gate_drive"; pb["type"] = "control";
    s["ports"] = json::array({pb});
    return s;
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

// --- resistor (snubber damping R, current sense, bias/bleed, feedback divider) ---
// RAS-conformant: ras/inputs/designRequirements.json requires deviceType + resistance + powerRating
// for the resistor branch (unevaluatedProperties:false there, so ONLY RAS-defined fields). `role`
// is the optional converter-function enum (snubber/damping/bleed/currentSense/divider/...).
inline json resistor(double resistance, double powerRating, const std::string& role) {
    json r;
    r["deviceType"] = "resistor";
    r["resistance"]["nominal"] = resistance;
    r["powerRating"] = powerRating;
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
