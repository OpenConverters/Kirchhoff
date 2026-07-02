#include "DatasheetModels.hpp"

#include <stdexcept>
#include <string>

namespace Kirchhoff {

namespace {
using nlohmann::json;

// Read a numeric requirement (plain number in the semiconductor designRequirements). Returns nullptr
// (via has=false) when absent so the caller can skip an optional field rather than fabricate one.
bool req_num(const json& dr, const char* key, double& out) {
    if (!dr.is_object() || !dr.contains(key) || !dr.at(key).is_number()) return false;
    out = dr.at(key).get<double>();
    return true;
}

// A gate threshold isn't a design requirement — it's a switch-model constant. The ideal switch uses
// 2.5 V and the PWM stimulus swings 0→5 V, so any value in that band drives identically; we keep 2.5.
constexpr double MODEL_GATE_VTH = 2.5;

// Build the MOSFET datasheet model from its requirements. onResistance is REQUIRED downstream (SAS real
// mosfet leaf) — without maximumOnResistance there is nothing real to model, so we leave the seed ideal.
bool derive_mosfet(json& mosfet, const json& dr) {
    if (mosfet.contains("manufacturerInfo")) return false;   // already a real/bound part
    double ron;
    if (!req_num(dr, "maximumOnResistance", ron)) return false;

    // The mosfet electrical block is a CLOSED schema requiring drainSourceVoltage, onResistance,
    // continuousDrainCurrent, gateThresholdVoltage AND totalGateCharge. Ratings + Rds(on) are real
    // design values; gateThresholdVoltage and totalGateCharge are GATE-DRIVE params that do NOT enter the
    // power-circuit sim (the gate is driven by an ideal PULSE source, the switch model is Ron/Vth only).
    // They exist purely to keep the object schema-valid, so we supply a model constant (Vth) and an
    // order-of-magnitude estimate (Qg ≈ 2 nC/A of rated current) rather than fabricating precise data —
    // and neither affects a single simulated sample.
    double vds = 0, idc = 0;
    const bool haveVds = req_num(dr, "ratedDrainSourceVoltage", vds);
    const bool haveIdc = req_num(dr, "ratedContinuousDrainCurrent", idc);

    json elec;
    elec["onResistance"] = ron;
    elec["gateThresholdVoltage"]["nominal"] = MODEL_GATE_VTH;
    elec["drainSourceVoltage"] = haveVds ? vds : 0.0;
    elec["continuousDrainCurrent"] = haveIdc ? idc : 0.0;
    elec["totalGateCharge"] = 2e-9 * (haveIdc ? idc : 1.0);   // schema-required, simulation-irrelevant

    json& mfg = mosfet["manufacturerInfo"];
    mfg["name"] = "requirements-derived";
    mfg["datasheetInfo"]["part"]["partNumber"] = "requirements-derived";
    mfg["datasheetInfo"]["part"]["technology"] = "Si";
    mfg["datasheetInfo"]["electrical"] = std::move(elec);
    return true;
}

// Build the diode datasheet model. forwardVoltage is REQUIRED downstream (SAS real diode leaf).
bool derive_diode(json& diode, const json& dr) {
    if (diode.contains("manufacturerInfo")) return false;
    double vf;
    if (!req_num(dr, "maximumForwardVoltage", vf)) return false;

    json elec;
    elec["forwardVoltage"] = vf;
    double v;
    if (req_num(dr, "ratedReverseVoltage", v)) elec["reverseVoltage"] = v;
    if (req_num(dr, "ratedForwardCurrent", v)) elec["forwardCurrent"] = v;

    json& mfg = diode["manufacturerInfo"];
    mfg["name"] = "requirements-derived";
    mfg["datasheetInfo"]["part"]["partNumber"] = "requirements-derived";
    mfg["datasheetInfo"]["part"]["technology"] = "Si";
    mfg["datasheetInfo"]["electrical"] = std::move(elec);
    return true;
}
}  // namespace

json derive_datasheet_models(const json& tasIn) {
    json tas = tasIn;
    if (!tas.contains("topology") || !tas.at("topology").contains("stages")) return tas;
    for (auto& stage : tas.at("topology").at("stages")) {
        if (!stage.contains("circuit") || !stage.at("circuit").is_object()) continue;
        auto& circuit = stage.at("circuit");
        if (!circuit.contains("components")) continue;
        for (auto& comp : circuit.at("components")) {
            if (!comp.contains("data") || !comp.at("data").is_object()) continue;
            auto& data = comp.at("data");
            if (!data.contains("semiconductor") || !data.at("semiconductor").is_object()) continue;
            // requirements live at data.inputs.designRequirements (per-component seed the design baked)
            if (!data.contains("inputs") || !data.at("inputs").is_object()
                || !data.at("inputs").contains("designRequirements"))
                continue;
            const json& dr = data.at("inputs").at("designRequirements");
            auto& semi = data.at("semiconductor");
            if (semi.contains("mosfet") && semi.at("mosfet").is_object()) derive_mosfet(semi.at("mosfet"), dr);
            else if (semi.contains("diode") && semi.at("diode").is_object()) derive_diode(semi.at("diode"), dr);
        }
    }
    return tas;
}

}  // namespace Kirchhoff
