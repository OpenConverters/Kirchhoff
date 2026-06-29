#include "TwoSwitchForward.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kMaxDuty = 0.5;
constexpr double kRippleRatio = 0.4;
} // namespace

TwoSwitchForwardDesign design_two_switch_forward(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    TwoSwitchForwardDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double iout = d.outputPower / d.outputVoltage;
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double n = vinMin * cfg::get(d.config, "maxDutyCycle", kMaxDuty) / (d.outputVoltage + d.diodeDrop);
    n = std::round(n * 100.0) / 100.0;
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * n / (d.switchingFrequency * iout));
    const double tOn = cfg::get(d.config, "maxDutyCycle", kMaxDuty) / d.switchingFrequency;
    d.outputInductance = (vinMax / n - d.diodeDrop - d.outputVoltage) * tOn / cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    d.dutyCycle = n * (d.outputVoltage + d.diodeDrop) / d.inputVoltage;
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = 100e-6;
    return d;
}

json build_two_switch_forward_tas(const TwoSwitchForwardDesign& d) {
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };
    auto mosfet = []() { json j; j["semiconductor"]["mosfet"] = json::object(); return j; };
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };

    const double n = d.turnsRatio, Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double T = 1.0 / fsw, Dn = d.dutyCycle, Vin = d.inputVoltage, Vout = d.outputVoltage;
    const double iout = d.outputPower / d.outputVoltage;

    // --- per-winding electrical stresses (two-switch forward; the two clamp diodes reset to ±Vin) ---
    // Magnetizing current ramp during ON, peak = Vin*D*T/Lm; resets through the clamp diodes (the
    // primary itself carries the reset current back to the rails, no separate demag winding).
    const double ImagPk  = Vin * Dn * T / Lm;
    const double ImagRms = ImagPk * std::sqrt(Dn / 3.0);
    const double dILout  = (Vin / n - d.diodeDrop - Vout) * (Dn * T) / d.outputInductance;
    const double IpkLout = iout + dILout / 2.0;
    const double IrmsLout = std::sqrt(iout * iout + dILout * dILout / 12.0);
    // Secondary carries the inductor current during ON (conducts D).
    const double IcSec   = iout, IpkSec = IpkLout;
    const double IrmsSec = std::sqrt(Dn) * std::sqrt(iout * iout + dILout * dILout / 12.0);
    // Primary = reflected secondary current (Iout/n) + magnetizing ramp, during ON.
    const double IcPri   = iout / n;
    const double IpkPri  = IpkSec / n + ImagPk;
    const double IrmsPri = std::sqrt(Dn) * IcPri + ImagRms;
    // Winding voltages: primary +Vin during ON, -Vin during reset (clamped by D1/D2); peak Vin, swing 2*Vin.
    const double vPriPk = Vin, vPriPkPk = 2.0 * Vin;
    const double vPriRms = Vin;                                  // square ±Vin
    const double vSecPk = Vin / n, vSecPkPk = Vin / n;
    const double vSecRms = std::sqrt(Dn) * (Vin / n);
    const double vLonF = Vin / n - d.diodeDrop - Vout, vLoff = Vout;
    const double vLoutPk = std::max(std::abs(vLonF), vLoff), vLoutPkPk = std::abs(vLonF) + vLoff;
    const double vLoutRms = std::sqrt(Dn * vLonF * vLonF + (1.0 - Dn) * vLoff * vLoff);

    // --- semiconductor stresses (two-switch forward) ---
    // Each primary switch blocks Vin_max: the two clamp diodes D1/D2 hold the switch nodes to the rails
    // during reset, so neither MOSFET sees more than the input bus. ratedVds = Vin_max / V_DERATE.
    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsPri * IrmsPri);
    // Clamp/reset diodes D1,D2 are REAL rectifiers (not FET body diodes): they steer the magnetizing
    // reset current back to the rails and reverse-block the full input bus Vin_max. They carry the
    // magnetizing reset current (peak ImagPk).
    const double ratedVrClamp = d.inputVoltageMax / cfg::v_derate_diode(d.config);
    const double maxVfClamp    = (ratedVrClamp < 100.0) ? 0.6 : 1.2;
    // Output forward/freewheel rectifiers reverse-block the secondary peak Vin_max/n, carry ~Iout.
    const double ratedVrSec = (d.inputVoltageMax / n) / cfg::v_derate_diode(d.config);
    const double maxVfSec    = (ratedVrSec < 100.0) ? 0.6 : 1.2;
    const double maxTrr      = 0.05 * T;
    const auto mreq = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0);
    auto mosfetReq = [&]() { json j = mosfet(); j["inputs"]["designRequirements"] = mreq; return j; };
    auto clampDiode = [&]() { json j = diode();
        j["inputs"]["designRequirements"] = req::diode(ratedVrClamp, ImagPk / 0.7, maxVfClamp, maxTrr); return j; };
    auto rectDiode = [&]() { json j = diode();
        j["inputs"]["designRequirements"] = req::diode(ratedVrSec, iout / 0.7, maxVfSec, maxTrr); return j; };

    // 2-winding transformer (primary + secondary, no demag): turnsRatios = [n] -> 2 excitations.
    std::vector<std::string> xfmrIso{"primary", "secondary"};
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {n}, xfmrIso, std::nullopt, 25.0, {
        req::winding_excitation("forwardPrimary",   fsw, IpkPri, IrmsPri, 0.0, IpkPri, Dn,
                                vPriPk, vPriRms, 0.0, vPriPkPk),
        req::winding_excitation("forwardSecondary", fsw, IpkSec, IrmsSec, IcSec, dILout, Dn,
                                vSecPk, vSecRms, 0.0, vSecPkPk)});
    // Output filter inductor: single winding (turnsRatios = []) -> 1 excitation, DC-biased at Iout.
    json lout; lout["magnetic"] = json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
        req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, iout, dILout, Dn,
                                vLoutPk, vLoutRms, 0.0, vLoutPkPk)});
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // forward cell: 2 switches (driven together) + 2 clamp diodes + transformer + output stage.
    json cell; cell["name"] = "two-switch-forward-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("Q1", mosfetReq()), comp("Q2", mosfetReq()),
                                      comp("D1", clampDiode()), comp("D2", clampDiode()), comp("T1", xfmr),
                                      comp("Dfwd", rectDiode()), comp("Dfw", rectDiode()), comp("Lout", lout)});
    cell["connections"] = json::array({
        // High side: vin -> Q1 -> sw1_out -> primary_start; clamp D1 (gnd -> sw1_out), clamp D2 (pri_gnd -> vin)
        conn("vin_net",  {pin("Q1", "drain"), pin("D2", "cathode"), prt("vin")}),
        conn("sw1_out",  {pin("Q1", "source"), pin("D1", "cathode"), pin("T1", "primary_start")}),
        conn("pri_gnd",  {pin("T1", "primary_end"), pin("Q2", "drain"), pin("D2", "anode")}),
        // secondary -> forward/freewheel diodes -> output inductor
        conn("sec_in",   {pin("T1", "secondary1_start"), pin("Dfwd", "anode")}),
        conn("sec_rect", {pin("Dfwd", "cathode"), pin("Dfw", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("D1", "anode"), pin("T1", "secondary1_end"),
                          pin("Dfw", "anode"), prt("gnd")}),
        conn("gate_net", {pin("Q1", "gate"), pin("Q2", "gate"), prt("gate")})});

    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("forwardCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("forwardCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("forwardCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("forwardCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "forwardCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    req::finalize_control_seeds(tas, "twoSwitchForwardConverter");  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
