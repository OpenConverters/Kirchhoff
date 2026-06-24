#include "TwoSwitchForward.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    if (j.contains("minimum") && j.contains("maximum"))
        return 0.5 * (j.at("minimum").get<double>() + j.at("maximum").get<double>());
    throw std::runtime_error("two-switch forward design: no nominal");
}
constexpr double kMaxDuty = 0.5;
constexpr double kRippleRatio = 0.4;
} // namespace

TwoSwitchForwardDesign design_two_switch_forward(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    TwoSwitchForwardDesign d{};
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
    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double iout = d.outputPower / d.outputVoltage;
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double n = vinMin * kMaxDuty / (d.outputVoltage + d.diodeDrop);
    n = std::round(n * 100.0) / 100.0;
    d.turnsRatio = n;
    d.magnetizingInductance = vinMin * n / (d.switchingFrequency * iout);
    const double tOn = kMaxDuty / d.switchingFrequency;
    d.outputInductance = (vinMax / n - d.diodeDrop - d.outputVoltage) * tOn / kRippleRatio;
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
    auto diode  = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };

    // 2-winding transformer (primary + secondary, no demag): turnsRatios = [n].
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    { json rn; rn["nominal"] = d.turnsRatio; xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }
    json lout; lout["magnetic"] = json::object();
    lout["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.outputInductance;
    lout["inputs"]["designRequirements"]["turnsRatios"] = json::array();
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // forward cell: 2 switches (driven together) + 2 clamp diodes + transformer + output stage.
    json cell; cell["name"] = "two-switch-forward-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("Q1", mosfet()), comp("Q2", mosfet()),
                                      comp("D1", diode()), comp("D2", diode()), comp("T1", xfmr),
                                      comp("Dfwd", diode()), comp("Dfw", diode()), comp("Lout", lout)});
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
    return tas;
}

} // namespace Kirchhoff
