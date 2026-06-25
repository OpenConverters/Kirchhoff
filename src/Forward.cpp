#include "Forward.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kMaxDuty = 0.5;       // single-switch forward (1:1 demag reset)
constexpr double kRippleRatio = 0.4;
} // namespace

ForwardDesign design_forward(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ForwardDesign d{};
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
    // Turns ratio n = Vin_min*D_max/(Vout+Vd) so D(Vin_min)=D_max (MKF). Rounded to 2 dp.
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double n = vinMin * cfg::get(d.config, "maxDutyCycle", kMaxDuty) / (d.outputVoltage + d.diodeDrop);
    n = std::round(n * 100.0) / 100.0;
    d.turnsRatio = n;
    // Magnetizing inductance: Lm = Vin_min / (fsw * reflected secondary current), reflected = Iout/n.
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * n / (d.switchingFrequency * iout));
    // Output inductor: Lout = (Vin_max/n - Vd - Vout) * tOn / rippleRatio,  tOn = D_max/fsw.
    const double tOn = cfg::get(d.config, "maxDutyCycle", kMaxDuty) / d.switchingFrequency;
    d.outputInductance = (vinMax / n - d.diodeDrop - d.outputVoltage) * tOn / cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    // Operating (deck) duty at the nominal input.
    d.dutyCycle = n * (d.outputVoltage + d.diodeDrop) / d.inputVoltage;
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = 100e-6;   // matches MKF forward (Cout=100u)
    return d;
}

json build_forward_tas(const ForwardDesign& d) {
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

    const double n = d.turnsRatio, Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double iout = d.outputPower / d.outputVoltage;

    // --- component PEAS docs (seeds; designRequirements carries the sim-relevant values) ---
    // 3-winding transformer: turnsRatios = [1 (demag/reset), n (secondary)].
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json r1; r1["nominal"] = 1.0; json rn; rn["nominal"] = n;
      xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({r1, rn}); }
    // Output filter inductor: single winding (turnsRatios = []).
    json lout; lout["magnetic"] = json::object();
    lout["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.outputInductance;
    lout["inputs"]["designRequirements"]["turnsRatios"] = json::array();
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    auto dpdoc = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // --- forward power cell: switch + 3-winding transformer + demag diode + forward/freewheel diodes
    // + output inductor. Dot orientations match MKF (primary & secondary in-phase; demag reversed). ---
    json cell; cell["name"] = "forward-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("Q1", mosfet), comp("T1", xfmr), comp("Ddemag", dpdoc()),
                                      comp("Dfwd", dpdoc()), comp("Dfw", dpdoc()), comp("Lout", lout)});
    cell["connections"] = json::array({
        // primary: vin -> Q1 -> pri_node(primary_start); primary_end -> gnd
        conn("vin_net",  {pin("Q1", "drain"), pin("Ddemag", "cathode"), prt("vin")}),
        conn("pri_node", {pin("Q1", "source"), pin("T1", "primary_start")}),
        // demag (secondary1): start at gnd (reversed), end -> demag diode anode
        conn("demag_in", {pin("T1", "secondary1_end"), pin("Ddemag", "anode")}),
        // secondary (secondary2): start at rectifier (in-phase dot), end -> gnd
        conn("sec_in",   {pin("T1", "secondary2_start"), pin("Dfwd", "anode")}),
        // rectified node: forward-diode cathode + freewheel-diode cathode + output inductor
        conn("sec_rect", {pin("Dfwd", "cathode"), pin("Dfw", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("T1", "primary_end"), pin("T1", "secondary1_start"),
                          pin("T1", "secondary2_end"), pin("Dfw", "anode"), prt("gnd")}),
        conn("gate_net", {pin("Q1", "gate"), prt("gate")})});

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
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fsw;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    (void)iout;
    return tas;
}

} // namespace Kirchhoff
