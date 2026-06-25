#include "Zeta.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kRippleRatioL1 = 0.4;   // ΔIL1 / IL1,avg
constexpr double kL2RipplePct = 0.30;    // ΔIL2 / IL2,avg
constexpr double kCcRipplePct = 0.05;    // ΔVCc / VCc
constexpr double kCoRipplePct = 0.01;    // ΔVo  / Vo
// Zeta CCM ideal-ish duty: D = (Vo+Vd) / (Vin*eff + Vo + Vd).
double duty(double vin, double vo, double vd, double eff) { return (vo + vd) / (vin * eff + vo + vd); }
} // namespace

ZetaDesign design_zeta(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ZetaDesign d{};
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

    const double iout = d.outputPower / d.outputVoltage, fsw = d.switchingFrequency;
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    d.dutyCycle = duty(d.inputVoltage, d.outputVoltage, d.diodeDrop, d.efficiency);

    // L1 sized at the worst corner (max Vin) for its current-ripple target (MKF).
    const double dMax = duty(vinMax, d.outputVoltage, d.diodeDrop, d.efficiency);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatioL1) * iL1avg;
    d.inductanceL1 = vinMax * dMax / (dIL1 * fsw);
    // L2, Cc, Cout at the operating point (both inductors see Vin·D during ON).
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    d.inductanceL2 = d.inputVoltage * d.dutyCycle / (dIL2 * fsw);
    const double dVcc = cfg::get(d.config, "couplingCapRipple", kCcRipplePct) * d.outputVoltage;       // VCc = Vout
    d.couplingCapacitance = iout * d.dutyCycle / (dVcc * fsw);
    const double dVo = cfg::get(d.config, "outputCapRipple", kCoRipplePct) * d.outputVoltage;
    d.outputCapacitance = dIL2 / (8.0 * fsw * dVo);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    return d;
}

json build_zeta_tas(const ZetaDesign& d) {
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

    const double fsw = d.switchingFrequency, iout = d.outputPower / d.outputVoltage;
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatioL1) * (iout * d.dutyCycle / (1.0 - d.dutyCycle));
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    const double vSwing = d.inputVoltage + d.outputVoltage;   // Cc holds ~Vo; switch/diode block Vin+Vo

    auto inductor = [&](double L, double iAvg, double iPkPk) {
        json m; m["magnetic"] = json::object();
        const double iPk = iAvg + iPkPk / 2.0, iRms = std::sqrt(iAvg * iAvg + iPkPk * iPkPk / 12.0);
        m["inputs"] = req::magnetic_inputs(L, 0.2, {}, {"primary"}, std::nullopt, 25.0,
            {req::winding_excitation("triangular", fsw, iPk, iRms, iAvg, iPkPk, d.dutyCycle,
                                     vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing)});
        return m;
    };
    json L1 = inductor(d.inductanceL1, iout * d.dutyCycle / (1.0 - d.dutyCycle), dIL1);
    json L2 = inductor(d.inductanceL2, iout, dIL2);
    json cc; cc["capacitor"] = json::object();
    cc["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.couplingCapacitance;
    cc["inputs"]["designRequirements"]["ratedVoltage"] = vSwing / cfg::v_derate(d.config);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage / cfg::v_derate(d.config);
    json mq = mosfet();
    mq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", vSwing / cfg::v_derate(d.config),
                                                     iout + iout * d.dutyCycle / (1.0 - d.dutyCycle),
                                                     0.01 * d.outputPower, 125.0);
    json md = diode();
    md["inputs"]["designRequirements"] = req::diode(vSwing / cfg::v_derate(d.config), iout / 0.7,
                                                    (vSwing < 100.0) ? 0.6 : 1.2, 0.05 / fsw);

    // Zeta cell — high-side switch, non-inverting. D1 catch: anode at gnd, cathode at node_X.
    json cell; cell["name"] = "zeta-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("Q1", mq), comp("L1", L1), comp("Cc", cc),
                                      comp("D1", md), comp("L2", L2)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
        // node_SW: switch -> L1 (to gnd) + coupling cap
        conn("node_sw",  {pin("Q1", "source"), pin("L1", "primary_start"), pin("Cc", "1")}),
        conn("gnd_net",  {pin("L1", "primary_end"), pin("D1", "anode"), prt("gnd")}),
        // node_X: coupling cap -> catch-diode cathode + output inductor
        conn("node_x",   {pin("Cc", "2"), pin("D1", "cathode"), pin("L2", "primary_start")}),
        conn("vout_net", {pin("L2", "primary_end"), prt("vout")}),
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
        pstage("zetaCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("zetaCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("zetaCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("zetaCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "zetaCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    return tas;
}

} // namespace Kirchhoff
