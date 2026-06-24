#include "Sepic.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
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
    throw std::runtime_error("sepic design: no nominal");
}
// MKF Sepic.h ripple targets.
constexpr double kRippleRatioL1 = 0.4;   // ΔIL1 / IL1,avg (current_ripple_ratio)
constexpr double kL2RipplePct = 0.30;    // ΔIL2 / IL2,avg
constexpr double kCsRipplePct = 0.05;    // ΔVCs / VCs
constexpr double kCoRipplePct = 0.01;    // ΔVo  / Vo
// SEPIC CCM ideal-ish duty: D = (Vo+Vd) / (Vin*eff + Vo + Vd).
double duty(double vin, double vo, double vd, double eff) { return (vo + vd) / (vin * eff + vo + vd); }
} // namespace

SepicDesign design_sepic(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    SepicDesign d{};
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
    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double iout = d.outputPower / d.outputVoltage, fsw = d.switchingFrequency;
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    const double Vo = d.outputVoltage + d.diodeDrop;
    // Operating duty (deck/stimulus) at the nominal input.
    d.dutyCycle = duty(d.inputVoltage, d.outputVoltage, d.diodeDrop, d.efficiency);

    // L1 sized at the worst corner (max Vin) for its current-ripple target (MKF process_design_requirements).
    const double dMax = duty(vinMax, d.outputVoltage, d.diodeDrop, d.efficiency);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double dIL1 = kRippleRatioL1 * iL1avg;
    d.inductanceL1 = vinMax * dMax / (dIL1 * fsw);
    // L2, Cs, Cout sized at the operating point (MKF generate_ngspice_circuit).
    const double dIL2 = kL2RipplePct * iout;
    d.inductanceL2 = d.inputVoltage * d.dutyCycle / (dIL2 * fsw);
    const double dVcs = kCsRipplePct * d.inputVoltage;        // VCs = Vin
    d.couplingCapacitance = iout * d.dutyCycle / (dVcs * fsw);
    const double dVo = kCoRipplePct * d.outputVoltage;
    d.outputCapacitance = dIL2 / (8.0 * fsw * dVo);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    return d;
}

json build_sepic_tas(const SepicDesign& d) {
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
    const double dIL1 = kRippleRatioL1 * (iout * d.dutyCycle / (1.0 - d.dutyCycle));
    const double dIL2 = kL2RipplePct * iout;
    const double vSwing = d.inputVoltage + d.outputVoltage;

    // Two single-winding inductors with full inputs (so they auto-bind to real magnetics).
    auto inductor = [&](double L, double iAvg, double iPkPk) {
        json m; m["magnetic"] = json::object();
        const double iPk = iAvg + iPkPk / 2.0, iRms = std::sqrt(iAvg * iAvg + iPkPk * iPkPk / 12.0);
        m["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"}, std::nullopt, 25.0,
            {req::winding_excitation("triangular", fsw, iPk, iRms, iAvg, iPkPk, d.dutyCycle,
                                     vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing)});
        return m;
    };
    json L1 = inductor(d.inductanceL1, iout * d.dutyCycle / (1.0 - d.dutyCycle), dIL1);
    json L2 = inductor(d.inductanceL2, iout, dIL2);
    json cs; cs["capacitor"] = json::object();
    cs["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.couplingCapacitance;
    cs["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) / cfg::v_derate(d.config);
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

    json cell; cell["name"] = "sepic-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("L1", L1), comp("Q1", mq), comp("Cs", cs),
                                      comp("L2", L2), comp("D1", md)});
    cell["connections"] = json::array({
        conn("vin_net", {pin("L1", "primary_start"), prt("vin")}),
        // node A: L1 -> switch + coupling cap
        conn("nodeA",   {pin("L1", "primary_end"), pin("Q1", "drain"), pin("Cs", "1")}),
        // node B: coupling cap -> L2 + rectifier
        conn("nodeB",   {pin("Cs", "2"), pin("L2", "primary_end"), pin("D1", "anode")}),
        conn("gnd_net", {pin("Q1", "source"), pin("L2", "primary_start"), prt("gnd")}),
        conn("vout_net",{pin("D1", "cathode"), prt("vout")}),
        conn("gate_net",{pin("Q1", "gate"), prt("gate")})});

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
        pstage("sepicCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("sepicCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("sepicCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("sepicCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "sepicCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    return tas;
}

} // namespace Kirchhoff
