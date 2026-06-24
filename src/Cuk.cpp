#include "Cuk.hpp"
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
    throw std::runtime_error("cuk design: no nominal");
}
constexpr double kRippleRatioL1 = 0.4;   // ΔIL1 / IL1,avg
constexpr double kL2RipplePct = 0.30;    // ΔIL2 / IL2,avg
constexpr double kC1RipplePct = 0.05;    // ΔVC1 / VC1
constexpr double kCoRipplePct = 0.01;    // ΔVo  / |Vo|
// Cuk CCM ideal-ish duty (n=1): D = (|Vo|+Vd) / (Vin*eff + |Vo| + Vd).
double duty(double vin, double voMag, double vd, double eff) { return (voMag + vd) / (vin * eff + voMag + vd); }
} // namespace

CukDesign design_cuk(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    CukDesign d{};
    // Output voltage is stored as a magnitude (MKF treats Cuk Vout as |Vo|); take abs to be robust to
    // a negative setpoint in the TAS.
    d.outputVoltageMag = std::fabs(nominal(dr.at("outputs").at(0).at("voltage")));
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

    const double iout = d.outputPower / d.outputVoltageMag, fsw = d.switchingFrequency;
    d.diodeDrop = req::dideal_diode_drop(iout);  // DIDEAL Vf at the operating rectifier current
    d.dutyCycle = duty(d.inputVoltage, d.outputVoltageMag, d.diodeDrop, d.efficiency);

    // L1 sized at the worst corner (max Vin) for its current-ripple target (MKF).
    const double dMax = duty(vinMax, d.outputVoltageMag, d.diodeDrop, d.efficiency);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double dIL1 = kRippleRatioL1 * iL1avg;
    d.inductanceL1 = vinMax * dMax / (dIL1 * fsw);
    // L2, C1, Cout at the operating point.
    const double dIL2 = kL2RipplePct * iout;
    d.inductanceL2 = d.outputVoltageMag * (1.0 - d.dutyCycle) / (dIL2 * fsw);
    const double VC1 = d.inputVoltage / (1.0 - d.dutyCycle);   // = Vin + |Vo|
    const double dVC1 = kC1RipplePct * VC1;
    d.couplingCapacitance = iout * d.dutyCycle / (dVC1 * fsw);
    const double dVo = kCoRipplePct * d.outputVoltageMag;
    d.outputCapacitance = dIL2 / (8.0 * fsw * dVo);
    d.loadResistance = d.outputVoltageMag * d.outputVoltageMag / d.outputPower;
    return d;
}

json build_cuk_tas(const CukDesign& d) {
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

    const double fsw = d.switchingFrequency, iout = d.outputPower / d.outputVoltageMag;
    const double dIL1 = kRippleRatioL1 * (iout * d.dutyCycle / (1.0 - d.dutyCycle));
    const double dIL2 = kL2RipplePct * iout;
    const double vSwing = d.inputVoltage + d.outputVoltageMag;   // C1 holds ~Vin+|Vo|; switch/diode block it

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
    json c1; c1["capacitor"] = json::object();
    c1["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.couplingCapacitance;
    c1["inputs"]["designRequirements"]["ratedVoltage"] = vSwing / req::V_DERATE;
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltageMag / req::V_DERATE;
    json mq = mosfet();
    mq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", vSwing / req::V_DERATE,
                                                     iout + iout * d.dutyCycle / (1.0 - d.dutyCycle),
                                                     0.01 * d.outputPower, 125.0);
    json md = diode();
    md["inputs"]["designRequirements"] = req::diode(vSwing / req::V_DERATE, iout / 0.7,
                                                    (vSwing < 100.0) ? 0.6 : 1.2, 0.05 / fsw);

    // RC snubber across the freewheel diode — a REAL component (the DAB cell snubs its switches the same
    // way). It damps the ideal-diode commutation dV/dt so the Cuk's resonant coupling loop converges in
    // ngspice (otherwise "timestep too small" at startup). 100 Ω · 1 nF is negligible at the power-stage
    // scale (RC = 100 ns « the switching period; bleed « the amperes of inductor current), so it does not
    // shift the operating point — it just makes the emitted deck simulable for any consumer.
    json rsnub; rsnub["resistor"] = json::object();
    rsnub["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rsnub["inputs"]["designRequirements"]["resistance"]["nominal"] = 100.0;
    json csnub; csnub["capacitor"] = json::object();
    csnub["inputs"]["designRequirements"]["capacitance"]["nominal"] = 1e-9;
    csnub["inputs"]["designRequirements"]["ratedVoltage"] = vSwing / req::V_DERATE;

    // Cuk cell — inverting. Dot/orientation mirror MKF: D1 anode at nodeB, cathode at gnd; L2 nodeB->vout(neg).
    json cell; cell["name"] = "cuk-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("L1", L1), comp("Q1", mq), comp("C1", c1),
                                      comp("D1", md), comp("L2", L2),
                                      comp("Rsnub", rsnub), comp("Csnub", csnub)});
    cell["connections"] = json::array({
        conn("vin_net", {pin("L1", "primary_start"), prt("vin")}),
        conn("nodeA",   {pin("L1", "primary_end"), pin("Q1", "drain"), pin("C1", "1")}),
        // node B: coupling cap -> freewheel diode (anode) + output inductor + diode RC snubber
        conn("nodeB",   {pin("C1", "2"), pin("D1", "anode"), pin("L2", "primary_start"), pin("Rsnub", "1")}),
        conn("snub",    {pin("Rsnub", "2"), pin("Csnub", "1")}),
        conn("gnd_net", {pin("Q1", "source"), pin("D1", "cathode"), pin("Csnub", "2"), prt("gnd")}),
        conn("vout_net",{pin("L2", "primary_end"), prt("vout")}),     // negative output
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
    // Output voltage is NEGATIVE (inverting): the synthesized load measures v(Vout) < 0.
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = -d.outputVoltageMag; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        pstage("cukCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("cukCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("cukCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("cukCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "cukCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    return tas;
}

} // namespace Kirchhoff
