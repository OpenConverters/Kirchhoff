#include "Fsbb.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    if (j.contains("minimum") && j.contains("maximum"))
        return 0.5 * (j.at("minimum").get<double>() + j.at("maximum").get<double>());
    throw std::runtime_error("fsbb design: no nominal");
}
constexpr double kRippleRatio = 0.4;   // inductor current ripple (MKF 4SBB K)
constexpr double kDeadFrac    = 0.01;  // 100ns per-leg dead time at 100kHz
} // namespace

FsbbDesign design_fsbb(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    FsbbDesign d{};
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

    const double Vin = d.inputVoltage, Vo = d.outputVoltage, Fs = d.switchingFrequency;
    const double Io = d.outputPower / Vo;
    d.deadFraction = kDeadFrac;
    // Buck-boost simultaneous mode gain M = D/(1-D) = Vo/Vin  =>  D = Vo/(Vin+Vo).
    d.dutyCycle = Vo / (Vin + Vo);

    // Worst-case inductor: max(L_buck @ Vin_max, L_boost @ Vin_min). Each region's formula is only
    // valid on its side of Vo (returns 0 otherwise). (MKF compute_worst_case_inductance.)
    const double Lbuck  = (vinMax > Vo) ? Vo * (vinMax - Vo) / (kRippleRatio * Io * Fs * vinMax) : 0.0;
    const double Lboost = (vinMin < Vo) ? (vinMin * vinMin) * (Vo - vinMin)
                                          / (kRippleRatio * Io * Fs * Vo * Vo) : 0.0;
    d.inductance = std::max(Lbuck, Lboost);

    d.loadResistance = Vo * Vo / d.outputPower;
    d.config = cfg::object_of(tasInputs);
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

json build_fsbb_tas(const FsbbDesign& d) {
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

    // Single inductor L (single-winding magnetic).
    json lind; lind["magnetic"] = json::object();
    lind["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.inductance;
    lind["inputs"]["designRequirements"]["turnsRatios"] = json::array();

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    // H-bridge buck-boost cell. Buck leg Q1(vin->sw1)/Q2(sw1->gnd); inductor sw1->sw2; boost leg
    // Q3(vout->sw2)/Q4(sw2->gnd). Body diodes anti-parallel to each switch (Q3 oriented drain=vout so
    // its body diode sw2->vout is the boost freewheel path). Snubber caps at sw1/sw2 for dead-time dV/dt.
    json cell; cell["name"] = "fsbb-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("g1"), port("g2"), port("g3"), port("g4")});
    cell["components"] = json::array({
        comp("Q1", mosfet()), comp("Q2", mosfet()), comp("Q3", mosfet()), comp("Q4", mosfet()),
        comp("D1", diode()),  comp("D2", diode()),  comp("D3", diode()),  comp("D4", diode()),
        comp("L", lind), comp("Csw1", snub()), comp("Csw2", snub())});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), pin("Q3", "drain"), pin("D1", "cathode"), prt("vin")}),
        conn("sw1_net",  {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("D1", "anode"), pin("D2", "cathode"),
                          pin("L", "primary_start"), pin("Csw1", "1")}),
        conn("sw2_net",  {pin("Q4", "drain"), pin("Q3", "source"),
                          pin("D4", "cathode"), pin("D3", "anode"),
                          pin("L", "primary_end"), pin("Csw2", "1")}),
        // Q3 high-side to the output: drain=vout (body diode D3 sw2->vout = boost freewheel).
        conn("vout_net", {pin("Q3", "drain"), pin("D3", "cathode"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("Q4", "source"),
                          pin("D2", "anode"), pin("D4", "anode"),
                          pin("Csw1", "2"), pin("Csw2", "2"), prt("gnd")}),
        conn("g1_net", {pin("Q1", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), prt("g2")}),
        conn("g3_net", {pin("Q3", "gate"), prt("g3")}),
        conn("g4_net", {pin("Q4", "gate"), prt("g4")})});

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
        pstage("fsbbCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("fsbbCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("fsbbCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("fsbbCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Q1+Q4 ON during D (charge); Q2+Q3 ON during (1-D) (discharge). Each leg has a dead band: the
    // low-going switch (Q2 in leg1, Q3 in leg2) starts a dead-band after its partner turns off and is
    // trimmed not to wrap past the period; body diodes carry the inductor current across the dead time.
    const double D = d.dutyCycle, dt = d.deadFraction;
    auto stim = [&](const char* sw, const char* g, double duty, double phaseDeg) {
        json st; st["stage"] = "fsbbCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phaseDeg"] = phaseDeg;
        (void)g; return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("Q1", "g1", D, 0.0),
        stim("Q4", "g4", D, 0.0),
        stim("Q2", "g2", (1.0 - D) - 2.0 * dt, (D + dt) * 360.0),
        stim("Q3", "g3", (1.0 - D) - 2.0 * dt, (D + dt) * 360.0)});
    return tas;
}

} // namespace Kirchhoff
