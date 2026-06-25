#include "Src.hpp"
#include "Dimension.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kBridgeFactor  = 0.5;   // half-bridge: Vo_fha = 0.5·Vin
constexpr double kQualityFactor = 2.0;   // MKF Src default
constexpr double kLmRatio       = 10.0;  // Lm = 10·Lr (large, keeps Lm out of the resonance)
constexpr double kSwitchDuty    = 0.45;
} // namespace

SrcDesign design_src(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    SrcDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
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

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;
    const double Iout = d.outputPower / Vo;

    double n = (cfg::get(d.config, "bridgeFactor", kBridgeFactor) * Vin) / Vo;
    d.turnsRatio = std::round(n * 100.0) / 100.0;

    // Two-element series tank (no resonant Lm): Rac = 8n²/π²·Rload, Zr = Q·Rac, operate at fr = fsw,
    // Lr = Zr/(2π·fr), Cr = 1/(2π·fr·Zr). Lm made large (10·Lr) so it does not load the resonance.
    const double Rload = Vo / Iout;
    const double Rac = (8.0 * n * n) / (M_PI * M_PI) * Rload;
    const double fr = d.switchingFrequency;   // designed/operated at series resonance
    const double Zr = cfg::get(d.config, "qualityFactor", kQualityFactor) * Rac;
    d.resonantFrequency = fr;
    d.resonantInductance = Zr / (2.0 * M_PI * fr);
    d.resonantCapacitance = 1.0 / (2.0 * M_PI * fr * Zr);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        cfg::get(d.config, "inductanceRatio", kLmRatio) * d.resonantInductance);

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 47e-6;
    return d;
}

json build_src_tas(const SrcDesign& d) {
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

    const double n = d.turnsRatio;

    json cr; cr["capacitor"] = json::object();
    cr["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.resonantCapacitance;
    cr["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2;

    json lr; lr["magnetic"] = json::object();
    lr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.resonantInductance;
    lr["inputs"]["designRequirements"]["turnsRatios"] = json::array();

    json t1; t1["magnetic"] = json::object();
    t1["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    { json rn; rn["nominal"] = n; t1["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn, rn}); }
    t1["inputs"]["designRequirements"]["coupling"] = cfg::get(d.config, "transformerCoupling", 0.999);

    auto busCap = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::get(d.config, "busSplitCap", 10e-6);
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2; return c; };

    json cout; cout["capacitor"] = json::object();
    cout["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cout["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::rectifier_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 3; return c; };
    auto snubR = [&]() { json c; c["resistor"] = json::object();
        c["inputs"]["designRequirements"]["deviceType"] = "resistor";
        c["inputs"]["designRequirements"]["resistance"]["nominal"] = cfg::snubber_res(d.config); return c; };

    json cell; cell["name"] = "src-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")});
    cell["components"] = json::array({
        comp("Q1", mosfet()), comp("Q2", mosfet()), comp("Dq1", diode()), comp("Dq2", diode()),
        comp("Chi", busCap()), comp("Clo", busCap()),
        comp("Cr", cr), comp("Lr", lr), comp("T1", t1),
        comp("D1", diode()), comp("D2", diode()), comp("Cout", cout),
        comp("Rsn1", snubR()), comp("Csn1", snubC()), comp("Rsn2", snubR()), comp("Csn2", snubC())});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), pin("Dq1", "cathode"), pin("Chi", "1"), prt("vin")}),
        conn("sw_node",  {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("Dq1", "anode"), pin("Dq2", "cathode"), pin("Cr", "1")}),
        conn("msplit",   {pin("Chi", "2"), pin("Clo", "1"), pin("T1", "primary_end")}),
        conn("cr_mid",   {pin("Cr", "2"), pin("Lr", "primary_start")}),
        conn("pri_top",  {pin("Lr", "primary_end"), pin("T1", "primary_start")}),
        conn("sec_top",  {pin("T1", "secondary1_start"), pin("D1", "anode"),
                          pin("Rsn1", "1"), pin("Csn1", "1")}),
        conn("sec_bot",  {pin("T1", "secondary2_end"), pin("D2", "anode"),
                          pin("Rsn2", "1"), pin("Csn2", "1")}),
        conn("vout_net", {pin("D1", "cathode"), pin("D2", "cathode"),
                          pin("Rsn1", "2"), pin("Csn1", "2"), pin("Rsn2", "2"), pin("Csn2", "2"),
                          pin("Cout", "1"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("Dq2", "anode"), pin("Clo", "2"),
                          pin("T1", "secondary1_end"), pin("T1", "secondary2_start"),
                          pin("Cout", "2"), prt("gnd")}),
        conn("g1_net", {pin("Q1", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), prt("g2")})});

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
        pstage("srcCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("srcCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("srcCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("srcCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "srcCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    return tas;
}

} // namespace Kirchhoff
