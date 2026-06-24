#include "Cllc.hpp"
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
    throw std::runtime_error("cllc design: no nominal");
}
constexpr double kQualityFactor   = 0.3;   // MKF Cllc default (Infineon AN: 0.2–0.4)
constexpr double kInductanceRatio = 4.45;  // k = Lm/Lr1 (MKF defaultInductanceRatio)
constexpr double kSwitchDuty      = 0.47;  // ~50% minus dead time
} // namespace

CllcDesign design_cllc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    CllcDesign d{};
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
    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;

    // n = Vin_nom/Vout (full bridge both sides). fr = fsw (operated at resonance).
    double n = Vin / Vo;
    d.turnsRatio = n;
    const double fr = d.switchingFrequency;
    d.resonantFrequency = fr;

    // Infineon FHA: Ro = 8n²/π²·Rload, Cr1 = 1/(2π·Q·fr·Ro), Lr1 = 1/((2π·fr)²·Cr1), Lm = k·Lr1.
    // Symmetric tank (a=b=1): Lr2 = Lr1/n², Cr2 = n²·Cr1.  (MKF Cllc::calculate_resonant_parameters)
    const double Rload = Vo * Vo / d.outputPower;
    const double Ro = (8.0 * n * n / (M_PI * M_PI)) * Rload;
    const double wr = 2.0 * M_PI * fr;
    d.primaryResonantCapacitance = 1.0 / (2.0 * M_PI * cfg::get(d.config, "qualityFactor", kQualityFactor) * fr * Ro);
    d.primaryResonantInductance = 1.0 / (wr * wr * d.primaryResonantCapacitance);
    d.magnetizingInductance = cfg::get(d.config, "inductanceRatio", kInductanceRatio) * d.primaryResonantInductance;
    d.secondaryResonantInductance = d.primaryResonantInductance / (n * n);
    d.secondaryResonantCapacitance = n * n * d.primaryResonantCapacitance;

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 10e-6;    // matches MKF CLLC (Cout=10u)
    return d;
}

json build_cllc_tas(const CllcDesign& d) {
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

    auto capBrick = [&](double c, double vrated) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vrated; return j; };
    auto indBrick = [&](double L) { json j; j["magnetic"] = json::object();
        j["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = L;
        j["inputs"]["designRequirements"]["turnsRatios"] = json::array(); return j; };

    json cr1 = capBrick(d.primaryResonantCapacitance, d.inputVoltage * 2);
    json lr1 = indBrick(d.primaryResonantInductance);
    json lr2 = indBrick(d.secondaryResonantInductance);
    json cr2 = capBrick(d.secondaryResonantCapacitance, d.outputVoltage * 2);
    json cout = capBrick(d.outputCapacitance, d.outputVoltage * 2);

    // Transformer: primary Lpri = Lm, single secondary, turnsRatios=[n], K=0.9999.
    json t1; t1["magnetic"] = json::object();
    t1["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    { json rn; rn["nominal"] = n; t1["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    json cell; cell["name"] = "cllc-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("g1"), port("g2")});
    cell["components"] = json::array({
        // primary full bridge + body diodes
        comp("Q1", mosfet()), comp("Q2", mosfet()), comp("Q3", mosfet()), comp("Q4", mosfet()),
        comp("DS1", diode()), comp("DS2", diode()), comp("DS3", diode()), comp("DS4", diode()),
        // primary tank + transformer + secondary tank
        comp("Cr1", cr1), comp("Lr1", lr1), comp("T1", t1), comp("Lr2", lr2), comp("Cr2", cr2),
        // secondary active synchronous rectifier (4 switches) + body diodes (full-bridge rectifying)
        comp("Qa", mosfet()), comp("Qb", mosfet()), comp("Qc", mosfet()), comp("Qd", mosfet()),
        comp("DSa", diode()), comp("DSb", diode()), comp("DSc", diode()), comp("DSd", diode()),
        comp("Cout", cout)});
    cell["connections"] = json::array({
        // ── Primary full bridge. Diagonal pairs (Q1,Q4) on g1, (Q2,Q3) on g2 -> vab=±Vin.
        conn("vin_net",  {pin("Q1", "drain"), pin("Q3", "drain"),
                          pin("DS1", "cathode"), pin("DS3", "cathode"), prt("vin")}),
        conn("node_a",   {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("DS1", "anode"), pin("DS2", "cathode"), pin("Cr1", "1")}),
        conn("node_b",   {pin("Q3", "source"), pin("Q4", "drain"),
                          pin("DS3", "anode"), pin("DS4", "cathode"), pin("T1", "primary_end")}),
        // Primary series tank: node_a -> Cr1 -> Lr1 -> Lpri(=Lm) -> node_b.
        conn("c1_mid",   {pin("Cr1", "2"), pin("Lr1", "primary_start")}),
        conn("pri_top",  {pin("Lr1", "primary_end"), pin("T1", "primary_start")}),
        // Secondary series tank: sec_p -> Lr2 -> Cr2 -> node_c ; sec_n -> node_d.
        conn("sec_p",    {pin("T1", "secondary1_start"), pin("Lr2", "primary_start")}),
        conn("l2_mid",   {pin("Lr2", "primary_end"), pin("Cr2", "1")}),
        conn("node_c",   {pin("Cr2", "2"), pin("Qa", "source"), pin("Qb", "drain"),
                          pin("DSa", "anode"), pin("DSb", "cathode")}),
        conn("node_d",   {pin("T1", "secondary1_end"), pin("Qc", "source"), pin("Qd", "drain"),
                          pin("DSc", "anode"), pin("DSd", "cathode")}),
        // ── Secondary active bridge. Diagonal pairs (Qa,Qd) on g1, (Qb,Qc) on g2 — synchronous with
        // the primary (forward power flow). Body diodes DSa..DSd form a full-bridge rectifier that lets
        // the converter start once the output is precharged (simulation.initialConditions).
        conn("vout_net", {pin("Qa", "drain"), pin("Qc", "drain"),
                          pin("DSa", "cathode"), pin("DSc", "cathode"), pin("Cout", "1"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("Q4", "source"),
                          pin("DS2", "anode"), pin("DS4", "anode"),
                          pin("Qb", "source"), pin("Qd", "source"),
                          pin("DSb", "anode"), pin("DSd", "anode"), pin("Cout", "2"), prt("gnd")}),
        conn("g1_net", {pin("Q1", "gate"), pin("Q4", "gate"), pin("Qa", "gate"), pin("Qd", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), pin("Q3", "gate"), pin("Qb", "gate"), pin("Qc", "gate"), prt("g2")})});

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
        pstage("cllcCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("cllcCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("cllcCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("cllcCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Both bridges are square waves: g1 (Q1,Q4,Qa,Qd) phase 0, g2 (Q2,Q3,Qb,Qc) phase 180. The
    // secondary is gated synchronously with the primary (forward power flow).
    auto stim = [&](const char* sw, const char* sig, double phaseDeg) {
        json st; st["stage"] = "cllcCell"; st["component"] = sw; st["signal"] = sig;
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    // Only ONE stimulus per shared gate node: Q1 drives the g1 port (shared by Q1/Q4/Qa/Qd), Q2 drives
    // g2 (shared by Q2/Q3/Qb/Qc). Emitting one per switch would put 4 voltage sources on one node
    // (singular). The four switches on each gate are driven together — exactly the 2-signal CLLC drive.
    tas["simulation"]["stimulus"] = json::array({
        stim("Q1", "gate", 0.0), stim("Q2", "gate", 180.0)});
    // Precharge the output to its target so the active synchronous rectifier can start and the deck runs
    // with use-initial-conditions (skipping the resonant tank's singular DC operating point).
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    return tas;
}

} // namespace Kirchhoff
