#include "Clllc.hpp"
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
    throw std::runtime_error("clllc design: no nominal");
}
constexpr double kQualityFactor   = 0.4;   // MKF Clllc default
constexpr double kInductanceRatio = 6.0;   // k = Lm/Lr1 (MKF Clllc inductanceRatioK)
constexpr double kSwitchDuty      = 0.47;  // primary bridge ~50% minus dead time
constexpr double kVdsHysteresis   = 1e-3;  // SR comparator hysteresis on the switch Vds [V]
} // namespace

ClllcDesign design_clllc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ClllcDesign d{};
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
    d.inputVoltageMin = vinMin; d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;
    double n = Vin / Vo;
    d.turnsRatio = n;
    const double fr = d.switchingFrequency;
    d.resonantFrequency = fr;
    const double Rload = Vo * Vo / d.outputPower;
    const double Ro = (8.0 * n * n / (M_PI * M_PI)) * Rload;
    const double wr = 2.0 * M_PI * fr;
    d.primaryResonantCapacitance = 1.0 / (2.0 * M_PI * kQualityFactor * fr * Ro);
    d.primaryResonantInductance = 1.0 / (wr * wr * d.primaryResonantCapacitance);
    d.magnetizingInductance = kInductanceRatio * d.primaryResonantInductance;
    d.secondaryResonantInductance = d.primaryResonantInductance / (n * n);
    d.secondaryResonantCapacitance = n * n * d.primaryResonantCapacitance;
    d.switchDuty = kSwitchDuty;
    d.loadResistance = Rload;
    d.outputCapacitance = 100e-6;
    return d;
}

json build_clllc_tas(const ClllcDesign& d) {
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
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L) { json j; j["magnetic"] = json::object();
        j["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = L;
        j["inputs"]["designRequirements"]["turnsRatios"] = json::array(); return j; };

    const double n = d.turnsRatio;
    json t1; t1["magnetic"] = json::object();
    t1["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    { json rn; rn["nominal"] = n; t1["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    // ───────────────────────── POWER stage ─────────────────────────
    // Exposes the two SR bridge midpoints (node_c, node_d) so the control stage can sense each SR
    // switch's drain-source for diode-emulating synchronous rectification.
    json pcell; pcell["name"] = "clllc-power";
    pcell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2"),
                                  port("gE"), port("gF"), port("gG"), port("gH"),
                                  port("nodeC"), port("nodeD")});
    pcell["components"] = json::array({
        comp("Q1", mosfet()), comp("Q2", mosfet()), comp("Q3", mosfet()), comp("Q4", mosfet()),
        comp("DS1", diode()), comp("DS2", diode()), comp("DS3", diode()), comp("DS4", diode()),
        comp("Cr1", capBrick(d.primaryResonantCapacitance, d.inputVoltage * 2)),
        comp("Lr1", indBrick(d.primaryResonantInductance)), comp("T1", t1),
        comp("Lr2", indBrick(d.secondaryResonantInductance)),
        comp("Cr2", capBrick(d.secondaryResonantCapacitance, d.outputVoltage * 2)),
        comp("QE", mosfet()), comp("QF", mosfet()), comp("QG", mosfet()), comp("QH", mosfet()),
        comp("DSE", diode()), comp("DSF", diode()), comp("DSG", diode()), comp("DSH", diode()),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2))});
    pcell["connections"] = json::array({
        // Primary full bridge. (Q1,Q4) on g1, (Q2,Q3) on g2 -> ±Vin.
        conn("vin_net",  {pin("Q1","drain"), pin("Q3","drain"), pin("DS1","cathode"), pin("DS3","cathode"), prt("vin")}),
        conn("node_a",   {pin("Q1","source"), pin("Q2","drain"), pin("DS1","anode"), pin("DS2","cathode"), pin("Cr1","1")}),
        conn("node_b",   {pin("Q3","source"), pin("Q4","drain"), pin("DS3","anode"), pin("DS4","cathode"), pin("T1","primary_end")}),
        conn("c1_mid",   {pin("Cr1","2"), pin("Lr1","primary_start")}),
        conn("pri_top",  {pin("Lr1","primary_end"), pin("T1","primary_start")}),
        // Secondary tank: sec_p -> Lr2 -> Cr2 -> node_c ; sec_n -> node_d.
        conn("sec_p",    {pin("T1","secondary1_start"), pin("Lr2","primary_start")}),
        conn("l2_mid",   {pin("Lr2","primary_end"), pin("Cr2","1")}),
        conn("node_c",   {pin("Cr2","2"), pin("QE","source"), pin("QF","drain"),
                          pin("DSE","anode"), pin("DSF","cathode"), prt("nodeC")}),
        conn("node_d",   {pin("T1","secondary1_end"), pin("QG","source"), pin("QH","drain"),
                          pin("DSG","anode"), pin("DSH","cathode"), prt("nodeD")}),
        // SR full bridge (diode-emulating SR via the control stage). Body diodes rectify / enable start.
        conn("vout_net", {pin("QE","drain"), pin("QG","drain"), pin("DSE","cathode"), pin("DSG","cathode"),
                          pin("Cout","1"), prt("vout")}),
        conn("gnd_net",  {pin("Q2","source"), pin("Q4","source"), pin("DS2","anode"), pin("DS4","anode"),
                          pin("QF","source"), pin("QH","source"), pin("DSF","anode"), pin("DSH","anode"),
                          pin("Cout","2"), prt("gnd")}),
        // Primary gates (open-loop stimulus); SR gates (driven by the control stage).
        conn("g1_net", {pin("Q1","gate"), pin("Q4","gate"), prt("g1")}),
        conn("g2_net", {pin("Q2","gate"), pin("Q3","gate"), prt("g2")}),
        conn("gE_net", {pin("QE","gate"), prt("gE")}),
        conn("gF_net", {pin("QF","gate"), prt("gF")}),
        conn("gG_net", {pin("QG","gate"), prt("gG")}),
        conn("gH_net", {pin("QH","gate"), prt("gH")})});

    // ──────────────────── CONTROL stage (swappable) ────────────────────
    // ONE CTAS `controller` component — a full-bridge synchronous-rectifier controller — placed as a
    // single part. Its ctas_to_cias lowering expands it to four diode-emulating comparators (each SR
    // switch gated while its body diode would conduct). Logical 8-pin interface: the two SR bridge
    // midpoints (nodeC/nodeD), the output rails (vSense/gSense), and the four SR gates. Swap this whole
    // stage for a different controller without touching the power topology.
    auto syncRect = [&](double hyst) { json j; j["controller"]["type"] = "synchronousRectifier";
        j["controller"]["topology"] = "fullBridge";
        j["controller"]["electrical"]["hysteresis"] = hyst; return j; };
    json ccell; ccell["name"] = "clllc-sr-control";
    ccell["ports"] = json::array({port("nodeC"), port("nodeD"), port("vSense"), port("gSense"),
                                  port("gE"), port("gF"), port("gG"), port("gH")});
    ccell["components"] = json::array({comp("SR", syncRect(kVdsHysteresis))});
    ccell["connections"] = json::array({
        conn("nodeC",  {pin("SR","nodeC"), prt("nodeC")}),
        conn("nodeD",  {pin("SR","nodeD"), prt("nodeD")}),
        conn("vSense", {pin("SR","vSense"), prt("vSense")}),
        conn("gSense", {pin("SR","gSense"), prt("gSense")}),
        conn("gE", {pin("SR","gE"), prt("gE")}), conn("gF", {pin("SR","gF"), prt("gF")}),
        conn("gG", {pin("SR","gG"), prt("gG")}), conn("gH", {pin("SR","gH"), prt("gH")})});

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
        pstage("clllcPower", "switchingCell", pcell, bind("vin", "dcBus"), bind("vout", "dcOutput")),
        pstage("srControl", "control", ccell, bind("nodeC", "sense"), bind("gE", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("clllcPower", "vin")}),
        // GND + Vout are shared with the control stage's sense inputs (vSense/gSense reference them).
        isc("GND", "externalPort", "input", {sp("clllcPower", "gnd"), sp("srControl", "gSense")}),
        isc("Vout", "externalPort", "output", {sp("clllcPower", "vout"), sp("srControl", "vSense")}),
        // SR bridge midpoints: power -> control (per-switch Vds sensing)
        isc("nodeC", "wire", "", {sp("clllcPower", "nodeC"), sp("srControl", "nodeC")}),
        isc("nodeD", "wire", "", {sp("clllcPower", "nodeD"), sp("srControl", "nodeD")}),
        // control drives -> each SR gate (one comparator per switch)
        isc("driveE", "wire", "", {sp("srControl", "gE"), sp("clllcPower", "gE")}),
        isc("driveF", "wire", "", {sp("srControl", "gF"), sp("clllcPower", "gF")}),
        isc("driveG", "wire", "", {sp("srControl", "gG"), sp("clllcPower", "gG")}),
        isc("driveH", "wire", "", {sp("srControl", "gH"), sp("clllcPower", "gH")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // ONLY the primary bridge is open-loop driven; the SR is closed-loop via the control stage.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "clllcPower"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    return tas;
}

} // namespace Kirchhoff
