#include "Clllc.hpp"
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
constexpr double kQualityFactor   = 0.4;   // MKF Clllc default
constexpr double kInductanceRatio = 6.0;   // k = Lm/Lr1 (MKF Clllc inductanceRatioK)
constexpr double kSwitchDuty      = 0.47;  // primary bridge ~50% minus dead time
constexpr double kSenseResistance = 0.01;  // in-line current-sense resistor in the secondary tank [Ω]
constexpr double kSenseHysteresis = 5e-3;  // SR comparator hysteresis on the i·Rsense signal [V]
} // namespace

ClllcDesign design_clllc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ClllcDesign d{};
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
    d.inputVoltageMin = vinMin; d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;
    double n = Vin / Vo;
    d.turnsRatio = n;
    const double fr = d.switchingFrequency;
    d.resonantFrequency = fr;
    const double Rload = Vo * Vo / d.outputPower;
    const double Ro = (8.0 * n * n / (M_PI * M_PI)) * Rload;
    const double wr = 2.0 * M_PI * fr;
    d.primaryResonantCapacitance = 1.0 / (2.0 * M_PI * cfg::get(d.config, "qualityFactor", kQualityFactor) * fr * Ro);
    d.primaryResonantInductance = 1.0 / (wr * wr * d.primaryResonantCapacitance);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        cfg::get(d.config, "inductanceRatio", kInductanceRatio) * d.primaryResonantInductance);
    d.secondaryResonantInductance = d.primaryResonantInductance / (n * n);
    d.secondaryResonantCapacitance = n * n * d.primaryResonantCapacitance;
    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
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
    auto resBrick = [&](double r) { json j; j["resistor"] = json::object();
        j["inputs"]["designRequirements"]["deviceType"] = "resistor";
        j["inputs"]["designRequirements"]["resistance"]["nominal"] = r; return j; };

    const double n = d.turnsRatio;
    json t1; t1["magnetic"] = json::object();
    t1["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    { json rn; rn["nominal"] = n; t1["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    // ───────────────────────── POWER stage ─────────────────────────
    // A small in-line sense resistor in the secondary tank exposes the tank-current sign (senseP/senseM)
    // so the control stage can drive the SR diagonals current-aware.
    json pcell; pcell["name"] = "clllc-power";
    pcell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2"),
                                  port("gE"), port("gF"), port("gG"), port("gH"),
                                  port("senseP"), port("senseM")});
    pcell["components"] = json::array({
        comp("Q1", mosfet()), comp("Q2", mosfet()), comp("Q3", mosfet()), comp("Q4", mosfet()),
        comp("DS1", diode()), comp("DS2", diode()), comp("DS3", diode()), comp("DS4", diode()),
        comp("Cr1", capBrick(d.primaryResonantCapacitance, d.inputVoltage * 2)),
        comp("Lr1", indBrick(d.primaryResonantInductance)), comp("T1", t1),
        comp("Lr2", indBrick(d.secondaryResonantInductance)),
        comp("Cr2", capBrick(d.secondaryResonantCapacitance, d.outputVoltage * 2)),
        comp("Rsense", resBrick(cfg::get(d.config, "senseResistance", kSenseResistance))),
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
        // Secondary tank: sec_p -> Lr2 -> Cr2 -> senseP -> Rsense -> node_c ; sec_n -> node_d.
        conn("sec_p",    {pin("T1","secondary1_start"), pin("Lr2","primary_start")}),
        conn("l2_mid",   {pin("Lr2","primary_end"), pin("Cr2","1")}),
        conn("senseP",   {pin("Cr2","2"), pin("Rsense","1"), prt("senseP")}),
        conn("node_c",   {pin("Rsense","2"), pin("QE","source"), pin("QF","drain"),
                          pin("DSE","anode"), pin("DSF","cathode"), prt("senseM")}),
        conn("node_d",   {pin("T1","secondary1_end"), pin("QG","source"), pin("QH","drain"),
                          pin("DSG","anode"), pin("DSH","cathode")}),
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
    // ONE CTAS `controller` component — a current-sensed full-bridge synchronous-rectifier controller —
    // placed as a single part. Its ctas_to_cias lowering expands it to two comparators that read the
    // tank-current sign (across the sense resistor) and gate the two rectifying diagonals. Logical 4-pin
    // interface: senseP/senseM (across Rsense) and gA/gB (the two diagonal gate signals). Swap this whole
    // stage for a different controller without touching the power topology.
    // The CTAS controller's agnostic ideal control law (CTAS controller.behavioral): the lib lowers it.
    auto syncRect = [&](double hyst) { json j; json& b = j["controller"]["behavioral"];
        b["controlScheme"] = "synchronousRectifier"; b["topology"] = "fullBridge"; b["sensing"] = "current";
        b["hysteresis"] = hyst; b["driveHigh"] = 5.0; b["driveLow"] = 0.0; b["threshold"] = 0.0; return j; };
    json ccell; ccell["name"] = "clllc-sr-control";
    ccell["ports"] = json::array({port("senseP"), port("senseM"), port("gA"), port("gB")});
    ccell["components"] = json::array({comp("SR", syncRect(cfg::get(d.config, "senseHysteresis", kSenseHysteresis)))});
    ccell["connections"] = json::array({
        conn("senseP", {pin("SR","senseP"), prt("senseP")}),
        conn("senseM", {pin("SR","senseM"), prt("senseM")}),
        conn("gA", {pin("SR","gA"), prt("gA")}), conn("gB", {pin("SR","gB"), prt("gB")})});

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
        pstage("srControl", "control", ccell, bind("senseP", "sense"), bind("gA", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("clllcPower", "vin")}),
        isc("GND", "externalPort", "input", {sp("clllcPower", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("clllcPower", "vout")}),
        // tank-current sense: power -> control
        isc("senseP", "wire", "", {sp("clllcPower", "senseP"), sp("srControl", "senseP")}),
        isc("senseM", "wire", "", {sp("clllcPower", "senseM"), sp("srControl", "senseM")}),
        // control drives -> the two SR diagonals (one comparator output fans out to two power gates):
        //   diagonal A (i>0): QE (gE) + QH (gH) ; diagonal B (i<0): QF (gF) + QG (gG)
        isc("driveA", "wire", "", {sp("srControl", "gA"), sp("clllcPower", "gE"), sp("clllcPower", "gH")}),
        isc("driveB", "wire", "", {sp("srControl", "gB"), sp("clllcPower", "gF"), sp("clllcPower", "gG")})});

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
