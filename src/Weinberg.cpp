#include "Weinberg.hpp"
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
    throw std::runtime_error("weinberg design: no nominal");
}
constexpr double kRippleRatio = 0.30;   // input-inductor (L1) current ripple (MKF Weinberg default)
constexpr double kDTarget     = 0.55;   // boost-regime duty target used to size n (MKF)
constexpr double kCoRipplePct = 0.01;   // output-cap voltage ripple fraction (MKF)
constexpr double kMaxDuty     = 0.95;

// MKF Weinberg::calculate_duty_cycle (boost branch — the design always lands D>0.5 here).
double duty_boost(double Vin, double Vo, double n, double eta) {
    double M = Vo / (Vin * eta);
    return 1.0 - 1.0 / (2.0 * n * M);
}
} // namespace

WeinbergDesign design_weinberg(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    WeinbergDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
    {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower  = op.at("outputs").at(0).at("power").get<double>();
    }
    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage, Fs = d.switchingFrequency, eta = d.efficiency;
    const double Iout = d.outputPower / Vo;

    // Turns ratio sized at MAX Vin to keep D ≥ 0.55 (boost regime) across the input range:
    //   n = 1 / (2·M·(1−D_target)),  M = Vo/(Vin_max·η).   (MKF process_design_requirements)
const double Vd = req::dideal_diode_drop(d.outputPower / Vo);  // DIDEAL Vf at the rectifier current
    const double Mmax = (Vo + Vd) / (vinMax * eta);
    double n = 1.0 / (2.0 * Mmax * (1.0 - cfg::get(d.config, "boostDutyTarget", kDTarget)));
    d.turnsRatio = std::round(n * 1000.0) / 1000.0;

    // Boost-regime duty at nominal Vin (the deck simulates at nominal Vin).
d.dutyCycle = duty_boost(Vin, Vo + Vd, d.turnsRatio, eta);
    d.switchDuty = d.dutyCycle;

    // L1 sized at MIN Vin (worst case): ΔI_L1 = ripple·I_L1perWinding, I_L1perWinding = Iin/2,
    // Iin = Iout/(η·M_boost). L1 = Vin_min·D_eff/(ΔI_L1·Fs), D_eff = max(2D−1, D).
    const double Dmin = duty_boost(vinMin, Vo, d.turnsRatio, eta);
    const double Mboost = 1.0 / (2.0 * d.turnsRatio * (1.0 - Dmin));
    const double Iin = Iout / (eta * Mboost);
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatio) * (Iin / 2.0);
    const double dEff = std::max(2.0 * Dmin - 1.0, Dmin);
    d.inputInductance = vinMin * dEff / (dIL1 * Fs);
    d.magnetizingInductance = d.inputInductance;   // Lpri_half ≈ L1 (MKF mirrors Lpri to the L1 magnitude)

    // Output cap from the 1% ripple target: Co = Iout·D/(ΔVo·Fs), ΔVo = coRipplePct·Vo.
    d.outputCapacitance = Iout * d.dutyCycle / (cfg::get(d.config, "outputCapRipple", kCoRipplePct) * Vo * Fs);
    d.loadResistance = Vo / Iout;
    return d;
}

json build_weinberg_tas(const WeinbergDesign& d) {
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
    auto res = [](double r) { json c; c["resistor"] = json::object();
        c["inputs"]["designRequirements"]["deviceType"] = "resistor";
        c["inputs"]["designRequirements"]["resistance"]["nominal"] = r; return c; };

    const double n = d.turnsRatio;

    // Input coupled inductor L1 (1:1, two windings), K=0.999. winding0=L1a (vin->l1a_mid),
    // secondary1=L1b (vin->l1b_mid). Both dots at vin (primary_start/secondary1_start).
    json l1; l1["magnetic"] = json::object();
    l1["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.inputInductance;
    { json r1; r1["nominal"] = 1.0; l1["inputs"]["designRequirements"]["turnsRatios"] = json::array({r1}); }
    l1["inputs"]["designRequirements"]["coupling"] = cfg::get(d.config, "transformerCoupling", 0.999);

    // Main transformer: CT primary (2 halves) + CT secondary (2 halves) = 4 coupled windings via
    // turnsRatios=[1, n, n]. Opposite-dot wound (encoded by the start/end node order below). K=0.9999.
    json t1; t1["magnetic"] = json::object();
    t1["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    { json r1; r1["nominal"] = 1.0; json rn; rn["nominal"] = n;
      t1["inputs"]["designRequirements"]["turnsRatios"] = json::array({r1, rn, rn}); }

    json cout; cout["capacitor"] = json::object();
    cout["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cout["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // SERIES-RC snubber (100Ω + 100pF) across each push-pull switch drain (which swings to 2·Vin/(1−D))
    // and each rectifier diode, to damp the leakage spike (no D3 recovery diodes). The series C BLOCKS the
    // DC blocking voltage, so the R (sized ≈√(L/C) for critical damping) dissipates only the switching
    // transition — NOT the continuous V_drain²/R that a drain-to-ground R∥C bleeds (which at the high
    // boost-reset voltage starved the output ~7%, dropping efficiency to ~57%). Intended divergence from
    // MKF's lossy R∥C: Kirchhoff delivers spec open-loop.
    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::rectifier_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage * 6 + d.outputVoltage); return c; };

    json cell; cell["name"] = "weinberg-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")});
    cell["components"] = json::array({
        comp("L1", l1), comp("T1", t1),
        // Tiny series R between each input-inductor winding and its transformer primary half: a NUMERICAL
        // loop-breaker for the otherwise-singular all-inductor mesh (coupled L1 + coupled T1 primary), not a
        // physical DCR. 1 mΩ is the minimum that converges; the old 0.05 Ω was an arbitrary lossy value that
        // burned ~2.5% of the output at the high-current corner (150 V/10 A: 4.5%→2.0% once shrunk to ideal).
        comp("Rdcra", res(cfg::loop_breaker_res(d.config, d.loadResistance))), comp("Rdcrb", res(cfg::loop_breaker_res(d.config, d.loadResistance))),
        comp("S1", mosfet()), comp("S2", mosfet()),
        comp("Dpos", diode()), comp("Dneg", diode()), comp("Cout", cout),
        comp("RsnS1", res(cfg::snubber_res(d.config))), comp("CsnS1", snubC()),
        comp("RsnS2", res(cfg::snubber_res(d.config))), comp("CsnS2", snubC()),
        comp("RsnDp", res(cfg::snubber_res(d.config))), comp("CsnDp", snubC()),
        comp("RsnDn", res(cfg::snubber_res(d.config))), comp("CsnDn", snubC())});
    cell["connections"] = json::array({
        // Input: both L1 windings fed from vin (current-fed front end).
        conn("vin_net",  {pin("L1", "primary_start"), pin("L1", "secondary1_start"), prt("vin")}),
        conn("l1a_mid",  {pin("L1", "primary_end"), pin("Rdcra", "1")}),
        conn("l1b_mid",  {pin("L1", "secondary1_end"), pin("Rdcrb", "1")}),
        // Primary center-tap halves: each input-inductor winding feeds one primary half-winding.
        conn("priCT_a",  {pin("Rdcra", "2"), pin("T1", "primary_end")}),
        conn("priCT_b",  {pin("Rdcrb", "2"), pin("T1", "secondary1_start")}),
        // Push-pull switch drains (dot ends of the primary halves) + series-RC snubber (drain -> C -> R -> gnd).
        conn("drainQ1",  {pin("T1", "primary_start"), pin("S1", "drain"), pin("CsnS1", "1")}),
        conn("drainQ2",  {pin("T1", "secondary1_end"), pin("S2", "drain"), pin("CsnS2", "1")}),
        conn("snS1",     {pin("CsnS1", "2"), pin("RsnS1", "1")}),
        conn("snS2",     {pin("CsnS2", "2"), pin("RsnS2", "1")}),
        // Secondary CT-FW rectifier: each secondary half -> its diode -> out_node. Secondary CT = gnd.
        // Series-RC snubber across each diode (anode -> C -> R -> cathode).
        conn("diodePos", {pin("T1", "secondary2_end"), pin("Dpos", "anode"), pin("CsnDp", "1")}),
        conn("diodeNeg", {pin("T1", "secondary3_start"), pin("Dneg", "anode"), pin("CsnDn", "1")}),
        conn("snDp",     {pin("CsnDp", "2"), pin("RsnDp", "1")}),
        conn("snDn",     {pin("CsnDn", "2"), pin("RsnDn", "1")}),
        conn("out_node", {pin("Dpos", "cathode"), pin("Dneg", "cathode"),
                          pin("RsnDp", "2"), pin("RsnDn", "2"), pin("Cout", "1"), prt("vout")}),
        // Ground: switch sources, secondary center-tap (both secondary-half gnd ends), snubber-R returns.
        conn("gnd_net",  {pin("S1", "source"), pin("S2", "source"),
                          pin("T1", "secondary2_start"), pin("T1", "secondary3_end"),
                          pin("RsnS1", "2"), pin("RsnS2", "2"),
                          pin("Cout", "2"), prt("gnd")}),
        conn("g1_net", {pin("S1", "gate"), prt("g1")}),
        conn("g2_net", {pin("S2", "gate"), prt("g2")})});

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
        pstage("weinbergCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("weinbergCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("weinbergCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("weinbergCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Push-pull drive: Q1 phase 0, Q2 phase 180; each ON for D·Tsw within its half-period (the pulses
    // overlap by (2D−1)·Tsw when D>0.5 — the boost regime).
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "weinbergCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("S1", 0.0), stim("S2", 180.0)});
    return tas;
}

} // namespace Kirchhoff
