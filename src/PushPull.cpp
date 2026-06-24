#include "PushPull.hpp"
#include "KirchhoffConfig.hpp"
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
    throw std::runtime_error("push-pull design: no nominal");
}
constexpr double kMaxDuty = 0.48;     // MKF PushPull default (D < 0.5 strictly)
constexpr double kRippleRatio = 0.4;  // output-inductor current ripple
} // namespace

PushPullDesign design_push_pull(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PushPullDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    d.maxDutyCycle = cfg::get(d.config, "maxDutyCycle", kMaxDuty);
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

    const double iout = d.outputPower / d.outputVoltage, fsw = d.switchingFrequency, T = 1.0 / fsw;
    // Turns ratio (MKF): N = D_max * 2 * Vin_min / (Vout + Vd). Rounded to 2 dp.
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double N = d.maxDutyCycle * 2.0 * vinMin / (d.outputVoltage + d.diodeDrop);
    N = std::round(N * 100.0) / 100.0;
    d.turnsRatio = N;
    // Magnetizing inductance per half (MKF): Lm = Vin_min * tOn / Iprimary, tOn = D_max*T,
    // Iprimary = Pout / Vin_min / eff.
    const double tOn = d.maxDutyCycle * T;
    const double iPrimary = d.outputPower / vinMin / d.efficiency;
    d.magnetizingInductance = vinMin * tOn / iPrimary;
    // Output inductor (MKF, worst case = max Vin): tOn_sec = (T/2)*(Vout+Vd)*N/Vin; ΔI = ripple*Iout;
    // Lout = (Vin/N - Vout) * tOn_sec / ΔI.
    const double tOnSec = (T / 2.0) * (d.outputVoltage + d.diodeDrop) * N / vinMax;
    const double dILout = cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * iout;
    d.outputInductance = (vinMax / N - d.outputVoltage) * tOnSec / dILout;
    // Operating per-switch duty at nominal Vin: Vout = 2*D*Vin/N  ->  D = N*Vout/(2*Vin).
    d.dutyCycle = N * (d.outputVoltage + d.diodeDrop) / (2.0 * d.inputVoltage);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

json build_push_pull_tas(const PushPullDesign& d) {
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

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;

    // 4-winding transformer: turnsRatios = [1 (2nd primary half), N (sec top), N (sec bot)].
    // Dot/terminal order mirrors MKF: Lpri_top pri_top->center_tap; Lpri_bot center_tap->pri_bot;
    // Lsec_top sec_top->gnd; Lsec_bot gnd->sec_bot.
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json r1; r1["nominal"] = 1.0; json rn; rn["nominal"] = N;
      xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({r1, rn, rn}); }
    json lout; lout["magnetic"] = json::object();
    lout["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.outputInductance;
    lout["inputs"]["designRequirements"]["turnsRatios"] = json::array();
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;
    // Snubber caps at the switching nodes. A push-pull's center-tapped transformer leaves the primary
    // magnetizing current without a path during the dead time between the two 180-deg phases; with an
    // IDEAL transformer (no parasitic C) the node voltage runs away and ngspice fails (timestep too
    // small). A small node-to-gnd snubber cap gives that current a finite-dV/dt path — physically real
    // in any push-pull, and small enough (2.2 nF) to leave Vout within ~2% (energy is reactive, ~no
    // loss). MKF's reference deck uses the same technique (switch + rectifier snubbers).
    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    json cell; cell["name"] = "push-pull-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate1"), port("gate2")});
    cell["components"] = json::array({comp("T1", xfmr), comp("Q1", mosfet()), comp("Q2", mosfet()),
                                      comp("Dtop", diode()), comp("Dbot", diode()), comp("Lout", lout),
                                      comp("Csn1", snub()), comp("Csn2", snub()),
                                      comp("Csn3", snub()), comp("Csn4", snub())});
    cell["connections"] = json::array({
        // center tap (= Vin) shared by both primary halves
        conn("vin_net",   {pin("T1", "primary_end"), pin("T1", "secondary1_start"), prt("vin")}),
        // primary half tops -> low-side switches (+ snubber cap to gnd)
        conn("pri_top",   {pin("T1", "primary_start"), pin("Q1", "drain"), pin("Csn1", "1")}),
        conn("pri_bot",   {pin("T1", "secondary1_end"), pin("Q2", "drain"), pin("Csn2", "1")}),
        // secondary halves -> full-wave rectifier diodes (center tap = gnd) (+ snubber cap to gnd)
        conn("sec_top",   {pin("T1", "secondary2_start"), pin("Dtop", "anode"), pin("Csn3", "1")}),
        conn("sec_bot",   {pin("T1", "secondary3_end"), pin("Dbot", "anode"), pin("Csn4", "1")}),
        conn("sec_rect",  {pin("Dtop", "cathode"), pin("Dbot", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net",  {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",   {pin("Q1", "source"), pin("Q2", "source"),
                           pin("T1", "secondary2_end"), pin("T1", "secondary3_start"),
                           pin("Csn1", "2"), pin("Csn2", "2"), pin("Csn3", "2"), pin("Csn4", "2"), prt("gnd")}),
        conn("gate1_net", {pin("Q1", "gate"), prt("gate1")}),
        conn("gate2_net", {pin("Q2", "gate"), prt("gate2")})});

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
        pstage("pushPullCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("pushPullCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("pushPullCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("pushPullCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Two PWM drives 180 deg apart (the interleaved push-pull switching). The stimulus targets each
    // switch's "gate" pin (exposed on ports gate1/gate2); Q2 is phase-shifted half a period.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "pushPullCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    return tas;
}

} // namespace Kirchhoff
