#include "Acf.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kDuty       = 0.45;   // main-switch operating duty (MKF ACF maximumDutyCycle)
constexpr double kDeadFrac   = 0.01;   // 100ns dead time between main & clamp switches at 100kHz
constexpr double kRippleRatio = 0.4;   // output-inductor current ripple
} // namespace

AcfDesign design_acf(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    AcfDesign d{};
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

    const double Vo = d.outputVoltage, Fs = d.switchingFrequency, Io = d.outputPower / Vo;
    d.diodeDrop = req::dideal_diode_drop(Io);  // forward+freewheel ~ one DIDEAL drop at the rectifier current
    const double D = cfg::get(d.config, "operatingDutyCycle", kDuty);
    d.dutyCycle = D;
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", kDeadFrac);

    // Turns ratio n = Vin_min*D/(Vo+Vd) so the forward gain reaches Vo at min input (MKF). Vd=0.
    double n = d.inputVoltage * D / (Vo + d.diodeDrop);  // operating Vin (open-loop hits spec at the op point, not +5% at vinMin)
    d.turnsRatio = std::round(n * 100.0) / 100.0;
    n = d.turnsRatio;

    // Magnetizing inductance: Lm = Vin_min * n / (Fs * Io)  (reflected secondary current Io/n).
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * n / (Fs * Io));
    // Output inductor: Lo = (Vin_max/n - Vd - Vo) * tOn / ripple,  tOn = D/Fs.
    const double tOn = D / Fs;
    d.outputInductance = (vinMax / n - d.diodeDrop - Vo) * tOn / cfg::get(d.config, "inductorRippleRatio", kRippleRatio);

    d.clampCapacitance = 10e-6;   // active-clamp capacitor (matches MKF; value sets reset ring, not Vo)
    d.loadResistance = Vo * Vo / d.outputPower;
    d.outputCapacitance = 100e-6; // matches MKF ACF (Cout=100u)
    return d;
}

json build_acf_tas(const AcfDesign& d) {
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

    const double n = d.turnsRatio, Lm = d.magnetizingInductance, fsw = d.switchingFrequency;

    // 2-winding transformer (primary + 1 secondary, NO demag winding — active clamp resets it).
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json rn; rn["nominal"] = n; xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    json lout; lout["magnetic"] = json::object();
    lout["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.outputInductance;
    lout["inputs"]["designRequirements"]["turnsRatios"] = json::array();

    json cc; cc["capacitor"] = json::object();   // active-clamp capacitor
    cc["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.clampCapacitance;
    cc["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // Numerical node snubber on the switching node. The ACF is the one switching topology that shipped WITHOUT
    // one, so at high Vin / light load the active-clamp node (whose only DC path is the clamp switch ROFF)
    // produces an unresolvably fast dV/dt -> ngspice "timestep too small". A small node-to-gnd cap tames it,
    // exactly like the bridge midpoints. Named "Csn" so the real-fidelity strip replaces it with the switch's
    // Coss when a real part is bound (and the real deck's cshunt covers convergence).
    json snb; snb["capacitor"] = json::object();
    snb["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
    snb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;

    // Active-clamp forward cell: main switch Q1 (vin->sw), 2-winding transformer (sw->gnd primary), the
    // clamp leg (Sc: vin->clamp_node, Cc: clamp_node->sw — clamp_node gets its DC path through Sc's
    // ROFF, like MKF's 1Meg bleeder), and the forward output (Dfwd + Dfw + Lout). Dot orientation
    // matches MKF (primary & secondary in-phase: primary_start=sw, secondary1_start=sec_in).
    json cell; cell["name"] = "acf-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate_main"), port("gate_clamp")});
    cell["components"] = json::array({comp("Q1", mosfet()), comp("Sc", mosfet()), comp("Cc", cc),
                                      comp("T1", xfmr), comp("Dfwd", diode()), comp("Dfw", diode()),
                                      comp("Lout", lout), comp("Csn", snb), comp("Csn2", snb)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), pin("Sc", "drain"), prt("vin")}),
        conn("sw_node",  {pin("Q1", "source"), pin("T1", "primary_start"), pin("Cc", "2"), pin("Csn", "1")}),
        // clamp_node is the stiff one (its only DC path is the clamp switch ROFF); a node snubber there clears
        // the last high-Vin/light-load/high-fsw corner. Both Csn* strip together for a real-fidelity deck.
        conn("clamp_node", {pin("Sc", "source"), pin("Cc", "1"), pin("Csn2", "1")}),
        // secondary -> forward rectifier (forward diode + freewheel diode) -> output inductor
        conn("sec_in",   {pin("T1", "secondary1_start"), pin("Dfwd", "anode")}),
        conn("sec_rect", {pin("Dfwd", "cathode"), pin("Dfw", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("T1", "primary_end"), pin("T1", "secondary1_end"),
                          pin("Dfw", "anode"), pin("Csn", "2"), pin("Csn2", "2"), prt("gnd")}),
        conn("gate_main_net",  {pin("Q1", "gate"), prt("gate_main")}),
        conn("gate_clamp_net", {pin("Sc", "gate"), prt("gate_clamp")})});

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
        pstage("acfCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("acfCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("acfCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("acfCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Main switch Q1 (duty D, phase 0); clamp switch Sc complementary (on during the reset interval),
    // a dead-band after Q1 turns off and trimmed not to wrap past the period.
    const double D = d.dutyCycle, dt = d.deadFraction;
    auto stim = [&](const char* sw, const char* sig, double duty, double phaseDeg) {
        json st; st["stage"] = "acfCell"; st["component"] = sw; st["signal"] = sig;
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fsw;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("Q1", "gate", D, 0.0),
        stim("Sc", "gate", (1.0 - D) - 2.0 * dt, (D + dt) * 360.0)});
    return tas;
}

} // namespace Kirchhoff
