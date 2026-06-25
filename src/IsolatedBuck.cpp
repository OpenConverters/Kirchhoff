#include "IsolatedBuck.hpp"
#include "Dimension.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kRippleRatio = 0.4;   // primary-inductor current ripple (MKF IsolatedBuck default)
} // namespace

IsolatedBuckDesign design_isolated_buck(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    if (dr.at("outputs").size() < 2)
        throw std::runtime_error("isolated_buck design: needs 2 outputs (primary + isolated secondary)");
    IsolatedBuckDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.primaryVoltage   = nominal(dr.at("outputs").at(0).at("voltage"));
    d.secondaryVoltage = nominal(dr.at("outputs").at(1).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
    {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage   = op.at("inputVoltage").get<double>();
        d.primaryPower   = op.at("outputs").at(0).at("power").get<double>();
        d.secondaryPower = op.at("outputs").at(1).at("power").get<double>();
    }
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vpri = d.primaryVoltage, Vsec = d.secondaryVoltage, Fs = d.switchingFrequency;
    const double Ipri = d.primaryPower / Vpri, Isec = d.secondaryPower / Vsec;

    // D = V_pri / (Vin·η).  N = V_pri / (V_sec + Vd), ideal Vd=0.  (MKF IsolatedBuck.)
    d.dutyCycle  = Vpri / (Vin * d.efficiency);
    double N = Vpri / Vsec;  // measured output is the primary buck rail (no rectifier drop); secondary is internal
    d.turnsRatio = std::round(N * 100.0) / 100.0;

    // Lmag = (Vin_max − V_pri)·V_pri / (Vin_max·Fs·ΔI),  ΔI = ripple·(I_pri + ΣI_sec/N) (reflected).
    const double Imax = Ipri + Isec / d.turnsRatio;
    const double dI = cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Imax;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        (vinMax - Vpri) * Vpri / (vinMax * Fs * dI));

    d.loadResistance          = Vpri * Vpri / d.primaryPower;     // primary (synthesized at output port)
    d.secondaryLoadResistance = Vsec * Vsec / d.secondaryPower;   // secondary (explicit internal)
    d.outputCapacitance  = 100e-6;    // Cpri (matches MKF)
    d.secondaryCapacitance = 100e-6;  // Cout_sec (matches MKF)
    return d;
}

json build_isolated_buck_tas(const IsolatedBuckDesign& d) {
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

    // Coupled inductor (2-winding magnetic): primary winding = the buck inductor (Lmag), secondary
    // winding gives the isolated flyback rail. turnsRatios = [N].
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json rn; rn["nominal"] = N; xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    json cpri; cpri["capacitor"] = json::object();
    cpri["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cpri["inputs"]["designRequirements"]["ratedVoltage"] = d.primaryVoltage * 2;

    json csec; csec["capacitor"] = json::object();
    csec["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.secondaryCapacitance;
    csec["inputs"]["designRequirements"]["ratedVoltage"] = d.secondaryVoltage * 2;

    // Explicit internal secondary load (the harness only synthesizes ONE load, at the primary output
    // port). RAS resistor brick: deviceType "resistor" + designRequirements.resistance.
    json rsec; rsec["resistor"] = json::object();
    rsec["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rsec["inputs"]["designRequirements"]["resistance"]["nominal"] = d.secondaryLoadResistance;

    json cell; cell["name"] = "flybuck-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")});
    cell["components"] = json::array({
        comp("QS1", mosfet()), comp("QS2", mosfet()),
        comp("DS1", diode()),  comp("DS2", diode()),
        comp("T1", xfmr), comp("Cpri", cpri),
        comp("Dsec", diode()), comp("Csec", csec), comp("Rsec", rsec)});
    cell["connections"] = json::array({
        // High-side buck switch QS1 (vin->sw) + synchronous low-side QS2 (sw->gnd) driven complementary
        // with a small dead time. Anti-parallel body diodes DS1 (sw->vin) / DS2 (gnd->sw) freewheel the
        // buck-inductor current during the dead time so sw_node never floats AND there is no
        // QS1/QS2 shoot-through (a hard vin->gnd short every crossover, which MKF's deck hides via its
        // S1-channel current sense + spike clamp but Kirchhoff would otherwise charge to the source).
        conn("vin_net",  {pin("QS1", "drain"), pin("DS1", "cathode"), prt("vin")}),
        conn("sw_node",  {pin("QS1", "source"), pin("QS2", "drain"), pin("T1", "primary_start"),
                          pin("DS1", "anode"), pin("DS2", "cathode")}),
        // Primary buck rail = the coupled inductor's primary winding output, with Cpri across it. This
        // is the COMPARED output (the harness synthesizes the primary load here).
        conn("vpri_out", {pin("T1", "primary_end"), pin("Cpri", "1"), prt("vout")}),
        // Secondary winding -> flyback rectifier diode -> isolated rail (internal). Winding dot at gnd
        // (T1.secondary1_start) matches MKF's "Lsec 0 sec_in" so the diode conducts during QS1 OFF.
        conn("sec_in",   {pin("T1", "secondary1_end"), pin("Dsec", "anode")}),
        conn("vout_sec", {pin("Dsec", "cathode"), pin("Csec", "1"), pin("Rsec", "1")}),
        conn("gnd_net",  {pin("QS2", "source"), pin("DS2", "anode"), pin("T1", "secondary1_start"),
                          pin("Cpri", "2"), pin("Csec", "2"), pin("Rsec", "2"), prt("gnd")}),
        conn("g1_net", {pin("QS1", "gate"), prt("g1")}),
        conn("g2_net", {pin("QS2", "gate"), prt("g2")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "vpri"; o["voltage"]["nominal"] = d.primaryVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "vpri"; o["power"] = d.primaryPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        pstage("flybuckCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("flybuckCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("flybuckCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("flybuckCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // QS1 buck switch at duty D (phase 0) — D alone sets V_pri = D·Vin, so it keeps the full duty. QS2
    // synchronous rectifier conducts the rest of the period MINUS a small dead time on each edge
    // (deadFrac), so QS1/QS2 never overlap. The two dead-time gaps are bridged by the body diodes.
    constexpr double deadFrac = 0.02;   // 200 ns at 100 kHz, like the other bridge ports
    auto stim = [&](const char* sw, double duty, double phaseDeg) {
        json st; st["stage"] = "flybuckCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("QS1", d.dutyCycle, 0.0),
        stim("QS2", (1.0 - d.dutyCycle) - 2.0 * deadFrac, (d.dutyCycle + deadFrac) * 360.0)});
    return tas;
}

} // namespace Kirchhoff
