#include "IsolatedBuckBoost.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
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
    throw std::runtime_error("isolated_buck_boost design: no nominal");
}
constexpr double kRippleRatio = 0.4;   // primary-inductor current ripple (MKF IsolatedBuckBoost default)
} // namespace

IsolatedBuckBoostDesign design_isolated_buck_boost(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    if (dr.at("outputs").size() < 2)
        throw std::runtime_error("isolated_buck_boost design: needs 2 outputs (primary + isolated secondary)");
    IsolatedBuckBoostDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.primaryVoltage   = nominal(dr.at("outputs").at(0).at("voltage"));   // magnitude (rail is inverting)
    d.secondaryVoltage = nominal(dr.at("outputs").at(1).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
    {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage   = op.at("inputVoltage").get<double>();
        d.primaryPower   = op.at("outputs").at(0).at("power").get<double>();
        d.secondaryPower = op.at("outputs").at(1).at("power").get<double>();
    }
    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vpri = d.primaryVoltage, Vsec = d.secondaryVoltage, Fs = d.switchingFrequency;
    const double Ipri = d.primaryPower / Vpri, Isec = d.secondaryPower / Vsec;

    // Flyback duty D = V_pri / (Vin·η + V_pri).  N = V_pri/(V_sec + Vd), ideal Vd=0.
const double Vd = req::dideal_diode_drop(Ipri);  // DIDEAL Vf at the primary rectifier current
    d.dutyCycle  = (Vpri + Vd) / (Vin * d.efficiency + Vpri + Vd);
    double N = Vpri / Vsec;  // measured output is the primary buck rail (no rectifier drop); secondary is internal
    d.turnsRatio = std::round(N * 100.0) / 100.0;

    // Lmag = V_pri·Vin_max / ((V_pri + Vin_max)·2·Fs·ΔI),  ΔI = ripple·(I_pri + ΣI_sec/N) (reflected).
    const double Imax = Ipri + Isec / d.turnsRatio;
    const double dI = kRippleRatio * Imax;
    d.magnetizingInductance = Vpri * vinMax / ((Vpri + vinMax) * 2.0 * Fs * dI);

    d.loadResistance          = Vpri * Vpri / d.primaryPower;     // primary (synthesized at output port)
    d.secondaryLoadResistance = Vsec * Vsec / d.secondaryPower;   // secondary (explicit internal)
    d.outputCapacitance  = 100e-6;    // Cpri (matches MKF)
    d.secondaryCapacitance = 100e-6;  // Cout_sec (matches MKF)
    return d;
}

json build_isolated_buck_boost_tas(const IsolatedBuckBoostDesign& d) {
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

    // Coupled inductor (2-winding flyback magnetic): primary winding = flyback primary (Lmag, tied to
    // gnd), secondary winding gives the isolated rail. turnsRatios = [N]. Flyback dot polarity: primary
    // dot at pri_in, secondary dot at gnd (opposite ends) so the secondary blocks during ON / conducts
    // during OFF — encoded by primary_start=pri_in, secondary1_start=gnd (the dotted "start" terminals).
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json rn; rn["nominal"] = N; xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    json cpri; cpri["capacitor"] = json::object();
    cpri["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cpri["inputs"]["designRequirements"]["ratedVoltage"] = d.primaryVoltage * 2;

    json csec; csec["capacitor"] = json::object();
    csec["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.secondaryCapacitance;
    csec["inputs"]["designRequirements"]["ratedVoltage"] = d.secondaryVoltage * 2;

    json rsec; rsec["resistor"] = json::object();
    rsec["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rsec["inputs"]["designRequirements"]["resistance"]["nominal"] = d.secondaryLoadResistance;

    json cell; cell["name"] = "flybuckboost-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1")});
    cell["components"] = json::array({
        comp("QS1", mosfet()), comp("T1", xfmr),
        comp("Dpri", diode()), comp("Cpri", cpri),
        comp("Dsec", diode()), comp("Csec", csec), comp("Rsec", rsec)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("QS1", "drain"), prt("vin")}),
        // pri_in: switch source + coupled-inductor primary (to gnd) + primary diode cathode. During ON
        // S1 ramps i(Lpri) from Vin; during OFF the inductor pulls pri_in negative and Dpri (anode at
        // the negative rail vpri_out, cathode pri_in) conducts, discharging the inductor into Cpri and
        // driving vpri_out MORE negative -> inverting buck-boost rail.
        conn("pri_in",   {pin("QS1", "source"), pin("T1", "primary_start"), pin("Dpri", "cathode")}),
        // Inverting primary rail = COMPARED output (negative). Harness synthesizes the primary load here
        // and compares on magnitude (like Ćuk).
        conn("vpri_out", {pin("Dpri", "anode"), pin("Cpri", "1"), prt("vout")}),
        // Secondary winding -> flyback rectifier -> isolated rail (internal). Dot at gnd
        // (T1.secondary1_start) gives flyback polarity: Dsec conducts during QS1 OFF.
        conn("sec_in",   {pin("T1", "secondary1_end"), pin("Dsec", "anode")}),
        conn("vout_sec", {pin("Dsec", "cathode"), pin("Csec", "1"), pin("Rsec", "1")}),
        conn("gnd_net",  {pin("T1", "primary_end"), pin("T1", "secondary1_start"),
                          pin("Cpri", "2"), pin("Csec", "2"), pin("Rsec", "2"), prt("gnd")}),
        conn("g1_net", {pin("QS1", "gate"), prt("g1")})});

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
        pstage("flybuckboostCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("flybuckboostCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("flybuckboostCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("flybuckboostCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Single flyback switch QS1 at duty D (sets |V_pri| = Vin·D/(1−D)).
    { json st; st["stage"] = "flybuckboostCell"; st["component"] = "QS1"; st["signal"] = "gate";
      st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
      st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phaseDeg"] = 0.0;
      tas["simulation"]["stimulus"] = json::array({st}); }
    return tas;
}

} // namespace Kirchhoff
