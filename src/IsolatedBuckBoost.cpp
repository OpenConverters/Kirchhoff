#include "IsolatedBuckBoost.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
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
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vpri = d.primaryVoltage, Vsec = d.secondaryVoltage, Fs = d.switchingFrequency;
    const double Ipri = d.primaryPower / Vpri, Isec = d.secondaryPower / Vsec;

    // Flyback duty D = V_pri / (Vin·η + V_pri).  N = V_pri/(V_sec + Vd), ideal Vd=0.
const double Vd = req::dideal_diode_drop(Ipri);  // DIDEAL Vf at the primary rectifier current
    d.dutyCycle  = (Vpri + Vd) / (Vin * d.efficiency + Vpri + Vd);
    double N = Vpri / Vsec;  // measured output is the primary buck rail (no rectifier drop); secondary is internal
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(N * 100.0) / 100.0);
    // Lmag = V_pri·Vin_max / ((V_pri + Vin_max)·2·Fs·ΔI),  ΔI = ripple·(I_pri + ΣI_sec/N) (reflected).
    const double Imax = Ipri + Isec / d.turnsRatio;
    const double dI = cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Imax;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        Vpri * vinMax / ((Vpri + vinMax) * 2.0 * Fs * dI));

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
    auto mosfet = [](json reqs = json()) { json j; j["semiconductor"]["mosfet"] = json::object();
        if (!reqs.is_null()) j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        if (!reqs.is_null()) j["inputs"]["designRequirements"] = reqs; return j; };

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw, D = d.dutyCycle;

    // --- coupled-inductor (flyback) winding stresses (REUSE the design's voltages/powers) ---
    // Magnetizing current center referred to primary = the reflected total load current
    // Imax = I_pri + ΣI_sec/N (same quantity design_isolated_buck_boost used to size ΔI). Primary winding
    // conducts during ON (D); ripple ΔI = Vin·D·T/Lmag. Secondary conducts during OFF (1−D).
    const double IpriLoad = d.primaryPower / d.primaryVoltage;
    const double IsecLoad = d.secondaryPower / d.secondaryVoltage;
    const double IcMag = IpriLoad + IsecLoad / N;                 // magnetizing center, primary-referred
    const double dIpri = d.inputVoltage * D * T / Lm;             // primary pk-pk ripple
    const double IpkPri = IcMag + dIpri / 2.0;
    const double IrmsPri = std::sqrt(D) * std::sqrt(IcMag * IcMag + dIpri * dIpri / 12.0);
    // Secondary winding: peak = primary peak reflected by N; conducts during OFF; DC offset = the
    // secondary load current it delivers per full period.
    const double IcSec = IsecLoad, dIsec = dIpri * N;
    const double IpkSec = IpkPri * N;
    const double IrmsSec = std::sqrt(1.0 - D) * std::sqrt((IcMag * N) * (IcMag * N) + dIsec * dIsec / 12.0);

    // Winding voltages (volt-second balanced, flyback): primary sees +Vin during ON / −V_pri during OFF;
    // secondary mirrors the primary scaled by 1/N. Evaluated at the nominal operating point.
    const double VpriOn = d.inputVoltage, VpriOff = d.primaryVoltage;
    const double vPriPk = std::max(VpriOn, VpriOff), vPriPkPk = VpriOn + VpriOff;
    const double vPriRms = std::sqrt(D * VpriOn * VpriOn + (1.0 - D) * VpriOff * VpriOff);
    const double vSecPk = vPriPk / N, vSecPkPk = vPriPkPk / N;
    const double vSecRms = vPriRms / N;

    // --- semiconductor stresses (flyback-class, max-stress corner Vin_max) ---
    // QS1 blocks Vin_max + the reflected primary-rail voltage during OFF (clamp-limited).
    const double VdsStress = d.inputVoltageMax + d.primaryVoltage;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.primaryPower / (IrmsPri * IrmsPri);
    // Dpri (primary inverting-rail rectifier): blocks Vin during ON; carries the primary load current.
    const double VrPri    = d.inputVoltageMax / cfg::v_derate_diode(d.config);
    const double maxVfPri = (VrPri < 100.0) ? 0.6 : 1.2;
    // Dsec (isolated secondary flyback rectifier): blocks Vsec + reflected Vin/N; carries Isec.
    const double VrSec    = (d.secondaryVoltage + d.inputVoltageMax / N) / cfg::v_derate_diode(d.config);
    const double maxVfSec = (VrSec < 100.0) ? 0.6 : 1.2;

    // Coupled inductor (2-winding flyback magnetic): primary winding = flyback primary (Lmag, tied to
    // gnd), secondary winding gives the isolated rail. turnsRatios = [N]. Flyback dot polarity: primary
    // dot at pri_in, secondary dot at gnd (opposite ends) so the secondary blocks during ON / conducts
    // during OFF — encoded by primary_start=pri_in, secondary1_start=gnd (the dotted "start" terminals).
    // (Non-galvanic isolationVoltage not carried on the spec -> std::nullopt.)
    std::vector<std::string> isoSides{"primary", "secondary"};
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {N}, isoSides, std::nullopt, 25.0, {
        req::winding_excitation("flybackPrimary",   fsw, IpkPri, IrmsPri, IcMag, dIpri, D,
                                vPriPk, vPriRms, 0.0, vPriPkPk),
        req::winding_excitation("flybackSecondary", fsw, IpkSec, IrmsSec, IcSec, dIsec, 1.0 - D,
                                vSecPk, vSecRms, 0.0, vSecPkPk)});

    json cpri; cpri["capacitor"] = json::object();
    cpri["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cpri["inputs"]["designRequirements"]["ratedVoltage"] = d.primaryVoltage * 2;

    json csec; csec["capacitor"] = json::object();
    csec["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.secondaryCapacitance;
    csec["inputs"]["designRequirements"]["ratedVoltage"] = d.secondaryVoltage * 2;

    json rsec; rsec["resistor"] = json::object();
    rsec["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rsec["inputs"]["designRequirements"]["resistance"]["nominal"] = d.secondaryLoadResistance;
    rsec["inputs"]["designRequirements"]["powerRating"] =
        d.secondaryVoltage * d.secondaryVoltage / d.secondaryLoadResistance;  // load: P = V^2/R
    rsec["inputs"]["designRequirements"]["role"] = "bleed";

    json cell; cell["name"] = "flybuckboost-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1")});
    cell["components"] = json::array({
        comp("QS1", mosfet(req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0))), comp("T1", xfmr),
        comp("Dpri", diode(req::diode(VrPri, IpriLoad / 0.7, maxVfPri, 0.05 * T))), comp("Cpri", cpri),
        comp("Dsec", diode(req::diode(VrSec, IsecLoad / 0.7, maxVfSec, 0.05 * T))), comp("Csec", csec), comp("Rsec", rsec)});
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
        req::control_stage("pwmController"),
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
      st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phase"] = 0.0;
      tas["simulation"]["stimulus"] = json::array({st}); }
    req::finalize_control_seeds(tas, "isolatedBuckBoostConverter");  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
