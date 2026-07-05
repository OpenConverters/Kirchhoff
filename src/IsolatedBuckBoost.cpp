#include "IsolatedBuckBoost.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_isolated_buck_boost + excitations_processed/winding_current
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
    d.secondaryVoltage = std::abs(nominal(dr.at("outputs").at(1).at("voltage")));  // magnitude (rail may be inverting)
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
    const json& opOut = tasInputs.at("operatingPoints").at(0).at("outputs");
    const size_t nOut = dr.at("outputs").size();

    // Flyback duty D = V_pri / (Vin·η + V_pri).  N = V_pri/(V_sec + Vd), ideal Vd=0.
const double Vd = req::dideal_diode_drop(Ipri);  // DIDEAL Vf at the primary rectifier current
    d.dutyCycle  = (Vpri + Vd) / (Vin * d.efficiency + Vpri + Vd);
    double N = Vpri / Vsec;  // measured output is the primary buck rail (no rectifier drop); secondary is internal
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(N * 100.0) / 100.0);

    // Per-isolated-rail turns ratios (outputs[1..], ABT #86). secondaries[0] (output[1]) reproduces the
    // scalar d.turnsRatio; each extra rail k gets its own N_k = (V_pri+Vd)/(V_sec_k+Vd_k) (it has a real
    // flyback rectifier). The magnetizing current reflects EVERY rail: ΔI = ripple·(I_pri + Σ I_sec_k/N_k).
    std::vector<double> secTurns;
    double sumReflected = 0.0;
    for (size_t k = 1; k < nOut; ++k) {
        const double Vsec_k = std::abs(nominal(dr.at("outputs").at(k).at("voltage")));
        const double Isec_k = opOut.at(k).at("power").get<double>() / Vsec_k;
        double Nk;
        if (k == 1) {
            Nk = d.turnsRatio;
        } else {
            const double Vd_k = req::dideal_diode_drop(Isec_k);
            Nk = req::provided_turns_ratio(dr, k - 1).value_or(
                std::round((Vpri + Vd) / (Vsec_k + Vd_k) * 100.0) / 100.0);
        }
        secTurns.push_back(Nk);
        sumReflected += Isec_k / Nk;
    }
    // Lmag = V_pri·Vin_max / ((V_pri + Vin_max)·2·Fs·ΔI),  ΔI = ripple·(I_pri + ΣI_sec/N) (reflected).
    const double Imax = Ipri + sumReflected;   // single-secondary: I_pri + I_sec/N (byte-identical)
    const double dI = cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Imax;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        Vpri * vinMax / ((Vpri + vinMax) * 2.0 * Fs * dI));

    d.loadResistance          = Vpri * Vpri / d.primaryPower;     // primary (synthesized at output port)
    d.secondaryLoadResistance = Vsec * Vsec / d.secondaryPower;   // secondary (explicit internal)
    d.outputCapacitance  = 100e-6;    // Cpri (matches MKF)
    d.secondaryCapacitance = 100e-6;  // Cout_sec (matches MKF)

    // Isolated-secondary legs. secondaries[0] duplicates the secondary* scalars byte-for-byte; extra rails
    // carry their own turns ratio / load / cap and are exposed on external vout<i> ports by the builder.
    for (size_t k = 1; k < nOut; ++k) {
        IsolatedBuckBoostSecondaryLeg leg{};
        leg.voltage    = std::abs(nominal(dr.at("outputs").at(k).at("voltage")));
        leg.power      = opOut.at(k).at("power").get<double>();
        leg.turnsRatio = secTurns.at(k - 1);
        if (k == 1) {
            leg.loadResistance = d.secondaryLoadResistance;
            leg.capacitance    = d.secondaryCapacitance;
        } else {
            leg.loadResistance = leg.voltage * leg.voltage / leg.power;
            leg.capacitance    = 100e-6;
        }
        d.secondaries.push_back(leg);
    }
    return d;
}

static json build_isolated_buck_boost_tas_multi(const IsolatedBuckBoostDesign& d);

json build_isolated_buck_boost_tas(const IsolatedBuckBoostDesign& d) {
    // Multi-output (>1 isolated secondary, ABT #86) takes the generalized deck; the single-secondary
    // (2-output) default stays byte-identical below.
    if (d.secondaries.size() > 1) return build_isolated_buck_boost_tas_multi(d);
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
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };
    auto diode  = [](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw;

    // --- coupled-inductor (2-winding flyback) stresses from the SINGLE FHA source (the SPICE-validated
    // analytical solver). analytical_isolated_buck_boost returns the transformer's Primary + Secondary 0
    // excitations. The WORST-CASE corner (Vin_min → higher duty D=Vpri/(Vin+Vpri)·η → higher primary
    // current) drives the component RATINGS; the DECLARED nominal operating point is what the TAS embeds.
    namespace AN = Kirchhoff::analytical;
    const double IpriLoad = d.primaryPower / d.primaryVoltage;
    const double IsecLoad = d.secondaryPower / d.secondaryVoltage;
    const MAS::OperatingPoint aopWorst = AN::analytical_isolated_buck_boost(d.inputVoltageMin, d.primaryVoltage,
                                            IpriLoad, d.secondaryVoltage, IsecLoad, fsw, Lm, N, 0.0, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_isolated_buck_boost(d.inputVoltage,    d.primaryVoltage,
                                            IpriLoad, d.secondaryVoltage, IsecLoad, fsw, Lm, N, 0.0, d.efficiency);
    // Primary winding stresses from the worst-case corner (winding 0).
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");

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
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {N}, isoSides, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"));

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

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Single flyback switch QS1 at duty D (sets |V_pri| = Vin·D/(1−D)).
    { json st; st["stage"] = "flybuckboostCell"; st["component"] = "QS1"; st["signal"] = "gate";
      st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
      st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phase"] = 0.0;
      tas["simulation"]["stimulus"] = json::array({st}); }
    req::finalize_control_seeds(tas, Topology::ISOLATED_BUCK_BOOST_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

// Multi-output isolated buck-boost (ABT #86): the inverting non-isolated primary rail (output[0], node
// "vout"/"Vout", negative) plus N isolated flyback secondaries (outputs[1..]). Each isolated rail gets its
// own secondary winding, flyback rectifier, and output cap, and is exposed on an EXTERNAL vout<i> port
// (the assembler synthesizes its load) so every rail is independently observable/regulated. The primary
// side (QS1, T1 primary, Dpri, Cpri) is identical to the single-output deck; only the secondary side loops.
static json build_isolated_buck_boost_tas_multi(const IsolatedBuckBoostDesign& d) {
    auto port = [](const std::string& n) { json p; p["name"] = n; return p; };
    auto pin  = [](const std::string& c, const std::string& p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const std::string& p) { json e; e["port"] = p; return e; };
    auto conn = [](const std::string& name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const std::string& name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto sp = [](const char* st, const std::string& po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const std::string& name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };
    auto mosfet = [](json reqs = json()) { json j; j["semiconductor"]["mosfet"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };
    auto diode  = [](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };

    const double Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw;
    const size_t nSec = d.secondaries.size();

    // Per-rail vectors for the single FHA source. The primary reflects EVERY rail's referred current.
    namespace AN = Kirchhoff::analytical;
    const double IpriLoad = d.primaryPower / d.primaryVoltage;
    std::vector<double> Vsecs, Isecs, Ns;
    for (const auto& leg : d.secondaries) {
        Vsecs.push_back(leg.voltage);
        Isecs.push_back(leg.power / leg.voltage);
        Ns.push_back(leg.turnsRatio);
    }
    const MAS::OperatingPoint aopWorst = AN::analytical_isolated_buck_boost(d.inputVoltageMin, d.primaryVoltage,
                                            IpriLoad, Vsecs, Isecs, fsw, Lm, Ns, 0.0, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_isolated_buck_boost(d.inputVoltage,    d.primaryVoltage,
                                            IpriLoad, Vsecs, Isecs, fsw, Lm, Ns, 0.0, d.efficiency);
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");

    // --- semiconductor stresses (flyback-class, max-stress corner Vin_max) ---
    const double VdsStress = d.inputVoltageMax + d.primaryVoltage;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.primaryPower / (IrmsPri * IrmsPri);
    const double VrPri    = d.inputVoltageMax / cfg::v_derate_diode(d.config);
    const double maxVfPri = (VrPri < 100.0) ? 0.6 : 1.2;

    // Coupled inductor: primary winding (Lmag, tied to gnd) + one secondary winding per isolated rail.
    // turnsRatios = [N_0, N_1, …]; isolationSides = [primary, secondary, tertiary, …] (each rail its own
    // galvanic side). Flyback dot polarity encoded by primary_start=pri_in, secondary<k>_start=gnd.
    std::vector<std::string> isoSides{"primary"};
    for (size_t i = 0; i < nSec; ++i) isoSides.push_back(req::isolation_side(1 + i));
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, Ns, isoSides, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"));

    json cpri; cpri["capacitor"] = json::object();
    cpri["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cpri["inputs"]["designRequirements"]["ratedVoltage"] = d.primaryVoltage * 2;

    // Base primary-side components (shared): switch + coupled inductor + primary rectifier + primary cap.
    std::vector<json> comps{
        comp("QS1", mosfet(req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0))), comp("T1", xfmr),
        comp("Dpri", diode(req::diode(VrPri, IpriLoad / 0.7, maxVfPri, 0.05 * T))), comp("Cpri", cpri)};
    std::vector<json> cports{port("vin"), port("gnd"), port("vout")};
    std::vector<json> conns{
        conn("vin_net",  {pin("QS1", "drain"), prt("vin")}),
        conn("pri_in",   {pin("QS1", "source"), pin("T1", "primary_start"), pin("Dpri", "cathode")}),
        conn("vpri_out", {pin("Dpri", "anode"), pin("Cpri", "1"), prt("vout")})};
    std::vector<json> gndEps{pin("T1", "primary_end"), pin("Cpri", "2")};

    // One isolated flyback rail per secondary: winding secondary<i+1> -> Dsec<i+1> -> Csec<i+1> -> external
    // vout<i+2> port (the assembler synthesizes each rail's load). Dot at gnd gives flyback polarity.
    for (size_t i = 0; i < nSec; ++i) {
        const auto& leg = d.secondaries[i];
        const double IsecLoad = leg.power / leg.voltage, N_i = leg.turnsRatio;
        const std::string wnd  = "secondary" + std::to_string(i + 1);
        const std::string dsec = "Dsec" + std::to_string(i + 1);
        const std::string csec = "Csec" + std::to_string(i + 1);
        const std::string voutP = "vout" + std::to_string(i + 2);   // vout2, vout3, …
        // Dsec_i blocks Vsec_i + reflected Vin/N_i during ON; carries Isec_i.
        const double VrSec    = (leg.voltage + d.inputVoltageMax / N_i) / cfg::v_derate_diode(d.config);
        const double maxVfSec = (VrSec < 100.0) ? 0.6 : 1.2;
        json csecJ; csecJ["capacitor"] = json::object();
        csecJ["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.capacitance;
        csecJ["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;

        comps.push_back(comp(dsec, diode(req::diode(VrSec, IsecLoad / 0.7, maxVfSec, 0.05 * T))));
        comps.push_back(comp(csec, csecJ));
        cports.push_back(port(voutP));
        conns.push_back(conn("sec_in" + std::to_string(i + 1),   {pin("T1", wnd + "_end"), pin(dsec, "anode")}));
        conns.push_back(conn(voutP + "_net",                     {pin(dsec, "cathode"), pin(csec, "1"), prt(voutP)}));
        gndEps.push_back(pin("T1", wnd + "_start"));
        gndEps.push_back(pin(csec, "2"));
    }
    gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));
    cports.push_back(port("g1"));
    conns.push_back(conn("g1_net", {pin("QS1", "gate"), prt("g1")}));

    json cell; cell["name"] = "flybuckboost-cell";
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    json opDoc; opDoc["name"] = "full_load"; opDoc["inputVoltage"] = d.inputVoltage; opDoc["ambientTemperature"] = 25.0;
    { json o; o["name"] = "vpri"; o["voltage"]["nominal"] = d.primaryVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o});
      json oo; oo["name"] = "vpri"; oo["power"] = d.primaryPower; opDoc["outputs"] = json::array({oo}); }
    for (size_t i = 0; i < nSec; ++i) {
        const std::string oname = "vsec" + std::to_string(i + 1);
        json o; o["name"] = oname; o["voltage"]["nominal"] = d.secondaries[i].voltage; o["regulation"] = "voltage";
        dreq["outputs"].push_back(o);
        json oo; oo["name"] = oname; oo["power"] = d.secondaries[i].power; opDoc["outputs"].push_back(oo);
    }
    tas["inputs"]["operatingPoints"] = json::array({opDoc});

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("flybuckboostCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("flybuckboostCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("flybuckboostCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("flybuckboostCell", "vout")})};
    for (size_t i = 0; i < nSec; ++i) {
        const std::string g = "Vout" + std::to_string(i + 2), pt = "vout" + std::to_string(i + 2);
        iscs.push_back(isc(g, "externalPort", "output", {sp("flybuckboostCell", pt)}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    { json st; st["stage"] = "flybuckboostCell"; st["component"] = "QS1"; st["signal"] = "gate";
      st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
      st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phase"] = 0.0;
      tas["simulation"]["stimulus"] = json::array({st}); }
    req::finalize_control_seeds(tas, Topology::ISOLATED_BUCK_BOOST_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
