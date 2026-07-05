#include "IsolatedBuck.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_isolated_buck + excitations_processed/winding_current
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
    const size_t nOut = dr.at("outputs").size();
    if (nOut < 2)
        throw std::runtime_error("isolated_buck design: needs >=2 outputs (primary + >=1 isolated secondary)");
    const json& op = tasInputs.at("operatingPoints").at(0);
    IsolatedBuckDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.primaryVoltage   = nominal(dr.at("outputs").at(0).at("voltage"));
    d.secondaryVoltage = nominal(dr.at("outputs").at(1).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
    d.inputVoltage   = op.at("inputVoltage").get<double>();
    d.primaryPower   = op.at("outputs").at(0).at("power").get<double>();
    d.secondaryPower = op.at("outputs").at(1).at("power").get<double>();
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vpri = d.primaryVoltage, Vsec = d.secondaryVoltage, Fs = d.switchingFrequency;
    const double Ipri = d.primaryPower / Vpri;

    // D = V_pri / (Vin·η).  N = V_pri / V_sec, ideal Vd=0.  (MKF IsolatedBuck.)
    d.dutyCycle  = Vpri / (Vin * d.efficiency);
    double N = Vpri / Vsec;  // measured output is the primary buck rail (no rectifier drop); secondary is internal
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(N * 100.0) / 100.0);

    // Per-secondary legs (multi-output: N isolated coupled secondaries, ABT #86). Each rail scales its own
    // turns ratio and reflects Iout_sec_k/N_k into the primary. With a SINGLE isolated secondary the rail is
    // internal (measured output is the primary buck rail) and keeps MKF's ideal-Vd ratio (== d.turnsRatio,
    // byte-identical deck). With >1 secondary every rail is EXPOSED on its own port through a real flyback
    // rectifier, so — like the forward family — the turns ratio compensates the diode drop:
    // N_k = V_pri/(V_sec_k + Vd_k); otherwise the low-voltage rails droop by ~Vd under load.
    const bool multiSecondary = (nOut > 2);
    for (size_t k = 0; k + 1 < nOut; ++k) {
        IsolatedBuckSecondaryLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(k + 1).at("voltage"));
        leg.power   = op.at("outputs").at(k + 1).at("power").get<double>();
        if (leg.voltage <= 0.0 || leg.power <= 0.0)
            throw std::invalid_argument("isolated_buck design: secondary output " + std::to_string(k + 1) +
                                        " needs a positive voltage and power");
        if (!multiSecondary && k == 0) {
            leg.turnsRatio = d.turnsRatio;   // single internal secondary: byte-identical scalar ratio
        } else {
            const double Vd = req::dideal_diode_drop(leg.power / leg.voltage);   // exposed rectifier drop
            double Nk = Vpri / (leg.voltage + Vd);
            leg.turnsRatio = req::provided_turns_ratio(dr, k).value_or(std::round(Nk * 100.0) / 100.0);
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        leg.capacitance = 100e-6;   // Cout_sec (matches MKF)
        d.secondaries.push_back(leg);
    }

    // Lmag = (Vin_max − V_pri)·V_pri / (Vin_max·Fs·ΔI),  ΔI = ripple·(I_pri + Σ I_sec_k/N_k) (reflected).
    double sumReflected = 0.0;
    for (const auto& leg : d.secondaries) sumReflected += (leg.power / leg.voltage) / leg.turnsRatio;
    const double Imax = Ipri + sumReflected;
    const double dI = cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Imax;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        (vinMax - Vpri) * Vpri / (vinMax * Fs * dI));

    d.loadResistance          = Vpri * Vpri / d.primaryPower;     // primary (synthesized at output port)
    d.secondaryLoadResistance = d.secondaries[0].loadResistance;  // first secondary (== secondaries[0])
    d.outputCapacitance  = 100e-6;    // Cpri (matches MKF)
    d.secondaryCapacitance = d.secondaries[0].capacitance;  // Cout_sec (== secondaries[0])
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
    auto mosfet = [](json reqs = json()) { json j; j["semiconductor"]["mosfet"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };
    auto diode  = [](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };

    // ===== MULTI-OUTPUT PATH (N isolated secondaries, ABT #86) ==============================
    // With more than one isolated secondary each rail becomes its OWN coupled secondary winding +
    // flyback rectifier + output cap, exposed on an external vout<i> port (the assembler synthesizes
    // each rail's load). The single-secondary path below stays byte-identical (secondary INTERNAL).
    if (d.secondaries.size() > 1) {
        namespace AN = Kirchhoff::analytical;
        const double Lm = d.magnetizingInductance;
        const double fsw = d.switchingFrequency, T = 1.0 / fsw;
        const size_t nSec = d.secondaries.size();
        const double IpriLoad = d.primaryPower / d.primaryVoltage;

        std::vector<double> secV, secI, secN;
        for (const auto& leg : d.secondaries) {
            secV.push_back(leg.voltage);
            secI.push_back(leg.power / leg.voltage);
            secN.push_back(leg.turnsRatio);
        }
        // Coupled-inductor stresses from the SINGLE FHA source (SPICE-validated analytical solver).
        // Worst-case corner (Vin_min → higher duty → higher magnetizing current) drives ratings; the
        // declared nominal operating point is what the TAS embeds. (Design uses ideal Vd=0.)
        const MAS::OperatingPoint aopWorst = AN::analytical_isolated_buck(d.inputVoltageMin, d.primaryVoltage,
                                                IpriLoad, secV, secI, secN, fsw, Lm, 0.0, d.efficiency);
        const MAS::OperatingPoint aopNom   = AN::analytical_isolated_buck(d.inputVoltage,    d.primaryVoltage,
                                                IpriLoad, secV, secI, secN, fsw, Lm, 0.0, d.efficiency);
        const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
        const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");

        // Synchronous buck pair QS1/QS2 each block Vin_max and carry the primary-winding current.
        const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
        const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.primaryPower / (IrmsPri * IrmsPri);

        // Coupled inductor: primary winding (= buck inductor) + one secondary winding per isolated rail.
        // turnsRatios = [N_0, N_1, …]; isolationSides = [primary, secondary, tertiary, …] (each rail on
        // its own ground). Sourced from analytical_isolated_buck at the nominal operating point.
        std::vector<double> xfmrRatios;  std::vector<std::string> isoSides{"primary"};
        for (size_t k = 0; k < nSec; ++k) { xfmrRatios.push_back(d.secondaries[k].turnsRatio);
                                            isoSides.push_back(req::isolation_side(k + 1)); }
        json xfmr; xfmr["magnetic"] = json::object();
        xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, xfmrRatios, isoSides, std::nullopt, 25.0,
            AN::excitations_processed(aopNom, "T1"));

        json cpri; cpri["capacitor"] = json::object();
        cpri["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
        cpri["inputs"]["designRequirements"]["ratedVoltage"] = d.primaryVoltage * 2;

        std::vector<json> comps{
            comp("QS1", mosfet(req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0))),
            comp("QS2", mosfet(req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0))),
            comp("DS1", diode(req::body_diode(ratedVds, IpkPri))), comp("DS2", diode(req::body_diode(ratedVds, IpkPri))),
            comp("T1", xfmr), comp("Cpri", cpri)};
        std::vector<json> cports{port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")};
        std::vector<json> conns{
            conn("vin_net",  {pin("QS1", "drain"), pin("DS1", "cathode"), prt("vin")}),
            conn("sw_node",  {pin("QS1", "source"), pin("QS2", "drain"), pin("T1", "primary_start"),
                              pin("DS1", "anode"), pin("DS2", "cathode")}),
            conn("vpri_out", {pin("T1", "primary_end"), pin("Cpri", "1"), prt("vout")})};
        // gnd rail accumulates the low-side switch, DS2, Cpri return, and each secondary's dot + Csec return.
        std::vector<json> gndEps{pin("QS2", "source"), pin("DS2", "anode"), pin("Cpri", "2")};

        for (size_t k = 0; k < nSec; ++k) {
            const auto& leg = d.secondaries[k];
            const double iSec = leg.power / leg.voltage;
            const std::string kk = std::to_string(k);
            const std::string dName = "Dsec" + kk, cName = "Csec" + kk;
            const std::string wStart = "secondary" + std::to_string(k + 1) + "_start";
            const std::string wEnd   = "secondary" + std::to_string(k + 1) + "_end";
            const std::string voutP  = "vout" + std::to_string(k + 2);   // vout2, vout3, …

            // Secondary flyback rectifier: blocks Vsec + reflected (Vin−Vpri)/N_k; carries Isec_k.
            const double VrSec  = (leg.voltage + (d.inputVoltageMax - d.primaryVoltage) / leg.turnsRatio) / cfg::v_derate_diode(d.config);
            const double maxVf  = (VrSec < 100.0) ? 0.6 : 1.2;
            comps.push_back(comp(dName.c_str(), diode(req::diode(VrSec, iSec / 0.7, maxVf, 0.05 * T))));
            json csec; csec["capacitor"] = json::object();
            csec["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.capacitance;
            csec["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;
            comps.push_back(comp(cName.c_str(), csec));

            cports.push_back(port(voutP.c_str()));
            // Secondary winding (dot/start at gnd) -> flyback diode -> isolated rail -> external port.
            conns.push_back(conn(("sec_in" + kk).c_str(),  {pin("T1", wEnd.c_str()), pin(dName.c_str(), "anode")}));
            conns.push_back(conn((voutP + "_net").c_str(), {pin(dName.c_str(), "cathode"), pin(cName.c_str(), "1"), prt(voutP.c_str())}));
            gndEps.push_back(pin("T1", wStart.c_str()));
            gndEps.push_back(pin(cName.c_str(), "2"));
        }
        gndEps.push_back(prt("gnd"));
        conns.push_back(conn("gnd_net", gndEps));
        conns.push_back(conn("g1_net", {pin("QS1", "gate"), prt("g1")}));
        conns.push_back(conn("g2_net", {pin("QS2", "gate"), prt("g2")}));

        json cell; cell["name"] = "flybuck-cell";
        cell["ports"] = cports;  cell["components"] = comps;  cell["connections"] = conns;

        json tas;
        json& dreq = tas["inputs"]["designRequirements"];
        dreq["efficiency"] = d.efficiency;
        dreq["inputType"] = "dc";
        dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
        dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
        dreq["outputs"] = json::array();
        json opDoc; opDoc["name"] = "full_load"; opDoc["inputVoltage"] = d.inputVoltage; opDoc["ambientTemperature"] = 25.0;
        opDoc["outputs"] = json::array();
        { json o; o["name"] = "vpri"; o["voltage"]["nominal"] = d.primaryVoltage; o["regulation"] = "voltage";
          dreq["outputs"].push_back(o);
          json oo; oo["name"] = "vpri"; oo["power"] = d.primaryPower; opDoc["outputs"].push_back(oo); }
        for (size_t k = 0; k < nSec; ++k) {
            const std::string on = "vsec" + std::to_string(k + 1);
            json o; o["name"] = on; o["voltage"]["nominal"] = d.secondaries[k].voltage; o["regulation"] = "voltage";
            dreq["outputs"].push_back(o);
            json oo; oo["name"] = on; oo["power"] = d.secondaries[k].power; opDoc["outputs"].push_back(oo);
        }
        tas["inputs"]["operatingPoints"] = json::array({opDoc});

        tas["topology"]["stages"] = json::array({
            req::control_stage("pwmController"),
            pstage("flybuckCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
        std::vector<json> iscs{
            isc("Vin", "externalPort", "input", {sp("flybuckCell", "vin")}),
            isc("GND", "externalPort", "input", {sp("flybuckCell", "gnd")}),
            isc("Vout", "externalPort", "output", {sp("flybuckCell", "vout")})};
        for (size_t k = 0; k < nSec; ++k) {
            const std::string g = "Vout" + std::to_string(k + 2), pt = "vout" + std::to_string(k + 2);
            iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("flybuckCell", pt.c_str())}));
        }
        tas["topology"]["interStageConnections"] = iscs;

        json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
        tas["simulation"]["analyses"] = json::array({an});
        constexpr double deadFrac = 0.02;
        auto stim = [&](const char* sw, double duty, double phaseDeg) {
            json st; st["stage"] = "flybuckCell"; st["component"] = sw; st["signal"] = "gate";
            st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
            st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
            return st; };
        tas["simulation"]["stimulus"] = json::array({
            stim("QS1", d.dutyCycle, 0.0),
            stim("QS2", (1.0 - d.dutyCycle) - 2.0 * deadFrac, (d.dutyCycle + deadFrac) * 360.0)});
        req::finalize_control_seeds(tas, Topology::ISOLATED_BUCK_CONVERTER);
        return tas;
    }
    // ===== single-secondary path (secondary INTERNAL — byte-identical to the legacy deck) ==========
    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw;

    // --- coupled-inductor (2-winding) stresses from the SINGLE FHA source (the SPICE-validated analytical
    // solver). analytical_isolated_buck returns the transformer's Primary + Secondary 0 excitations. The
    // WORST-CASE corner (Vin_min → higher duty D=Vpri/(Vin·η) → higher magnetizing current) drives the
    // component RATINGS; the DECLARED nominal operating point is what the TAS embeds. (Design uses ideal Vd=0.)
    namespace AN = Kirchhoff::analytical;
    const double IpriLoad = d.primaryPower / d.primaryVoltage;
    const double IsecLoad = d.secondaryPower / d.secondaryVoltage;
    const MAS::OperatingPoint aopWorst = AN::analytical_isolated_buck(d.inputVoltageMin, d.primaryVoltage,
                                            IpriLoad, d.secondaryVoltage, IsecLoad, fsw, Lm, N, 0.0, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_isolated_buck(d.inputVoltage,    d.primaryVoltage,
                                            IpriLoad, d.secondaryVoltage, IsecLoad, fsw, Lm, N, 0.0, d.efficiency);
    // Primary winding (= buck inductor) stresses from the worst-case corner (winding 0).
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");

    // --- semiconductor stresses (max-stress corner Vin_max) ---
    // Synchronous buck pair QS1 (high-side) / QS2 (low-side): the switch node swings 0..Vin, so each
    // blocks Vin_max. Both carry the primary-winding (buck-inductor) current.
    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.primaryPower / (IrmsPri * IrmsPri);
    // Dsec (isolated secondary flyback rectifier): blocks Vsec + reflected (Vin−Vpri)/N; carries Isec.
    const double VrSec    = (d.secondaryVoltage + (d.inputVoltageMax - d.primaryVoltage) / N) / cfg::v_derate_diode(d.config);
    const double maxVfSec = (VrSec < 100.0) ? 0.6 : 1.2;

    // Coupled inductor (2-winding magnetic): primary winding = the buck inductor (Lmag), secondary
    // winding gives the isolated flyback rail. turnsRatios = [N]. (Non-galvanic isolationVoltage not
    // carried on the spec -> std::nullopt.)
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

    // Explicit internal secondary load (the harness only synthesizes ONE load, at the primary output
    // port). RAS resistor brick: deviceType "resistor" + designRequirements.resistance.
    json rsec; rsec["resistor"] = json::object();
    rsec["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rsec["inputs"]["designRequirements"]["resistance"]["nominal"] = d.secondaryLoadResistance;
    rsec["inputs"]["designRequirements"]["powerRating"] =
        d.secondaryVoltage * d.secondaryVoltage / d.secondaryLoadResistance;  // load: P = V^2/R
    rsec["inputs"]["designRequirements"]["role"] = "bleed";

    json cell; cell["name"] = "flybuck-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")});
    cell["components"] = json::array({
        comp("QS1", mosfet(req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0))),
        comp("QS2", mosfet(req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0))),
        // DS1/DS2 are anti-parallel BODY diodes across QS1/QS2 — tagged role:bodyDiode (folded into the
        // FET in the BOM), ratings mirror the switch they shadow (block Vin, carry the inductor peak).
        comp("DS1", diode(req::body_diode(ratedVds, IpkPri))),  comp("DS2", diode(req::body_diode(ratedVds, IpkPri))),
        comp("T1", xfmr), comp("Cpri", cpri),
        comp("Dsec", diode(req::diode(VrSec, IsecLoad / 0.7, maxVfSec, 0.05 * T))), comp("Csec", csec), comp("Rsec", rsec)});
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
        req::control_stage("pwmController"),
        pstage("flybuckCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("flybuckCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("flybuckCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("flybuckCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // QS1 buck switch at duty D (phase 0) — D alone sets V_pri = D·Vin, so it keeps the full duty. QS2
    // synchronous rectifier conducts the rest of the period MINUS a small dead time on each edge
    // (deadFrac), so QS1/QS2 never overlap. The two dead-time gaps are bridged by the body diodes.
    constexpr double deadFrac = 0.02;   // 200 ns at 100 kHz, like the other bridge ports
    auto stim = [&](const char* sw, double duty, double phaseDeg) {
        json st; st["stage"] = "flybuckCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("QS1", d.dutyCycle, 0.0),
        stim("QS2", (1.0 - d.dutyCycle) - 2.0 * deadFrac, (d.dutyCycle + deadFrac) * 360.0)});
    req::finalize_control_seeds(tas, Topology::ISOLATED_BUCK_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
