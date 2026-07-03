#include "TwoSwitchForward.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_two_switch_forward + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kMaxDuty = 0.5;
constexpr double kRippleRatio = 0.4;
} // namespace

TwoSwitchForwardDesign design_two_switch_forward(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    TwoSwitchForwardDesign d{};
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

    const double iout = d.outputPower / d.outputVoltage;
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double n = vinMin * cfg::get(d.config, "maxDutyCycle", kMaxDuty) / (d.outputVoltage + d.diodeDrop);
    n = std::round(n * 100.0) / 100.0;
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * n / (d.switchingFrequency * iout));
    const double tOn = cfg::get(d.config, "maxDutyCycle", kMaxDuty) / d.switchingFrequency;
    d.outputInductance = (vinMax / n - d.diodeDrop - d.outputVoltage) * tOn / cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    d.dutyCycle = n * (d.outputVoltage + d.diodeDrop) / d.inputVoltage;
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = 100e-6;

    // Per-output legs (multi-output: N isolated secondaries, ABT #86). Each secondary sees the same primary
    // volt-seconds, so its turns ratio scales with (Vout_i+Vd_i): n_i = Vin_min·D_max/(Vout_i+Vd_i). outputs[0]
    // reproduces the scalars above byte-for-byte.
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = dr.at("outputs").size();
    for (size_t i = 0; i < nOut; ++i) {
        TwoSwitchForwardOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"));
        if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
            leg.power = tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        else
            leg.power = nominal(dr.at("outputs").at(i).at("power"));
        const double iout_i = leg.power / leg.voltage;
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;
            leg.diodeDrop = d.diodeDrop;
            leg.outputInductance = d.outputInductance;
            leg.outputCapacitance = d.outputCapacitance;
        } else {
            leg.diodeDrop = req::dideal_diode_drop(iout_i);
            double ni = vinMin * cfg::get(d.config, "maxDutyCycle", kMaxDuty) / (leg.voltage + leg.diodeDrop);
            ni = std::round(ni * 100.0) / 100.0;
            leg.turnsRatio = req::provided_turns_ratio(dr, i).value_or(ni);
            leg.outputInductance = (vinMax / leg.turnsRatio - leg.diodeDrop - leg.voltage) * tOn / ripple;
            leg.outputCapacitance = 100e-6;
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        d.outputs.push_back(leg);
    }
    return d;
}

json build_two_switch_forward_tas(const TwoSwitchForwardDesign& d) {
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
    auto dpdoc  = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };

    namespace AN = Kirchhoff::analytical;
    const double Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double T = 1.0 / fsw, Dn = d.dutyCycle, Vin = d.inputVoltage;
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = d.outputs.size();

    // Per-output vectors for the multi-secondary FHA solver (ABT #86). No demag winding: turnsRatios = [n0,n1,…].
    std::vector<double> Vouts, iouts, turnsRatios;
    std::vector<std::string> xfmrIso{"primary"};
    std::vector<bool> ceil;
    double totalOutputPower = 0.0;
    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        Vouts.push_back(leg.voltage);
        iouts.push_back(leg.power / leg.voltage);
        turnsRatios.push_back(leg.turnsRatio);
        xfmrIso.push_back(req::isolation_side(1 + i));   // sec0->secondary, sec1->tertiary, …
        ceil.push_back(true);                            // each duty-derived secondary ratio is a {maximum} ceiling
        totalOutputPower += leg.power;
    }

    // --- MAIN magnetic (the transformer T1) sourced from the SINGLE FHA solver ---
    // analytical_two_switch_forward returns "First primary" + Secondary 0..N-1 — matching T1's
    // turnsRatios = [n0,n1,…]. The output-filter inductors Lout_i are SEPARATE magnetics (inline excitation).
    // The magnetizing reset peak (clamp-diode rating) stays inline — a clean Vin*D*T/Lm.
    //
    // CORNER: n is sized for D_max = 0.5 at Vin_min (the transformer-reset limit), so evaluating the solver
    // at Vin_min sits exactly on its t1 <= T/2 guard. The declared NOMINAL operating point (D_nom < 0.5) is
    // used — also the corner the existing current ratings used.
    const MAS::OperatingPoint aopNom = AN::analytical_two_switch_forward(Vin, Vouts, iouts, turnsRatios, fsw, Lm,
                                                             d.outputs[0].outputInductance, ripple, d.outputs[0].diodeDrop);
    const double IpkPri  = AN::winding_current(aopNom, 0, "peak");   // primary peak (switch rating)
    const double IrmsPri = AN::winding_current(aopNom, 0, "rms");    // primary rms (switch RdsOn conduction)
    const double ImagPk  = Vin * Dn * T / Lm;                        // magnetizing reset peak (clamp-diode rating)

    // --- semiconductor stresses (two-switch forward) ---
    // Each primary switch blocks Vin_max (the clamp diodes hold the switch nodes to the rails during reset).
    // ratedVds = Vin_max / V_DERATE; RdsOn budget uses the TOTAL output power (all rails via the one primary).
    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * totalOutputPower / (IrmsPri * IrmsPri);
    // Clamp/reset diodes D1,D2 are REAL rectifiers: they reverse-block Vin_max and carry the reset current ImagPk.
    const double ratedVrClamp = d.inputVoltageMax / cfg::v_derate_diode(d.config);
    const double maxVfClamp    = (ratedVrClamp < 100.0) ? 0.6 : 1.2;
    const double maxTrr        = 0.05 * T;
    const auto mreq = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0);
    auto mosfetReq = [&]() { json j = mosfet(); j["inputs"]["designRequirements"] = mreq; return j; };
    auto clampDiode = [&]() { json j = dpdoc();
        j["inputs"]["designRequirements"] = req::diode(ratedVrClamp, ImagPk / 0.7, maxVfClamp, maxTrr); return j; };

    // Transformer (primary + N secondaries, no demag): turnsRatios = [n0,n1,…] -> (1+N) excitations (from the solver).
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, xfmrIso, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"), ceil);   // each duty-derived ratio {maximum} (abt #49)
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // forward cell: 2 switches (driven together) + 2 clamp diodes + transformer + per-output output stage.
    // Main rail (output 0) keeps its Cout in the output-filter stage; extra rails carry Cout_i in the cell.
    std::vector<json> comps{comp("Q1", mosfetReq()), comp("Q2", mosfetReq()),
                            comp("D1", clampDiode()), comp("D2", clampDiode()), comp("T1", xfmr)};
    std::vector<json> cports{port("vin"), port("gnd"), port("vout"), port("gate")};
    std::vector<json> conns;
    std::vector<json> gndEps{pin("Q2", "source"), pin("D1", "anode")};
    std::vector<json> priGndEps{pin("T1", "primary_end"), pin("Q2", "drain"), pin("D2", "anode")};
    // Numerical node snubber on the clamp node (pri_gnd) — REQUIRED only for the multi-output deck: the extra
    // secondaries add coupled-winding LC rings that make the ideal clamp diode D2 recovery hit "timestep too
    // small". A small node-to-gnd cap tames the dV/dt, exactly like the ACF switching-node snubber. Tagged so
    // the real-fidelity strip replaces it with the switch Coss. Single-output stays byte-identical (no snubber).
    if (nOut > 1) {
        json snb; snb["capacitor"] = json::object();
        snb["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
        snb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;
        cfg::mark_numerical_aid(snb);   // dV/dt convergence aid — explicitly tagged for the real-fidelity strip (ABT #96)
        comps.push_back(comp("Csn", snb));
        priGndEps.push_back(pin("Csn", "1"));
        gndEps.push_back(pin("Csn", "2"));
    }
    conns.push_back(conn("vin_net", {pin("Q1", "drain"), pin("D2", "cathode"), prt("vin")}));
    conns.push_back(conn("sw1_out", {pin("Q1", "source"), pin("D1", "cathode"), pin("T1", "primary_start")}));
    conns.push_back(conn("pri_gnd", priGndEps));

    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const std::string sfx  = (i == 0) ? std::string() : std::to_string(i + 1);
        const std::string swS  = "secondary" + std::to_string(1 + i) + "_start";
        const std::string swE  = "secondary" + std::to_string(1 + i) + "_end";
        const std::string dfwdN = "Dfwd" + sfx, dfwN = "Dfw" + sfx, loutN = "Lout" + sfx, coutN = "Cout" + sfx;
        const double iout_i = leg.power / leg.voltage;

        // Output filter inductor Lout_i — inline excitation.
        const double dILout  = (Vin / leg.turnsRatio - leg.diodeDrop - leg.voltage) * (Dn * T) / leg.outputInductance;
        const double IpkLout = iout_i + dILout / 2.0;
        const double IrmsLout = std::sqrt(iout_i * iout_i + dILout * dILout / 12.0);
        const double vLonF = Vin / leg.turnsRatio - leg.diodeDrop - leg.voltage, vLoff = leg.voltage;
        const double vLoutPk = std::max(std::abs(vLonF), vLoff), vLoutPkPk = std::abs(vLonF) + vLoff;
        const double vLoutRms = std::sqrt(Dn * vLonF * vLonF + (1.0 - Dn) * vLoff * vLoff);
        json lout; lout["magnetic"] = json::object();
        lout["inputs"] = req::magnetic_inputs(leg.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, iout_i, dILout, Dn,
                                    vLoutPk, vLoutRms, 0.0, vLoutPkPk)});
        // Output forward/freewheel rectifiers reverse-block Vin_max/n_i, carry ~Iout_i.
        const double ratedVrSec = (d.inputVoltageMax / leg.turnsRatio) / cfg::v_derate_diode(d.config);
        const double maxVfSec   = (ratedVrSec < 100.0) ? 0.6 : 1.2;
        json dfwd = dpdoc(); dfwd["inputs"]["designRequirements"] = req::diode(ratedVrSec, iout_i / 0.7, maxVfSec, maxTrr);
        json dfw  = dpdoc(); dfw["inputs"]["designRequirements"]  = req::diode(ratedVrSec, iout_i / 0.7, maxVfSec, maxTrr);

        comps.push_back(comp(dfwdN.c_str(), dfwd));
        comps.push_back(comp(dfwN.c_str(), dfw));
        comps.push_back(comp(loutN.c_str(), lout));

        conns.push_back(conn(("sec_in" + sfx).c_str(),   {pin("T1", swS.c_str()), pin(dfwdN.c_str(), "anode")}));
        conns.push_back(conn(("sec_rect" + sfx).c_str(), {pin(dfwdN.c_str(), "cathode"), pin(dfwN.c_str(), "cathode"),
                                                          pin(loutN.c_str(), "primary_start")}));
        if (i == 0) {
            conns.push_back(conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}));
        } else {
            json capi; capi["capacitor"] = json::object();
            capi["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.outputCapacitance;
            capi["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;
            comps.push_back(comp(coutN.c_str(), capi));
            const std::string voutP = "vout" + sfx;
            cports.push_back(port(voutP.c_str()));
            conns.push_back(conn((voutP + "_net").c_str(), {pin(loutN.c_str(), "primary_end"),
                                                            pin(coutN.c_str(), "1"), prt(voutP.c_str())}));
            gndEps.push_back(pin(coutN.c_str(), "2"));
        }
        gndEps.push_back(pin("T1", swE.c_str()));
        gndEps.push_back(pin(dfwN.c_str(), "anode"));
    }
    gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));
    conns.push_back(conn("gate_net", {pin("Q1", "gate"), pin("Q2", "gate"), prt("gate")}));

    json cell; cell["name"] = "two-switch-forward-cell";
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;

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
    dreq["outputs"] = json::array();
    json opDoc; opDoc["name"] = "full_load"; opDoc["inputVoltage"] = d.inputVoltage; opDoc["ambientTemperature"] = 25.0;
    opDoc["outputs"] = json::array();
    for (size_t i = 0; i < nOut; ++i) {
        const std::string oname = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        json o; o["name"] = oname; o["voltage"]["nominal"] = d.outputs[i].voltage; o["regulation"] = "voltage";
        dreq["outputs"].push_back(o);
        json oo; oo["name"] = oname; oo["power"] = d.outputs[i].power; opDoc["outputs"].push_back(oo);
    }
    tas["inputs"]["operatingPoints"] = json::array({opDoc});

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("forwardCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("forwardCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("forwardCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("forwardCell", "vout"), sp("filter", "in")})};
    for (size_t i = 1; i < nOut; ++i) {
        const std::string g = "Vout" + std::to_string(i + 1), pt = "vout" + std::to_string(i + 1);
        iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("forwardCell", pt.c_str())}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "forwardCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    req::finalize_control_seeds(tas, Topology::TWO_SWITCH_FORWARD_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
