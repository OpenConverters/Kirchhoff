#include "Zeta.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_zeta + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kRippleRatioL1 = 0.4;   // ΔIL1 / IL1,avg
constexpr double kL2RipplePct = 0.30;    // ΔIL2 / IL2,avg
constexpr double kCcRipplePct = 0.05;    // ΔVCc / VCc
constexpr double kCoRipplePct = 0.01;    // ΔVo  / Vo
// Zeta CCM ideal-ish duty: D = (Vo+Vd) / (Vin*eff + Vo + Vd).
double duty(double vin, double vo, double vd, double eff) { return (vo + vd) / (vin * eff + vo + vd); }
} // namespace

ZetaDesign design_zeta(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ZetaDesign d{};
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

    const double iout = d.outputPower / d.outputVoltage, fsw = d.switchingFrequency;
    // MKF Zeta variant: synchronous rectifier (MOSFET replacing the catch diode D1), complementary to Q1.
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);
    // Coupled-inductor variant (ABT #89): L1 + L2 on one core (1:1) with mutual coupling k.
    d.coupledInductor = cfg::get_bool(d.config, "coupledInductor", false);
    d.couplingCoefficient = cfg::get(d.config, "couplingCoefficient", 0.999);
    if (d.coupledInductor && !(d.couplingCoefficient > 0.0 && d.couplingCoefficient < 1.0))
        throw std::invalid_argument("design_zeta: couplingCoefficient must be in (0,1), got "
                                    + std::to_string(d.couplingCoefficient));
    // Sync MOSFET has no forward drop → size duty with Vd=0 so the open-loop deck lands on target.
    d.diodeDrop = d.synchronousRectifier ? 0.0 : req::dideal_diode_drop(d.outputPower / d.outputVoltage);
    d.dutyCycle = duty(d.inputVoltage, d.outputVoltage, d.diodeDrop, d.efficiency);

    // L1 sized at the worst corner (max Vin) for its current-ripple target (MKF).
    const double dMax = duty(vinMax, d.outputVoltage, d.diodeDrop, d.efficiency);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatioL1) * iL1avg;
    d.inductanceL1 = vinMax * dMax / (dIL1 * fsw);
    // L2, Cc, Cout at the operating point (both inductors see Vin·D during ON).
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    d.inductanceL2 = d.inputVoltage * d.dutyCycle / (dIL2 * fsw);
    const double dVcc = cfg::get(d.config, "couplingCapRipple", kCcRipplePct) * d.outputVoltage;       // VCc = Vout
    d.couplingCapacitance = iout * d.dutyCycle / (dVcc * fsw);
    const double dVo = cfg::get(d.config, "outputCapRipple", kCoRipplePct) * d.outputVoltage;
    d.outputCapacitance = dIL2 / (8.0 * fsw * dVo);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    return d;
}

json build_zeta_tas(const ZetaDesign& d) {
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
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };

    // --- L1 (the MAIN coupled inductor) sourced from the SINGLE FHA solver (SPICE-validated analytical) ---
    // Worst-case corner (Vin_min) drives the switch rating; the declared nominal OP is what the TAS embeds
    // for L1. analytical_zeta returns ONE winding ("Primary" = L1); L2 (the secondary coupled inductor, whose
    // excitation is NOT one of the solver's windings) keeps its inline computation.
    namespace AN = Kirchhoff::analytical;
    const double fsw = d.switchingFrequency, iout = d.outputPower / d.outputVoltage;
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    const double vSwing = d.inputVoltage + d.outputVoltage;   // nominal operating swing (L2 excitation embed)
    const double vSwingRating = d.inputVoltageMax + d.outputVoltage;   // worst-case corner for VOLTAGE ratings
    const MAS::OperatingPoint aopWorst = AN::analytical_zeta(d.inputVoltageMin, d.outputVoltage, iout, fsw,
                                                            d.inductanceL1, d.diodeDrop, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_zeta(d.inputVoltage,    d.outputVoltage, iout, fsw,
                                                            d.inductanceL1, d.diodeDrop, d.efficiency);
    const double IL1avg = AN::winding_current(aopWorst, 0, "offset");   // L1 average (input current) at the worst corner

    // L2 (secondary coupled inductor) — inline single-winding excitation (not one of the solver's windings).
    auto inductor = [&](double L, double iAvg, double iPkPk) {
        json m; m["magnetic"] = json::object();
        const double iPk = iAvg + iPkPk / 2.0, iRms = std::sqrt(iAvg * iAvg + iPkPk * iPkPk / 12.0);
        m["inputs"] = req::magnetic_inputs(L, 0.2, {}, {"primary"}, std::nullopt, 25.0,
            {req::winding_excitation("triangular", fsw, iPk, iRms, iAvg, iPkPk, d.dutyCycle,
                                     vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing)});
        return m;
    };
    json L1; L1["magnetic"] = json::object();
    L1["inputs"] = req::magnetic_inputs(d.inductanceL1, 0.2, {}, {"primary"}, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "L1"));
    json L2 = inductor(d.inductanceL2, iout, dIL2);

    // Coupled-inductor variant (ABT #89): L1 and L2 share ONE core as a single 2-winding magnetic (1:1)
    // with mutual coupling k. Winding 0 (primary) = L1 (solver excitation); winding 1 (secondary1) = L2
    // (inline). leakageInductance = Lp·(1-k²) sets the coupling K from the coefficient in both decks.
    json L12;
    if (d.coupledInductor) {
        std::vector<json> w12 = AN::excitations_processed(aopNom);   // winding 0 = L1 (non-capturing)
        const double iL2Pk = iout + dIL2 / 2.0, iL2Rms = std::sqrt(iout * iout + dIL2 * dIL2 / 12.0);
        w12.push_back(req::winding_excitation("triangular", fsw, iL2Pk, iL2Rms, iout, dIL2, d.dutyCycle,
                                              vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing));   // winding 1 = L2
        L12["magnetic"] = json::object();
        L12["inputs"] = req::magnetic_inputs(d.inductanceL1, 0.2, /*1:1*/ {1.0}, {"primary", "secondary"},
            std::nullopt, 25.0, w12);
        L12["inputs"]["designRequirements"]["leakageInductance"] = json::array({
            json{{"nominal", d.inductanceL1 * (1.0 - d.couplingCoefficient * d.couplingCoefficient)}} });
    }
    auto l1s = [&]() { return d.coupledInductor ? pin("L12", "primary_start") : pin("L1", "primary_start"); };
    auto l1e = [&]() { return d.coupledInductor ? pin("L12", "primary_end")   : pin("L1", "primary_end");   };
    auto l2s = [&]() { return d.coupledInductor ? pin("L12", "secondary1_start") : pin("L2", "primary_start"); };
    auto l2e = [&]() { return d.coupledInductor ? pin("L12", "secondary1_end")   : pin("L2", "primary_end");   };
    auto magComps = [&](std::vector<json> rest) {   // magnetics + the rest (order is irrelevant to ngspice)
        std::vector<json> v = d.coupledInductor ? std::vector<json>{comp("L12", L12)}
                                                : std::vector<json>{comp("L1", L1), comp("L2", L2)};
        for (auto& r : rest) v.push_back(std::move(r));
        return json(v);
    };
    json cc; cc["capacitor"] = json::object();
    cc["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.couplingCapacitance;
    cc["inputs"]["designRequirements"]["ratedVoltage"] = vSwingRating / cfg::v_derate_capacitor(d.config);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage / cfg::v_derate_capacitor(d.config);
    // Switch RMS: during the on-time (duty D) the main switch carries IL1 + IL2 (≈ IL1avg + iout);
    // maxRdsOn = loss_fraction·Pout / Isw_rms² (OHMS — sibling topologies all divide by Isw_rms²).
    const double IswRms = std::sqrt(d.dutyCycle) * (IL1avg + iout);
    json mq = mosfet();
    mq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", vSwingRating / cfg::v_derate_mosfet(d.config),
                                                     iout + IL1avg,
                                                     cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms), 125.0);
    json md = diode();
    md["inputs"]["designRequirements"] = req::diode(vSwingRating / cfg::v_derate_diode(d.config), iout / 0.7,
                                                    (vSwingRating < 100.0) ? 0.6 : 1.2, 0.05 / fsw);
    // Synchronous rectifier: a low-side MOSFET Q2 (channel node_X->gnd, mirroring the catch diode) + its
    // body diode D2, driven complementary to Q1 (MKF Zeta synchronous variant).
    json syncFet = mosfet();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier",
        vSwingRating / cfg::v_derate_mosfet(d.config), iout + IL1avg,
        cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms), 125.0);
    json bodyD = diode();
    bodyD["inputs"]["designRequirements"] = req::body_diode(vSwingRating / cfg::v_derate_diode(d.config), iout / 0.7);

    // Zeta cell — high-side switch, non-inverting. D1 catch: anode at gnd, cathode at node_X.
    json cell; cell["name"] = "zeta-cell";
    if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = magComps({comp("Q1", mq), comp("Cc", cc), comp("Q2", syncFet), comp("D2", bodyD)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
            conn("node_sw",  {pin("Q1", "source"), l1s(), pin("Cc", "1")}),
            // catch node -> gnd: sync MOSFET drain at node_X, source at gnd (body-diode anode=gnd,cathode=node_X)
            conn("gnd_net",  {l1e(), pin("Q2", "source"), pin("D2", "anode"), prt("gnd")}),
            conn("node_x",   {pin("Cc", "2"), pin("Q2", "drain"), pin("D2", "cathode"), l2s()}),
            conn("vout_net", {l2e(), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = magComps({comp("Q1", mq), comp("Cc", cc), comp("D1", md)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
            // node_SW: switch -> L1 (to gnd) + coupling cap
            conn("node_sw",  {pin("Q1", "source"), l1s(), pin("Cc", "1")}),
            conn("gnd_net",  {l1e(), pin("D1", "anode"), prt("gnd")}),
            // node_X: coupling cap -> catch-diode cathode + output inductor
            conn("node_x",   {pin("Cc", "2"), pin("D1", "cathode"), l2s()}),
            conn("vout_net", {l2e(), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")})});
    }

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
        req::control_stage("pwmController"),
        pstage("zetaCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("zetaCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("zetaCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("zetaCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "zetaCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "zetaCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::ZETA_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
