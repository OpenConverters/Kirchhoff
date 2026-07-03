#include "Sepic.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_sepic + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
// MKF Sepic.h ripple targets.
constexpr double kRippleRatioL1 = 0.4;   // ΔIL1 / IL1,avg (current_ripple_ratio)
constexpr double kL2RipplePct = 0.30;    // ΔIL2 / IL2,avg
constexpr double kCsRipplePct = 0.05;    // ΔVCs / VCs
constexpr double kCoRipplePct = 0.01;    // ΔVo  / Vo
// SEPIC CCM ideal-ish duty: D = (Vo+Vd) / (Vin*eff + Vo + Vd).
double duty(double vin, double vo, double vd, double eff) { return (vo + vd) / (vin * eff + vo + vd); }
} // namespace

SepicDesign design_sepic(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    SepicDesign d{};
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
    // MKF Sepic variant: synchronous rectifier (low-side MOSFET replacing D1), driven complementary to Q1.
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);
    // A synchronous MOSFET rectifier has no forward drop, so the duty (and hence Vout) must be sized with
    // Vd=0 — otherwise the open-loop deck over-delivers by ~Vd/Vout (Buck does the same).
    d.diodeDrop = d.synchronousRectifier ? 0.0 : req::dideal_diode_drop(d.outputPower / d.outputVoltage);
    // Operating duty (deck/stimulus) at the nominal input.
    d.dutyCycle = duty(d.inputVoltage, d.outputVoltage, d.diodeDrop, d.efficiency);

    // L1 sized at the worst corner (max Vin) for its current-ripple target (MKF process_design_requirements).
    const double dMax = duty(vinMax, d.outputVoltage, d.diodeDrop, d.efficiency);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatioL1) * iL1avg;
    d.inductanceL1 = vinMax * dMax / (dIL1 * fsw);
    // L2, Cs, Cout sized at the operating point (MKF generate_ngspice_circuit).
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    d.inductanceL2 = d.inputVoltage * d.dutyCycle / (dIL2 * fsw);
    const double dVcs = cfg::get(d.config, "couplingCapRipple", kCsRipplePct) * d.inputVoltage;        // VCs = Vin
    d.couplingCapacitance = iout * d.dutyCycle / (dVcs * fsw);
    const double dVo = cfg::get(d.config, "outputCapRipple", kCoRipplePct) * d.outputVoltage;
    d.outputCapacitance = dIL2 / (8.0 * fsw * dVo);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    return d;
}

json build_sepic_tas(const SepicDesign& d) {
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
    // for L1. analytical_sepic returns ONE winding ("Primary" = L1); L2 (the secondary coupled inductor,
    // whose excitation is NOT one of the solver's windings) keeps its inline computation.
    namespace AN = Kirchhoff::analytical;
    const double fsw = d.switchingFrequency, iout = d.outputPower / d.outputVoltage;
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    const double vSwing = d.inputVoltage + d.outputVoltage;   // nominal operating swing (L2 excitation embed)
    const double vSwingRating = d.inputVoltageMax + d.outputVoltage;   // worst-case corner for VOLTAGE ratings
    const MAS::OperatingPoint aopWorst = AN::analytical_sepic(d.inputVoltageMin, d.outputVoltage, iout, fsw,
                                                             d.inductanceL1, d.diodeDrop, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_sepic(d.inputVoltage,    d.outputVoltage, iout, fsw,
                                                             d.inductanceL1, d.diodeDrop, d.efficiency);
    const double IL1avg = AN::winding_current(aopWorst, 0, "offset");   // L1 average (input current) at the worst corner

    // L2 (secondary coupled inductor) — inline single-winding excitation (not one of the solver's windings).
    auto inductor = [&](double L, double iAvg, double iPkPk) {
        json m; m["magnetic"] = json::object();
        const double iPk = iAvg + iPkPk / 2.0, iRms = std::sqrt(iAvg * iAvg + iPkPk * iPkPk / 12.0);
        m["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"}, std::nullopt, 25.0,
            {req::winding_excitation("triangular", fsw, iPk, iRms, iAvg, iPkPk, d.dutyCycle,
                                     vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing)});
        return m;
    };
    json L1; L1["magnetic"] = json::object();
    L1["inputs"] = req::magnetic_inputs(d.inductanceL1, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, AN::excitations_processed(aopNom, "L1"));
    json L2 = inductor(d.inductanceL2, iout, dIL2);
    json cs; cs["capacitor"] = json::object();
    cs["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.couplingCapacitance;
    cs["inputs"]["designRequirements"]["ratedVoltage"] = vSwingRating / cfg::v_derate_capacitor(d.config);
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
    // Synchronous rectifier: a MOSFET Q2 (channel nodeB->vout) + its body diode D2, driven complementary
    // to Q1. Carries the rectifier current through I^2*Rds instead of a diode's I*Vf (MKF Sepic V4).
    json syncFet = mosfet();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier",
        vSwingRating / cfg::v_derate_mosfet(d.config), iout + IL1avg,
        cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms), 125.0);
    json bodyD = diode();
    bodyD["inputs"]["designRequirements"] = req::body_diode(vSwingRating / cfg::v_derate_diode(d.config), iout / 0.7);

    json cell; cell["name"] = "sepic-cell";
    if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = json::array({comp("L1", L1), comp("Q1", mq), comp("Cs", cs),
                                          comp("L2", L2), comp("Q2", syncFet), comp("D2", bodyD)});
        cell["connections"] = json::array({
            conn("vin_net", {pin("L1", "primary_start"), prt("vin")}),
            conn("nodeA",   {pin("L1", "primary_end"), pin("Q1", "drain"), pin("Cs", "1")}),
            // node B: coupling cap -> L2 + sync rectifier source (+ body-diode anode)
            conn("nodeB",   {pin("Cs", "2"), pin("L2", "primary_end"), pin("Q2", "source"), pin("D2", "anode")}),
            conn("gnd_net", {pin("Q1", "source"), pin("L2", "primary_start"), prt("gnd")}),
            conn("vout_net",{pin("Q2", "drain"), pin("D2", "cathode"), prt("vout")}),
            conn("gate_net",{pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = json::array({comp("L1", L1), comp("Q1", mq), comp("Cs", cs),
                                          comp("L2", L2), comp("D1", md)});
        cell["connections"] = json::array({
            conn("vin_net", {pin("L1", "primary_start"), prt("vin")}),
            // node A: L1 -> switch + coupling cap
            conn("nodeA",   {pin("L1", "primary_end"), pin("Q1", "drain"), pin("Cs", "1")}),
            // node B: coupling cap -> L2 + rectifier
            conn("nodeB",   {pin("Cs", "2"), pin("L2", "primary_end"), pin("D1", "anode")}),
            conn("gnd_net", {pin("Q1", "source"), pin("L2", "primary_start"), prt("gnd")}),
            conn("vout_net",{pin("D1", "cathode"), prt("vout")}),
            conn("gate_net",{pin("Q1", "gate"), prt("gate")})});
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
        pstage("sepicCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("sepicCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("sepicCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("sepicCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "sepicCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        // Q2 (sync rectifier) drives complementary to Q1 with a dead band on each edge; body diode D2 carries
        // the current across the dead time. duty = (1-D) - 2*dt, phased to start after Q1 turns off.
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "sepicCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::SEPIC_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
