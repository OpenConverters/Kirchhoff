#include "Boost.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_boost + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"
#include <vector>
#include <stdexcept>
#include <string>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
} // namespace

BoostDesign design_boost(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    BoostDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    // Ideal design assumes a lossless diode (Vd=0), matching MKF's ideal reference
    // (Boost::calculate_duty_cycle with diodeVoltageDrop=0) -> D = 1 - Vin/Vout.
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }

    // Output rectifier variant (config-gated, no schema change; default = diode for back-compat).
    // "synchronous" replaces the high-side output diode with a high-side MOSFET (abt #67) — lower
    // conduction loss (I^2*Rds vs I*Vf) -> higher efficiency.
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);   // 1% of the period per dead band

    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    const double Vo = d.outputVoltage + d.diodeDrop;
    // With a synchronous rectifier the output path is a near-ideal MOSFET (no Vf), so NO diode-drop
    // compensation: D = 1 - Vin*eff/Vout. With a diode, compensate for its forward drop as MKF does.
    d.dutyCycle = d.synchronousRectifier
        ? 1.0 - d.inputVoltage * d.efficiency / d.outputVoltage
        : 1.0 - d.inputVoltage * d.efficiency / Vo;  // MKF Boost::calculate_duty_cycle
    // A boost can only STEP UP: it needs Vout > Vin/efficiency, otherwise the duty goes <= 0 and the
    // sized output capacitance (iout*D/(fsw*ripple*Vo)) flips NEGATIVE (abt #68). Fail loud rather than
    // emit a degraded/negative design (CLAUDE.md no-fallback rule) — a mis-specced step-down boost is a
    // spec error the caller must fix, not silently clamp.
    if (d.dutyCycle <= 0.0)
        throw std::runtime_error(
            "Kirchhoff boost: requires Vout > Vin/efficiency (boost cannot step down); got Vin="
            + std::to_string(d.inputVoltage) + " V, Vout=" + std::to_string(d.outputVoltage)
            + " V, efficiency=" + std::to_string(d.efficiency) + " -> duty <= 0");

    // Inductance — faithful port of MKF Boost::process_design_requirements():
    //   maximumCurrentRiple = currentRippleRatio · Iout
    //   L = Vin_max·(Vout − Vin_max) / (maximumCurrentRiple · fsw · Vout)
    // sized at the maximum input voltage corner.
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;
    const double rippleRatio = cfg::ripple_ratio(d.config, 0.4);
    const double iout = d.outputPower / d.outputVoltage;
    const double maxCurrentRipple = rippleRatio * iout;
    d.inductance = req::provided_inductance(dr).value_or(
        vinMax * (d.outputVoltage - vinMax) / (maxCurrentRipple * d.switchingFrequency * d.outputVoltage));
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = iout * d.dutyCycle / (d.switchingFrequency * cfg::output_ripple_fraction(d.config) * d.outputVoltage);
    return d;
}

json build_boost_tas(const BoostDesign& d) {
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };

    // --- per-component stresses from the SINGLE FHA source (the SPICE-validated analytical solver) ---
    // Worst-case corner (Vin_min) drives the ratings; the declared nominal OP is what the TAS embeds.
    namespace AN = Kirchhoff::analytical;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw, L_H = d.inductance;
    const double Iout = d.outputPower / d.outputVoltage;
    const MAS::OperatingPoint aopWorst = AN::analytical_boost(d.inputVoltageMin, d.outputVoltage, Iout, fsw,
                                                              L_H, 0.0, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_boost(d.inputVoltage,    d.outputVoltage, Iout, fsw,
                                                              L_H, 0.0, d.efficiency);
    const double Dmax  = AN::winding_current(aopWorst, 0, "dutyCycle");
    const double IpkL  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsL = AN::winding_current(aopWorst, 0, "rms");
    const double IswRms   = std::sqrt(Dmax) * IrmsL;                       // switch conducts during D
    const double IdiodeRms = std::sqrt(1.0 - Dmax) * IrmsL;               // diode conducts during 1-D
    const double IcoutRms = std::sqrt(std::max(0.0, IdiodeRms * IdiodeRms - Iout * Iout));
    // VOLTAGES at Vin_max: switch and diode both block Vout.
    const double ratedVds = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double ratedVr  = d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;

    json ind; ind["magnetic"] = json::object();
    ind["inputs"] = req::magnetic_inputs(d.inductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, AN::excitations_processed(aopNom, "L1"));
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0);
    json diode;  diode["semiconductor"]["diode"] = json::object();
    diode["inputs"]["designRequirements"] = req::diode(ratedVr, Iout / 0.7, maxVf, 0.05 * T);
    // Synchronous output rectifier (abt #67): a HIGH-SIDE MOSFET Q2 (drain=vout, source=sw_node) carries
    // the off-time output current through its channel (I^2*Rds, much less than a diode's I*Vf) plus its own
    // body diode D2 across the dead time. Sized like the main switch (blocks Vout, carries the inductor pk).
    json syncFet; syncFet["semiconductor"]["mosfet"] = json::object();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier", ratedVds, IpkL, maxRdsOn, 125.0);
    json bodyD; bodyD["semiconductor"]["diode"] = json::object();
    bodyD["inputs"]["designRequirements"] = req::body_diode(ratedVds, Iout / 0.7);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltage / cfg::v_derate_capacitor(d.config), IcoutRms,
        req::ESR_RIPPLE_FRACTION * d.outputVoltage / IpkL, "outputFilter");

    // The boost power stage is ONE functional block (the switching cell): inductor + switch + diode.
    // A canonical TAS power stage is a series two-port (dcBus in -> pulsatingDc out); the shunt switch
    // and the inductor are internal to it, not separate stages. (sw_node stays brick-internal.)
    // DEFAULT: high-side output diode D1 (sw_node->vout). SYNCHRONOUS: high-side MOSFET Q2 (drain=vout,
    // source=sw_node) + its body diode D2 (anode=sw_node, cathode=vout, same orientation as D1).
    json cell; cell["name"] = "boost-switching-cell";
    if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = json::array({comp("L1", ind), comp("Q1", mosfet),
                                          comp("Q2", syncFet), comp("D2", bodyD)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("L1", "primary_start"), prt("vin")}),
            conn("sw_node",  {pin("L1", "primary_end"), pin("Q1", "drain"),
                              pin("Q2", "source"), pin("D2", "anode")}),
            conn("gnd_net",  {pin("Q1", "source"), prt("gnd")}),
            conn("vout_net", {pin("Q2", "drain"), pin("D2", "cathode"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = json::array({comp("L1", ind), comp("Q1", mosfet), comp("D1", diode)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("L1", "primary_start"), prt("vin")}),
            conn("sw_node",  {pin("L1", "primary_end"), pin("Q1", "drain"), pin("D1", "anode")}),
            conn("gnd_net",  {pin("Q1", "source"), prt("gnd")}),
            conn("vout_net", {pin("D1", "cathode"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")})});
    }

    // Output filter = Cout only; the LOAD is synthesized from the outputs requirement by the
    // assembler (it is a boundary condition, not a converter stage — the dual of the input source).
    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };

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
        pstage("switchingCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput")),
        req::control_stage("pwmController")});   // sourced control IC (BOM); skipped in the power deck
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("switchingCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("switchingCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("switchingCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "switchingCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        // Q2 drives complementary to the low-side main switch Q1 with a dead band on each edge; the body
        // diode D2 carries the inductor current across the dead time (prevents shoot-through vout->gnd).
        // duty = (1-D) - 2*dt, phased to start after Q1 turns off. The gate is ground-referenced like Q1
        // (the deck models the switch as a gate-to-ground VC-switch, so the floating source is transparent).
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "switchingCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::BOOST_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
