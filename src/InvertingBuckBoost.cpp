#include "InvertingBuckBoost.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single waveform source: analytical_inverting_buck_boost + timing
#include "KirchhoffConfig.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
// DCM is designed with this margin below the critical inductance at Vin,min, full load: at L = Lcrit the
// valley sits exactly on zero and a few percent of tolerance tips the converter back into CCM.
constexpr double kDcmMargin = 0.8;
} // namespace

InvertingBuckBoostDesign design_inverting_buck_boost(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    InvertingBuckBoostDesign d{};
    d.config = cfg::object_of(tasInputs);
    if (dr.at("outputs").size() > 1)
        throw std::invalid_argument("inverting_buck_boost design: this topology has a single output rail, but "
                                    + std::to_string(dr.at("outputs").size()) + " outputs were requested");
    // The rail is negative; a spec may state it signed or as a magnitude (as for Ćuk).
    d.outputVoltageMag = std::fabs(nominal(dr.at("outputs").at(0).at("voltage")));
    if (!(d.outputVoltageMag > 0))
        throw std::invalid_argument("inverting_buck_boost design: output voltage must be non-zero");
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
    d.inputVoltageMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);

    const std::string mode = cfg::get_str(d.config, "mode", "ccm");
    if (mode != "ccm" && mode != "dcm")
        throw std::invalid_argument("design_inverting_buck_boost: config.mode must be 'ccm' or 'dcm', got '" + mode + "'");
    d.dcm = (mode == "dcm");
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    if (d.dcm && d.synchronousRectifier)
        // A complementary-driven synchronous FET conducts negative current once the inductor runs dry, which
        // forces CCM: the two requests contradict each other.
        throw std::invalid_argument("design_inverting_buck_boost: mode 'dcm' cannot be combined with a "
                                    "complementary synchronous rectifier (it would force CCM)");
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);

    const double iout = d.outputPower / d.outputVoltageMag, fsw = d.switchingFrequency;
    d.diodeDrop = d.synchronousRectifier ? 0.0 : req::rectifier_drop(d.config, iout);
    const double vOff = d.outputVoltageMag + d.diodeDrop;
    auto ccmDuty = [&](double vin) { return vOff / (vin * d.efficiency + vOff); };

    // Critical inductance at a given Vin: the CCM valley iL_avg − ΔiL/2 reaches zero.
    //   Lcrit = Vin·D·(1−D)/(2·Iout·fsw); smallest at Vin,min, which is where DCM must still hold.
    auto lcrit = [&](double vin) { const double D = ccmDuty(vin); return vin * D * (1.0 - D) / (2.0 * iout * fsw); };
    d.criticalInductance = lcrit(d.inputVoltageMin);

    if (d.dcm) {
        d.inductance = req::provided_inductance(dr).value_or(kDcmMargin * d.criticalInductance);
        if (d.inductance >= d.criticalInductance)
            throw std::invalid_argument("design_inverting_buck_boost: mode 'dcm' with the pinned inductance "
                + std::to_string(d.inductance) + " H, which is at or above the critical inductance "
                + std::to_string(d.criticalInductance) + " H at Vin,min — the converter would run in CCM");
    } else {
        // Ripple sized at Vin,max, where ΔiL = Vin·D/(L·fsw) is largest, against the inductor average there.
        const double dMaxVin = ccmDuty(d.inputVoltageMax);
        const double iAvgAtVinMax = iout / (1.0 - dMaxVin);
        const double ripple = cfg::max_current_ripple(d.config, cfg::ripple_ratio(d.config, 0.4), iAvgAtVinMax,
                                                      iAvgAtVinMax, "design_inverting_buck_boost",
                                                      d.config.contains("rippleRatio"));
        d.inductance = req::provided_inductance(dr).value_or(
            d.inputVoltageMax * dMaxVin / (ripple * fsw));
    }
    d.dutyCycle = analytical::inverting_buck_boost_timing(d.inputVoltage, d.outputVoltageMag, iout, fsw,
                                                          d.inductance, d.diodeDrop, d.efficiency).dutyCycle;
    d.loadResistance = d.outputVoltageMag * d.outputVoltageMag / d.outputPower;
    // Cout alone carries the load while Q1 conducts: ΔVo = Iout·tOn/Cout. Sized at the longest on-time
    // (Vin,min) for the configured ripple fraction.
    const double tOnMax = analytical::inverting_buck_boost_timing(d.inputVoltageMin, d.outputVoltageMag, iout, fsw,
                                                                  d.inductance, d.diodeDrop, d.efficiency).tOn;
    d.outputCapacitance = iout * tOnMax / (cfg::output_ripple_fraction(d.config) * d.outputVoltageMag);
    return d;
}

json build_inverting_buck_boost_tas(const InvertingBuckBoostDesign& d) {
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

    // Stresses from the single waveform source: the WORST corner (Vin,min: longest on-time, highest inductor
    // average) rates the parts; the DECLARED operating point is what the magnetic is designed against.
    namespace AN = Kirchhoff::analytical;
    const double fsw = d.switchingFrequency, iout = d.outputPower / d.outputVoltageMag;
    const double maxDuty = cfg::get(d.config, "maximumDutyCycle", 0.95);
    const MAS::OperatingPoint aopWorst = AN::analytical_inverting_buck_boost(
        d.inputVoltageMin, d.outputVoltageMag, iout, fsw, d.inductance, d.diodeDrop, d.efficiency, maxDuty);
    const MAS::OperatingPoint aopNom = AN::analytical_inverting_buck_boost(
        d.inputVoltage, d.outputVoltageMag, iout, fsw, d.inductance, d.diodeDrop, d.efficiency, maxDuty);
    const double IpkL   = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsL  = AN::winding_current(aopWorst, 0, "rms");
    const double Dmax   = AN::inverting_buck_boost_timing(d.inputVoltageMin, d.outputVoltageMag, iout, fsw,
                                                          d.inductance, d.diodeDrop, d.efficiency).dutyCycle;
    cfg::check_maximum_switch_current(d.config, IpkL, "build_inverting_buck_boost_tas");
    const double IswRms = std::sqrt(Dmax) * IrmsL;
    // Q1 and the rectifier both block the full swing Vin + |Vo| (the sw node moves between +Vin and −|Vo|).
    const double vSwing = d.inputVoltageMax + d.outputVoltageMag + d.diodeDrop;
    cfg::check_maximum_drain_source_voltage(d.config, vSwing, "build_inverting_buck_boost_tas");
    const double ratedVds = vSwing / cfg::v_derate_mosfet(d.config);
    const double ratedVr  = vSwing / cfg::v_derate_diode(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms);
    // Cout: pulsed charge — the rectifier current (iL during the off interval) minus the load.
    const double IcoutRms = std::sqrt(std::max(0.0, IrmsL * IrmsL * (1.0 - Dmax) - iout * iout));

    json ind; ind["magnetic"] = json::object();
    ind["inputs"] = req::magnetic_inputs(d.inductance, 0.2, {}, {"primary"}, std::nullopt, 25.0,
                                         AN::excitations_processed(aopNom, "L1"));
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0);
    json diode; diode["semiconductor"]["diode"] = json::object();
    diode["inputs"]["designRequirements"] = req::diode(ratedVr, iout / 0.7, (ratedVr < 100.0) ? 0.6 : 1.2, 0.05 / fsw);
    json syncFet; syncFet["semiconductor"]["mosfet"] = json::object();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier", ratedVds, IpkL, maxRdsOn, 125.0);
    json bodyD; bodyD["semiconductor"]["diode"] = json::object();
    bodyD["inputs"]["designRequirements"] = req::body_diode(ratedVr, iout / 0.7);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltageMag / cfg::v_derate_capacitor(d.config), IcoutRms,
        cfg::get(d.config, "esrRippleFraction", req::ESR_RIPPLE_FRACTION) * d.outputVoltageMag / IpkL,
        "outputFilter");

    // Switching cell: Q1 high side (Vin -> sw), L1 sw -> gnd, rectifier from the negative rail into sw.
    json cell; cell["name"] = "inverting-buck-boost-cell";
    if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = json::array({comp("Q1", mosfet), comp("L1", ind), comp("Q2", syncFet), comp("D2", bodyD)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
            // Q2's body diode must point the way D1 does (anode at the negative rail): source at vout.
            conn("sw_node",  {pin("Q1", "source"), pin("L1", "primary_start"), pin("Q2", "drain"), pin("D2", "cathode")}),
            conn("gnd_net",  {pin("L1", "primary_end"), prt("gnd")}),
            conn("vout_net", {pin("Q2", "source"), pin("D2", "anode"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = json::array({comp("Q1", mosfet), comp("L1", ind), comp("D1", diode)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
            conn("sw_node",  {pin("Q1", "source"), pin("L1", "primary_start"), pin("D1", "cathode")}),
            conn("gnd_net",  {pin("L1", "primary_end"), prt("gnd")}),
            conn("vout_net", {pin("D1", "anode"), prt("vout")}),
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
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = -d.outputVoltageMag; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("switchingCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin",  "externalPort", "input",  {sp("switchingCell", "vin")}),
        isc("GND",  "externalPort", "input",  {sp("switchingCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("switchingCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "switchingCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "switchingCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::INVERTING_BUCK_BOOST_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
