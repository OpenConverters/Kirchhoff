#include "Buck.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
} // namespace

BuckDesign design_buck(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    BuckDesign d{};
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

    // Freewheel rectifier variant (config-gated, no schema change; default = diode for back-compat).
    // "synchronous" replaces the freewheeling diode with a low-side MOSFET (abt #67) — lower conduction
    // loss (I^2*Rds vs I*Vf) -> higher efficiency.
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);   // 1% of the period per dead band

    // Buck duty (MKF): D = (Vout+Vd) / ((Vin+Vd)*eff). Ideal -> Vout/Vin.
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    const double Vo = d.outputVoltage + d.diodeDrop;
    // With a synchronous rectifier the freewheel path is a near-ideal MOSFET (no Vf), so NO diode-drop
    // compensation: D = Vout/(Vin*eff). With a diode, compensate for its forward drop as MKF does.
    d.dutyCycle = d.synchronousRectifier
        ? d.outputVoltage / (d.inputVoltage * d.efficiency)
        : Vo / ((d.inputVoltage + d.diodeDrop) * d.efficiency);

    // Inductance (MKF Buck::process_design_requirements):
    //   maximumCurrentRiple = currentRippleRatio * Iout
    //   L = Vout*(Vin_max - Vout) / (maximumCurrentRiple * fsw * Vin_max)
    const double rippleRatio = cfg::ripple_ratio(d.config, 0.4);
    const double iout = d.outputPower / d.outputVoltage;
    const double maxCurrentRipple = rippleRatio * iout;
    d.inductance = req::provided_inductance(dr).value_or(
        d.outputVoltage * (vinMax - d.outputVoltage) / (maxCurrentRipple * d.switchingFrequency * vinMax));
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    // Buck output ripple ΔV = ΔIL / (8*fsw*Cout); size Cout for the configured ripple fraction (~1% default).
    d.outputCapacitance = maxCurrentRipple / (8.0 * d.switchingFrequency * cfg::output_ripple_fraction(d.config) * d.outputVoltage);
    return d;
}

json build_buck_tas(const BuckDesign& d) {
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

    // --- per-component stresses (worst-case corner) ---
    const double fsw = d.switchingFrequency, T = 1.0 / fsw, L = d.inductance;
    const double iout = d.outputPower / d.outputVoltage;
    const double Dmax = (d.outputVoltage) / (d.inputVoltageMin * d.efficiency);   // duty at Vin_min
    const double dIL = (d.inputVoltageMin - d.outputVoltage) * Dmax / (L * fsw);   // pk-pk inductor ripple
    const double IpkL = iout + dIL / 2.0;
    const double IrmsL = std::sqrt(iout * iout + dIL * dIL / 12.0);
    const double IswRms = std::sqrt(Dmax) * IrmsL;
    const double IdiodeAvg = iout * (1.0 - Dmax);
    const double IcoutRms = dIL / (2.0 * std::sqrt(3.0));               // triangular ripple into Cout
    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);          // switch + diode both block Vin
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms);
    const double maxVf = (ratedVds < 100.0) ? 0.6 : 1.2;

    // Inductor voltage: +(Vin-Vout) ON / -Vout OFF (volt-second balanced).
    const double vOn = d.inputVoltage - d.outputVoltage, vIndPk = std::max(vOn, d.outputVoltage);
    const double vIndRms = std::sqrt(d.dutyCycle * vOn * vOn + (1.0 - d.dutyCycle) * d.outputVoltage * d.outputVoltage);

    // --- component PEAS docs (seed + detailed requirements) ---
    json ind; ind["magnetic"] = json::object();
    ind["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"}, std::nullopt, 25.0, {
        req::winding_excitation("triangular", fsw, IpkL, IrmsL, iout, dIL, d.dutyCycle,
                                vIndPk, vIndRms, 0.0, d.inputVoltage)});
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0);
    json diode; diode["semiconductor"]["diode"] = json::object();
    diode["inputs"]["designRequirements"] = req::diode(ratedVds, IdiodeAvg / 0.7, maxVf, 0.05 * T);
    // Synchronous freewheel rectifier (abt #67): a low-side MOSFET Q2 (drain=sw_node, source=gnd) carries
    // the freewheel current through its channel (I^2*Rds, much less than a diode's I*Vf) plus its own body
    // diode D2 across the dead time. Sized like the main switch (blocks Vin, carries the inductor current).
    json syncFet; syncFet["semiconductor"]["mosfet"] = json::object();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier", ratedVds, IpkL, maxRdsOn, 125.0);
    json bodyD; bodyD["semiconductor"]["diode"] = json::object();
    bodyD["inputs"]["designRequirements"] = req::body_diode(ratedVds, IdiodeAvg / 0.7);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltage / cfg::v_derate_capacitor(d.config), IcoutRms,
        req::ESR_RIPPLE_FRACTION * d.outputVoltage / IpkL, "outputFilter");

    // --- switching cell brick (Q high-side + L + freewheel rectifier) ---
    // DEFAULT: freewheeling diode D1 (sw_node->gnd). SYNCHRONOUS: low-side MOSFET Q2 + its body diode D2.
    json cell; cell["name"] = "buck-switching-cell";
    if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = json::array({comp("Q1", mosfet), comp("L1", ind),
                                          comp("Q2", syncFet), comp("D2", bodyD)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
            conn("sw_node",  {pin("Q1", "source"), pin("L1", "primary_start"),
                              pin("Q2", "drain"), pin("D2", "cathode")}),
            conn("gnd_net",  {pin("Q2", "source"), pin("D2", "anode"), prt("gnd")}),
            conn("vout_net", {pin("L1", "primary_end"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = json::array({comp("Q1", mosfet), comp("L1", ind), comp("D1", diode)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
            conn("sw_node",  {pin("Q1", "source"), pin("L1", "primary_start"), pin("D1", "cathode")}),
            conn("gnd_net",  {pin("D1", "anode"), prt("gnd")}),
            conn("vout_net", {pin("L1", "primary_end"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")})});
    }

    // Output filter = Cout only; the LOAD is synthesized from the outputs requirement by the assembler.
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
        pstage("switchingCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("switchingCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("switchingCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("switchingCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "switchingCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        // Q2 drives complementary to Q1 with a dead band on each edge; the body diode D2 carries the
        // inductor current across the dead time (prevents shoot-through). duty = (1-D) - 2*dt, phased to
        // start after Q1 turns off. Mirrors the FSBB low-side complementary drive.
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "switchingCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::BUCK_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
