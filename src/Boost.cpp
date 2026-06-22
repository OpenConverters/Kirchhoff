#include "Boost.hpp"
#include "ComponentRequirements.hpp"
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    if (j.contains("minimum") && j.contains("maximum"))
        return 0.5 * (j.at("minimum").get<double>() + j.at("maximum").get<double>());
    throw std::runtime_error("boost design: no nominal");
}
} // namespace

BoostDesign design_boost(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    BoostDesign d{};
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    // Ideal design assumes a lossless diode (Vd=0), matching MKF's ideal reference
    // (Boost::calculate_duty_cycle with diodeVoltageDrop=0) -> D = 1 - Vin/Vout.
    d.diodeDrop = 0.0;
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }

    const double Vo = d.outputVoltage + d.diodeDrop;
    d.dutyCycle = 1.0 - d.inputVoltage * d.efficiency / Vo;  // MKF Boost::calculate_duty_cycle

    // Inductance — faithful port of MKF Boost::process_design_requirements():
    //   maximumCurrentRiple = currentRippleRatio · Iout
    //   L = Vin_max·(Vout − Vin_max) / (maximumCurrentRiple · fsw · Vout)
    // sized at the maximum input voltage corner.
    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;
    const double rippleRatio = 0.4;
    const double iout = d.outputPower / d.outputVoltage;
    const double maxCurrentRipple = rippleRatio * iout;
    d.inductance = vinMax * (d.outputVoltage - vinMax)
                 / (maxCurrentRipple * d.switchingFrequency * d.outputVoltage);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = iout * d.dutyCycle / (d.switchingFrequency * 0.01 * d.outputVoltage);
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

    // --- per-component electrical stresses (worst-case corner) -> detailed, sourceable requirements ---
    const double fsw = d.switchingFrequency, T = 1.0 / fsw, L_H = d.inductance;
    const double Pin = d.outputPower / d.efficiency;
    const double Iout = d.outputPower / d.outputVoltage;
    // CURRENTS at Vin_min (max duty / max inductor current corner).
    const double Dmax = 1.0 - d.inputVoltageMin * d.efficiency / d.outputVoltage;
    const double ILavg = Pin / d.inputVoltageMin;
    const double dIL   = d.inputVoltageMin * Dmax * T / L_H;     // inductor pk-pk ripple
    const double IpkL  = ILavg + dIL / 2.0;
    const double IrmsL = std::sqrt(ILavg * ILavg + dIL * dIL / 12.0);
    const double IswRms   = std::sqrt(Dmax) * IrmsL;                       // switch conducts during D
    const double IdiodeRms = std::sqrt(1.0 - Dmax) * IrmsL;               // diode conducts during 1-D
    const double IcoutRms = std::sqrt(std::max(0.0, IdiodeRms * IdiodeRms - Iout * Iout));
    // VOLTAGES at Vin_max: switch and diode both block Vout.
    const double ratedVds = d.outputVoltage / req::V_DERATE;
    const double ratedVr  = d.outputVoltage / req::V_DERATE;
    const double maxRdsOn = 0.01 * d.outputPower / (IswRms * IswRms);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;

    // component PEAS docs: discriminator + detailed inputs.designRequirements
    // Inductor voltage: +Vin (ON) / -(Vout-Vin) (OFF), volt-second balanced; evaluated at Vin_min.
    const double Vin = d.inputVoltageMin, Voff = d.outputVoltage - Vin;
    const double vIndPk = std::max(Vin, Voff), vIndPkPk = d.outputVoltage;
    const double vIndRms = std::sqrt(Dmax * Vin * Vin + (1.0 - Dmax) * Voff * Voff);
    json ind; ind["magnetic"] = json::object();
    ind["inputs"] = req::magnetic_inputs(d.inductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkL, IrmsL, ILavg, dIL, Dmax,
                                    vIndPk, vIndRms, 0.0, vIndPkPk)});
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0);
    json diode;  diode["semiconductor"]["diode"] = json::object();
    diode["inputs"]["designRequirements"] = req::diode(ratedVr, Iout / 0.7, maxVf, 0.05 * T);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltage / req::V_DERATE, IcoutRms,
        req::ESR_RIPPLE_FRACTION * d.outputVoltage / IpkL, "outputFilter");

    // The boost power stage is ONE functional block (the switching cell): inductor + switch + diode.
    // A canonical TAS power stage is a series two-port (dcBus in -> pulsatingDc out); the shunt switch
    // and the inductor are internal to it, not separate stages. (sw_node stays brick-internal.)
    json cell; cell["name"] = "boost-switching-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("L1", ind), comp("Q1", mosfet), comp("D1", diode)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("L1", "primary_start"), prt("vin")}),
        conn("sw_node",  {pin("L1", "primary_end"), pin("Q1", "drain"), pin("D1", "anode")}),
        conn("gnd_net",  {pin("Q1", "source"), prt("gnd")}),
        conn("vout_net", {pin("D1", "cathode"), prt("vout")}),
        conn("gate_net", {pin("Q1", "gate"), prt("gate")})});

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
    return tas;
}

} // namespace Kirchhoff
