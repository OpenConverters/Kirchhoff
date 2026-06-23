#include "Buck.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    if (j.contains("minimum") && j.contains("maximum"))
        return 0.5 * (j.at("minimum").get<double>() + j.at("maximum").get<double>());
    throw std::runtime_error("buck design: no nominal");
}
} // namespace

BuckDesign design_buck(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    BuckDesign d{};
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    d.diodeDrop = 0.8334;   // ideal reference (matches MKF Vd=0)
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }

    double vinMax = d.inputVoltage, vinMin = d.inputVoltage;
    {
        const json& iv = dr.at("inputVoltage");
        if (iv.is_object() && iv.contains("maximum")) vinMax = iv.at("maximum").get<double>();
        if (iv.is_object() && iv.contains("minimum")) vinMin = iv.at("minimum").get<double>();
    }
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    // Buck duty (MKF): D = (Vout+Vd) / ((Vin+Vd)*eff). Ideal -> Vout/Vin.
    const double Vo = d.outputVoltage + d.diodeDrop;
    d.dutyCycle = Vo / ((d.inputVoltage + d.diodeDrop) * d.efficiency);

    // Inductance (MKF Buck::process_design_requirements):
    //   maximumCurrentRiple = currentRippleRatio * Iout
    //   L = Vout*(Vin_max - Vout) / (maximumCurrentRiple * fsw * Vin_max)
    const double rippleRatio = 0.4;
    const double iout = d.outputPower / d.outputVoltage;
    const double maxCurrentRipple = rippleRatio * iout;
    d.inductance = d.outputVoltage * (vinMax - d.outputVoltage)
                 / (maxCurrentRipple * d.switchingFrequency * vinMax);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    // Buck output ripple ΔV = ΔIL / (8*fsw*Cout); size Cout for ~1% ripple.
    d.outputCapacitance = maxCurrentRipple / (8.0 * d.switchingFrequency * 0.01 * d.outputVoltage);
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
    const double ratedVds = d.inputVoltageMax / req::V_DERATE;          // switch + diode both block Vin
    const double maxRdsOn = 0.01 * d.outputPower / (IswRms * IswRms);
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
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltage / req::V_DERATE, IcoutRms,
        req::ESR_RIPPLE_FRACTION * d.outputVoltage / IpkL, "outputFilter");

    // --- switching cell brick (Q high-side + L + freewheeling D) ---
    json cell; cell["name"] = "buck-switching-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("Q1", mosfet), comp("L1", ind), comp("D1", diode)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), prt("vin")}),
        conn("sw_node",  {pin("Q1", "source"), pin("L1", "primary_start"), pin("D1", "cathode")}),
        conn("gnd_net",  {pin("D1", "anode"), prt("gnd")}),
        conn("vout_net", {pin("L1", "primary_end"), prt("vout")}),
        conn("gate_net", {pin("Q1", "gate"), prt("gate")})});

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
