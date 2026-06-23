#include "Vienna.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    if (j.contains("minimum") && j.contains("maximum"))
        return 0.5 * (j.at("minimum").get<double>() + j.at("maximum").get<double>());
    throw std::runtime_error("vienna design: no nominal");
}
constexpr double kSwitchDuty      = 0.5;
constexpr double kBusCapacitance  = 470e-6;
} // namespace

ViennaDesign design_vienna(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ViennaDesign d{};
    d.inputVoltageRms    = nominal(dr.at("inputVoltage"));
    d.lineFrequency      = nominal(dr.at("lineFrequency"));
    d.outputVoltage      = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency         = dr.value("efficiency", 1.0);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
        d.outputPower = tasInputs.at("operatingPoints").at(0).at("outputs").at(0).at("power").get<double>();
    else
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));

    d.switchDuty = kSwitchDuty;
    // Per-phase boost inductor: ~20 % ripple at the line peak, at the target switching frequency.
    const double vpeak = d.inputVoltageRms * std::sqrt(2.0);
    const double iPeak = (2.0 / 3.0) * d.outputPower / vpeak;   // per-phase peak (3φ, unity PF)
    const double dIL = 0.2 * std::max(iPeak, 1e-3);
    d.boostInductance = (d.outputVoltage * 0.5) / (4.0 * d.switchingFrequency * dIL);
    d.busCapacitance = kBusCapacitance;
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    return d;
}

json build_vienna_tas(const ViennaDesign& d) {
    auto port = [](const std::string& n) { json p; p["name"] = n; return p; };
    auto pin  = [](const std::string& c, const std::string& p) { json e; e["component"]=c; e["pin"]=p; return e; };
    auto prt  = [](const std::string& p) { json e; e["port"]=p; return e; };
    auto conn = [](const std::string& name, std::vector<json> eps) { json c; c["name"]=name; c["endpoints"]=eps; return c; };
    auto comp = [](const std::string& name, json data) { json c; c["name"]=name; c["data"]=data; return c; };
    auto bind = [](const char* p, const char* t) { json b; b["port"]=p; b["type"]=t; return b; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"]=st; e["port"]=po; return e; };
    auto isc = [](const std::string& name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"]=name; c["kind"]=kind; if (dir[0]) c["direction"]=dir; c["endpoints"]=eps; return c; };
    auto mosfet = []() { json j; j["semiconductor"]["mosfet"] = json::object(); return j; };
    auto diode  = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L) { json j; j["magnetic"] = json::object();
        j["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = L;
        j["inputs"]["designRequirements"]["turnsRatios"] = json::array(); return j; };
    auto resBrick = [&](double r) { json j; j["resistor"] = json::object();
        j["inputs"]["designRequirements"]["deviceType"] = "resistor";
        j["inputs"]["designRequirements"]["resistance"]["nominal"] = r; return j; };

    json pcell; pcell["name"] = "vienna-power";
    json ports = json::array({port("a"), port("b"), port("c"), port("gnd"), port("busP"), port("busN")});
    json comps = json::array({
        comp("Cp", capBrick(d.busCapacitance, d.outputVoltage)),   // +bus -> midpoint(gnd)
        comp("Cn", capBrick(d.busCapacitance, d.outputVoltage)),   // midpoint(gnd) -> -bus
        comp("Rload", resBrick(d.loadResistance))});               // across the full bus
    json conns = json::array({
        conn("busP_net", {pin("Cp","1"), prt("busP")}),
        conn("busN_net", {pin("Cn","2"), prt("busN")}),
        conn("mid",      {pin("Cp","2"), pin("Cn","1"), prt("gnd")})});
    // Rload across busP..busN
    conns.push_back(conn("rload_p", {pin("Rload","1"), prt("busP")}));
    conns.push_back(conn("rload_n", {pin("Rload","2"), prt("busN")}));

    // Per-phase Vienna leg: phase -> L -> X ; D+ (X->busP), D- (busN->X) ; bidirectional switch X<->gnd.
    const char* ph[3] = {"a", "b", "c"};
    std::vector<std::string> gateNames;
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        const std::string L="L"+p, SW="SW"+p, DP="Dp"+p, DN="Dn"+p, X="x"+p, G="g"+p;
        comps.push_back(comp(L, indBrick(d.boostInductance)));
        comps.push_back(comp(SW, mosfet()));
        comps.push_back(comp(DP, diode()));
        comps.push_back(comp(DN, diode()));
        conns.push_back(conn("phin_"+p, {pin(L,"primary_start"), prt(p)}));        // phase source -> L
        conns.push_back(conn(X, {pin(L,"primary_end"), pin(SW,"drain"),
                                 pin(DP,"anode"), pin(DN,"cathode")}));            // node X
        conns.push_back(conn("swret_"+p, {pin(SW,"source"), prt("gnd")}));         // switch -> midpoint
        conns.push_back(conn("dp_"+p, {pin(DP,"cathode"), prt("busP")}));          // X -> +bus
        conns.push_back(conn("dn_"+p, {pin(DN,"anode"), prt("busN")}));            // -bus -> X
        conns.push_back(conn(G+"_net", {pin(SW,"gate"), prt(G)}));
        gateNames.push_back(G);
    }
    // expose the three gate ports
    ports.push_back(port("ga")); ports.push_back(port("gb")); ports.push_back(port("gc"));
    pcell["ports"] = ports; pcell["components"] = comps; pcell["connections"] = conns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acThreePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=d.outputVoltage; o["regulation"]="voltage";
      dreq["outputs"]=json::array({o}); }
    { json op; op["name"]="full_load"; op["inputVoltage"]=d.inputVoltageRms; op["ambientTemperature"]=25.0;
      json o; o["name"]="out"; o["power"]=d.outputPower; op["outputs"]=json::array({o});
      tas["inputs"]["operatingPoints"]=json::array({op}); }

    tas["topology"]["stages"] = json::array({ [&]{ json s; s["name"]="viennaPower"; s["role"]="switchingCell";
        s["circuit"]=pcell; s["inputPort"]=bind("a","acInput"); s["outputPort"]=bind("busP","dcOutput"); return s; }() });
    // a/b/c are the three input phases; gnd is the source neutral = split-bus midpoint.
    tas["topology"]["interStageConnections"] = json::array({
        isc("PhaseA","externalPort","input",{sp("viennaPower","a")}),
        isc("PhaseB","externalPort","input",{sp("viennaPower","b")}),
        isc("PhaseC","externalPort","input",{sp("viennaPower","c")}),
        isc("GND","externalPort","input",{sp("viennaPower","gnd")}),
        // expose the split-bus rails as named nodes (no auto-load; the load is in the cell)
        isc("busP","wire","",{sp("viennaPower","busP")}),
        isc("busN","wire","",{sp("viennaPower","busN")})});

    json an; an["type"]="transient"; an["stopTime"]=0.04; an["maximumTimeStep"]=5e-7;
    tas["simulation"]["analyses"] = json::array({an});
    // Open-loop: PWM the three switches at the target frequency (current shaping / regulation is the
    // documented refinement). Precharge each half-bus to ±Vdc/2 about the grounded midpoint.
    json stim = json::array();
    for (int i = 0; i < 3; ++i) {
        json st; st["stage"]="viennaPower"; st["component"]=std::string("SW")+ph[i]; st["signal"]="gate";
        st["waveform"]["type"]="pwm"; st["waveform"]["frequency"]=d.switchingFrequency;
        st["waveform"]["dutyCycle"]=d.switchDuty; st["waveform"]["phaseDeg"]=0.0;
        stim.push_back(st);
    }
    tas["simulation"]["stimulus"] = stim;
    json ics = json::array();
    { json ic; ic["node"]="busP"; ic["voltage"]= 0.5*d.outputVoltage; ics.push_back(ic); }
    { json ic; ic["node"]="busN"; ic["voltage"]=-0.5*d.outputVoltage; ics.push_back(ic); }
    tas["simulation"]["initialConditions"] = ics;
    return tas;
}

} // namespace Kirchhoff
