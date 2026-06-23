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
constexpr double kBusCapacitance  = 470e-6;
constexpr double kSenseResistance = 0.1;
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

    const double pin   = d.outputPower / std::max(d.efficiency, 1e-6);
    const double vpeak = d.inputVoltageRms * std::sqrt(2.0);
    const double iPeak = pin * std::sqrt(2.0) / (3.0 * d.inputVoltageRms);  // per-phase peak (3φ, unity PF)
    const double dIL   = 0.3 * std::max(iPeak, 1e-3);
    d.senseResistance = kSenseResistance;
    // i_ref voltage = kref·V(phase) must equal iL·Rsense for iL = V(phase)/rEmul; per phase Pin/3 into
    // rEmul = (Vrms²)/(Pin/3): kref = Rsense·Pin/(3·Vrms²).
    d.referenceGain = d.senseResistance * pin / (3.0 * d.inputVoltageRms * d.inputVoltageRms);
    d.boostInductance = (d.outputVoltage * 0.5) / (4.0 * d.switchingFrequency * dIL);
    // Hysteresis on m = V(phase)·(iref − iL·Rsense): a ±dIL/2 band at the line peak (m-band ∝ |v|).
    d.currentHysteresis = 0.5 * dIL * d.senseResistance * vpeak;
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
    auto sp = [](const std::string& st, const std::string& po) { json e; e["stage"]=st; e["port"]=po; return e; };
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

    // Per-phase Vienna leg: phase -> Rsense -> nL -> L -> X ; D+ (X->busP), D- (busN->X) ; bidirectional
    // switch X<->gnd. V(phase)-V(nL) = iL·Rsense is the inductor-current sense.
    const char* ph[3] = {"a", "b", "c"};
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        const std::string RS="Rs"+p, L="L"+p, SW="SW"+p, DP="Dp"+p, DN="Dn"+p, X="x"+p, G="g"+p, NL="nl"+p;
        comps.push_back(comp(RS, resBrick(d.senseResistance)));
        comps.push_back(comp(L, indBrick(d.boostInductance)));
        comps.push_back(comp(SW, mosfet()));
        comps.push_back(comp(DP, diode()));
        comps.push_back(comp(DN, diode()));
        conns.push_back(conn("phin_"+p, {pin(RS,"1"), prt(p)}));                   // phase source -> Rsense
        conns.push_back(conn(NL, {pin(RS,"2"), pin(L,"primary_start"), prt(NL)})); // sense node -> L
        conns.push_back(conn(X, {pin(L,"primary_end"), pin(SW,"drain"),
                                 pin(DP,"anode"), pin(DN,"cathode")}));            // node X
        conns.push_back(conn("swret_"+p, {pin(SW,"source"), prt("gnd")}));         // switch -> midpoint
        conns.push_back(conn("dp_"+p, {pin(DP,"cathode"), prt("busP")}));          // X -> +bus
        conns.push_back(conn("dn_"+p, {pin(DN,"anode"), prt("busN")}));            // -bus -> X
        conns.push_back(conn(G+"_net", {pin(SW,"gate"), prt(G)}));
    }
    ports.push_back(port("nla")); ports.push_back(port("nlb")); ports.push_back(port("nlc"));
    ports.push_back(port("ga")); ports.push_back(port("gb")); ports.push_back(port("gc"));
    pcell["ports"] = ports; pcell["components"] = comps; pcell["connections"] = conns;

    // ──────────── CONTROL stage (swappable): per-phase current shaping ────────────
    // For each phase, gate the switch on V(phase)·(iref − iL·Rsense) > 0 — same sign as
    // sign(v)·error, which handles the Vienna polarity flip. The current error folds into ONE weighted
    // difference: err = V(nL) − (1−kref)·V(phase) = iref − iL·Rsense (a summer); multiply by V(phase)
    // (a multiplier); a hysteretic comparator gates the switch. Holds iL ≈ V(phase)/rEmul → unity PF.
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    const double oneMinusKref = 1.0 - d.referenceGain;
    json ccell; ccell["name"] = "vienna-current-control";
    json cports = json::array({port("gnd")});
    json ccomps = json::array();
    json cconns = json::array();
    json gndEps = json::array({prt("gnd")});
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        const std::string SUM="Sum"+p, MUL="Mul"+p, CMP="Cmp"+p, NL="nl"+p, ERR="err"+p, M="m"+p, G="g"+p;
        ccomps.push_back(comp(SUM, summer(1.0, -oneMinusKref)));   // err = V(nL) − (1−kref)·V(phase)
        ccomps.push_back(comp(MUL, multiplier()));                 // m = V(phase)·err
        ccomps.push_back(comp(CMP, comparator(d.currentHysteresis)));
        cports.push_back(port(p)); cports.push_back(port(NL)); cports.push_back(port(G));
        cconns.push_back(conn(p,   {pin(SUM,"inB"), pin(MUL,"inA"), prt(p)}));      // V(phase)
        cconns.push_back(conn(NL,  {pin(SUM,"inA"), prt(NL)}));                     // V(nL)
        cconns.push_back(conn(ERR, {pin(SUM,"out"), pin(MUL,"inB")}));
        cconns.push_back(conn(M,   {pin(MUL,"out"), pin(CMP,"inPlus")}));
        cconns.push_back(conn(G,   {pin(CMP,"out"), prt(G)}));
        gndEps.push_back(pin(CMP,"inMinus"));
    }
    cconns.push_back(json{{"name","gnd"},{"endpoints",gndEps}});
    ccell["ports"] = cports; ccell["components"] = ccomps; ccell["connections"] = cconns;

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

    auto pstage = [&](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"]=name; s["role"]=role; s["circuit"]=brick; s["inputPort"]=inb; s["outputPort"]=outb; return s; };
    tas["topology"]["stages"] = json::array({
        pstage("viennaPower", "switchingCell", pcell, bind("a","acInput"), bind("busP","dcOutput")),
        pstage("viennaControl", "control", ccell, bind("a","sense"), bind("ga","drive"))});
    // a/b/c are the three input phases (also tapped by the control); gnd = source neutral = midpoint.
    json interStage = json::array({
        isc("PhaseA","externalPort","input",{sp("viennaPower","a"), sp("viennaControl","a")}),
        isc("PhaseB","externalPort","input",{sp("viennaPower","b"), sp("viennaControl","b")}),
        isc("PhaseC","externalPort","input",{sp("viennaPower","c"), sp("viennaControl","c")}),
        isc("GND","externalPort","input",{sp("viennaPower","gnd"), sp("viennaControl","gnd")}),
        isc("busP","wire","",{sp("viennaPower","busP")}),
        isc("busN","wire","",{sp("viennaPower","busN")})});
    for (int i = 0; i < 3; ++i) {
        const std::string p = ph[i];
        interStage.push_back(isc("nl"+p,"wire","",{sp("viennaPower","nl"+p), sp("viennaControl","nl"+p)}));
        interStage.push_back(isc("drive"+p,"wire","",{sp("viennaControl","g"+p), sp("viennaPower","g"+p)}));
    }
    tas["topology"]["interStageConnections"] = interStage;

    json an; an["type"]="transient"; an["stopTime"]=0.04; an["maximumTimeStep"]=5e-7;
    tas["simulation"]["analyses"] = json::array({an});
    // Closed loop — the control stage drives the switches (no open-loop stimulus). Precharge each
    // half-bus to ±Vdc/2 about the grounded midpoint.
    json ics = json::array();
    { json ic; ic["node"]="busP"; ic["voltage"]= 0.5*d.outputVoltage; ics.push_back(ic); }
    { json ic; ic["node"]="busN"; ic["voltage"]=-0.5*d.outputVoltage; ics.push_back(ic); }
    tas["simulation"]["initialConditions"] = ics;
    return tas;
}

} // namespace Kirchhoff
