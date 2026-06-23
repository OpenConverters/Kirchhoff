#include "Pfc.hpp"
#include <cmath>
#include <vector>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    if (j.contains("minimum") && j.contains("maximum"))
        return 0.5 * (j.at("minimum").get<double>() + j.at("maximum").get<double>());
    throw std::runtime_error("pfc design: no nominal");
}
constexpr double kSenseResistance   = 0.1;     // input-current sense [Ω]
constexpr double kRippleFraction    = 0.30;    // hysteretic current ripple as a fraction of peak iL
constexpr double kOutputCapacitance = 220e-6;  // bus cap
} // namespace

PfcDesign design_pfc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PfcDesign d{};
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
    const double rEmul = d.inputVoltageRms * d.inputVoltageRms / pin;  // emulated input resistance
    const double iPeak = vpeak / rEmul;                                // peak inductor/line current

    d.senseResistance = kSenseResistance;
    // i_ref voltage = kref·V(busP) must equal the target sense voltage iL·Rsense = (V(busP)/rEmul)·Rsense.
    d.referenceGain = d.senseResistance / rEmul;
    // CCM ripple ΔiL = fraction·iPeak; size L for the target switching frequency at the line peak:
    //   f_peak = Vpk·(Vout−Vpk) / (ΔiL·L·Vout)  =>  L = Vpk·(Vout−Vpk) / (ΔiL·f·Vout).
    const double dIL = kRippleFraction * iPeak;
    d.boostInductance = vpeak * (d.outputVoltage - vpeak)
                        / (dIL * d.switchingFrequency * d.outputVoltage);
    d.currentHysteresis = 0.5 * dIL * d.senseResistance;   // half-band on the i·Rsense signal
    d.outputCapacitance = kOutputCapacitance;
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    return d;
}

json build_pfc_tas(const PfcDesign& d) {
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
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };

    // ───────────── POWER stage: diode bridge + boost + inductor-current sense ─────────────
    // Rsense is in SERIES with the inductor (busP -> Rsense -> nL -> L), so V(busP)-V(nL) = iL·Rsense is
    // the true input/inductor current (a return-side resistor would sense the load return, not iL). The
    // boost return is clean ground.
    json pcell; pcell["name"] = "pfc-power";
    pcell["ports"] = json::array({port("acLine"), port("acNeutral"), port("gnd"), port("vout"),
                                  port("busP"), port("nL"), port("g")});
    pcell["components"] = json::array({
        comp("D1", diode()), comp("D2", diode()), comp("D3", diode()), comp("D4", diode()),
        comp("Rsense", resBrick(d.senseResistance)),   // inductor-current sense (in series with L)
        comp("L", indBrick(d.boostInductance)), comp("SW", mosfet()), comp("D5", diode()),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        comp("Rref", resBrick(10e3))});                 // floating-mains common-mode reference
    pcell["connections"] = json::array({
        // Full bridge: A=acLine, B=acNeutral, top rail busP, bottom rail = ground.
        conn("acLine",  {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
        conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
        conn("busP",    {pin("D1","cathode"), pin("D2","cathode"), pin("Rsense","1"), prt("busP")}),
        conn("nL",      {pin("Rsense","2"), pin("L","primary_start"), prt("nL")}),
        conn("gnd_net", {pin("D3","anode"), pin("D4","anode"), pin("SW","source"),
                         pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
        // Boost: nL -> L -> swNode -> D5 -> vout ; SW shunts swNode to ground.
        conn("swNode",  {pin("L","primary_end"), pin("SW","drain"), pin("D5","anode")}),
        conn("vout_net",{pin("D5","cathode"), pin("Cout","1"), prt("vout")}),
        conn("g_net",   {pin("SW","gate"), prt("g")})});

    // ──────────────────── CONTROL stage (swappable): current-mode controller ────────────────────
    // Threshold node vth = V(busP)·(1−kref). The comparator gates on V(nL)−V(vth) = kref·V(busP) −
    // iL·Rsense, so the hysteretic loop holds iL ≈ (kref/Rsense)·V(busP) = V(busP)/rEmul: the inductor
    // current tracks the rectified line → unity PF. Common mode (busP) cancels in the differential
    // comparator. Divider: Rd1 (busP→vth) and Rd2 (vth→gnd) with Rd2/(Rd1+Rd2) = 1−kref.
    const double rd2 = 100e3;
    const double rd1 = rd2 * d.referenceGain / (1.0 - d.referenceGain);
    json ccell; ccell["name"] = "pfc-current-control";
    ccell["ports"] = json::array({port("busP"), port("nL"), port("gnd"), port("g")});
    ccell["components"] = json::array({
        comp("Rd1", resBrick(rd1)), comp("Rd2", resBrick(rd2)),
        comp("Cmp", comparator(d.currentHysteresis))});
    ccell["connections"] = json::array({
        conn("busP",  {pin("Rd1","1"), prt("busP")}),
        conn("vth",   {pin("Rd1","2"), pin("Rd2","1"), pin("Cmp","inMinus")}),
        conn("gnd",   {pin("Rd2","2"), prt("gnd")}),
        conn("nL",    {pin("Cmp","inPlus"), prt("nL")}),
        conn("g",     {pin("Cmp","out"), prt("g")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acSinglePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltageRms; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput")),
        pstage("pfcControl", "control", ccell, bind("nL", "sense"), bind("g", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd"), sp("pfcControl", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout")}),
        isc("busP", "wire", "", {sp("pfcPower", "busP"), sp("pfcControl", "busP")}),
        isc("nL", "wire", "", {sp("pfcPower", "nL"), sp("pfcControl", "nL")}),
        isc("drive", "wire", "", {sp("pfcControl", "g"), sp("pfcPower", "g")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.06; an["maximumTimeStep"] = 2e-7;
    tas["simulation"]["analyses"] = json::array({an});
    // The controller drives the switch (closed loop) — no open-loop stimulus. Precharge the bus so steady
    // state is reached in a few line cycles (the bus-cap RC is far longer than the sim window).
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    return tas;
}

} // namespace Kirchhoff
