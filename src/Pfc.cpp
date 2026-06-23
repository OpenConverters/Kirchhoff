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
constexpr double kSwitchDuty       = 0.2;     // fixed boost duty (chosen for DCM across the line cycle)
constexpr double kOutputCapacitance = 220e-6; // bus cap
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

    d.switchDuty = kSwitchDuty;
    // DCM boost PFC: Pin = Vrms²·D² / (2·L·fsw)  =>  L = Vrms²·D² / (2·fsw·Pin). (Pin ≈ Pout/η.)
    const double pin = d.outputPower / std::max(d.efficiency, 1e-6);
    d.boostInductance = d.inputVoltageRms * d.inputVoltageRms * d.switchDuty * d.switchDuty
                        / (2.0 * d.switchingFrequency * pin);
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

    // ───────────── POWER stage: diode bridge + boost ─────────────
    // Floating mains (acLine, acNeutral) -> full bridge (D1-D4, DC return = gnd) -> busP = |Vac| ->
    // boost (L, SW, D5, Cout) -> vout. Body-diode-free ideal boost switch driven open-loop (DCM).
    json pcell; pcell["name"] = "pfc-power";
    pcell["ports"] = json::array({port("acLine"), port("acNeutral"), port("gnd"), port("vout"), port("g")});
    pcell["components"] = json::array({
        comp("D1", diode()), comp("D2", diode()), comp("D3", diode()), comp("D4", diode()),
        comp("L", indBrick(d.boostInductance)), comp("SW", mosfet()), comp("D5", diode()),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        // Large reference resistor: gives the floating mains nodes a DC path to ground (the bridge
        // diodes are all off near the zero crossing, leaving the AC common-mode undefined -> singular).
        // 1 MΩ draws ~0.3 mA at the line peak — negligible vs the multi-amp DCM current.
        comp("Rref", resBrick(1e6))});
    pcell["connections"] = json::array({
        // Full bridge: A=acLine, B=acNeutral, top rail busP, bottom rail gnd(=0).
        conn("acLine",  {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
        conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
        conn("busP",    {pin("D1","cathode"), pin("D2","cathode"), pin("L","primary_start")}),
        conn("gnd_net", {pin("D3","anode"), pin("D4","anode"), pin("SW","source"),
                         pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
        // Boost: busP -> L -> swNode -> D5 -> vout ; SW shunts swNode to gnd.
        conn("swNode",  {pin("L","primary_end"), pin("SW","drain"), pin("D5","anode")}),
        conn("vout_net",{pin("D5","cathode"), pin("Cout","1"), prt("vout")}),
        conn("g_net",   {pin("SW","gate"), prt("g")})});

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
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.06; an["maximumTimeStep"] = 2e-7;
    tas["simulation"]["analyses"] = json::array({an});
    { json st; st["stage"] = "pfcPower"; st["component"] = "SW"; st["signal"] = "gate";
      st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
      st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = 0.0;
      tas["simulation"]["stimulus"] = json::array({st}); }
    // Precharge the bus to its target so we measure steady-state power factor in a few line cycles
    // (the bus-cap RC is far longer than the sim window).
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    return tas;
}

} // namespace Kirchhoff
