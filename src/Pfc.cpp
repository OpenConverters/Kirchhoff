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
    d.integralGain = 3.0;        // outer voltage loop (tuned for stable ~few-line-cycle settling)
    d.outputDividerGain = 0.01;  // V(voutScaled) = 0.01·V(vout) -> ~4 V at a 400 V bus
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
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto integrator = [&](double gain, double initial, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"] = gain; e["initial"] = initial; e["reference"] = ref;
        e["outputLow"] = lo; e["outputHigh"] = hi; return j; };

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

    // ──────────────── CONTROL stage (swappable): current loop + outer VOLTAGE loop ────────────────
    // INNER current loop: the comparator gates the switch on V(nL)−V(vth), where vth = V(busP)·gv. That
    // holds iL ≈ V(busP)·(1−gv)/Rsense ∝ |Vac| → unity PF (the differential sense cancels the busP common
    // mode). OUTER voltage loop: an integrator drives gv from the bus error so the drawn power regulates
    // the bus to its target — gv is the current-loop gain the voltage loop trims (a multiplier forms
    // vth = busP·gv). Bus low -> gv falls -> more current; bus high -> gv rises -> less current.
    //   voutScaled = kv·V(vout);  gv = clamp(g0 + ki·∫(voutScaled − kv·Vout_target)dt)
    const double kref = d.referenceGain;
    const double g0   = 1.0 - kref;                                  // nominal current-loop gain
    const double vrefScaled = d.outputDividerGain * d.outputVoltage; // bus setpoint, scaled
    const double rv2 = 1e3, rv1 = rv2 * (1.0 - d.outputDividerGain) / d.outputDividerGain;
    json ccell; ccell["name"] = "pfc-voltage-current-control";
    ccell["ports"] = json::array({port("busP"), port("nL"), port("vout"), port("gnd"), port("g")});
    ccell["components"] = json::array({
        comp("Rv1", resBrick(rv1)), comp("Rv2", resBrick(rv2)),
        comp("Iv", integrator(d.integralGain, g0, vrefScaled, 0.99, 0.99995)),
        comp("Mv", multiplier()),
        comp("Cmp", comparator(d.currentHysteresis))});
    ccell["connections"] = json::array({
        conn("vout",      {pin("Rv1","1"), prt("vout")}),
        conn("voutScaled",{pin("Rv1","2"), pin("Rv2","1"), pin("Iv","in")}),
        conn("gnd",       {pin("Rv2","2"), prt("gnd")}),
        conn("gv",        {pin("Iv","out"), pin("Mv","inB")}),
        conn("busP",      {pin("Mv","inA"), prt("busP")}),
        conn("vth",       {pin("Mv","out"), pin("Cmp","inMinus")}),
        conn("nL",        {pin("Cmp","inPlus"), prt("nL")}),
        conn("g",         {pin("Cmp","out"), prt("g")})});

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
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout"), sp("pfcControl", "vout")}),
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
