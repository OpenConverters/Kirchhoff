#include "Flyback.hpp"
#include "DimensionJson.hpp"
#include "RasConverter.hpp"
#include "CasConverter.hpp"
#include "SasConverter.hpp"
#include "MasConverter.hpp"
#include "CiasConverter.hpp"
#include "CiasCircuitConverter.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_flyback + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"

#include <sstream>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace Kirchhoff {

using nlohmann::json;

namespace {
double nominal(const json& j, const std::string& what) {
    try { return PEAS::resolve_dimensional_values(j); }
    catch (const std::exception&) { throw std::runtime_error("flyback design: no nominal/min/max for " + what); }
}
} // namespace

FlybackDesign design_flyback(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    FlybackDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"), "outputVoltage");
    d.switchingFrequency = nominal(dr.at("switchingFrequency"), "switchingFrequency");
    d.efficiency = dr.value("efficiency", 0.88);

    // operating point (Vin, Pout): prefer operatingPoints[0], else designRequirements.
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"), "inputVoltage");
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"), "outputPower");
    }

    // Minimum input voltage drives the turns ratio + inductance sizing (MKF semantics).
    const json& iv = dr.at("inputVoltage");
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;
    d.isolationVoltage = dr.value("isolationVoltage", 0.0);

    // --- CCM design — faithful port of MKF Flyback::process_design_requirements()
    // (maximumDutyCycle branch, single output, no drain-source-voltage limit; Vd=0). ---
    const double rippleRatio  = 0.4;   // currentRippleRatio
    const double maxDutyCycle = 0.5;   // MKF Flyback default maximumDutyCycle

    const double Pin = d.outputPower / d.efficiency;
    const double maxEffectiveLoadCurrent = d.outputPower / d.outputVoltage;        // (η=1 numerator)
    const double averageInputCurrent = Pin / vinMin;
    const double maxEffectiveLoadCurrentReflected =
        averageInputCurrent * (1.0 - maxDutyCycle) / maxDutyCycle;
    double n = maxEffectiveLoadCurrent / maxEffectiveLoadCurrentReflected;
    n = std::round(n * 100.0) / 100.0;   // MKF roundFloat(turnsRatio, 2)
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    // Steady-state CCM duty at the nominal operating Vin (the open-loop PWM duty). The secondary must
    // produce Vout+Vd so the output AFTER the rectifier drop is the spec'd Vout:
    //   Vout+Vd = Vin·D / (n·(1-D))  ->  D = n·(Vout+Vd) / (Vin + n·(Vout+Vd)).
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    const double Vor = n * (d.outputVoltage + d.diodeDrop);
    d.dutyCycle = Vor / (d.inputVoltage + Vor);

    // Magnetizing inductance sized at the maximum duty / minimum Vin corner (MKF).
    const double centerSecondaryRamp = maxEffectiveLoadCurrent / (1.0 - maxDutyCycle);
    const double centerPrimaryRamp   = centerSecondaryRamp / n;
    const double tOn = maxDutyCycle / d.switchingFrequency;
    const double voltsSeconds = vinMin * tOn;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        voltsSeconds / rippleRatio / centerPrimaryRamp);

    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    const double iout = d.outputPower / d.outputVoltage;
    d.outputCapacitance = iout * d.dutyCycle / (d.switchingFrequency * 0.01 * d.outputVoltage);
    d.inputCapacitance = 10e-6;
    return d;
}

namespace {
// Pull the single atom out of a to_cias leaf and rename it.
json atom(const json& leaf, const std::string& name) {
    json a = leaf.at("components").at(0);
    a["name"] = name;
    return a;
}
json pe(const std::string& c, const std::string& p) { json e; e["component"] = c; e["pin"] = p; return e; }
json po(const std::string& p) { json e; e["port"] = p; return e; }
json net(const std::string& name, std::vector<json> eps) {
    json n; n["name"] = name; n["endpoints"] = eps; return n;
}
} // namespace

json build_flyback_brick(const FlybackDesign& d, const PEAS::Fidelity& f) {
    // --- ideal/real atoms via the per-family to_cias generators ---
    json mosfetDoc; mosfetDoc["semiconductor"]["mosfet"] = json::object();
    json Q1 = atom(SAS::sas_to_cias(mosfetDoc, f), "Q1");

    json diodeDoc; diodeDoc["semiconductor"]["diode"] = json::object();
    json D1 = atom(SAS::sas_to_cias(diodeDoc, f), "D1");

    json magDoc; magDoc["magnetic"] = json::object();
    magDoc["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.magnetizingInductance;
    json ratios = json::array(); { json r; r["nominal"] = d.turnsRatio; ratios.push_back(r); }
    magDoc["inputs"]["designRequirements"]["turnsRatios"] = ratios;
    json T1 = atom(MAS::mas_to_cias(magDoc, f), "T1");

    auto capDoc = [](double c, double v) {
        json j; j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = v; return j;
    };
    json Cin  = atom(CAS::cas_to_cias(capDoc(d.inputCapacitance,  d.inputVoltage * 2), f), "Cin");
    json Cout = atom(CAS::cas_to_cias(capDoc(d.outputCapacitance, d.outputVoltage * 2), f), "Cout");

    json rDoc;
    rDoc["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rDoc["inputs"]["designRequirements"]["resistance"]["nominal"] = d.loadResistance;
    rDoc["inputs"]["designRequirements"]["powerRating"] = d.outputPower;
    json Rload = atom(RAS::ras_to_cias(rDoc, f), "Rload");

    // --- wire the flyback (secondary return tied to node 0 for the sim; isolation = later phase) ---
    // Dot convention: L*_pri primary_start->primary_end, L*_sec secondary1_start->secondary1_end, K>0.
    // Flyback: diode conducts during OFF, so the anode is the non-dot end (secondary1_end).
    json components = json::array();
    for (auto& c : {Q1, T1, D1, Cin, Cout, Rload}) components.push_back(c);

    json connections = json::array();
    connections.push_back(net("VIN",   { pe("T1", "primary_start"), pe("Cin", "1"), po("VIN") }));
    connections.push_back(net("DRAIN", { pe("T1", "primary_end"),   pe("Q1", "drain") }));
    connections.push_back(net("GATE",  { pe("Q1", "gate"), po("GATE") }));
    connections.push_back(net("0",     { pe("Q1", "source"), pe("Cin", "2"),
                                         pe("T1", "secondary1_start"),
                                         pe("Cout", "2"), pe("Rload", "2") }));
    connections.push_back(net("SEC",   { pe("T1", "secondary1_end"), pe("D1", "anode") }));
    connections.push_back(net("VOUT",  { pe("D1", "cathode"), pe("Cout", "1"),
                                         pe("Rload", "1"), po("VOUT") }));

    json ports = json::array();
    for (auto p : {"VIN", "GATE", "VOUT"}) ports.push_back(json{{"name", p}});

    json brick;
    brick["name"] = "flyback";
    brick["ports"] = ports;
    brick["components"] = components;
    brick["connections"] = connections;
    return brick;
}

std::string emit_flyback_ngspice(const FlybackDesign& d, const PEAS::Fidelity& f) {
    json brick = build_flyback_brick(d, f);
    CIAS::CiasCircuit circ = CIAS::CiasCircuit::from_json(brick);
    std::string cards = CIAS::CiasToNgspiceConverter().to_cards(circ);

    const double period = 1.0 / d.switchingFrequency;
    const double ton = d.dutyCycle * period;
    const double tstep = period / 200.0;
    const double tstop = 600.0 * period;  // settle (RC ~ Rload*Cout)
    const double measFrom = tstop - 50.0 * period;

    std::ostringstream os;
    os.precision(10);
    os << "* Kirchhoff flyback (" << (f.is_ideal() ? "ideal" : "real") << ") "
       << d.inputVoltage << "V -> " << d.outputVoltage << "V / " << d.outputPower << "W\n";
    os << "* designed: n=" << d.turnsRatio << " D=" << d.dutyCycle
       << " Lp=" << d.magnetizingInductance << " Rload=" << d.loadResistance
       << " Cout=" << d.outputCapacitance << "\n";
    os << cards;
    os << "Vin VIN 0 DC " << d.inputVoltage << "\n";
    os << "Vgate GATE 0 PULSE(0 5 0 1n 1n " << ton << " " << period << ")\n";
    os << ".options reltol=1e-3 abstol=1e-9 vntol=1e-6 method=gear\n";
    os << ".tran " << tstep << " " << tstop << " 0 " << tstep << "\n";
    os << ".control\n";
    os << "run\n";
    os << "meas tran vout AVG v(VOUT) from=" << measFrom << " to=" << tstop << "\n";
    os << "meas tran vout_pp PP v(VOUT) from=" << measFrom << " to=" << tstop << "\n";
    os << "meas tran iin AVG i(Vin) from=" << measFrom << " to=" << tstop << "\n";
    os << "print vout vout_pp iin\n";
    os << ".endc\n";
    os << ".end\n";
    return os.str();
}

json build_flyback_tas(const FlybackDesign& d) {
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) {
        json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };

    // --- per-component stresses from the SINGLE FHA source (the SPICE-validated analytical solver) ---
    // Worst-case corner (Vin_min) drives the ratings; the declared nominal OP is what the TAS embeds
    // (which also fixes a latent inconsistency: the old code mixed worst-case currents with nominal
    // voltages in the embedded excitation). analytical_flyback carries the DCM secondary correction.
    namespace AN = Kirchhoff::analytical;
    const double n = d.turnsRatio, fsw = d.switchingFrequency, T = 1.0 / fsw, Lm = d.magnetizingInductance;
    const double Pin = d.outputPower / d.efficiency;
    const double Iout = d.outputPower / d.outputVoltage;
    const double IinMin = Pin / d.inputVoltageMin;
    const MAS::OperatingPoint aopWorst = AN::analytical_flyback(d.inputVoltageMin, {d.outputVoltage}, {Iout},
                                                               {n}, fsw, Lm, 0.0, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_flyback(d.inputVoltage,    {d.outputVoltage}, {Iout},
                                                               {n}, fsw, Lm, 0.0, d.efficiency);
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");
    const double IpkSec  = AN::winding_current(aopWorst, 1, "peak");
    const double IrmsSec = AN::winding_current(aopWorst, 1, "rms");

    // VOLTAGES at Vin_max (max stress corner) for the semiconductor ratings.
    const double VdsStress = d.inputVoltageMax + n * d.outputVoltage;
    const double VrStress  = d.outputVoltage + d.inputVoltageMax / n;
    const double ratedVds = VdsStress / cfg::v_derate_mosfet(d.config);
    const double ratedVr  = VrStress  / cfg::v_derate_diode(d.config);
    const double maxRdsOn = 0.01 * d.outputPower / (IrmsPri * IrmsPri);   // <=1% of Pout conduction
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;               // Schottky-class if low V_R
    const double maxTrr   = 0.05 * T;
    const double IcoutRms = std::sqrt(std::max(0.0, IrmsSec * IrmsSec - Iout * Iout));
    const double IcinRms  = std::sqrt(std::max(0.0, IrmsPri * IrmsPri - IinMin * IinMin));

    // --- component PEAS docs: discriminator + detailed inputs.designRequirements ---
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0);

    json diode;  diode["semiconductor"]["diode"] = json::object();
    diode["inputs"]["designRequirements"] = req::diode(ratedVr, Iout / 0.7, maxVf, maxTrr);

    std::vector<std::string> isoSides{"primary", "secondary"};
    std::optional<double> isoV = d.isolationVoltage > 0 ? std::optional<double>(d.isolationVoltage) : std::nullopt;

    json mag; mag["magnetic"] = json::object();
    mag["inputs"] = req::magnetic_inputs(Lm, 0.1, {n}, isoSides, isoV, 25.0, AN::excitations_processed(aopNom, "T1"));

    json capCin; capCin["capacitor"] = json::object();
    capCin["inputs"]["designRequirements"] = req::capacitor(
        d.inputCapacitance, d.inputVoltageMax / cfg::v_derate_capacitor(d.config), IcinRms,
        req::ESR_RIPPLE_FRACTION * d.inputVoltage / IpkPri, "inputFilter");
    json capCout; capCout["capacitor"] = json::object();
    capCout["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltage / cfg::v_derate_capacitor(d.config), IcoutRms,
        req::ESR_RIPPLE_FRACTION * d.outputVoltage / IpkSec, "outputFilter");
    // --- stage bricks ---
    json inv; inv["name"] = "primary-switch";
    inv["ports"] = json::array({port("dc+"), port("sw"), port("gate"), port("dc-")});
    inv["components"] = json::array({comp("Q1", mosfet), comp("Cin", capCin)});
    inv["connections"] = json::array({
        conn("dc_pos", {pin("Cin", "1"), prt("dc+")}),
        conn("sw",     {pin("Q1", "drain"), prt("sw")}),
        conn("gate",   {pin("Q1", "gate"), prt("gate")}),
        conn("dc_neg", {pin("Q1", "source"), pin("Cin", "2"), prt("dc-")})});

    json xfmr; xfmr["name"] = "flyback-transformer";
    xfmr["ports"] = json::array({port("pri"), port("pri_rtn"), port("sec"), port("sec_rtn")});
    xfmr["components"] = json::array({comp("T1", mag)});
    xfmr["connections"] = json::array({
        conn("primary",       {pin("T1", "primary_start"), prt("pri")}),
        conn("primary_rtn",   {pin("T1", "primary_end"), prt("pri_rtn")}),
        conn("secondary",     {pin("T1", "secondary1_end"), prt("sec")}),
        conn("secondary_rtn", {pin("T1", "secondary1_start"), prt("sec_rtn")})});

    json rect; rect["name"] = "diode-rectifier";
    rect["ports"] = json::array({port("ac_in"), port("dc_out")});
    rect["components"] = json::array({comp("D1", diode)});
    rect["connections"] = json::array({
        conn("anode",   {pin("D1", "anode"), prt("ac_in")}),
        conn("cathode", {pin("D1", "cathode"), prt("dc_out")})});

    // The output filter is part of the converter; the LOAD is not — it is synthesized from the
    // outputs requirement by the assembler (the dual of the input source). So this stage is Cout only.
    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capCout)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    // typed terminal bindings (portType) — a powerStage has one input + one output; the isolation
    // stage may fan out (outputPorts[]).
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto istage = [](const char* name, json brick, json inb, std::vector<json> outbs) {
        json s; s["name"] = name; s["role"] = "isolation"; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPorts"] = outbs; return s; };

    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir;
        c["endpoints"] = eps; return c; };

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    if (d.isolationVoltage > 0) dreq["isolationVoltage"] = d.isolationVoltage;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("inverter", "inverter", inv, bind("dc+", "dcBus"), bind("sw", "hfAc")),
        istage("transformer", xfmr, bind("pri", "hfAc"), {bind("sec", "hfAc")}),
        pstage("rectifier", "outputRectifier", rect, bind("ac_in", "hfAc"), bind("dc_out", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("inverter", "dc+"), sp("transformer", "pri")}),
        isc("GND", "externalPort", "input",
            {sp("inverter", "dc-"), sp("transformer", "sec_rtn"), sp("filter", "rtn")}),
        isc("sw_node", "wire", "", {sp("inverter", "sw"), sp("transformer", "pri_rtn")}),
        isc("sec", "wire", "", {sp("transformer", "sec"), sp("rectifier", "ac_in")}),
        isc("Vout", "externalPort", "output", {sp("rectifier", "dc_out"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.006); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "inverter"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    req::finalize_control_seeds(tas, Topology::FLYBACK_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
