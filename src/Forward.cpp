#include "Forward.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kMaxDuty = 0.5;       // single-switch forward (1:1 demag reset)
constexpr double kRippleRatio = 0.4;
} // namespace

ForwardDesign design_forward(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ForwardDesign d{};
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

    const double iout = d.outputPower / d.outputVoltage;
    // Turns ratio n = Vin_min*D_max/(Vout+Vd) so D(Vin_min)=D_max (MKF). Rounded to 2 dp.
    d.diodeDrop = req::dideal_diode_drop(d.outputPower / d.outputVoltage);  // DIDEAL Vf at the operating rectifier current
    double n = vinMin * cfg::get(d.config, "maxDutyCycle", kMaxDuty) / (d.outputVoltage + d.diodeDrop);
    n = std::round(n * 100.0) / 100.0;
    d.turnsRatio = n;
    // Magnetizing inductance: Lm = Vin_min / (fsw * reflected secondary current), reflected = Iout/n.
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * n / (d.switchingFrequency * iout));
    // Output inductor: Lout = (Vin_max/n - Vd - Vout) * tOn / rippleRatio,  tOn = D_max/fsw.
    const double tOn = cfg::get(d.config, "maxDutyCycle", kMaxDuty) / d.switchingFrequency;
    d.outputInductance = (vinMax / n - d.diodeDrop - d.outputVoltage) * tOn / cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    // Operating (deck) duty at the nominal input.
    d.dutyCycle = n * (d.outputVoltage + d.diodeDrop) / d.inputVoltage;
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    d.outputCapacitance = 100e-6;   // matches MKF forward (Cout=100u)
    return d;
}

json build_forward_tas(const ForwardDesign& d) {
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

    const double n = d.turnsRatio, Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double T = 1.0 / fsw;
    const double iout = d.outputPower / d.outputVoltage;
    const double Dn = d.dutyCycle, Vin = d.inputVoltage, Vout = d.outputVoltage;

    // --- per-winding electrical stresses (single-switch forward, 1:1 demag reset) ---
    // Magnetizing current: linear ramp during ON, peak = Vin*D*T/Lm; it resets through the demag
    // winding during OFF. rms of a 0..Imag triangle over duty D = Imag_pk*sqrt(D/3).
    const double ImagPk  = Vin * Dn * T / Lm;
    const double ImagRms = ImagPk * std::sqrt(Dn / 3.0);
    // Output inductor: avg = Iout, pk-pk ripple from the existing design, pk/rms about that.
    const double dILout  = (Vin / n - d.diodeDrop - Vout) * (Dn * T) / d.outputInductance;
    const double IpkLout = iout + dILout / 2.0;
    const double IrmsLout = std::sqrt(iout * iout + dILout * dILout / 12.0);
    // Secondary winding carries the inductor current during the ON interval only (conducts D).
    const double IcSec   = iout;                         // flat-top inductor current reflected to sec
    const double IpkSec  = IpkLout;
    const double IrmsSec = std::sqrt(Dn) * std::sqrt(iout * iout + dILout * dILout / 12.0);
    // Primary carries the reflected secondary current (Iout/n) plus the magnetizing ramp, during ON.
    const double IcPri   = iout / n;
    const double IpkPri  = IpkSec / n + ImagPk;
    const double IrmsPri = std::sqrt(Dn) * IcPri + ImagRms;   // conservative sum of the two ON-interval rms parts
    // Winding voltages (volt-second basis at the nominal operating point). Primary: +Vin during ON,
    // -Vin during reset (1:1 demag clamps to -Vin); peak |Vin|, full swing 2*Vin. Demag mirrors the
    // primary (ratio 1). Secondary: +Vin/n during ON, 0 (freewheel) during OFF.
    const double vPriPk = Vin, vPriPkPk = 2.0 * Vin;
    const double vPriRms = std::sqrt(Dn * Vin * Vin + (1.0 - Dn) * Vin * Vin);  // = Vin (square ±Vin reset)
    const double vSecPk = Vin / n, vSecPkPk = Vin / n;
    const double vSecRms = std::sqrt(Dn) * (Vin / n);
    // Output inductor voltage: +(Vin/n - Vout) during ON, -Vout during OFF (diode drop folded in).
    const double vLonF = Vin / n - d.diodeDrop - Vout, vLoff = Vout;
    const double vLoutPk = std::max(std::abs(vLonF), vLoff), vLoutPkPk = std::abs(vLonF) + vLoff;
    const double vLoutRms = std::sqrt(Dn * vLonF * vLonF + (1.0 - Dn) * vLoff * vLoff);

    // --- component PEAS docs (complete magnetic seeds: designRequirements + per-winding excitations) ---
    // 3-winding transformer: turnsRatios = [1 (demag/reset), n (secondary)] -> 3 excitations.
    std::vector<std::string> xfmrIso{"primary", "primary", "secondary"};  // primary+demag on the primary side
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {1.0, n}, xfmrIso, std::nullopt, 25.0, {
        req::winding_excitation("forwardPrimary", fsw, IpkPri, IrmsPri, 0.0, IpkPri, Dn,
                                vPriPk, vPriRms, 0.0, vPriPkPk),
        req::winding_excitation("forwardDemag",   fsw, ImagPk, ImagRms, 0.0, ImagPk, 1.0 - Dn,
                                vPriPk, vPriRms, 0.0, vPriPkPk),
        req::winding_excitation("forwardSecondary", fsw, IpkSec, IrmsSec, IcSec, dILout, Dn,
                                vSecPk, vSecRms, 0.0, vSecPkPk)});
    // Output filter inductor: single winding (turnsRatios = []) -> 1 excitation, DC-biased at Iout.
    json lout; lout["magnetic"] = json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
        req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, iout, dILout, Dn,
                                vLoutPk, vLoutRms, 0.0, vLoutPkPk)});
    // --- semiconductor stresses (single-switch forward, 1:1 demag reset) ---
    // Primary switch blocks Vin_max + the reflected reset voltage. With a 1:1 demag winding the core
    // resets at -Vin, so during reset the open switch sees Vin_max (rail) + Vin_max (reflected demag)
    // = 2*Vin_max. Evaluated at the max-input corner. ratedVds = stress / V_DERATE.
    const double VdsStress = 2.0 * d.inputVoltageMax;
    const double ratedVds  = VdsStress / cfg::v_derate(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsPri * IrmsPri);
    // Forward/freewheel rectifiers block the secondary peak voltage Vin_max/n (the other diode clamps
    // the rectified node). The 1:1 demag/reset diode blocks Vin_max (it sees the primary rail during ON).
    const double VrSec     = d.inputVoltageMax / n;
    const double ratedVrSec   = VrSec / cfg::v_derate(d.config);
    const double ratedVrDemag = d.inputVoltageMax / cfg::v_derate(d.config);
    const double maxVfSec   = (ratedVrSec   < 100.0) ? 0.6 : 1.2;
    const double maxVfDemag = (ratedVrDemag < 100.0) ? 0.6 : 1.2;
    const double maxTrr     = 0.05 * T;

    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0);
    auto dpdoc = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };
    // Real rectifiers (none are FET body diodes): forward + freewheel each carry the inductor current
    // (~Iout, avg-with-margin Iout/0.7); the demag/reset diode carries the magnetizing reset current
    // (peak ImagPk, with the same Iout/0.7-style margin folded into a peak-derived rating).
    json ddemag = dpdoc();
    ddemag["inputs"]["designRequirements"] = req::diode(ratedVrDemag, ImagPk / 0.7, maxVfDemag, maxTrr);
    json dfwd = dpdoc();
    dfwd["inputs"]["designRequirements"] = req::diode(ratedVrSec, iout / 0.7, maxVfSec, maxTrr);
    json dfw = dpdoc();
    dfw["inputs"]["designRequirements"] = req::diode(ratedVrSec, iout / 0.7, maxVfSec, maxTrr);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // --- forward power cell: switch + 3-winding transformer + demag diode + forward/freewheel diodes
    // + output inductor. Dot orientations match MKF (primary & secondary in-phase; demag reversed). ---
    json cell; cell["name"] = "forward-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
    cell["components"] = json::array({comp("Q1", mosfet), comp("T1", xfmr), comp("Ddemag", ddemag),
                                      comp("Dfwd", dfwd), comp("Dfw", dfw), comp("Lout", lout)});
    cell["connections"] = json::array({
        // primary: vin -> Q1 -> pri_node(primary_start); primary_end -> gnd
        conn("vin_net",  {pin("Q1", "drain"), pin("Ddemag", "cathode"), prt("vin")}),
        conn("pri_node", {pin("Q1", "source"), pin("T1", "primary_start")}),
        // demag (secondary1): start at gnd (reversed), end -> demag diode anode
        conn("demag_in", {pin("T1", "secondary1_end"), pin("Ddemag", "anode")}),
        // secondary (secondary2): start at rectifier (in-phase dot), end -> gnd
        conn("sec_in",   {pin("T1", "secondary2_start"), pin("Dfwd", "anode")}),
        // rectified node: forward-diode cathode + freewheel-diode cathode + output inductor
        conn("sec_rect", {pin("Dfwd", "cathode"), pin("Dfw", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("T1", "primary_end"), pin("T1", "secondary1_start"),
                          pin("T1", "secondary2_end"), pin("Dfw", "anode"), prt("gnd")}),
        conn("gate_net", {pin("Q1", "gate"), prt("gate")})});

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
        pstage("forwardCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("forwardCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("forwardCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("forwardCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "forwardCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fsw;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    return tas;
}

} // namespace Kirchhoff
