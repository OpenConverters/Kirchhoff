#include "Forward.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_forward + excitations_processed/winding_current
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
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 1).value_or(n);
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

    // Per-output legs (multi-output: N isolated secondaries, ABT #86). Every secondary sees the SAME primary
    // volt-seconds, so each rail's turns ratio scales with its own (Vout_i+Vd_i): n_i = Vin_min·D_max/(Vout_i+Vd_i)
    // — i.e. each output regulates to its own Vout at the shared max duty. Rectifier/output-filter sizing
    // repeats the main-output formulas per rail. outputs[0] reproduces the scalars above byte-for-byte.
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = dr.at("outputs").size();
    for (size_t i = 0; i < nOut; ++i) {
        ForwardOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"));
        if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
            leg.power = tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        else
            leg.power = nominal(dr.at("outputs").at(i).at("power"));
        const double iout_i = leg.power / leg.voltage;
        leg.diodeDrop = req::dideal_diode_drop(iout_i);
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;
            leg.diodeDrop = d.diodeDrop;      // preserve the main rail's exact scalar value
            leg.outputInductance = d.outputInductance;
            leg.outputCapacitance = d.outputCapacitance;
        } else {
            double ni = vinMin * cfg::get(d.config, "maxDutyCycle", kMaxDuty) / (leg.voltage + leg.diodeDrop);
            ni = std::round(ni * 100.0) / 100.0;
            leg.turnsRatio = req::provided_turns_ratio(dr, 1 + i).value_or(ni);
            leg.outputInductance = (vinMax / leg.turnsRatio - leg.diodeDrop - leg.voltage) * tOn / ripple;
            leg.outputCapacitance = 100e-6;
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        d.outputs.push_back(leg);
    }
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

    namespace AN = Kirchhoff::analytical;
    const double Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double T = 1.0 / fsw;
    const double Dn = d.dutyCycle, Vin = d.inputVoltage;
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = d.outputs.size();

    // Per-output vectors for the multi-secondary FHA solver (ABT #86). Single output -> {1.0, n} / {"…","secondary"}
    // exactly as before (the extra legs simply do not appear), so the single-output TAS stays byte-identical.
    std::vector<double> Vouts, iouts, turnsRatios{1.0};
    std::vector<std::string> xfmrIso{"primary", "primary"};   // primary + demag both on the primary side
    std::vector<bool> ceil{false};                            // demag winding is structural 1:1 (nominal)
    double totalOutputPower = 0.0;
    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        Vouts.push_back(leg.voltage);
        iouts.push_back(leg.power / leg.voltage);
        turnsRatios.push_back(leg.turnsRatio);
        xfmrIso.push_back(req::isolation_side(1 + i));        // sec0->secondary, sec1->tertiary, …
        ceil.push_back(true);                                 // each duty-derived secondary ratio is a {maximum} ceiling
        totalOutputPower += leg.power;
    }

    // --- MAIN magnetic (the 3-winding transformer T1) sourced from the SINGLE FHA solver ---
    // analytical_forward returns the transformer windings — Primary, Demagnetization winding, Secondary 0..N-1 —
    // matching T1's turnsRatios = [demag(=1), n0, n1, …]. The output-filter inductors Lout_i are SEPARATE
    // magnetics (NOT solver windings) and keep their inline excitation. The magnetizing reset peak
    // (demag-diode rating) also stays inline — a clean Vin*D*T/Lm, whereas the solver folds the magnetizing
    // current into the primary/demag windings with an offset the diode rating should not read.
    //
    // CORNER: n is sized for D_max = 0.5 at Vin_min (the transformer-reset limit), so evaluating the solver
    // at Vin_min sits exactly on its t1 <= T/2 guard (it throws for some operating points). The declared
    // NOMINAL operating point (D_nom < 0.5) is used — which is also the corner the existing switch/rectifier
    // current ratings were already computed at (Vin, not Vin_min), so this is the faithful rating basis too.
    const MAS::OperatingPoint aopNom = AN::analytical_forward(Vin, Vouts, iouts, turnsRatios, fsw, Lm,
                                                             d.outputs[0].outputInductance, ripple, d.outputs[0].diodeDrop);
    const double IpkPri  = AN::winding_current(aopNom, 0, "peak");   // primary trapezoid peak (switch rating)
    const double IrmsPri = AN::winding_current(aopNom, 0, "rms");    // primary rms (switch RdsOn conduction)
    const double ImagPk  = Vin * Dn * T / Lm;                        // magnetizing reset peak (demag-diode rating)

    // --- component PEAS docs (complete magnetic seeds: designRequirements + per-winding excitations) ---
    // Transformer: turnsRatios = [1 (demag/reset), n0, n1, …] -> (2+N) excitations (from the solver).
    // magnetic_inputs takes the SECONDARY-side ratios (the primary is implicit/prepended), so it gets the
    // full [demag(1.0), n0, n1, …] list — same as analytical_forward's turnsRatios. isolationSides carries
    // the explicit primary too (its length is turnsRatios.size()+1). (abt #49: each duty-derived ratio {maximum})
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, xfmrIso, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"), ceil);

    // --- semiconductor stresses (single-switch forward, 1:1 demag reset) ---
    // Primary switch blocks Vin_max + the reflected reset voltage. With a 1:1 demag winding the core
    // resets at -Vin, so during reset the open switch sees Vin_max (rail) + Vin_max (reflected demag)
    // = 2*Vin_max. Evaluated at the max-input corner. ratedVds = stress / V_DERATE. RdsOn budget uses the
    // TOTAL output power (all rails flow through the one primary switch).
    const double VdsStress = 2.0 * d.inputVoltageMax;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * totalOutputPower / (IrmsPri * IrmsPri);
    const double ratedVrDemag = d.inputVoltageMax / cfg::v_derate_diode(d.config);
    const double maxVfDemag = (ratedVrDemag < 100.0) ? 0.6 : 1.2;
    const double maxTrr     = 0.05 * T;

    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOn, 125.0);
    auto dpdoc = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };
    // Demag/reset diode carries the magnetizing reset current (peak ImagPk, Iout/0.7-style margin).
    json ddemag = dpdoc();
    ddemag["inputs"]["designRequirements"] = req::diode(ratedVrDemag, ImagPk / 0.7, maxVfDemag, maxTrr);

    // --- forward power cell: switch + transformer + demag diode + per-output {forward/freewheel diodes +
    // output inductor (+ output cap for the extra rails)}. Dot orientations match MKF (primary & each
    // secondary in-phase; demag reversed). The main rail (output 0) keeps its Cout in the output-filter
    // stage (byte-identical single-output deck); extra rails carry Cout_i inside the cell + an external
    // vout_i port whose load the assembler synthesizes. ---
    std::vector<json> comps{comp("Q1", mosfet), comp("T1", xfmr), comp("Ddemag", ddemag)};
    std::vector<json> cports{port("vin"), port("gnd"), port("vout"), port("gate")};
    std::vector<json> conns;
    conns.push_back(conn("vin_net",  {pin("Q1", "drain"), pin("Ddemag", "cathode"), prt("vin")}));
    conns.push_back(conn("pri_node", {pin("Q1", "source"), pin("T1", "primary_start")}));
    // demag (secondary1): start at gnd (reversed), end -> demag diode anode
    conns.push_back(conn("demag_in", {pin("T1", "secondary1_end"), pin("Ddemag", "anode")}));
    std::vector<json> gndEps{pin("T1", "primary_end"), pin("T1", "secondary1_start")};

    json capd; capd["capacitor"] = json::object();   // main-rail output cap (output-filter stage)
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const std::string sfx  = (i == 0) ? std::string() : std::to_string(i + 1);   // "", "2", "3", …
        const std::string sw   = "secondary" + std::to_string(2 + i) + "_start";     // demag=secondary1, sec0=secondary2, …
        const std::string dfwdN = "Dfwd" + sfx, dfwN = "Dfw" + sfx, loutN = "Lout" + sfx, coutN = "Cout" + sfx;
        const double iout_i = leg.power / leg.voltage;

        // Output filter inductor Lout_i — inline excitation: avg = Iout_i, pk-pk ripple from the design;
        // +(Vin/n_i - Vd - Vout_i) during ON, -Vout_i during OFF.
        const double dILout  = (Vin / leg.turnsRatio - leg.diodeDrop - leg.voltage) * (Dn * T) / leg.outputInductance;
        const double IpkLout = iout_i + dILout / 2.0;
        const double IrmsLout = std::sqrt(iout_i * iout_i + dILout * dILout / 12.0);
        const double vLonF = Vin / leg.turnsRatio - leg.diodeDrop - leg.voltage, vLoff = leg.voltage;
        const double vLoutPk = std::max(std::abs(vLonF), vLoff), vLoutPkPk = std::abs(vLonF) + vLoff;
        const double vLoutRms = std::sqrt(Dn * vLonF * vLonF + (1.0 - Dn) * vLoff * vLoff);
        json lout; lout["magnetic"] = json::object();
        lout["inputs"] = req::magnetic_inputs(leg.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, iout_i, dILout, Dn,
                                    vLoutPk, vLoutRms, 0.0, vLoutPkPk)});

        // Forward/freewheel rectifiers block the secondary peak Vin_max/n_i, carry ~Iout_i.
        const double ratedVrSec = (d.inputVoltageMax / leg.turnsRatio) / cfg::v_derate_diode(d.config);
        const double maxVfSec   = (ratedVrSec < 100.0) ? 0.6 : 1.2;
        json dfwd = dpdoc(); dfwd["inputs"]["designRequirements"] = req::diode(ratedVrSec, iout_i / 0.7, maxVfSec, maxTrr);
        json dfw  = dpdoc(); dfw["inputs"]["designRequirements"]  = req::diode(ratedVrSec, iout_i / 0.7, maxVfSec, maxTrr);

        comps.push_back(comp(dfwdN.c_str(), dfwd));
        comps.push_back(comp(dfwN.c_str(), dfw));
        comps.push_back(comp(loutN.c_str(), lout));

        // secondary(2+i): start at rectifier (in-phase dot), end -> gnd
        conns.push_back(conn(("sec_in" + sfx).c_str(),   {pin("T1", sw.c_str()), pin(dfwdN.c_str(), "anode")}));
        conns.push_back(conn(("sec_rect" + sfx).c_str(), {pin(dfwdN.c_str(), "cathode"), pin(dfwN.c_str(), "cathode"),
                                                          pin(loutN.c_str(), "primary_start")}));
        if (i == 0) {
            conns.push_back(conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}));
        } else {
            json capi; capi["capacitor"] = json::object();
            capi["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.outputCapacitance;
            capi["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;
            comps.push_back(comp(coutN.c_str(), capi));
            const std::string voutP = "vout" + sfx;
            cports.push_back(port(voutP.c_str()));
            conns.push_back(conn((voutP + "_net").c_str(), {pin(loutN.c_str(), "primary_end"),
                                                            pin(coutN.c_str(), "1"), prt(voutP.c_str())}));
            gndEps.push_back(pin(coutN.c_str(), "2"));
        }
        gndEps.push_back(pin("T1", ("secondary" + std::to_string(2 + i) + "_end").c_str()));
        gndEps.push_back(pin(dfwN.c_str(), "anode"));
    }
    gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));
    conns.push_back(conn("gate_net", {pin("Q1", "gate"), prt("gate")}));

    json cell; cell["name"] = "forward-cell";
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;

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
    dreq["outputs"] = json::array();
    json opDoc; opDoc["name"] = "full_load"; opDoc["inputVoltage"] = d.inputVoltage; opDoc["ambientTemperature"] = 25.0;
    opDoc["outputs"] = json::array();
    for (size_t i = 0; i < nOut; ++i) {
        const std::string oname = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        json o; o["name"] = oname; o["voltage"]["nominal"] = d.outputs[i].voltage; o["regulation"] = "voltage";
        dreq["outputs"].push_back(o);
        json oo; oo["name"] = oname; oo["power"] = d.outputs[i].power; opDoc["outputs"].push_back(oo);
    }
    tas["inputs"]["operatingPoints"] = json::array({opDoc});

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("forwardCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("forwardCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("forwardCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("forwardCell", "vout"), sp("filter", "in")})};
    for (size_t i = 1; i < nOut; ++i) {
        const std::string g = "Vout" + std::to_string(i + 1), pt = "vout" + std::to_string(i + 1);
        iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("forwardCell", pt.c_str())}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "forwardCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fsw;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    req::finalize_control_seeds(tas, Topology::SINGLE_SWITCH_FORWARD_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
