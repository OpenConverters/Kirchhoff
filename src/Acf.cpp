#include "Acf.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_active_clamp_forward + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kDuty       = 0.45;   // main-switch operating duty (MKF ACF maximumDutyCycle)
constexpr double kDeadFrac   = 0.01;   // 100ns dead time between main & clamp switches at 100kHz
constexpr double kRippleRatio = 0.4;   // output-inductor current ripple
constexpr double kIdealSwRon = 0.01;   // ideal-deck switch on-resistance (matches SAS IDEAL_SW_RON) — the
                                       // synchronous-rectifier forward drop is Iout*Rds, not a diode Vf
} // namespace

AcfDesign design_acf(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    AcfDesign d{};
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

    const double Vo = d.outputVoltage, Fs = d.switchingFrequency, Io = d.outputPower / Vo;
    // Output rectifier is a SYNCHRONOUS rectifier (gated MOSFETs SRfwd/SRfw), so its forward drop is the
    // small ohmic Iout*Rds of the conducting SR — NOT a ~0.9 V diode Vf. Sizing n/duty against a diode Vf
    // would over-deliver Vout by ~Vf (fatal on a low-Vout rail: a 3.3 V/30 A design overshot to 3.9 V).
    d.diodeDrop = Io * kIdealSwRon;  // SR conduction drop = rectifier forward drop used for n/duty/Lo sizing
    const double Dmax = cfg::get(d.config, "operatingDutyCycle", kDuty);   // MAX forward duty, occurs at Vin_min
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", kDeadFrac);

    // Turns ratio at the MIN-Vin corner with the max duty: n = Vin_min·Dmax/(Vo+Vd) (MKF sizing). The forward
    // interval the solver derives is t1 = (Vo+Vd)·n·period/Vin, so at Vin_min t1 = Dmax·period < period/2 for
    // Dmax < 0.5 — i.e. the analytical embed's worst corner (Vin_min) stays inside the reset limit. (Sizing at
    // NOMINAL Vin, as before, put t1 at exactly Dmax·Vin_nom/Vin_min·period = period/2 for a ±10% range, which
    // the turns-ratio rounding then tipped over the guard.)
    double n = vinMin * Dmax / (Vo + d.diodeDrop);
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(n * 100.0) / 100.0);
    n = d.turnsRatio;

    // Open-loop deck drive: the duty that regulates at the NOMINAL operating point (the PtP deck simulates at
    // nominal Vin), D_op = (Vo+Vd)·n/Vin_nom ≤ Dmax. So the open-loop deck lands on spec at the OP while the
    // physical stage below is sized at the worst (Vin_min / Dmax) corner.
    const double D = (Vo + d.diodeDrop) * n / d.inputVoltage;
    d.dutyCycle = D;

    // Magnetizing inductance: Lm = Vin_min * n / (Fs * Io)  (reflected secondary current Io/n).
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        vinMin * n / (Fs * Io));
    // Output inductor: Lo = (Vin_max/n - Vd - Vo) * tOn / ripple,  tOn = D/Fs.
    const double tOn = D / Fs;
    d.outputInductance = (vinMax / n - d.diodeDrop - Vo) * tOn / cfg::get(d.config, "inductorRippleRatio", kRippleRatio);

    d.clampCapacitance = 10e-6;   // active-clamp capacitor (matches MKF; value sets reset ring, not Vo)
    d.loadResistance = Vo * Vo / d.outputPower;
    d.outputCapacitance = 100e-6; // matches MKF ACF (Cout=100u)

    // Per-output legs (multi-output: N isolated secondaries, ABT #86). Each secondary shares the primary
    // volt-seconds, so n_i = Vin_min·Dmax/(Vout_i+Vd_i). outputs[0] reproduces the scalars byte-for-byte.
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = dr.at("outputs").size();
    for (size_t i = 0; i < nOut; ++i) {
        AcfOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"));
        if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
            leg.power = tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        else
            leg.power = nominal(dr.at("outputs").at(i).at("power"));
        const double iout_i = leg.power / leg.voltage;
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;
            leg.diodeDrop = d.diodeDrop;
            leg.outputInductance = d.outputInductance;
            leg.outputCapacitance = d.outputCapacitance;
        } else {
            leg.diodeDrop = iout_i * kIdealSwRon;   // synchronous-rectifier conduction drop (like the main rail)
            double ni = vinMin * Dmax / (leg.voltage + leg.diodeDrop);
            ni = std::round(ni * 100.0) / 100.0;
            leg.turnsRatio = req::provided_turns_ratio(dr, i).value_or(ni);
            leg.outputInductance = (vinMax / leg.turnsRatio - leg.diodeDrop - leg.voltage) * tOn / ripple;
            leg.outputCapacitance = 100e-6;
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        d.outputs.push_back(leg);
    }
    return d;
}

json build_acf_tas(const AcfDesign& d) {
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

    namespace AN = Kirchhoff::analytical;
    const double Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double T = 1.0 / fsw, Dn = d.dutyCycle, Vin = d.inputVoltage;
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const size_t nOut = d.outputs.size();

    // Per-output vectors for the multi-secondary FHA solver (ABT #86). No demag winding (active clamp resets):
    // turnsRatios = [n0,n1,…].
    std::vector<double> Vouts, iouts, turnsRatios;
    std::vector<std::string> xfmrIso{"primary"};
    std::vector<bool> ceil;
    double totalOutputPower = 0.0;
    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        Vouts.push_back(leg.voltage);
        iouts.push_back(leg.power / leg.voltage);
        turnsRatios.push_back(leg.turnsRatio);
        xfmrIso.push_back(req::isolation_side(1 + i));
        ceil.push_back(true);
        totalOutputPower += leg.power;
    }

    // --- MAIN magnetic (the transformer T1) sourced from the SINGLE FHA solver ---
    // analytical_active_clamp_forward returns "First primary" + Secondary 0..N-1 — matching T1's
    // turnsRatios = [n0,n1,…]. The output-filter inductors Lout_i are SEPARATE magnetics (inline excitation);
    // the magnetizing reset current (clamp-switch rating) also stays inline. Worst-case corner (Vin_min)
    // drives the main-switch rating; the declared nominal OP is what the TAS embeds.
    const MAS::OperatingPoint aopWorst = AN::analytical_active_clamp_forward(d.inputVoltageMin, Vouts, iouts,
                                            turnsRatios, fsw, Lm, d.outputs[0].outputInductance, ripple, Dn, d.outputs[0].diodeDrop);
    const MAS::OperatingPoint aopNom   = AN::analytical_active_clamp_forward(Vin, Vouts, iouts,
                                            turnsRatios, fsw, Lm, d.outputs[0].outputInductance, ripple, Dn, d.outputs[0].diodeDrop);
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");   // primary peak (main-switch rating)
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");    // primary rms (main-switch RdsOn conduction)
    const double ImagPk  = Vin * Dn * T / Lm;                          // magnetizing reset peak (clamp-switch rating)
    // Clamp switch conducts during the RESET interval (1-Dn)*T, carrying the magnetizing triangle from
    // ImagPk down to 0; its full-period rms is ImagPk*sqrt((1-Dn)/3).
    const double ImagRms = ImagPk * std::sqrt((1.0 - Dn) / 3.0);      // magnetizing reset rms  (clamp-switch RdsOn)

    // --- semiconductor stresses (active-clamp forward) ---
    // Both the main switch Q1 and the clamp switch Sc sit across (Vin + Vclamp). Evaluated at the max-input
    // corner: VdsStress = Vin_max + Vin_max*D/(1-D). RdsOn budgets use the TOTAL output power.
    const double VresetMax = d.inputVoltageMax * Dn / (1.0 - Dn);
    const double VdsStress = d.inputVoltageMax + VresetMax;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnMain  = cfg::rds_on_loss_fraction(d.config) * totalOutputPower / (IrmsPri * IrmsPri);
    const double maxRdsOnClamp = (ImagRms > 0.0)
        ? cfg::rds_on_loss_fraction(d.config) * totalOutputPower / (ImagRms * ImagRms)
        : maxRdsOnMain;
    json mainSw = mosfet();
    mainSw["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOnMain, 125.0);
    json clampSw = mosfet();
    clampSw["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, ImagPk, maxRdsOnClamp, 125.0);
    // FET body diode (anti-parallel to the SR): a DEFERRED bodyDiode seed — the fill emits the FET's own
    // intrinsic diode, so the ngspice deck is unchanged, but the seed carries a SAS-valid designRequirements
    // (role bodyDiode) so the TAS validates. Ratings mirror the SR it shadows (blocks Vsec, carries ILout).
    auto bodyDio = [&](double ratedVr, double ratedIf) {
        json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(ratedVr, ratedIf); return j; };

    // Transformer (primary + N secondaries, no demag): turnsRatios = [n0,n1,…] -> (1+N) excitations (from the solver).
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, xfmrIso, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"), ceil);   // each duty-derived ratio {maximum} (abt #49)

    json cc; cc["capacitor"] = json::object();   // active-clamp capacitor
    cc["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.clampCapacitance;
    cc["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // Numerical node snubber on the switching + clamp nodes (Csn / Csn2). The ACF is the one switching
    // topology that shipped WITHOUT one, so at high Vin / light load the active-clamp node produces an
    // unresolvably fast dV/dt -> ngspice "timestep too small". A small node-to-gnd cap tames it. Both strip
    // together for a real-fidelity deck (replaced by the switch's Coss).
    json snb; snb["capacitor"] = json::object();
    snb["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
    snb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;
    cfg::mark_numerical_aid(snb);   // dV/dt convergence aid — explicitly tagged for the real-fidelity strip (ABT #96)

    // Active-clamp forward cell: main switch Q1 (vin->sw), transformer (sw->gnd primary), the clamp leg
    // (Sc: vin->clamp_node, Cc: clamp_node->sw), and per-output synchronous forward output (SRfwd_i + SRfw_i
    // + Lout_i). Main rail keeps its Cout in the output-filter stage; extra rails carry Cout_i in the cell.
    std::vector<json> comps{comp("Q1", mainSw), comp("Sc", clampSw), comp("Cc", cc), comp("T1", xfmr)};
    std::vector<json> cports{port("vin"), port("gnd"), port("vout"), port("gate_main"), port("gate_clamp")};
    std::vector<json> conns;
    conns.push_back(conn("vin_net",  {pin("Q1", "drain"), pin("Sc", "drain"), prt("vin")}));
    conns.push_back(conn("sw_node",  {pin("Q1", "source"), pin("T1", "primary_start"), pin("Cc", "2"), pin("Csn", "1")}));
    conns.push_back(conn("clamp_node", {pin("Sc", "source"), pin("Cc", "1"), pin("Csn2", "1")}));
    std::vector<json> gndEps{pin("T1", "primary_end")};
    std::vector<json> gateMainEps{pin("Q1", "gate")}, gateClampEps{pin("Sc", "gate")};

    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const std::string sfx  = (i == 0) ? std::string() : std::to_string(i + 1);
        const std::string swS  = "secondary" + std::to_string(1 + i) + "_start";
        const std::string swE  = "secondary" + std::to_string(1 + i) + "_end";
        const std::string srfwdN = "SRfwd" + sfx, dsfwdN = "DSfwd" + sfx, srfwN = "SRfw" + sfx, dsfwN = "DSfw" + sfx;
        const std::string loutN = "Lout" + sfx, coutN = "Cout" + sfx;
        const double iout_i = leg.power / leg.voltage;

        // Output filter inductor Lout_i — inline excitation.
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
        // Output SYNCHRONOUS rectifiers (SRfwd_i + SRfw_i): gated MOSFETs; each blocks Vin_max/n_i, carries ILout_i.
        const double ratedVdsSR  = (d.inputVoltageMax / leg.turnsRatio) / cfg::v_derate_mosfet(d.config);
        const double maxRdsOnSR  = (IrmsLout > 0.0)
            ? cfg::rds_on_loss_fraction(d.config) * leg.power / (IrmsLout * IrmsLout) : maxRdsOnMain;
        const json reqSR = req::mosfet("mainSwitch", ratedVdsSR, IpkLout, maxRdsOnSR, 125.0);
        json srfwd = mosfet(); srfwd["inputs"]["designRequirements"] = reqSR;
        json srfw  = mosfet(); srfw["inputs"]["designRequirements"]  = reqSR;

        comps.push_back(comp(srfwdN.c_str(), srfwd));
        comps.push_back(comp(dsfwdN.c_str(), bodyDio(d.inputVoltageMax / leg.turnsRatio, IpkLout)));
        comps.push_back(comp(srfwN.c_str(), srfw));
        comps.push_back(comp(dsfwN.c_str(), bodyDio(d.inputVoltageMax / leg.turnsRatio, IpkLout)));
        comps.push_back(comp(loutN.c_str(), lout));

        conns.push_back(conn(("sec_in" + sfx).c_str(),   {pin("T1", swS.c_str()), pin(srfwdN.c_str(), "source"), pin(dsfwdN.c_str(), "anode")}));
        conns.push_back(conn(("sec_rect" + sfx).c_str(), {pin(srfwdN.c_str(), "drain"), pin(dsfwdN.c_str(), "cathode"),
                                                          pin(srfwN.c_str(), "drain"), pin(dsfwN.c_str(), "cathode"),
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
        gndEps.push_back(pin("T1", swE.c_str()));
        gndEps.push_back(pin(srfwN.c_str(), "source"));
        gndEps.push_back(pin(dsfwN.c_str(), "anode"));
        gateMainEps.push_back(pin(srfwdN.c_str(), "gate"));
        gateClampEps.push_back(pin(srfwN.c_str(), "gate"));
    }
    comps.push_back(comp("Csn", snb));
    comps.push_back(comp("Csn2", snb));
    gndEps.push_back(pin("Csn", "2"));
    gndEps.push_back(pin("Csn2", "2"));
    gndEps.push_back(prt("gnd"));
    gateMainEps.push_back(prt("gate_main"));
    gateClampEps.push_back(prt("gate_clamp"));
    conns.push_back(conn("gnd_net", gndEps));
    conns.push_back(conn("gate_main_net", gateMainEps));
    conns.push_back(conn("gate_clamp_net", gateClampEps));

    json cell; cell["name"] = "acf-cell";
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
        pstage("acfCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("acfCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("acfCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("acfCell", "vout"), sp("filter", "in")})};
    for (size_t i = 1; i < nOut; ++i) {
        const std::string g = "Vout" + std::to_string(i + 1), pt = "vout" + std::to_string(i + 1);
        iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("acfCell", pt.c_str())}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Main switch Q1 (duty D, phase 0); clamp switch Sc complementary (on during the reset interval),
    // a dead-band after Q1 turns off and trimmed not to wrap past the period.
    const double D = d.dutyCycle, dt = d.deadFraction;
    auto stim = [&](const char* sw, const char* sig, double duty, double phaseDeg) {
        json st; st["stage"] = "acfCell"; st["component"] = sw; st["signal"] = sig;
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fsw;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("Q1", "gate", D, 0.0),
        stim("Sc", "gate", (1.0 - D) - 2.0 * dt, (D + dt) * 360.0)});
    req::finalize_control_seeds(tas, Topology::ACTIVE_CLAMP_FORWARD_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
