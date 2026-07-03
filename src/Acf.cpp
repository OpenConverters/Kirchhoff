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
    const double n = d.turnsRatio, Lm = d.magnetizingInductance, fsw = d.switchingFrequency;
    const double T = 1.0 / fsw, Dn = d.dutyCycle, Vin = d.inputVoltage, Vout = d.outputVoltage;
    const double iout = d.outputPower / Vout;
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);

    // --- MAIN magnetic (the 2-winding transformer T1) sourced from the SINGLE FHA solver ---
    // analytical_active_clamp_forward returns EXACTLY the two transformer windings — "First primary" +
    // "Secondary 0" — matching T1's turnsRatios = [n]. The output-filter inductor Lout is a SECOND magnetic
    // (NOT one of the solver's windings) and keeps its inline excitation; the magnetizing reset current
    // (clamp-switch rating) also stays inline (the solver folds it into the combined primary winding, so
    // there is no separate magnetizing winding to read). Worst-case corner (Vin_min) drives the main-switch
    // rating; the declared nominal OP is what the TAS embeds. (ACF sizes n at Vin_min with Dmax, so the
    // solver's t1 = Dmax·period < period/2 at Vin_min and is even smaller at Vin_nom — safe at both corners.)
    const MAS::OperatingPoint aopWorst = AN::analytical_active_clamp_forward(d.inputVoltageMin, {Vout}, {iout},
                                            {n}, fsw, Lm, d.outputInductance, ripple, Dn, d.diodeDrop);
    const MAS::OperatingPoint aopNom   = AN::analytical_active_clamp_forward(Vin, {Vout}, {iout},
                                            {n}, fsw, Lm, d.outputInductance, ripple, Dn, d.diodeDrop);
    const double IpkPri  = AN::winding_current(aopWorst, 0, "peak");   // primary peak (main-switch rating)
    const double IrmsPri = AN::winding_current(aopWorst, 0, "rms");    // primary rms (main-switch RdsOn conduction)
    const double ImagPk  = Vin * Dn * T / Lm;                          // magnetizing reset peak (clamp-switch rating)
    // Clamp switch conducts during the RESET interval (1-Dn)*T, carrying the magnetizing triangle from
    // ImagPk down to 0; its full-period rms is therefore ImagPk*sqrt((1-Dn)/3), NOT sqrt(Dn/3) (the ON
    // interval, when the magnetizing current flows through the MAIN switch, not the clamp).
    const double ImagRms = ImagPk * std::sqrt((1.0 - Dn) / 3.0);      // magnetizing reset rms  (clamp-switch RdsOn)

    // Output filter inductor Lout (single winding) — inline excitation (not one of the solver's windings).
    const double dILout  = (Vin / n - d.diodeDrop - Vout) * (Dn * T) / d.outputInductance;
    const double IpkLout = iout + dILout / 2.0;
    const double IrmsLout = std::sqrt(iout * iout + dILout * dILout / 12.0);
    const double vLonF = Vin / n - d.diodeDrop - Vout, vLoff = Vout;
    const double vLoutPk = std::max(std::abs(vLonF), vLoff), vLoutPkPk = std::abs(vLonF) + vLoff;
    const double vLoutRms = std::sqrt(Dn * vLonF * vLonF + (1.0 - Dn) * vLoff * vLoff);

    // --- semiconductor stresses (active-clamp forward) ---
    // Both the main switch Q1 and the clamp switch Sc sit across (Vin + Vclamp): when off, the node
    // they share swings to the clamp level Vin+Vreset. Evaluated at the max-input corner:
    //   VdsStress = Vin_max + Vin_max*D/(1-D)   (Vreset at the worst-case rail).
    const double VresetMax = d.inputVoltageMax * Dn / (1.0 - Dn);
    const double VdsStress = d.inputVoltageMax + VresetMax;
    const double ratedVds  = VdsStress / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnMain  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsPri * IrmsPri);
    // Clamp switch carries the magnetizing reset current (peak ImagPk, rms ImagRms); size its RdsOn to
    // its own conduction loss budget. It is a REAL FET (req::mosfet), not a body diode.
    const double maxRdsOnClamp = (ImagRms > 0.0)
        ? cfg::rds_on_loss_fraction(d.config) * d.outputPower / (ImagRms * ImagRms)
        : maxRdsOnMain;
    // Output SYNCHRONOUS rectifiers (SRfwd forward + SRfw freewheel): gated MOSFETs, not diodes. At a
    // low-Vout/high-current rail (e.g. 3.3 V / 30 A) the ideal-diode Vf (~0.92 V) was ~28% of the output on
    // the rectifier alone, so the ideal deck looked implausibly lossy; an SR's I²·Rds is a small fraction of
    // that. Each SR reverse-blocks the reflected secondary peak Vin_max/n and carries the output-inductor
    // current (peak IpkLout, rms IrmsLout). Body diodes (DSfwd/DSfw) are bare seeds — the FET's intrinsic
    // diode — for the switching dead-time and for startup rectification before the gates take over.
    const double ratedVdsSR  = (d.inputVoltageMax / n) / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnSR  = (IrmsLout > 0.0)
        ? cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsLout * IrmsLout) : maxRdsOnMain;
    const json reqSR = req::mosfet("mainSwitch", ratedVdsSR, IpkLout, maxRdsOnSR, 125.0);
    json mainSw = mosfet();
    mainSw["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkPri, maxRdsOnMain, 125.0);
    json clampSw = mosfet();
    clampSw["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, ImagPk, maxRdsOnClamp, 125.0);
    auto srFet   = [&]() { json j = mosfet(); j["inputs"]["designRequirements"] = reqSR; return j; };
    auto bodyDio = []()  { json j; j["semiconductor"]["diode"] = json::object(); return j; };  // bare (FET body diode)

    // 2-winding transformer (primary + 1 secondary, NO demag winding — active clamp resets it) -> 2 excitations (from the solver).
    std::vector<std::string> xfmrIso{"primary", "secondary"};
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {n}, xfmrIso, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "T1"),
        // n = Vin_min·Dmax/(Vout+Vd) is a duty CEILING; active clamp resets (no demag winding), so the
        // sole secondary ratio is emitted as {maximum}. (abt #49)
        /*turnsRatioIsCeiling=*/{true});
    // Output filter inductor: single winding (turnsRatios = []) -> 1 excitation, DC-biased at Iout.
    json lout; lout["magnetic"] = json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
        req::winding_excitation("triangular", fsw, IpkLout, IrmsLout, iout, dILout, Dn,
                                vLoutPk, vLoutRms, 0.0, vLoutPkPk)});

    json cc; cc["capacitor"] = json::object();   // active-clamp capacitor
    cc["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.clampCapacitance;
    cc["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // Numerical node snubber on the switching node. The ACF is the one switching topology that shipped WITHOUT
    // one, so at high Vin / light load the active-clamp node (whose only DC path is the clamp switch ROFF)
    // produces an unresolvably fast dV/dt -> ngspice "timestep too small". A small node-to-gnd cap tames it,
    // exactly like the bridge midpoints. Named "Csn" so the real-fidelity strip replaces it with the switch's
    // Coss when a real part is bound (and the real deck's cshunt covers convergence).
    json snb; snb["capacitor"] = json::object();
    snb["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
    snb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 4;
    cfg::mark_numerical_aid(snb);   // dV/dt convergence aid — explicitly tagged for the real-fidelity strip (ABT #96)

    // Active-clamp forward cell: main switch Q1 (vin->sw), 2-winding transformer (sw->gnd primary), the
    // clamp leg (Sc: vin->clamp_node, Cc: clamp_node->sw — clamp_node gets its DC path through Sc's
    // ROFF, like MKF's 1Meg bleeder), and the forward output (SRfwd + SRfw synchronous rectifier + Lout). Dot orientation
    // matches MKF (primary & secondary in-phase: primary_start=sw, secondary1_start=sec_in).
    json cell; cell["name"] = "acf-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate_main"), port("gate_clamp")});
    cell["components"] = json::array({comp("Q1", mainSw), comp("Sc", clampSw), comp("Cc", cc),
                                      comp("T1", xfmr),
                                      comp("SRfwd", srFet()), comp("DSfwd", bodyDio()),
                                      comp("SRfw", srFet()),  comp("DSfw", bodyDio()),
                                      comp("Lout", lout), comp("Csn", snb), comp("Csn2", snb)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), pin("Sc", "drain"), prt("vin")}),
        conn("sw_node",  {pin("Q1", "source"), pin("T1", "primary_start"), pin("Cc", "2"), pin("Csn", "1")}),
        // clamp_node is the stiff one (its only DC path is the clamp switch ROFF); a node snubber there clears
        // the last high-Vin/light-load/high-fsw corner. Both Csn* strip together for a real-fidelity deck.
        conn("clamp_node", {pin("Sc", "source"), pin("Cc", "1"), pin("Csn2", "1")}),
        // secondary -> SYNCHRONOUS forward rectifier (SRfwd) + freewheel SR (SRfw) -> output inductor.
        // Body-diode orientation matches the diodes they replace: DSfwd conducts sec_in->sec_rect (so
        // SRfwd source=sec_in, drain=sec_rect); DSfw conducts gnd->sec_rect (SRfw source=gnd, drain=sec_rect).
        conn("sec_in",   {pin("T1", "secondary1_start"), pin("SRfwd", "source"), pin("DSfwd", "anode")}),
        conn("sec_rect", {pin("SRfwd", "drain"), pin("DSfwd", "cathode"),
                          pin("SRfw", "drain"),  pin("DSfw", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("T1", "primary_end"), pin("T1", "secondary1_end"),
                          pin("SRfw", "source"), pin("DSfw", "anode"),
                          pin("Csn", "2"), pin("Csn2", "2"), prt("gnd")}),
        // SRfwd gated synchronous with Q1 (conducts during the power-transfer interval D); SRfw with the
        // clamp switch Sc (on during exactly the reset/freewheel interval 1-D) — no extra stimulus needed.
        conn("gate_main_net",  {pin("Q1", "gate"), pin("SRfwd", "gate"), prt("gate_main")}),
        conn("gate_clamp_net", {pin("Sc", "gate"), pin("SRfw", "gate"),  prt("gate_clamp")})});

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
        pstage("acfCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("acfCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("acfCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("acfCell", "vout"), sp("filter", "in")})});

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
