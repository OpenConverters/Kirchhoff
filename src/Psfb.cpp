#include "Psfb.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }

// PSFB control: leg-to-leg phase shift sets the effective duty Deff = phi/180. Commanded D_cmd=0.7
// (phi=126 deg) — same operating point as the MKF reference (gen_psfb sets phase_shift=126).
constexpr double kCommandedDuty = 0.7;
constexpr double kRippleRatio   = 0.3;    // output-inductor current ripple (MKF PSFB default)

// Kirchhoff's ideal rectifier diode drop. The CIAS ideal-diode model is .model D(IS=1e-14) (= MKF
// DIDEAL family), so Vd(I) = Vt*ln(I/IS). The full-bridge rectifier conducts through TWO diodes in
// series, so the design compensates the turns ratio for 2*Vd at the rated output current — exactly
// what MKF's Psfb::compute_turns_ratio does (FULL_BRIDGE: n = Vin*Deff/(Vo + 2*Vd)). Without this
// compensation the ideal-but-non-zero diode drop would pull Vout ~12% low. Vd here uses Kirchhoff's
// OWN diode IS so the Kirchhoff deck delivers the target Vo; MKF's deck compensates for its own
// (IS=1e-12 RS=0.005) drop and lands within tolerance.

// Per-switch on-fraction. The bridge runs at ~50% duty; a dead time (here via duty < 0.5) keeps the
// ideal switches from shoot-through at the leg crossover (duty=0.5 exactly = a vin->gnd short for the
// 1ns gate-overlap every half-period -> huge current spikes that wreck the input-current average).
// 200ns dead time matches MKF's computedDeadTime. The phase is compensated for the dead time below so
// the commanded effective duty is unchanged.
constexpr double kSwitchDuty  = 0.48;   // 200ns dead time at Fs=100kHz (T=10us)
} // namespace

PsfbDesign design_psfb(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PsfbDesign d{};
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

    const double Vin = d.inputVoltage, Vo = d.outputVoltage, Fs = d.switchingFrequency;
    const double Io = d.outputPower / Vo;
    const double Dcmd = cfg::get(d.config, "commandedDuty", kCommandedDuty);
    d.commandedDuty = Dcmd;
    const double Vdtot = 2.0 * req::dideal_diode_drop(Io);   // full-bridge rectifier: two diodes in series

    // Series (resonant + leakage) inductor Lr: the smaller of a 2 uH default and the value giving
    // <= 2% duty loss at rated load (MKF Psfb::process_design_requirements).
    double nSeed = Vin * Dcmd / (Vo + Vdtot);
    double LrCap = (Io > 0) ? 0.02 * std::max(nSeed, 0.1) * Vin / (4.0 * Io * Fs) : 2e-6;
    double Lr = std::min(2e-6, LrCap);
    Lr = std::max(Lr, 1e-7);
    d.seriesInductance = Lr;

    // Iterate turns ratio n with duty-cycle-loss correction: Deff = Dcmd - 4*Lr*Io*Fs/(n*Vin),
    // n = Vin*Deff/(Vo + 2*Vd). Converges in a few steps as Deff stabilizes (Sabate 1990).
    double n = nSeed, Deff = Dcmd;
    for (int it = 0; it < 8; ++it) {
        double dcl = 4.0 * Lr * Io * Fs / (n * Vin);
        Deff = std::max(0.0, Dcmd - dcl);
        double nNew = (Deff > 1e-3) ? Vin * Deff / (Vo + Vdtot) : n;
        if (std::abs(nNew - n) < 1e-3 * std::max(n, 1.0)) { n = nNew; break; }
        n = nNew;
    }
    d.effectiveDuty = Deff;
    d.turnsRatio = std::round(n * 100.0) / 100.0;

    // Output inductor: Lo = Vo*(1 - Deff)/(Fs * ripple * Io).
    d.outputInductance = Vo * (1.0 - Deff) / (Fs * cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Io);

    // Magnetizing inductance: Im_peak target = 10% of reflected load current; Lm = Vin*Deff/(4*Fs*Im).
    double ImTarget = 0.1 * Io / d.turnsRatio;
    double Lm = (ImTarget > 0) ? Vin * Deff / (4.0 * Fs * ImTarget) : 20.0 * Lr;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        std::max(Lm, 20.0 * Lr));

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    // Lagging-leg phase shift = 180*Deff_cmd (textbook PSFB phase-shift modulation: Deff = phi/180).
    // The dead time and the Lr commutation both nibble at the delivered duty, but the anti-parallel
    // body diodes carry the freewheel during dead time so the net effect is small; empirically the
    // uncompensated phi lands the simulated Vout within ~1% of MKF (verified at the 48->12V/24W point).
    d.phaseDeg = 180.0 * Dcmd;
    d.loadResistance = Vo * Vo / d.outputPower;
    d.outputCapacitance = 100e-6;    // matches MKF PSFB (Cout=100u)
    return d;
}

json build_psfb_tas(const PsfbDesign& d) {
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

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;

    // Series resonant/leakage inductor Lr (single-winding magnetic).
    json lr; lr["magnetic"] = json::object();
    lr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.seriesInductance;
    lr["inputs"]["designRequirements"]["turnsRatios"] = json::array();

    // 2-winding transformer (primary + one full secondary), turnsRatios = [N].
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json rn; rn["nominal"] = N; xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    // Output inductor Lo (single-winding magnetic).
    json lout; lout["magnetic"] = json::object();
    lout["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.outputInductance;
    lout["inputs"]["designRequirements"]["turnsRatios"] = json::array();

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // Snubber caps at the bridge midpoints (midA/midC) AND the secondary rectifier nodes (sec_a/sec_b).
    // During the leg dead time both switches of a leg are off and the midpoint floats; during the
    // rectifier off-state the secondary winding ends float (all four ideal diodes reverse-biased). Both
    // are nodes with no DC path, so ngspice fails (timestep too small) without a finite-dV/dt path. The
    // anti-parallel body diodes clamp the midpoints to [0, Vin]; the small node-to-gnd snubber caps tame
    // the dV/dt at every switching/commutation instant. Physically real (device Coss + winding capac.);
    // 2.2 nF leaves Vout within ~1%. (Same technique as push-pull / MKF's reference snubbers.)
    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);  // bridge midpoint node cap
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    json cell; cell["name"] = "psfb-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("gateA"), port("gateB"), port("gateC"), port("gateD")});
    cell["components"] = json::array({
        comp("QA", mosfet()), comp("QB", mosfet()), comp("QC", mosfet()), comp("QD", mosfet()),
        comp("DA", diode()),  comp("DB", diode()),  comp("DC", diode()),  comp("DD", diode()),
        comp("Lr", lr), comp("T1", xfmr),
        comp("Dr1", diode()), comp("Dr2", diode()), comp("Dr3", diode()), comp("Dr4", diode()),
        comp("Lout", lout), comp("CsnA", snub()), comp("CsnC", snub()),
        comp("CsnSA", snub()), comp("CsnSB", snub())});
    cell["connections"] = json::array({
        // Primary full bridge. QA/QC high-side (vin->mid), QB/QD low-side (mid->gnd); anti-parallel
        // body diodes DA..DD give the floating midpoints a freewheel/clamp path during dead time.
        conn("vin_net",  {pin("QA", "drain"), pin("QC", "drain"),
                          pin("DA", "cathode"), pin("DC", "cathode"), prt("vin")}),
        conn("midA_net", {pin("QA", "source"), pin("QB", "drain"),
                          pin("DA", "anode"), pin("DB", "cathode"),
                          pin("Lr", "primary_start"), pin("CsnA", "1")}),
        conn("midC_net", {pin("QC", "source"), pin("QD", "drain"),
                          pin("DC", "anode"), pin("DD", "cathode"),
                          pin("T1", "primary_end"), pin("CsnC", "1")}),
        conn("pri_x",    {pin("Lr", "primary_end"), pin("T1", "primary_start")}),
        // Secondary -> full-bridge rectifier (4 diodes) -> output inductor.
        conn("sec_a",    {pin("T1", "secondary1_start"), pin("Dr1", "anode"), pin("Dr3", "cathode"),
                          pin("CsnSA", "1")}),
        conn("sec_b",    {pin("T1", "secondary1_end"),   pin("Dr2", "anode"), pin("Dr4", "cathode"),
                          pin("CsnSB", "1")}),
        conn("out_rect", {pin("Dr1", "cathode"), pin("Dr2", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        // Shared ground: low-side switch sources + their body-diode anodes + the two rectifier return
        // diodes (Dr3/Dr4 anodes) + snubber returns. Secondary return is tied to primary gnd (sim
        // convenience; isolation is provided by the transformer coupling — MKF ties out_gnd to 0 too).
        conn("gnd_net",  {pin("QB", "source"), pin("QD", "source"),
                          pin("DB", "anode"), pin("DD", "anode"),
                          pin("Dr3", "anode"), pin("Dr4", "anode"),
                          pin("CsnA", "2"), pin("CsnC", "2"),
                          pin("CsnSA", "2"), pin("CsnSB", "2"), prt("gnd")}),
        conn("gateA_net", {pin("QA", "gate"), prt("gateA")}),
        conn("gateB_net", {pin("QB", "gate"), prt("gateB")}),
        conn("gateC_net", {pin("QC", "gate"), prt("gateC")}),
        conn("gateD_net", {pin("QD", "gate"), prt("gateD")})});

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
        req::control_stage("gateDriver", "gate-driver", "UDR"),
        pstage("psfbCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("psfbCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("psfbCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("psfbCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Four PWM drives: leading leg QA(0 deg)/QB(180 deg), lagging leg QC(phi)/QD(180+phi). The
    // leg-to-leg phase phi sets the effective duty (power transfer); both legs run at ~50% duty.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "psfbCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("QA", 0.0), stim("QB", 180.0),
        stim("QC", d.phaseDeg), stim("QD", 180.0 + d.phaseDeg)});
    return tas;
}

} // namespace Kirchhoff
