#include "Dab.hpp"
#include "Dimension.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }

// SPS (Single-Phase-Shift) modulation: only the inter-bridge outer shift D3 is non-zero (D1=D2=0).
// MKF's process_design_requirements picks D3 ≈ 25° (good controllability margin) when no series
// inductance / phase is specified, then solves L for the rated power at that D3. We reproduce that
// exact choice so Kirchhoff designs the same N / L / Lm as MKF.
constexpr double kD3Deg       = 25.0;
// Per-switch on-fraction = (halfPeriod − deadTime)/period. MINIMAL dead time (~20 ns at 100 kHz — the
// least that prevents ideal-bridge shoot-through and keeps ngspice convergent). Real designs minimise
// dead time; the body-diode conduction a LARGE dead time causes is exactly what pulls the open-loop
// output below the lossless SPS target, so with minimal dead time the DAB delivers spec with no fudge.
constexpr double kSwitchDuty  = 0.499;
} // namespace

DabDesign design_dab(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    DabDesign d{};
    const json config = cfg::object_of(tasInputs);
    const double d3deg = cfg::get(config, "dabPhaseShiftDeg", kD3Deg);
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
    const double P = d.outputPower;
    const double D3 = d3deg * M_PI / 180.0;

    // 1. Turns ratio N = V1_nom / V2_nom (MKF rounds to 2 decimals).
    double N = Vin / Vo;
    d.turnsRatio = std::round(N * 100.0) / 100.0;

    // 2. Series inductance L for the rated power at D3 = 25° (SPS):
    //    L = N·V1·V2·D3·(π−|D3|) / (2π²·Fs·P).  (MKF Dab::compute_series_inductance — the exact ideal SPS
    //    power, not FHA.) With minimal dead time (above) the open-loop output lands on spec; no derating.
    d.seriesInductance = N * Vin * Vo * D3 * (M_PI - std::abs(D3)) / (2.0 * M_PI * M_PI * Fs * P);

    // 3. Magnetizing inductance: max(Vin²/(1.2·Fs·P), 10·L) — 30% magnetizing-ripple target, floored
    //    at 10× the series inductance (MKF Dab::process_design_requirements step 4).
    double LmFromCurrent = Vin * Vin / (1.2 * Fs * P);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        std::max(LmFromCurrent, 10.0 * d.seriesInductance));

    d.phaseShiftDeg = d3deg;
    d.switchDuty = cfg::get(config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Vo * Vo / P;
    d.config = config;
    d.outputCapacitance = cfg::get(config, "outputCapacitance", 100e-6);  // DAB has no output L; MKF uses 100u
    return d;
}

json build_dab_tas(const DabDesign& d) {
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

    // Series (resonant + leakage) inductor Lr — single-winding magnetic.
    json lr; lr["magnetic"] = json::object();
    lr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = d.seriesInductance;
    lr["inputs"]["designRequirements"]["turnsRatios"] = json::array();

    // 2-winding transformer (primary + one secondary), turnsRatios = [N]. K=0.9999 (MAS default) so the
    // coupled-L matrix is non-singular with Lr in series (the whole reason DAB needs K<1).
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"]["designRequirements"]["magnetizingInductance"]["nominal"] = Lm;
    { json rn; rn["nominal"] = N; xfmr["inputs"]["designRequirements"]["turnsRatios"] = json::array({rn}); }

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // Per-switch R∥C across every switch's two power terminals (8 switches) — but the R and C play DIFFERENT
    // roles, so they're named (and handled) differently:
    //   Csn* = a dV/dt SNUBBER: tames the floating-midpoint dV/dt at each switching/dead-time event. C =
    //          energy-budget rule at V_block=Vin. The deck's real-fidelity path STRIPS it (a real switch's
    //          Coss does this physically) — hence the "Csn" snubber name.
    //   Rbias* = a FUNCTIONAL bias resistor: the DC path that DEFINES the floating midpoint during the dead
    //          time. R = bias-loss rule (<= biasLossFrac of rated P at Vin); a small R would bleed Vin²/R
    //          (~kW at 800 V), starving Vout. A switch's Coss does NOT provide a DC bias path, so this is
    //          NOT a snubber — named "Rbias" precisely so the snubber strip never removes it.
    const double snubCval = cfg::snubber_cap(d.config, d.outputPower, d.inputVoltage, d.switchingFrequency);
    const double snubRval = cfg::bias_res(d.config, d.inputVoltage, d.outputPower);
    auto biasR = [&]() { json c; c["resistor"] = json::object();
        c["inputs"]["designRequirements"]["deviceType"] = "resistor";
        c["inputs"]["designRequirements"]["resistance"]["nominal"] = snubRval; return c; };
    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = snubCval;
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    json cell; cell["name"] = "dab-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("gateA"), port("gateB"), port("gateC"), port("gateD"),
                                 port("gateE"), port("gateF"), port("gateG"), port("gateH")});
    cell["components"] = json::array({
        // primary bridge + body diodes
        comp("QA", mosfet()), comp("QB", mosfet()), comp("QC", mosfet()), comp("QD", mosfet()),
        comp("DA", diode()),  comp("DB", diode()),  comp("DC", diode()),  comp("DD", diode()),
        // secondary active bridge + body diodes
        comp("QE", mosfet()), comp("QF", mosfet()), comp("QG", mosfet()), comp("QH", mosfet()),
        comp("DE", diode()),  comp("DF", diode()),  comp("DG", diode()),  comp("DH", diode()),
        comp("Lr", lr), comp("T1", xfmr), comp("Cout", capd),
        // per-switch RC snubbers (hi = across top switch, lo = across bottom switch)
        comp("RbiasA_hi", biasR()), comp("CsnA_hi", snubC()), comp("RbiasA_lo", biasR()), comp("CsnA_lo", snubC()),
        comp("RbiasC_hi", biasR()), comp("CsnC_hi", snubC()), comp("RbiasC_lo", biasR()), comp("CsnC_lo", snubC()),
        comp("RbiasE_hi", biasR()), comp("CsnE_hi", snubC()), comp("RbiasE_lo", biasR()), comp("CsnE_lo", snubC()),
        comp("RbiasG_hi", biasR()), comp("CsnG_hi", snubC()), comp("RbiasG_lo", biasR()), comp("CsnG_lo", snubC())});
    cell["connections"] = json::array({
        // ── Primary full bridge. QA/QC high-side (vin->mid), QB/QD low-side (mid->gnd); anti-parallel
        // body diodes DA..DD freewheel/clamp the floating midpoints during the leg dead time.
        conn("vin_net",  {pin("QA", "drain"), pin("QC", "drain"),
                          pin("DA", "cathode"), pin("DC", "cathode"),
                          pin("RbiasA_hi", "1"), pin("CsnA_hi", "1"),
                          pin("RbiasC_hi", "1"), pin("CsnC_hi", "1"), prt("vin")}),
        conn("midA_net", {pin("QA", "source"), pin("QB", "drain"),
                          pin("DA", "anode"), pin("DB", "cathode"), pin("Lr", "primary_start"),
                          pin("RbiasA_hi", "2"), pin("CsnA_hi", "2"),
                          pin("RbiasA_lo", "1"), pin("CsnA_lo", "1")}),
        conn("midC_net", {pin("QC", "source"), pin("QD", "drain"),
                          pin("DC", "anode"), pin("DD", "cathode"), pin("T1", "primary_end"),
                          pin("RbiasC_hi", "2"), pin("CsnC_hi", "2"),
                          pin("RbiasC_lo", "1"), pin("CsnC_lo", "1")}),
        conn("pri_x",    {pin("Lr", "primary_end"), pin("T1", "primary_start")}),
        // ── Secondary active bridge. QE/QG high-side (vout->sec), QF/QH low-side (sec->gnd); body
        // diodes DE..DH freewheel during the secondary leg dead time. The bridge is driven (not a
        // passive rectifier): the D3 phase shift vs the primary sets the transferred power.
        conn("sec_a",    {pin("T1", "secondary1_start"), pin("QE", "source"), pin("QF", "drain"),
                          pin("DE", "anode"), pin("DF", "cathode"),
                          pin("RbiasE_hi", "2"), pin("CsnE_hi", "2"),
                          pin("RbiasE_lo", "1"), pin("CsnE_lo", "1")}),
        conn("sec_b",    {pin("T1", "secondary1_end"), pin("QG", "source"), pin("QH", "drain"),
                          pin("DG", "anode"), pin("DH", "cathode"),
                          pin("RbiasG_hi", "2"), pin("CsnG_hi", "2"),
                          pin("RbiasG_lo", "1"), pin("CsnG_lo", "1")}),
        conn("vout_net", {pin("QE", "drain"), pin("QG", "drain"),
                          pin("DE", "cathode"), pin("DG", "cathode"), pin("Cout", "1"),
                          pin("RbiasE_hi", "1"), pin("CsnE_hi", "1"),
                          pin("RbiasG_hi", "1"), pin("CsnG_hi", "1"), prt("vout")}),
        // ── Shared ground: all four low-side switch sources + their body-diode anodes + Cout return +
        // all low-side snubber returns. Secondary return tied to primary gnd (sim convenience; the
        // transformer provides isolation — MKF also references both bridges to 0).
        conn("gnd_net",  {pin("QB", "source"), pin("QD", "source"), pin("QF", "source"), pin("QH", "source"),
                          pin("DB", "anode"), pin("DD", "anode"), pin("DF", "anode"), pin("DH", "anode"),
                          pin("Cout", "2"),
                          pin("RbiasA_lo", "2"), pin("CsnA_lo", "2"), pin("RbiasC_lo", "2"), pin("CsnC_lo", "2"),
                          pin("RbiasE_lo", "2"), pin("CsnE_lo", "2"), pin("RbiasG_lo", "2"), pin("CsnG_lo", "2"),
                          prt("gnd")}),
        conn("gateA_net", {pin("QA", "gate"), prt("gateA")}),
        conn("gateB_net", {pin("QB", "gate"), prt("gateB")}),
        conn("gateC_net", {pin("QC", "gate"), prt("gateC")}),
        conn("gateD_net", {pin("QD", "gate"), prt("gateD")}),
        conn("gateE_net", {pin("QE", "gate"), prt("gateE")}),
        conn("gateF_net", {pin("QF", "gate"), prt("gateF")}),
        conn("gateG_net", {pin("QG", "gate"), prt("gateG")}),
        conn("gateH_net", {pin("QH", "gate"), prt("gateH")})});

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
        pstage("dabCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("dabCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("dabCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("dabCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Eight PWM drives. Primary: diagonal pairs (QA,QD) at 0° and (QB,QC) at 180° -> ±Vin square wave.
    // Secondary: the same square wave phase-shifted by D3 — diagonal pairs (QE,QH) at D3 and (QF,QG)
    // at D3+180°. The inter-bridge phase D3 drives the power transfer (SPS modulation).
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "dabCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phaseDeg"] = phaseDeg;
        return st; };
    const double p3 = d.phaseShiftDeg;
    tas["simulation"]["stimulus"] = json::array({
        stim("QA", 0.0),       stim("QB", 180.0),       stim("QC", 180.0),       stim("QD", 0.0),
        stim("QE", p3),        stim("QF", 180.0 + p3),  stim("QG", 180.0 + p3),  stim("QH", p3)});
    return tas;
}

} // namespace Kirchhoff
