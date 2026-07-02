#include "Dab.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
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
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(N * 100.0) / 100.0);
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
    auto mosfet = [](json reqs = json()) { json j; j["semiconductor"]["mosfet"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };
    auto diode  = [&](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs.is_null()
            ? req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage) : reqs; return j; };

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, Lr_H = d.seriesInductance;

    // --- SPS tank-current stress (the current shared by Lr and the transformer primary) ---
    // SPS inductor current is the piecewise-linear (trapezoidal) tank current. Primary-referred levels:
    // V1 = Vin, V2' = N·Vout; the inductor sees (V1+V2') for the phase-shift fraction φ then (V1−V2') for
    // the rest of the half-period, sign-symmetric. di/dθ = v_L/(ωL). Extrema (half-wave symmetric, so
    // i(π) = −i(0)):
    const double w = 2.0 * M_PI * fsw;                            // electrical angular frequency
    const double phi = d.phaseShiftDeg * M_PI / 180.0;           // outer phase shift D3 (rad)
    const double V1t = d.inputVoltage, V2t = N * d.outputVoltage; // primary-referred bridge square levels
    const double i0  = -(V1t * M_PI + V2t * (2.0 * phi - M_PI)) / (2.0 * w * Lr_H);  // i_L at θ=0
    const double iphi = i0 + (V1t + V2t) * phi / (w * Lr_H);                          // i_L at θ=φ
    const double IpkTank = std::max(std::abs(i0), std::abs(iphi));
    // RMS over a half period of the two linear segments [i0→iphi over φ] and [iphi→−i0 over π−φ]:
    auto segSq = [](double a, double b, double len) { return len * (a * a + a * b + b * b) / 3.0; };
    const double IrmsTank = std::sqrt((segSq(i0, iphi, phi) + segSq(iphi, -i0, M_PI - phi)) / M_PI);
    const double dITank = std::abs(iphi - i0);                   // pk-pk swing within a half-period

    // Inductor voltage levels (square, sign-symmetric); RMS = sqrt(d·(V1+V2')² + (1−d)·(V1−V2')²).
    const double dFrac = phi / M_PI;
    const double vLrPk = std::max(std::abs(V1t + V2t), std::abs(V1t - V2t)), vLrPkPk = 2.0 * vLrPk;
    const double vLrRms = std::sqrt(dFrac * (V1t + V2t) * (V1t + V2t) + (1.0 - dFrac) * (V1t - V2t) * (V1t - V2t));

    // Series (resonant + leakage) inductor Lr — single-winding magnetic carrying the full tank current.
    json lr; lr["magnetic"] = json::object();
    lr["inputs"] = req::magnetic_inputs(Lr_H, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkTank, IrmsTank, 0.0, dITank, std::nullopt,
                                    vLrPk, vLrRms, 0.0, vLrPkPk)});

    // 2-winding transformer (primary + one secondary), turnsRatios = [N]. K=0.9999 (MAS default) so the
    // coupled-L matrix is non-singular with Lr in series (the whole reason DAB needs K<1).
    // Primary winding carries the tank current (Lr is in series with it); secondary current is the tank
    // current reflected by N (ampere-turns balance). Bridge windings see ±square voltages: primary ±Vin,
    // secondary ±Vout. No DC bias (AC-driven). Galvanic-only isolation (no isolationVoltage on spec).
    const double IpkSec = IpkTank * N, IrmsSec = IrmsTank * N;  // secondary bridge stresses (reflected tank current)

    // --- semiconductor stresses: primary bridge blocks Vin, secondary bridge blocks Vout ---
    // QA..QD (primary full bridge) block the DC-link Vin_max and carry the tank current (IrmsTank).
    // QE..QH (secondary active bridge) block Vout and carry the reflected tank current (IrmsSec).
    // All eight are real, independently-driven switches (no passive rectifier — the secondary is a driven
    // bridge, the phase shift sets the power transfer); their anti-parallel diodes DA..DH are BODY diodes.
    const double ratedVdsPri = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double ratedVdsSec = d.outputVoltage  / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnPri = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsTank * IrmsTank);
    const double maxRdsOnSec = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsSec  * IrmsSec);
    // Transformer (T1) EMBEDDED excitations from the SINGLE FHA source — the SPICE-validated analytical
    // DAB solver (SPS modulation: inner shifts D1=D2=0, outer D3=phaseShiftDeg). It emits exactly 2
    // windings (primary + one secondary), matching this 2-winding bridge transformer; the series inductor
    // Lr and the switch ratings keep the inline tank stresses above (Lr is not a transformer winding).
    // The analytical primary current = tank iL + magnetizing Im (more complete than the tank-only inline
    // value). Evaluated at nominal Vin — the corner the inline tank ratings already used.
    namespace AN = Kirchhoff::analytical;
    const double IoutDab = d.outputPower / d.outputVoltage;
    const MAS::OperatingPoint aopT1 = AN::analytical_dab(d.inputVoltage, {d.outputVoltage}, {IoutDab}, {N},
                                                         fsw, Lm, Lr_H, 0.0, 0.0, d.phaseShiftDeg);
    const std::vector<json> windings = AN::excitations_processed(aopT1, "T1");
    std::vector<std::string> isoSides{"primary", "secondary"};
    json xfmr; xfmr["magnetic"] = json::object();
    // NB: DAB keeps its DESIGNED (nominal) Lm — unlike the other transformers it ties Lm to the series
    // inductor (Lm = max(.., 10*Lr)) for the magnetizing-ripple/ZVS, so maximising Lm (lmIsMinimum)
    // detunes its operating point and it stops regulating (verified). abt #56.
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {N}, isoSides, std::nullopt, 25.0, windings);

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
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = snubRval;
        dr["powerRating"] = d.inputVoltage * d.inputVoltage / snubRval;  // bias bleed: P = V^2/R
        dr["role"] = "bleed"; return c; };
    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = snubCval;
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    json cell; cell["name"] = "dab-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("gateA"), port("gateB"), port("gateC"), port("gateD"),
                                 port("gateE"), port("gateF"), port("gateG"), port("gateH")});
    auto mosfetPri = [&]() { return mosfet(req::mosfet("mainSwitch", ratedVdsPri, IpkTank, maxRdsOnPri, 125.0)); };
    auto mosfetSec = [&]() { return mosfet(req::mosfet("mainSwitch", ratedVdsSec, IpkSec,  maxRdsOnSec, 125.0)); };
    cell["components"] = json::array({
        // primary bridge + (requirement-less) anti-parallel BODY diodes
        comp("QA", mosfetPri()), comp("QB", mosfetPri()), comp("QC", mosfetPri()), comp("QD", mosfetPri()),
        comp("DA", diode()),  comp("DB", diode()),  comp("DC", diode()),  comp("DD", diode()),
        // secondary active bridge + (requirement-less) anti-parallel BODY diodes
        comp("QE", mosfetSec()), comp("QF", mosfetSec()), comp("QG", mosfetSec()), comp("QH", mosfetSec()),
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
        req::control_stage("pwmController"),
        req::control_stage("gateDriver", "gate-driver", "UDR"),
        pstage("dabCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("dabCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("dabCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("dabCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Eight PWM drives. Primary: diagonal pairs (QA,QD) at 0° and (QB,QC) at 180° -> ±Vin square wave.
    // Secondary: the same square wave phase-shifted by D3 — diagonal pairs (QE,QH) at D3 and (QF,QG)
    // at D3+180°. The inter-bridge phase D3 drives the power transfer (SPS modulation).
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "dabCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    const double p3 = d.phaseShiftDeg;
    tas["simulation"]["stimulus"] = json::array({
        stim("QA", 0.0),       stim("QB", 180.0),       stim("QC", 180.0),       stim("QD", 0.0),
        stim("QE", p3),        stim("QF", 180.0 + p3),  stim("QG", 180.0 + p3),  stim("QH", p3)});
    req::finalize_control_seeds(tas, Topology::DUAL_ACTIVE_BRIDGE_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
