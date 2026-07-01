#include "Psfb.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <stdexcept>

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
    // Rectifier variant (FB default — MKF's CT deck is a fake CT; Kirchhoff's CT uses two REAL secondary
    // half-windings so it delivers full Vout). Selected from the config override; no schema change.
    d.rectifierType = parse_rectifier_type(cfg::get_str(d.config, "rectifierType", "fullBridge"),
                                           RectifierType::FullBridge);
    if (d.rectifierType == RectifierType::VoltageDoubler)
        throw std::runtime_error("Kirchhoff PSFB: voltageDoubler rectifier not supported "
                                 "(PSFB variants: fullBridge, centerTapped, currentDoubler)");
    // Diodes in the conduction path set the turns-ratio drop compensation: FB stacks two (Vo+2Vd); the
    // center-tapped and current-doubler secondaries conduct through one at a time (Vo+Vd).
    const double Vdtot = rectifier_path_diodes(d.rectifierType) * req::dideal_diode_drop(Io);

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
    // CURRENT_DOUBLER delivers ~half the winding-reflected voltage (the two inductors each conduct half
    // the period), so the turns ratio must be halved to hit the same Vo. cdOutputFactor (~0.5) is
    // documented + config-overridable (rectifier conduction nudges the realized factor slightly).
    if (d.rectifierType == RectifierType::CurrentDoubler)
        n *= cfg::get(d.config, "cdOutputFactor", 0.5);
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(n * 100.0) / 100.0);
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
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, Tsw = 1.0 / fsw;
    const double Vo = d.outputVoltage, Io = d.outputPower / Vo;
    const double Vin = d.inputVoltage, Deff = d.effectiveDuty;

    // --- stresses ---
    // Output inductor (CCM): avg=Io, ripple from the Lo sizing volt-seconds.
    const double dILo = Vo * (1.0 - Deff) / (d.outputInductance * fsw);   // pk-pk (= ripple sizing)
    const double IloPk  = Io + dILo / 2.0;
    const double IloRms = std::sqrt(Io * Io + dILo * dILo / 12.0);
    const double vLoPk = std::max(std::abs(Vin / N - Vo), Vo), vLoPkPk = Vin / N, vLoRms = Vo;

    // Transformer: PRIMARY carries the reflected output current (Io/N) during the power-transfer phase
    // plus the magnetizing current; SECONDARY carries Io. Magnetizing ramp Im_pp from Vin*Deff over the
    // active half (the same volt-seconds that sized Lm). Primary sees +-Vin (full bridge); secondary
    // +-Vin/N. Two-level rectangular -> vRms = sqrt(Deff)*Von (effective on-fraction per polarity).
    const double dILm = Vin * Deff * Tsw / (2.0 * Lm);   // magnetizing pk-pk
    const double IpriCtr = Io / N;
    const double IpriPk  = IpriCtr + dILm / 2.0;
    const double IpriRms = std::sqrt(Deff) * std::sqrt(IpriCtr * IpriCtr + dILm * dILm / 12.0);
    const double IsecPk  = Io + dILo / 2.0;             // = output-inductor peak (secondary feeds Lout)
    const double IsecRms = std::sqrt(Deff) * std::sqrt(Io * Io + dILo * dILo / 12.0);
    const double Vs = Vin / N;
    const double vPriPk = Vin, vPriPkPk = 2.0 * Vin, vPriRms = std::sqrt(Deff) * Vin;
    const double vSecPk = Vs,  vSecPkPk = 2.0 * Vs,  vSecRms = std::sqrt(Deff) * Vs;

    // --- semiconductor ratings (sourceable requirements) ---
    // Primary full-bridge switches QA..QD: each blocks the full bus (Vin_max) when off; they carry the
    // primary current (reflected load + magnetizing), peak = IpriPk, rms = IpriRms. Anti-parallel DA..DD
    // are the FETs' body diodes (left requirement-less).
    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IpriRms * IpriRms);
    json mosfetReq; mosfetReq["semiconductor"]["mosfet"] = json::object();
    mosfetReq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpriPk, maxRdsOn, 125.0);
    // Secondary full-bridge rectifier Dr1..Dr4: each off diode blocks the secondary winding voltage
    // (peak Vs); each carries the output current while conducting. REAL rectifiers -> req::diode.
    const double ratedVr  = vSecPk / cfg::v_derate_diode(d.config);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;
    json diodeReq; diodeReq["semiconductor"]["diode"] = json::object();
    diodeReq["inputs"]["designRequirements"] = req::diode(ratedVr, Io / 0.7, maxVf, 0.05 * Tsw);

    // Series resonant/leakage inductor Lr (single-winding magnetic). Carries the full primary current;
    // the tank is AC (no DC bias). Voltage is the leg-to-leg commutation drop ~Vin during commutation.
    const double IlrPk = IpriPk, IlrRms = IpriRms;
    json lr; lr["magnetic"] = json::object();
    lr["inputs"] = req::magnetic_inputs(d.seriesInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IlrPk, IlrRms, 0.0, 2.0 * IpriCtr, std::nullopt,
                                    Vin, std::sqrt(Deff) * Vin, 0.0, 2.0 * Vin)});

    // Transformer windings: FULL_BRIDGE has ONE full secondary (turnsRatios=[N]); CENTER_TAPPED and
    // CURRENT_DOUBLER have TWO secondary half-windings (turnsRatios=[N,N]). primary + wpo secondaries.
    const int wpo = rectifier_windings_per_output(d.rectifierType);
    std::vector<std::string> isoSides{"primary"};
    std::vector<double> turnsRatios;
    for (int w = 0; w < wpo; ++w) { isoSides.push_back("secondary"); turnsRatios.push_back(N); }

    // Transformer excitations from the SINGLE FHA source — the SPICE-validated analytical PSFB solver.
    // FULL_BRIDGE (default) emits Primary + ONE bipolar secondary (2 windings); CENTER_TAPPED emits
    // Primary + two half-windings (3 windings); both line up with the wpo-built winding structure.
    // CURRENT_DOUBLER is NOT modelled by the solver (its two output-inductor secondaries have distinct
    // physics), so it keeps the inline model. Lr + the switch/diode ratings stay inline (Lr is not a
    // transformer winding).
    namespace AN = Kirchhoff::analytical;
    std::vector<json> xwindings;
    if (d.rectifierType == RectifierType::FullBridge || d.rectifierType == RectifierType::CenterTapped) {
        const AN::SrcRectifier rect = (d.rectifierType == RectifierType::CenterTapped)
                                      ? AN::SrcRectifier::CENTER_TAPPED : AN::SrcRectifier::FULL_BRIDGE;
        const MAS::OperatingPoint aopT1 = AN::analytical_psfb(Vin, {Vo}, {Io}, {N}, fsw, Lm,
                                                              d.seriesInductance, d.outputInductance,
                                                              d.phaseDeg, 0.0, rect);
        xwindings = AN::excitations_processed(aopT1);
    } else {
        xwindings.push_back(req::winding_excitation("psfbPrimary", fsw, IpriPk, IpriRms, 0.0, dILm, Deff,
                                vPriPk, vPriRms, 0.0, vPriPkPk));
        for (int w = 0; w < wpo; ++w)
            xwindings.push_back(req::winding_excitation("psfbSecondary", fsw, IsecPk, IsecRms, 0.0, dILo, Deff,
                                    vSecPk, vSecRms, 0.0, vSecPkPk));
    }
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, isoSides, std::nullopt, 25.0, xwindings,
        /*turnsRatioIsCeiling=*/{}, /*lmIsMinimum (PSFB transformer: maximise Lm ungapped -> K~0.999,
          low leakage; PSFB has a SEPARATE series Lr for ZVS so it does NOT tie Lm to Lr like DAB. abt #56)=*/true);

    // Output inductor Lo (single-winding magnetic, DC-biased at Io).
    json lout; lout["magnetic"] = json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IloPk, IloRms, Io, dILo, std::nullopt,
                                    vLoPk, vLoRms, 0.0, vLoPkPk)});

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

    // CURRENT_DOUBLER second output inductor + loop-breaker (CD uses Lout as Lo1 plus a second Lo2).
    auto outInductor2 = [&]() { json m; m["magnetic"] = json::object();
        m["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IloPk, IloRms, Io, dILo, std::nullopt,
                                    vLoPk, vLoRms, 0.0, vLoPkPk)});
        return m; };
    auto loopBreakR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"]; dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = cfg::loop_breaker_res(d.config, d.loadResistance);
        dr["powerRating"] = 0.25; dr["role"] = "balancing"; return c; };

    json cell; cell["name"] = "psfb-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("gateA"), port("gateB"), port("gateC"), port("gateD")});

    // Primary full bridge + Lr + transformer primary — identical for every rectifier variant.
    std::vector<json> comps{
        comp("QA", mosfetReq), comp("QB", mosfetReq), comp("QC", mosfetReq), comp("QD", mosfetReq),
        comp("DA", diode()),  comp("DB", diode()),  comp("DC", diode()),  comp("DD", diode()),
        comp("Lr", lr), comp("T1", xfmr)};
    std::vector<json> conns{
        conn("vin_net",  {pin("QA", "drain"), pin("QC", "drain"),
                          pin("DA", "cathode"), pin("DC", "cathode"), prt("vin")}),
        conn("midA_net", {pin("QA", "source"), pin("QB", "drain"),
                          pin("DA", "anode"), pin("DB", "cathode"),
                          pin("Lr", "primary_start"), pin("CsnA", "1")}),
        conn("midC_net", {pin("QC", "source"), pin("QD", "drain"),
                          pin("DC", "anode"), pin("DD", "cathode"),
                          pin("T1", "primary_end"), pin("CsnC", "1")}),
        conn("pri_x",    {pin("Lr", "primary_end"), pin("T1", "primary_start")})};
    // gnd_net base: low-side switch sources + body-diode anodes + bridge-midpoint snubber returns. Each
    // rectifier variant appends its own secondary returns before the net is emitted.
    std::vector<json> gndEps{pin("QB", "source"), pin("QD", "source"),
                             pin("DB", "anode"), pin("DD", "anode"),
                             pin("CsnA", "2"), pin("CsnC", "2")};

    switch (d.rectifierType) {
    case RectifierType::FullBridge: {
        // One secondary -> 4-diode bridge -> output inductor. (Original validated topology.)
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Dr3", diodeReq),
            comp("Dr4", diodeReq), comp("Lout", lout), comp("CsnA", snub()), comp("CsnC", snub()),
            comp("CsnSA", snub()), comp("CsnSB", snub())});
        conns.push_back(conn("sec_a", {pin("T1", "secondary1_start"), pin("Dr1", "anode"),
                                       pin("Dr3", "cathode"), pin("CsnSA", "1")}));
        conns.push_back(conn("sec_b", {pin("T1", "secondary1_end"), pin("Dr2", "anode"),
                                       pin("Dr4", "cathode"), pin("CsnSB", "1")}));
        conns.push_back(conn("out_rect", {pin("Dr1", "cathode"), pin("Dr2", "cathode"),
                                          pin("Lout", "primary_start")}));
        conns.push_back(conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}));
        gndEps.insert(gndEps.end(), {pin("Dr3", "anode"), pin("Dr4", "anode"),
                                     pin("CsnSA", "2"), pin("CsnSB", "2"), prt("gnd")});
        conns.push_back(conn("gnd_net", gndEps));
        break; }
    case RectifierType::CenterTapped: {
        // Two REAL secondary half-windings -> 2 diodes -> output inductor; secondary CT = gnd. (A proper
        // center tap — not MKF's fake one — so it delivers full Vout.)
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq),
            comp("Lout", lout), comp("CsnA", snub()), comp("CsnC", snub()),
            comp("CsnSA", snub()), comp("CsnSB", snub())});
        conns.push_back(conn("sec_a", {pin("T1", "secondary1_start"), pin("Dr1", "anode"), pin("CsnSA", "1")}));
        conns.push_back(conn("sec_b", {pin("T1", "secondary2_end"),   pin("Dr2", "anode"), pin("CsnSB", "1")}));
        conns.push_back(conn("out_rect", {pin("Dr1", "cathode"), pin("Dr2", "cathode"),
                                          pin("Lout", "primary_start")}));
        conns.push_back(conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}));
        gndEps.insert(gndEps.end(), {pin("T1", "secondary1_end"), pin("T1", "secondary2_start"),
                                     pin("CsnSA", "2"), pin("CsnSB", "2"), prt("gnd")});
        conns.push_back(conn("gnd_net", gndEps));
        break; }
    case RectifierType::CurrentDoubler: {
        // ONE secondary winding (its two ends = node_a/node_b) -> 2 catch diodes (cathode at each end,
        // anode at gnd) + 2 output inductors Lout(=Lo1) and Lo2. Each carries Io/2; a tiny loop-breaker R
        // between Lo2 and vout breaks the winding+Lo1+Lo2 all-inductor loop. (wpo=1, like the LLC CD.)
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq),
            comp("Lout", lout), comp("Lo2", outInductor2()), comp("Rlb", loopBreakR()),
            comp("CsnA", snub()), comp("CsnC", snub()), comp("CsnSA", snub()), comp("CsnSB", snub())});
        conns.push_back(conn("node_a", {pin("T1", "secondary1_start"), pin("Dr1", "cathode"),
                                        pin("Lout", "primary_start"), pin("CsnSA", "1")}));
        conns.push_back(conn("node_b", {pin("T1", "secondary1_end"), pin("Dr2", "cathode"),
                                        pin("Lo2", "primary_start"), pin("CsnSB", "1")}));
        conns.push_back(conn("lo2_out", {pin("Lo2", "primary_end"), pin("Rlb", "1")}));
        conns.push_back(conn("vout_net", {pin("Lout", "primary_end"), pin("Rlb", "2"), prt("vout")}));
        gndEps.insert(gndEps.end(), {pin("Dr1", "anode"), pin("Dr2", "anode"),
                                     pin("CsnSA", "2"), pin("CsnSB", "2"), prt("gnd")});
        conns.push_back(conn("gnd_net", gndEps));
        break; }
    case RectifierType::VoltageDoubler:
        throw std::runtime_error("Kirchhoff PSFB: voltageDoubler rectifier not supported");
    }
    conns.push_back(conn("gateA_net", {pin("QA", "gate"), prt("gateA")}));
    conns.push_back(conn("gateB_net", {pin("QB", "gate"), prt("gateB")}));
    conns.push_back(conn("gateC_net", {pin("QC", "gate"), prt("gateC")}));
    conns.push_back(conn("gateD_net", {pin("QD", "gate"), prt("gateD")}));
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
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("QA", 0.0), stim("QB", 180.0),
        stim("QC", d.phaseDeg), stim("QD", 180.0 + d.phaseDeg)});
    req::finalize_control_seeds(tas, Topology::PHASE_SHIFTED_FULL_BRIDGE_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
