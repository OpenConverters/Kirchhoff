#include "Weinberg.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kRippleRatio = 0.30;   // input-inductor (L1) current ripple (MKF Weinberg default)
constexpr double kDTarget     = 0.55;   // boost-regime duty target used to size n (MKF)
constexpr double kCoRipplePct = 0.01;   // output-cap voltage ripple fraction (MKF)
constexpr double kMaxDuty     = 0.95;

// MKF Weinberg::calculate_duty_cycle (boost branch — the design always lands D>0.5 here).
double duty_boost(double Vin, double Vo, double n, double eta) {
    double M = Vo / (Vin * eta);
    return 1.0 - 1.0 / (2.0 * n * M);
}
} // namespace

WeinbergDesign design_weinberg(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    WeinbergDesign d{};
    d.config = cfg::object_of(tasInputs);
    // Primary-drive + rectifier variants (config-gated, no schema change; defaults = MKF V1 classic).
    {
        const std::string v = cfg::get_str(d.config, "variant", "classic");
        if (v == "classic")      d.variant = WeinbergVariant::Classic;
        else if (v == "bridge")  d.variant = WeinbergVariant::Bridge;
        else throw std::invalid_argument("design_weinberg: unknown variant '" + v + "' (expected 'classic' or 'bridge')");
    }
    d.synchronousRectifier = cfg::get_bool(d.config, "synchronousRectifier", false);
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.02);   // SR / bridge complementary dead band
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
    {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower  = op.at("outputs").at(0).at("power").get<double>();
    }
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage, Fs = d.switchingFrequency, eta = d.efficiency;
    const double Iout = d.outputPower / Vo;

    // Turns ratio sized at MAX Vin to keep D ≥ 0.55 (boost regime) across the input range:
    //   n = 1 / (2·M·(1−D_target)),  M = Vo/(Vin_max·η).   (MKF process_design_requirements)
const double Vd = req::dideal_diode_drop(d.outputPower / Vo);  // DIDEAL Vf at the rectifier current
    const double Mmax = (Vo + Vd) / (vinMax * eta);
    double n = 1.0 / (2.0 * Mmax * (1.0 - cfg::get(d.config, "boostDutyTarget", kDTarget)));
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 1).value_or(std::round(n * 1000.0) / 1000.0);
    // Boost-regime duty at nominal Vin (the deck simulates at nominal Vin).
d.dutyCycle = duty_boost(Vin, Vo + Vd, d.turnsRatio, eta);
    d.switchDuty = d.dutyCycle;

    // L1 sized at MIN Vin (worst case): ΔI_L1 = ripple·I_L1perWinding, I_L1perWinding = Iin/2,
    // Iin = Iout·M_boost/η (power balance Vin·Iin·η = Vo·Iout, M = Vo/Vin). L1 = Vin_min·D_eff/(ΔI_L1·Fs),
    // D_eff = max(2D−1, D).
    const double Dmin = duty_boost(vinMin, Vo, d.turnsRatio, eta);
    const double Mboost = 1.0 / (2.0 * d.turnsRatio * (1.0 - Dmin));
    const double Iin = Iout * Mboost / eta;
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatio) * (Iin / 2.0);
    const double dEff = std::max(2.0 * Dmin - 1.0, Dmin);
    d.inputInductance = vinMin * dEff / (dIL1 * Fs);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        d.inputInductance);   // Lpri_half ≈ L1 (MKF mirrors Lpri to the L1 magnitude)

    // Output cap from the 1% ripple target: Co = Iout·D/(ΔVo·Fs), ΔVo = coRipplePct·Vo.
    d.outputCapacitance = Iout * d.dutyCycle / (cfg::get(d.config, "outputCapRipple", kCoRipplePct) * Vo * Fs);
    d.loadResistance = Vo / Iout;
    return d;
}

json build_weinberg_tas(const WeinbergDesign& d) {
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
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](const json& reqs) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto res = [&](double r) { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = r;
        // Conservative requirement floor: min(I^2*R, V^2/R) (exact for loads; safe for damping/snubber).
        const double Iout_ = d.outputPower / d.outputVoltage, Vb_ = d.outputVoltage;
        const double i2r_ = Iout_*Iout_*r, v2r_ = Vb_*Vb_/r;
        dr["powerRating"] = (i2r_ < v2r_ ? i2r_ : v2r_);
        return c; };

    const double n = d.turnsRatio, fsw = d.switchingFrequency;
    const double Vin = d.inputVoltage, Vout = d.outputVoltage, D = d.switchDuty;

    // --- semiconductor-only electrical stresses (nominal operating corner) ---
    // The transformer/inductor WINDING excitations come from the analytical solver below (the single FHA
    // source); only the values the switch/diode ratings need are computed inline here. The current-fed
    // front end draws Iin = Pin/Vin; each push-pull primary half carries the full input current during its
    // ON-fraction D. Primary halves reflect ±n·Vout (n = Np/Ns), secondaries ±Vout.
    const double Iin  = d.outputPower / (d.efficiency * Vin);
    const double Iout = d.outputPower / Vout;
    const double IL1avg = Iin / 2.0;
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatio) * IL1avg;
    const double IpkPri  = Iin + dIL1 / 2.0;
    const double IrmsPri = std::sqrt(D) * Iin;
    // BRIDGE (ABT #88): the H-bridge drives the two primary halves in SERIES, so the transformer sees TWICE
    // the primary turns of a single push-pull half → half the volts/turn. To land the shared operating point
    // (same Vout, D) the bridge is wound with half the primary-referred turns ratio (nXfmr = n/2); that both
    // restores the gain and genuinely HALVES each device's reflected blocking voltage (n·Vout vs 2·n·Vout) —
    // the canonical high-bus benefit of the bridge variant. Overridable via config "bridgeTurnsScale".
    const bool bridge = (d.variant == WeinbergVariant::Bridge);
    const double nXfmr = bridge ? cfg::get(d.config, "bridgeTurnsScale", 0.5) * n : n;
    const double vPriPkPk = 2.0 * nXfmr * Vout;   // primary-switch reflected blocking voltage (≡ Vin/(1−D) classic)
    const double vSecPkPk = 2.0 * Vout;       // Dpos/Dneg blocking voltage

    // ── semiconductor requirements (boost regime, nominal operating corner) ──
    // Push-pull primary switches S1/S2: in a current-fed push-pull each off-switch drain is clamped by
    // the conducting half + the reflected secondary to ~2·n·Vout (the full primary pk-pk reflected
    // voltage, ≡ Vin/(1−D)); it carries the full input current during its conduction (duty D). Secondary CT-FW
    // rectifiers Dpos/Dneg are REAL output rectifiers: each blocks the full secondary pk-pk (2·Vout)
    // when its partner conducts, and supplies Iout on alternate half-cycles. (No anti-parallel FET;
    // the energy-recovery path is the series-RC snubber, so no separate recovery diode here.)
    // BRIDGE variant (ABT #88): the 4-switch H-bridge drives the two primary halves in SERIES from the
    // current-fed boosted rail, and the transformer is wound with half the primary-referred turns
    // (nXfmr = n/2, above) to keep the shared operating point. That makes each device block the boosted
    // rail = 2·nXfmr·Vout = n·Vout — HALF the push-pull's 2·n·Vout, the canonical high-bus benefit.
    const double ratedVdsW = vPriPkPk / cfg::v_derate_mosfet(d.config);   // classic 2*n*Vout ; bridge n*Vout
    const double ratedVrW  = vSecPkPk / cfg::v_derate_diode(d.config);   // Dpos/Dneg block ~2*Vout
    const double maxRdsOnW = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsPri * IrmsPri);
    const double maxVfW    = (ratedVrW < 100.0) ? 0.6 : 1.2;
    const double maxTrrW   = 0.05 / fsw;
    const json reqSW   = req::mosfet("mainSwitch", ratedVdsW, IpkPri, maxRdsOnW, 125.0);
    const json reqRect = req::diode(ratedVrW, Iout / 0.7, maxVfW, maxTrrW);  // CT full-wave: each carries Iout-avg
    // Synchronous-rectifier MOSFET (ABT #88): replaces a CT-FW freewheel diode with a channel (I^2*Rds vs
    // I*Vf) + its anti-parallel body diode across the dead time. Blocks the same 2*Vout secondary swing and
    // carries the same Iout-avg as the diode it shadows.
    const double srRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / std::max(Iout * Iout, 1e-9);
    const json reqSR      = req::mosfet("synchronousRectifier", ratedVrW, Iout / 0.7, srRdsOn, 125.0);
    const json reqSRbody  = req::body_diode(ratedVrW, Iout / 0.7);

    // Input coupled inductor L1 (1:1, two windings), K=0.999. winding0=L1a (vin->l1a_mid),
    // secondary1=L1b (vin->l1b_mid). Both dots at vin (primary_start/secondary1_start).
    // turnsRatios=[1] -> 2 physical windings -> 2 excitations (both carry Iin/2, DC-biased).
    // Both magnetics' excitations come from the SINGLE FHA source — analytical_weinberg emits all SIX
    // windings in order [L1a, L1b, T1_pri_a, T1_pri_b, T1_sec_a, T1_sec_b]; slice them into the two
    // magnetics. This CORRECTS the inline model, which undersized L1 by ~40% on peak current and used a
    // zero DC offset on the DC-biased current-fed windings — verified against the ngspice deck (the
    // input-inductor and push-pull-primary windings measure identical DC-biased current, and the
    // secondary windings carry a real DC bias, none of which the inline zero-offset model captured).
    namespace AN = Kirchhoff::analytical;
    // analytical_weinberg emits ONE operating point spanning BOTH magnetics (6 windings, order
    // [L1a, L1b, T1_pri_a, T1_pri_b, T1_sec_a, T1_sec_b]). Slice it into a 2-winding L1 op and a 4-winding
    // T1 op so each magnetic captures an HONEST per-component operating point into the full-waveform
    // registry (keys "L1"/"T1", matching the TAS component names). The earlier code baked the TAS via the
    // unnamed overload and so registered NOTHING — Weinberg was the only topology missing from
    // analyticalWaveforms, forcing the wizard onto its synthesis fallback.
    // The analytical excitation uses the REAL operating point (turns ratio n, duty D) so the L1/primary
    // CURRENTS it records match the deck; the bridge's halved primary-referred turns only rescale the
    // reflected VOLTAGE (captured via nXfmr in the transformer ratios + switch rating below).
    const MAS::OperatingPoint wOp = AN::analytical_weinberg(
        Vin, Vout, Iout, fsw, d.inputInductance, n, 0.0, d.efficiency, bridge);
    const auto& allExc = wOp.get_excitations_per_winding();
    if (allExc.size() != 6)
        throw std::runtime_error("build_weinberg_tas: analytical_weinberg must emit 6 windings (L1 x2 + T1 x4), got "
                                 + std::to_string(allExc.size()));
    MAS::OperatingPoint opL1, opT1;
    opL1.get_mutable_excitations_per_winding().assign(allExc.begin(), allExc.begin() + 2);
    opT1.get_mutable_excitations_per_winding().assign(allExc.begin() + 2, allExc.end());
    const std::vector<json> wL1 = AN::excitations_processed(opL1, "L1");
    const std::vector<json> wT1 = AN::excitations_processed(opT1, "T1");

    json l1; l1["magnetic"] = json::object();
    l1["inputs"] = req::magnetic_inputs(d.inputInductance, 0.2, {1.0}, {"primary", "primary"},
        std::nullopt, 25.0, wL1);
    { const double kCpl = cfg::get(d.config, "transformerCoupling", 0.999);
      l1["inputs"]["designRequirements"]["leakageInductance"] =
          json::array({ json{{"nominal", (1.0 - kCpl*kCpl) * d.inputInductance}} }); }

    // Main transformer: CT primary (2 halves) + CT secondary (2 halves) = 4 coupled windings via
    // turnsRatios=[1, n, n]. Opposite-dot wound (encoded by the start/end node order below). K=0.9999.
    // Isolated -> isolationSides {primary, primary, secondary, secondary}; WeinbergDesign carries no
    // isolationVoltage -> nullopt.
    json t1; t1["magnetic"] = json::object();
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, {1.0, nXfmr, nXfmr},
        {"primary", "primary", "secondary", "secondary"}, std::nullopt, 25.0, wT1,
        // n = 1/(2·M·(1−D_target)) is the boost-regime duty-derived ratio -> emit the two secondary ratios
        // as {maximum}; the 1.0 second-primary half is a structural 1:1 and stays {nominal}. (abt #49)
        /*turnsRatioIsCeiling=*/{false, true, true});

    json cout; cout["capacitor"] = json::object();
    cout["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cout["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // SERIES-RC snubber (100Ω + 100pF) across each push-pull switch drain (which swings to 2·Vin/(1−D))
    // and each rectifier diode, to damp the leakage spike (no D3 recovery diodes). The series C BLOCKS the
    // DC blocking voltage, so the R (sized ≈√(L/C) for critical damping) dissipates only the switching
    // transition — NOT the continuous V_drain²/R that a drain-to-ground R∥C bleeds (which at the high
    // boost-reset voltage starved the output ~7%, dropping efficiency to ~57%). Intended divergence from
    // MKF's lossy R∥C: Kirchhoff delivers spec open-loop.
    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::rectifier_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage * 6 + d.outputVoltage);
        cfg::mark_numerical_aid(c); return c; };   // rectifier dV/dt aid — tagged for the real strip (ABT #96)
    // The RC-snubber resistor paired with each snubC. Wraps the generic res() (shared with the FUNCTIONAL
    // Rdcr* loop-breakers, which must survive the real deck) and adds the numerical-aid tag so ONLY the
    // Rsn* snubber resistors are stripped at real fidelity (ABT #96).
    auto snubR = [&]() { json c = res(cfg::snubber_res(d.config)); cfg::mark_numerical_aid(c); return c; };

    // ── switching cell ──────────────────────────────────────────────────────────────────────────────
    // SHARED across every variant: input coupled inductor L1 + its two loop-breakers, the 4-winding main
    // transformer T1, the output cap, and the two secondary CT-FW rectifier snubbers. VARIANT-gated: the
    // PRIMARY drive (classic 2-switch push-pull vs 4-switch H-bridge) and the secondary rectifier device
    // (freewheel diodes vs SR MOSFETs + body diodes). (ABT #88)
    const bool useSR = d.synchronousRectifier;

    std::vector<json> comps{
        comp("L1", l1), comp("T1", t1), comp("Cout", cout),
        // Tiny series R between each input-inductor winding and the transformer primary: a NUMERICAL
        // loop-breaker for the otherwise-singular all-inductor mesh (coupled L1 + coupled T1 primary), not a
        // physical DCR. 1 mΩ is the minimum that converges.
        comp("Rdcra", res(cfg::loop_breaker_res(d.config, d.loadResistance))),
        comp("Rdcrb", res(cfg::loop_breaker_res(d.config, d.loadResistance))),
        comp("RsnDp", snubR()), comp("CsnDp", snubC()),
        comp("RsnDn", snubR()), comp("CsnDn", snubC())};
    std::vector<json> conns{
        // Input: both L1 windings fed from vin (current-fed front end).
        conn("vin_net", {pin("L1", "primary_start"), pin("L1", "secondary1_start"), prt("vin")}),
        conn("l1a_mid", {pin("L1", "primary_end"), pin("Rdcra", "1")}),
        conn("l1b_mid", {pin("L1", "secondary1_end"), pin("Rdcrb", "1")}),
        // Series-RC snubber across each rectifier device (device -> C -> R -> out_node).
        conn("snDp", {pin("CsnDp", "2"), pin("RsnDp", "1")}),
        conn("snDn", {pin("CsnDn", "2"), pin("RsnDn", "1")})};
    // out_node / gnd endpoints are accumulated by the branches, then emitted once at the end.
    std::vector<json> outEps{pin("Cout", "1"), prt("vout"), pin("RsnDp", "2"), pin("RsnDn", "2")};
    std::vector<json> gndEps{pin("Cout", "2"), prt("gnd"),
                             pin("T1", "secondary2_start"), pin("T1", "secondary3_end")};
    std::vector<std::string> gatePorts;

    // ── secondary CT-FW rectifier ──
    if (useSR) {
        // SR MOSFETs Spos/Sneg (drain = out_node, source = secondary-half end) + anti-parallel body diodes
        // conduct source->out_node exactly as the diode they replace; the channel carries Iout at I^2*Rds
        // (vs the diode's I*Vf) and the body diode covers the dead band. Complementary drive (gp/gn).
        comps.insert(comps.end(), {comp("Spos", mosfet(reqSR)), comp("Sneg", mosfet(reqSR)),
                                   comp("Dpos", diode(reqSRbody)), comp("Dneg", diode(reqSRbody))});
        conns.push_back(conn("diodePos", {pin("T1", "secondary2_end"), pin("Spos", "source"),
                                          pin("Dpos", "anode"), pin("CsnDp", "1")}));
        conns.push_back(conn("diodeNeg", {pin("T1", "secondary3_start"), pin("Sneg", "source"),
                                          pin("Dneg", "anode"), pin("CsnDn", "1")}));
        outEps.insert(outEps.end(), {pin("Spos", "drain"), pin("Sneg", "drain"),
                                     pin("Dpos", "cathode"), pin("Dneg", "cathode")});
        conns.push_back(conn("gp_net", {pin("Spos", "gate"), prt("gp")}));
        conns.push_back(conn("gn_net", {pin("Sneg", "gate"), prt("gn")}));
        gatePorts.insert(gatePorts.end(), {"gp", "gn"});
    } else {
        // Real CT-FW rectifier diodes: each secondary half -> its diode -> out_node; secondary CT = gnd.
        comps.insert(comps.end(), {comp("Dpos", diode(reqRect)), comp("Dneg", diode(reqRect))});
        conns.push_back(conn("diodePos", {pin("T1", "secondary2_end"), pin("Dpos", "anode"), pin("CsnDp", "1")}));
        conns.push_back(conn("diodeNeg", {pin("T1", "secondary3_start"), pin("Dneg", "anode"), pin("CsnDn", "1")}));
        outEps.insert(outEps.end(), {pin("Dpos", "cathode"), pin("Dneg", "cathode")});
    }

    // ── primary drive ──
    if (d.variant == WeinbergVariant::Bridge) {
        // 4-switch H-BRIDGE primary (ABT #88). The input coupled inductor L1 current-feeds the boosted rail
        // railP (= Vin/(1−D)); the bridge drives the two transformer primary halves in SERIES (their inner
        // junction priC is the winding centre, left floating and DC-referenced by a large bleed Rctr).
        //   leg 1: S1 (railP->nodeA, high) + S2 (nodeA->gnd, low);  leg 2: S3 (railP->nodeB, high) + S4 (nodeB->gnd, low)
        // Diagonal PWM (each device duty D): (S1,S4) phase 0 apply +railP across the primary, (S2,S3) phase
        // 180 apply −railP; for D>0.5 the pairs overlap (all four on) and short the primary to gnd so L1
        // charges (the boost storage interval). Each off device blocks the full rail railP = 2·nXfmr·Vout =
        // n·Vout — HALF the push-pull's 2·n·Vout (the transformer is wound with half the turns, nXfmr = n/2).
        // Large bleed: DC reference for the floating series-primary centre (numerical aid, stripped at real
        // fidelity where the winding capacitance references it physically).
        json rctr = res(1.0e3 * d.loadResistance); cfg::mark_numerical_aid(rctr);
        comps.insert(comps.end(), {
            comp("S1", mosfet(reqSW)), comp("S2", mosfet(reqSW)),
            comp("S3", mosfet(reqSW)), comp("S4", mosfet(reqSW)),
            comp("RsnS1", snubR()), comp("CsnS1", snubC()),
            comp("RsnS2", snubR()), comp("CsnS2", snubC()),
            comp("Rctr", rctr)});
        conns.push_back(conn("railP",  {pin("Rdcra", "2"), pin("Rdcrb", "2"),
                                        pin("S1", "drain"), pin("S3", "drain")}));
        conns.push_back(conn("nodeA",  {pin("T1", "primary_start"), pin("S1", "source"),
                                        pin("S2", "drain"), pin("CsnS1", "1")}));
        conns.push_back(conn("nodeB",  {pin("T1", "secondary1_end"), pin("S3", "source"),
                                        pin("S4", "drain"), pin("CsnS2", "1")}));
        conns.push_back(conn("priC",   {pin("T1", "primary_end"), pin("T1", "secondary1_start"), pin("Rctr", "1")}));
        conns.push_back(conn("snS1",   {pin("CsnS1", "2"), pin("RsnS1", "1")}));
        conns.push_back(conn("snS2",   {pin("CsnS2", "2"), pin("RsnS2", "1")}));
        conns.push_back(conn("g1_net", {pin("S1", "gate"), prt("g1")}));
        conns.push_back(conn("g2_net", {pin("S2", "gate"), prt("g2")}));
        conns.push_back(conn("g3_net", {pin("S3", "gate"), prt("g3")}));
        conns.push_back(conn("g4_net", {pin("S4", "gate"), prt("g4")}));
        gndEps.insert(gndEps.end(), {pin("S2", "source"), pin("S4", "source"),
                                     pin("Rctr", "2"), pin("RsnS1", "2"), pin("RsnS2", "2")});
        gatePorts.insert(gatePorts.begin(), {"g1", "g2", "g3", "g4"});
    } else {
        // Classic 2-switch current-fed push-pull: each input-inductor winding feeds one primary center-tap
        // half; the dot ends (drainQ1/drainQ2) are switched to gnd by S1/S2 (180° apart, overlapping for D>0.5).
        comps.insert(comps.end(), {
            comp("S1", mosfet(reqSW)), comp("S2", mosfet(reqSW)),
            comp("RsnS1", snubR()), comp("CsnS1", snubC()),
            comp("RsnS2", snubR()), comp("CsnS2", snubC())});
        conns.push_back(conn("priCT_a", {pin("Rdcra", "2"), pin("T1", "primary_end")}));
        conns.push_back(conn("priCT_b", {pin("Rdcrb", "2"), pin("T1", "secondary1_start")}));
        conns.push_back(conn("drainQ1", {pin("T1", "primary_start"), pin("S1", "drain"), pin("CsnS1", "1")}));
        conns.push_back(conn("drainQ2", {pin("T1", "secondary1_end"), pin("S2", "drain"), pin("CsnS2", "1")}));
        conns.push_back(conn("snS1",    {pin("CsnS1", "2"), pin("RsnS1", "1")}));
        conns.push_back(conn("snS2",    {pin("CsnS2", "2"), pin("RsnS2", "1")}));
        conns.push_back(conn("g1_net",  {pin("S1", "gate"), prt("g1")}));
        conns.push_back(conn("g2_net",  {pin("S2", "gate"), prt("g2")}));
        gndEps.insert(gndEps.end(), {pin("S1", "source"), pin("S2", "source"),
                                     pin("RsnS1", "2"), pin("RsnS2", "2")});
        gatePorts.insert(gatePorts.begin(), {"g1", "g2"});
    }

    conns.push_back(conn("out_node", outEps));
    conns.push_back(conn("gnd_net", gndEps));

    json cell; cell["name"] = "weinberg-cell";
    std::vector<json> portList{port("vin"), port("gnd"), port("vout")};
    for (const auto& g : gatePorts) portList.push_back(port(g.c_str()));
    cell["ports"] = portList;
    cell["components"] = comps;
    cell["connections"] = conns;

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
        pstage("weinbergCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("weinbergCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("weinbergCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("weinbergCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Drive: each primary switch is ON for D·Tsw within its half-period; the two phases overlap by
    // (2D−1)·Tsw when D>0.5 (the boost storage interval).
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "weinbergCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    std::vector<json> stimuli;
    if (d.variant == WeinbergVariant::Bridge) {
        // Diagonal H-bridge PWM: pair (S1,S4) phase 0, pair (S2,S3) phase 180. For D>0.5 the pairs overlap
        // (all four on -> primary shorted, L1 charges) exactly as the push-pull's two phases overlap.
        stimuli = {stim("S1", 0.0), stim("S4", 0.0), stim("S2", 180.0), stim("S3", 180.0)};
    } else {
        // Push-pull drive: S1 phase 0, S2 phase 180.
        stimuli = {stim("S1", 0.0), stim("S2", 180.0)};
    }
    if (d.synchronousRectifier) {
        // SR gates track the SINGLE-CONDUCTION window of each secondary half (NOT the full switch duty): a
        // CT-FW secondary conducts only while one primary drives alone, width (1−D) starting at (D−0.5)·T
        // (for D>0.5 the two conduction windows are (D−0.5)·T apart). Gating over the whole duty would keep
        // the channel on through the reverse half and short the output — so drive exactly the diode's
        // window, inset by a dead band on each edge (the body diode covers the edges). (ABT #88)
        const double dead = d.deadFraction;
        const double srDuty = std::max(0.0, (1.0 - d.switchDuty) - 2.0 * dead);
        const double srPhase = (d.switchDuty - 0.5 + dead) * 360.0;
        // The H-bridge drives the primary with the opposite polarity to the push-pull for the same phase,
        // so the secondary half that conducts in each single-conduction window is swapped — default the
        // SR-gate assignment to the variant (overridable via config "srPhaseSwap").
        const bool swap = cfg::get_bool(d.config, "srPhaseSwap", d.variant == WeinbergVariant::Bridge);
        auto srStim = [&](const char* sw, double phaseDeg) {
            json st; st["stage"] = "weinbergCell"; st["component"] = sw; st["signal"] = "gate";
            st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
            st["waveform"]["dutyCycle"] = srDuty; st["waveform"]["phase"] = phaseDeg; return st; };
        stimuli.push_back(srStim("Spos", srPhase + (swap ? 180.0 : 0.0)));
        stimuli.push_back(srStim("Sneg", srPhase + (swap ? 0.0 : 180.0)));
    }
    tas["simulation"]["stimulus"] = stimuli;
    req::finalize_control_seeds(tas, Topology::WEINBERG_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
