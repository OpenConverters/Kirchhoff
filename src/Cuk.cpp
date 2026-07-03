#include "Cuk.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_cuk + excitations_processed/winding_current
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kRippleRatioL1 = 0.4;   // ΔIL1 / IL1,avg
constexpr double kL2RipplePct = 0.30;    // ΔIL2 / IL2,avg
constexpr double kC1RipplePct = 0.05;    // ΔVC1 / VC1
constexpr double kCoRipplePct = 0.01;    // ΔVo  / |Vo|
// Cuk CCM ideal-ish duty (n=1): D = (|Vo|+Vd) / (Vin*eff + |Vo| + Vd).
double duty(double vin, double voMag, double vd, double eff) { return (voMag + vd) / (vin * eff + voMag + vd); }
} // namespace

CukDesign design_cuk(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    CukDesign d{};
    d.config = cfg::object_of(tasInputs);
    // Output voltage is stored as a magnitude (MKF treats Cuk Vout as |Vo|); take abs to be robust to
    // a negative setpoint in the TAS.
    d.outputVoltageMag = std::fabs(nominal(dr.at("outputs").at(0).at("voltage")));
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

    const double iout = d.outputPower / d.outputVoltageMag, fsw = d.switchingFrequency;
    // MKF Cuk variant: synchronous rectifier (MOSFET replacing the catch diode D1), complementary to Q1.
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);
    // Coupled-inductor variant (ABT #89): L1 + L2 on one core (1:1) with mutual coupling k.
    d.coupledInductor = cfg::get_bool(d.config, "coupledInductor", false);
    d.couplingCoefficient = cfg::get(d.config, "couplingCoefficient", 0.999);
    if (d.coupledInductor && !(d.couplingCoefficient > 0.0 && d.couplingCoefficient < 1.0))
        throw std::invalid_argument("design_cuk: couplingCoefficient must be in (0,1), got "
                                    + std::to_string(d.couplingCoefficient));
    // Isolated Ćuk (V3, ABT #90): a transformer across the coupling cap. turnsRatio is the KH-convention
    // n = Np/Ns (primary:secondary, as everywhere else), so the secondary is the primary /n and the D/(1-D)
    // transfer sizes against the PRIMARY-referred output |Vo|·n. Bidirectional (V5): reverse power flow.
    // Isolation and the 1:1 coupled-inductor variant are mutually exclusive (the isolated cell already splits
    // the two inductors onto opposite sides of the transformer).
    d.isolated = cfg::get_bool(d.config, "isolated", false);
    d.bidirectional = cfg::get_bool(d.config, "bidirectional", false);
    // Bidirectional Ćuk (V5) — reverse power flow — is not yet wired (the inverting single-switch cell makes
    // the LV/HV source-load swap sign-awkward, unlike the symmetric CLLC bridge). Reject it loudly rather
    // than silently building a forward deck (no-silent-shortcuts). Tracked as the remaining ABT #90 item.
    if (d.bidirectional)
        throw std::invalid_argument("design_cuk: bidirectional (reverse power flow) Ćuk is not yet "
                                    "implemented (ABT #90 V5 follow-up); only forward flow is supported");
    if (d.isolated && d.coupledInductor)
        throw std::invalid_argument("design_cuk: 'isolated' and 'coupledInductor' are mutually exclusive");
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(cfg::get(d.config, "turnsRatio", 1.0));
    if (d.isolated && !(d.turnsRatio > 0.0))
        throw std::invalid_argument("design_cuk: isolated turnsRatio must be > 0");
    const double n = d.isolated ? d.turnsRatio : 1.0;
    // Sync MOSFET has no forward drop → size duty with Vd=0 so the open-loop deck lands on target.
    d.diodeDrop = d.synchronousRectifier ? 0.0 : req::dideal_diode_drop(iout);
    // Isolated: reflect the output to the primary (|Vo|·n, since n = Np/Ns) for the D/(1-D) sizing; the
    // transformer steps it back down by n to |Vo| on the secondary.
    const double voRefDuty = d.outputVoltageMag * n;
    d.dutyCycle = duty(d.inputVoltage, voRefDuty, d.diodeDrop, d.efficiency);

    // L1 sized at the worst corner (max Vin) for its current-ripple target (MKF).
    const double dMax = duty(vinMax, voRefDuty, d.diodeDrop, d.efficiency);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double dIL1 = cfg::get(d.config, "l1RippleRatio", kRippleRatioL1) * iL1avg;
    d.inductanceL1 = vinMax * dMax / (dIL1 * fsw);
    // L2, C1, Cout at the operating point.
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    d.inductanceL2 = d.outputVoltageMag * (1.0 - d.dutyCycle) / (dIL2 * fsw);
    const double VC1 = d.inputVoltage / (1.0 - d.dutyCycle);   // = Vin + |Vo|
    const double dVC1 = cfg::get(d.config, "couplingCapRipple", kC1RipplePct) * VC1;
    d.couplingCapacitance = iout * d.dutyCycle / (dVC1 * fsw);
    const double dVo = cfg::get(d.config, "outputCapRipple", kCoRipplePct) * d.outputVoltageMag;
    d.outputCapacitance = dIL2 / (8.0 * fsw * dVo);
    d.loadResistance = d.outputVoltageMag * d.outputVoltageMag / d.outputPower;
    if (d.isolated) {
        // Secondary coupling cap C1b (holds ~|Vo|/D on the secondary side) — same ripple fraction as C1.
        const double VC1b = d.outputVoltageMag / std::max(d.dutyCycle, 1e-3);
        const double dVC1b = cfg::get(d.config, "couplingCapRipple", kC1RipplePct) * VC1b;
        d.secondaryCouplingCapacitance = iout * (1.0 - d.dutyCycle) / (dVC1b * fsw);
        // Transformer magnetizing inductance: large enough that the magnetizing current is a small fraction
        // of the reflected load current (the transformer stores no net power — the Ćuk moves energy through
        // the coupling caps). ΔIm = VC1·D·T/Lm ≤ ImagFrac·(iout/n).
        const double ImagFrac = cfg::get(d.config, "magnetizingCurrentFraction", 0.10);
        const double IrefLoad = iout / n;
        d.magnetizingInductance = VC1 * d.dutyCycle / (std::max(ImagFrac * IrefLoad, 1e-3) * fsw);
    }
    return d;
}

json build_cuk_tas(const CukDesign& d) {
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
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };

    // --- L1 (the MAIN coupled inductor) sourced from the SINGLE FHA solver (SPICE-validated analytical) ---
    // Worst-case corner (Vin_min) drives the switch rating; the declared nominal OP is what the TAS embeds
    // for L1. analytical_cuk returns ONE winding ("Primary" = L1); L2 (the secondary coupled inductor, whose
    // excitation is NOT one of the solver's windings) keeps its inline computation.
    namespace AN = Kirchhoff::analytical;
    const double fsw = d.switchingFrequency, iout = d.outputPower / d.outputVoltageMag;
    const double dIL2 = cfg::get(d.config, "l2RippleRatio", kL2RipplePct) * iout;
    const double vSwing = d.inputVoltage + d.outputVoltageMag;   // nominal operating swing (L2 excitation embed)
    const double vSwingRating = d.inputVoltageMax + d.outputVoltageMag;   // worst-case corner for VOLTAGE ratings
    const MAS::OperatingPoint aopWorst = AN::analytical_cuk(d.inputVoltageMin, d.outputVoltageMag, iout, fsw,
                                                           d.inductanceL1, d.diodeDrop, d.efficiency);
    const MAS::OperatingPoint aopNom   = AN::analytical_cuk(d.inputVoltage,    d.outputVoltageMag, iout, fsw,
                                                           d.inductanceL1, d.diodeDrop, d.efficiency);
    const double IL1avg = AN::winding_current(aopWorst, 0, "offset");   // L1 average (input current) at the worst corner

    // L2 (secondary coupled inductor) — inline single-winding excitation (not one of the solver's windings).
    auto inductor = [&](double L, double iAvg, double iPkPk) {
        json m; m["magnetic"] = json::object();
        const double iPk = iAvg + iPkPk / 2.0, iRms = std::sqrt(iAvg * iAvg + iPkPk * iPkPk / 12.0);
        m["inputs"] = req::magnetic_inputs(L, 0.2, {}, {"primary"}, std::nullopt, 25.0,
            {req::winding_excitation("triangular", fsw, iPk, iRms, iAvg, iPkPk, d.dutyCycle,
                                     vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing)});
        return m;
    };
    json L1; L1["magnetic"] = json::object();
    L1["inputs"] = req::magnetic_inputs(d.inductanceL1, 0.2, {}, {"primary"}, std::nullopt, 25.0,
        AN::excitations_processed(aopNom, "L1"));
    json L2 = inductor(d.inductanceL2, iout, dIL2);

    // Coupled-inductor variant (ABT #89): L1 and L2 share ONE core as a single 2-winding magnetic (1:1)
    // with mutual coupling k. Winding 0 (primary) = L1 (solver excitation); winding 1 (secondary1) = L2
    // (inline). leakageInductance = Lp·(1-k²) sets the coupling K from the coefficient in both decks.
    json L12;
    if (d.coupledInductor) {
        std::vector<json> w12 = AN::excitations_processed(aopNom);   // winding 0 = L1 (non-capturing)
        const double iL2Pk = iout + dIL2 / 2.0, iL2Rms = std::sqrt(iout * iout + dIL2 * dIL2 / 12.0);
        w12.push_back(req::winding_excitation("triangular", fsw, iL2Pk, iL2Rms, iout, dIL2, d.dutyCycle,
                                              vSwing, vSwing / std::sqrt(3.0), 0.0, vSwing));   // winding 1 = L2
        L12["magnetic"] = json::object();
        L12["inputs"] = req::magnetic_inputs(d.inductanceL1, 0.2, /*1:1*/ {1.0}, {"primary", "secondary"},
            std::nullopt, 25.0, w12);
        L12["inputs"]["designRequirements"]["leakageInductance"] = json::array({
            json{{"nominal", d.inductanceL1 * (1.0 - d.couplingCoefficient * d.couplingCoefficient)}} });
    }
    // Winding pins. For the coupled magnetic the two windings share a core with a POSITIVE mutual coupling
    // K (dot at each winding's *_start). The Ćuk is INVERTING, so its L2 must be wound with the opposite
    // dot orientation vs L1 for the flux to steer ripple without altering the DC operating point — the
    // secondary winding is entered from its *_end (nodeB) so the dot lands on the vout side. (SEPIC/Zeta are
    // non-inverting and use the natural start=first-node orientation.) Getting this wrong drives a
    // transformer action that shifts Vout far off target.
    auto l1s = [&]() { return d.coupledInductor ? pin("L12", "primary_start") : pin("L1", "primary_start"); };
    auto l1e = [&]() { return d.coupledInductor ? pin("L12", "primary_end")   : pin("L1", "primary_end");   };
    auto l2s = [&]() { return d.coupledInductor ? pin("L12", "secondary1_end")   : pin("L2", "primary_start"); };
    auto l2e = [&]() { return d.coupledInductor ? pin("L12", "secondary1_start") : pin("L2", "primary_end");   };
    auto magComps = [&](std::vector<json> rest) {   // magnetics + the rest (order is irrelevant to ngspice)
        std::vector<json> v = d.coupledInductor ? std::vector<json>{comp("L12", L12)}
                                                : std::vector<json>{comp("L1", L1), comp("L2", L2)};
        for (auto& r : rest) v.push_back(std::move(r));
        return json(v);
    };
    json c1; c1["capacitor"] = json::object();
    c1["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.couplingCapacitance;
    c1["inputs"]["designRequirements"]["ratedVoltage"] = vSwingRating / cfg::v_derate_capacitor(d.config);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltageMag / cfg::v_derate_capacitor(d.config);
    // Switch RMS: during the on-time (duty D) the main switch carries IL1 + IL2 (≈ IL1avg + iout);
    // maxRdsOn = loss_fraction·Pout / Isw_rms² (OHMS — sibling topologies all divide by Isw_rms²).
    const double IswRms = std::sqrt(d.dutyCycle) * (IL1avg + iout);
    json mq = mosfet();
    mq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", vSwingRating / cfg::v_derate_mosfet(d.config),
                                                     iout + IL1avg,
                                                     cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms), 125.0);
    json md = diode();
    md["inputs"]["designRequirements"] = req::diode(vSwingRating / cfg::v_derate_diode(d.config), iout / 0.7,
                                                    (vSwingRating < 100.0) ? 0.6 : 1.2, 0.05 / fsw);

    // RC snubber across the freewheel diode / sync-FET node — a REAL, sourced + rendered component. The Cuk
    // is HARD-switched, so this series R–C is a genuine board damper (it also lets the resonant coupling loop
    // converge in ngspice — otherwise "timestep too small" at startup). 100 Ω · 1 nF is negligible at the
    // power-stage scale (RC = 100 ns « the switching period; bleed « the amperes of inductor current), so it
    // does not shift the operating point. REAL refdes Crc_sw/Rrc_sw (role "snubber") — deliberately NOT the
    // Csn*/Rsn*/Csw* numerical-aid prefixes, so the fidelity snubber-strip keeps it in the REAL deck too
    // (the earlier "Csnub"/"Rsnub" names collided with that prefix and were wrongly stripped).
    json rsnub; rsnub["resistor"] = json::object();
    rsnub["inputs"]["designRequirements"]["deviceType"] = "resistor";
    rsnub["inputs"]["designRequirements"]["resistance"]["nominal"] = cfg::snubber_res(d.config);
    // RC-snubber R: P = C*V^2*f (cap energy dumped through R each cycle; V = switch-node swing).
    rsnub["inputs"]["designRequirements"]["powerRating"] =
        cfg::diode_snubber_cap(d.config) * vSwing * vSwing * d.switchingFrequency;
    rsnub["inputs"]["designRequirements"]["role"] = "snubber";
    json csnub; csnub["capacitor"] = json::object();
    csnub["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::diode_snubber_cap(d.config);
    csnub["inputs"]["designRequirements"]["ratedVoltage"] = vSwing / cfg::v_derate_capacitor(d.config);

    // Synchronous rectifier: a MOSFET Q2 (channel nodeB->gnd, mirroring the catch diode) + body diode D2.
    json syncFet = mosfet();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier",
        vSwingRating / cfg::v_derate_mosfet(d.config), iout + IL1avg,
        cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms), 125.0);
    json bodyD = diode();
    bodyD["inputs"]["designRequirements"] = req::body_diode(vSwingRating / cfg::v_derate_diode(d.config), iout / 0.7);

    // Isolated Ćuk (V3, ABT #90): a 2-winding transformer bridges the two coupling caps. C1 (primary) sits
    // between nodeA and the transformer primary; C1b (secondary) between the transformer secondary and nodeB;
    // the secondary rectifier + L2 deliver the output referred through n = Ns/Np. The two windings share the
    // system reference for the ideal-deck DC operating point (the transformer still provides the n transfer;
    // real galvanic isolation is a board property). Magnetizing Lm is sized large so it stores no net power.
    const double N = d.turnsRatio;   // = 1.0 for the non-isolated cell
    json t1, c1b;
    if (d.isolated) {
        const double vPri = d.inputVoltage / (1.0 - d.dutyCycle);         // primary coupling-node AC swing
        const double IprimAvg = iout / N, IsecAvg = iout;
        const double dIm = vPri * d.dutyCycle / (d.magnetizingInductance * fsw);   // magnetizing pk-pk
        std::vector<json> tw = {
            req::winding_excitation("triangular", fsw, IprimAvg + dIm / 2.0, IprimAvg, IprimAvg, dIm,
                                    d.dutyCycle, vPri, vPri / std::sqrt(2.0), 0.0, vPri),
            req::winding_excitation("triangular", fsw, IsecAvg * 1.3, IsecAvg, IsecAvg, IsecAvg * 0.3,
                                    d.dutyCycle, vPri * N, vPri * N / std::sqrt(2.0), 0.0, vPri * N)};
        t1["magnetic"] = json::object();
        t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.2, {N}, {"primary", "secondary"},
            std::nullopt, 25.0, tw);
        c1b["capacitor"] = json::object();
        c1b["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.secondaryCouplingCapacitance;
        c1b["inputs"]["designRequirements"]["ratedVoltage"] =
            (d.outputVoltageMag / std::max(d.dutyCycle, 1e-3)) * 2.0 / cfg::v_derate_capacitor(d.config);
    }

    // Cuk cell — inverting. Dot/orientation mirror MKF: D1 anode at nodeB, cathode at gnd; L2 nodeB->vout(neg).
    json cell; cell["name"] = "cuk-cell";
    if (d.isolated) {
        const bool sync = d.synchronousRectifier;
        std::vector<json> comps = {comp("L1", L1), comp("Q1", mq), comp("C1", c1), comp("T1", t1),
                                   comp("C1b", c1b), comp("L2", L2), comp("Rrc_sw", rsnub), comp("Crc_sw", csnub)};
        std::vector<json> gnd = {pin("Q1", "source"), pin("T1", "primary_end"), pin("T1", "secondary1_end"),
                                 pin("Crc_sw", "2"), prt("gnd")};
        std::vector<json> nodeB = {pin("C1b", "2"), l2s(), pin("Rrc_sw", "1")};
        std::vector<json> ports = {port("vin"), port("gnd"), port("vout"), port("gate")};
        if (sync) {
            comps.push_back(comp("Q2", syncFet)); comps.push_back(comp("D2", bodyD));
            nodeB.push_back(pin("Q2", "drain")); nodeB.push_back(pin("D2", "anode"));
            gnd.push_back(pin("Q2", "source")); gnd.push_back(pin("D2", "cathode"));
            ports.push_back(port("gate2"));
        } else {
            comps.push_back(comp("D1", md));
            nodeB.push_back(pin("D1", "anode")); gnd.push_back(pin("D1", "cathode"));
        }
        cell["ports"] = ports;
        cell["components"] = comps;
        std::vector<json> conns = {
            conn("vin_net", {l1s(), prt("vin")}),
            conn("nodeA",   {l1e(), pin("Q1", "drain"), pin("C1", "1")}),
            conn("nodeP",   {pin("C1", "2"), pin("T1", "primary_start")}),          // primary coupling -> xfmr pri
            conn("nodeS",   {pin("T1", "secondary1_start"), pin("C1b", "1")}),       // xfmr sec -> secondary coupling
            conn("nodeB",   nodeB),
            conn("snub",    {pin("Rrc_sw", "2"), pin("Crc_sw", "1")}),
            conn("gnd_net", gnd),
            conn("vout_net",{l2e(), prt("vout")}),
            conn("gate_net",{pin("Q1", "gate"), prt("gate")})};
        if (sync) conns.push_back(conn("gate2_net", {pin("Q2", "gate"), prt("gate2")}));
        cell["connections"] = conns;
    } else if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = magComps({comp("Q1", mq), comp("C1", c1), comp("Q2", syncFet), comp("D2", bodyD),
                                       comp("Rrc_sw", rsnub), comp("Crc_sw", csnub)});
        cell["connections"] = json::array({
            conn("vin_net", {l1s(), prt("vin")}),
            conn("nodeA",   {l1e(), pin("Q1", "drain"), pin("C1", "1")}),
            // node B: coupling cap -> sync MOSFET drain (+ body-diode anode) + output inductor + RC snubber
            conn("nodeB",   {pin("C1", "2"), pin("Q2", "drain"), pin("D2", "anode"), l2s(), pin("Rrc_sw", "1")}),
            conn("snub",    {pin("Rrc_sw", "2"), pin("Crc_sw", "1")}),
            conn("gnd_net", {pin("Q1", "source"), pin("Q2", "source"), pin("D2", "cathode"), pin("Crc_sw", "2"), prt("gnd")}),
            conn("vout_net",{l2e(), prt("vout")}),
            conn("gate_net",{pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = magComps({comp("Q1", mq), comp("C1", c1), comp("D1", md),
                                       comp("Rrc_sw", rsnub), comp("Crc_sw", csnub)});
        cell["connections"] = json::array({
            conn("vin_net", {l1s(), prt("vin")}),
            conn("nodeA",   {l1e(), pin("Q1", "drain"), pin("C1", "1")}),
            // node B: coupling cap -> freewheel diode (anode) + output inductor + diode RC snubber
            conn("nodeB",   {pin("C1", "2"), pin("D1", "anode"), l2s(), pin("Rrc_sw", "1")}),
            conn("snub",    {pin("Rrc_sw", "2"), pin("Crc_sw", "1")}),
            conn("gnd_net", {pin("Q1", "source"), pin("D1", "cathode"), pin("Crc_sw", "2"), prt("gnd")}),
            conn("vout_net",{l2e(), prt("vout")}),     // negative output
            conn("gate_net",{pin("Q1", "gate"), prt("gate")})});
    }

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
    // Output voltage is NEGATIVE (inverting): the synthesized load measures v(Vout) < 0.
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = -d.outputVoltageMag; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("cukCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("cukCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("cukCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("cukCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "cukCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "cukCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::CUK_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
