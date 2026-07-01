#include "Ahb.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }

// Operating duty D the turns ratio is sized at. The AHB gain Vo=2*Vin*D*(1-D)/n is NON-MONOTONIC,
// peaking at D=0.5. n is sized so the secondary delivers (Vo+Vd) at THIS duty, so the open-loop deck
// hits Vo at D regardless of D's value (mkf-equivalence). But the closed-loop regulator (real DATASHEET
// deck) must climb the gain curve toward the D=0.5 peak to cover the REAL rectifier drop (~2-3 V, far
// more than the ideal-diode Vd the turns ratio compensates) + transformer leakage. Sizing at D=0.45 put
// the design point right next to the peak -> the peak/design gain ratio is ~1.01 (zero headroom) and the
// real deck tops out ~11 V, never reaching 12 V. Sizing LOWER (0.30) moves the design point away from the
// peak: peak/design = 0.5/(2*D*(1-D)) ~= 1.19 -> ~19 % headroom, so the lossy real deck reaches Vo at a
// stable duty (~0.40) BELOW the high-duty circulating-current cliff (pin runs away as D->0.5). (abt #58)
constexpr double kDuty       = 0.30;
constexpr double kDeadFrac   = 0.01;   // 100ns dead time at 100kHz between the complementary switches
constexpr double kRippleRatio = 0.30;  // output-inductor current ripple

// Kirchhoff's ideal rectifier diode drop (CIAS ideal-diode .model D(IS=1e-14); Vd=Vt*ln(I/IS)). The
// full-bridge rectifier conducts through TWO diodes in series, so the design compensates the turns
// ratio for 2*Vd at rated output current — mirrors MKF's compute_turns_ratio (FULL_BRIDGE uses 2*Vd).
} // namespace

AhbDesign design_ahb(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    AhbDesign d{};
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

    const double Vo = d.outputVoltage, Fs = d.switchingFrequency, Tsw = 1.0 / Fs;
    const double Io = d.outputPower / Vo;
    const double D = cfg::get(d.config, "operatingDutyCycle", kDuty);
    d.dutyCycle = D;
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", kDeadFrac);
    // Rectifier variant (FB default; AHB's natural fourth MAS variant is AHB_FLYBACK, not voltage-doubler).
    d.rectifierType = parse_rectifier_type(cfg::get_str(d.config, "rectifierType", "fullBridge"),
                                           RectifierType::FullBridge);
    if (d.rectifierType == RectifierType::VoltageDoubler)
        throw std::runtime_error("Kirchhoff AHB: voltageDoubler rectifier not supported "
                                 "(AHB variants: fullBridge, centerTapped, currentDoubler)");
    // Path diodes set the drop compensation: FB stacks two (Vo+2Vd); CT/CD conduct through one (Vo+Vd).
    const double Vdtot = rectifier_path_diodes(d.rectifierType) * req::dideal_diode_drop(Io);

    // Turns ratio sized at NOMINAL Vin (the operating point the open-loop deck runs at) so Kirchhoff
    // delivers the target Vo there. Vo + Vdtot = 2*D*(1-D)*Vin_nom/n. (MKF sizes at Vin_min and runs the
    // deck at Vin_nom, landing slightly under Vo — both are within the equivalence band.)
    double n = 2.0 * D * (1.0 - D) * d.inputVoltage / (Vo + Vdtot);
    // CURRENT_DOUBLER delivers ~half the winding-reflected voltage, so halve n to hit the same Vo.
    if (d.rectifierType == RectifierType::CurrentDoubler)
        n *= cfg::get(d.config, "cdOutputFactor", 0.5);
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(n * 100.0) / 100.0);
    n = d.turnsRatio;

    // Magnetizing inductance for ZVS assist: target Im_pk = 10% of reflected load current, sized at
    // Vin_max. Lm = (1-D)*Vin_max*D*Tsw/(2*Im_target).
    const double ImTarget = std::max(0.10 * Io / n, 1e-3);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        (1.0 - D) * vinMax * D * Tsw / (2.0 * ImTarget));

    // Output inductor (CCM): Lo = Vo*(1 - 2*D*(1-D))/(ripple*Io*Fs).
    d.outputInductance = Vo * (1.0 - 2.0 * D * (1.0 - D)) / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Io * Fs);

    // DC-blocking cap sized for <=5% ripple of V_Cb=(1-D)*Vin (Cb = Ipri_pk*D/(Fs*dVCb)).
    const double dILm = (1.0 - D) * vinMax * D * Tsw / d.magnetizingInductance;
    const double IpriPk = Io / n + dILm / 2.0;
    const double VCb = (1.0 - D) * d.inputVoltage;
    const double dVCb = std::max(0.05 * VCb, 1e-3);
    d.dcBlockingCapacitance = IpriPk * D / (Fs * dVCb);

    d.loadResistance = Vo * Vo / d.outputPower;
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

json build_ahb_tas(const AhbDesign& d) {
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
    const double Vo = d.outputVoltage, Io = d.outputPower / Vo, Dn = d.dutyCycle;

    // --- transformer winding stresses (worst-case current corner = Vin_max, max magnetizing ramp) ---
    // Magnetizing ripple dILm reuses line ~69's volt-second balance at Vin_max. Primary winding carries
    // the reflected load current (Io/N) plus the magnetizing current; secondary carries Io while
    // conducting. Voltages: primary sees +Vin_max during the power-transfer interval; secondary the
    // reflected (Vin_max/N) — both gated by the duty. Evaluated at the nominal operating point for V.
    const double dILm = (1.0 - Dn) * d.inputVoltageMax * Dn * Tsw / Lm; // magnetizing pk-pk ripple (line ~69)
    const double IpriCtr = Io / N;                                      // reflected load current center
    const double IpriPk  = IpriCtr + dILm / 2.0;                        // + magnetizing ripple (line ~70)
    const double IpriRms = std::sqrt(Dn) * std::sqrt(IpriCtr * IpriCtr + dILm * dILm / 12.0);
    const double IsecCtr = Io;                                          // secondary carries Io when conducting
    const double dIsecRefl = dILm * N;                                  // ripple reflected to the secondary
    const double IsecPk  = Io + dIsecRefl / 2.0;
    const double IsecRms = std::sqrt(Dn) * std::sqrt(IsecCtr * IsecCtr + dIsecRefl * dIsecRefl / 12.0);
    // Winding voltages (volt-second basis at nominal Vin): primary +Vin during D, ~0 during freewheel;
    // secondary mirrors at Vin/N. Two-level rectangular -> vRms = sqrt(D*Von^2).
    const double Vp = d.inputVoltage, Vs = Vp / N;
    const double vPriPk = Vp, vPriPkPk = Vp, vPriRms = std::sqrt(Dn) * Vp;
    const double vSecPk = Vs, vSecPkPk = Vs, vSecRms = std::sqrt(Dn) * Vs;

    // --- output inductor stresses: avg=Io, ripple dILo from Lo volt-seconds (CCM) ---
    const double dILo = Vo * (1.0 - 2.0 * Dn * (1.0 - Dn)) / (d.outputInductance * fsw);  // pk-pk (= ripple sizing)
    const double IloPk  = Io + dILo / 2.0;
    const double IloRms = std::sqrt(Io * Io + dILo * dILo / 12.0);
    // Output-inductor voltage: Vin/N - Vo during conduction (~2*D), -Vo during freewheel; |swing| ~ Vo.
    const double vLoPk = std::max(std::abs(Vs - Vo), Vo), vLoPkPk = Vs, vLoRms = Vo;

    // --- semiconductor ratings (sourceable requirements) ---
    // Primary half-bridge switches Q1/Q2: each blocks the full bus (Vin_max) when off; they carry the
    // primary current (reflected load + magnetizing), peak = IpriPk, rms = IpriRms. Anti-parallel D1/D2
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

    // DC-blocking capacitor Cb.
    json cb; cb["capacitor"] = json::object();
    cb["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.dcBlockingCapacitance;
    cb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2;

    // Transformer windings: FULL_BRIDGE has ONE full secondary (turnsRatios=[N]); CENTER_TAPPED has TWO
    // half-windings (turnsRatios=[N,N]); CURRENT_DOUBLER uses ONE (wpo=1, both ends drive the inductors).
    const int wpo = rectifier_windings_per_output(d.rectifierType);
    std::vector<std::string> isoSides{"primary"};
    std::vector<double> turnsRatios;
    for (int w = 0; w < wpo; ++w) { isoSides.push_back("secondary"); turnsRatios.push_back(N); }

    // Transformer excitations from the SINGLE FHA source — the SPICE-validated analytical AHB solver.
    // FULL_BRIDGE (default) emits Primary + ONE secondary; CENTER_TAPPED emits Primary + two half-windings;
    // both line up with the wpo winding structure. CURRENT_DOUBLER is not modelled by the solver, so it
    // keeps the inline model. currentRippleRatio is chosen so the solver's internal Lo matches the deck's
    // actual output inductor (ripple = dILo/Io). This CORRECTS the inline secondary model, which used a
    // zero DC offset (real deck: Io*(2D-1) bias, carried by the gapped energy-transfer transformer) and a
    // sqrt(Dn) RMS factor (AHB conducts BOTH intervals — no freewheel — so RMS ~ Io); verified vs ngspice.
    namespace AN = Kirchhoff::analytical;
    std::vector<json> xwindings;
    if (d.rectifierType == RectifierType::FullBridge || d.rectifierType == RectifierType::CenterTapped) {
        const AN::SrcRectifier rect = (d.rectifierType == RectifierType::CenterTapped)
                                      ? AN::SrcRectifier::CENTER_TAPPED : AN::SrcRectifier::FULL_BRIDGE;
        const double rippleRatio = dILo / Io;   // makes the solver's compute_lo_min Lo == d.outputInductance
        const MAS::OperatingPoint aopT1 = AN::analytical_asymmetric_half_bridge(
            d.inputVoltage, {Vo}, {Io}, {N}, fsw, Lm, Dn, rippleRatio, 0.0, rect);
        xwindings = AN::excitations_processed(aopT1);
    } else {
        xwindings.push_back(req::winding_excitation("ahbPrimary", fsw, IpriPk, IpriRms, 0.0, dILm, Dn,
                                vPriPk, vPriRms, 0.0, vPriPkPk));
        for (int w = 0; w < wpo; ++w)
            xwindings.push_back(req::winding_excitation("ahbSecondary", fsw, IsecPk, IsecRms, 0.0, dIsecRefl, Dn,
                                    vSecPk, vSecRms, 0.0, vSecPkPk));
    }
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, isoSides, std::nullopt, 25.0, xwindings,
        /*turnsRatioIsCeiling=*/{}, /*lmIsMinimum (AHB transformer is a forward-class ENERGY-TRANSFER
          transformer with a SEPARATE output inductor; Lm is only ZVS assist -> maximise it UNGAPPED so the
          adviser does not gap the core and overfill the winding into a high-leakage, high-DCR degenerate
          coil that drops the real-deck gain below target. K 0.94->0.999. Same fix as PSFB, abt #56/#58)=*/true);

    // Output inductor Lo (single-winding magnetic, DC-biased at Io).
    json lout; lout["magnetic"] = json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IloPk, IloRms, Io, dILo, std::nullopt,
                                    vLoPk, vLoRms, 0.0, vLoPkPk)});

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    // Snubber caps at the switch midpoint (floats during dead time) and the secondary rectifier nodes
    // (float when all four rectifier diodes are off). Body diodes clamp the midpoint; the small
    // node-to-gnd caps tame dV/dt — physically real device/winding capacitance. (PSFB template.)
    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        return c; };

    // CURRENT_DOUBLER second output inductor + loop-breaker.
    auto outInductor2 = [&]() { json m; m["magnetic"] = json::object();
        m["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IloPk, IloRms, Io, dILo, std::nullopt,
                                    vLoPk, vLoRms, 0.0, vLoPkPk)});
        return m; };
    auto loopBreakR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"]; dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = cfg::loop_breaker_res(d.config, d.loadResistance);
        dr["powerRating"] = 0.25; dr["role"] = "balancing"; return c; };

    json cell; cell["name"] = "ahb-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate1"), port("gate2")});

    // Half bridge + DC-blocking cap + transformer primary — identical for every rectifier variant.
    std::vector<json> comps{
        comp("Q1", mosfetReq), comp("Q2", mosfetReq), comp("D1", diode()), comp("D2", diode()),
        comp("Cb", cb), comp("T1", xfmr), comp("Csw", snub())};
    std::vector<json> conns{
        conn("vin_net",  {pin("Q1", "drain"), pin("D1", "cathode"), pin("Cb", "1"), prt("vin")}),
        conn("sw_net",   {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("D1", "anode"), pin("D2", "cathode"),
                          pin("T1", "primary_end"), pin("Csw", "1")}),
        conn("cb_mid",   {pin("Cb", "2"), pin("T1", "primary_start")})};
    std::vector<json> gndEps{pin("Q2", "source"), pin("D2", "anode"), pin("Csw", "2")};

    switch (d.rectifierType) {
    case RectifierType::FullBridge: {
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Dr3", diodeReq),
            comp("Dr4", diodeReq), comp("Lout", lout), comp("CsnSA", snub()), comp("CsnSB", snub())});
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
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Lout", lout),
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
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq),
            comp("Lout", lout), comp("Lo2", outInductor2()), comp("Rlb", loopBreakR()),
            comp("CsnSA", snub()), comp("CsnSB", snub())});
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
        throw std::runtime_error("Kirchhoff AHB: voltageDoubler rectifier not supported");
    }
    conns.push_back(conn("gate1_net", {pin("Q1", "gate"), prt("gate1")}));
    conns.push_back(conn("gate2_net", {pin("Q2", "gate"), prt("gate2")}));
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
        pstage("ahbCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("ahbCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("ahbCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("ahbCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Complementary drive: Q1 on for D, Q2 on for (1-D), with a dead time between them. Q1 leads at
    // phase 0; Q2 starts a dead-band after Q1 turns off (phase = (D + deadFrac)*360) and is trimmed to
    // end a dead-band before the period wraps.
    const double D = d.dutyCycle, dt = d.deadFraction;
    auto stim = [&](const char* sw, double duty, double phaseDeg) {
        json st; st["stage"] = "ahbCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({
        stim("Q1", D, 0.0),
        stim("Q2", (1.0 - D) - 2.0 * dt, (D + dt) * 360.0)});
    req::finalize_control_seeds(tas, Topology::ASYMMETRIC_HALF_BRIDGE_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
