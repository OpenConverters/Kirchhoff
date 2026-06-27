#include "Ahb.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }

constexpr double kDuty       = 0.45;   // operating duty D (MKF AHB maximumDutyCycle default / OP duty)
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
    const double Vdtot = 2.0 * req::dideal_diode_drop(Io);   // full-bridge rectifier: two diodes in series

    // Turns ratio sized at NOMINAL Vin (the operating point the open-loop deck runs at) so Kirchhoff
    // delivers the target Vo there. FULL_BRIDGE: Vo + 2*Vd = 2*D*(1-D)*Vin_nom/n. (MKF sizes at Vin_min
    // and runs the deck at Vin_nom, landing slightly under Vo — both are within the equivalence band.)
    double n = 2.0 * D * (1.0 - D) * d.inputVoltage / (Vo + Vdtot);
    d.turnsRatio = std::round(n * 100.0) / 100.0;
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
    auto diode  = []() { json j; j["semiconductor"]["diode"] = json::object(); return j; };

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
    const double ratedVds = d.inputVoltageMax / cfg::v_derate(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IpriRms * IpriRms);
    json mosfetReq; mosfetReq["semiconductor"]["mosfet"] = json::object();
    mosfetReq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpriPk, maxRdsOn, 125.0);
    // Secondary full-bridge rectifier Dr1..Dr4: each off diode blocks the secondary winding voltage
    // (peak Vs); each carries the output current while conducting. REAL rectifiers -> req::diode.
    const double ratedVr  = vSecPk / cfg::v_derate(d.config);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;
    json diodeReq; diodeReq["semiconductor"]["diode"] = json::object();
    diodeReq["inputs"]["designRequirements"] = req::diode(ratedVr, Io / 0.7, maxVf, 0.05 * Tsw);

    // DC-blocking capacitor Cb.
    json cb; cb["capacitor"] = json::object();
    cb["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.dcBlockingCapacitance;
    cb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2;

    // 2-winding transformer (primary + one full secondary), turnsRatios = [N] -> 2 excitations.
    std::vector<std::string> isoSides{"primary", "secondary"};
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {N}, isoSides, std::nullopt, 25.0, {
        req::winding_excitation("ahbPrimary",   fsw, IpriPk, IpriRms, 0.0, dILm, Dn,
                                vPriPk, vPriRms, 0.0, vPriPkPk),
        req::winding_excitation("ahbSecondary", fsw, IsecPk, IsecRms, 0.0, dIsecRefl, Dn,
                                vSecPk, vSecRms, 0.0, vSecPkPk)});

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

    json cell; cell["name"] = "ahb-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate1"), port("gate2")});
    cell["components"] = json::array({
        comp("Q1", mosfetReq), comp("Q2", mosfetReq), comp("D1", diode()), comp("D2", diode()),
        comp("Cb", cb), comp("T1", xfmr),
        comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Dr3", diodeReq), comp("Dr4", diodeReq),
        comp("Lout", lout), comp("Csw", snub()), comp("CsnSA", snub()), comp("CsnSB", snub())});
    cell["connections"] = json::array({
        // Half bridge: Q1 high-side (vin->sw), Q2 low-side (sw->gnd) + anti-parallel body diodes.
        conn("vin_net",  {pin("Q1", "drain"), pin("D1", "cathode"), pin("Cb", "1"), prt("vin")}),
        conn("sw_net",   {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("D1", "anode"), pin("D2", "cathode"),
                          pin("T1", "primary_end"), pin("Csw", "1")}),
        // DC-blocking cap in series with the primary: vin -> Cb -> primary(cb_mid->sw).
        conn("cb_mid",   {pin("Cb", "2"), pin("T1", "primary_start")}),
        // Secondary -> full-bridge rectifier (4 diodes) -> output inductor.
        conn("sec_a",    {pin("T1", "secondary1_start"), pin("Dr1", "anode"), pin("Dr3", "cathode"),
                          pin("CsnSA", "1")}),
        conn("sec_b",    {pin("T1", "secondary1_end"),   pin("Dr2", "anode"), pin("Dr4", "cathode"),
                          pin("CsnSB", "1")}),
        conn("out_rect", {pin("Dr1", "cathode"), pin("Dr2", "cathode"), pin("Lout", "primary_start")}),
        conn("vout_net", {pin("Lout", "primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("D2", "anode"),
                          pin("Dr3", "anode"), pin("Dr4", "anode"),
                          pin("Csw", "2"), pin("CsnSA", "2"), pin("CsnSB", "2"), prt("gnd")}),
        conn("gate1_net", {pin("Q1", "gate"), prt("gate1")}),
        conn("gate2_net", {pin("Q2", "gate"), prt("gate2")})});

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
    req::finalize_control_seeds(tas, "asymmetricHalfBridgeConverter");  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
