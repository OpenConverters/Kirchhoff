#include "Pshb.hpp"
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
constexpr double kCommandedDuty = 0.7;   // sizes n at MKF's operating point (phi=126)
// The NPC leg + dead time + body-diode drops deliver slightly less than the commanded width, so the
// outer (power-transfer) switches are widened by this trim to land Vout on MKF's (the phase-shift
// control knob; sizing n is left at the commanded duty so it matches MKF's turns ratio).
constexpr double kOuterTrim     = 0.01;
constexpr double kRippleRatio   = 0.3;
constexpr double kDeadFrac      = 0.01;
} // namespace

PshbDesign design_pshb(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PshbDesign d{};
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
    d.inputVoltageMin = vinMin; d.inputVoltageMax = vinMax;

    const double Vo = d.outputVoltage, Fs = d.switchingFrequency, Io = d.outputPower / Vo;
    const double Vhb = 0.5 * d.inputVoltage;     // split-cap half bus
    const double Dcmd = cfg::get(d.config, "commandedDuty", kCommandedDuty);
    d.commandedDuty = Dcmd;
    const double Vdtot = 2.0 * req::dideal_diode_drop(Io);   // full-bridge rectifier

    double nSeed = Vhb * Dcmd / (Vo + Vdtot);
    double Lr = std::min(2e-6, (Io > 0) ? 0.02 * std::max(nSeed, 0.1) * Vhb / (4.0 * Io * Fs) : 2e-6);
    Lr = std::max(Lr, 1e-7);
    d.seriesInductance = Lr;

    double n = nSeed, Deff = Dcmd;
    for (int it = 0; it < 8; ++it) {
        double dcl = 4.0 * Lr * Io * Fs / (n * Vhb);
        Deff = std::max(0.0, Dcmd - dcl);
        double nNew = (Deff > 1e-3) ? Vhb * Deff / (Vo + Vdtot) : n;
        if (std::abs(nNew - n) < 1e-3 * std::max(n, 1.0)) { n = nNew; break; }
        n = nNew;
    }
    d.effectiveDuty = Deff;
    d.turnsRatio = std::round(n * 100.0) / 100.0;
    d.outputInductance = Vo * (1.0 - Deff) / (Fs * cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Io);
    double ImTarget = 0.1 * Io / d.turnsRatio;
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        std::max((ImTarget > 0) ? Vhb * Deff / (4.0 * Fs * ImTarget) : 20.0 * Lr, 20.0 * Lr));
    d.splitCapacitance = 470e-6;
    d.phaseDeg = 180.0 * Dcmd;
    d.switchDuty = 0.5;
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", kDeadFrac);
    d.loadResistance = Vo * Vo / d.outputPower;
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

json build_pshb_tas(const PshbDesign& d) {
    auto port = [](const char* n){ json p; p["name"]=n; return p; };
    auto pin  = [](const char* c, const char* p){ json e; e["component"]=c; e["pin"]=p; return e; };
    auto prt  = [](const char* p){ json e; e["port"]=p; return e; };
    auto conn = [](const char* name, std::vector<json> eps){ json c; c["name"]=name; c["endpoints"]=eps; return c; };
    auto comp = [](const char* name, json data){ json c; c["name"]=name; c["data"]=data; return c; };
    auto bind = [](const char* p, const char* t){ json b; b["port"]=p; b["type"]=t; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb){
        json s; s["name"]=name; s["role"]=role; s["circuit"]=brick; s["inputPort"]=inb; s["outputPort"]=outb; return s; };
    auto sp = [](const char* st, const char* po){ json e; e["stage"]=st; e["port"]=po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps){
        json c; c["name"]=name; c["kind"]=kind; if(dir[0]) c["direction"]=dir; c["endpoints"]=eps; return c; };
    auto diode  = [&](){ json j; j["semiconductor"]["diode"]=json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, Tsw = 1.0 / fsw;
    const double Vo = d.outputVoltage, Io = d.outputPower / Vo;
    const double Vhb = 0.5 * d.inputVoltage, Deff = d.effectiveDuty;   // split-cap half bus (sizing basis)

    // --- stresses ---
    // Output inductor (CCM): avg=Io, ripple from the Lo sizing volt-seconds.
    const double dILo = Vo * (1.0 - Deff) / (d.outputInductance * fsw);   // pk-pk (= ripple sizing)
    const double IloPk  = Io + dILo / 2.0;
    const double IloRms = std::sqrt(Io * Io + dILo * dILo / 12.0);
    const double vLoPk = std::max(std::abs(Vhb / N - Vo), Vo), vLoPkPk = Vhb / N, vLoRms = Vo;

    // Transformer: the primary swings +-Vhb (= +-Vin/2, the split-cap half bus). PRIMARY carries the
    // reflected output current (Io/N) plus magnetizing; SECONDARY carries Io. Magnetizing ramp from
    // Vhb*Deff over the active half (same volt-seconds that sized Lm). vRms = sqrt(Deff)*Von.
    const double dILm = Vhb * Deff * Tsw / (2.0 * Lm);   // magnetizing pk-pk
    const double IpriCtr = Io / N;
    const double IpriPk  = IpriCtr + dILm / 2.0;
    const double IpriRms = std::sqrt(Deff) * std::sqrt(IpriCtr * IpriCtr + dILm * dILm / 12.0);
    const double IsecPk  = Io + dILo / 2.0;             // secondary feeds Lout
    const double IsecRms = std::sqrt(Deff) * std::sqrt(Io * Io + dILo * dILo / 12.0);
    const double Vs = Vhb / N;
    const double vPriPk = Vhb, vPriPkPk = 2.0 * Vhb, vPriRms = std::sqrt(Deff) * Vhb;
    const double vSecPk = Vs,  vSecPkPk = 2.0 * Vs,  vSecRms = std::sqrt(Deff) * Vs;

    // --- semiconductor ratings (sourceable requirements) ---
    // 3-level NPC stack switches S1..S4: each device blocks the split-cap half bus (Vhb = Vin/2) when
    // off; they carry the primary current (reflected load + magnetizing), peak = IpriPk, rms = IpriRms.
    // Anti-parallel Db1..Db4 are the FETs' body diodes (left requirement-less).
    const double ratedVds = Vhb / cfg::v_derate(d.config);
    const double maxRdsOn  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IpriRms * IpriRms);
    json mosfetReq; mosfetReq["semiconductor"]["mosfet"] = json::object();
    mosfetReq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpriPk, maxRdsOn, 125.0);
    // NPC neutral-point clamp diodes DC1/DC2: REAL diodes (not anti-parallel to any single FET) that
    // clamp the inner nodes to the cap midpoint and freewheel the primary current; each blocks Vhb.
    const double ratedVrClamp = Vhb / cfg::v_derate(d.config);
    const double maxVfClamp    = (ratedVrClamp < 100.0) ? 0.6 : 1.2;
    json clampDiodeReq; clampDiodeReq["semiconductor"]["diode"] = json::object();
    clampDiodeReq["inputs"]["designRequirements"] = req::diode(ratedVrClamp, IpriPk, maxVfClamp, 0.05 * Tsw);
    // Secondary full-bridge rectifier Dr1..Dr4: each off diode blocks the secondary winding voltage
    // (peak Vs); each carries the output current while conducting. REAL rectifiers -> req::diode.
    const double ratedVr  = vSecPk / cfg::v_derate(d.config);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;
    json diodeReq; diodeReq["semiconductor"]["diode"] = json::object();
    diodeReq["inputs"]["designRequirements"] = req::diode(ratedVr, Io / 0.7, maxVf, 0.05 * Tsw);

    // Series resonant/leakage inductor Lr (single-winding magnetic): full primary current, AC (no DC).
    json lr; lr["magnetic"]=json::object();
    lr["inputs"] = req::magnetic_inputs(d.seriesInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpriPk, IpriRms, 0.0, 2.0 * IpriCtr, std::nullopt,
                                    Vhb, std::sqrt(Deff) * Vhb, 0.0, 2.0 * Vhb)});

    // 2-winding transformer (primary + one full secondary), turnsRatios = [N] -> 2 excitations.
    std::vector<std::string> isoSides{"primary", "secondary"};
    json xfmr; xfmr["magnetic"]=json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, {N}, isoSides, std::nullopt, 25.0, {
        req::winding_excitation("pshbPrimary",   fsw, IpriPk, IpriRms, 0.0, dILm, Deff,
                                vPriPk, vPriRms, 0.0, vPriPkPk),
        req::winding_excitation("pshbSecondary", fsw, IsecPk, IsecRms, 0.0, dILo, Deff,
                                vSecPk, vSecRms, 0.0, vSecPkPk)});

    // Output inductor Lo (single-winding magnetic, DC-biased at Io).
    json lout; lout["magnetic"]=json::object();
    lout["inputs"] = req::magnetic_inputs(d.outputInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IloPk, IloRms, Io, dILo, std::nullopt,
                                    vLoPk, vLoRms, 0.0, vLoPkPk)});
    auto cap = [](double c, double v){ json j; j["capacitor"]=json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"]=c;
        j["inputs"]["designRequirements"]["ratedVoltage"]=v; return j; };
    json csplit = cap(d.splitCapacitance, d.inputVoltage * 2);
    json capd   = cap(d.outputCapacitance, d.outputVoltage * 2);
    json snub   = cap(cfg::node_snubber_cap(d.config), (d.inputVoltage + d.outputVoltage) * 3);

    json cell; cell["name"]="pshb-cell";
    cell["ports"]=json::array({port("vin"),port("gnd"),port("vout"),port("g1"),port("g2"),port("g3"),port("g4")});
    cell["components"]=json::array({
        comp("CsHi",csplit), comp("CsLo",csplit),
        comp("S1",mosfetReq), comp("S2",mosfetReq), comp("S3",mosfetReq), comp("S4",mosfetReq),
        comp("Db1",diode()), comp("Db2",diode()), comp("Db3",diode()), comp("Db4",diode()),
        comp("DC1",clampDiodeReq), comp("DC2",clampDiodeReq),
        comp("Lr",lr), comp("T1",xfmr),
        comp("Dr1",diodeReq), comp("Dr2",diodeReq), comp("Dr3",diodeReq), comp("Dr4",diodeReq),
        comp("Lout",lout), comp("CsnB",snub), comp("CsnSA",snub), comp("CsnSB",snub)});
    cell["connections"]=json::array({
        conn("vin_net",  {pin("CsHi","1"), pin("S1","drain"), pin("Db1","cathode"), prt("vin")}),
        conn("mid_cap",  {pin("CsHi","2"), pin("CsLo","1"), pin("DC1","anode"), pin("DC2","cathode"),
                          pin("T1","primary_end")}),
        conn("nH",       {pin("S1","source"), pin("S2","drain"), pin("Db1","anode"), pin("Db2","cathode"),
                          pin("DC1","cathode")}),
        conn("bridge_a", {pin("S2","source"), pin("S3","drain"), pin("Db2","anode"), pin("Db3","cathode"),
                          pin("Lr","primary_start"), pin("CsnB","1")}),
        conn("nL",       {pin("S3","source"), pin("S4","drain"), pin("Db3","anode"), pin("Db4","cathode"),
                          pin("DC2","anode")}),
        conn("pri_x",    {pin("Lr","primary_end"), pin("T1","primary_start")}),
        conn("sec_a",    {pin("T1","secondary1_start"), pin("Dr1","anode"), pin("Dr3","cathode"), pin("CsnSA","1")}),
        conn("sec_b",    {pin("T1","secondary1_end"),   pin("Dr2","anode"), pin("Dr4","cathode"), pin("CsnSB","1")}),
        conn("out_rect", {pin("Dr1","cathode"), pin("Dr2","cathode"), pin("Lout","primary_start")}),
        conn("vout_net", {pin("Lout","primary_end"), prt("vout")}),
        conn("gnd_net",  {pin("CsLo","2"), pin("S4","source"), pin("Db4","anode"),
                          pin("Dr3","anode"), pin("Dr4","anode"),
                          pin("CsnB","2"), pin("CsnSA","2"), pin("CsnSB","2"), prt("gnd")}),
        conn("g1_net",{pin("S1","gate"),prt("g1")}), conn("g2_net",{pin("S2","gate"),prt("g2")}),
        conn("g3_net",{pin("S3","gate"),prt("g3")}), conn("g4_net",{pin("S4","gate"),prt("g4")})});

    json filt; filt["name"]="output-filter";
    filt["ports"]=json::array({port("in"),port("rtn")});
    filt["components"]=json::array({comp("Cout",capd)});
    filt["connections"]=json::array({conn("out",{pin("Cout","1"),prt("in")}), conn("ret",{pin("Cout","2"),prt("rtn")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"]=d.efficiency; dreq["inputType"]="dc";
    dreq["inputVoltage"]={{"minimum",d.inputVoltageMin},{"nominal",d.inputVoltage},{"maximum",d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"]=d.switchingFrequency;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=d.outputVoltage; o["regulation"]="voltage"; dreq["outputs"]=json::array({o}); }
    { json op; op["name"]="full_load"; op["inputVoltage"]=d.inputVoltage; op["ambientTemperature"]=25.0;
      json o; o["name"]="out"; o["power"]=d.outputPower; op["outputs"]=json::array({o});
      tas["inputs"]["operatingPoints"]=json::array({op}); }
    tas["topology"]["stages"]=json::array({
        req::control_stage("pwmController"),
        req::control_stage("gateDriver", "gate-driver", "UDR"),
        pstage("pshbCell","switchingCell",cell,bind("vin","dcBus"),bind("vout","pulsatingDc")),
        pstage("filter","outputFilter",filt,bind("in","pulsatingDc"),bind("in","dcOutput"))});
    tas["topology"]["interStageConnections"]=json::array({
        isc("Vin","externalPort","input",{sp("pshbCell","vin")}),
        isc("GND","externalPort","input",{sp("pshbCell","gnd"),sp("filter","rtn")}),
        isc("Vout","externalPort","output",{sp("pshbCell","vout"),sp("filter","in")})});
    json an; an["type"]="transient"; an["stopTime"]=0.004; an["maximumTimeStep"]=5e-8;
    tas["simulation"]["analyses"]=json::array({an});
    // 3-level NPC drive: inner pair S2/S3 ~50% complementary (S3 phase 180); outer pair S1/S4 narrower
    // (D_cmd/2 of the period) and in phase with S2/S4-leg, setting the +/-Vin/2 power-transfer width
    // (the phase-shift modulation). Dead time trims the inner pair.
    const double D = d.commandedDuty, dt = d.deadFraction, outer = D / 2.0 + cfg::get(d.config, "outerTrim", kOuterTrim);
    auto stim = [&](const char* sw, double duty, double phaseDeg){
        json st; st["stage"]="pshbCell"; st["component"]=sw; st["signal"]="gate";
        st["waveform"]["type"]="pwm"; st["waveform"]["frequency"]=d.switchingFrequency;
        st["waveform"]["dutyCycle"]=duty; st["waveform"]["phase"]=phaseDeg; return st; };
    tas["simulation"]["stimulus"]=json::array({
        stim("S1", outer, 0.0),
        stim("S2", 0.5 - dt, 0.0),
        stim("S3", 0.5 - dt, (0.5 + dt) * 360.0),
        stim("S4", outer, 180.0)});
    req::finalize_control_seeds(tas, "phaseShiftedHalfBridgeConverter");  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
