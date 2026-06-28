#include "Src.hpp"
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
constexpr double kBridgeFactor  = 0.5;   // half-bridge: Vo_fha = 0.5·Vin
constexpr double kQualityFactor = 2.0;   // MKF Src default
constexpr double kLmRatio       = 10.0;  // Lm = 10·Lr (large, keeps Lm out of the resonance)
constexpr double kSwitchDuty    = 0.45;
} // namespace

SrcDesign design_src(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    SrcDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
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

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;
    const double Iout = d.outputPower / Vo;

    // Turns ratio: n = k_bridge·Vin/(Vo+Vd). The SRC tank gain PEAKS at unity at fr (it can only step
    // DOWN off resonance — there is no parallel Lm to boost like the LLC), so the realized output at fr is
    // 0.5·Vin/n MINUS the rectifier drop Vd. Without the +Vd compensation the converter tops out at Vo−Vd
    // and can NEVER regulate to Vo (at 12 V the 0.85 V CT-rectifier drop is ~7% — the closed-loop search
    // lands ~11.1 V and reports regulated=False). Compensating Vd puts the fr peak at Vo so the regulator
    // can hit target just off resonance. (Mirrors the LLC n; Vd is negligible at 48 V, ~7% at 12 V.)
    const double Vd = req::dideal_diode_drop(Iout);
    double n = (cfg::get(d.config, "bridgeFactor", kBridgeFactor) * Vin) / (Vo + Vd);
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(n * 100.0) / 100.0);
    // Two-element series tank (no resonant Lm): Rac = 8n²/π²·Rload, Zr = Q·Rac, operate at fr = fsw,
    // Lr = Zr/(2π·fr), Cr = 1/(2π·fr·Zr). Lm made large (10·Lr) so it does not load the resonance.
    const double Rload = Vo / Iout;
    const double Rac = (8.0 * n * n) / (M_PI * M_PI) * Rload;
    const double fr = d.switchingFrequency;   // designed/operated at series resonance
    const double Zr = cfg::get(d.config, "qualityFactor", kQualityFactor) * Rac;
    d.resonantFrequency = fr;
    d.resonantInductance = Zr / (2.0 * M_PI * fr);
    d.resonantCapacitance = 1.0 / (2.0 * M_PI * fr * Zr);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        cfg::get(d.config, "inductanceRatio", kLmRatio) * d.resonantInductance);

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 47e-6;
    return d;
}

json build_src_tas(const SrcDesign& d) {
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
    // Bare seeds (no designRequirements). Body diodes (anti-parallel to a FET) use these as-is — the HS
    // fill DEFERS a requirement-less diode as a FET body diode. REAL switches/rectifiers take a `req`.
    auto mosfet = []() { json j; j["semiconductor"]["mosfet"] = json::object(); return j; };
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };
    auto mosfetReq = [](const json& r) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };
    auto diodeReq  = [](const json& r) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };

    const double n = d.turnsRatio;

    // --- resonant-tank stresses (FHA, evaluated at the nominal point, operated AT series resonance fr) ---
    // The SRC tank is SINUSOIDAL at fr, so every magnetic excitation is a sine (label "sinusoidal",
    // vRms=vPk/√2, vPkPk=2·vPk, offset 0). Half-bridge: the tank sees a ±Vin/2 square (fund. rms √2·Vin/π).
    // Unlike LLC the magnetizing Lm is large (10·Lr) and kept out of the resonance, so the tank current is
    // essentially the real load current (Pin/Vtank1_rms); the small magnetizing component is added too.
    const double fr   = d.resonantFrequency, Tfr = 1.0 / fr;
    const double Pin  = d.outputPower / d.efficiency;
    const double Iout = d.outputPower / d.outputVoltage;
    const double Vtank1Rms = std::sqrt(2.0) * d.inputVoltage / M_PI;     // fund. rms of ±Vin/2 square
    const double IloadRms  = Pin / Vtank1Rms;                            // real (in-phase) tank current
    const double ImagPk    = (d.inputVoltage / 2.0) * (Tfr / 4.0) / d.magnetizingInductance;  // Lm triangle pk
    const double ImagRms   = ImagPk / std::sqrt(3.0);
    const double ItankRms  = std::sqrt(IloadRms * IloadRms + ImagRms * ImagRms);  // primary winding current
    const double ItankPk   = std::sqrt(2.0) * ItankRms;                 // sinusoidal peak
    const double ItankPkPk = 2.0 * ItankPk;
    // Each CT secondary half conducts a rectified half-sine; combined the two deliver Iout. Half-sine
    // present 50%: peak = (π/2)·Iout, rms = (π/4)·Iout per half.
    const double IsecPk   = (M_PI / 2.0) * Iout, IsecRms = (M_PI / 4.0) * Iout, IsecPkPk = IsecPk;
    // Winding voltages (sinusoidal at fr): Lr sees i·Zr (Zr=2π·fr·Lr); transformer primary clamps to the
    // reflected output ±n·Vo; each CT secondary half clamps to ±Vo.
    const double Zr     = 2.0 * M_PI * fr * d.resonantInductance;
    const double vLrPk  = ItankPk * Zr, vLrRms = vLrPk / std::sqrt(2.0), vLrPkPk = 2.0 * vLrPk;
    const double vPriPk = n * d.outputVoltage, vPriRms = vPriPk / std::sqrt(2.0), vPriPkPk = 2.0 * vPriPk;
    const double vSecPk = d.outputVoltage,     vSecRms = vSecPk / std::sqrt(2.0), vSecPkPk = 2.0 * vSecPk;

    // --- semiconductor requirements (sourceable) ---
    // Primary half-bridge MOSFETs Q1/Q2 each block the full bus Vin when off and carry the resonant
    // tank current (peak ItankPk, rms ItankRms — the same primary current that drives the magnetic).
    const double ratedVdsQ = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnQ  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (ItankRms * ItankRms);
    const json reqQ = req::mosfet("mainSwitch", ratedVdsQ, ItankPk, maxRdsOnQ, 125.0);
    // Center-tapped rectifier diodes D1/D2: each non-conducting half is reverse-biased to ~2·Vout, and
    // conducts a rectified half-sine of the reflected tank current (peak IsecPk, avg current Iout).
    // NOT a body diode — an independent output rectifier.
    const double ratedVrD = 2.0 * d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxVfD    = (ratedVrD < 100.0) ? 0.6 : 1.2;
    const json reqD = req::diode(ratedVrD, IsecPk, maxVfD, 0.05 * Tfr);

    json cr; cr["capacitor"] = json::object();
    cr["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.resonantCapacitance;
    cr["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2;

    // Resonant inductor Lr: its OWN single-winding magnetic (carries the full sinusoidal tank current).
    json lr; lr["magnetic"] = json::object();
    lr["inputs"] = req::magnetic_inputs(d.resonantInductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vLrPk, vLrRms, 0.0, vLrPkPk)});

    // Transformer: primary Lpri = Lm, two CT secondary halves (turnsRatios=[n,n]), K=0.999 (MKF).
    // 3 physical windings = turnsRatios.size()+1: primary (tank current) + two CT secondary halves.
    std::vector<std::string> isoSides{"primary", "secondary", "secondary"};
    json t1; t1["magnetic"] = json::object();
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, {n, n}, isoSides,
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vPriPk, vPriRms, 0.0, vPriPkPk),
            req::winding_excitation("sinusoidal", fr, IsecPk, IsecRms, 0.0, IsecPkPk, std::nullopt,
                                    vSecPk, vSecRms, 0.0, vSecPkPk),
            req::winding_excitation("sinusoidal", fr, IsecPk, IsecRms, 0.0, IsecPkPk, std::nullopt,
                                    vSecPk, vSecRms, 0.0, vSecPkPk)});
    { const double kCpl = cfg::get(d.config, "transformerCoupling", 0.999);
      t1["inputs"]["designRequirements"]["leakageInductance"] =
          json::array({ json{{"nominal", (1.0 - kCpl*kCpl) * d.magnetizingInductance}} }); }

    auto busCap = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::get(d.config, "busSplitCap", 10e-6);
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2; return c; };

    json cout; cout["capacitor"] = json::object();
    cout["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cout["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::rectifier_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 3; return c; };
    auto snubR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = cfg::snubber_res(d.config);
        // RC-snubber R dissipates the cap energy each cycle: P = C*V^2*f (V = clamped reverse swing).
        const double vClamp = d.outputVoltage * 3.0;
        dr["powerRating"] = cfg::rectifier_snubber_cap(d.config) * vClamp * vClamp * d.switchingFrequency;
        dr["role"] = "snubber"; return c; };
    // Cap-divider BALANCING resistors — give the bus-split midpoint msplit a DC reference at Vbus/2.
    // Without them msplit (Chi/Clo junction + transformer primary) has no DC path -> singular MNA
    // matrix -> garbage Vout at off-design frequencies and the regulator never converges (abt #54).
    auto balR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = 100000.0;
        dr["powerRating"] = 0.5;
        dr["role"] = "balancing"; return c; };
    // Half-bridge switching-node numerical dV/dt snubber (Csw -> stripped when the FET carries real Coss).
    // The IDEAL switch slews sw_node infinitely fast; at some operating points ngspice can't resolve the
    // edge and aborts ("Timestep too small ... node sw_node"). A small cap to ground mimics Coss so the
    // ideal deck converges; named Csw* so the assembler drops it once a real FET's Coss does the limiting
    // physically, leaving the DATASHEET/MKF_MODEL deck untouched (abt #54).
    auto swSnub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = 100e-12;
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2; return c; };

    json cell; cell["name"] = "src-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")});
    cell["components"] = json::array({
        comp("Q1", mosfetReq(reqQ)), comp("Q2", mosfetReq(reqQ)),
        comp("Dq1", diode()), comp("Dq2", diode()),   // Dq1/Dq2 = Q1/Q2 body diodes -> bare seed (deferred)
        comp("Chi", busCap()), comp("Clo", busCap()),
        comp("Rbal_hi", balR()), comp("Rbal_lo", balR()),   // define bus-split midpoint at DC (abt #54)
        comp("Cr", cr), comp("Lr", lr), comp("T1", t1),
        comp("D1", diodeReq(reqD)), comp("D2", diodeReq(reqD)), comp("Cout", cout),
        comp("Csw1", swSnub()),   // half-bridge sw_node dV/dt snubber (stripped when Coss is real)
        comp("Rsn1", snubR()), comp("Csn1", snubC()), comp("Rsn2", snubR()), comp("Csn2", snubC())});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), pin("Dq1", "cathode"), pin("Chi", "1"),
                          pin("Rbal_hi", "1"), prt("vin")}),
        conn("sw_node",  {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("Dq1", "anode"), pin("Dq2", "cathode"), pin("Cr", "1"),
                          pin("Csw1", "1")}),
        conn("msplit",   {pin("Chi", "2"), pin("Clo", "1"), pin("T1", "primary_end"),
                          pin("Rbal_hi", "2"), pin("Rbal_lo", "1")}),
        conn("cr_mid",   {pin("Cr", "2"), pin("Lr", "primary_start")}),
        conn("pri_top",  {pin("Lr", "primary_end"), pin("T1", "primary_start")}),
        conn("sec_top",  {pin("T1", "secondary1_start"), pin("D1", "anode"),
                          pin("Rsn1", "1"), pin("Csn1", "1")}),
        conn("sec_bot",  {pin("T1", "secondary2_end"), pin("D2", "anode"),
                          pin("Rsn2", "1"), pin("Csn2", "1")}),
        conn("vout_net", {pin("D1", "cathode"), pin("D2", "cathode"),
                          pin("Rsn1", "2"), pin("Csn1", "2"), pin("Rsn2", "2"), pin("Csn2", "2"),
                          pin("Cout", "1"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("Dq2", "anode"), pin("Clo", "2"),
                          pin("Rbal_lo", "2"), pin("Csw1", "2"),
                          pin("T1", "secondary1_end"), pin("T1", "secondary2_start"),
                          pin("Cout", "2"), prt("gnd")}),
        conn("g1_net", {pin("Q1", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), prt("g2")})});

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
        req::control_stage("llcController"),
        pstage("srcCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("srcCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("srcCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("srcCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "srcCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    req::finalize_control_seeds(tas, "seriesResonantConverter");  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
