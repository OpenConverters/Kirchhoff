#include "Cllc.hpp"
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
constexpr double kQualityFactor   = 0.3;   // MKF Cllc default (Infineon AN: 0.2–0.4)
constexpr double kInductanceRatio = 4.45;  // k = Lm/Lr1 (MKF defaultInductanceRatio)
constexpr double kSwitchDuty      = 0.47;  // ~50% minus dead time
constexpr double kGainHeadroom    = 1.08;  // size n for M=1 at fr -> 1.08·Vo, so the nominal operating
                                           // point sits just ABOVE fr (efficient) not at the M=1 peak
} // namespace

CllcDesign design_cllc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    CllcDesign d{};
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

    // n = Vin_nom/(headroom·Vout) (full bridge both sides). fr = fsw. With zero headroom (n=Vin/Vo) the
    // nominal point sits exactly at the fr gain PEAK (M=1), so any real loss (FET Rds, magnetic DCR/core,
    // rectifier drop) sags Vout below target and the regulator must dive FAR below resonance to recover —
    // high circulating current, ~50% efficiency (abt #62). Sizing ~8% gain headroom puts the nominal point
    // just ABOVE fr on the efficient monotonic edge: the regulator trims frequency DOWN toward fr to cover
    // losses while staying near resonance. The realized headroom n flows into the pinned turnsRatio below.
    double n = Vin / (cfg::get(d.config, "gainHeadroom", kGainHeadroom) * Vo);
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    const double fr = d.switchingFrequency;
    d.resonantFrequency = fr;

    // Infineon FHA: Ro = 8n²/π²·Rload, Cr1 = 1/(2π·Q·fr·Ro), Lr1 = 1/((2π·fr)²·Cr1), Lm = k·Lr1.
    // Symmetric tank (a=b=1): Lr2 = Lr1/n², Cr2 = n²·Cr1.  (MKF Cllc::calculate_resonant_parameters)
    const double Rload = Vo * Vo / d.outputPower;
    const double Ro = (8.0 * n * n / (M_PI * M_PI)) * Rload;
    const double wr = 2.0 * M_PI * fr;
    d.primaryResonantCapacitance = 1.0 / (2.0 * M_PI * cfg::get(d.config, "qualityFactor", kQualityFactor) * fr * Ro);
    d.primaryResonantInductance = 1.0 / (wr * wr * d.primaryResonantCapacitance);
    const auto pinnedLm = req::provided_inductance(dr);
    d.magnetizingInductance = pinnedLm.value_or(
        cfg::get(d.config, "inductanceRatio", kInductanceRatio) * d.primaryResonantInductance);
    // della-Pollock resonant tank CO-DESIGN: the closed-loop realize pins Lm to the REALIZED magnetizing
    // inductance of the chosen transformer core (sized for a saturation margin, so typically larger than
    // k·Lr1). Once Lm is fixed it no longer equals k·Lr1, so the tank is DETUNED (k=Lm/Lr1 drifts, the gain
    // curve shifts, the converter is pushed off resonance and can't reach target Vout). Re-size the PRIMARY
    // tank around the pinned Lm to PRESERVE the design ratio k AND keep Lr1–Cr1 resonant at fr:
    //   Lr1 = Lm/k,  Cr1 = 1/((2π·fr)²·Lr1).  The secondary tank (Lr2/Cr2 below) follows from the new
    // Lr1/Cr1, staying symmetric (Lr2·Cr2 = Lr1·Cr1 → same fr). Lr1/Lr2 are their own freshly-designed
    // magnetics and Cr1/Cr2 near-nominal (role="resonant") sourced caps, so all track the new values; only
    // the pinned transformer is fixed. (No pin → original Q·Ro sizing stands; the mkf_equivalence ideal
    // deck never pins Lm and is unchanged.)
    if (pinnedLm) {
        const double k = cfg::get(d.config, "inductanceRatio", kInductanceRatio);
        d.primaryResonantInductance = *pinnedLm / k;
        d.primaryResonantCapacitance = 1.0 / (wr * wr * d.primaryResonantInductance);
    }
    d.secondaryResonantInductance = d.primaryResonantInductance / (n * n);
    d.secondaryResonantCapacitance = n * n * d.primaryResonantCapacitance;

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 10e-6;    // matches MKF CLLC (Cout=10u)
    return d;
}

json build_cllc_tas(const CllcDesign& d) {
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
    // fill DEFERS a requirement-less diode as a FET body diode. REAL switches take a `req`.
    auto mosfet = []() { json j; j["semiconductor"]["mosfet"] = json::object(); return j; };
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };
    auto mosfetReq = [](const json& r) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };

    const double n = d.turnsRatio;

    auto capBrick = [&](double c, double vrated) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vrated; return j; };

    // --- resonant-tank stresses (FHA, evaluated at the nominal point, operated AT resonance fr) ---
    // CLLC is a FULL bridge both sides, so the primary tank sees a ±Vin square (fund. rms 2√2·Vin/π).
    // Tank is SINUSOIDAL → every magnetic excitation is a sine ("sinusoidal", vRms=vPk/√2, vPkPk=2·vPk,
    // offset 0). Primary tank current = real load current (Pin/Vtank1_rms) + reactive magnetizing
    // (Lm sees ±Vin, triangle peak Vin·(T/4)/Lm). The secondary tank/winding carries the reflected real
    // load current ×n (n=Vin/Vo>1 here).
    const double fr   = d.resonantFrequency, Tfr = 1.0 / fr;
    const double Pin  = d.outputPower / d.efficiency;
    const double Vtank1Rms = 2.0 * std::sqrt(2.0) * d.inputVoltage / M_PI;  // fund. rms of ±Vin square
    const double IloadRms  = Pin / Vtank1Rms;                               // real primary tank current
    const double ImagPk    = d.inputVoltage * (Tfr / 4.0) / d.magnetizingInductance;  // Lm triangle pk
    const double ImagRms   = ImagPk / std::sqrt(3.0);
    const double ItankRms  = std::sqrt(IloadRms * IloadRms + ImagRms * ImagRms);  // primary winding current
    const double ItankPk   = std::sqrt(2.0) * ItankRms, ItankPkPk = 2.0 * ItankPk;
    const double IsecRms   = IloadRms * n;                                  // reflected to the secondary
    const double IsecPk    = std::sqrt(2.0) * IsecRms, IsecPkPk = 2.0 * IsecPk;
    // Winding voltages (sinusoidal at fr): each Lr sees i·Z (Z=2π·fr·L); transformer primary clamps to
    // the reflected output ±n·Vo (≈±Vin), the single secondary to ±Vo.
    const double Zr1    = 2.0 * M_PI * fr * d.primaryResonantInductance;
    const double Zr2    = 2.0 * M_PI * fr * d.secondaryResonantInductance;
    const double vLr1Pk = ItankPk * Zr1, vLr1Rms = vLr1Pk / std::sqrt(2.0), vLr1PkPk = 2.0 * vLr1Pk;
    const double vLr2Pk = IsecPk * Zr2,  vLr2Rms = vLr2Pk / std::sqrt(2.0), vLr2PkPk = 2.0 * vLr2Pk;
    const double vPriPk = n * d.outputVoltage, vPriRms = vPriPk / std::sqrt(2.0), vPriPkPk = 2.0 * vPriPk;
    const double vSecPk = d.outputVoltage,     vSecRms = vSecPk / std::sqrt(2.0), vSecPkPk = 2.0 * vSecPk;

    // --- semiconductor requirements (sourceable). All rectification is ACTIVE (synchronous-rectifier
    // MOSFETs both sides); the only diodes (DS1..DS4, DSa..DSd) are FET body diodes -> left bare/deferred.
    // Primary full-bridge Q1..Q4: each blocks the full bus Vin when off, carries the primary tank current
    // (peak ItankPk, rms ItankRms — the same primary current that drives the magnetic).
    const double ratedVdsPri = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnPri  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (ItankRms * ItankRms);
    const json reqPri = req::mosfet("mainSwitch", ratedVdsPri, ItankPk, maxRdsOnPri, 125.0);
    // Secondary SR full-bridge Qa..Qd: each blocks the output rail Vout when off, carries the secondary
    // (reflected) tank current (peak IsecPk, rms IsecRms).
    const double ratedVdsSec = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnSec  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IsecRms * IsecRms);
    const json reqSec = req::mosfet("mainSwitch", ratedVdsSec, IsecPk, maxRdsOnSec, 125.0);

    json cr1 = capBrick(d.primaryResonantCapacitance, d.inputVoltage * 2);
    // RESONANT caps set the tank frequency, so they must be sourced CLOSE to nominal — the default fill
    // treats capacitance as a ripple MINIMUM and oversizes up to 2x (and may pick a lossy electrolytic),
    // which detunes the CLLC tank (a 1.9nF Cr1 sourced at 3.3nF dropped fr 126→97kHz, below the regulator's
    // bracket, so the converter could never boost to target). role=resonant tells the HS fill to pick the
    // NEAREST value with a proper (film) dielectric, not oversize (abt #54, as the LLC Cr already does).
    cr1["inputs"]["designRequirements"]["role"] = "resonant";
    // Primary resonant inductor Lr1: its OWN single-winding magnetic (full primary tank current).
    json lr1; lr1["magnetic"] = json::object();
    lr1["inputs"] = req::magnetic_inputs(d.primaryResonantInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vLr1Pk, vLr1Rms, 0.0, vLr1PkPk)});
    // Secondary resonant inductor Lr2: its OWN single-winding magnetic (reflected secondary tank current).
    json lr2; lr2["magnetic"] = json::object();
    lr2["inputs"] = req::magnetic_inputs(d.secondaryResonantInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, IsecPk, IsecRms, 0.0, IsecPkPk, std::nullopt,
                                    vLr2Pk, vLr2Rms, 0.0, vLr2PkPk)});
    json cr2 = capBrick(d.secondaryResonantCapacitance, d.outputVoltage * 2);
    cr2["inputs"]["designRequirements"]["role"] = "resonant";   // secondary tank cap — source near nominal (abt #54)
    json cout = capBrick(d.outputCapacitance, d.outputVoltage * 2);

    // Transformer: primary Lpri = Lm, single secondary, turnsRatios=[n], K=0.9999.
    // 2 physical windings = turnsRatios.size()+1: primary (tank current) + secondary (reflected ×n).
    std::vector<std::string> isoSides{"primary", "secondary"};
    json t1; t1["magnetic"] = json::object();
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, {n}, isoSides,
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vPriPk, vPriRms, 0.0, vPriPkPk),
            req::winding_excitation("sinusoidal", fr, IsecPk, IsecRms, 0.0, IsecPkPk, std::nullopt,
                                    vSecPk, vSecRms, 0.0, vSecPkPk)});

    json cell; cell["name"] = "cllc-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("g1"), port("g2")});
    cell["components"] = json::array({
        // primary full bridge + body diodes (DS1..DS4 = Q1..Q4 body diodes -> bare seed, deferred)
        comp("Q1", mosfetReq(reqPri)), comp("Q2", mosfetReq(reqPri)),
        comp("Q3", mosfetReq(reqPri)), comp("Q4", mosfetReq(reqPri)),
        comp("DS1", diode()), comp("DS2", diode()), comp("DS3", diode()), comp("DS4", diode()),
        // primary tank + transformer + secondary tank
        comp("Cr1", cr1), comp("Lr1", lr1), comp("T1", t1), comp("Lr2", lr2), comp("Cr2", cr2),
        // secondary active synchronous rectifier (4 switches) + body diodes (DSa..DSd = Qa..Qd body
        // diodes -> bare seed, deferred; the rectifiers are the SR MOSFETs themselves)
        comp("Qa", mosfetReq(reqSec)), comp("Qb", mosfetReq(reqSec)),
        comp("Qc", mosfetReq(reqSec)), comp("Qd", mosfetReq(reqSec)),
        comp("DSa", diode()), comp("DSb", diode()), comp("DSc", diode()), comp("DSd", diode()),
        comp("Cout", cout)});
    cell["connections"] = json::array({
        // ── Primary full bridge. Diagonal pairs (Q1,Q4) on g1, (Q2,Q3) on g2 -> vab=±Vin.
        conn("vin_net",  {pin("Q1", "drain"), pin("Q3", "drain"),
                          pin("DS1", "cathode"), pin("DS3", "cathode"), prt("vin")}),
        conn("node_a",   {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("DS1", "anode"), pin("DS2", "cathode"), pin("Cr1", "1")}),
        conn("node_b",   {pin("Q3", "source"), pin("Q4", "drain"),
                          pin("DS3", "anode"), pin("DS4", "cathode"), pin("T1", "primary_end")}),
        // Primary series tank: node_a -> Cr1 -> Lr1 -> Lpri(=Lm) -> node_b.
        conn("c1_mid",   {pin("Cr1", "2"), pin("Lr1", "primary_start")}),
        conn("pri_top",  {pin("Lr1", "primary_end"), pin("T1", "primary_start")}),
        // Secondary series tank: sec_p -> Lr2 -> Cr2 -> node_c ; sec_n -> node_d.
        conn("sec_p",    {pin("T1", "secondary1_start"), pin("Lr2", "primary_start")}),
        conn("l2_mid",   {pin("Lr2", "primary_end"), pin("Cr2", "1")}),
        conn("node_c",   {pin("Cr2", "2"), pin("Qa", "source"), pin("Qb", "drain"),
                          pin("DSa", "anode"), pin("DSb", "cathode")}),
        conn("node_d",   {pin("T1", "secondary1_end"), pin("Qc", "source"), pin("Qd", "drain"),
                          pin("DSc", "anode"), pin("DSd", "cathode")}),
        // ── Secondary active bridge. Diagonal pairs (Qa,Qd) on g1, (Qb,Qc) on g2 — synchronous with
        // the primary (forward power flow). Body diodes DSa..DSd form a full-bridge rectifier that lets
        // the converter start once the output is precharged (simulation.initialConditions).
        conn("vout_net", {pin("Qa", "drain"), pin("Qc", "drain"),
                          pin("DSa", "cathode"), pin("DSc", "cathode"), pin("Cout", "1"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("Q4", "source"),
                          pin("DS2", "anode"), pin("DS4", "anode"),
                          pin("Qb", "source"), pin("Qd", "source"),
                          pin("DSb", "anode"), pin("DSd", "anode"), pin("Cout", "2"), prt("gnd")}),
        conn("g1_net", {pin("Q1", "gate"), pin("Q4", "gate"), pin("Qa", "gate"), pin("Qd", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), pin("Q3", "gate"), pin("Qb", "gate"), pin("Qc", "gate"), prt("g2")})});

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
        pstage("cllcCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("cllcCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("cllcCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("cllcCell", "vout")})});

    json an; an["type"] = "transient"; an["stopTime"] = 0.004; an["maximumTimeStep"] = 5e-8;
    tas["simulation"]["analyses"] = json::array({an});
    // Both bridges are square waves: g1 (Q1,Q4,Qa,Qd) phase 0, g2 (Q2,Q3,Qb,Qc) phase 180. The
    // secondary is gated synchronously with the primary (forward power flow).
    auto stim = [&](const char* sw, const char* sig, double phaseDeg) {
        json st; st["stage"] = "cllcCell"; st["component"] = sw; st["signal"] = sig;
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    // Only ONE stimulus per shared gate node: Q1 drives the g1 port (shared by Q1/Q4/Qa/Qd), Q2 drives
    // g2 (shared by Q2/Q3/Qb/Qc). Emitting one per switch would put 4 voltage sources on one node
    // (singular). The four switches on each gate are driven together — exactly the 2-signal CLLC drive.
    tas["simulation"]["stimulus"] = json::array({
        stim("Q1", "gate", 0.0), stim("Q2", "gate", 180.0)});
    // Precharge the output to its target so the active synchronous rectifier can start and the deck runs
    // with use-initial-conditions (skipping the resonant tank's singular DC operating point).
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::CLLC_RESONANT_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
