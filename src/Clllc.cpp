#include "Clllc.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kQualityFactor   = 0.4;   // MKF Clllc default
constexpr double kInductanceRatio = 6.0;   // k = Lm/Lr1 (MKF Clllc inductanceRatioK)
constexpr double kSwitchDuty      = 0.47;  // primary bridge ~50% minus dead time
constexpr double kSenseResistance = 0.01;  // in-line current-sense resistor in the secondary tank [Ω]
constexpr double kSenseHysteresis = 5e-3;  // SR comparator hysteresis on the i·Rsense signal [V]
} // namespace

ClllcDesign design_clllc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    ClllcDesign d{};
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
    d.inputVoltageMin = vinMin; d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;
    double n = Vin / Vo;
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    const double fr = d.switchingFrequency;
    d.resonantFrequency = fr;
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
    //   Lr1 = Lm/k,  Cr1 = 1/((2π·fr)²·Lr1).  The secondary tank (Lr2/Cr2 below) follows, staying symmetric
    // (Lr2·Cr2 = Lr1·Cr1 → same fr). Lr1/Lr2 are their own freshly-designed magnetics and Cr1/Cr2
    // near-nominal (role="resonant") sourced caps, so all track the new values; only the pinned transformer
    // is fixed. (No pin → original Q·Ro sizing stands; the mkf_equivalence ideal deck never pins Lm.)
    if (pinnedLm) {
        const double k = cfg::get(d.config, "inductanceRatio", kInductanceRatio);
        d.primaryResonantInductance = *pinnedLm / k;
        d.primaryResonantCapacitance = 1.0 / (wr * wr * d.primaryResonantInductance);
    }
    d.secondaryResonantInductance = d.primaryResonantInductance / (n * n);
    d.secondaryResonantCapacitance = n * n * d.primaryResonantCapacitance;
    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 100e-6;
    // Power-flow direction (ABT #85). "reverse" makes the Vout side the source and delivers to the Vin side;
    // the symmetric tank/turns-ratio sizing is unchanged, only the deck wiring flips.
    const std::string dir = cfg::get_str(d.config, "powerFlowDirection", "forward");
    if (dir != "forward" && dir != "reverse")
        throw std::invalid_argument("design_clllc: config.powerFlowDirection must be 'forward' or 'reverse', got '" + dir + "'");
    d.reverse = (dir == "reverse");
    return d;
}

json build_clllc_tas(const ClllcDesign& d) {
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
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };
    auto mosfetReq = [](const json& r) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    // RESONANT caps set the tank frequency, so they must be sourced CLOSE to nominal — the default fill
    // treats capacitance as a ripple MINIMUM and oversizes up to 2x (and may pick a lossy electrolytic),
    // which detunes the tank and drops fr below the regulator's bracket (the converter can then never reach
    // target). role=resonant tells the HS fill to pick the NEAREST value with a proper (film) dielectric,
    // not oversize (abt #54, as the LLC Cr already does).
    auto resCap = [&](double c, double vr) { json j = capBrick(c, vr);
        j["inputs"]["designRequirements"]["role"] = "resonant"; return j; };
    auto resBrick = [&](double r) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = r;
        // Conservative requirement floor: a resistor dissipates I^2*R (series) or V^2/R (shunt);
        // the physical value is the smaller. Exact for load resistors (=Pout), safe for sense/divider.
        const double Iout_ = d.outputPower / d.outputVoltage, Vb_ = d.outputVoltage;
        const double i2r_ = Iout_*Iout_*r, v2r_ = Vb_*Vb_/r;
        dr["powerRating"] = (i2r_ < v2r_ ? i2r_ : v2r_);
        return j; };

    const double n = d.turnsRatio;

    // --- resonant-tank stresses (FHA, evaluated at the nominal point, operated AT resonance fr) ---
    // CLLLC is a FULL bridge both sides, so the primary tank sees a ±Vin square (fund. rms 2√2·Vin/π).
    // Tank is SINUSOIDAL → every magnetic excitation is a sine ("sinusoidal", vRms=vPk/√2, vPkPk=2·vPk,
    // offset 0). Primary tank current = real load current (Pin/Vtank1_rms) + reactive magnetizing
    // (Lm sees ±Vin, triangle peak Vin·(T/4)/Lm). Secondary tank/winding carries reflected load ×n
    // (n=Vin/Vo>1). The third "L" (Lr2 secondary resonant inductor) is its own single-winding magnetic.
    const double fr   = d.resonantFrequency, Tfr = 1.0 / fr;
    const double Pin  = d.outputPower / d.efficiency;
    // The DRIVING bus sets the tank fundamental: forward drives Vin, reverse drives Vout (ABT #85). Symmetric
    // tank => identical sizing; only WHICH winding is the driver (real load + magnetizing current) vs the
    // receiver (reflected real load current) flips. ItankX = Vin-side winding + Lr1; IsecX = Vout-side + Lr2.
    const double Vdrive        = d.reverse ? d.outputVoltage : d.inputVoltage;
    const double VtankDriveRms = 2.0 * std::sqrt(2.0) * Vdrive / M_PI;      // fund. rms of the ±Vdrive square
    const double IloadRms      = Pin / VtankDriveRms;                       // real current in the driving winding
    const double LmDrive       = d.reverse ? d.magnetizingInductance / (n * n) : d.magnetizingInductance;  // Lm ref. to driver
    const double ImagPk        = Vdrive * (Tfr / 4.0) / LmDrive;            // Lm triangle pk (driver side)
    const double ImagRms       = ImagPk / std::sqrt(3.0);
    const double IdriveRms     = std::sqrt(IloadRms * IloadRms + ImagRms * ImagRms);  // driving winding total
    const double IrecvRms      = IloadRms * (d.reverse ? 1.0 / n : n);      // reflected to the receiving winding
    const double ItankRms  = d.reverse ? IrecvRms : IdriveRms;             // Vin-side winding (Lr1 + Q1..Q4)
    const double ItankPk   = std::sqrt(2.0) * ItankRms, ItankPkPk = 2.0 * ItankPk;
    const double IsecRms   = d.reverse ? IdriveRms : IrecvRms;             // Vout-side winding (Lr2 + QE..QH)
    const double IsecPk    = std::sqrt(2.0) * IsecRms, IsecPkPk = 2.0 * IsecPk;
    const double Zr1    = 2.0 * M_PI * fr * d.primaryResonantInductance;
    const double Zr2    = 2.0 * M_PI * fr * d.secondaryResonantInductance;
    const double vLr1Pk = ItankPk * Zr1, vLr1Rms = vLr1Pk / std::sqrt(2.0), vLr1PkPk = 2.0 * vLr1Pk;
    const double vLr2Pk = IsecPk * Zr2,  vLr2Rms = vLr2Pk / std::sqrt(2.0), vLr2PkPk = 2.0 * vLr2Pk;
    // Transformer winding voltages come from the embedded analytical excitations below (no inline calc).

    // --- semiconductor requirements (sourceable). All rectification is ACTIVE (synchronous-rectifier
    // MOSFETs both sides); the only diodes (DS1..DS4, DSE..DSH) are FET body diodes -> left bare/deferred.
    // Primary full-bridge Q1..Q4: each blocks the full bus Vin when off, carries the primary tank current
    // (peak ItankPk, rms ItankRms — the same primary current that drives the magnetic).
    const double ratedVdsPri = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnPri  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (ItankRms * ItankRms);
    const json reqPri = req::mosfet("mainSwitch", ratedVdsPri, ItankPk, maxRdsOnPri, 125.0);
    // Secondary SR full-bridge QE..QH: each blocks the output rail Vout when off, carries the secondary
    // (reflected) tank current (peak IsecPk, rms IsecRms).
    const double ratedVdsSec = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnSec  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IsecRms * IsecRms);
    const json reqSec = req::mosfet("mainSwitch", ratedVdsSec, IsecPk, maxRdsOnSec, 125.0);

    // Primary/secondary resonant inductors: each its OWN single-winding magnetic.
    json lr1; lr1["magnetic"] = json::object();
    lr1["inputs"] = req::magnetic_inputs(d.primaryResonantInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vLr1Pk, vLr1Rms, 0.0, vLr1PkPk)});
    json lr2; lr2["magnetic"] = json::object();
    lr2["inputs"] = req::magnetic_inputs(d.secondaryResonantInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, IsecPk, IsecRms, 0.0, IsecPkPk, std::nullopt,
                                    vLr2Pk, vLr2Rms, 0.0, vLr2PkPk)});

    // Transformer: primary Lpri = Lm, single secondary, turnsRatios=[n].
    // 2 physical windings = turnsRatios.size()+1: primary (tank current) + secondary (reflected ×n).
    // Transformer (T1) EMBEDDED excitations from the SINGLE FHA source (the SPICE-validated analytical CLLLC
    // solver): full-bridge drive (bridgeVoltageFactor=1.0) -> primary + single secondary (2 windings),
    // matching. Lr1/Lr2 (primary/secondary resonant inductors) and the switch/diode ratings keep the inline
    // tank FHA (they are not transformer windings). (CLLLC FHA is exact at unity gain — 0.024 NRMSE.)
    namespace AN = Kirchhoff::analytical;
    std::vector<std::string> isoSides{"primary", "secondary"};
    MAS::OperatingPoint aopT1;
    if (!d.reverse) {
        const double IoutT = d.outputPower / d.outputVoltage;
        aopT1 = AN::analytical_clllc(d.inputVoltage, {d.outputVoltage}, {IoutT}, {n}, fr,
            d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
            d.secondaryResonantInductance, d.secondaryResonantCapacitance, 1.0, AN::SrcRectifier::FULL_BRIDGE);
    } else {
        // Reverse (ABT #85): the Vout (LV) side drives, power flows to Vin (HV). Reflect the FHA about the
        // symmetric tank — driving tank = Vout-side tank (Lr2/Cr2), Lm referred to the Vout winding (Lm/n²),
        // turns ratio Vout->Vin = 1/n. analytical_clllc returns [Primary(driver=Vout), Secondary(receiver=
        // Vin)]; reorder to T1's physical winding order [primary(Vin), secondary(Vout)] before capture.
        const double IinT = d.outputPower / d.inputVoltage;
        MAS::OperatingPoint raw = AN::analytical_clllc(d.outputVoltage, {d.inputVoltage}, {IinT}, {1.0 / n}, fr,
            d.magnetizingInductance / (n * n), d.secondaryResonantInductance, d.secondaryResonantCapacitance,
            d.primaryResonantInductance, d.primaryResonantCapacitance, 1.0, AN::SrcRectifier::FULL_BRIDGE);
        auto& ex = raw.get_mutable_excitations_per_winding();
        if (ex.size() >= 2) {
            MAS::OperatingPointExcitation vinW = ex[1];   vinW.set_name(std::string("Primary"));       // Vin = receiver
            MAS::OperatingPointExcitation voutW = ex[0];  voutW.set_name(std::string("Secondary 0"));  // Vout = driver
            aopT1.get_mutable_excitations_per_winding().push_back(vinW);
            aopT1.get_mutable_excitations_per_winding().push_back(voutW);
        } else {
            aopT1 = raw;
        }
    }
    json t1; t1["magnetic"] = json::object();
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, {n}, isoSides,
        std::nullopt, 25.0, AN::excitations_processed(aopT1, "T1"));

    // ───────────────────────── POWER stage ─────────────────────────
    // A small in-line sense resistor in the secondary tank exposes the tank-current sign (senseP/senseM)
    // so the control stage can drive the SR diagonals current-aware.
    json pcell; pcell["name"] = "clllc-power";
    pcell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("g1"), port("g2"),
                                  port("senseP"), port("senseM")});
    pcell["components"] = json::array({
        // primary full bridge + body diodes (DS1..DS4 = Q1..Q4 body diodes -> bare seed, deferred)
        comp("Q1", mosfetReq(reqPri)), comp("Q2", mosfetReq(reqPri)),
        comp("Q3", mosfetReq(reqPri)), comp("Q4", mosfetReq(reqPri)),
        comp("DS1", diode()), comp("DS2", diode()), comp("DS3", diode()), comp("DS4", diode()),
        comp("Cr1", resCap(d.primaryResonantCapacitance, d.inputVoltage * 2)),
        comp("Lr1", lr1), comp("T1", t1),
        comp("Lr2", lr2),
        comp("Cr2", resCap(d.secondaryResonantCapacitance, d.outputVoltage * 2)),
        comp("Rsense", resBrick(cfg::get(d.config, "senseResistance", kSenseResistance))),
        // secondary SR full bridge + body diodes (DSE..DSH = QE..QH body diodes -> bare seed, deferred)
        comp("QE", mosfetReq(reqSec)), comp("QF", mosfetReq(reqSec)),
        comp("QG", mosfetReq(reqSec)), comp("QH", mosfetReq(reqSec)),
        comp("DSE", diode()), comp("DSF", diode()), comp("DSG", diode()), comp("DSH", diode()),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2))});
    pcell["connections"] = json::array({
        // Primary full bridge. (Q1,Q4) on g1, (Q2,Q3) on g2 -> ±Vin.
        conn("vin_net",  {pin("Q1","drain"), pin("Q3","drain"), pin("DS1","cathode"), pin("DS3","cathode"), prt("vin")}),
        conn("node_a",   {pin("Q1","source"), pin("Q2","drain"), pin("DS1","anode"), pin("DS2","cathode"), pin("Cr1","1")}),
        conn("node_b",   {pin("Q3","source"), pin("Q4","drain"), pin("DS3","anode"), pin("DS4","cathode"), pin("T1","primary_end")}),
        conn("c1_mid",   {pin("Cr1","2"), pin("Lr1","primary_start")}),
        conn("pri_top",  {pin("Lr1","primary_end"), pin("T1","primary_start")}),
        // Secondary tank: sec_p -> Lr2 -> Cr2 -> senseP -> Rsense -> node_c ; sec_n -> node_d.
        conn("sec_p",    {pin("T1","secondary1_start"), pin("Lr2","primary_start")}),
        conn("l2_mid",   {pin("Lr2","primary_end"), pin("Cr2","1")}),
        conn("senseP",   {pin("Cr2","2"), pin("Rsense","1"), prt("senseP")}),
        conn("node_c",   {pin("Rsense","2"), pin("QE","source"), pin("QF","drain"),
                          pin("DSE","anode"), pin("DSF","cathode"), prt("senseM")}),
        conn("node_d",   {pin("T1","secondary1_end"), pin("QG","source"), pin("QH","drain"),
                          pin("DSG","anode"), pin("DSH","cathode")}),
        // SR full bridge (diode-emulating SR via the control stage). Body diodes rectify / enable start.
        conn("vout_net", {pin("QE","drain"), pin("QG","drain"), pin("DSE","cathode"), pin("DSG","cathode"),
                          pin("Cout","1"), prt("vout")}),
        conn("gnd_net",  {pin("Q2","source"), pin("Q4","source"), pin("DS2","anode"), pin("DS4","anode"),
                          pin("QF","source"), pin("QH","source"), pin("DSF","anode"), pin("DSH","anode"),
                          pin("Cout","2"), prt("gnd")}),
        // Gates: both bridges run off the SAME two stimulus signals. Primary diagonals (Q1,Q4) on g1 /
        // (Q2,Q3) on g2 give vab=±Vin; the secondary SR diagonals are driven SYNCHRONOUS with them —
        // diagonal A (QE,QH) on g1, diagonal B (QF,QG) on g2 — so each SR FET conducts in lock-step with
        // its own body diode (DSE..DSH still rectify / enable the cold start). This is the same 2-signal
        // lock-step SR drive CLLC uses. (The current-sensed srControl stage below is sourced for the BOM
        // but its behavioural gate law is NOT lowered into the power deck — a control stage is skipped by
        // the assembler — so wiring the SR gates to the separate driveA/driveB control nets left them
        // FLOATING and the converter delivered 0 V; abt #60.)
        conn("g1_net", {pin("Q1","gate"), pin("Q4","gate"), pin("QE","gate"), pin("QH","gate"), prt("g1")}),
        conn("g2_net", {pin("Q2","gate"), pin("Q3","gate"), pin("QF","gate"), pin("QG","gate"), prt("g2")})});

    // ──────────────────── CONTROL stage (swappable) ────────────────────
    // ONE CTAS `controller` component — a current-sensed full-bridge synchronous-rectifier controller —
    // placed as a single part. Its ctas_to_cias lowering expands it to two comparators that read the
    // tank-current sign (across the sense resistor) and gate the two rectifying diagonals. Logical 4-pin
    // interface: senseP/senseM (across Rsense) and gA/gB (the two diagonal gate signals). Swap this whole
    // stage for a different controller without touching the power topology.
    // The CTAS controller's agnostic ideal control law (CTAS controller.behavioral): the lib lowers it.
    auto syncRect = [&](double hyst) { json j; json& b = j["controller"]["behavioral"];
        b["controlScheme"] = "synchronousRectifier"; b["topology"] = "fullBridge"; b["sensing"] = "current";
        b["hysteresis"] = hyst; b["driveHigh"] = 5.0; b["driveLow"] = 0.0; b["threshold"] = 0.0; return j; };
    json ccell; ccell["name"] = "clllc-sr-control";
    ccell["ports"] = json::array({port("senseP"), port("senseM"), port("gA"), port("gB")});
    ccell["components"] = json::array({comp("SR", syncRect(cfg::get(d.config, "senseHysteresis", kSenseHysteresis)))});
    ccell["connections"] = json::array({
        conn("senseP", {pin("SR","senseP"), prt("senseP")}),
        conn("senseM", {pin("SR","senseM"), prt("senseM")}),
        conn("gA", {pin("SR","gA"), prt("gA")}), conn("gB", {pin("SR","gB"), prt("gB")})});

    // Source/delivered rails (ABT #85). Forward: source Vin, deliver Vout. Reverse: source Vout (LV),
    // deliver Vin (HV). The assembler drives the DC source on the "input" external port at inputVoltage and
    // sizes the load on the "output" port from outputs[0]; flipping these + the isc directions below gives
    // reverse power flow.
    const double srcV    = d.reverse ? d.outputVoltage : d.inputVoltage;
    const double srcVmin = d.reverse ? d.outputVoltage : d.inputVoltageMin;
    const double srcVmax = d.reverse ? d.outputVoltage : d.inputVoltageMax;
    const double deliverV = d.reverse ? d.inputVoltage : d.outputVoltage;
    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", srcVmin}, {"nominal", srcV}, {"maximum", srcVmax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = deliverV; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = srcV; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("llcController"),
        pstage("clllcPower", "switchingCell", pcell, bind("vin", "dcBus"), bind("vout", "dcOutput")),
        pstage("srControl", "control", ccell, bind("senseP", "sense"), bind("gA", "drive"))});
    // Node names stay tied to the physical rails (Vin = HV port, Vout = LV port); only the source/load
    // DIRECTION flips for reverse power flow (ABT #85). The sense wires (power->control) are unaffected.
    json powerRails = d.reverse
        ? json::array({
            isc("Vout", "externalPort", "input",  {sp("clllcPower", "vout")}),   // LV rail sources
            isc("GND",  "externalPort", "input",  {sp("clllcPower", "gnd")}),
            isc("Vin",  "externalPort", "output", {sp("clllcPower", "vin")})})    // HV rail delivered
        : json::array({
            isc("Vin",  "externalPort", "input",  {sp("clllcPower", "vin")}),
            isc("GND",  "externalPort", "input",  {sp("clllcPower", "gnd")}),
            isc("Vout", "externalPort", "output", {sp("clllcPower", "vout")})});
    // tank-current sense: power -> control (the SR controller IC is sourced for the BOM and reads the sense
    // resistor; the SR power gates are driven in lock-step off g1/g2 in the power cell, so no driveA/driveB
    // gate nets are wired — they would float, abt #60).
    powerRails.push_back(isc("senseP", "wire", "", {sp("clllcPower", "senseP"), sp("srControl", "senseP")}));
    powerRails.push_back(isc("senseM", "wire", "", {sp("clllcPower", "senseM"), sp("srControl", "senseM")}));
    tas["topology"]["interStageConnections"] = std::move(powerRails);

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // ONLY the primary bridge is open-loop driven; the SR is closed-loop via the control stage.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "clllcPower"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    // Precharge the DELIVERED bus so the active SR can start; forward precharges Vout, reverse Vin (ABT #85).
    { json ic; ic["node"] = d.reverse ? "Vin" : "Vout"; ic["voltage"] = deliverV;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::CLLLC_RESONANT_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
