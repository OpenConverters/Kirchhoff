#include "Fsbb.hpp"
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
constexpr double kRippleRatio = 0.4;   // inductor current ripple (MKF 4SBB K)
constexpr double kDeadFrac    = 0.01;  // 100ns per-leg dead time at 100kHz
} // namespace

FsbbDesign design_fsbb(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    FsbbDesign d{};
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
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", kDeadFrac);

    // --- transition sub-mode + power-flow direction + interleaving (ABT #94) ---
    d.transitionMode = cfg::get_str(d.config, "transitionMode", "splitPwm");   // MKF default: splitPwm
    if (d.transitionMode != "splitPwm" && d.transitionMode != "simultaneous")
        throw std::invalid_argument("design_fsbb: config.transitionMode must be 'splitPwm' or 'simultaneous', got '"
                                    + d.transitionMode + "'");
    d.splitRatio = cfg::get(d.config, "fsbbSplitRatio", 0.5);
    if (!(d.splitRatio > 0.0 && d.splitRatio <= 1.0))
        throw std::invalid_argument("design_fsbb: config.fsbbSplitRatio must be in (0, 1]");
    const std::string dir = cfg::get_str(d.config, "powerFlowDirection", "forward");
    if (dir != "forward" && dir != "reverse")
        throw std::invalid_argument("design_fsbb: config.powerFlowDirection must be 'forward' or 'reverse', got '"
                                    + dir + "'");
    d.reverse = (dir == "reverse");
    // Interleaved multi-phase (ABT #94): config.phaseCount = N builds N phase-shifted 4-switch buck-boost
    // legs sharing the input/output bus (interleaved by 360/N degrees). Each leg carries Iout/N, so its
    // inductor is sized for the per-phase current below. N=1 (default) is the ordinary single-phase deck.
    const double phaseCountRaw = cfg::get(d.config, "phaseCount", 1.0);
    d.phaseCount = static_cast<int>(std::llround(phaseCountRaw));
    if (static_cast<double>(d.phaseCount) != phaseCountRaw)
        throw std::invalid_argument("design_fsbb: config.phaseCount must be a whole number, got "
                                    + std::to_string(phaseCountRaw));
    if (d.phaseCount < 1 || d.phaseCount > 6)   // 1 (single-phase) .. 6 interleaved legs
        throw std::invalid_argument("design_fsbb: config.phaseCount must be in [1, 6], got "
                                    + std::to_string(d.phaseCount));

    // Source / delivered rails. Forward: source Vin, deliver Vo. Reverse (bidirectional, ABT #94): the Vout
    // rail sources and the Vin rail receives — the symmetric synchronous H-bridge conducts both ways, so
    // reverse is the same converter with the two legs' roles swapped. Region/duty are sized on (Vsrc,Vdel).
    const double Vsrc = d.reverse ? Vo : Vin;
    const double Vdel = d.reverse ? Vin : Vo;
    const double Io = d.outputPower / Vdel;     // current into the delivered (load) rail
    // Per-phase inductor current: each of the N interleaved legs carries Iout/N, so its inductor is sized
    // for the per-phase current (⇒ N× the single-phase L). N=1 ⇒ IoPhase==Io (byte-identical).
    const double IoPhase = Io / static_cast<double>(d.phaseCount);
    // Simultaneous buck-boost gain M = D/(1-D) = Vdel/Vsrc  =>  D = Vdel/(Vsrc+Vdel).
    d.dutyCycle = Vdel / (Vsrc + Vdel);
    // Operating-region classification at the NOMINAL point (MKF FourSwitchBuckBoost::Region). A real 4SBB
    // controller runs only the buck leg when Vsrc comfortably exceeds Vdel (BUCK), only the boost leg when
    // Vdel exceeds Vsrc (BOOST), and commutes all four switches in the narrow transition band (BUCK_BOOST).
    // The KH deck previously ran the 4-switch SIMULTANEOUS scheme for EVERY point, giving buck/boost-dominant
    // designs the wrong (lossier) gate drive. `fsbbTransitionBand` (default 0.10) is the ±fraction of Vdel
    // around Vsrc within which we stay in the all-switching buck-boost region.
    const double band = cfg::get(d.config, "fsbbTransitionBand", 0.10);
    if (Vsrc > Vdel * (1.0 + band)) {
        d.region = "buck";
        d.regionDuty = Vdel / (Vsrc * d.efficiency);
    } else if (Vsrc < Vdel * (1.0 - band)) {
        d.region = "boost";
        d.regionDuty = 1.0 - (Vsrc * d.efficiency) / Vdel;
    } else {
        d.region = "buckBoost";
        d.regionDuty = d.dutyCycle;
    }
    // Split-PWM transition-band duties (only used in the buckBoost region with transitionMode="splitPwm").
    // t1 = boost-leg-LS charge duty = κ·D; t2 = buck-leg-HS duty = Vdel·(1−t1)/Vsrc (volt-second balanced so
    // the open-loop deck lands on the delivered target). At κ=1 ⇒ t1=t2=D ⇒ collapses to SIMULTANEOUS.
    d.splitBoostDuty = d.splitRatio * d.dutyCycle;
    d.splitBuckDuty  = Vdel * (1.0 - d.splitBoostDuty) / Vsrc;

    // Worst-case inductor: max(L_buck @ Vin_max, L_boost @ Vin_min). Each region's formula is only
    // valid on its side of Vo (returns 0 otherwise). (MKF compute_worst_case_inductance.)
    const double Lbuck  = (vinMax > Vo) ? Vo * (vinMax - Vo) / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * IoPhase * Fs * vinMax) : 0.0;
    const double Lboost = (vinMin < Vo) ? (vinMin * vinMin) * (Vo - vinMin)
                                          / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * IoPhase * Fs * Vo * Vo) : 0.0;
    double L = std::max(Lbuck, Lboost);
    if (L <= 0.0) {
        // Exact-unity / single-point spec (Vin_min == Vin_max == Vo, e.g. a 12->12 V request with no input
        // range): neither the buck (vinMax>Vo) nor the boost (vinMin<Vo) region's formula fires, leaving
        // L = 0 — a broken deck (the inductor vanishes, output collapses to ~Vin·D ≈ 3.3 V). Size it
        // directly for the 4-switch SIMULTANEOUS mode the design always runs: the charge phase applies Vin
        // across L for D·T, with ΔIL targeted at kRippleRatio (config "inductorRippleRatio") of the buck-boost current Io/(1-D):
        //     L = Vin·D·(1-D) / (kRippleRatio·Io·Fs).   (ABT #26)
        const double D = d.dutyCycle;
        L = Vin * D * (1.0 - D) / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * IoPhase * Fs);
    }
    d.inductance = req::provided_inductance(dr).value_or(
        L);

    d.loadResistance = Vdel * Vdel / d.outputPower;   // load sits on the delivered rail (Vo fwd / Vin rev)
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

static json build_fsbb_interleaved_tas(const FsbbDesign& d);   // ABT #94: N phase-shifted legs

json build_fsbb_tas(const FsbbDesign& d) {
    // Interleaved multi-phase (ABT #94): N>1 phase-shifted 4-switch buck-boost legs share the input/output
    // bus. Routed to the dedicated builder; the single-phase (N=1) path below is left byte-identical.
    if (d.phaseCount > 1) return build_fsbb_interleaved_tas(d);
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

    // Single inductor L (single-winding magnetic) sourced from the SINGLE FHA source. The transition-band
    // waveform depends on the sub-mode (ABT #94): SPLIT_PWM (MKF default) phase-shifts the two legs for a
    // strictly lower inductor ripple; "simultaneous" runs all four switches together (charge +Vin for D·T,
    // discharge −Vo for (1−D)·T, D = Vo/(Vin+Vo)). Both are regular at Vo==Vin. BUCK/BOOST regions embed the
    // per-region (buck-/boost-like) waveform. Ratings pull from the region's worst corner (buck: Vin_max =
    // max ripple & block voltage; boost/bb: Vin_min = max duty & inductor current).
    const double fsw = d.switchingFrequency, L_H = d.inductance;
    const double Vo = d.outputVoltage;
    namespace AN = Kirchhoff::analytical;
    // Source / delivered rails (ABT #94). Forward: source Vin, deliver Vo. Reverse (bidirectional): the Vout
    // rail sources and delivers to Vin — the symmetric synchronous H-bridge with the two legs' roles swapped.
    const double srcNom  = d.reverse ? Vo : d.inputVoltage;
    const double delV    = d.reverse ? d.inputVoltage : Vo;
    const double Idel    = d.outputPower / delV;              // current into the delivered (load) rail
    // Reverse source (the Vout rail) is a single regulated value with no tolerance corner, so the reverse
    // ratings pull from the nominal point; forward keeps the per-region worst input corner.
    const double srcWorst = d.reverse ? srcNom
                                      : ((d.region == "buck") ? d.inputVoltageMax : d.inputVoltageMin);
    const bool splitBand = (d.region == "buckBoost" && d.transitionMode == "splitPwm");
    const AN::FsbbMode kMode = (d.region == "buckBoost")
        ? (splitBand ? AN::FsbbMode::SPLIT_PWM : AN::FsbbMode::SIMULTANEOUS)
        : AN::FsbbMode::BUCK_BOOST_AUTO;
    const double maxDuty = cfg::get(d.config, "maximumDutyCycle", 0.95);   // ABT #95: configurable (default matches solver)
    const MAS::OperatingPoint aopWorst = AN::analytical_fsbb(srcWorst, delV, Idel, fsw, L_H, d.efficiency, kMode, d.splitRatio, maxDuty);
    const MAS::OperatingPoint aopNom   = AN::analytical_fsbb(srcNom,   delV, Idel, fsw, L_H, d.efficiency, kMode, d.splitRatio, maxDuty);
    const double IpkL  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsL = AN::winding_current(aopWorst, 0, "rms");
    json lind; lind["magnetic"] = json::object();
    lind["inputs"] = req::magnetic_inputs(d.inductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, AN::excitations_processed(aopNom, "L"));

    // --- semiconductor stresses: all four H-bridge switches share one rating ---
    // Buck leg Q1/Q2 nodes swing 0..Vin_max; boost leg Q3/Q4 nodes swing 0..Vout. Worst-case block
    // voltage = max(Vin_max, Vout). Each switch carries the inductor current (IpkL / IrmsL).
    const double Vblock   = std::max(d.inputVoltageMax, Vo);
    const double ratedVds = Vblock / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsL * IrmsL);

    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        cfg::mark_numerical_aid(c);   // dV/dt convergence aid — tagged for the real-fidelity strip (ABT #96)
        return c; };

    // REAL EMI/ring-damper RC snubber across each hard-switched node (sw1, sw2) to ground. The 4SBB is a
    // HARD-switched H-bridge (no ZVS): each leg's node rings at turn-off against the switch Coss + layout
    // inductance, so a real series R–C damper is a genuine board part here (unlike the sim-only Csw* node
    // caps). Sized from the ENERGY BUDGET (cfg::snubber_cap = eps·P/(Vblock²·fsw)) + cfg::snubber_res so it
    // stores « throughput and does not move the operating point. REAL refdes (Crc*/Rrc*, role "snubber")
    // -> rendered + sourced, and NOT matched by the numerical-aid strip (Csn*/Rsn*/Csw*).
    const double rcSnubC = cfg::snubber_cap(d.config, d.outputPower, Vblock, fsw);
    const double rcSnubR = cfg::snubber_res(d.config);
    const auto rcSnub = req::snubber(rcSnubC, rcSnubR, Vblock, fsw);
    const json& rcCap = rcSnub.first;
    const json& rcRes = rcSnub.second;

    // H-bridge buck-boost cell. Buck leg Q1(vin->sw1)/Q2(sw1->gnd); inductor sw1->sw2; boost leg
    // Q3(vout->sw2)/Q4(sw2->gnd). Body diodes anti-parallel to each switch (Q3 oriented drain=vout so
    // its body diode sw2->vout is the boost freewheel path). Snubber caps at sw1/sw2 for dead-time dV/dt.
    json cell; cell["name"] = "fsbb-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"),
                                 port("g1"), port("g2"), port("g3"), port("g4")});
    cell["components"] = json::array({
        comp("Q1", mosfet(req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0))),
        comp("Q2", mosfet(req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0))),
        comp("Q3", mosfet(req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0))),
        comp("Q4", mosfet(req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0))),
        // D1..D4 are anti-parallel BODY diodes across Q1..Q4 -> requirement-less (carried by the FET).
        comp("D1", diode()),  comp("D2", diode()),  comp("D3", diode()),  comp("D4", diode()),
        comp("L", lind), comp("Csw1", snub()), comp("Csw2", snub()),
        // REAL series-RC EMI snubbers (sourced, rendered) across each switching node.
        comp("Crc_sw1", rcCap), comp("Rrc_sw1", rcRes),
        comp("Crc_sw2", rcCap), comp("Rrc_sw2", rcRes)});
    cell["connections"] = json::array({
        conn("vin_net",  {pin("Q1", "drain"), pin("D1", "cathode"), prt("vin")}),
        conn("sw1_net",  {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("D1", "anode"), pin("D2", "cathode"),
                          pin("L", "primary_start"), pin("Csw1", "1"), pin("Crc_sw1", "1")}),
        conn("sw2_net",  {pin("Q4", "drain"), pin("Q3", "source"),
                          pin("D4", "cathode"), pin("D3", "anode"),
                          pin("L", "primary_end"), pin("Csw2", "1"), pin("Crc_sw2", "1")}),
        // series-RC snubber midpoints: cap top plate at the sw node, damping R to ground.
        conn("rc_sw1_mid", {pin("Crc_sw1", "2"), pin("Rrc_sw1", "1")}),
        conn("rc_sw2_mid", {pin("Crc_sw2", "2"), pin("Rrc_sw2", "1")}),
        // Q3 high-side to the output: drain=vout (body diode D3 sw2->vout = boost freewheel).
        conn("vout_net", {pin("Q3", "drain"), pin("D3", "cathode"), prt("vout")}),
        conn("gnd_net",  {pin("Q2", "source"), pin("Q4", "source"),
                          pin("D2", "anode"), pin("D4", "anode"),
                          pin("Csw1", "2"), pin("Csw2", "2"),
                          pin("Rrc_sw1", "2"), pin("Rrc_sw2", "2"), prt("gnd")}),
        conn("g1_net", {pin("Q1", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), prt("g2")}),
        conn("g3_net", {pin("Q3", "gate"), prt("g3")}),
        conn("g4_net", {pin("Q4", "gate"), prt("g4")})});

    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    // Source / delivered rails at the deck level (ABT #94). Forward: source Vin (with its tolerance), deliver
    // Vo. Reverse: the Vout rail sources (a single regulated value) and the Vin rail is the delivered output;
    // the assembler drives a DC source on the "input" external port and sizes the load on the "output" port,
    // so flipping the external-port directions realises reverse power flow (same mechanism as the Ćuk/CLLC
    // bidirectional). The output filter cap rides the delivered rail either way.
    const double srcNomV = d.reverse ? Vo : d.inputVoltage;
    const double srcMinV = d.reverse ? Vo : d.inputVoltageMin;
    const double srcMaxV = d.reverse ? Vo : d.inputVoltageMax;
    const double deliverV = d.reverse ? d.inputVoltage : Vo;
    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", srcMinV}, {"nominal", srcNomV}, {"maximum", srcMaxV}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = deliverV; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = srcNomV; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("fsbbCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    // Node names stay tied to the physical rails (Vin, Vout); only the source/load DIRECTION flips (ABT #94).
    // Reverse: the Vout rail sources and the output filter cap moves onto the delivered Vin rail.
    tas["topology"]["interStageConnections"] = d.reverse
        ? json::array({
            isc("Vout", "externalPort", "input",  {sp("fsbbCell", "vout")}),                   // Vout sources
            isc("GND",  "externalPort", "input",  {sp("fsbbCell", "gnd"), sp("filter", "rtn")}),
            isc("Vin",  "externalPort", "output", {sp("fsbbCell", "vin"), sp("filter", "in")})})  // Vin delivered
        : json::array({
            isc("Vin",  "externalPort", "input",  {sp("fsbbCell", "vin")}),
            isc("GND",  "externalPort", "input",  {sp("fsbbCell", "gnd"), sp("filter", "rtn")}),
            isc("Vout", "externalPort", "output", {sp("fsbbCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Region-dependent gate drive (MKF FourSwitchBuckBoost). Each leg has a dead band on its complementary
    // pair; body diodes carry the inductor current across the dead time. The four switches are addressed
    // through leg-role ALIASES so forward and reverse share one code path (ABT #94): the "buck leg" is the
    // SOURCE-side half-bridge, the "boost leg" the DELIVERED-side half-bridge. Forward source = Vin leg
    // (Q1/Q2), delivered = Vout leg (Q3/Q4). Reverse source = Vout leg (Q3/Q4), delivered = Vin leg (Q1/Q2),
    // so the roles swap. (bkHS drain=source rail, bkLS→gnd; bsHS drain=delivered rail, bsLS→gnd.)
    //   BUCK  (Vsrc>Vdel): only the buck (source) leg switches (bkHS PWM @ D=Vdel/Vsrc, bkLS complementary);
    //                      the boost leg is static — bsHS ON connects its node→delivered rail, bsLS OFF.
    //   BOOST (Vdel>Vsrc): only the boost (delivered) leg switches (bsLS PWM @ D=1−Vsrc/Vdel charges L, bsHS
    //                      complementary); the buck leg is static — bkHS ON connects source→its node, bkLS OFF.
    //   BUCK_BOOST simultaneous: all four commute (bkHS+bsLS charge D, bkLS+bsHS discharge).
    //   BUCK_BOOST splitPwm    : the two legs run at DIFFERENT phase-shifted duties — bkHS @ t2, bsLS @ t1
    //                            (t1=κD charge, t2=Vdel(1−t1)/Vsrc), so the inductor sees a mild (Vsrc−Vdel)
    //                            freewheel interval instead of the full swing → strictly lower ripple.
    const char* bkHS = d.reverse ? "Q3" : "Q1";   // source-side high-side
    const char* bkLS = d.reverse ? "Q4" : "Q2";   // source-side low-side
    const char* bsHS = d.reverse ? "Q1" : "Q3";   // delivered-side high-side
    const char* bsLS = d.reverse ? "Q2" : "Q4";   // delivered-side low-side
    const double dt = d.deadFraction, Dr = d.regionDuty;
    auto stim = [&](const char* sw, double duty, double phaseDeg) {
        json st; st["stage"] = "fsbbCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    auto compl_of = [&](double duty) { return std::max(0.0, (1.0 - duty) - 2.0 * dt); };
    if (d.region == "buck") {
        tas["simulation"]["stimulus"] = json::array({
            stim(bkHS, Dr, 0.0),                                   // buck HS PWM
            stim(bkLS, compl_of(Dr), (Dr + dt) * 360.0),          // buck LS complementary
            stim(bsHS, 1.0, 0.0),                                  // boost HS static ON (node->delivered)
            stim(bsLS, 0.0, 0.0)});                               // boost LS static OFF
    } else if (d.region == "boost") {
        tas["simulation"]["stimulus"] = json::array({
            stim(bkHS, 1.0, 0.0),                                  // buck HS static ON (source->node)
            stim(bkLS, 0.0, 0.0),                                  // buck LS static OFF
            stim(bsLS, Dr, 0.0),                                   // boost LS PWM (charge L)
            stim(bsHS, compl_of(Dr), (Dr + dt) * 360.0)});        // boost HS complementary
    } else if (splitBand) {
        // Phase-shifted split PWM (MKF default). bkHS on [0,t2] (states 1+2), bsLS on [0,t1] (state 1 only);
        // the complements bkLS/bsHS fill the rest with per-leg dead time. Charge (+Vsrc) for t1, mild
        // freewheel (Vsrc−Vdel) for t2−t1, discharge (−Vdel) for 1−t2 — volt-second balanced onto Vdel.
        const double t1 = d.splitBoostDuty, t2 = d.splitBuckDuty;
        tas["simulation"]["stimulus"] = json::array({
            stim(bkHS, t2, 0.0),
            stim(bsLS, t1, 0.0),
            stim(bkLS, compl_of(t2), (t2 + dt) * 360.0),
            stim(bsHS, compl_of(t1), (t1 + dt) * 360.0)});
    } else {   // buckBoost, simultaneous
        tas["simulation"]["stimulus"] = json::array({
            stim(bkHS, Dr, 0.0),
            stim(bsLS, Dr, 0.0),
            stim(bkLS, compl_of(Dr), (Dr + dt) * 360.0),
            stim(bsHS, compl_of(Dr), (Dr + dt) * 360.0)});
    }
    req::finalize_control_seeds(tas, Topology::FOUR_SWITCH_BUCK_BOOST_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

// ─────────────────────────────────────────────────────────────────────────────
// INTERLEAVED multi-phase FSBB (ABT #94): N phase-shifted 4-switch buck-boost legs share ONE input bus,
// ONE output bus, and ONE output cap. Each leg is a full copy of the single-phase H-bridge cell (Q1..Q4 +
// body diodes + inductor + node snubbers), carrying Iout/N; the legs are staggered by 360/N degrees so their
// pulsating input/output currents interleave and the net bus ripple is reduced. Every region (buck / boost /
// buckBoost simultaneous / buckBoost splitPwm) and the forward/reverse direction compose unchanged — only the
// per-leg current, the per-leg inductance (sized in design_fsbb), and a per-leg phase offset differ from the
// single-phase deck. MKF FourSwitchBuckBoost multi-phase (numberOfPhases). N=1 never reaches here.
// ─────────────────────────────────────────────────────────────────────────────
static json build_fsbb_interleaved_tas(const FsbbDesign& d) {
    const int N = d.phaseCount;
    auto port = [](const std::string& n) { json p; p["name"] = n; return p; };
    auto pin  = [](const std::string& c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const std::string& name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const std::string& name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto sfx = [](const char* base, int k) { return std::string(base) + "_" + std::to_string(k); };

    const double fsw = d.switchingFrequency;
    const double Vo = d.outputVoltage;
    namespace AN = Kirchhoff::analytical;
    // Source / delivered rails (same aliasing as single-phase). Forward: source Vin, deliver Vo.
    const double srcNom  = d.reverse ? Vo : d.inputVoltage;
    const double delV    = d.reverse ? d.inputVoltage : Vo;
    // Per-leg delivered current: each of the N legs supplies Iout/N.
    const double IdelPhase = (d.outputPower / delV) / static_cast<double>(N);
    const double powPhase  = d.outputPower / static_cast<double>(N);   // per-leg throughput
    const double srcWorst = d.reverse ? srcNom
                                      : ((d.region == "buck") ? d.inputVoltageMax : d.inputVoltageMin);
    const bool splitBand = (d.region == "buckBoost" && d.transitionMode == "splitPwm");
    const AN::FsbbMode kMode = (d.region == "buckBoost")
        ? (splitBand ? AN::FsbbMode::SPLIT_PWM : AN::FsbbMode::SIMULTANEOUS)
        : AN::FsbbMode::BUCK_BOOST_AUTO;
    // Per-leg inductor excitation (identical across legs — the 360/N stagger does not change a leg's own
    // waveform SHAPE, only its phase, which magnetics sizing is agnostic to). d.inductance is already the
    // per-leg value (sized for Iout/N in design_fsbb).
    const double L_H = d.inductance;
    const MAS::OperatingPoint aopWorst = AN::analytical_fsbb(srcWorst, delV, IdelPhase, fsw, L_H, d.efficiency, kMode, d.splitRatio);
    const MAS::OperatingPoint aopNom   = AN::analytical_fsbb(srcNom,   delV, IdelPhase, fsw, L_H, d.efficiency, kMode, d.splitRatio);
    const double IpkL  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsL = AN::winding_current(aopWorst, 0, "rms");
    const json indExc  = AN::excitations_processed(aopNom, "L").at(0);

    // Per-leg semiconductor stresses: each switch blocks the shared bus (max(Vin_max, Vo)) but carries only
    // the per-leg inductor current, and dissipates a per-leg share (powPhase) of the conduction budget.
    const double Vblock   = std::max(d.inputVoltageMax, Vo);
    const double ratedVds = Vblock / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * powPhase / (IrmsL * IrmsL);
    const json reqSW = req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0);
    const json reqBodyD = req::body_diode(d.inputVoltage, powPhase / d.inputVoltage);

    // Per-leg node snubbers. Csw* are SIM-only dV/dt aids (tagged numerical). Crc*/Rrc* are REAL EMI/ring
    // dampers sized from the per-leg energy budget (each hard-switched leg rings independently).
    auto snub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::node_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        cfg::mark_numerical_aid(c);
        return c; };
    const double rcSnubC = cfg::snubber_cap(d.config, powPhase, Vblock, fsw);
    const double rcSnubR = cfg::snubber_res(d.config);
    const auto rcSnub = req::snubber(rcSnubC, rcSnubR, Vblock, fsw);
    const json& rcCap = rcSnub.first;
    const json& rcRes = rcSnub.second;
    auto indBrick = [&]() { json j; j["magnetic"] = json::object();
        j["inputs"] = req::magnetic_inputs(d.inductance, 0.2, /*single winding*/ {}, {"primary"},
            std::nullopt, 25.0, {indExc}); return j; };

    // ── switching cell: N legs sharing vin / gnd / vout ──────────────────────────────────────────────
    json cell; cell["name"] = "fsbb-cell-interleaved";
    std::vector<json> ports = {port("vin"), port("gnd"), port("vout")};
    for (int k = 1; k <= N; ++k)
        for (const char* g : {"g1", "g2", "g3", "g4"}) ports.push_back(port(sfx(g, k)));
    cell["ports"] = ports;

    std::vector<json> comps;
    std::vector<json> vinEps, gndEps, voutEps;
    std::vector<json> conns;
    for (int k = 1; k <= N; ++k) {
        comps.push_back(comp(sfx("Q1", k), mosfet(reqSW)));
        comps.push_back(comp(sfx("Q2", k), mosfet(reqSW)));
        comps.push_back(comp(sfx("Q3", k), mosfet(reqSW)));
        comps.push_back(comp(sfx("Q4", k), mosfet(reqSW)));
        auto bodyD = [&](const char* nm) { json j; j["semiconductor"]["diode"] = json::object();
            j["inputs"]["designRequirements"] = reqBodyD; return comp(nm, j); };
        comps.push_back(bodyD(sfx("D1", k).c_str()));
        comps.push_back(bodyD(sfx("D2", k).c_str()));
        comps.push_back(bodyD(sfx("D3", k).c_str()));
        comps.push_back(bodyD(sfx("D4", k).c_str()));
        comps.push_back(comp(sfx("L", k), indBrick()));
        comps.push_back(comp(sfx("Csw1", k), snub()));
        comps.push_back(comp(sfx("Csw2", k), snub()));
        comps.push_back(comp(sfx("Crc_sw1", k), rcCap));
        comps.push_back(comp(sfx("Rrc_sw1", k), rcRes));
        comps.push_back(comp(sfx("Crc_sw2", k), rcCap));
        comps.push_back(comp(sfx("Rrc_sw2", k), rcRes));

        // shared-bus endpoints (aggregated across every leg)
        vinEps.push_back(pin(sfx("Q1", k), "drain"));  vinEps.push_back(pin(sfx("D1", k), "cathode"));
        voutEps.push_back(pin(sfx("Q3", k), "drain")); voutEps.push_back(pin(sfx("D3", k), "cathode"));
        for (const json& e : { pin(sfx("Q2", k), "source"), pin(sfx("Q4", k), "source"),
                               pin(sfx("D2", k), "anode"),  pin(sfx("D4", k), "anode"),
                               pin(sfx("Csw1", k), "2"),    pin(sfx("Csw2", k), "2"),
                               pin(sfx("Rrc_sw1", k), "2"), pin(sfx("Rrc_sw2", k), "2") }) gndEps.push_back(e);

        // per-leg switching nodes + snubber midpoints + gate breakouts
        conns.push_back(conn(sfx("sw1_net", k), {pin(sfx("Q1", k), "source"), pin(sfx("Q2", k), "drain"),
            pin(sfx("D1", k), "anode"), pin(sfx("D2", k), "cathode"),
            pin(sfx("L", k), "primary_start"), pin(sfx("Csw1", k), "1"), pin(sfx("Crc_sw1", k), "1")}));
        conns.push_back(conn(sfx("sw2_net", k), {pin(sfx("Q4", k), "drain"), pin(sfx("Q3", k), "source"),
            pin(sfx("D4", k), "cathode"), pin(sfx("D3", k), "anode"),
            pin(sfx("L", k), "primary_end"), pin(sfx("Csw2", k), "1"), pin(sfx("Crc_sw2", k), "1")}));
        conns.push_back(conn(sfx("rc_sw1_mid", k), {pin(sfx("Crc_sw1", k), "2"), pin(sfx("Rrc_sw1", k), "1")}));
        conns.push_back(conn(sfx("rc_sw2_mid", k), {pin(sfx("Crc_sw2", k), "2"), pin(sfx("Rrc_sw2", k), "1")}));
        conns.push_back(conn(sfx("g1_net", k), {pin(sfx("Q1", k), "gate"), prt(sfx("g1", k).c_str())}));
        conns.push_back(conn(sfx("g2_net", k), {pin(sfx("Q2", k), "gate"), prt(sfx("g2", k).c_str())}));
        conns.push_back(conn(sfx("g3_net", k), {pin(sfx("Q3", k), "gate"), prt(sfx("g3", k).c_str())}));
        conns.push_back(conn(sfx("g4_net", k), {pin(sfx("Q4", k), "gate"), prt(sfx("g4", k).c_str())}));
    }
    vinEps.push_back(prt("vin"));
    voutEps.push_back(prt("vout"));
    gndEps.push_back(prt("gnd"));
    // shared buses first, then per-leg nets (deterministic order).
    std::vector<json> allConns = {conn("vin_net", vinEps), conn("vout_net", voutEps), conn("gnd_net", gndEps)};
    for (json& c : conns) allConns.push_back(c);
    cell["components"] = comps;
    cell["connections"] = allConns;

    // one shared output filter cap on the delivered rail
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    capd["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;
    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    // ── requirements + operating point (source / delivered rails; forward or reverse) ────────────────
    const double srcNomV = d.reverse ? Vo : d.inputVoltage;
    const double srcMinV = d.reverse ? Vo : d.inputVoltageMin;
    const double srcMaxV = d.reverse ? Vo : d.inputVoltageMax;
    const double deliverV = d.reverse ? d.inputVoltage : Vo;
    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", srcMinV}, {"nominal", srcNomV}, {"maximum", srcMaxV}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = deliverV; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = srcNomV; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage("fsbbCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = d.reverse
        ? json::array({
            isc("Vout", "externalPort", "input",  {sp("fsbbCell", "vout")}),
            isc("GND",  "externalPort", "input",  {sp("fsbbCell", "gnd"), sp("filter", "rtn")}),
            isc("Vin",  "externalPort", "output", {sp("fsbbCell", "vin"), sp("filter", "in")})})
        : json::array({
            isc("Vin",  "externalPort", "input",  {sp("fsbbCell", "vin")}),
            isc("GND",  "externalPort", "input",  {sp("fsbbCell", "gnd"), sp("filter", "rtn")}),
            isc("Vout", "externalPort", "output", {sp("fsbbCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});

    // ── gate drive: the single-phase region scheme, replicated per leg with a 360/N phase stagger ────
    const double dt = d.deadFraction, Dr = d.regionDuty;
    auto wrap = [](double p) { double q = std::fmod(p, 360.0); return q < 0.0 ? q + 360.0 : q; };
    auto stim = [&](const std::string& sw, double duty, double phaseDeg) {
        json st; st["stage"] = "fsbbCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = wrap(phaseDeg); return st; };
    auto compl_of = [&](double duty) { return std::max(0.0, (1.0 - duty) - 2.0 * dt); };
    std::vector<json> stimuli;
    for (int k = 1; k <= N; ++k) {
        const double off = 360.0 * static_cast<double>(k - 1) / static_cast<double>(N);
        // leg-role aliases (same as single-phase): source-side "buck" leg + delivered-side "boost" leg.
        const std::string bkHS = sfx(d.reverse ? "Q3" : "Q1", k);
        const std::string bkLS = sfx(d.reverse ? "Q4" : "Q2", k);
        const std::string bsHS = sfx(d.reverse ? "Q1" : "Q3", k);
        const std::string bsLS = sfx(d.reverse ? "Q2" : "Q4", k);
        if (d.region == "buck") {
            stimuli.push_back(stim(bkHS, Dr, off));
            stimuli.push_back(stim(bkLS, compl_of(Dr), (Dr + dt) * 360.0 + off));
            stimuli.push_back(stim(bsHS, 1.0, off));
            stimuli.push_back(stim(bsLS, 0.0, off));
        } else if (d.region == "boost") {
            stimuli.push_back(stim(bkHS, 1.0, off));
            stimuli.push_back(stim(bkLS, 0.0, off));
            stimuli.push_back(stim(bsLS, Dr, off));
            stimuli.push_back(stim(bsHS, compl_of(Dr), (Dr + dt) * 360.0 + off));
        } else if (splitBand) {
            const double t1 = d.splitBoostDuty, t2 = d.splitBuckDuty;
            stimuli.push_back(stim(bkHS, t2, off));
            stimuli.push_back(stim(bsLS, t1, off));
            stimuli.push_back(stim(bkLS, compl_of(t2), (t2 + dt) * 360.0 + off));
            stimuli.push_back(stim(bsHS, compl_of(t1), (t1 + dt) * 360.0 + off));
        } else {   // buckBoost simultaneous
            stimuli.push_back(stim(bkHS, Dr, off));
            stimuli.push_back(stim(bsLS, Dr, off));
            stimuli.push_back(stim(bkLS, compl_of(Dr), (Dr + dt) * 360.0 + off));
            stimuli.push_back(stim(bsHS, compl_of(Dr), (Dr + dt) * 360.0 + off));
        }
    }
    tas["simulation"]["stimulus"] = stimuli;
    req::finalize_control_seeds(tas, Topology::FOUR_SWITCH_BUCK_BOOST_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
