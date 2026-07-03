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
    // Interleaved (multi-phase) FSBB is not wired yet — reject loudly rather than silently building a
    // single-phase deck (no-silent-shortcuts). Tracked as the remaining ABT #94 follow-up.
    if (cfg::get(d.config, "phaseCount", 1.0) != 1.0)
        throw std::invalid_argument("design_fsbb: interleaved (phaseCount > 1) FSBB is not implemented "
                                    "(ABT #94 follow-up); only single-phase is supported");

    // Source / delivered rails. Forward: source Vin, deliver Vo. Reverse (bidirectional, ABT #94): the Vout
    // rail sources and the Vin rail receives — the symmetric synchronous H-bridge conducts both ways, so
    // reverse is the same converter with the two legs' roles swapped. Region/duty are sized on (Vsrc,Vdel).
    const double Vsrc = d.reverse ? Vo : Vin;
    const double Vdel = d.reverse ? Vin : Vo;
    const double Io = d.outputPower / Vdel;     // current into the delivered (load) rail
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
    const double Lbuck  = (vinMax > Vo) ? Vo * (vinMax - Vo) / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Io * Fs * vinMax) : 0.0;
    const double Lboost = (vinMin < Vo) ? (vinMin * vinMin) * (Vo - vinMin)
                                          / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Io * Fs * Vo * Vo) : 0.0;
    double L = std::max(Lbuck, Lboost);
    if (L <= 0.0) {
        // Exact-unity / single-point spec (Vin_min == Vin_max == Vo, e.g. a 12->12 V request with no input
        // range): neither the buck (vinMax>Vo) nor the boost (vinMin<Vo) region's formula fires, leaving
        // L = 0 — a broken deck (the inductor vanishes, output collapses to ~Vin·D ≈ 3.3 V). Size it
        // directly for the 4-switch SIMULTANEOUS mode the design always runs: the charge phase applies Vin
        // across L for D·T, with ΔIL targeted at kRippleRatio (config "inductorRippleRatio") of the buck-boost current Io/(1-D):
        //     L = Vin·D·(1-D) / (kRippleRatio·Io·Fs).   (ABT #26)
        const double D = d.dutyCycle;
        L = Vin * D * (1.0 - D) / (cfg::get(d.config, "inductorRippleRatio", kRippleRatio) * Io * Fs);
    }
    d.inductance = req::provided_inductance(dr).value_or(
        L);

    d.loadResistance = Vdel * Vdel / d.outputPower;   // load sits on the delivered rail (Vo fwd / Vin rev)
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

json build_fsbb_tas(const FsbbDesign& d) {
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
    const MAS::OperatingPoint aopWorst = AN::analytical_fsbb(srcWorst, delV, Idel, fsw, L_H, d.efficiency, kMode, d.splitRatio);
    const MAS::OperatingPoint aopNom   = AN::analytical_fsbb(srcNom,   delV, Idel, fsw, L_H, d.efficiency, kMode, d.splitRatio);
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

} // namespace Kirchhoff
