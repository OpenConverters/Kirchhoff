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
    const double Io = d.outputPower / Vo;
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", kDeadFrac);
    // Buck-boost simultaneous mode gain M = D/(1-D) = Vo/Vin  =>  D = Vo/(Vin+Vo).
    d.dutyCycle = Vo / (Vin + Vo);
    // Operating-region classification at the NOMINAL input (MKF FourSwitchBuckBoost::Region). A real 4SBB
    // controller runs only the buck leg when Vin comfortably exceeds Vo (BUCK), only the boost leg when Vo
    // exceeds Vin (BOOST), and commutes all four switches in the narrow transition band (BUCK_BOOST). The
    // KH deck previously ran the 4-switch SIMULTANEOUS scheme for EVERY point, giving buck/boost-dominant
    // designs the wrong (lossier) gate drive. `fsbbTransitionBand` (default 0.10) is the ±fraction of Vo
    // around Vin within which we stay in the all-switching buck-boost region.
    const double band = cfg::get(d.config, "fsbbTransitionBand", 0.10);
    if (Vin > Vo * (1.0 + band)) {
        d.region = "buck";
        d.regionDuty = Vo / (Vin * d.efficiency);
    } else if (Vin < Vo * (1.0 - band)) {
        d.region = "boost";
        d.regionDuty = 1.0 - (Vin * d.efficiency) / Vo;
    } else {
        d.region = "buckBoost";
        d.regionDuty = d.dutyCycle;
    }

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

    d.loadResistance = Vo * Vo / d.outputPower;
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

    // Single inductor L (single-winding magnetic) sourced from the SINGLE FHA source — analytical_fsbb in
    // SIMULTANEOUS mode: the 4-switch buck-boost mode this deck runs for EVERY Vin/Vo (charge phase +Vin
    // across L for D·T, discharge −Vo for (1-D)·T, D = Vo/(Vin+Vo)), regular at Vo==Vin. Ratings pull from
    // the Vin_min corner (max duty / max inductor current, mirrors Boost); the embedded excitation is the
    // nominal-Vin operating point (same convention as the boost/pfc/vienna single-inductor conversions).
    const double fsw = d.switchingFrequency, L_H = d.inductance;
    const double Vo = d.outputVoltage, Io = d.outputPower / Vo;
    namespace AN = Kirchhoff::analytical;
    // Region-aware waveform source: BUCK/BOOST regions embed analytical_fsbb's per-region (buck-/boost-like)
    // inductor waveform; the transition band embeds the 4-switch SIMULTANEOUS waveform. Ratings pull from
    // the region's own worst corner (buck: Vin_max = max ripple & block voltage; boost/bb: Vin_min = max
    // duty & inductor current).
    const AN::FsbbMode kMode = (d.region == "buckBoost") ? AN::FsbbMode::SIMULTANEOUS
                                                         : AN::FsbbMode::BUCK_BOOST_AUTO;
    const double vinWorst = (d.region == "buck") ? d.inputVoltageMax : d.inputVoltageMin;
    const MAS::OperatingPoint aopWorst = AN::analytical_fsbb(vinWorst,        Vo, Io, fsw, L_H, d.efficiency, kMode);
    const MAS::OperatingPoint aopNom   = AN::analytical_fsbb(d.inputVoltage,  Vo, Io, fsw, L_H, d.efficiency, kMode);
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
        pstage("fsbbCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("fsbbCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("fsbbCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("fsbbCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Region-dependent gate drive (MKF FourSwitchBuckBoost). Each leg has a dead band on its complementary
    // pair; body diodes carry the inductor current across the dead time.
    //   BUCK  (Vin>Vo): only the buck leg switches (Q1 PWM @ D=Vo/Vin, Q2 complementary); the boost leg is
    //                   static — Q3 ON connects sw2→vout, Q4 OFF. Inductor behaves like a buck output choke.
    //   BOOST (Vo>Vin): only the boost leg switches (Q4 PWM @ D=1−Vin/Vo charges L, Q3 complementary); the
    //                   buck leg is static — Q1 ON connects vin→sw1, Q2 OFF.
    //   BUCK_BOOST    : all four commute (Q1+Q4 charge D, Q2+Q3 discharge) — the transition-band scheme.
    const double dt = d.deadFraction, Dr = d.regionDuty;
    auto stim = [&](const char* sw, double duty, double phaseDeg) {
        json st; st["stage"] = "fsbbCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    if (d.region == "buck") {
        tas["simulation"]["stimulus"] = json::array({
            stim("Q1", Dr, 0.0),                                   // buck HS PWM
            stim("Q2", std::max(0.0, (1.0 - Dr) - 2.0 * dt), (Dr + dt) * 360.0),  // buck LS complementary
            stim("Q3", 1.0, 0.0),                                  // boost HS static ON (sw2->vout)
            stim("Q4", 0.0, 0.0)});                                // boost LS static OFF
    } else if (d.region == "boost") {
        tas["simulation"]["stimulus"] = json::array({
            stim("Q1", 1.0, 0.0),                                  // buck HS static ON (vin->sw1)
            stim("Q2", 0.0, 0.0),                                  // buck LS static OFF
            stim("Q4", Dr, 0.0),                                   // boost LS PWM (charge L)
            stim("Q3", std::max(0.0, (1.0 - Dr) - 2.0 * dt), (Dr + dt) * 360.0)});  // boost HS complementary
    } else {
        tas["simulation"]["stimulus"] = json::array({
            stim("Q1", Dr, 0.0),
            stim("Q4", Dr, 0.0),
            stim("Q2", std::max(0.0, (1.0 - Dr) - 2.0 * dt), (Dr + dt) * 360.0),
            stim("Q3", std::max(0.0, (1.0 - Dr) - 2.0 * dt), (Dr + dt) * 360.0)});
    }
    req::finalize_control_seeds(tas, Topology::FOUR_SWITCH_BUCK_BOOST_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
