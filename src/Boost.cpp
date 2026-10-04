#include "Boost.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"   // single FHA source: analytical_boost + excitations_processed/winding_current
#include "KirchhoffConfig.hpp"
#include "Interleave.hpp"
#include "Fcml.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }

// Interleaved boost (ABT #1437): the output capacitor is fed by the N staggered DIODE currents — each phase's
// inductor current while its switch is off — so the per-phase inductor waveform from the analytical solver is
// gated to its off interval (in DCM the inductor is already at zero after tOn + tOff, so the gate stays exact).
InterleavedRipple boost_interleaved_cap_ripple(const BoostDesign& d, double vin) {
    const double T = 1.0 / d.switchingFrequency;
    const double iPhase = d.outputPower / d.outputVoltage / d.phaseCount;
    const MAS::OperatingPoint op = analytical::analytical_boost(vin, d.outputVoltage, iPhase, d.switchingFrequency,
        d.inductance, req::analytical_rectifier_drop(d.config), d.efficiency);
    const double tOn = analytical::winding_current(op, 0, "dutyCycle") * T;
    auto [t, i] = analytical::winding_current_samples(op, 0, T);
    // Insert the switch-off edge as a step so the gate is exact rather than smeared across a segment.
    std::vector<double> tt, ii;
    for (std::size_t k = 0; k < t.size(); ++k) {
        if (k > 0 && t[k - 1] < tOn && t[k] > tOn) {
            const double v = i[k - 1] + (tOn - t[k - 1]) / (t[k] - t[k - 1]) * (i[k] - i[k - 1]);
            tt.push_back(tOn); ii.push_back(0.0); tt.push_back(tOn); ii.push_back(v);
        }
        tt.push_back(t[k]);
        ii.push_back(t[k] < tOn ? 0.0 : (t[k] == tOn ? 0.0 : i[k]));
    }
    return interleaved_capacitor_ripple(tt, ii, T, d.phaseCount);
}
} // namespace

BoostDesign design_boost(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    BoostDesign d{};
    d.config = cfg::object_of(tasInputs);
    // ONE output, and say so. This cell has a single output rail; asking for more used to design
    // outputs[0] and DISCARD the rest without a word — the caller got a one-output converter back
    // and the schematic drew it as the answer (ABT #752). flyback and llc already refuse; so does this.
    if (dr.at("outputs").size() > 1)
        throw std::invalid_argument("boost design: this topology has a single output rail, but "
                                    + std::to_string(dr.at("outputs").size())
                                    + " outputs were requested");
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    // Ideal design assumes a lossless diode (Vd=0), matching MKF's ideal reference
    // (Boost::calculate_duty_cycle with diodeVoltageDrop=0) -> D = 1 - Vin/Vout.
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }

    // Output rectifier variant (config-gated, no schema change; default = diode for back-compat).
    // "synchronous" replaces the high-side output diode with a high-side MOSFET (abt #67) — lower
    // conduction loss (I^2*Rds vs I*Vf) -> higher efficiency.
    d.synchronousRectifier = (cfg::get_str(d.config, "rectifier", "diode") == std::string("synchronous"));
    d.deadFraction = cfg::get(d.config, "deadTimeFraction", 0.01);   // 1% of the period per dead band
    // Interleaving (ABT #1437): N cells share the rails, each carrying Iout/N; N = 1 is unchanged.
    d.phaseCount = phase_count(d.config, "design_boost");
    const double N = static_cast<double>(d.phaseCount);
    // Flying-capacitor multilevel (ABT #1439): complementary pairs by construction, so always synchronous.
    d.levels = fcml_levels(d.config, "design_boost");
    if (d.levels > 2) {
        if (d.phaseCount > 1)
            throw std::invalid_argument("design_boost: an interleaved FCML (phaseCount > 1 with levels > 2) is not modelled");
        if (d.config.contains("rectifier") && cfg::get_str(d.config, "rectifier", "") != std::string("synchronous"))
            throw std::invalid_argument("design_boost: an FCML is synchronous by construction (complementary pairs); "
                                        "rectifier '" + cfg::get_str(d.config, "rectifier", "") + "' contradicts levels > 2");
        d.synchronousRectifier = true;
    }

    d.diodeDrop = req::rectifier_drop(d.config, d.outputPower / d.outputVoltage / N);  // DIDEAL Vf at the operating rectifier current
    const double Vo = d.outputVoltage + d.diodeDrop;
    // With a synchronous rectifier the output path is a near-ideal MOSFET (no Vf), so NO diode-drop
    // compensation: D = 1 - Vin*eff/Vout. With a diode, compensate for its forward drop as MKF does.
    d.dutyCycle = d.synchronousRectifier
        ? 1.0 - d.inputVoltage * d.efficiency / d.outputVoltage
        : 1.0 - d.inputVoltage * d.efficiency / Vo;  // MKF Boost::calculate_duty_cycle
    // A boost can only STEP UP: it needs Vout > Vin/efficiency, otherwise the duty goes <= 0 and the
    // sized output capacitance (iout*D/(fsw*ripple*Vo)) flips NEGATIVE (abt #68). Fail loud rather than
    // emit a degraded/negative design (CLAUDE.md no-fallback rule) — a mis-specced step-down boost is a
    // spec error the caller must fix, not silently clamp.
    if (d.dutyCycle <= 0.0)
        throw std::runtime_error(
            "Kirchhoff boost: requires Vout > Vin/efficiency (boost cannot step down); got Vin="
            + std::to_string(d.inputVoltage) + " V, Vout=" + std::to_string(d.outputVoltage)
            + " V, efficiency=" + std::to_string(d.efficiency) + " -> duty <= 0");

    // Inductance — faithful port of MKF Boost::process_design_requirements():
    //   maximumCurrentRiple = currentRippleRatio · Iout
    //   L = Vin_max·(Vout − Vin_max) / (maximumCurrentRiple · fsw · Vout)
    // sized at the maximum input voltage corner.
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;
    const double rippleRatio = cfg::ripple_ratio(d.config, 0.4);
    const double iout = d.outputPower / d.outputVoltage / N;             // per phase
    // Ripple-ratio rule by default; config "maximumSwitchCurrent" instead sizes L so the peak inductor/
    // switch current (Iin + ΔIL/2) lands exactly on the cap (ABT #95). The boost inductor is the INPUT
    // inductor: its average at the vinMax sizing corner is Iin = Pout/(η·vinMax).
    const double iLavg = d.outputPower / (d.efficiency * vinMax) / N;    // per phase
    const double maxCurrentRipple = cfg::max_current_ripple(d.config, rippleRatio, iout, iLavg, "design_boost", d.config.contains("rippleRatio"));
    d.inductance = req::provided_inductance(dr).value_or(
        vinMax * (d.outputVoltage - vinMax) / (maxCurrentRipple * d.switchingFrequency * d.outputVoltage));
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;
    if (d.levels > 2) {
        // FCML: the inductor sees a Vout/(N−1) step at (N−1)·fs. L for the worst ripple over the input range; Cout as the
        // plain boost (the output is fed by the top pair once per period); each flying capacitor for its charge at the
        // largest inductor current (Vin_min).
        // The cell runs the efficiency-compensated duty, i.e. the lossless cell's output at Vo/η (buck D·Vin, boost
        // Vin/(1−D) — both Vo/η): size from that waveform, the one the deck switches.
        const int M = d.levels - 1;
        const double voCell = d.outputVoltage / d.efficiency;
        double vsMax = 0, qMax = 0;
        for (int i = 0; i <= 40; ++i) {
            const double vin = d.inputVoltageMin + (d.inputVoltageMax - d.inputVoltageMin) * i / 40.0;
            vsMax = std::max(vsMax, analytical::fcml_ripple_volt_seconds(true, d.levels, vin, voCell, d.switchingFrequency));
            qMax = std::max(qMax, analytical::fcml_flying_charge_per_amp(true, d.levels, vin, voCell, d.switchingFrequency)
                                  * d.outputPower / (d.efficiency * vin));
        }
        if (!req::provided_inductance(dr) && !(vsMax > 0))
            throw std::invalid_argument("design_boost: the FCML inductor ripple vanishes over the whole input range (D·(N−1) is "
                                        "a whole number there), so there is no ripple to size L from; pin the inductance "
                                        "or widen the input range");
        d.inductance = req::provided_inductance(dr).value_or(vsMax / maxCurrentRipple);
        d.outputCapacitance = iout * d.dutyCycle / (d.switchingFrequency * cfg::output_ripple_fraction(d.config) * d.outputVoltage);
        const double cfRipple = cfg::get(d.config, "flyingCapacitorRipple", 0.05);
        if (!(cfRipple > 0)) throw std::invalid_argument("design_boost: flyingCapacitorRipple must be > 0");
        d.flyingCapacitance = qMax / (cfRipple * d.outputVoltage / M);
    } else if (d.phaseCount == 1) {
        d.outputCapacitance = iout * d.dutyCycle / (d.switchingFrequency * cfg::output_ripple_fraction(d.config) * d.outputVoltage);
    } else {
        // Sized from the summed diode currents at every input corner (their ripple partly cancels).
        d.outputCapacitance = 0;
        for (double vin : {d.inputVoltageMin, d.inputVoltage, d.inputVoltageMax})
            d.outputCapacitance = std::max(d.outputCapacitance,
                boost_interleaved_cap_ripple(d, vin).chargePeakToPeak / (cfg::output_ripple_fraction(d.config) * d.outputVoltage));
    }
    return d;
}

json build_boost_tas(const BoostDesign& d);

// N-phase interleaved boost (ABT #1437): one phase designed at Pout/N, assembled by the single-phase builder,
// replicated by interleave_stage; the operating point and the output capacitor are re-stamped for the whole
// converter (see build_buck_interleaved_tas).
static json build_boost_interleaved_tas(const BoostDesign& d) {
    BoostDesign one = d;
    one.phaseCount = 1;
    one.outputPower = d.outputPower / d.phaseCount;
    json tas = interleave_stage(build_boost_tas(one), "switchingCell", d.phaseCount, {"vin", "gnd", "vout"});
    tas["inputs"]["operatingPoints"][0]["outputs"][0]["power"] = d.outputPower;

    double rippleRms = 0, ipkPhase = 0;
    for (double vin : {d.inputVoltageMin, d.inputVoltage, d.inputVoltageMax}) {
        rippleRms = std::max(rippleRms, boost_interleaved_cap_ripple(d, vin).acCurrentRms);
        const MAS::OperatingPoint op = analytical::analytical_boost(vin, d.outputVoltage, d.outputPower / d.outputVoltage / d.phaseCount,
            d.switchingFrequency, d.inductance, req::analytical_rectifier_drop(d.config), d.efficiency);
        ipkPhase = std::max(ipkPhase, analytical::winding_current(op, 0, "peak"));
    }
    const json capReq = req::capacitor(d.outputCapacitance, d.outputVoltage / cfg::v_derate_capacitor(d.config), rippleRms,
        cfg::get(d.config, "esrRippleFraction", req::ESR_RIPPLE_FRACTION) * d.outputVoltage / (d.phaseCount * ipkPhase),
        "outputFilter");
    bool stamped = false;
    for (auto& st : tas.at("topology").at("stages"))
        if (st.at("name") == "filter")
            for (auto& c : st.at("circuit").at("components"))
                if (c.at("name") == "Cout") { c["data"]["inputs"]["designRequirements"] = capReq; stamped = true; }
    if (!stamped) throw std::logic_error("build_boost_interleaved_tas: the TAS has no filter-stage Cout");
    return tas;
}

json build_boost_tas(const BoostDesign& d) {
    if (d.phaseCount > 1) return build_boost_interleaved_tas(d);
    if (d.levels > 2) {
        FcmlCell c;
        c.boost = true; c.levels = d.levels;
        c.inputVoltage = d.inputVoltage; c.inputVoltageMin = d.inputVoltageMin; c.inputVoltageMax = d.inputVoltageMax;
        c.outputVoltage = d.outputVoltage; c.outputPower = d.outputPower; c.switchingFrequency = d.switchingFrequency;
        c.efficiency = d.efficiency; c.dutyCycle = d.dutyCycle; c.deadFraction = d.deadFraction;
        c.inductance = d.inductance; c.flyingCapacitance = d.flyingCapacitance; c.outputCapacitance = d.outputCapacitance;
        c.config = d.config;
        return build_fcml_tas(c);
    }
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };

    // --- per-component stresses from the SINGLE FHA source (the SPICE-validated analytical solver) ---
    // Worst-case corner (Vin_min) drives the ratings; the declared nominal OP is what the TAS embeds.
    namespace AN = Kirchhoff::analytical;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw, L_H = d.inductance;
    const double Iout = d.outputPower / d.outputVoltage;
    // maximumDutyCycle gate (ABT #95): default 1.0 preserves the historical guard (boost already throws at
    // D>=1 and D<=0); a user tightens it via config to get a loud throw before the worst-case duty runs away.
    const double maxDuty = cfg::get(d.config, "maximumDutyCycle", 1.0);
    const MAS::OperatingPoint aopWorst = AN::analytical_boost(d.inputVoltageMin, d.outputVoltage, Iout, fsw,
                                                              L_H, req::analytical_rectifier_drop(d.config), d.efficiency, maxDuty);
    const MAS::OperatingPoint aopNom   = AN::analytical_boost(d.inputVoltage,    d.outputVoltage, Iout, fsw,
                                                              L_H, req::analytical_rectifier_drop(d.config), d.efficiency, maxDuty);
    const double Dmax  = AN::winding_current(aopWorst, 0, "dutyCycle");
    const double IpkL  = AN::winding_current(aopWorst, 0, "peak");
    const double IrmsL = AN::winding_current(aopWorst, 0, "rms");
    const double IswRms   = std::sqrt(Dmax) * IrmsL;                       // switch conducts during D
    const double IdiodeRms = std::sqrt(1.0 - Dmax) * IrmsL;               // diode conducts during 1-D
    const double IcoutRms = std::sqrt(std::max(0.0, IdiodeRms * IdiodeRms - Iout * Iout));
    // VOLTAGES at Vin_max: switch and diode both block Vout.
    const double ratedVds = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double ratedVr  = d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IswRms * IswRms);
    const double maxVf    = (ratedVr < 100.0) ? 0.6 : 1.2;

    json ind; ind["magnetic"] = json::object();
    ind["inputs"] = req::magnetic_inputs(d.inductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, AN::excitations_processed(aopNom, "L1"));
    json mosfet; mosfet["semiconductor"]["mosfet"] = json::object();
    mosfet["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpkL, maxRdsOn, 125.0);
    json diode;  diode["semiconductor"]["diode"] = json::object();
    diode["inputs"]["designRequirements"] = req::diode(ratedVr, Iout / 0.7, maxVf, 0.05 * T);
    // Synchronous output rectifier (abt #67): a HIGH-SIDE MOSFET Q2 (drain=vout, source=sw_node) carries
    // the off-time output current through its channel (I^2*Rds, much less than a diode's I*Vf) plus its own
    // body diode D2 across the dead time. Sized like the main switch (blocks Vout, carries the inductor pk).
    json syncFet; syncFet["semiconductor"]["mosfet"] = json::object();
    syncFet["inputs"]["designRequirements"] = req::mosfet("synchronousRectifier", ratedVds, IpkL, maxRdsOn, 125.0);
    json bodyD; bodyD["semiconductor"]["diode"] = json::object();
    bodyD["inputs"]["designRequirements"] = req::body_diode(ratedVds, Iout / 0.7);
    json capd; capd["capacitor"] = json::object();
    capd["inputs"]["designRequirements"] = req::capacitor(
        d.outputCapacitance, d.outputVoltage / cfg::v_derate_capacitor(d.config), IcoutRms,
        cfg::get(d.config, "esrRippleFraction", req::ESR_RIPPLE_FRACTION) * d.outputVoltage / IpkL,
        "outputFilter");

    // The boost power stage is ONE functional block (the switching cell): inductor + switch + diode.
    // A canonical TAS power stage is a series two-port (dcBus in -> pulsatingDc out); the shunt switch
    // and the inductor are internal to it, not separate stages. (sw_node stays brick-internal.)
    // DEFAULT: high-side output diode D1 (sw_node->vout). SYNCHRONOUS: high-side MOSFET Q2 (drain=vout,
    // source=sw_node) + its body diode D2 (anode=sw_node, cathode=vout, same orientation as D1).
    json cell; cell["name"] = "boost-switching-cell";
    if (d.synchronousRectifier) {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate"), port("gate2")});
        cell["components"] = json::array({comp("L1", ind), comp("Q1", mosfet),
                                          comp("Q2", syncFet), comp("D2", bodyD)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("L1", "primary_start"), prt("vin")}),
            conn("sw_node",  {pin("L1", "primary_end"), pin("Q1", "drain"),
                              pin("Q2", "source"), pin("D2", "anode")}),
            conn("gnd_net",  {pin("Q1", "source"), prt("gnd")}),
            conn("vout_net", {pin("Q2", "drain"), pin("D2", "cathode"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")}),
            conn("gate2_net",{pin("Q2", "gate"), prt("gate2")})});
    } else {
        cell["ports"] = json::array({port("vin"), port("gnd"), port("vout"), port("gate")});
        cell["components"] = json::array({comp("L1", ind), comp("Q1", mosfet), comp("D1", diode)});
        cell["connections"] = json::array({
            conn("vin_net",  {pin("L1", "primary_start"), prt("vin")}),
            conn("sw_node",  {pin("L1", "primary_end"), pin("Q1", "drain"), pin("D1", "anode")}),
            conn("gnd_net",  {pin("Q1", "source"), prt("gnd")}),
            conn("vout_net", {pin("D1", "cathode"), prt("vout")}),
            conn("gate_net", {pin("Q1", "gate"), prt("gate")})});
    }

    // Output filter = Cout only; the LOAD is synthesized from the outputs requirement by the
    // assembler (it is a boundary condition, not a converter stage — the dual of the input source).
    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", capd)});
    filt["connections"] = json::array({
        conn("out", {pin("Cout", "1"), prt("in")}),
        conn("ret", {pin("Cout", "2"), prt("rtn")})});

    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };

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
        pstage("switchingCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput")),
        req::control_stage("pwmController")});   // sourced control IC (BOM); skipped in the power deck
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("switchingCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("switchingCell", "gnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("switchingCell", "vout"), sp("filter", "in")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    json st; st["stage"] = "switchingCell"; st["component"] = "Q1"; st["signal"] = "gate";
    st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
    st["waveform"]["dutyCycle"] = d.dutyCycle;
    tas["simulation"]["stimulus"] = json::array({st});
    if (d.synchronousRectifier) {
        // Q2 drives complementary to the low-side main switch Q1 with a dead band on each edge; the body
        // diode D2 carries the inductor current across the dead time (prevents shoot-through vout->gnd).
        // duty = (1-D) - 2*dt, phased to start after Q1 turns off. The gate is ground-referenced like Q1
        // (the deck models the switch as a gate-to-ground VC-switch, so the floating source is transparent).
        const double dt = d.deadFraction;
        json st2; st2["stage"] = "switchingCell"; st2["component"] = "Q2"; st2["signal"] = "gate";
        st2["waveform"]["type"] = "pwm"; st2["waveform"]["frequency"] = d.switchingFrequency;
        st2["waveform"]["dutyCycle"] = std::max(0.0, (1.0 - d.dutyCycle) - 2.0 * dt);
        st2["waveform"]["phase"] = (d.dutyCycle + dt) * 360.0;
        tas["simulation"]["stimulus"].push_back(st2);
    }
    req::finalize_control_seeds(tas, Topology::BOOST_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
