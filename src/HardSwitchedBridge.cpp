#include "HardSwitchedBridge.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kMaxDuty = 0.45;           // per switch at Vin_min; 10 % of each half period stays dead
constexpr double kRippleRatio = 0.3;        // output-inductor pk-pk ripple over its mean, at Vin_max
constexpr double kMagnetizingFraction = 0.2;   // n·ΔIm/2 over the per-inductor current
constexpr double kSplitCapRipple = 0.02;    // half bridge: midpoint pk-pk ripple over Vin_min/2

// Duty a turns ratio needs: Vb·D = (Vo + Vd_path)·n / g, g = 2 for the full-wave rectifiers (FB, CT — one power
// pulse per half period into one inductor) and 1 for the current doubler (each inductor gets one pulse per period).
double rectifier_gain(RectifierType r) { return (r == RectifierType::CurrentDoubler) ? 1.0 : 2.0; }

analytical::SrcRectifier solver_rectifier(RectifierType r) {
    switch (r) {
        case RectifierType::CenterTapped:   return analytical::SrcRectifier::CENTER_TAPPED;
        case RectifierType::FullBridge:     return analytical::SrcRectifier::FULL_BRIDGE;
        case RectifierType::CurrentDoubler: return analytical::SrcRectifier::CURRENT_DOUBLER;
        case RectifierType::VoltageDoubler: break;
    }
    throw std::invalid_argument("Kirchhoff hard-switched bridge: voltageDoubler rectifier not supported "
                                "(variants: centerTapped, fullBridge, currentDoubler)");
}

HardSwitchedBridgeDesign design_bridge(const json& tasInputs, bool fullBridge) {
    const char* who = fullBridge ? "design_full_bridge: " : "design_half_bridge: ";
    const json& dr = tasInputs.at("designRequirements");
    HardSwitchedBridgeDesign d{};
    d.fullBridge = fullBridge;
    d.config = cfg::object_of(tasInputs);
    if (dr.at("outputs").size() != 1)
        throw std::invalid_argument(std::string(who) + "one output only (got " + std::to_string(dr.at("outputs").size()) + ")");
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    if (!(d.outputVoltage > 0))
        throw std::invalid_argument(std::string(who) + "the output voltage must be positive");
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
    d.inputVoltageMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    d.rectifierType = parse_rectifier_type(cfg::get_str(d.config, "rectifierType", "centerTapped"),
                                           RectifierType::CenterTapped);
    solver_rectifier(d.rectifierType);   // throws for the voltage doubler

    const double k = fullBridge ? 1.0 : 0.5, g = rectifier_gain(d.rectifierType);
    const double T = 1.0 / d.switchingFrequency, Vo = d.outputVoltage, Io = d.outputPower / Vo;
    const bool cd = (d.rectifierType == RectifierType::CurrentDoubler);
    const double iShare = cd ? 0.5 * Io : Io;   // mean current of each output inductor
    d.diodeDrop = req::rectifier_drop(d.config, Io);
    const double voPlusVd = Vo + rectifier_path_diodes(d.rectifierType) * d.diodeDrop;
    d.maxDutyCycle = cfg::get(d.config, "maxDutyCycle", kMaxDuty);
    if (!(d.maxDutyCycle > 0 && d.maxDutyCycle < 0.5))
        throw std::invalid_argument(std::string(who) + "maxDutyCycle must be in (0, 0.5), got " + std::to_string(d.maxDutyCycle));

    // Turns ratio from the duty at Vin_min, rounded DOWN to 2 dp so the rounding never pushes D past the maximum.
    const std::optional<double> pinnedN = req::provided_turns_ratio(dr, 0);
    const double nDerived = std::floor(d.maxDutyCycle * g * k * d.inputVoltageMin / voPlusVd * 100.0) / 100.0;
    if (!pinnedN && !(nDerived > 0))
        throw std::invalid_argument(std::string(who) + "Vin_min = " + std::to_string(d.inputVoltageMin) +
                                    " V is too low for any turns ratio");
    d.turnsRatio = pinnedN.value_or(nDerived);
    const double n = d.turnsRatio;
    auto duty_at = [&](double vin) { return n * voPlusVd / (g * k * vin); };
    const double dAtVinMin = duty_at(d.inputVoltageMin);
    if (dAtVinMin > d.maxDutyCycle * (1.0 + 1e-9))
        throw std::invalid_argument(std::string(who) + "turns ratio n = " + std::to_string(n) + (pinnedN ? " (pinned)" : "") +
                                    " needs a per-switch duty " + std::to_string(dAtVinMin) + " at Vin_min = " +
                                    std::to_string(d.inputVoltageMin) + " V, above maxDutyCycle " +
                                    std::to_string(d.maxDutyCycle) + "; the largest ratio is " +
                                    std::to_string(d.maxDutyCycle * g * k * d.inputVoltageMin / voPlusVd));
    d.dutyCycle = duty_at(d.inputVoltage);
    d.bridgeVoltage = k * d.inputVoltage;
    const double dAtVinMax = duty_at(d.inputVoltageMax);

    // Output inductor: the ripple is largest at Vin_max (shortest power interval, longest freewheel). It falls
    // at (Vo + Vd_path)/Lo for (½ − D)·T (full-wave) or (1 − D)·T (current doubler).
    const double ripple = cfg::get(d.config, "inductorRippleRatio", kRippleRatio);
    const double tFall = (cd ? 1.0 - dAtVinMax : 0.5 - dAtVinMax) * T;
    d.outputInductance = voPlusVd * tFall / (ripple * iShare);
    if (d.config.contains("outputInductance")) {
        d.outputInductance = cfg::get(d.config, "outputInductance", 0.0);
        if (!(d.outputInductance > 0)) throw std::invalid_argument(std::string(who) + "outputInductance must be > 0 H");
    }
    const double iLoMinAtVinMax = iShare - 0.5 * voPlusVd * tFall / d.outputInductance;
    if (!(iLoMinAtVinMax > 0))
        throw std::invalid_argument(std::string(who) + "the output inductor runs discontinuous at Vin_max (DCM is not designed)");

    // Magnetizing inductance. ΔIm = Vb·D·T/Lm is the same at every Vin (Vb·D is fixed by the output); during the
    // dead time the rectifier carries n·ΔIm/2 against the freewheeling inductor current, so it must stay below
    // the smallest one (Vin_max) or a diode drops out and the transformer is no longer clamped.
    const double vbD = voPlusVd * n / g;
    const double frac = cfg::get(d.config, "magnetizingCurrentFraction", kMagnetizingFraction);
    if (!(frac > 0)) throw std::invalid_argument(std::string(who) + "magnetizingCurrentFraction must be > 0");
    d.magnetizingInductance = req::provided_inductance(dr).value_or(n * vbD * T / (2.0 * frac * iShare));
    const double nImHalf = n * 0.5 * vbD * T / d.magnetizingInductance;
    if (!(nImHalf < iLoMinAtVinMax))
        throw std::invalid_argument(std::string(who) + "the reflected magnetizing current n·ΔIm/2 = " + std::to_string(nImHalf) +
                                    " A reaches the smallest freewheeling inductor current " + std::to_string(iLoMinAtVinMax) +
                                    " A at Vin_max: a rectifier diode would drop out in the dead time (raise Lm or "
                                    "lower inductorRippleRatio)");

    if (!fullBridge) {
        // Split capacitors: in each power interval the primary current (≈ the reflected inductor current) flows
        // through the midpoint, which both capacitors share; the midpoint moves q/(2·Cs) per interval.
        const double sr = cfg::get(d.config, "splitCapacitorRipple", kSplitCapRipple);
        if (!(sr > 0)) throw std::invalid_argument(std::string(who) + "splitCapacitorRipple must be > 0");
        const double q = (iShare / n) * dAtVinMin * T;
        d.splitCapacitance = q / (2.0 * sr * 0.5 * d.inputVoltageMin);
        d.balancingResistance = cfg::bias_res(d.config, 0.5 * d.inputVoltageMax, d.outputPower);
    }
    d.loadResistance = Vo * Vo / d.outputPower;
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", 100e-6);
    return d;
}

json build_bridge_tas(const HardSwitchedBridgeDesign& d) {
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
    const char* who = d.fullBridge ? "build_full_bridge_tas" : "build_half_bridge_tas";

    namespace AN = Kirchhoff::analytical;
    const double k = d.fullBridge ? 1.0 : 0.5;
    const double n = d.turnsRatio, Lm = d.magnetizingInductance, Lo = d.outputInductance;
    const double fsw = d.switchingFrequency, T = 1.0 / fsw;
    const double Vo = d.outputVoltage, Io = d.outputPower / Vo, Vin = d.inputVoltage;
    const bool cd = (d.rectifierType == RectifierType::CurrentDoubler);
    const double iShare = cd ? 0.5 * Io : Io;
    const AN::SrcRectifier rect = solver_rectifier(d.rectifierType);

    // Transformer excitations from the exact solver: the nominal point is embedded, the Vin_min corner (longest
    // power interval, largest inductor current peak) rates the switches.
    const MAS::OperatingPoint aopNom = AN::analytical_hard_switched_bridge(Vin, k, Vo, Io, n, fsw, Lm, Lo, d.diodeDrop, rect);
    const MAS::OperatingPoint aopWorst = AN::analytical_hard_switched_bridge(d.inputVoltageMin, k, Vo, Io, n, fsw, Lm, Lo,
                                                                             d.diodeDrop, rect);
    const double IpriPk = AN::winding_current(aopWorst, 0, "peak");
    const double IpriRms = AN::winding_current(aopWorst, 0, "rms");
    cfg::check_maximum_switch_current(d.config, IpriPk, who);
    cfg::check_maximum_drain_source_voltage(d.config, d.inputVoltageMax, who);

    // --- semiconductors. Every bridge switch blocks the whole bus (half bridge: the off switch sees Vin when its
    // partner conducts; full bridge: the same per leg). The body diodes return the leakage current at turn-off.
    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOn = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IpriRms * IpriRms);
    json mosfetReq; mosfetReq["semiconductor"]["mosfet"] = json::object();
    mosfetReq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, IpriPk, maxRdsOn, 125.0);
    auto bodyDiode = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltageMax, IpriPk); return j; };
    // Rectifier: the off diode of a centre tap blocks both half-windings (2·Vs); a full bridge or current doubler
    // diode blocks one winding (Vs). In the dead time one diode can carry the whole Io.
    const double vSecMax = k * d.inputVoltageMax / n;
    const double vrStress = (d.rectifierType == RectifierType::CenterTapped) ? 2.0 * vSecMax : vSecMax;
    const double ratedVr = vrStress / cfg::v_derate_diode(d.config);
    json diodeReq; diodeReq["semiconductor"]["diode"] = json::object();
    diodeReq["inputs"]["designRequirements"] = req::diode(ratedVr, Io / 0.7, (ratedVr < 100.0) ? 0.6 : 1.2, 0.05 * T);

    // --- transformer: one secondary winding (FB/CD) or two half-windings (CT), each at the duty-derived ratio,
    // which is a ceiling (a larger n would need more duty than Vin_min allows).
    const int wpo = rectifier_windings_per_output(d.rectifierType);
    std::vector<std::string> isoSides{"primary"};
    std::vector<double> turnsRatios;
    std::vector<bool> ceil;
    for (int w = 0; w < wpo; ++w) { isoSides.push_back("secondary"); turnsRatios.push_back(n); ceil.push_back(true); }
    json xfmr; xfmr["magnetic"] = json::object();
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, isoSides, std::nullopt, 25.0,
                                          AN::excitations_processed(aopNom, "T1"), ceil,
                                          /*lmIsMinimum: an energy-transfer transformer*/ true);

    // --- output inductor(s), at the nominal point: rising over the power interval, falling over the rest.
    const double voPlusVd = Vo + rectifier_path_diodes(d.rectifierType) * d.diodeDrop;
    const double D = d.dutyCycle, vs = k * Vin / n;
    const double tRise = D * T, tFall = (cd ? 1.0 - D : 0.5 - D) * T;
    const double dILo = voPlusVd * tFall / Lo;
    const double vLoOn = vs - voPlusVd, vLoOff = voPlusVd;
    const double dutyOn = tRise / (tRise + tFall);
    auto outInductor = [&]() { json m; m["magnetic"] = json::object();
        m["inputs"] = req::magnetic_inputs(Lo, 0.2, {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", cd ? fsw : 2.0 * fsw, iShare + 0.5 * dILo,
                                    std::sqrt(iShare * iShare + dILo * dILo / 12.0), iShare, dILo, dutyOn,
                                    std::max(vLoOn, vLoOff),
                                    std::sqrt(dutyOn * vLoOn * vLoOn + (1.0 - dutyOn) * vLoOff * vLoOff),
                                    0.0, vLoOn + vLoOff)});
        return m; };

    auto cap = [](double c, double v){ json j; j["capacitor"]=json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"]=c;
        j["inputs"]["designRequirements"]["ratedVoltage"]=v; return j; };
    // Numerical node caps (ABT #96): an ideal switch node with both switches off has nothing but the primary
    // leakage on it; the cap gives it a finite dV/dt. Energy-budget sized (the bridge's primary current is large
    // enough for the fixed 2.2 nF, but this keeps the turn-off ramp a negligible slice of the dead time).
    auto nodeSnub = [&]() { json c = cap(cfg::snubber_cap(d.config, d.outputPower, d.inputVoltage, fsw), 3.0 * d.inputVoltage);
        cfg::mark_numerical_aid(c); return c; };
    // Rectifier node caps: the rectifier's small solver constant, not the 2.2 nF bridge-node one. At every turn-on a
    // centre-tap node swings 2·Vs; through the transformer 2.2 nF there drew an 8–10 A primary spike (4–5× the load
    // current) that no real winding capacitance of this order produces.
    auto secSnub = [&]() { json c = cap(cfg::rectifier_snubber_cap(d.config), 3.0 * (vSecMax + Vo));
        cfg::mark_numerical_aid(c); return c; };
    // REAL series-RC damper across the primary: the tightly coupled transformer's leakage rings against the node
    // caps at every hard turn-off and nothing else damps it (ideal windings) — same rule as PSHB's Crc_pri.
    const double vPri = k * d.inputVoltageMax;
    const auto rc = req::snubber(cfg::snubber_cap(d.config, d.outputPower, vPri, fsw), cfg::snubber_res(d.config), vPri, fsw);

    std::vector<json> comps, conns;
    std::vector<json> gndEps, sgndEps;
    std::string priStart, priEnd;   // the nets the primary sits between
    std::vector<json> cports{port("vin"), port("gnd"), port("sgnd"), port("vout")};
    if (!d.fullBridge) {
        json rb; rb["resistor"] = json::object();
        rb["inputs"]["designRequirements"] = req::resistor(d.balancingResistance,
            2.0 * std::pow(0.5 * d.inputVoltageMax, 2) / d.balancingResistance, "bleed");
        const json csplit = cap(d.splitCapacitance, d.inputVoltageMax / cfg::v_derate_capacitor(d.config));
        comps = {comp("CsHi", csplit), comp("CsLo", csplit), comp("RbHi", rb), comp("RbLo", rb),
                 comp("Q1", mosfetReq), comp("Q2", mosfetReq), comp("Dq1", bodyDiode()), comp("Dq2", bodyDiode()),
                 comp("T1", xfmr), comp("Csn1", nodeSnub()), comp("Crc_pri", rc.first), comp("Rrc_pri", rc.second)};
        conns = {
            conn("vin_net", {pin("CsHi","1"), pin("RbHi","1"), pin("Q1","drain"), pin("Dq1","cathode"), prt("vin")}),
            conn("sw", {pin("Q1","source"), pin("Q2","drain"), pin("Dq1","anode"), pin("Dq2","cathode"),
                        pin("T1","primary_start"), pin("Csn1","1"), pin("Crc_pri","1")}),
            conn("mid", {pin("CsHi","2"), pin("CsLo","1"), pin("RbHi","2"), pin("RbLo","1"),
                         pin("T1","primary_end"), pin("Rrc_pri","2")}),
            conn("rc_pri_mid", {pin("Crc_pri","2"), pin("Rrc_pri","1")}),
            conn("g1_net", {pin("Q1","gate"), prt("g1")}),
            conn("g2_net", {pin("Q2","gate"), prt("g2")})};
        gndEps = {pin("CsLo","2"), pin("RbLo","2"), pin("Q2","source"), pin("Dq2","anode"), pin("Csn1","2")};
        cports.push_back(port("g1")); cports.push_back(port("g2"));
    } else {
        comps = {comp("Q1", mosfetReq), comp("Q2", mosfetReq), comp("Q3", mosfetReq), comp("Q4", mosfetReq),
                 comp("Dq1", bodyDiode()), comp("Dq2", bodyDiode()), comp("Dq3", bodyDiode()), comp("Dq4", bodyDiode()),
                 comp("T1", xfmr), comp("Csn1", nodeSnub()), comp("Csn2", nodeSnub()),
                 comp("Crc_pri", rc.first), comp("Rrc_pri", rc.second)};
        conns = {
            conn("vin_net", {pin("Q1","drain"), pin("Dq1","cathode"), pin("Q3","drain"), pin("Dq3","cathode"), prt("vin")}),
            conn("leg_a", {pin("Q1","source"), pin("Q2","drain"), pin("Dq1","anode"), pin("Dq2","cathode"),
                           pin("T1","primary_start"), pin("Csn1","1"), pin("Crc_pri","1")}),
            conn("leg_b", {pin("Q3","source"), pin("Q4","drain"), pin("Dq3","anode"), pin("Dq4","cathode"),
                           pin("T1","primary_end"), pin("Csn2","1"), pin("Rrc_pri","2")}),
            conn("rc_pri_mid", {pin("Crc_pri","2"), pin("Rrc_pri","1")}),
            conn("g1_net", {pin("Q1","gate"), prt("g1")}),
            conn("g2_net", {pin("Q2","gate"), prt("g2")}),
            conn("g3_net", {pin("Q3","gate"), prt("g3")}),
            conn("g4_net", {pin("Q4","gate"), prt("g4")})};
        gndEps = {pin("Q2","source"), pin("Dq2","anode"), pin("Q4","source"), pin("Dq4","anode"),
                  pin("Csn1","2"), pin("Csn2","2")};
        for (const char* g : {"g1", "g2", "g3", "g4"}) cports.push_back(port(g));
    }

    // --- rectifier + output inductor(s); the secondary return is its own node (ABT #778).
    switch (d.rectifierType) {
    case RectifierType::CenterTapped:
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Lout", outInductor()),
                                   comp("CsnSA", secSnub()), comp("CsnSB", secSnub())});
        conns.push_back(conn("sec_a", {pin("T1","secondary1_start"), pin("Dr1","anode"), pin("CsnSA","1")}));
        conns.push_back(conn("sec_b", {pin("T1","secondary2_end"), pin("Dr2","anode"), pin("CsnSB","1")}));
        conns.push_back(conn("out_rect", {pin("Dr1","cathode"), pin("Dr2","cathode"), pin("Lout","primary_start")}));
        conns.push_back(conn("vout_net", {pin("Lout","primary_end"), prt("vout")}));
        sgndEps = {pin("T1","secondary1_end"), pin("T1","secondary2_start"), pin("CsnSA","2"), pin("CsnSB","2")};
        break;
    case RectifierType::FullBridge:
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Dr3", diodeReq),
                                   comp("Dr4", diodeReq), comp("Lout", outInductor()),
                                   comp("CsnSA", secSnub()), comp("CsnSB", secSnub())});
        conns.push_back(conn("sec_a", {pin("T1","secondary1_start"), pin("Dr1","anode"), pin("Dr3","cathode"), pin("CsnSA","1")}));
        conns.push_back(conn("sec_b", {pin("T1","secondary1_end"), pin("Dr2","anode"), pin("Dr4","cathode"), pin("CsnSB","1")}));
        conns.push_back(conn("out_rect", {pin("Dr1","cathode"), pin("Dr2","cathode"), pin("Lout","primary_start")}));
        conns.push_back(conn("vout_net", {pin("Lout","primary_end"), prt("vout")}));
        sgndEps = {pin("Dr3","anode"), pin("Dr4","anode"), pin("CsnSA","2"), pin("CsnSB","2")};
        break;
    case RectifierType::CurrentDoubler: {
        json rlb; rlb["resistor"] = json::object();
        // The loop breaker between the two inductor outputs: no RAS role fits it, and role is optional.
        auto& rdr = rlb["inputs"]["designRequirements"];
        rdr["deviceType"] = "resistor";
        rdr["resistance"]["nominal"] = cfg::loop_breaker_res(d.config, d.loadResistance);
        rdr["powerRating"] = 0.25;
        comps.insert(comps.end(), {comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Lout", outInductor()),
                                   comp("Lo2", outInductor()), comp("Rlb", rlb),
                                   comp("CsnSA", secSnub()), comp("CsnSB", secSnub())});
        conns.push_back(conn("node_a", {pin("T1","secondary1_start"), pin("Dr1","cathode"), pin("Lout","primary_start"), pin("CsnSA","1")}));
        conns.push_back(conn("node_b", {pin("T1","secondary1_end"), pin("Dr2","cathode"), pin("Lo2","primary_start"), pin("CsnSB","1")}));
        conns.push_back(conn("lo2_out", {pin("Lo2","primary_end"), pin("Rlb","1")}));
        conns.push_back(conn("vout_net", {pin("Lout","primary_end"), pin("Rlb","2"), prt("vout")}));
        sgndEps = {pin("Dr1","anode"), pin("Dr2","anode"), pin("CsnSA","2"), pin("CsnSB","2")};
        break; }
    case RectifierType::VoltageDoubler:
        solver_rectifier(d.rectifierType);   // throws
    }
    gndEps.push_back(prt("gnd"));
    sgndEps.push_back(prt("sgnd"));
    conns.push_back(conn("gnd_net", gndEps));
    conns.push_back(conn("sgnd_net", sgndEps));

    const char* cellName = d.fullBridge ? "fullBridgeCell" : "halfBridgeCell";
    json cell; cell["name"] = d.fullBridge ? "full-bridge-cell" : "half-bridge-cell";
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;

    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", cap(d.outputCapacitance, 2.0 * Vo))});
    filt["connections"] = json::array({conn("out", {pin("Cout","1"), prt("in")}), conn("ret", {pin("Cout","2"), prt("rtn")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency; dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = fsw;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = Vo; o["regulation"] = "voltage"; dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = Vin; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }
    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        pstage(cellName, "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp(cellName, "vin")}),
        isc("GND", "externalPort", "input", {sp(cellName, "gnd")}),
        isc("SGND", "externalPort", "input", {sp(cellName, "sgnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp(cellName, "vout"), sp("filter", "in")})});
    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Symmetric PWM: the +Vb pair at phase 0, the −Vb pair half a period later, each on for D·T.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = cellName; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = fsw;
        st["waveform"]["dutyCycle"] = d.dutyCycle; st["waveform"]["phase"] = phaseDeg; return st; };
    if (d.fullBridge)
        tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q4", 0.0), stim("Q3", 180.0), stim("Q2", 180.0)});
    else
        tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    req::finalize_control_seeds(tas, d.fullBridge ? Topology::FULL_BRIDGE_CONVERTER : Topology::HALF_BRIDGE_CONVERTER);
    return tas;
}
} // namespace

HardSwitchedBridgeDesign design_half_bridge(const json& tasInputs) { return design_bridge(tasInputs, false); }
HardSwitchedBridgeDesign design_full_bridge(const json& tasInputs) { return design_bridge(tasInputs, true); }

json build_half_bridge_tas(const HardSwitchedBridgeDesign& d) {
    if (d.fullBridge) throw std::invalid_argument("build_half_bridge_tas: the design is a full bridge");
    return build_bridge_tas(d);
}
json build_full_bridge_tas(const HardSwitchedBridgeDesign& d) {
    if (!d.fullBridge) throw std::invalid_argument("build_full_bridge_tas: the design is a half bridge");
    return build_bridge_tas(d);
}

} // namespace Kirchhoff
