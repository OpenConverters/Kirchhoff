#include "TwoSwitchFlyback.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
// Default duty cap: Vor = Vin,min·D/(1−D) ≈ 0.82·Vin,min, leaving the clamp diodes reverse-biased through
// the whole reset interval with margin for the rectifier drop and efficiency.
constexpr double kDefaultMaxDuty = 0.45;
} // namespace

TwoSwitchFlybackDesign design_two_switch_flyback(const json& tasInputs) {
    json spec = tasInputs;
    json config = cfg::object_of(spec);
    if (config.contains("maxDutyCycle")) {
        const double dMax = config.at("maxDutyCycle").get<double>();
        if (!(dMax < 0.5))
            throw std::invalid_argument("design_two_switch_flyback: config.maxDutyCycle must be < 0.5 (at 0.5 the "
                "reflected voltage reaches Vin,min and the clamp diodes return the stored energy to the input), got "
                + std::to_string(dMax));
    } else {
        config["maxDutyCycle"] = kDefaultMaxDuty;
    }
    const std::string mode = cfg::get_str(config, "mode", "ccm");
    if (mode == "qrm")
        throw std::invalid_argument("design_two_switch_flyback: quasi-resonant valley switching is not modelled "
                                    "for the clamped two-switch cell (use ccm, dcm or bcm)");
    spec["config"] = config;
    TwoSwitchFlybackDesign d = design_flyback(spec);
    for (std::size_t i = 0; i < d.outputs.size(); ++i) {
        const double vor = d.outputs[i].turnsRatio * (d.outputs[i].voltage + d.outputs[i].diodeDrop);
        if (vor >= d.inputVoltageMin)
            throw std::invalid_argument("design_two_switch_flyback: output " + std::to_string(i)
                + " reflects Vor = " + std::to_string(vor) + " V, at or above Vin,min = "
                + std::to_string(d.inputVoltageMin) + " V — the clamp diodes would conduct through the reset "
                "(lower the turns ratio or maxDutyCycle)");
    }
    return d;
}

json build_two_switch_flyback_tas(const TwoSwitchFlybackDesign& d) {
    // The transformer, rectifiers, filters, magnetic excitation and stimulus timing are the flyback's: the
    // two-switch cell changes only what drives the primary. Build the flyback, then replace its switch stage.
    json tas = build_flyback_tas(d);
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto sp   = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };

    json* inv = nullptr;
    for (auto& st : tas.at("topology").at("stages")) if (st.at("name") == "inverter") inv = &st;
    if (!inv) throw std::logic_error("build_two_switch_flyback_tas: the flyback TAS has no 'inverter' stage");
    json brick = inv->at("circuit");
    json q1, cin, cclmp, rclmp;
    for (const auto& c : brick.at("components")) {
        if (c.at("name") == "Q1") q1 = c.at("data");
        if (c.at("name") == "Cin") cin = c.at("data");
        if (c.at("name") == "Cclmp") cclmp = c.at("data");
        if (c.at("name") == "Rclmp") rclmp = c.at("data");
        if (c.at("name") == "Cres")
            throw std::logic_error("build_two_switch_flyback_tas: a resonant drain capacitor is not modelled here");
    }
    if (q1.is_null() || cin.is_null() || cclmp.is_null() || rclmp.is_null())
        throw std::logic_error("build_two_switch_flyback_tas: the flyback switch stage lacks Q1, Cin or its RC clamp");

    // Each switch, and each clamp diode, blocks Vin. (The clamps are Dc1/Dc2: D1 is the output rectifier.)
    const double ipk = q1.at("inputs").at("designRequirements").at("ratedContinuousDrainCurrent").get<double>();
    json sw = q1;
    sw["inputs"]["designRequirements"]["ratedDrainSourceVoltage"] = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    json clamp; clamp["semiconductor"]["diode"] = json::object();
    const double ratedVr = d.inputVoltageMax / cfg::v_derate_diode(d.config);
    // Rated for the full primary peak: at turn-off the primary current commutates into Dc1/Dc2 at its peak
    // value and decays as the leakage energy returns to Vin.
    clamp["inputs"]["designRequirements"] = req::diode(ratedVr, ipk, (ratedVr < 100.0) ? 0.6 : 1.2,
                                                       0.05 / d.switchingFrequency);

    brick["name"] = "two-switch-primary";
    brick["ports"] = json::array({port("dc+"), port("hi"), port("sw"), port("gate"), port("gate2"), port("dc-")});
    // Commutation capacitance across each switch (drain-source, where the real part's Coss sits). The
    // single-switch stage's RC clamp kept the primary nodes capacitive; without it the switched nodes have none,
    // and at turn-off ngspice must hand the primary current to Dc1/Dc2 in zero time ("timestep too small").
    // The value is the flyback's documented drain-node capacitance (Coss + winding, config resonantCapacitance),
    // NOT the bridge family's 2.2 nF node constant: a flyback primary carries a fraction of an ampere, and the
    // turn-off ramp through Vin + Vor on 2.2 nF took a tenth of the period at 375 V, adding enough volt-seconds
    // to lift the open-loop output 18 %. Tagged as numerical aids: the real-fidelity deck strips them for the
    // switches' own capacitance.
    json snb; snb["capacitor"] = json::object();
    snb["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::qr_resonant_cap(d.config);
    snb["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltageMax / cfg::v_derate_capacitor(d.config);
    cfg::mark_numerical_aid(snb);

    // The flyback's RC damper stays, now across the primary itself (hi to sw): the clamp diodes return the
    // leakage energy to Vin, but once they stop conducting the leakage inductance rings against the switches'
    // capacitance, and without damping that ring drives ngspice to "timestep too small" at 375 V. Same part,
    // same energy-budget sizing as the single-switch stage.
    brick["components"] = json::array({comp("Q1", sw), comp("Q2", sw), comp("Dc1", clamp), comp("Dc2", clamp),
                                       comp("Cin", cin), comp("Csn1", snb), comp("Csn2", snb),
                                       comp("Cclmp", cclmp), comp("Rclmp", rclmp)});
    brick["connections"] = json::array({
        conn("dc_pos", {pin("Cin", "1"), pin("Q1", "drain"), pin("Dc2", "cathode"), pin("Csn1", "1"), prt("dc+")}),
        conn("hi",     {pin("Q1", "source"), pin("Dc1", "cathode"), pin("Csn1", "2"), pin("Cclmp", "1"), prt("hi")}),
        conn("sw",     {pin("Q2", "drain"), pin("Dc2", "anode"), pin("Csn2", "1"), pin("Rclmp", "2"), prt("sw")}),
        conn("clmp_mid", {pin("Cclmp", "2"), pin("Rclmp", "1")}),
        conn("gate",   {pin("Q1", "gate"), prt("gate")}),
        conn("gate2",  {pin("Q2", "gate"), prt("gate2")}),
        conn("dc_neg", {pin("Q2", "source"), pin("Dc1", "anode"), pin("Csn2", "2"), pin("Cin", "2"), prt("dc-")})});
    (*inv)["circuit"] = brick;

    // The primary's dotted start now hangs off Q1's source, not straight off the Vin rail.
    bool rewired = false;
    for (auto& isc : tas.at("topology").at("interStageConnections")) {
        if (isc.at("name") != "Vin") continue;
        json eps = json::array();
        for (const auto& e : isc.at("endpoints"))
            if (!(e.value("stage", "") == "transformer" && e.value("port", "") == "pri")) eps.push_back(e);
            else rewired = true;
        isc["endpoints"] = eps;
    }
    if (!rewired) throw std::logic_error("build_two_switch_flyback_tas: the flyback Vin rail does not feed transformer.pri");
    json pri; pri["name"] = "pri_hi"; pri["kind"] = "wire";
    pri["endpoints"] = json::array({sp("inverter", "hi"), sp("transformer", "pri")});
    tas["topology"]["interStageConnections"].push_back(pri);

    // Both switches are gated by the same PWM (Q1's floating, Q2's ground-referenced).
    json q2stim;
    for (const auto& s : tas.at("simulation").at("stimulus"))
        if (s.value("stage", "") == "inverter" && s.value("component", "") == "Q1") q2stim = s;
    if (q2stim.is_null()) throw std::logic_error("build_two_switch_flyback_tas: no Q1 gate stimulus to mirror");
    q2stim["component"] = "Q2";
    tas["simulation"]["stimulus"].push_back(q2stim);

    req::finalize_control_seeds(tas, Topology::TWO_SWITCH_FLYBACK_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
