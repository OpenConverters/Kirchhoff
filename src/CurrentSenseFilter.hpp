#pragma once
// Current-sense filter of the closed-loop PFC / Vienna controllers (ABT #2524).
//
// The inner current loops of the PFC and Vienna decks compare the shunt voltage V(ref) − V(sense) =
// iL·Rsense against a reference with an IDEAL comparator / latch: zero delay, infinite bandwidth. With an
// ideal inductor that is harmless. A real inductor model (an MKF-exported subcircuit) carries the winding's
// self-capacitance Cself across its terminals, and the current through the shunt is the terminal current
// i_int + Cself·dV_L/dt: at every switch edge the switch-node swing (≈ Vbus in ≈ 1 ns) drives a displacement
// current as large as the inductor current itself through Cself, so the SENSED current collapses (or jumps)
// for that nanosecond. The ideal comparator sees a band crossing, re-toggles the switch, which re-launches
// the edge, and the loop chatters at the solver's own time step: ngspice's step collapses to picoseconds at
// the first switching edge with enough current, the run accepts millions of points and ends in
// std::bad_alloc (WASM heap) / a failed run. Measured on the UPS PFC (Cself 24 pF, 9.8 A) and the drives
// Vienna (Cself 44 pF) decks; removing Cself or filtering the sense signal removes the chatter.
//
// Every real current-mode controller filters its current-sense input (an RC on the CS pin, plus leading-edge
// blanking) for exactly this reason. Kirchhoff models that filter: a first-order low-pass of time constant
// tau on the SHUNT voltage, built from the same ideal AAS blocks as the rest of the control law. The control
// law then reads V(ref) − LPF[V(ref) − V(sense)] wherever it read V(sense), so every law (hysteretic,
// peak-current latch, zero-current detection, average-current) sees the filtered current without being
// rewritten. The filter acts on the small shunt voltage, not on the bus-level node voltages, so the
// solver's relative tolerance applies to the current signal itself.
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
namespace req {

// tau = kCurrentSenseFilterPeriodFraction / fsw. Derivation: the filter must (a) average the self-
// capacitance edge spike, whose width Cself·Vbus/iL is ~1-2 ns for the inductors seen (24-44 pF, 9-30 A,
// 385-700 V), down to a small fraction of the hysteresis band; (b) delay the true current ramp by much less
// than the shortest on/off interval, since the band overshoot is slope·tau. One hundredth of the switching
// period does both: 154 ns at 65 kHz (the spike is attenuated ~100x, the band overshoot is a few % of the
// ripple at the line peak), the same order as the CS-pin RC + blanking of commercial CCM PFC controllers.
constexpr double kCurrentSenseFilterPeriodFraction = 0.01;

inline double current_sense_filter_time_constant(double switchingFrequency) {
    if (!(switchingFrequency > 0.0))
        throw std::invalid_argument("current_sense_filter_time_constant: switching frequency must be > 0 Hz, got "
                                    + std::to_string(switchingFrequency));
    return kCurrentSenseFilterPeriodFraction / switchingFrequency;
}

// Insert the current-sense filter into a control-stage circuit `ccell` (components + connections + ports).
// `refPort` / `sensePort` are the control cell's ports on the two ends of the shunt (V(ref) − V(sense) =
// iL·Rsense). The single connection that carries the port `sensePort` is rewired: the port leaves it (a new
// net feeds the port into the filter) and the filter output V(ref) − LPF[V(ref) − V(sense)] drives the
// control-law pins it held. Throws when the ports are not wired exactly as expected.
inline void insert_current_sense_filter(nlohmann::json& ccell, const std::string& refPort,
                                        const std::string& sensePort, double tau) {
    using nlohmann::json;
    if (!(tau > 0.0))
        throw std::invalid_argument("insert_current_sense_filter: tau must be > 0 s, got " + std::to_string(tau));
    json& conns = ccell.at("connections");
    auto carries_port = [](const json& c, const std::string& p) {
        for (const auto& e : c.at("endpoints"))
            if (e.contains("port") && e.at("port") == p) return true;
        return false;
    };
    json* senseConn = nullptr;
    json* refConn = nullptr;
    for (auto& c : conns) {
        if (carries_port(c, sensePort)) {
            if (senseConn) throw std::runtime_error("insert_current_sense_filter: port '" + sensePort
                                                    + "' is on more than one connection");
            senseConn = &c;
        }
        if (carries_port(c, refPort)) {
            if (refConn) throw std::runtime_error("insert_current_sense_filter: port '" + refPort
                                                  + "' is on more than one connection");
            refConn = &c;
        }
    }
    if (!senseConn) throw std::runtime_error("insert_current_sense_filter: no connection carries port '" + sensePort + "'");
    if (!refConn) throw std::runtime_error("insert_current_sense_filter: no connection carries port '" + refPort + "'");
    if (senseConn == refConn)
        throw std::runtime_error("insert_current_sense_filter: ports '" + refPort + "' and '" + sensePort
                                 + "' are on the same connection");

    const std::string sub = "Fsub_" + sensePort;   // shunt = V(ref) − V(sense) = iL·Rsense
    const std::string err = "Ferr_" + sensePort;   // e = shunt − y
    const std::string itg = "Fint_" + sensePort;   // y = ∫ e/tau dt  (first-order low-pass of the shunt voltage)
    const std::string out = "Fout_" + sensePort;   // V(ref) − y: the filtered stand-in for V(sense)
    auto pin = [](const std::string& c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto summer = [](double gA, double gB) {
        json j; json& b = j["analog"]["summer"]["behavioral"]; b["gainA"] = gA; b["gainB"] = gB; return j; };
    json integ; {
        json& b = integ["analog"]["integrator"]["behavioral"];
        b["gain"] = 1.0 / tau;
        b["initial"] = 0.0;      // the shunt carries no current at t = 0 (the PFC/Vienna inductors start empty)
        b["reference"] = 0.0;
    }
    auto comp = [](const std::string& n, json data) { json c; c["name"] = n; c["data"] = std::move(data); return c; };
    json& comps = ccell.at("components");
    comps.push_back(comp(sub, summer(1.0, -1.0)));
    comps.push_back(comp(err, summer(1.0, -1.0)));
    comps.push_back(comp(itg, integ));
    comps.push_back(comp(out, summer(1.0, -1.0)));

    // The control-law pins of the old sense net now read the filter output.
    json kept = json::array();
    for (const auto& e : senseConn->at("endpoints"))
        if (!(e.contains("port") && e.at("port") == sensePort)) kept.push_back(e);
    kept.push_back(pin(out, "out"));
    (*senseConn)["endpoints"] = kept;
    (*senseConn)["name"] = sensePort + "_filtered";
    (*refConn)["endpoints"].push_back(pin(sub, "inA"));
    (*refConn)["endpoints"].push_back(pin(out, "inA"));

    auto conn = [](const std::string& n, std::vector<json> eps) { json c; c["name"] = n; c["endpoints"] = eps; return c; };
    json sensePortEp; sensePortEp["port"] = sensePort;
    conns.push_back(conn(sensePort + "_sense", {sensePortEp, pin(sub, "inB")}));
    conns.push_back(conn(sensePort + "_shunt", {pin(sub, "out"), pin(err, "inA")}));
    conns.push_back(conn(sensePort + "_ferr", {pin(err, "out"), pin(itg, "in")}));
    conns.push_back(conn(sensePort + "_fshunt", {pin(itg, "out"), pin(err, "inB"), pin(out, "inB")}));
}

} // namespace req
} // namespace Kirchhoff
