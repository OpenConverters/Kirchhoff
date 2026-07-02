#pragma once

// Shared TAS -> ngspice node/name reconstruction. Given a TAS document and an ngspice run result, resolve
// which ngspice node vector a given (stage, component, pin) sits on, and read that node's voltage samples.
// The naming MUST match TasAssembler exactly (sanitize/node_for_stage_port) so reconstructed tokens equal
// the deck's. Both ComponentWaveforms (per-node switch/rectifier V/I) and ConverterExtract (per-winding
// node voltages) reuse this — one copy of the convention, so every waveform in the app shares one node
// naming rule.

#include "NgspiceRunner.hpp"   // NgspiceRunResult

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace Kirchhoff {
namespace ngnodes {

inline std::string lower(std::string s) {
    for (char& ch : s) ch = (char)std::tolower((unsigned char)ch);
    return s;
}

// ngspice identifier-safe form — MUST match TasAssembler::sanitize so reconstructed node/device names
// equal the deck's.
inline std::string sanitize(const std::string& s) {
    std::string r;
    for (char c : s) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') r += c;
        else if (c == '+') r += 'p';
        else if (c == '-') r += 'n';
        else r += '_';
    }
    return r;
}

inline bool contains_ci(const std::string& hay, const std::string& needle) {
    return lower(hay).find(lower(needle)) != std::string::npos;
}

// Inter-stage wiring: (stage,port) -> group name; group -> top-level node (gnd group collapses to "0").
struct InterStage {
    std::map<std::pair<std::string, std::string>, std::string> groupOf;
    std::map<std::string, std::string> groupKind;

    std::string group_node(const std::string& g) const {
        auto it = groupKind.find(g);
        if (it != groupKind.end() && it->second == "externalPort" && contains_ci(g, "gnd")) return "0";
        return g;
    }
    // The top-level node a stage port resolves to: the inter-stage group node if grouped, else the
    // private net "<stage>__<port>" (matches TasAssembler::node_for_stage_port exactly).
    std::string node_for_stage_port(const std::string& stage, const std::string& port) const {
        auto it = groupOf.find({stage, port});
        if (it != groupOf.end()) return group_node(it->second);
        return stage + "__" + port;
    }
};

inline InterStage read_inter_stage(const nlohmann::json& topo) {
    InterStage is;
    for (const auto& ic : topo.value("interStageConnections", nlohmann::json::array())) {
        const std::string name = ic.at("name").get<std::string>();
        is.groupKind[name] = ic.at("kind").get<std::string>();
        for (const auto& ep : ic.at("endpoints"))
            is.groupOf[{ep.at("stage").get<std::string>(), ep.at("port").get<std::string>()}] = name;
    }
    return is;
}

// Per stage: (component,pin) -> brick net; brick net -> the port it is exposed on (if any).
struct BrickNets {
    std::map<std::pair<std::string, std::string>, std::string> pinNet;
    std::map<std::string, std::string> netPort;
};

inline BrickNets read_brick_nets(const nlohmann::json& brick) {
    BrickNets bn;
    for (const auto& conn : brick.at("connections")) {
        const std::string net = conn.at("name").get<std::string>();
        for (const auto& ep : conn.at("endpoints")) {
            if (ep.contains("component"))
                bn.pinNet[{ep.at("component").get<std::string>(), ep.at("pin").get<std::string>()}] = net;
            else if (ep.contains("port"))
                bn.netPort[net] = ep.at("port").get<std::string>();
        }
    }
    return bn;
}

// The ngspice vector name for the node a component pin sits on (already lowercased for lookup).
// A net exposed on a stage port resolves to its top-level node; an internal net becomes the
// hierarchical "x<stage>.<net>". Returns "" if the pin isn't on any net (caller treats as unknown).
inline std::string node_of_pin(const std::string& stage, const std::string& comp, const std::string& pin,
                               const BrickNets& bn, const InterStage& is) {
    auto it = bn.pinNet.find({comp, pin});
    if (it == bn.pinNet.end()) return "";
    const std::string& net = it->second;
    auto pit = bn.netPort.find(net);
    if (pit != bn.netPort.end())
        return lower(is.node_for_stage_port(stage, pit->second));   // exposed -> top-level node
    return lower("x" + sanitize(stage) + "." + net);                // internal hierarchical node
}

// A node's voltage samples (by value): ground is identically zero; any other node is looked up by
// name. `ok` is set false when a non-ground node isn't a sim vector (a reconstruction bug — the caller
// then skips the voltage rather than inventing one).
inline std::vector<double> node_voltage(const std::string& node, const NgspiceRunResult& r, bool& ok) {
    ok = true;
    if (node == "0") return std::vector<double>(r.time.size(), 0.0);
    auto it = r.vectors.find(node);
    if (it != r.vectors.end()) return it->second;
    for (const auto& kv : r.vectors)   // defensive case-insensitive scan (ngspice already lowercases)
        if (lower(kv.first) == node) return kv.second;
    ok = false;
    return {};
}

}  // namespace ngnodes
}  // namespace Kirchhoff
