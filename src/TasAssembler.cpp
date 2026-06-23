#include "TasAssembler.hpp"
#include "RasConverter.hpp"
#include "CasConverter.hpp"
#include "SasConverter.hpp"
#include "AasConverter.hpp"
#include "CtasConverter.hpp"
#include "MasConverter.hpp"
#include "CiasConverter.hpp"
#include "CiasCircuitConverter.hpp"

#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace Kirchhoff {

using nlohmann::json;

namespace {

bool contains_ci(const std::string& s, const std::string& sub) {
    std::string a = s, b = sub;
    std::transform(a.begin(), a.end(), a.begin(), ::tolower);
    std::transform(b.begin(), b.end(), b.begin(), ::tolower);
    return a.find(b) != std::string::npos;
}

// Dispatch a PEAS component to its family to_cias generator (decision 5: the orchestrator expands).
// AAS (analog) and CTAS (controller) now have real converter libs, exactly like RAS/CAS/SAS/MAS.
json component_to_leaf(const json& data, const PEAS::Fidelity& f) {
    if (data.contains("resistor"))      return RAS::ras_to_cias(data, f);
    if (data.contains("capacitor"))     return CAS::cas_to_cias(data, f);
    if (data.contains("semiconductor")) return SAS::sas_to_cias(data, f);
    if (data.contains("magnetic"))      return MAS::mas_to_cias(data, f);
    if (data.contains("analog"))        return AAS::aas_to_cias(data, f);    // AAS analog block
    if (data.contains("controller"))    return CTAS::ctas_to_cias(data, f);  // CTAS controller
    throw std::runtime_error("TasAssembler: component data has no PEAS discriminator "
                             "(resistor/capacitor/semiconductor/magnetic/analog/controller)");
}

double op_input_voltage(const json& inputs) {
    if (inputs.contains("operatingPoints") && !inputs.at("operatingPoints").empty())
        return inputs.at("operatingPoints").at(0).at("inputVoltage").get<double>();
    const json& iv = inputs.at("designRequirements").at("inputVoltage");
    if (iv.is_number()) return iv.get<double>();
    if (iv.contains("nominal")) return iv.at("nominal").get<double>();
    throw std::runtime_error("TasAssembler: no input voltage");
}

double num_or_nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.is_object() && j.contains("nominal")) return j.at("nominal").get<double>();
    throw std::runtime_error("TasAssembler: expected number or {nominal}");
}

// The load is NOT a converter component — it is the boundary condition the converter drives,
// the dual of the input source. Synthesize the matching ngspice element from the per-operating-point
// output condition (TAS inputs.operatingPoints[k].outputs[i]): magnitude = current|power, behaviour =
// loadType (default 'resistive'). Vout = the OP voltage setpoint or the designRequirements nominal.
std::string emit_load_card(const std::string& node, const json& inputs, size_t outputIndex) {
    const json& dr = inputs.at("designRequirements");
    double vout = num_or_nominal(dr.at("outputs").at(outputIndex).at("voltage"));

    std::string loadType = "resistive";
    bool hasCurrent = false, hasPower = false;
    double current = 0, power = 0;
    if (inputs.contains("operatingPoints") && !inputs.at("operatingPoints").empty()) {
        const json& opOut = inputs.at("operatingPoints").at(0).at("outputs").at(outputIndex);
        loadType = opOut.value("loadType", "resistive");
        if (opOut.contains("voltage")) vout = opOut.at("voltage").get<double>();
        if (opOut.contains("current")) { current = opOut.at("current").get<double>(); hasCurrent = true; }
        if (opOut.contains("power"))   { power   = opOut.at("power").get<double>();   hasPower = true; }
    }
    if (!hasCurrent && !hasPower)
        throw std::runtime_error("TasAssembler: output " + std::to_string(outputIndex) +
                                 " has neither current nor power to size the load");
    // Fill in the missing magnitude from Vout (schema guarantees exactly one of current/power).
    const double P = hasPower ? power : vout * current;
    const double I = hasCurrent ? current : (vout != 0 ? power / vout : 0);

    std::ostringstream c;
    c.precision(10);
    if (loadType == "resistive") {
        if (P <= 0 || vout == 0)
            throw std::runtime_error("TasAssembler: cannot size a resistive load (need Vout>0 and power>0)");
        c << "Rload " << node << " 0 " << (vout * vout / P) << "\n";
    } else if (loadType == "constantCurrent") {
        // Ideal current sink: I flows node -> 0 through the source, i.e. drawn out of the output.
        c << "Iload " << node << " 0 DC " << I << "\n";
    } else if (loadType == "constantPower") {
        // CPL — behavioural sink drawing constant power P (negative incremental resistance). A naive
        // i = P/v pins the output at 0 forever at startup (v->0 => ~unbounded sink). A resistive soft
        // knee instead becomes a near-short at low v and traps a weak source at a spurious low
        // equilibrium. Use a CURRENT-LIMITED CPL: i = P/max(v, Vk). Below Vk it draws a constant Imax
        // = P/Vk (not a short, so the output can climb through startup); above Vk it is exact P/v —
        // and Vk = Vout/2 keeps the operating point (v ~ Vout) firmly in the true constant-power region.
        if (P <= 0 || vout <= 0)
            throw std::runtime_error("TasAssembler: cannot size a constant-power load (need Vout>0 and power>0)");
        const double vKnee = 0.5 * vout;   // Imax = P/Vk = 2*Iop; constant power for v > Vout/2
        c << "Bload " << node << " 0 I = " << P << " / max(V(" << node << "), " << vKnee << ")\n";
    } else {
        throw std::runtime_error("TasAssembler: loadType '" + loadType + "' is not supported by the "
                                 "ngspice emitter yet (battery / converter loads need extra parameters)");
    }
    return c.str();
}

// Fidelity inferred PER COMPONENT from its data: a BOUND part (real architecture / manufacturerInfo
// from the librarian's selection) -> DATASHEET (real device model); a pre-sourcing SEED (empty part +
// requirements) -> REQUIREMENTS (ideal). This lets the SAME TAS get progressively more real as parts
// are bound (the Heaviside librarian round-trip), and mixed (real magnetic + ideal passives) decks.
PEAS::Fidelity infer_fidelity(const json& data, const PEAS::Fidelity& base) {
    auto bound = [](const json& obj) {
        return obj.is_object() && (obj.contains("manufacturerInfo") || obj.contains("core") || obj.contains("coil"));
    };
    bool real = false;
    if (data.contains("semiconductor") && data.at("semiconductor").is_object()) {
        const json& s = data.at("semiconductor");
        for (const char* k : {"mosfet", "diode", "igbt", "bjt"})
            if (s.contains(k) && bound(s.at(k))) real = true;
    } else if (data.contains("capacitor")) real = bound(data.at("capacitor"));
    else if (data.contains("resistor"))    real = bound(data.at("resistor"));
    else if (data.contains("magnetic"))    real = bound(data.at("magnetic"));
    PEAS::Fidelity f = base;
    f.origin = real ? PEAS::Fidelity::Origin::DATASHEET : PEAS::Fidelity::Origin::REQUIREMENTS;
    // A magnetic carrying an MKF-exported subcircuit uses the MKF_MODEL origin specifically (the real
    // magnetic path), as opposed to the DATASHEET (datasheet-parasitics) path used by other parts.
    if (data.contains("magnetic") && data.at("magnetic").is_object()
        && data.at("magnetic").contains("modelOutputs")
        && data.at("magnetic").at("modelOutputs").is_object()
        && data.at("magnetic").at("modelOutputs").contains("spiceSubcircuit"))
        f.origin = PEAS::Fidelity::Origin::MKF_MODEL;
    return f;
}

// ngspice node/identifier-safe form of an arbitrary TAS port/stage name.
std::string sanitize(const std::string& s) {
    std::string r;
    for (char c : s) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') r += c;
        else if (c == '+') r += 'p';
        else if (c == '-') r += 'n';
        else r += '_';
    }
    return r;
}

} // namespace

static std::string tas_to_spice(const json& tasDoc, const PEAS::Fidelity& fidelity,
                                CIAS::SpiceDialect dialect) {
    const bool lt = (dialect == CIAS::SpiceDialect::Ltspice);
    const json& topo = tasDoc.at("topology");
    const json& stages = topo.at("stages");
    const json interStage = topo.value("interStageConnections", json::array());
    const json sim = tasDoc.value("simulation", json::object());

    // --- 1. inter-stage groups: (stage,port) -> group name; group -> {kind,direction,nodeName} ---
    std::map<std::pair<std::string, std::string>, std::string> groupOf;
    std::map<std::string, std::string> groupKind, groupDir;
    for (const auto& ic : interStage) {
        const std::string name = ic.at("name").get<std::string>();
        groupKind[name] = ic.value("kind", "wire");
        groupDir[name] = ic.value("direction", "");
        for (const auto& ep : ic.at("endpoints"))
            groupOf[{ep.at("stage").get<std::string>(), ep.at("port").get<std::string>()}] = name;
    }
    auto group_node = [&](const std::string& g) -> std::string {
        if (groupKind[g] == "externalPort" && contains_ci(g, "gnd")) return "0";
        return g;
    };

    // The external node a stage port resolves to: the inter-stage group node if the port is
    // grouped, else a private net "<stage>__<port>" (e.g. a gate driven only by the stimulus).
    auto node_for_stage_port = [&](const std::string& stage, const std::string& port) -> std::string {
        auto it = groupOf.find({stage, port});
        if (it != groupOf.end()) return group_node(it->second);
        return stage + "__" + port;
    };

    // --- 2. each stage -> its OWN .subckt (CIAS brick), instantiated with an X line ---
    // Each stage in topology.stages[] is an independent CIAS brick; we keep that structure in the
    // netlist (one subcircuit per stage) instead of flattening every atom into one circuit. The
    // subcircuit's formal nodes are the stage's ports (in order); the X line wires them to the
    // global nodes from the inter-stage connections.
    CIAS::CiasToNgspiceConverter conv;
    std::ostringstream subckts, instances;
    // Real-magnetic ngspice subcircuits (MKF export) are GLOBAL defs hoisted to the deck top level
    // (deduplicated by reference); stage bodies only instantiate them via an X line. They can't be
    // inlined in a stage brick — the brick serializer mangles nested .subckt/.param/.ends.
    std::ostringstream magSubckts;
    std::set<std::string> seenMagRefs;
    // (stage,component,pin) -> the stage port it is exposed on (for the stimulus to reach it).
    std::map<std::tuple<std::string, std::string, std::string>, std::string> pinPort;

    for (const auto& stage : stages) {
        if (!stage.contains("circuit") || !stage.at("circuit").is_object()) continue;  // control stages
        const std::string sname = stage.at("name").get<std::string>();
        const json& brick = stage.at("circuit");

        // Build the per-stage CIAS atom-brick (ports + expanded atoms + local wiring), sanitizing
        // port names to ngspice-safe identifiers.
        json sub;
        sub["name"] = sanitize(sname);
        sub["ports"] = json::array();
        for (const auto& p : brick.at("ports"))
            sub["ports"].push_back(json{{"name", sanitize(p.at("name").get<std::string>())}});

        sub["components"] = json::array();
        sub["connections"] = json::array();
        // Expand each component to its (possibly multi-atom) leaf and INLINE it: the leaf's ports ARE
        // the component's pins; its internal nets become "<comp>__<net>"; its atoms become "<comp>"
        // (single-atom, deck unchanged) or "<comp>_<atom>" (multi-atom equivalent circuit, e.g. a real
        // capacitor = C + series ESR). Fidelity is inferred per component (bound part -> real model).
        // leaf port -> the atom pin(s) it exposes. A vector (not a single pin) so one leaf port can fan
        // out to several atom pins — e.g. a controller leaf whose `nodeC` sense port feeds two
        // comparators. Single-atom leaves (every passive/switch) map a port to exactly one pin.
        std::map<std::string, std::map<std::string, std::vector<std::pair<std::string, std::string>>>> portToAtomPin;
        for (const auto& comp : brick.at("components")) {
            const std::string cname = comp.at("name").get<std::string>();
            const json& data = comp.at("data");
            // Hoist a real magnetic's MKF subcircuit to the deck top level (once per reference); the
            // stage body (below) only emits the X instance that references it.
            if (data.contains("magnetic") && data.at("magnetic").is_object()
                && data.at("magnetic").contains("modelOutputs")
                && data.at("magnetic").at("modelOutputs").is_object()
                && data.at("magnetic").at("modelOutputs").contains("spiceSubcircuit")) {
                const json& sk = data.at("magnetic").at("modelOutputs").at("spiceSubcircuit");
                if (seenMagRefs.insert(sk.at("reference").get<std::string>()).second) {
                    std::string text = sk.at("text").get<std::string>();
                    magSubckts << text;
                    if (text.empty() || text.back() != '\n') magSubckts << "\n";
                    magSubckts << "\n";
                }
            }
            json leaf = component_to_leaf(data, infer_fidelity(data, fidelity));
            const bool single = leaf.at("components").size() == 1;
            auto aname = [&](const std::string& lc) { return single ? cname : cname + "_" + lc; };
            for (const auto& lc : leaf.at("components"))
                sub["components"].push_back(json{{"name", aname(lc.at("name").get<std::string>())},
                                                 {"data", lc.at("data")}});
            for (const auto& lconn : leaf.at("connections")) {
                std::string lport;
                std::vector<std::pair<std::string, std::string>> pins;
                for (const auto& ep : lconn.at("endpoints")) {
                    if (ep.contains("port")) lport = ep.at("port").get<std::string>();
                    else if (ep.contains("component"))
                        pins.push_back({aname(ep.at("component").get<std::string>()),
                                        ep.at("pin").get<std::string>()});
                }
                if (!lport.empty() && !pins.empty()) {
                    portToAtomPin[cname][lport] = pins;            // leaf port -> all atom pins it exposes
                } else {                                            // internal net of the equivalent circuit
                    json ic; ic["name"] = cname + "__" + lconn.at("name").get<std::string>();
                    ic["endpoints"] = json::array();
                    for (const auto& pr : pins)
                        ic["endpoints"].push_back(json{{"component", pr.first}, {"pin", pr.second}});
                    sub["connections"].push_back(ic);
                }
            }
        }

        // The stage brick's connections, each (component,pin) remapped to the real atom+pin it became.
        for (const auto& conn : brick.at("connections")) {
            std::string exposedPort;
            for (const auto& ep : conn.at("endpoints"))
                if (ep.contains("port")) exposedPort = ep.at("port").get<std::string>();

            json c;
            c["name"] = conn.at("name").get<std::string>();
            c["endpoints"] = json::array();
            for (const auto& ep : conn.at("endpoints")) {
                if (ep.contains("component")) {
                    const std::string cn = ep.at("component").get<std::string>();
                    const std::string pn = ep.at("pin").get<std::string>();
                    auto cit = portToAtomPin.find(cn);
                    if (cit == portToAtomPin.end() || !cit->second.count(pn))
                        throw std::runtime_error("TasAssembler: pin '" + pn + "' of " + sname + "." + cn +
                                                 " is not exposed by its component leaf");
                    for (const auto& ap : cit->second.at(pn))   // fan-out: one brick pin -> ≥1 atom pin
                        c["endpoints"].push_back(json{{"component", ap.first}, {"pin", ap.second}});
                    if (!exposedPort.empty()) pinPort[{sname, cn, pn}] = exposedPort;
                } else if (ep.contains("port")) {
                    c["endpoints"].push_back(json{{"port", sanitize(ep.at("port").get<std::string>())}});
                }
            }
            sub["connections"].push_back(c);
        }

        subckts << conv.to_subckt(CIAS::CiasCircuit::from_json(sub), dialect) << "\n";

        // instance: X<stage> <node per port, in declaration order> <subcktName>
        instances << "X" << sanitize(sname);
        for (const auto& p : brick.at("ports"))
            instances << " " << node_for_stage_port(sname, p.at("name").get<std::string>());
        instances << " " << sanitize(sname) << "\n";
    }

    // --- 3. testbench: subckt defs + instances + input source(s) + load + stimulus + analysis ---
    std::ostringstream os;
    os.precision(10);
    os << "* Kirchhoff TAS-assembled deck (" << (fidelity.is_ideal() ? "ideal" : "real")
       << ", one subcircuit per stage)\n";
    os << magSubckts.str();   // global real-magnetic subcircuit defs (hoisted, deduped)
    os << subckts.str();
    os << instances.str();

    const double vin = op_input_voltage(tasDoc.at("inputs"));
    const json& dreq = tasDoc.at("inputs").at("designRequirements");
    const std::string inputType = dreq.value("inputType", "dc");
    std::string outputNode;
    std::vector<std::string> acInputNodes;   // collected for a single floating AC source (acSinglePhase)
    for (const auto& [g, kind] : groupKind) {
        if (kind != "externalPort") continue;
        const std::string node = group_node(g);
        if (node == "0") continue;
        if (groupDir[g] == "input") {
            if (inputType == "acSinglePhase" || inputType == "acThreePhase")
                acInputNodes.push_back(node);   // emit AC source(s) below
            else os << "V" << g << " " << node << " 0 DC " << vin << "\n";
        }
        if (groupDir[g] == "output") {
            outputNode = node;
            // Synthesize the load from the per-OP outputs condition (boundary condition, not a
            // converter stage — the dual of the input source above). loadType picks R / I-sink / CPL.
            os << emit_load_card(node, tasDoc.at("inputs"), 0);
        }
    }
    // Single-phase AC input: ONE sinusoidal source. `inputVoltage` is RMS line voltage (schema), so the
    // SIN amplitude is vin·√2; lineFrequency is required for AC. The source FLOATS across the two input
    // terminals (line, neutral) feeding a diode bridge whose DC return is ground — exactly the
    // non-isolated boost-PFC front end. (Falls back to source-to-ground if only one input node exists.)
    if (inputType == "acSinglePhase") {
        if (!dreq.contains("lineFrequency"))
            throw std::runtime_error("TasAssembler: acSinglePhase input requires designRequirements.lineFrequency");
        const json& lf = dreq.at("lineFrequency");
        const double lineFreq = lf.is_number() ? lf.get<double>() : lf.at("nominal").get<double>();
        const double vpeak = vin * std::sqrt(2.0);
        const std::string nL = acInputNodes.empty() ? "acLine" : acInputNodes[0];
        const std::string nN = acInputNodes.size() >= 2 ? acInputNodes[1] : std::string("0");
        os << "Vac " << nL << " " << nN << " SIN(0 " << vpeak << " " << lineFreq << ")\n";
    }
    // Three-phase AC input: THREE sinusoidal sources, 120° apart, each from a phase node to ground
    // (neutral = ground = the converter's DC midpoint — a 4-wire connection that keeps every node
    // referenced, vs a floating star). `inputVoltage` is the per-phase (line-to-neutral) RMS, so each
    // amplitude is vin·√2. Phase nodes are taken in sorted group order as A, B, C (0°, −120°, −240°).
    if (inputType == "acThreePhase") {
        if (!dreq.contains("lineFrequency"))
            throw std::runtime_error("TasAssembler: acThreePhase input requires designRequirements.lineFrequency");
        if (acInputNodes.size() < 3)
            throw std::runtime_error("TasAssembler: acThreePhase input needs 3 phase nodes (got "
                                     + std::to_string(acInputNodes.size()) + ")");
        const json& lf = dreq.at("lineFrequency");
        const double lineFreq = lf.is_number() ? lf.get<double>() : lf.at("nominal").get<double>();
        const double vpeak = vin * std::sqrt(2.0);
        const char* names[3] = {"Vpha", "Vphb", "Vphc"};
        const double phase[3] = {0.0, -120.0, -240.0};
        for (int i = 0; i < 3; ++i)
            os << names[i] << " " << acInputNodes[i] << " 0 SIN(0 " << vpeak << " " << lineFreq
               << " 0 0 " << phase[i] << ")\n";
    }

    double fsw = 100000, dmax = 0.5;
    for (const auto& st : sim.value("stimulus", json::array())) {
        const json& wf = st.at("waveform");
        if (wf.value("type", "") != "pwm") continue;
        const std::string stage = st.at("stage").get<std::string>();
        const std::string comp = st.at("component").get<std::string>();
        const std::string sig = st.value("signal", "gate");
        auto it = pinPort.find({stage, comp, sig});
        if (it == pinPort.end())
            throw std::runtime_error("TasAssembler: stimulus target " + stage + "." + comp +
                                     "." + sig + " is not exposed on a stage port");
        const std::string stimNode = node_for_stage_port(stage, it->second);
        fsw = wf.value("frequency", 100000.0);
        const double duty = wf.value("dutyCycle", 0.5);
        const double period = 1.0 / fsw, ton = duty * period;
        // Optional phase shift (degrees) -> PULSE delay TD = phaseDeg/360 * period. Enables
        // interleaved / phase-shifted multi-switch drives (push-pull 180 deg, bridges, etc.).
        const double phaseDeg = wf.value("phaseDeg", 0.0);
        const double td = (phaseDeg / 360.0) * period;
        os << "Vstim_" << stage << "_" << comp << " " << stimNode << " 0 PULSE(0 5 "
           << td << " 1n 1n " << ton << " " << period << ")\n";
    }

    // transient analysis
    double stopTime = 0, maxStep = 0;
    for (const auto& an : sim.value("analyses", json::array())) {
        if (an.value("type", "") == "transient") {
            stopTime = an.value("stopTime", 0.0);
            maxStep = an.value("maximumTimeStep", 0.0);
        }
    }
    const double period = 1.0 / fsw;
    if (stopTime <= 0) stopTime = 600 * period;
    if (maxStep <= 0) maxStep = period / 200.0;

    // Optional initial conditions: pre-charge nodes at t=0 (e.g. a resonant converter's output cap)
    // and run the transient with use-initial-conditions (UIC) so ngspice SKIPS the DC operating point.
    // Two things this unblocks: a resonant tank whose series caps make the DC operating point singular
    // (UIC steps over it), and an active synchronous rectifier that can't self-start into a 0 V output
    // (the precharge gives it a rail to rectify into). The IC node names are inter-stage group /
    // external-port names, mapped to their top-level deck node here (so the in-subckt output cap, whose
    // terminal IS the external port node, is initialised correctly). Simulator-agnostic in TAS; this is
    // just the ngspice realisation.
    const json initialConditions = sim.value("initialConditions", json::array());
    const bool useIc = !initialConditions.empty();
    for (const auto& ic : initialConditions)
        os << ".ic v(" << group_node(ic.at("node").get<std::string>()) << ")="
           << ic.at("voltage").get<double>() << "\n";

    // Gear integration + tight tolerances tame the stiff ideal diodes; both ngspice and LTspice accept
    // these option names + the Gear method.
    os << ".options reltol=1e-3 abstol=1e-9 vntol=1e-6 method=gear\n";
    os << ".tran " << maxStep << " " << stopTime << " 0 " << maxStep << (useIc ? " uic" : "") << "\n";
    const double from = stopTime - 50 * period;
    if (lt) {
        // LTspice (-b batch) auto-runs the .tran and evaluates deck-level .meas directives — no ngspice
        // .control/interactive block. The same measurement, in LTspice's .meas dialect.
        if (!outputNode.empty())
            os << ".meas tran vout AVG V(" << outputNode << ") from=" << from << " to=" << stopTime << "\n";
        os << ".end\n";
    } else {
        os << ".control\nrun\n";
        if (!outputNode.empty()) {
            os << "meas tran vout AVG v(" << outputNode << ") from=" << from << " to=" << stopTime << "\n";
            os << "print vout\n";
        }
        os << ".endc\n.end\n";
    }
    return os.str();
}

std::string tas_to_ngspice(const json& tasDoc, const PEAS::Fidelity& fidelity) {
    return tas_to_spice(tasDoc, fidelity, CIAS::SpiceDialect::Ngspice);
}

// Second backend: the SAME assembled circuit rendered in the LTspice dialect. Proves the CIAS→backend
// boundary is simulator-agnostic, not ngspice-shaped (see tests/test_ltspice_backend.cpp).
std::string tas_to_ltspice(const json& tasDoc, const PEAS::Fidelity& fidelity) {
    return tas_to_spice(tasDoc, fidelity, CIAS::SpiceDialect::Ltspice);
}

} // namespace Kirchhoff
