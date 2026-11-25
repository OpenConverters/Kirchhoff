#include "ComponentWaveforms.hpp"

#include "TasAssembler.hpp"          // tas_to_ngspice
#include "ConverterExtract.hpp"      // topology_waveforms (magnetic names + the main one)
#include "NgspiceRunner.hpp"         // run_ngspice_in_process, ngspice_in_process_available
#include "NgspiceNodes.hpp"          // shared TAS->ngspice node/name reconstruction (sanitize/node_of_pin/...)
#include "DimensionJson.hpp"         // PEAS::resolve_dimensional_values (nlohmann::json overload)
#include "processors/WaveformProcessor.h"  // shared DSP (harmonics/processed) — reused, not re-implemented

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {

namespace {

using nlohmann::json;
using WP = OpenMagnetics::WaveformProcessor;
using ngnodes::lower;
using ngnodes::sanitize;
using ngnodes::InterStage;
using ngnodes::BrickNets;
using ngnodes::read_inter_stage;
using ngnodes::read_brick_nets;
using ngnodes::node_of_pin;
using ngnodes::node_voltage;

// A component's SPICE device letter + savecurrents current key, per kind at REQUIREMENTS fidelity
// (ideal switch S, diode D, capacitor C, resistor R). Real-model (DATASHEET mosfet -> M) devices are a
// follow-up: their vector simply won't match and the component is omitted from currents (still gets V).
struct DeviceType { char letter; const char* key; };
bool device_type_of(const json& data, DeviceType& out) {
    if (data.contains("semiconductor") && data.at("semiconductor").is_object()) {
        const json& s = data.at("semiconductor");
        if (s.contains("mosfet")) { out = {'s', "i"};  return true; }   // ideal switch
        if (s.contains("diode"))  { out = {'d', "id"}; return true; }
        return false;
    }
    if (data.contains("capacitor")) { out = {'c', "i"}; return true; }
    if (data.contains("resistor"))  { out = {'r', "i"}; return true; }
    return false;   // magnetic (handled elsewhere) / controller / unknown
}

std::string kind_of(const json& data) {
    if (data.contains("semiconductor") && data.at("semiconductor").is_object()) {
        const json& s = data.at("semiconductor");
        if (s.contains("mosfet")) return "mosfet";
        if (s.contains("diode"))  return "diode";
    }
    if (data.contains("capacitor")) return "capacitor";
    if (data.contains("resistor"))  return "resistor";
    return "other";
}

// The two terminals whose node difference is the component's headline voltage, plus a label.
// (mosfet -> V_DS across drain/source; diode -> anode/cathode; 2-terminal passive -> pins 1/2.)
struct VoltagePins { std::string pos, neg, label; };
bool voltage_pins_of(const std::string& kind, VoltagePins& out) {
    if (kind == "mosfet") { out = {"drain", "source", "V_DS"}; return true; }
    if (kind == "diode")  { out = {"anode", "cathode", "V_AK"}; return true; }
    if (kind == "capacitor" || kind == "resistor") { out = {"1", "2", "V"}; return true; }
    return false;
}

// Resample a raw (time, signal) pair over the switching period [tBeg, tBeg + period) onto N=128 points at
// t∈[0,period) — the exact grid the winding extraction uses, so every waveform in the app shares one
// convention. The window is chosen once per run (resolve_window) and shared by every component and rail.
MAS::Waveform resample_period(const std::vector<double>& time, const std::vector<double>& sig,
                              double tBeg, double period) {
    const int N = 128;
    std::vector<double> data(N), t(N);
    size_t j = 0;
    for (int k = 0; k < N; ++k) {
        double tt = tBeg + period * k / N;
        while (j + 1 < time.size() && time[j + 1] < tt) ++j;
        if (j + 1 >= time.size()) j = time.size() - 2;
        double f = (time[j + 1] - time[j] > 0) ? (tt - time[j]) / (time[j + 1] - time[j]) : 0.0;
        data[k] = sig[j] + f * (sig[std::min(j + 1, sig.size() - 1)] - sig[j]);
        t[k] = period * k / N;
    }
    MAS::Waveform wf;
    wf.set_ancillary_label(MAS::WaveformLabel::CUSTOM);
    wf.set_data(data);
    wf.set_time(t);
    return wf;
}

// A resampled waveform + its processed stats, serialized to the same shape the frontend already reads
// from an excitation's current/voltage side ({waveform:{data,time}, processed:{...}}).
json signal_json(const MAS::Waveform& wf, double fsw) {
    MAS::SignalDescriptor sd;
    sd.set_waveform(wf);
    sd.set_harmonics(WP::calculate_harmonics_data(wf, fsw));
    sd.set_processed(WP::calculate_processed_data(wf, fsw));
    json j = sd;
    j.erase("harmonics");   // the frontend only needs waveform + processed; harmonics bloat the payload
    return j;
}

// The switching period every component is read over (see ComponentWaveforms.hpp for the modes).
struct Window {
    std::string at;          // resolved mode: "end" | "inductorPeak" | "linePhase"
    double start = 0.0;      // absolute run time [s]
    std::string magnetic;    // inductorPeak only
};

Window resolve_window(const json& tas, const json& spec, const NgspiceRunResult& r, double period,
                      double linePeriod /* 0 = DC input */) {
    if (!spec.is_object())
        throw std::runtime_error("component_waveforms: window must be a JSON object, got " + spec.dump());
    for (const auto& kv : spec.items())
        if (kv.key() != "at" && kv.key() != "magnetic" && kv.key() != "linePhaseDegrees")
            throw std::runtime_error("component_waveforms: unknown window key '" + kv.key() + "'");
    std::string at = spec.value("at", std::string("auto"));
    if (at == "auto") at = linePeriod > 0 ? "inductorPeak" : "end";
    const double tEnd = r.time.back();
    Window w;
    w.at = at;
    if (at == "end") { w.start = tEnd - period; return w; }
    if (at != "inductorPeak" && at != "linePhase")
        throw std::runtime_error("component_waveforms: unknown window 'at' value '" + at
                                 + "' (expected auto, end, inductorPeak or linePhase)");
    if (!(linePeriod > 0))
        throw std::runtime_error("component_waveforms: window '" + at + "' needs an AC-input converter "
                                 "(designRequirements.inputType acSinglePhase / acThreePhase with lineFrequency)");
    if (tEnd - r.time.front() < linePeriod * (1.0 - 1e-6))
        throw std::runtime_error("component_waveforms: the run (" + std::to_string(tEnd) + " s) is shorter than one "
                                 "line cycle (" + std::to_string(linePeriod) + " s); no line-cycle window exists");
    const double t0 = tEnd - linePeriod;   // the last simulated line cycle
    if (at == "linePhase") {
        if (!spec.contains("linePhaseDegrees") || !spec.at("linePhaseDegrees").is_number())
            throw std::runtime_error("component_waveforms: window 'linePhase' needs a numeric linePhaseDegrees");
        const double deg = spec.at("linePhaseDegrees").get<double>();
        if (!(deg >= 0.0 && deg <= 360.0))
            throw std::runtime_error("component_waveforms: linePhaseDegrees must be within 0..360, got "
                                     + std::to_string(deg));
        // The line sources are SIN(0 Vpk fline) with phase A at 0° at t = 0, so the line phase of absolute time t
        // is 360·frac(t·fline). Centre the window on the requested phase inside the last line cycle, kept whole.
        const double cyclesBefore = std::floor(t0 / linePeriod + 1e-9);
        double tc = (cyclesBefore + deg / 360.0) * linePeriod;
        if (tc < t0 - 1e-12 * linePeriod) tc += linePeriod;
        tc = std::clamp(tc, t0 + period / 2, tEnd - period / 2);
        w.start = tc - period / 2;
        return w;
    }
    // inductorPeak: the magnetic's primary winding branch current, as extract_operating_point reads it.
    const std::vector<MagneticExtract> mags = topology_waveforms(tas);
    std::string name;
    if (spec.contains("magnetic")) {
        if (!spec.at("magnetic").is_string())
            throw std::runtime_error("component_waveforms: window magnetic must be a string");
        name = spec.at("magnetic").get<std::string>();
        bool found = false;
        for (const auto& m : mags) if (m.name == name) { found = true; break; }
        if (!found) throw std::runtime_error("component_waveforms: window magnetic '" + name + "' is not in the TAS");
    } else {
        for (const auto& m : mags) if (m.isMain) { name = m.name; break; }
        if (name.empty()) throw std::runtime_error("component_waveforms: the TAS has no main magnetic");
    }
    const std::string token = "l" + lower(name) + "_pri";
    const std::vector<double>* sig = nullptr;
    for (const auto& kv : r.vectors) {
        const std::string k = lower(kv.first);
        if (k.find("#branch") != std::string::npos && k.find(token) != std::string::npos) { sig = &kv.second; break; }
    }
    if (!sig)
        throw std::runtime_error("component_waveforms: no ngspice branch matching '" + token
                                 + "' for the primary winding of magnetic '" + name + "'");
    // Largest magnitude of the one-switching-period moving average (the line envelope of the current), the
    // same criterion extract_operating_point uses for an AC-input magnetic's switching-period excitation.
    std::vector<double> cum(r.time.size(), 0.0);
    for (size_t i = 1; i < r.time.size(); ++i)
        cum[i] = cum[i - 1] + 0.5 * ((*sig)[i] + (*sig)[i - 1]) * (r.time[i] - r.time[i - 1]);
    auto integral_at = [&](double t) {
        const size_t j = static_cast<size_t>(std::upper_bound(r.time.begin(), r.time.end(), t) - r.time.begin());
        if (j == 0) return cum.front();
        if (j >= r.time.size()) return cum.back();
        const double f = (t - r.time[j - 1]) / (r.time[j] - r.time[j - 1]);
        return cum[j - 1] + f * (cum[j] - cum[j - 1]);
    };
    bool first = true;
    double best = 0.0;
    // One candidate centre per switching period across the line cycle (at least 512).
    const int steps = std::max(512, static_cast<int>(std::ceil(linePeriod / period)));
    for (int k = 0; k <= steps; ++k) {
        const double tc = t0 + period / 2 + (linePeriod - period) * k / steps;
        const double avg = std::abs(integral_at(tc + period / 2) - integral_at(tc - period / 2)) / period;
        if (first || avg > best) { best = avg; w.start = tc - period / 2; first = false; }
    }
    w.magnetic = name;
    return w;
}

double switching_frequency(const json& tas) {
    const json& dr = tas.at("inputs").at("designRequirements");
    if (dr.contains("switchingFrequency"))
        return PEAS::resolve_dimensional_values(dr.at("switchingFrequency"));
    throw std::runtime_error("component_waveforms: designRequirements.switchingFrequency missing");
}

}  // namespace

nlohmann::json component_waveforms(const json& tas, const PEAS::Fidelity& fidelity, const json& windowSpec) {
    if (!ngspice_in_process_available())
        throw std::runtime_error("component_waveforms: Kirchhoff built without libngspice");
    if (!tas.contains("topology") || !tas.at("topology").contains("stages"))
        throw std::runtime_error("component_waveforms: TAS has no topology.stages");

    const std::string deck = tas_to_ngspice(tas, fidelity);
    NgspiceRunResult r = run_ngspice_in_process(deck);
    if (!r.success)
        throw std::runtime_error("component_waveforms: ngspice run failed: " + r.error);
    if (r.time.size() < 2)
        throw std::runtime_error("component_waveforms: ngspice produced no transient data");

    const double fsw = switching_frequency(tas);
    if (!(fsw > 0)) throw std::runtime_error("component_waveforms: non-positive switching frequency");
    const double period = 1.0 / fsw;
    if (r.time.back() < period)
        throw std::runtime_error("component_waveforms: transient shorter than one switching period");

    // AC-input converters (PFC, Vienna) carry a line cycle; the window may be placed inside it.
    const json& dreq = tas.at("inputs").at("designRequirements");
    const std::string inputType = dreq.value("inputType", std::string("dc"));
    double linePeriod = 0.0;
    if (inputType == "acSinglePhase" || inputType == "acThreePhase") {
        if (!dreq.contains("lineFrequency"))
            throw std::runtime_error("component_waveforms: AC-input TAS without lineFrequency");
        linePeriod = 1.0 / PEAS::resolve_dimensional_values(dreq.at("lineFrequency"));
    }
    const Window win = resolve_window(tas, windowSpec, r, period, linePeriod);
    const double tBeg = win.start;

    const json& topo = tas.at("topology");
    const InterStage is = read_inter_stage(topo);

    json components = json::array();

    for (const auto& stage : topo.at("stages")) {
        if (!stage.contains("circuit") || !stage.at("circuit").is_object()) continue;
        const json& brick = stage.at("circuit");
        if (!brick.contains("components") || !brick.contains("connections")) continue;
        const std::string sname = stage.at("name").get<std::string>();
        const std::string inst = "x" + lower(sanitize(sname));
        const BrickNets bn = read_brick_nets(brick);

        for (const auto& comp : brick.at("components")) {
            if (!comp.contains("data") || !comp.at("data").is_object()) continue;
            const json& data = comp.at("data");
            DeviceType dt;
            if (!device_type_of(data, dt)) continue;   // magnetic / controller / unknown -> not here
            const std::string ref = comp.value("name", std::string{});
            const std::string kind = kind_of(data);

            // savecurrents token for this device's main atom. A single-atom (ideal) leaf is
            // "@<L>.<inst>.<L><ref>[<key>]"; a real multi-atom leaf suffixes the atom name
            // ("<L><ref>_q" for the switch of a mosfet = Q + Coss + body-diode). The parasitic atoms
            // sit under OTHER device letters (Coss->c, body diode->d), so within letter <L> the
            // "<L><ref>" / "<L><ref>_…" match uniquely picks the device's own conduction branch.
            const std::string base = std::string("@") + dt.letter + "." + inst + "." + dt.letter + lower(sanitize(ref));
            const std::vector<double>* cur = nullptr;
            auto exact = r.vectors.find(base + "[" + dt.key + "]");
            if (exact != r.vectors.end()) cur = &exact->second;
            else {
                const std::string pre = base + "_", suf = std::string("[") + dt.key + "]";
                for (const auto& kv : r.vectors)
                    if (kv.first.rfind(pre, 0) == 0 && kv.first.size() >= suf.size()
                        && kv.first.compare(kv.first.size() - suf.size(), suf.size(), suf) == 0) {
                        cur = &kv.second; break;
                    }
            }
            if (!cur) continue;   // stripped / not simulated -> omit (honest)

            json cj;
            cj["ref"] = ref;
            cj["stage"] = sname;
            cj["kind"] = kind;
            cj["current"] = signal_json(resample_period(r.time, *cur, tBeg, period), fsw);

            // voltage across the headline terminals, when both nodes are resolvable
            VoltagePins vp;
            if (voltage_pins_of(kind, vp)) {
                const std::string np = node_of_pin(sname, ref, vp.pos, bn, is);
                const std::string nn = node_of_pin(sname, ref, vp.neg, bn, is);
                bool okP = false, okN = false;
                std::vector<double> A = node_voltage(np, r, okP);
                std::vector<double> B = node_voltage(nn, r, okN);
                if (okP && okN) {
                    std::vector<double> vdiff(r.time.size());
                    for (size_t i = 0; i < vdiff.size(); ++i)
                        vdiff[i] = (i < A.size() ? A[i] : 0.0) - (i < B.size() ? B[i] : 0.0);
                    json vj = signal_json(resample_period(r.time, vdiff, tBeg, period), fsw);
                    vj["label"] = vp.label;
                    cj["voltage"] = std::move(vj);
                }
            }
            components.push_back(std::move(cj));
        }
    }

    // ── External rails (ABT #905 follow-up) ───────────────────────────────────────────────────
    // The converter view wants the converter's own terminals, not only the per-component overlays:
    // without these the input rail and each output rail are only visible indirectly (as the output
    // cap's voltage). The assembler names external ports Vin / Vout / Vout2… and synthesizes the
    // source as V<port> and each load as Rload / Rload<i>, so read those. A rail whose vector is not
    // in the run is OMITTED rather than faked — same rule the component loop uses.
    std::string inPort;
    std::vector<std::string> outPorts;
    for (const auto& ic : topo.value("interStageConnections", json::array())) {
        if (ic.value("kind", std::string{}) != "externalPort") continue;
        const std::string nm = ic.value("name", std::string{});
        const std::string dir = ic.value("direction", std::string{});
        if (dir == "output") outPorts.push_back(nm);
        // GND / SGND are returns, not the driven input rail.
        else if (dir == "input" && inPort.empty() && nm != "GND" && nm != "SGND") inPort = nm;
    }

    // Flat {data,time} — the shape the wizard's converter view consumes for rails (components use
    // the nested signal shape instead).
    auto rail_signal = [&](const std::vector<double>& v) {
        return json(resample_period(r.time, v, tBeg, period));
    };
    // ngspice spells a saved branch current either "@<dev>[i]" (savecurrents) or "<dev>#branch"
    // (voltage sources). Try both before giving up.
    auto branch_current = [&](const std::string& dev, bool& ok) -> const std::vector<double>* {
        ok = false;
        for (const std::string key : {"@" + dev + "[i]", dev + "#branch"}) {
            auto it = r.vectors.find(key);
            if (it != r.vectors.end()) { ok = true; return &it->second; }
        }
        return nullptr;
    };

    json wj = json{{"at", win.at}, {"start", tBeg}, {"end", tBeg + period}};
    if (linePeriod > 0) {
        wj["linePeriod"] = linePeriod;
        const double centre = tBeg + period / 2;
        wj["linePhaseDegrees"] = 360.0 * (centre / linePeriod - std::floor(centre / linePeriod));
    }
    if (!win.magnetic.empty()) wj["magnetic"] = win.magnetic;
    json out = json{
        {"engine", "ngspice"},
        {"referencePeriod", period},
        {"window", std::move(wj)},
        {"components", std::move(components)},
    };

    if (!inPort.empty()) {
        bool ok = false;
        const std::vector<double> vin = node_voltage(lower(sanitize(inPort)), r, ok);
        if (ok) out["inputVoltage"] = rail_signal(vin);
        bool iok = false;
        // The synthesized DC source is V<port> (e.g. port "Vin" -> element VVin -> vector vvin).
        // SPICE reports a source's branch current INTO its + terminal, so a supply delivering power
        // reads negative. Negate it: "Input Current" means the current the converter DRAWS.
        if (const std::vector<double>* iin = branch_current("v" + lower(sanitize(inPort)), iok)) {
            std::vector<double> drawn(iin->size());
            for (size_t k = 0; k < iin->size(); ++k) drawn[k] = -(*iin)[k];
            out["inputCurrent"] = rail_signal(drawn);
        }
    }

    json outV = json::array(), outI = json::array();
    for (size_t i = 0; i < outPorts.size(); ++i) {
        bool ok = false;
        const std::vector<double> v = node_voltage(lower(sanitize(outPorts[i])), r, ok);
        outV.push_back(ok ? rail_signal(v) : json(nullptr));
        // Loads are named bare for output 0 and numeric-suffixed after it (TasAssembler).
        const std::string load = "rload" + (i == 0 ? std::string{} : std::to_string(i));
        bool iok = false;
        const std::vector<double>* iv = branch_current(load, iok);
        outI.push_back(iok ? rail_signal(*iv) : json(nullptr));
    }
    if (!outV.empty()) out["outputVoltages"] = std::move(outV);
    if (!outI.empty()) out["outputCurrents"] = std::move(outI);

    return out;
}

}  // namespace Kirchhoff
