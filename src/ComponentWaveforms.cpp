#include "ComponentWaveforms.hpp"

#include "TasAssembler.hpp"          // tas_to_ngspice
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

// Resample a raw (time, signal) pair over the LAST `period` onto N=128 points at t∈[0,period) — the
// exact grid the winding extraction uses, so every waveform in the app shares one convention.
MAS::Waveform resample_last_period(const std::vector<double>& time, const std::vector<double>& sig,
                                   double period) {
    const double tEnd = time.back();
    const double tBeg = tEnd - period;
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

double switching_frequency(const json& tas) {
    const json& dr = tas.at("inputs").at("designRequirements");
    if (dr.contains("switchingFrequency"))
        return PEAS::resolve_dimensional_values(dr.at("switchingFrequency"));
    throw std::runtime_error("component_waveforms: designRequirements.switchingFrequency missing");
}

}  // namespace

nlohmann::json component_waveforms(const json& tas, const PEAS::Fidelity& fidelity) {
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
            cj["current"] = signal_json(resample_last_period(r.time, *cur, period), fsw);

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
                    json vj = signal_json(resample_last_period(r.time, vdiff, period), fsw);
                    vj["label"] = vp.label;
                    cj["voltage"] = std::move(vj);
                }
            }
            components.push_back(std::move(cj));
        }
    }

    return json{
        {"engine", "ngspice"},
        {"referencePeriod", period},
        {"components", std::move(components)},
    };
}

}  // namespace Kirchhoff
