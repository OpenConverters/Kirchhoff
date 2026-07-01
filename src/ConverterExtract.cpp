#include "ConverterExtract.hpp"

#include "TasAssembler.hpp"          // tas_to_ngspice
#include "NgspiceRunner.hpp"         // run_ngspice_in_process, ngspice_in_process_available
#include "processors/WaveformProcessor.h"  // the shared DSP (sampled/harmonics/processed) — reused, not re-implemented

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace Kirchhoff {

namespace {

using nlohmann::json;

// Walk the TAS topology.stages[].circuit.components[] and collect every magnetic component (the ones the
// build carries a `data.magnetic` object + `data.inputs` for). Preserves stage/component order.
struct RawMagnetic { std::string name; const json* inputs; size_t windings; };

std::vector<RawMagnetic> raw_magnetics(const json& tas) {
    std::vector<RawMagnetic> out;
    if (!tas.contains("topology") || !tas.at("topology").contains("stages")) return out;
    for (const auto& st : tas.at("topology").at("stages")) {
        if (!st.contains("circuit") || !st.at("circuit").is_object() || !st.at("circuit").contains("components"))
            continue;
        for (const auto& c : st.at("circuit").at("components")) {
            if (!c.contains("data") || !c.at("data").is_object()) continue;
            const json& data = c.at("data");
            if (!data.contains("magnetic")) continue;                 // only magnetics
            if (!data.contains("inputs") || !data.at("inputs").is_object()) continue;
            size_t nw = 0;
            if (data.at("inputs").contains("operatingPoints") && data.at("inputs").at("operatingPoints").is_array()
                && !data.at("inputs").at("operatingPoints").empty()) {
                const json& op0 = data.at("inputs").at("operatingPoints").at(0);
                if (op0.contains("excitationsPerWinding") && op0.at("excitationsPerWinding").is_array())
                    nw = op0.at("excitationsPerWinding").size();
            }
            out.push_back({c.value("name", std::string{}), &data.at("inputs"), nw});
        }
    }
    return out;
}

// The main magnetic = the one with the most windings (the transformer for isolated topologies; the single
// inductor for non-isolated). Ties resolve to the first in stage order.
size_t main_index(const std::vector<RawMagnetic>& mags) {
    size_t best = 0, bestW = 0;
    for (size_t i = 0; i < mags.size(); ++i)
        if (mags[i].windings > bestW) { bestW = mags[i].windings; best = i; }
    return best;
}

std::string lower(std::string s) { for (char& ch : s) ch = (char)std::tolower((unsigned char)ch); return s; }

}  // namespace

std::vector<MagneticExtract> topology_waveforms(const json& tas) {
    auto mags = raw_magnetics(tas);
    if (mags.empty())
        throw std::runtime_error("topology_waveforms: TAS has no magnetic components");
    const size_t mi = main_index(mags);
    std::vector<MagneticExtract> out;
    out.reserve(mags.size());
    for (size_t i = 0; i < mags.size(); ++i) {
        MagneticExtract e;
        e.name = mags[i].name;
        e.inputs = mags[i].inputs->get<MAS::Inputs>();   // MAS quicktype from_json
        e.isMain = (i == mi);
        out.push_back(std::move(e));
    }
    return out;
}

namespace {

// ANALYTICAL: the operating point the analytical build already assembled into the selected magnetic's
// MAS::Inputs (operatingPoints[0]).
MAS::OperatingPoint analytical_operating_point_of(const std::vector<RawMagnetic>& mags, size_t idx) {
    const json& in = *mags[idx].inputs;
    if (!in.contains("operatingPoints") || !in.at("operatingPoints").is_array() || in.at("operatingPoints").empty())
        throw std::runtime_error("extract_operating_point: magnetic '" + mags[idx].name + "' has no operatingPoints");
    return in.at("operatingPoints").at(0).get<MAS::OperatingPoint>();
}

// NGSPICE: run the deck and rebuild the selected magnetic's operating point from the simulated winding
// currents/voltages. Each winding of the magnetic maps to an inductor branch in the deck; we match the
// winding to its ngspice vector by the winding name (primary/secondary/...) and rebuild each excitation
// with the shared WaveformProcessor (sampled -> harmonics -> processed), preserving the winding labels &
// structure the analytical operating point already has.
MAS::OperatingPoint ngspice_operating_point_of(const json& tas, const std::vector<RawMagnetic>& mags, size_t idx) {
    if (!ngspice_in_process_available())
        throw std::runtime_error("extract_operating_point(NGSPICE): Kirchhoff built without libngspice");
    // Start from the analytical operating point (correct winding count / labels / voltages) and overwrite
    // the currents with the simulated ones where we can find the matching branch.
    MAS::OperatingPoint op = analytical_operating_point_of(mags, idx);

    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string deck = tas_to_ngspice(tas, ideal);
    NgspiceRunResult r = run_ngspice_in_process(deck);
    if (!r.success)
        throw std::runtime_error("extract_operating_point(NGSPICE): sim failed: " + r.error);
    if (r.time.size() < 2)
        throw std::runtime_error("extract_operating_point(NGSPICE): sim produced no transient data");

    // Switching frequency for the settle window + processing comes from the magnetic's own excitation
    // (the analytical build stamped every winding with the operating frequency). No fallback: if it is
    // absent the operating point is malformed and we must not silently invent a window.
    const auto& excs0 = op.get_excitations_per_winding();
    if (excs0.empty())
        throw std::runtime_error("extract_operating_point(NGSPICE): magnetic '" + mags[idx].name
                                 + "' has no windings");
    const double fsw = excs0.front().get_frequency();
    if (!(fsw > 0))
        throw std::runtime_error("extract_operating_point(NGSPICE): magnetic '" + mags[idx].name
                                 + "' has no positive excitation frequency");
    const double period = 1.0 / fsw;
    const double tEnd = r.time.back();
    const double tBeg = std::max(0.0, tEnd - period);

    // The CIAS->ngspice serializer names each winding's inductor deterministically (CiasCircuitConverter.cpp):
    // the primary is "L<comp>_pri", secondary i is "L<comp>_sec<i>". ngspice reports each inductor's branch
    // current lowercased as "...l<comp>_<suffix>#branch". We reconstruct that exact token per winding and
    // look it up — NO name-heuristics and NO fallback: if a winding's branch is absent the extraction is
    // wrong and we throw loudly (per the no-silent-fallback rule) rather than silently keep the analytical
    // current.
    const std::string magTok = "l" + lower(mags[idx].name) + "_";
    auto& excs = op.get_mutable_excitations_per_winding();
    for (size_t w = 0; w < excs.size(); ++w) {
        const std::string token = magTok + (w == 0 ? std::string("pri") : "sec" + std::to_string(w));
        const std::vector<double>* sig = nullptr;
        for (const auto& kv : r.vectors) {
            std::string k = lower(kv.first);
            if (k.find("#branch") == std::string::npos) continue;
            if (k.find(token) != std::string::npos) { sig = &kv.second; break; }
        }
        if (!sig) {
            std::string avail;
            for (const auto& kv : r.vectors)
                if (kv.first.find("#branch") != std::string::npos) avail += " " + kv.first;
            throw std::runtime_error("extract_operating_point(NGSPICE): no ngspice branch matching '" + token
                                     + "' for winding " + std::to_string(w) + " of magnetic '" + mags[idx].name
                                     + "'. Available branches:" + avail);
        }

        const int N = 128;
        std::vector<double> data(N), time(N);
        size_t j = 0;
        for (int k = 0; k < N; ++k) {
            double t = tBeg + period * k / N;
            while (j + 1 < r.time.size() && r.time[j + 1] < t) ++j;
            double f = (r.time[j + 1] - r.time[j] > 0) ? (t - r.time[j]) / (r.time[j + 1] - r.time[j]) : 0.0;
            data[k] = (*sig)[j] + f * ((*sig)[std::min(j + 1, sig->size() - 1)] - (*sig)[j]);
            time[k] = period * k / N;
        }
        MAS::Waveform wf;
        wf.set_ancillary_label(MAS::WaveformLabel::CUSTOM);
        wf.set_data(data);
        wf.set_time(time);
        MAS::SignalDescriptor cur;
        cur.set_waveform(wf);
        cur.set_harmonics(OpenMagnetics::WaveformProcessor::calculate_harmonics_data(wf, fsw));
        cur.set_processed(OpenMagnetics::WaveformProcessor::calculate_processed_data(wf, fsw));
        excs[w].set_current(cur);
    }
    return op;
}

}  // namespace

MAS::OperatingPoint extract_operating_point(const json& tas, ExtractEngine engine, const std::string& magneticName) {
    auto mags = raw_magnetics(tas);
    if (mags.empty())
        throw std::runtime_error("extract_operating_point: TAS has no magnetic components");
    size_t idx = main_index(mags);
    if (!magneticName.empty()) {
        bool found = false;
        for (size_t i = 0; i < mags.size(); ++i)
            if (mags[i].name == magneticName) { idx = i; found = true; break; }
        if (!found) throw std::runtime_error("extract_operating_point: magnetic '" + magneticName + "' not found in TAS");
    }
    switch (engine) {
        case ExtractEngine::ANALYTICAL: return analytical_operating_point_of(mags, idx);
        case ExtractEngine::NGSPICE:    return ngspice_operating_point_of(tas, mags, idx);
    }
    throw std::runtime_error("extract_operating_point: unknown engine");
}

}  // namespace Kirchhoff
