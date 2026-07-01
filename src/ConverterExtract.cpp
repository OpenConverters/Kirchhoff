#include "ConverterExtract.hpp"

#include "TasAssembler.hpp"          // tas_to_ngspice
#include "NgspiceRunner.hpp"         // run_ngspice_in_process, ngspice_in_process_available
#include "DimensionJson.hpp"         // PEAS::resolve_dimensional_values (nlohmann::json overload)
#include "processors/WaveformProcessor.h"  // the shared DSP (sampled/harmonics/processed) — reused, not re-implemented

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
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

namespace {

using nlohmann::json;

// A processed-data field of an excitation side (current/voltage), or nullopt if absent. Reads the raw TAS
// (the build embeds current.processed.{peak,rms,offset,peakToPeak,dutyCycle} + a label) rather than going
// through MAS getters — this stays a pure read over the document.
std::optional<double> processed_field(const json& exc, const char* side, const char* field) {
    if (!exc.contains(side) || !exc.at(side).is_object()) return std::nullopt;
    const json& s = exc.at(side);
    if (!s.contains("processed") || !s.at("processed").is_object()) return std::nullopt;
    const json& p = s.at("processed");
    if (!p.contains(field) || !p.at(field).is_number()) return std::nullopt;
    return p.at(field).get<double>();
}

// Every capacitor component in the TAS, with its designed value / rating / role.
struct RawCapacitor { std::string name; std::optional<double> capacitance, ratedVoltage; std::string role; };

std::vector<RawCapacitor> raw_capacitors(const json& tas) {
    std::vector<RawCapacitor> out;
    if (!tas.contains("topology") || !tas.at("topology").contains("stages")) return out;
    for (const auto& st : tas.at("topology").at("stages")) {
        if (!st.contains("circuit") || !st.at("circuit").is_object() || !st.at("circuit").contains("components"))
            continue;
        for (const auto& c : st.at("circuit").at("components")) {
            if (!c.contains("data") || !c.at("data").is_object() || !c.at("data").contains("capacitor")) continue;
            const json& data = c.at("data");
            RawCapacitor rc;
            rc.name = c.value("name", std::string{});
            if (data.contains("inputs") && data.at("inputs").contains("designRequirements")) {
                const json& dr = data.at("inputs").at("designRequirements");
                if (dr.contains("capacitance")) rc.capacitance = PEAS::resolve_dimensional_values(dr.at("capacitance"));
                if (dr.contains("ratedVoltage") && dr.at("ratedVoltage").is_number())
                    rc.ratedVoltage = dr.at("ratedVoltage").get<double>();
                if (dr.contains("role") && dr.at("role").is_string()) rc.role = dr.at("role").get<std::string>();
            }
            out.push_back(std::move(rc));
        }
    }
    return out;
}

}  // namespace

MAS::Inputs main_magnetic_inputs(const json& tas) {
    auto mags = raw_magnetics(tas);
    if (mags.empty())
        throw std::runtime_error("main_magnetic_inputs: TAS has no magnetic components");
    return mags[main_index(mags)].inputs->get<MAS::Inputs>();
}

json extra_components_inputs(const json& tas) {
    auto mags = raw_magnetics(tas);
    if (mags.empty())
        throw std::runtime_error("extra_components_inputs: TAS has no magnetic components");
    const size_t mi = main_index(mags);
    json out = json::array();
    // every NON-main magnetic (output inductor, resonant Lr, CM choke, ...) as a full MAS::Inputs
    for (size_t i = 0; i < mags.size(); ++i) {
        if (i == mi) continue;
        out.push_back(json{{"componentType", "magnetic"}, {"name", mags[i].name}, {"inputs", *mags[i].inputs}});
    }
    // every capacitor as its (CAS::Inputs-shaped) designRequirements
    for (const auto& c : raw_capacitors(tas)) {
        json dr = json::object();
        if (c.capacitance)  dr["capacitance"]["nominal"] = *c.capacitance;
        if (c.ratedVoltage) dr["ratedVoltage"] = *c.ratedVoltage;
        if (!c.role.empty()) dr["role"] = c.role;
        out.push_back(json{{"componentType", "capacitor"}, {"name", c.name},
                           {"inputs", json{{"designRequirements", std::move(dr)}}}});
    }
    return out;
}

json diagnostics(const json& tas) {
    auto mags = raw_magnetics(tas);
    if (mags.empty())
        throw std::runtime_error("diagnostics: TAS has no magnetic components");
    const size_t mi = main_index(mags);
    auto caps = raw_capacitors(tas);

    json d = json::object();

    // --- component inventory: every magnetic with its designed values ---------------------------------
    json magsArr = json::array();
    for (size_t i = 0; i < mags.size(); ++i) {
        const json& dr = mags[i].inputs->at("designRequirements");
        json m = json::object();
        m["name"] = mags[i].name;
        m["isMain"] = (i == mi);
        m["windings"] = mags[i].windings;
        if (dr.contains("magnetizingInductance"))
            m["magnetizingInductance"] = PEAS::resolve_dimensional_values(dr.at("magnetizingInductance"));
        if (dr.contains("turnsRatios") && dr.at("turnsRatios").is_array()) {
            json trs = json::array();
            for (const auto& t : dr.at("turnsRatios")) trs.push_back(PEAS::resolve_dimensional_values(t));
            m["turnsRatios"] = std::move(trs);
        }
        magsArr.push_back(std::move(m));
    }
    d["magnetics"] = std::move(magsArr);

    json capsArr = json::array();
    for (const auto& c : caps) {
        json j = json::object();
        j["name"] = c.name;
        if (c.capacitance)  j["capacitance"]  = *c.capacitance;
        if (c.ratedVoltage) j["ratedVoltage"] = *c.ratedVoltage;
        if (!c.role.empty()) j["role"] = c.role;
        capsArr.push_back(std::move(j));
    }
    d["capacitors"] = std::move(capsArr);

    // --- computed{}: the cross-topology "computed*" values MKF surfaced -------------------------------
    // (main transformer Lm + turns ratio; resonant Cr from the role="resonant" cap; resonant/extra Lr from
    // the non-main single-winding magnetics — surfaced by name since the TAS carries no magnetic role.)
    json computed = json::object();
    {
        const json& dr = mags[mi].inputs->at("designRequirements");
        if (dr.contains("magnetizingInductance"))
            computed["magnetizingInductance"] = PEAS::resolve_dimensional_values(dr.at("magnetizingInductance"));
        if (dr.contains("turnsRatios") && dr.at("turnsRatios").is_array() && !dr.at("turnsRatios").empty())
            computed["turnsRatio"] = PEAS::resolve_dimensional_values(dr.at("turnsRatios").at(0));
    }
    for (const auto& c : caps)
        if (c.role == "resonant" && c.capacitance) { computed["resonantCapacitance"] = *c.capacitance; break; }
    // extra (non-main) single-winding magnetics = resonant / output inductors
    json extraInductors = json::array();
    for (size_t i = 0; i < mags.size(); ++i) {
        if (i == mi || mags[i].windings > 1) continue;
        const json& dr = mags[i].inputs->at("designRequirements");
        if (!dr.contains("magnetizingInductance")) continue;
        json j = json::object();
        j["name"] = mags[i].name;
        j["inductance"] = PEAS::resolve_dimensional_values(dr.at("magnetizingInductance"));
        extraInductors.push_back(std::move(j));
    }
    if (!extraInductors.empty()) computed["extraInductors"] = std::move(extraInductors);
    d["computed"] = std::move(computed);

    // --- per operating point stresses (main magnetic) -------------------------------------------------
    // MKF's flat + perOp[] shape: one row per OP with per-winding peak/rms current & voltage + duty, and an
    // inferred CCM flag (main-winding current stays > 0 over the cycle). The flat top-level fields mirror
    // the first OP (MKF's "first-OP, get_last_* fallback").
    const json& mainInputs = *mags[mi].inputs;
    json opsArr = json::array();
    if (mainInputs.contains("operatingPoints") && mainInputs.at("operatingPoints").is_array()) {
        for (const auto& op : mainInputs.at("operatingPoints")) {
            if (!op.contains("excitationsPerWinding") || !op.at("excitationsPerWinding").is_array()) continue;
            json row = json::object();
            if (op.contains("name")) row["operatingPointName"] = op.at("name");
            json windings = json::array();
            bool ccm = true;
            bool ccmKnown = false;
            for (size_t w = 0; w < op.at("excitationsPerWinding").size(); ++w) {
                const json& exc = op.at("excitationsPerWinding").at(w);
                json we = json::object();
                if (exc.contains("frequency") && exc.at("frequency").is_number())
                    we["frequency"] = exc.at("frequency").get<double>();
                for (auto side : {"current", "voltage"})
                    for (auto field : {"peak", "rms", "offset", "peakToPeak", "dutyCycle"})
                        if (auto v = processed_field(exc, side, field)) we[std::string(side) + "_" + field] = *v;
                windings.push_back(std::move(we));
                // CCM inference from the PRIMARY winding current: continuous if it never crosses zero.
                if (w == 0) {
                    auto off = processed_field(exc, "current", "offset");
                    auto pkpk = processed_field(exc, "current", "peakToPeak");
                    if (off && pkpk) { ccm = (std::abs(*off) - *pkpk / 2.0) > 0.0; ccmKnown = true; }
                }
            }
            row["windings"] = std::move(windings);
            if (ccmKnown) row["isCcm"] = ccm;
            opsArr.push_back(std::move(row));
        }
    }
    // flat mirror of the first OP
    if (!opsArr.empty()) {
        const json& first = opsArr.front();
        if (first.contains("isCcm")) d["isCcm"] = first.at("isCcm");
        if (first.contains("windings") && !first.at("windings").empty()) {
            const json& w0 = first.at("windings").at(0);
            if (w0.contains("frequency")) d["switchingFrequency"] = w0.at("frequency");
            if (w0.contains("current_peak")) d["primaryPeakCurrent"] = w0.at("current_peak");
            if (w0.contains("current_rms")) d["primaryRmsCurrent"] = w0.at("current_rms");
            if (w0.contains("current_dutyCycle")) d["dutyCycle"] = w0.at("current_dutyCycle");
        }
    }
    d["operatingPoints"] = std::move(opsArr);

    return d;
}

}  // namespace Kirchhoff
