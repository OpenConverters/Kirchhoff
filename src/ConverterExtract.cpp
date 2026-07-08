#include "ConverterExtract.hpp"

#include "TasAssembler.hpp"          // tas_to_ngspice
#include "NgspiceRunner.hpp"         // run_ngspice_in_process, ngspice_in_process_available
#include "NgspiceNodes.hpp"          // shared TAS->ngspice node reconstruction (node_of_pin/node_voltage)
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
struct RawMagnetic { std::string name; const json* inputs; size_t windings; std::string stage; const json* circuit; };

std::vector<RawMagnetic> raw_magnetics(const json& tas) {
    std::vector<RawMagnetic> out;
    if (!tas.contains("topology") || !tas.at("topology").contains("stages")) return out;
    for (const auto& st : tas.at("topology").at("stages")) {
        if (!st.contains("circuit") || !st.at("circuit").is_object() || !st.at("circuit").contains("components"))
            continue;
        const std::string stageName = st.value("name", std::string{});
        const json& circuit = st.at("circuit");
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
            out.push_back({c.value("name", std::string{}), &data.at("inputs"), nw, stageName, &circuit});
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
MAS::OperatingPoint ngspice_operating_point_of(const json& tas, const std::vector<RawMagnetic>& mags, size_t idx,
                                               const PEAS::Fidelity& fidelity) {
    if (!ngspice_in_process_available())
        throw std::runtime_error("extract_operating_point(NGSPICE): Kirchhoff built without libngspice");
    // Start from the analytical operating point (correct winding count / labels / voltages) and overwrite
    // the currents with the simulated ones where we can find the matching branch.
    MAS::OperatingPoint op = analytical_operating_point_of(mags, idx);

    const std::string deck = tas_to_ngspice(tas, fidelity);
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
    // The extraction resamples the LAST switching period [tEnd-period, tEnd]. A transient shorter than one
    // period has no full cycle to read; extracting it would resample past the end of the time vector. Throw
    // loudly (per the no-fallback rule) instead of extrapolating a bogus waveform.
    if (tEnd < period)
        throw std::runtime_error("extract_operating_point(NGSPICE): transient span " + std::to_string(tEnd)
                                 + "s is shorter than one switching period " + std::to_string(period)
                                 + "s — cannot extract a cycle for magnetic '" + mags[idx].name + "'");
    const double tBeg = tEnd - period;

    // Resample a full-length simulated signal (one sample per r.time point) onto N=128 points over the LAST
    // switching period, mapped to t∈[0,period) — the exact grid ComponentWaveforms and the winding current
    // share, so every waveform in the app lines up.
    const int N = 128;
    auto resample_last_period = [&](const std::vector<double>& src) -> MAS::Waveform {
        std::vector<double> data(N), time(N);
        size_t j = 0;
        for (int k = 0; k < N; ++k) {
            double t = tBeg + period * k / N;
            while (j + 1 < r.time.size() && r.time[j + 1] < t) ++j;
            if (j + 1 >= r.time.size()) j = r.time.size() - 2;   // clamp: keep [j, j+1] in bounds (size>=2)
            double f = (r.time[j + 1] - r.time[j] > 0) ? (t - r.time[j]) / (r.time[j + 1] - r.time[j]) : 0.0;
            data[k] = src[j] + f * (src[std::min(j + 1, src.size() - 1)] - src[j]);
            time[k] = period * k / N;
        }
        MAS::Waveform wf;
        wf.set_ancillary_label(MAS::WaveformLabel::CUSTOM);
        wf.set_data(data);
        wf.set_time(time);
        return wf;
    };
    auto to_signal = [&](const MAS::Waveform& wf) -> MAS::SignalDescriptor {
        MAS::SignalDescriptor sd;
        sd.set_waveform(wf);
        sd.set_harmonics(OpenMagnetics::WaveformProcessor::calculate_harmonics_data(wf, fsw));
        sd.set_processed(OpenMagnetics::WaveformProcessor::calculate_processed_data(wf, fsw));
        return sd;
    };

    // Node reconstruction for the winding VOLTAGES: the winding pins follow the TAS convention
    // primary_/secondary<i>_{start,end}; each maps to an ngspice node via the shared node resolver. The
    // winding voltage is the node difference V(start) − V(end) (passive sign, same orientation as the
    // branch current). This replaces the analytical voltage that used to ride through — the whole point of
    // ABT #3: the "Simulated" view must not mix simulated current with a synthesized voltage.
    const ngnodes::InterStage inter = ngnodes::read_inter_stage(tas.at("topology"));
    const ngnodes::BrickNets brickNets = ngnodes::read_brick_nets(*mags[idx].circuit);
    const std::string& stage = mags[idx].stage;

    // The CIAS->ngspice serializer names each winding's inductor deterministically (CiasCircuitConverter.cpp):
    // the primary is "L<comp>_pri", secondary i is "L<comp>_sec<i>". ngspice reports each inductor's branch
    // current lowercased as "...l<comp>_<suffix>#branch". We reconstruct that exact token per winding and
    // look it up — NO name-heuristics and NO fallback: if a winding's branch is absent the extraction is
    // wrong and we throw loudly (per the no-silent-fallback rule) rather than silently keep the analytical
    // current.
    const std::string magTok = "l" + lower(mags[idx].name) + "_";
    auto& excs = op.get_mutable_excitations_per_winding();
    for (size_t w = 0; w < excs.size(); ++w) {
        const std::string suffix = (w == 0 ? std::string("pri") : "sec" + std::to_string(w));
        const std::string token = magTok + suffix;
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
        excs[w].set_current(to_signal(resample_last_period(*sig)));

        // Winding voltage = difference of the two terminal node voltages.
        const std::string posPin = (w == 0 ? "primary_start" : "secondary" + std::to_string(w) + "_start");
        const std::string negPin = (w == 0 ? "primary_end"   : "secondary" + std::to_string(w) + "_end");
        const std::string np = ngnodes::node_of_pin(stage, mags[idx].name, posPin, brickNets, inter);
        const std::string nn = ngnodes::node_of_pin(stage, mags[idx].name, negPin, brickNets, inter);
        bool okP = false, okN = false;
        const std::vector<double> A = ngnodes::node_voltage(np, r, okP);
        const std::vector<double> B = ngnodes::node_voltage(nn, r, okN);
        if (!okP || !okN)
            throw std::runtime_error("extract_operating_point(NGSPICE): could not resolve voltage nodes for "
                                     "winding " + std::to_string(w) + " of magnetic '" + mags[idx].name
                                     + "' (pins '" + posPin + "'->'" + np + "', '" + negPin + "'->'" + nn
                                     + "'); cannot build a simulated winding voltage");
        std::vector<double> vdiff(r.time.size());
        for (size_t i = 0; i < vdiff.size(); ++i)
            vdiff[i] = (i < A.size() ? A[i] : 0.0) - (i < B.size() ? B[i] : 0.0);
        excs[w].set_voltage(to_signal(resample_last_period(vdiff)));
    }
    return op;
}

}  // namespace

MAS::OperatingPoint extract_operating_point(const json& tas, ExtractEngine engine, const std::string& magneticName,
                                            const PEAS::Fidelity& fidelity) {
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
        case ExtractEngine::NGSPICE:    return ngspice_operating_point_of(tas, mags, idx, fidelity);
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

MAS::Inputs main_magnetic_inputs(const json& tas, const std::string& magneticName) {
    auto mags = raw_magnetics(tas);
    if (mags.empty())
        throw std::runtime_error("main_magnetic_inputs: TAS has no magnetic components");
    // A topology can carry more than one magnetic (LLC/CLLC/CLLLC transformer + resonant inductor;
    // SEPIC/Cuk/Zeta two inductors; PSFB/DAB transformer + output inductor). When the caller names one
    // (the drawer passes the BOM row's ref == component name), design THAT magnetic; otherwise default
    // to the main one (most windings). Mirrors extract_operating_point's by-name selection.
    size_t idx = main_index(mags);
    if (!magneticName.empty()) {
        bool found = false;
        for (size_t i = 0; i < mags.size(); ++i)
            if (mags[i].name == magneticName) { idx = i; found = true; break; }
        if (!found)
            throw std::runtime_error("main_magnetic_inputs: magnetic '" + magneticName + "' not found in TAS");
    }
    return mags[idx].inputs->get<MAS::Inputs>();
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
    // Converter-level switching frequency from the TAS root (every topology builder writes
    // inputs.designRequirements.switchingFrequency). For AC-input topologies (PFC, Vienna) the
    // MAIN magnetic's excitation is stamped at the LINE frequency (50/60 Hz envelope), so
    // winding-0's excitation frequency is NOT the switching frequency, and the DC-biased-triangle
    // CCM inference below is meaningless for it (ABT #149: "DCM @ 50 Hz" on a CCM 65 kHz PFC).
    std::optional<double> converterFsw;
    if (tas.contains("inputs") && tas.at("inputs").is_object()) {
        const json& ti = tas.at("inputs");
        if (ti.contains("designRequirements") && ti.at("designRequirements").contains("switchingFrequency"))
            converterFsw = PEAS::resolve_dimensional_values(ti.at("designRequirements").at("switchingFrequency"));
    }
    // An excitation "runs at the switching rate" when its frequency is commensurate with the
    // declared converter fsw; a line-frequency envelope sits orders of magnitude below it. When the
    // TAS declares no fsw we cannot tell and keep the legacy first-winding behavior.
    auto atSwitchingRate = [&](const json& exc) {
        if (!converterFsw) return true;
        if (!exc.contains("frequency") || !exc.at("frequency").is_number()) return true;
        return exc.at("frequency").get<double>() > 0.5 * *converterFsw;
    };
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
                // Only valid on a switching-rate excitation — a line-frequency sine envelope always
                // "fails" this DC-biased-triangle test and would report DCM regardless of the design.
                if (w == 0 && atSwitchingRate(exc)) {
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
            // Prefer the converter-declared switching frequency: for AC-input topologies w0 runs at
            // the line frequency and mirroring it here surfaced "50 Hz" as the switching frequency.
            if (converterFsw) d["switchingFrequency"] = *converterFsw;
            else if (w0.contains("frequency")) d["switchingFrequency"] = w0.at("frequency");
            if (w0.contains("current_peak")) d["primaryPeakCurrent"] = w0.at("current_peak");
            if (w0.contains("current_rms")) d["primaryRmsCurrent"] = w0.at("current_rms");
            // A line-frequency envelope's 0.500 "duty cycle" is the sine's midpoint, not a
            // converter duty — only mirror duty from a switching-rate excitation.
            if (w0.contains("current_dutyCycle") && atSwitchingRate(first.at("windings").at(0)))
                d["dutyCycle"] = w0.at("current_dutyCycle");
        }
    }
    d["operatingPoints"] = std::move(opsArr);

    return d;
}

}  // namespace Kirchhoff
