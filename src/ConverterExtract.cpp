#include "ConverterExtract.hpp"

#include "TasAssembler.hpp"          // tas_to_ngspice
#include "NgspiceRunner.hpp"         // run_ngspice_in_process, ngspice_in_process_available
#include "NgspiceNodes.hpp"          // shared TAS->ngspice node reconstruction (node_of_pin/node_voltage)
#include "DimensionJson.hpp"         // PEAS::resolve_dimensional_values (nlohmann::json overload)
#include "processors/WaveformProcessor.h"  // the shared DSP (sampled/harmonics/processed) — reused, not re-implemented

#include <algorithm>
#include <limits>
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

    // AC-input converters (PFC, Vienna) carry the magnetic's operating point at the PEAK OF THE LINE (the
    // analytical solvers size the switching period there). Their run ends on a whole number of line
    // cycles, which is a line ZERO crossing for phase A — reading the last switching period of the run
    // returned an all-zero Vienna phase-A excitation. For them the window is the switching period at the
    // line peak of the winding's current.
    const json& dreqTas = tas.at("inputs").at("designRequirements");
    const std::string inputType = dreqTas.value("inputType", std::string("dc"));
    const bool acInput = (inputType == "acSinglePhase" || inputType == "acThreePhase");
    double linePeriod = 0.0;
    if (acInput) {
        if (!dreqTas.contains("lineFrequency"))
            throw std::runtime_error("extract_operating_point(NGSPICE): AC-input TAS without lineFrequency");
        linePeriod = 1.0 / PEAS::resolve_dimensional_values(dreqTas.at("lineFrequency"));
    }
    // The steady-state comparison window: one switching period (the check runs for DC converters only).
    const double cmpPeriod = period;

    // The ngspice branch token per winding (see below) — needed by the steady-state check as well.
    const std::string magTok = "l" + lower(mags[idx].name) + "_";
    auto branch_of = [&](const NgspiceRunResult& rr, size_t w) -> const std::vector<double>* {
        const std::string suffix = (w == 0 ? std::string("pri") : "sec" + std::to_string(w));
        const std::string token = magTok + suffix;
        for (const auto& kv : rr.vectors) {
            std::string k = lower(kv.first);
            if (k.find("#branch") == std::string::npos) continue;
            if (k.find(token) != std::string::npos) return &kv.second;
        }
        return nullptr;
    };

    // STEADY STATE, VERIFIED (DC converters). The operating point is only meaningful once the converter has settled: a
    // run that stops inside its start-up transient (output capacitors still charging, an output LC still
    // ringing) returned currents several times the design value and winding voltages with a DC mean,
    // and the advisers then found no core for them — silently. The caller's stop time (the wizard's
    // steady-state periods) is where we START: each run is checked by comparing every winding current over
    // its last comparison window with the same window at mid-run. A deck that has not settled and declares
    // its slow states (simulation.initialConditions) is solved for its periodic steady state by shooting
    // (below) and re-run from it — briefly when the solve converged, over kVerifyPeriods when it did not;
    // otherwise, or if that run still differs, the run is repeated with twice the stop time. A circuit that has not settled within those budgets is an
    // error, never an extracted operating point.
    constexpr int    kMaxDoublings = 10;      // up to 1024 x the requested window ...
    constexpr double kMaxComparisonPeriods = 20000.0;  // ... never longer than 20000 switching periods,
                                              // so a browser run stays finite ...
    constexpr double kMaxRunBytes = 512e6;    // ... and never a run whose captured vectors exceed ~512 MB:
                                              // an AC-input deck (122 saved vectors, 0.5 us step) doubled
                                              // blindly reached tens of GB and exhausted the host / WASM heap.
    constexpr double kSteadyTolerance = 0.05; // NRMSE between the two windows, per winding current
    constexpr double kShootTolerance = 1e-5;  // |Φ(x) − x| per declared state, relative to its group's scale
    constexpr double kShotRunPeriods = 8.0;   // a converged shot is re-run for this many periods and read at
                                              // the end (undeclared fast states settle in the first one)
    constexpr double kVerifyPeriods = 1024.0; // an unconverged shot is verified over this many periods: long
                                              // enough for a slow output-filter ring to show if it is there
    auto run_deck = [&](const json& t) {
        std::string deck = tas_to_ngspice(t, fidelity);
        // The extraction reads only inductor branch currents and node voltages, which ngspice saves by
        // default. `savecurrents` (every device terminal current, for the per-component overlays) is ~10x
        // the vectors and was what made long settle runs expensive in memory; drop it for these runs.
        for (std::string::size_type at; (at = deck.find(" savecurrents")) != std::string::npos; )
            deck.erase(at, std::string(" savecurrents").size());
        NgspiceRunResult rr = run_ngspice_in_process(deck);
        if (!rr.success)
            throw std::runtime_error("extract_operating_point(NGSPICE): sim failed: " + rr.error);
        if (rr.time.size() < 2)
            throw std::runtime_error("extract_operating_point(NGSPICE): sim produced no transient data");
        return rr;
    };
    auto set_stop_time = [](json& t, double stop) {
        for (auto& an : t.at("simulation").at("analyses"))
            if (an.value("type", "") == "transient") an["stopTime"] = stop;
    };
    auto stop_time_of = [](const json& t) {
        for (const auto& an : t.at("simulation").at("analyses"))
            if (an.value("type", "") == "transient") return an.at("stopTime").get<double>();
        throw std::runtime_error("extract_operating_point(NGSPICE): the TAS has no transient analysis");
    };
    // The simulated vector of one declared initial condition: a winding's branch current, or a node voltage
    // (a "<stage>.<net>" node matched exactly, so it cannot land on another stage's net of the same name).
    auto ic_vector = [&](const NgspiceRunResult& rr, const json& ic) -> const std::vector<double>& {
        if (ic.contains("current")) {
            const int w = ic.value("winding", 0);
            const std::string tok = lower("l" + ic.at("component").get<std::string>() + "_"
                                          + (w == 0 ? std::string("pri") : "sec" + std::to_string(w)));
            const std::string stagePrefix = lower("x" + ngnodes::sanitize(ic.at("stage").get<std::string>()) + ".");
            for (const auto& kv : rr.vectors) {
                const std::string k = lower(kv.first);
                if (k.find("#branch") != std::string::npos && k.find(tok) != std::string::npos
                    && k.find(stagePrefix) != std::string::npos)
                    return kv.second;
            }
            throw std::runtime_error("extract_operating_point(NGSPICE): initial current " + ic.dump()
                                     + " names no winding branch in the simulated deck");
        }
        const std::string node = ic.at("node").get<std::string>();
        const auto dot = node.find('.');
        const std::string deckNode = (dot == std::string::npos)
            ? lower(node)
            : lower("x" + ngnodes::sanitize(node.substr(0, dot)) + "." + node.substr(dot + 1));
        for (const auto& kv : rr.vectors) {
            const std::string k = lower(kv.first);
            if (k == deckNode || k == "v(" + deckNode + ")") return kv.second;
        }
        throw std::runtime_error("extract_operating_point(NGSPICE): initial condition node '" + node
                                 + "' has no voltage in the simulated deck");
    };
    auto value_at = [](const NgspiceRunResult& rr, const std::vector<double>& v, double t) {
        size_t j = static_cast<size_t>(std::upper_bound(rr.time.begin(), rr.time.end(), t) - rr.time.begin());
        if (j == 0) return v.front();
        if (j >= rr.time.size()) return v.back();
        const double f = (t - rr.time[j - 1]) / (rr.time[j] - rr.time[j - 1]);
        return v[j - 1] + f * (v[j] - v[j - 1]);
    };

    // PERIODIC STEADY STATE BY SHOOTING (Aprille & Trick). The declared initial conditions x are the deck's
    // slow states (output rails, blocking caps, inductor and winding currents). Φ(x) is their value after
    // kShootPeriods switching periods started from x; the steady state is the fixed point Φ(x) = x, found
    // by Newton with a finite-difference Jacobian — a few dozen runs of a few periods each. This is what a
    // lossless deck needs: its output filters (Q in the hundreds) ring for longer than any affordable run,
    // and the isolated buck's secondary current is set by a millivolt difference between its two rails, so
    // no average of a ringing run pins the state closely enough. States the TAS does not declare (snubbers,
    // switch capacitances) are fast and restart from zero inside the first of the kShootPeriods periods.
    //   The fixed point, not a long run, IS the periodic steady state (it is what a PSS analysis computes). A
    // long run of a lossless deck does not stay on it: ngspice's per-step truncation error (reltol 1e-3)
    // keeps re-exciting modes that decay over thousands of periods (the isolated buck's 4-period Floquet
    // multiplier is 0.9991) or grow slowly (the AHB's is 1.0097 — no loss damps it), so a run's last period
    // wanders around the orbit and a mid-run comparison passes or fails by where the wander happens to be.
    // Returns the residual reached; the run that follows re-applies the steady-state comparison.
    auto shoot = [&](json& t) -> double {
        constexpr int    kShootPeriods = 4;
        constexpr int    kMaxNewton = 16;
        json& ics = t.at("simulation").at("initialConditions");
        const size_t n = ics.size();
        std::vector<double> x(n);
        std::vector<bool> isCurrent(n);
        for (size_t i = 0; i < n; ++i) {
            isCurrent[i] = ics[i].contains("current");
            x[i] = ics[i].at(isCurrent[i] ? "current" : "voltage").get<double>();
        }
        json ts = t;
        const double tShoot = kShootPeriods * period;
        set_stop_time(ts, tShoot);
        auto phi = [&](const std::vector<double>& xx) {
            for (size_t i = 0; i < n; ++i) ts.at("simulation").at("initialConditions")[i][isCurrent[i] ? "current" : "voltage"] = xx[i];
            const NgspiceRunResult rr = run_deck(ts);
            // The state just before the edge that starts the next cycle — where the initial conditions sit.
            const double tSample = std::min(tShoot, rr.time.back()) - 1e-6 * period;
            std::vector<double> out(n);
            for (size_t i = 0; i < n; ++i) out[i] = value_at(rr, ic_vector(rr, ts.at("simulation").at("initialConditions")[i]), tSample);
            return out;
        };
        // Scales: voltages against the largest declared voltage, currents against the largest current seen.
        auto scales = [&](const std::vector<double>& a, const std::vector<double>& b) {
            double vs = 0.0, cs = 0.0;
            for (size_t i = 0; i < n; ++i) {
                double& g = isCurrent[i] ? cs : vs;
                g = std::max(g, std::max(std::abs(a[i]), std::abs(b[i])));
            }
            std::vector<double> sc(n);
            for (size_t i = 0; i < n; ++i) sc[i] = isCurrent[i] ? cs : vs;
            return sc;
        };
        std::vector<double> pxNext;
        double residual = std::numeric_limits<double>::infinity();
        for (int it = 0; it < kMaxNewton; ++it) {
            const std::vector<double> px = pxNext.empty() ? phi(x) : pxNext;
            std::vector<double> sc = scales(x, px);
            double worst = 0.0;
            for (size_t i = 0; i < n; ++i) if (sc[i] > 0) worst = std::max(worst, std::abs(px[i] - x[i]) / sc[i]);
            residual = worst;
            if (worst <= kShootTolerance) break;
            // J = dΦ/dx − I, column by column.
            std::vector<std::vector<double>> J(n, std::vector<double>(n + 1));
            for (size_t j = 0; j < n; ++j) {
                const double h = 1e-4 * (sc[j] > 0 ? sc[j] : 1.0);
                std::vector<double> xp = x; xp[j] += h;
                const std::vector<double> pp = phi(xp);
                for (size_t i = 0; i < n; ++i) J[i][j] = (pp[i] - px[i]) / h - (i == j ? 1.0 : 0.0);
            }
            for (size_t i = 0; i < n; ++i) J[i][n] = -(px[i] - x[i]);
            // Gaussian elimination with partial pivoting.
            bool singular = false;
            for (size_t c = 0; c < n && !singular; ++c) {
                size_t piv = c;
                for (size_t rI = c + 1; rI < n; ++rI) if (std::abs(J[rI][c]) > std::abs(J[piv][c])) piv = rI;
                if (!(std::abs(J[piv][c]) > 1e-12)) { singular = true; break; }
                std::swap(J[c], J[piv]);
                for (size_t rI = c + 1; rI < n; ++rI) {
                    const double f = J[rI][c] / J[c][c];
                    for (size_t k = c; k <= n; ++k) J[rI][k] -= f * J[c][k];
                }
            }
            // A neutral state (a floating node with nothing setting its level) has no fixed point to find;
            // stop and let the verification run judge the state we have.
            if (singular) break;
            std::vector<double> dx(n);
            for (size_t c = n; c-- > 0; ) {
                double acc = J[c][n];
                for (size_t k = c + 1; k < n; ++k) acc -= J[c][k] * dx[k];
                dx[c] = acc / J[c][c];
            }
            // Limit the step to half of each group's scale: a diode that starts or stops conducting makes Φ
            // piecewise, and a full Newton step across such a kink can overshoot far.
            double lim = 1.0;
            for (size_t i = 0; i < n; ++i)
                if (sc[i] > 0 && std::abs(dx[i]) > 0.5 * sc[i]) lim = std::min(lim, 0.5 * sc[i] / std::abs(dx[i]));
            // Backtrack: accept the (limited) step only if it lowers the residual, else halve it. The residual
            // of the accepted point is the next iteration's, so no run is wasted.
            double step = lim;
            std::vector<double> xn(n), pn;
            double worstN = 0.0;
            for (int bt = 0; bt < 6; ++bt, step *= 0.5) {
                for (size_t i = 0; i < n; ++i) xn[i] = x[i] + step * dx[i];
                pn = phi(xn);
                const std::vector<double> scn = scales(xn, pn);
                worstN = 0.0;
                for (size_t i = 0; i < n; ++i) if (scn[i] > 0) worstN = std::max(worstN, std::abs(pn[i] - xn[i]) / scn[i]);
                if (worstN < worst) break;
            }
            if (!(worstN < worst)) break;   // no step lowers the residual: Newton has done what it can
            x = xn;
            pxNext = pn;
            residual = worstN;
        }
        for (size_t i = 0; i < n; ++i) ics[i][isCurrent[i] ? "current" : "voltage"] = x[i];
        return residual;
    };

    json tasRun = tas;
    NgspiceRunResult r;
    double lastMismatch = 0.0;
    std::string lastMismatchWinding;
    bool steady = false;
    bool shot = false;
    for (int attempt = 0; attempt <= kMaxDoublings + 1; ++attempt) {
        r = run_deck(tasRun);
        // AC-input decks (PFC, Vienna) are read at the line peak of their LAST cycle and are not extended:
        // their bus-voltage loops settle over many line cycles (Vienna's envelope still drifts ~10 % after
        // 9 cycles), which a browser run cannot afford. Their settle verification is an open item; until
        // then they keep the single run they always had rather than turning into errors here.
        if (acInput) { steady = true; break; }
        const double tEndRun = r.time.back();
        // The earlier window ends a WHOLE number of comparison periods before the end (about mid-run), so a
        // periodic waveform lines up with itself; comparing against the literal mid-point shifted the phase
        // and flagged perfectly steady decks. A relative epsilon absorbs the float round-off of the stop time.
        const double span = tEndRun - r.time.front();
        if (span < 2.0 * cmpPeriod * (1.0 - 1e-6))
            throw std::runtime_error("extract_operating_point(NGSPICE): transient span " + std::to_string(tEndRun)
                                     + "s is shorter than two comparison windows of " + std::to_string(cmpPeriod)
                                     + "s — cannot verify steady state for magnetic '" + mags[idx].name + "'");
        const double periodsBack = std::max(1.0, std::floor(0.5 * span / cmpPeriod * (1.0 + 1e-9)));
        const double tMid = tEndRun - periodsBack * cmpPeriod;
        // Compare the winding currents over [tEnd-cmp, tEnd] and [tMid-cmp, tMid] on a common 256-point grid.
        auto window = [&](const std::vector<double>& src, double tStop) {
            const int M = 256;
            std::vector<double> out(M);
            for (int k = 0; k < M; ++k) {
                const double t = tStop - cmpPeriod + cmpPeriod * k / M;
                size_t j = static_cast<size_t>(std::upper_bound(r.time.begin(), r.time.end(), t) - r.time.begin());
                if (j == 0) { out[k] = src.front(); continue; }
                if (j >= r.time.size()) { out[k] = src.back(); continue; }
                const double f = (t - r.time[j - 1]) / (r.time[j] - r.time[j - 1]);
                out[k] = src[j - 1] + f * (src[j] - src[j - 1]);
            }
            return out;
        };
        // Normalise every winding's difference by the LARGEST winding rms, so a winding that legitimately
        // carries almost nothing (an unloaded or blocked rail) is not judged on its own noise.
        double refRms = 0.0;
        std::vector<std::pair<std::vector<double>, std::vector<double>>> pairs;
        for (size_t w = 0; w < excs0.size(); ++w) {
            const std::vector<double>* sig = branch_of(r, w);
            if (!sig) break;   // the extraction below throws the specific missing-branch error
            auto a = window(*sig, tEndRun), b = window(*sig, tMid);
            double ra = 0.0; for (double v : a) ra += v * v;
            refRms = std::max(refRms, std::sqrt(ra / a.size()));
            pairs.emplace_back(std::move(a), std::move(b));
        }
        if (pairs.size() != excs0.size()) { steady = true; break; }   // let the extraction name the branch
        if (!(refRms > 0))
            throw std::runtime_error("extract_operating_point(NGSPICE): every winding current of magnetic '"
                                     + mags[idx].name + "' is zero at the end of the run");
        lastMismatch = 0.0;
        for (size_t w = 0; w < pairs.size(); ++w) {
            double d2 = 0.0;
            for (size_t k = 0; k < pairs[w].first.size(); ++k) {
                const double dv = pairs[w].first[k] - pairs[w].second[k];
                d2 += dv * dv;
            }
            const double nrmse = std::sqrt(d2 / pairs[w].first.size()) / refRms;
            if (nrmse > lastMismatch) { lastMismatch = nrmse; lastMismatchWinding = std::to_string(w); }
        }
        if (lastMismatch <= kSteadyTolerance) { steady = true; break; }
        const double bytesThisRun = 8.0 * static_cast<double>(r.time.size()) * static_cast<double>(r.vectors.size() + 1);
        const bool declaresIc = tasRun.contains("simulation") && tasRun.at("simulation").contains("initialConditions")
                                && !tasRun.at("simulation").at("initialConditions").empty();
        if (declaresIc && !shot) {
            // Not settled: solve for the periodic steady state. Converged, it is re-run briefly from that
            // state; not converged, the shot state is only a better start and is verified over kVerifyPeriods.
            const double residual = shoot(tasRun);
            shot = true;
            if (residual <= kShootTolerance) {
                set_stop_time(tasRun, kShotRunPeriods * period);
                continue;
            }
            const double factor = std::max(1.0, kVerifyPeriods * period / span * (1.0 - 1e-9));
            if (factor * bytesThisRun > kMaxRunBytes)
                throw std::runtime_error("extract_operating_point(NGSPICE): verifying the steady state of magnetic '"
                                         + mags[idx].name + "' over " + std::to_string(kVerifyPeriods)
                                         + " periods would exceed the simulation memory budget");
            set_stop_time(tasRun, std::max(stop_time_of(tasRun), kVerifyPeriods * period));
            continue;
        }
        // Not settled: run again for twice as long — within the memory and length budget.
        if (2.0 * bytesThisRun > kMaxRunBytes || 2.0 * span > kMaxComparisonPeriods * cmpPeriod) break;
        set_stop_time(tasRun, 2.0 * stop_time_of(tasRun));
    }
    if (!steady)
        throw std::runtime_error("extract_operating_point(NGSPICE): magnetic '" + mags[idx].name
                                 + "' did not reach steady state within " + std::to_string(r.time.back())
                                 + " s (winding " + lastMismatchWinding + " still changes by "
                                 + std::to_string(100.0 * lastMismatch) + " % between mid-run and the end)");

    const double tEnd = r.time.back();
    // The extraction resamples ONE switching period. DC converters: the LAST period of the run. AC-input
    // converters: the period centred on the line peak of the primary winding's current in the last line
    // cycle (the running one-switching-period average, i.e. the line envelope, at its largest magnitude).
    double tBeg = tEnd - period;
    if (acInput) {
        const std::vector<double>* sig = branch_of(r, 0);
        if (!sig)
            throw std::runtime_error("extract_operating_point(NGSPICE): no ngspice branch for winding 0 of magnetic '"
                                     + mags[idx].name + "'");
        const double t0 = tEnd - linePeriod;
        // Prefix integral for the moving average over one switching period.
        std::vector<double> cum(r.time.size(), 0.0);
        for (size_t i = 1; i < r.time.size(); ++i)
            cum[i] = cum[i - 1] + 0.5 * ((*sig)[i] + (*sig)[i - 1]) * (r.time[i] - r.time[i - 1]);
        auto integral_at = [&](double t) {
            size_t j = static_cast<size_t>(std::upper_bound(r.time.begin(), r.time.end(), t) - r.time.begin());
            if (j == 0) return cum[0];
            if (j >= r.time.size()) return cum.back();
            const double f = (t - r.time[j - 1]) / (r.time[j] - r.time[j - 1]);
            return cum[j - 1] + f * (cum[j] - cum[j - 1]);
        };
        double best = -1.0;
        const int steps = 512;
        for (int k = 0; k <= steps; ++k) {
            const double tc = t0 + period / 2 + (linePeriod - period) * k / steps;   // window centre
            const double avg = (integral_at(tc + period / 2) - integral_at(tc - period / 2)) / period;
            if (std::abs(avg) > best) { best = std::abs(avg); tBeg = tc - period / 2; }
        }
    }

    // Resample a full-length simulated signal (one sample per r.time point) onto N=128 points over the LAST
    // switching period, mapped to t∈[0,period) — the exact grid ComponentWaveforms and the winding current
    // share, so every waveform in the app lines up.
    const int N = 128;
    // One switching period on N uniform samples. Each sample is the AVERAGE of the simulated signal over its
    // bin (centred on the sample time), not the instantaneous value there: a commutation spike lasting a few
    // nanoseconds (leakage inductance against a snubber capacitor at a switching edge) landed on one grid
    // point and was stretched to a whole 1/N of the period — an AHB flyback secondary carrying 17 A came back
    // with a 1.2 kA sample and a 110 A rms. The bin average keeps the charge of such an event and every
    // harmonic well below N/2, which is all the magnetic sees.
    auto resample_last_period = [&](const std::vector<double>& src) -> MAS::Waveform {
        std::vector<double> data(N), time(N);
        const double dt = period / N;
        auto interp = [&](size_t j, double t) {   // value at t within [time[j], time[j+1]]
            const double h = r.time[j + 1] - r.time[j];
            const double f = (h > 0) ? (t - r.time[j]) / h : 0.0;
            return src[j] + f * (src[j + 1] - src[j]);
        };
        size_t j = 0;
        for (int k = 0; k < N; ++k) {
            const double a = tBeg + dt * (k - 0.5), b = tBeg + dt * (k + 0.5);
            while (j + 2 < r.time.size() && r.time[j + 1] <= a) ++j;
            double area = 0.0;
            for (size_t i = j; i + 1 < r.time.size() && r.time[i] < b; ++i) {
                const double lo = std::max(a, r.time[i]), hi = std::min(b, r.time[i + 1]);
                if (hi > lo) area += 0.5 * (interp(i, lo) + interp(i, hi)) * (hi - lo);
            }
            data[k] = area / dt;
            time[k] = dt * k;
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
    auto& excs = op.get_mutable_excitations_per_winding();
    // The analytical excitations are the MAS reference convention for each winding's current and voltage
    // (a secondary's current is positive in its power direction, e.g. out of the dot for a DAB or forward
    // secondary). The deck's branch current is positive INTO the winding's start pin, and a builder may
    // wire a winding either way round; reported raw, a Weinberg primary or a two-switch-forward secondary
    // came back with the opposite sign — flipping DC offsets, so MKF saw a DC magnetizing current that is
    // not there and rejected every core at saturation. Express each simulated signal in the analytical
    // convention: the sign of its correlation with the analytical waveform over the same cycle. A signal
    // too uncorrelated to tell is an error. The analytical models emit the MAS excitation convention
    // (voltages in the dot reference, primary-side currents passive, others source; guarded by
    // [convention]), so the oriented simulation is in that convention too, whichever way round a builder
    // wired a winding.
    const std::vector<MAS::OperatingPointExcitation> analyticalExcs = excs;
    auto oriented = [&](const MAS::Waveform& sim, const std::optional<MAS::SignalDescriptor>& ref,
                        size_t w, const char* what) -> MAS::Waveform {
        // A winding whose analytical excitation is processed data only (a label with peak/rms/offset, as
        // the AHB flyback and current-doubler windings carry) has no reference shape: its simulated signal
        // stays in the deck's own, stated convention — current into the start pin, voltage V(start) − V(end).
        if (!ref || !ref->get_waveform() || ref->get_waveform()->get_data().size() < 2) return sim;
        // get_waveform() returns the optional BY VALUE: keep a copy, never a reference into the temporary.
        const MAS::Waveform refWf = *ref->get_waveform();
        const std::vector<double> rd = refWf.get_data();
        std::vector<double> rt;
        if (refWf.get_time() && refWf.get_time()->size() == rd.size())
            rt = *refWf.get_time();
        else { rt.resize(rd.size()); for (size_t k = 0; k < rd.size(); ++k) rt[k] = period * k / rd.size(); }
        const auto& sd = sim.get_data();
        const std::vector<double> st = *sim.get_time();   // get_time() returns the optional by value
        double dot = 0.0, ns = 0.0, nr = 0.0;
        for (size_t k = 0; k < sd.size(); ++k) {
            const double t = st[k];
            size_t j = static_cast<size_t>(std::upper_bound(rt.begin(), rt.end(), t) - rt.begin());
            double rv;
            if (j == 0) rv = rd.front();
            else if (j >= rt.size()) rv = rd.back();
            else { const double f = (t - rt[j - 1]) / (rt[j] - rt[j - 1]); rv = rd[j - 1] + f * (rd[j] - rd[j - 1]); }
            dot += sd[k] * rv; ns += sd[k] * sd[k]; nr += rv * rv;
        }
        if (!(ns > 0) || !(nr > 0)) return sim;   // an all-zero winding has no orientation to fix
        const double corr = dot / std::sqrt(ns * nr);
        if (std::abs(corr) < 0.2)
            throw std::runtime_error(std::string("extract_operating_point(NGSPICE): cannot orient the simulated ") + what
                                     + " of winding " + std::to_string(w) + " of magnetic '" + mags[idx].name
                                     + "' against the analytical one (correlation " + std::to_string(corr) + ")");
        if (corr > 0) return sim;
        MAS::Waveform flipped = sim;
        std::vector<double> neg(sd.size());
        for (size_t k = 0; k < sd.size(); ++k) neg[k] = -sd[k];
        flipped.set_data(neg);
        return flipped;
    };
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
        excs[w].set_current(to_signal(oriented(resample_last_period(*sig), analyticalExcs[w].get_current(), w, "current")));

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
        excs[w].set_voltage(to_signal(oriented(resample_last_period(vdiff), analyticalExcs[w].get_voltage(), w, "voltage")));
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
