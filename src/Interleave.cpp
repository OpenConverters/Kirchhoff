#include "Interleave.hpp"
#include "KirchhoffConfig.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {

std::string sfx(const std::string& base, int k) { return base + "_" + std::to_string(k); }

bool is_port_endpoint(const json& e) { return e.contains("port") && !e.contains("component"); }

// One endpoint of phase k: components and non-shared ports are renamed, shared ports stay as they are.
json phase_endpoint(const json& e, int k, const std::set<std::string>& shared) {
    json out = e;
    if (e.contains("component")) out["component"] = sfx(e.at("component").get<std::string>(), k);
    else if (is_port_endpoint(e) && !shared.count(e.at("port").get<std::string>()))
        out["port"] = sfx(e.at("port").get<std::string>(), k);
    return out;
}

} // namespace

json interleave_stage(const json& tas, const std::string& stageName, int N, const std::set<std::string>& shared) {
    if (N < 1) throw std::invalid_argument("interleave_stage: phase count must be >= 1");
    if (N == 1) return tas;
    json out = tas;
    json* stage = nullptr;
    for (auto& st : out.at("topology").at("stages"))
        if (st.at("name") == stageName) stage = &st;
    if (!stage) throw std::invalid_argument("interleave_stage: no stage named '" + stageName + "'");
    const json& brick = stage->at("circuit");

    // Every shared port must exist on the brick: a typo would silently replicate an input rail per phase.
    std::set<std::string> brickPorts;
    for (const auto& p : brick.at("ports")) brickPorts.insert(p.at("name").get<std::string>());
    for (const auto& s : shared)
        if (!brickPorts.count(s))
            throw std::invalid_argument("interleave_stage: shared port '" + s + "' is not a port of stage '" + stageName + "'");

    json ports = json::array();
    for (const auto& p : brick.at("ports")) {
        const std::string name = p.at("name").get<std::string>();
        if (shared.count(name)) { ports.push_back(p); continue; }
        for (int k = 1; k <= N; ++k) { json q = p; q["name"] = sfx(name, k); ports.push_back(q); }
    }
    json comps = json::array();
    for (int k = 1; k <= N; ++k)
        for (const auto& c : brick.at("components")) {
            json q = c; q["name"] = sfx(c.at("name").get<std::string>(), k); comps.push_back(q);
        }
    json conns = json::array();
    for (const auto& c : brick.at("connections")) {
        bool touchesShared = false;
        for (const auto& e : c.at("endpoints"))
            if (is_port_endpoint(e) && shared.count(e.at("port").get<std::string>())) touchesShared = true;
        if (touchesShared) {
            // One rail for every phase: the shared port once, each phase's component endpoints.
            json merged = c; merged["endpoints"] = json::array();
            for (const auto& e : c.at("endpoints")) if (is_port_endpoint(e)) merged["endpoints"].push_back(e);
            for (int k = 1; k <= N; ++k)
                for (const auto& e : c.at("endpoints"))
                    if (!is_port_endpoint(e)) merged["endpoints"].push_back(phase_endpoint(e, k, shared));
            conns.push_back(merged);
        } else {
            for (int k = 1; k <= N; ++k) {
                json q = c; q["name"] = sfx(c.at("name").get<std::string>(), k); q["endpoints"] = json::array();
                for (const auto& e : c.at("endpoints")) q["endpoints"].push_back(phase_endpoint(e, k, shared));
                conns.push_back(q);
            }
        }
    }
    json newBrick = brick;
    newBrick["ports"] = ports; newBrick["components"] = comps; newBrick["connections"] = conns;
    (*stage)["circuit"] = newBrick;

    // Inter-stage wiring may only use the shared rails: a per-phase port reached from outside has no single
    // replacement, so it is refused rather than guessed.
    for (const auto& isc : out.at("topology").value("interStageConnections", json::array()))
        for (const auto& e : isc.at("endpoints"))
            if (e.value("stage", "") == stageName && !shared.count(e.at("port").get<std::string>()))
                throw std::invalid_argument("interleave_stage: inter-stage connection '" + isc.value("name", "")
                    + "' reaches per-phase port '" + e.at("port").get<std::string>() + "' of stage '" + stageName + "'");
    if (out.contains("simulation") && out.at("simulation").contains("initialConditions"))
        for (const auto& ic : out.at("simulation").at("initialConditions"))
            if (ic.value("stage", "") == stageName)
                throw std::invalid_argument("interleave_stage: an initial condition on stage '" + stageName
                                            + "' cannot be replicated per phase");

    if (out.contains("simulation") && out.at("simulation").contains("stimulus")) {
        json stim = json::array();
        for (const auto& s : out.at("simulation").at("stimulus")) {
            if (s.value("stage", "") != stageName) { stim.push_back(s); continue; }
            for (int k = 1; k <= N; ++k) {
                json q = s;
                q["component"] = sfx(s.at("component").get<std::string>(), k);
                const double phase0 = s.at("waveform").value("phase", 0.0);
                q["waveform"]["phase"] = std::fmod(phase0 + 360.0 * (k - 1) / N, 360.0);
                stim.push_back(q);
            }
        }
        out["simulation"]["stimulus"] = stim;
    }
    return out;
}

InterleavedRipple interleaved_capacitor_ripple(const std::vector<double>& time, const std::vector<double>& current,
                                               double period, int N) {
    if (N < 1) throw std::invalid_argument("interleaved_capacitor_ripple: phase count must be >= 1");
    if (time.size() != current.size() || time.size() < 2)
        throw std::invalid_argument("interleaved_capacitor_ripple: need a time and a current sample per point");
    if (!(period > 0)) throw std::invalid_argument("interleaved_capacitor_ripple: period must be > 0");
    // Sample one phase on a uniform grid, M points per period with M a multiple of N so the stagger is exact.
    const int perPhase = 512, M = perPhase * N;
    std::vector<double> one(M);
    std::size_t j = 0;
    for (int m = 0; m < M; ++m) {
        const double t = period * (m + 0.5) / M;   // mid-sample: never lands on a step's repeated time stamp
        while (j + 1 < time.size() && time[j + 1] <= t) ++j;
        if (j + 1 >= time.size()) { one[m] = current.back(); continue; }
        const double dt = time[j + 1] - time[j];
        one[m] = dt > 0 ? current[j] + (t - time[j]) / dt * (current[j + 1] - current[j]) : current[j + 1];
    }
    std::vector<double> sum(M, 0.0);
    for (int k = 0; k < N; ++k)
        for (int m = 0; m < M; ++m) sum[m] += one[(m + k * perPhase) % M];
    double mean = 0;
    for (double x : sum) mean += x;
    mean /= M;
    double q = 0, qMin = 0, qMax = 0, acc2 = 0;
    const double h = period / M;
    for (int m = 0; m < M; ++m) {
        const double ac = sum[m] - mean;
        q += ac * h;
        qMin = std::min(qMin, q); qMax = std::max(qMax, q);
        acc2 += ac * ac;
    }
    return {qMax - qMin, std::sqrt(acc2 / M)};
}

int phase_count(const json& config, const char* who) {
    const double raw = cfg::get(config, "phaseCount", 1.0);
    const int n = static_cast<int>(std::llround(raw));
    if (static_cast<double>(n) != raw)
        throw std::invalid_argument(std::string(who) + ": config.phaseCount must be a whole number, got " + std::to_string(raw));
    if (n < 1 || n > 6)
        throw std::invalid_argument(std::string(who) + ": config.phaseCount must be in [1, 6], got " + std::to_string(n));
    return n;
}

} // namespace Kirchhoff
