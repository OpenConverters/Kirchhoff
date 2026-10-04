// Flying-capacitor multilevel buck / boost (ABT #1439), config.levels on the ordinary buck / boost. No MKF model
// exists, so it is validated standalone: the exact inductor waveform against the plain converter (N = 2) and the
// (N−1)·fs / Vtop/(N−1) scaling, the design rules, the TAS (pairs, flying capacitors, phase-shifted drive), and in
// ngspice for N = 3, 4 on both: the output is delivered, the inductor current follows the solver, and every flying
// capacitor sits at k·Vtop/(N−1) — the natural balance of phase-shifted PWM.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include "Buck.hpp"
#include "Boost.hpp"
#include "Fcml.hpp"
#include "TasAssembler.hpp"
#include "NgspiceRunner.hpp"
#include "Fidelity.hpp"

using nlohmann::json;
using Catch::Matchers::WithinRel;

namespace {

json spec(double vin, double vout, double power, double fsw, json config = json::object()) {
    json s;
    s["designRequirements"]["efficiency"] = 1.0;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({{{"name", "out"}, {"voltage", {{"nominal", vout}}}}});
    s["operatingPoints"] = json::array({{{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}}});
    if (!config.empty()) s["config"] = config;
    return s;
}

std::string num(double v) { std::ostringstream o; o.precision(10); o << v; return o.str(); }

std::vector<double> resample(const std::vector<double>& time, const std::vector<double>& val,
                             double tBeg, double period, int N) {
    std::vector<double> out(N, 0.0);
    size_t j = 0;
    for (int k = 0; k < N; ++k) {
        const double t = tBeg + period * k / N;
        while (j + 1 < time.size() && time[j + 1] < t) ++j;
        if (j + 1 >= time.size()) { out[k] = val.back(); continue; }
        const double f = (time[j + 1] > time[j]) ? (t - time[j]) / (time[j + 1] - time[j]) : 0.0;
        out[k] = val[j] + f * (val[j + 1] - val[j]);
    }
    return out;
}

double phase_invariant_nrmse(const std::vector<double>& a, const std::vector<double>& b) {
    const int N = static_cast<int>(b.size());
    // The ripple SHAPE: both means removed (the mean is checked on its own against the simulated load) and the error
    // normalised to the ripple's rms, so the mean alone can neither hide nor fake a wrong shape.
    double ma = 0, mb = 0;
    for (int k = 0; k < N; ++k) { ma += a[k] / N; mb += b[k] / N; }
    double ss = 0;
    for (double x : b) ss += (x - mb) * (x - mb);
    const double ac = std::sqrt(ss / N);
    double best = std::numeric_limits<double>::infinity();
    for (int s = 0; s < N; ++s) {
        double acc = 0;
        for (int k = 0; k < N; ++k) { const double d = (a[(k + s) % N] - ma) - (b[k] - mb); acc += d * d; }
        best = std::min(best, std::sqrt(acc / N) / ac);
    }
    return best;
}

const std::vector<double>* find_vector(const Kirchhoff::NgspiceRunResult& r, std::vector<std::string> needles) {
    for (const auto& kv : r.vectors) {
        std::string k = kv.first;
        std::transform(k.begin(), k.end(), k.begin(), ::tolower);
        bool all = true;
        for (const auto& n : needles) if (k.find(n) == std::string::npos) all = false;
        if (all) return &kv.second;
    }
    return nullptr;
}

const json* stage_brick(const json& tas, const char* name) {
    for (const auto& st : tas.at("topology").at("stages")) if (st.at("name") == name) return &st.at("circuit");
    return nullptr;
}

double ripple_of(const MAS::OperatingPoint& op) { return Kirchhoff::analytical::winding_current(op, 0, "peakToPeak"); }

struct Case { bool boost; double vin, vout, power, fsw; };
const Case kBuck{false, 48.0, 12.0, 120.0, 200e3};
const Case kBoost{true, 200.0, 500.0, 500.0, 100e3};

json tas_of(const Case& c, int levels) {
    const json s = spec(c.vin, c.vout, c.power, c.fsw, {{"levels", levels}});
    return c.boost ? Kirchhoff::build_boost_tas(Kirchhoff::design_boost(s)) : Kirchhoff::build_buck_tas(Kirchhoff::design_buck(s));
}

} // namespace

TEST_CASE("FCML inductor: N = 2 is the plain converter; N levels step Vtop/(N-1) at (N-1)·fs", "[fcml][solver]") {
    namespace AN = Kirchhoff::analytical;
    const double L = 10e-6, fs = 200e3, T = 1 / fs;
    // N = 2 buck: ΔI = Vin·D(1−D)T/L.
    CHECK_THAT(ripple_of(AN::analytical_fcml(false, 2, 48, 12, 10, fs, L)), WithinRel(48 * 0.25 * 0.75 * T / L, 1e-6));
    // N = 3 buck at D = 0.25: sub-period T/2, step 24 V, sub-duty 0.5 → ΔI = 24·0.5·0.5·(T/2)/L.
    CHECK_THAT(ripple_of(AN::analytical_fcml(false, 3, 48, 12, 10, fs, L)), WithinRel(24 * 0.25 * (T / 2) / L, 1e-6));
    // N = 4 boost, 200 → 400 V (D = 0.5): step 400/3, D·3 = 1.5 → sub-duty 0.5 over T/3.
    CHECK_THAT(ripple_of(AN::analytical_fcml(true, 4, 200, 400, 2, fs, L)), WithinRel((400.0 / 3) * 0.25 * (T / 3) / L, 1e-6));
    // The mean is the load (buck) / the input current (boost).
    CHECK_THAT(AN::winding_current(AN::analytical_fcml(true, 3, 200, 400, 2, fs, L), 0, "offset"), WithinRel(4.0, 1e-3));
    CHECK_THROWS(AN::analytical_fcml(false, 3, 48, 12, 0.01, fs, L));   // discontinuous
    CHECK_THROWS(AN::analytical_fcml(true, 3, 48, 12, 1, fs, L));       // a boost cannot step down
}

TEST_CASE("FCML design: (N-1)² smaller inductor, flying capacitors sized, contradictions refused", "[fcml][design]") {
    using Catch::Matchers::ContainsSubstring;
    const auto plain = Kirchhoff::design_buck(spec(48, 12, 120, 200e3, {{"rectifier", "synchronous"}}));
    for (int n : {3, 4}) {
        INFO("levels " << n);
        const auto d = Kirchhoff::design_buck(spec(48, 12, 120, 200e3, {{"levels", n}}));
        CHECK(d.synchronousRectifier);
        CHECK(d.flyingCapacitance > 0);
        // Closed form, independent of the solver's integration: within a sub-period T/(N−1) the switch node steps
        // Vin/(N−1) with sub-duty δ = frac(D·(N−1)), so ΔI·L = (Vin/(N−1))·δ(1−δ)·T/(N−1); L is sized for its worst
        // over the input range at the ripple ratio (0.4 of Io).
        double worst = 0;
        for (int i = 0; i <= 400; ++i) {
            const double vin = 43.2 + (52.8 - 43.2) * i / 400.0, x = 12.0 / vin * (n - 1), delta = x - std::floor(x);
            worst = std::max(worst, vin / (n - 1) * delta * (1 - delta) / (200e3 * (n - 1)));
        }
        CHECK_THAT(d.inductance, WithinRel(worst / (0.4 * 10.0), 0.01));
        // and it is far below the plain buck's (which sees Vin_max·D(1−D)·T at its worst)
        CHECK(d.inductance < plain.inductance / (n - 1));
        const auto b = Kirchhoff::design_boost(spec(200, 400, 400, 100e3, {{"levels", n}}));
        CHECK(b.flyingCapacitance > 0);
    }
    // At efficiency η the cell switches D = Vo/(η·Vin): L is sized from that waveform (the lossless cell's output
    // Vo/η), the one the deck runs — not from Vo/Vin.
    {
        json s = spec(48, 12, 120, 200e3, {{"levels", 3}});
        s["designRequirements"]["efficiency"] = 0.75;   // far enough from 1 that sizing from Vo would differ by ~4 %
        const auto d = Kirchhoff::design_buck(s);
        CHECK_THAT(d.dutyCycle, WithinRel(12.0 / (0.75 * 48.0), 1e-9));
        double worst = 0;
        for (int i = 0; i <= 400; ++i) {
            const double vin = 43.2 + (52.8 - 43.2) * i / 400.0, x = 12.0 / 0.75 / vin * 2, delta = x - std::floor(x);
            worst = std::max(worst, vin / 2 * delta * (1 - delta) / (200e3 * 2));
        }
        CHECK_THAT(d.inductance, WithinRel(worst / (0.4 * 10.0), 0.01));
    }
    // A fixed input where D·(N−1) is whole has no ripple to size L from: refused, not L = 0.
    {
        json s = spec(48, 12, 120, 200e3, {{"levels", 5}});
        s["designRequirements"]["inputVoltage"] = {{"minimum", 48.0}, {"nominal", 48.0}, {"maximum", 48.0}};
        CHECK_THROWS_WITH(Kirchhoff::design_buck(s), ContainsSubstring("ripple vanishes"));
    }
    CHECK_THROWS_WITH(Kirchhoff::design_buck(spec(48, 12, 120, 200e3, {{"levels", 3}, {"rectifier", "diode"}})), ContainsSubstring("synchronous"));
    CHECK_THROWS_WITH(Kirchhoff::design_buck(spec(48, 12, 120, 200e3, {{"levels", 3}, {"phaseCount", 2}})), ContainsSubstring("interleaved"));
    CHECK_THROWS(Kirchhoff::design_boost(spec(200, 400, 400, 100e3, {{"levels", 7}})));
    CHECK_THROWS(Kirchhoff::design_boost(spec(200, 400, 400, 100e3, {{"levels", 2.5}})));
}

TEST_CASE("FCML TAS: N-1 pairs, N-2 flying capacitors, phase-shifted complementary drive", "[fcml][tas]") {
    for (const Case& c : {kBuck, kBoost})
        for (int n : {3, 4}) {
            INFO((c.boost ? "boost" : "buck") << " N=" << n);
            const json tas = tas_of(c, n);
            const json* cell = stage_brick(tas, "switchingCell");
            REQUIRE(cell != nullptr);
            int q = 0, cf = 0;
            for (const auto& comp : cell->at("components")) {
                const std::string name = comp.at("name");
                if (name.rfind("QH", 0) == 0 || name.rfind("QL", 0) == 0) ++q;
                if (name.rfind("Cf", 0) == 0) ++cf;
            }
            CHECK(q == 2 * (n - 1));
            CHECK(cf == n - 2);
            const auto& stim = tas.at("simulation").at("stimulus");
            REQUIRE(stim.size() == static_cast<size_t>(2 * (n - 1)));
            for (int k = 1; k < n; ++k) {
                const auto& mainSw = stim.at(2 * (k - 1));
                CHECK(mainSw.at("component") == std::string(c.boost ? "QL" : "QH") + std::to_string(k));
                CHECK_THAT(mainSw.at("waveform").at("phase").get<double>() + 1e-9, WithinRel((k - 1) * 360.0 / (n - 1) + 1e-9, 1e-9));
            }
            CHECK(tas.at("simulation").contains("initialConditions") == c.boost);
        }
}

TEST_CASE("FCML in ngspice: Vout, inductor current as solved, flying capacitors balanced", "[fcml][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    namespace AN = Kirchhoff::analytical;
    for (const Case& c : {kBuck, kBoost})
        for (int n : {3, 4}) {
            const std::string label = std::string(c.boost ? "boost" : "buck") + " N=" + std::to_string(n);
            INFO(label);
            const json s = spec(c.vin, c.vout, c.power, c.fsw, {{"levels", n}});
            double L, D;
            json tas;
            if (c.boost) { const auto d = Kirchhoff::design_boost(s); L = d.inductance; D = d.dutyCycle; tas = Kirchhoff::build_boost_tas(d); }
            else { const auto d = Kirchhoff::design_buck(s); L = d.inductance; D = d.dutyCycle; tas = Kirchhoff::build_buck_tas(d); }
            PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
            std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
            // Both run long. The buck settles its flying capacitors itself; the boost has no natural balancing (its
            // imbalance mode faces only the lossless inductor and the stiff source) and with equal duties the output's
            // switching ripple walks it ~1 V/ms, so its deck switches the balanced-orbit duty trims a balancing
            // controller settles on and starts on that orbit (fcml_balanced_orbit, ABT #1681).
            const double T = 1.0 / c.fsw, tstop = 3000.0 * T, tstep = T / 400.0;
            deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                      ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep) + (c.boost ? " uic" : ""));
            const Kirchhoff::NgspiceRunResult res = Kirchhoff::run_ngspice_in_process(deck);
            INFO("ngspice error: " << res.error);
            REQUIRE(res.success);
            const double avgWin = 40.0 * T;
            const auto vo = res.average("v(Vout)", tstop - avgWin, tstop);
            REQUIRE(vo.has_value());
            const auto* il = find_vector(res, {"l1", "#branch"});
            REQUIRE(il != nullptr);
            const MAS::OperatingPoint op = AN::analytical_fcml(c.boost, n, c.vin, c.vout, c.power / c.vout, c.fsw, L);
            const auto an = AN::winding_current_samples(op, 0, T);
            const std::vector<double> sim = resample(res.time, *il, tstop - T, T, 400);
            const std::vector<double> ana = resample(an.first, an.second, 0.0, T, 400);
            const double nrmse = std::min(phase_invariant_nrmse(ana, sim), [&] {
                std::vector<double> neg(sim); for (double& x : neg) x = -x; return phase_invariant_nrmse(ana, neg); }());
            const double ripSim = *std::max_element(sim.begin(), sim.end()) - *std::min_element(sim.begin(), sim.end());
            const double ripAn = ripple_of(op);
            const double vtop = c.boost ? c.vout : c.vin;
            std::cerr << "[fcml " << label << "] L=" << L << " D=" << D << " Vout=" << *vo << "  ripple-NRMSE=" << nrmse
                      << "  ΔIL sim/an " << ripSim << "/" << ripAn;
            // The ideal deck's switches still have an on-resistance, N−1 of them in series at every instant: the
            // open-loop output is the lossless one less that drop on the inductor's mean current.
            std::smatch m;
            REQUIRE(std::regex_search(deck, m, std::regex(R"(SW_QH1 SW\([^)]*Ron=([0-9.eE+-]+))")));
            const double ron = std::stod(m[1].str());
            REQUIRE(ron > 0);
            const double iL = c.boost ? c.power / c.vin : c.power / c.vout, drop = (n - 1) * ron * iL;
            const double voExpected = c.boost ? (c.vin - drop) / (1.0 - D) : D * c.vin - drop;
            std::cerr << " (expected " << voExpected << ")";
            CHECK_THAT(*vo, WithinRel(voExpected, 0.005));
            // The inductor's mean is the load the simulated output draws (buck), or that power from the input (boost).
            double simMean = 0;
            for (double x : sim) simMean += x / sim.size();
            const double rLoad = c.vout * c.vout / c.power;
            CHECK_THAT(std::fabs(simMean), WithinRel(c.boost ? *vo * *vo / rLoad / c.vin : *vo / rLoad, 0.01));
            CHECK(nrmse < 0.08);
            CHECK_THAT(ripSim, WithinRel(ripAn, 0.08));
            for (int k = 1; k < n - 1; ++k) {
                const auto vh = res.average("v(xswitchingcell.nh" + std::to_string(k) + ")", tstop - avgWin, tstop);
                const auto vl = res.average("v(xswitchingcell.nl" + std::to_string(k) + ")", tstop - avgWin, tstop);
                REQUIRE(vh.has_value());
                REQUIRE(vl.has_value());
                std::cerr << "  Vcf" << k << "=" << (*vh - *vl);
                CHECK_THAT(*vh - *vl, WithinRel(k * vtop / (n - 1), 0.03));
            }
            std::cerr << "\n";
        }
}

TEST_CASE("FCML balanced orbit: periodic, flying capacitors at their levels, trims sum to zero", "[fcml][orbit]") {
    namespace AN = Kirchhoff::analytical;
    for (const Case& c : {kBuck, kBoost})
        for (int n : {3, 4}) {
            INFO((c.boost ? "boost" : "buck") << " N=" << n);
            const json s = spec(c.vin, c.vout, c.power, c.fsw, {{"levels", n}});
            double L, Cf, Cout, D;
            if (c.boost) { const auto d = Kirchhoff::design_boost(s); L = d.inductance; Cf = d.flyingCapacitance; Cout = d.outputCapacitance; D = d.dutyCycle; }
            else { const auto d = Kirchhoff::design_buck(s); L = d.inductance; Cf = d.flyingCapacitance; Cout = d.outputCapacitance; D = d.dutyCycle; }
            const double R = c.vout * c.vout / c.power;
            const AN::FcmlOrbit o = AN::fcml_balanced_orbit(c.boost, n, c.vin, D, R, c.fsw, L, Cf, Cout);
            REQUIRE(o.dutyTrims.size() == static_cast<size_t>(n - 1));
            double sum = 0;
            for (double t : o.dutyTrims) { sum += t; CHECK(std::fabs(t) < 0.05); }
            CHECK(std::fabs(sum) < 1e-12);
            // the orbit's output is the lossless one at D, and each flying capacitor starts near its level
            CHECK_THAT(o.outputVoltage, WithinRel(c.boost ? c.vin / (1 - D) : D * c.vin, 0.03));
            const double vtop = c.boost ? c.vout : c.vin;
            for (int k = 1; k < n - 1; ++k) CHECK_THAT(o.flyingCapacitorVoltages[k - 1], WithinRel(k * vtop / (n - 1), 0.06));
        }
    CHECK_THROWS(AN::fcml_balanced_orbit(true, 2, 200, 0.6, 500, 100e3, 1e-3, 1e-6, 1e-6));
}
