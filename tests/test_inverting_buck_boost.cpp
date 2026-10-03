// Inverting buck-boost (ABT #1433). No MKF model exists for this topology, so it is validated standalone:
// closed-form design checks, then the analytical inductor waveform against ngspice (phase-invariant NRMSE,
// the same measure test_nrmse_gate uses) and the simulated output rail against the requested −|Vo|.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include "InvertingBuckBoost.hpp"
#include "ConverterAnalytical.hpp"
#include "TasAssembler.hpp"
#include "NgspiceRunner.hpp"
#include "Fidelity.hpp"

using nlohmann::json;
using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

namespace {

json spec(double vin, double vout, double power, double fsw, json config = json::object(), double eta = 0.95) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.8}, {"nominal", vin}, {"maximum", vin * 1.2}};
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
    double ss = 0;
    for (double x : b) ss += x * x;
    const double rmsB = std::sqrt(ss / N);
    double best = std::numeric_limits<double>::infinity();
    for (int s = 0; s < N; ++s) {
        double acc = 0;
        for (int k = 0; k < N; ++k) { const double d = a[(k + s) % N] - b[k]; acc += d * d; }
        best = std::min(best, std::sqrt(acc / N) / rmsB);
    }
    return best;
}

struct SimResult { double vout; double nrmse; double simPeak; double anPeak; double simPP; double anPP; };

// Simulate the design's own deck to steady state; compare the L1 current over the last period with the
// analytical waveform at the deck's operating point, and average the output rail.
SimResult simulate(const Kirchhoff::InvertingBuckBoostDesign& d) {
    const json tas = Kirchhoff::build_inverting_buck_boost_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    const double period = 1.0 / d.switchingFrequency;
    const double tstop = 2500.0 * period, tstep = period / 400.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    INFO("ngspice error: " << r.error);
    REQUIRE(r.success);
    const auto vo = r.average("v(Vout)", tstop - 40.0 * period, tstop);
    REQUIRE(vo.has_value());

    std::string key;
    for (const auto& kv : r.vectors)
        if (kv.first.find("ll1") != std::string::npos && kv.first.find("#branch") != std::string::npos) key = kv.first;
    INFO("no L1 branch-current vector in the ngspice result");
    REQUIRE(!key.empty());

    const double iout = d.outputPower / d.outputVoltageMag;
    const MAS::OperatingPoint op = Kirchhoff::analytical::analytical_inverting_buck_boost(
        d.inputVoltage, d.outputVoltageMag, iout, d.switchingFrequency, d.inductance, d.diodeDrop, d.efficiency);
    // Copies: get_current()/get_waveform() return optionals BY VALUE — a reference into them dangles.
    const auto current = op.get_excitations_per_winding().at(0).get_current();
    REQUIRE(current.has_value());
    const auto waveform = current->get_waveform();
    REQUIRE(waveform.has_value());
    const MAS::Waveform wf = *waveform;
    const std::vector<double>& aVal = wf.get_data();
    std::vector<double> aTime;
    if (wf.get_time() && wf.get_time()->size() == aVal.size()) aTime = *wf.get_time();
    else for (size_t i = 0; i < aVal.size(); ++i) aTime.push_back(period * i / (aVal.size() - 1));

    const int N = 400;
    const std::vector<double> a = resample(aTime, aVal, 0.0, period, N);
    std::vector<double> s = resample(r.time, r.vectors.at(key), tstop - period, period, N);
    // The deck's branch current runs primary_start -> primary_end (sw -> gnd), the analytical winding
    // reference; a sign flip would be a wiring error, so it is NOT folded away here.
    auto mean = [](const std::vector<double>& v) { double m = 0; for (double x : v) m += x; return m / v.size(); };
    std::cerr << "  L1 sim min/mean/max " << *std::min_element(s.begin(), s.end()) << "/" << mean(s) << "/"
              << *std::max_element(s.begin(), s.end()) << "  analytical " << *std::min_element(a.begin(), a.end())
              << "/" << mean(a) << "/" << *std::max_element(a.begin(), a.end()) << "\n";
    auto pp = [](const std::vector<double>& v) { return *std::max_element(v.begin(), v.end()) - *std::min_element(v.begin(), v.end()); };
    return {*vo, phase_invariant_nrmse(a, s), *std::max_element(s.begin(), s.end()),
            *std::max_element(a.begin(), a.end()), pp(s), pp(a)};
}

} // namespace

TEST_CASE("inverting buck-boost: CCM duty, inductance and a negative rail in the TAS", "[ibb][design]") {
    const auto d = Kirchhoff::design_inverting_buck_boost(spec(24, -12, 24, 200e3));
    CHECK_FALSE(d.dcm);
    CHECK_THAT(d.outputVoltageMag, WithinRel(12.0, 1e-12));
    const double vOff = 12.0 + d.diodeDrop;
    CHECK_THAT(d.dutyCycle, WithinRel(vOff / (24.0 * 0.95 + vOff), 1e-9));
    CHECK(d.inductance > d.criticalInductance);   // CCM sizing lands above the boundary

    // A magnitude-stated setpoint designs the same converter.
    const auto m = Kirchhoff::design_inverting_buck_boost(spec(24, 12, 24, 200e3));
    CHECK_THAT(m.inductance, WithinRel(d.inductance, 1e-12));
    CHECK_THAT(m.dutyCycle, WithinRel(d.dutyCycle, 1e-12));

    const json tas = Kirchhoff::build_inverting_buck_boost_tas(d);
    CHECK(tas["inputs"]["designRequirements"]["outputs"][0]["voltage"]["nominal"].get<double>() == -12.0);
}

TEST_CASE("inverting buck-boost: DCM sizes below the critical inductance and refuses contradictions", "[ibb][design]") {
    const auto d = Kirchhoff::design_inverting_buck_boost(spec(24, -12, 24, 200e3, {{"mode", "dcm"}}));
    CHECK(d.dcm);
    CHECK(d.inductance < d.criticalInductance);
    const auto tm = Kirchhoff::analytical::inverting_buck_boost_timing(d.inputVoltageMin, d.outputVoltageMag,
        d.outputPower / d.outputVoltageMag, d.switchingFrequency, d.inductance, d.diodeDrop, d.efficiency);
    CHECK(tm.dcm);   // still dry at the corner where DCM is hardest to hold

    CHECK_THROWS(Kirchhoff::design_inverting_buck_boost(spec(24, -12, 24, 200e3, {{"mode", "dcm"}, {"rectifier", "synchronous"}})));
    CHECK_THROWS(Kirchhoff::design_inverting_buck_boost(spec(24, -12, 24, 200e3, {{"mode", "bcm"}})));
    json two = spec(24, -12, 24, 200e3);
    two["designRequirements"]["outputs"].push_back({{"name", "out2"}, {"voltage", {{"nominal", -5}}}});
    CHECK_THROWS(Kirchhoff::design_inverting_buck_boost(two));
}

TEST_CASE("inverting buck-boost: analytical CCM inductor current matches the closed form", "[ibb][analytical]") {
    const double vin = 24, vo = 12, iout = 2, fsw = 200e3, L = 22e-6;
    const MAS::OperatingPoint op = Kirchhoff::analytical::analytical_inverting_buck_boost(vin, vo, iout, fsw, L);
    const double D = vo / (vin + vo);
    using Kirchhoff::analytical::winding_current;
    // 1 %: the processed values come from the sampled waveform, as in the buck/boost solver tests.
    const double iAvg = iout / (1 - D), ripple = vin * D / (L * fsw);
    CHECK_THAT(winding_current(op, 0, "offset"), WithinRel(iAvg, 0.01));
    CHECK_THAT(winding_current(op, 0, "peakToPeak"), WithinRel(ripple, 0.01));
    CHECK_THAT(winding_current(op, 0, "peak"), WithinRel(iAvg + ripple / 2, 0.01));
}

TEST_CASE("inverting buck-boost: ngspice regulates to -|Vo| and matches the analytical L1 current", "[ibb][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — the simulation half of this gate did not run");
        return;
    }
    struct Case { const char* name; json config; double nrmseMax; };
    // Measured 0.002 (CCM diode), 0.006 (CCM synchronous: the dead-time body-diode intervals), 0.002 (DCM).
    // Gated at 0.03: a wrong duty, slope or mode moves it by an order of magnitude more.
    for (const Case& c : {Case{"ccm diode", json::object(), 0.03},
                          Case{"ccm synchronous", {{"rectifier", "synchronous"}}, 0.03},
                          Case{"dcm", {{"mode", "dcm"}}, 0.03}}) {
        DYNAMIC_SECTION(c.name) {
            const auto d = Kirchhoff::design_inverting_buck_boost(spec(24, -12, 24, 200e3, c.config, 1.0));
            const SimResult s = simulate(d);
            std::cerr << "[ibb " << c.name << "] Vout=" << s.vout << " NRMSE=" << s.nrmse
                      << " Ipk sim/an=" << s.simPeak << "/" << s.anPeak << "\n";
            CHECK(s.vout < 0);
            CHECK_THAT(s.vout, WithinRel(-12.0, 0.05));
            CHECK(s.nrmse < c.nrmseMax);
            // The NRMSE is normalised by an RMS that CCM's DC dominates, so a wrong slope barely moves it;
            // the ripple itself is compared directly.
            CHECK_THAT(s.simPP, WithinRel(s.anPP, 0.05));
        }
    }
}
