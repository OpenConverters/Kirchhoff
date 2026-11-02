// "I know the design I want" duty pin (ABT #2134): config.forcedDutyCycle drives the main switch at exactly that
// duty and the rails it delivers into the requested loads become the result; config.forcedDeadTime sets the
// active-clamp forward's main↔clamp dead time.
//
// Before this, only the old MKF flyback read a desired duty (and broke volt-second balance doing it); Kirchhoff
// derived every duty from the requested output voltage, so a user who stated D = 0.5 for an active-clamp
// forward got D = 0.18 from the pinned turns ratio instead. Checked through the api layer every caller uses.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "KirchhoffApi.hpp"
#include "Psfb.hpp"
#include "IsolatedBuck.hpp"
#include "ConverterAnalytical.hpp"
#include "Pshb.hpp"

using json = nlohmann::json;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using Catch::Matchers::ContainsSubstring;

namespace {

json spec(double vmin, double vnom, double vmax, double vop, const std::vector<std::pair<double, double>>& outs,
          double fsw, const json& extraRequirements, const json& config) {
    json s;
    json& dr = s["designRequirements"];
    dr["inputVoltage"] = {{"minimum", vmin}, {"nominal", vnom}, {"maximum", vmax}};
    dr["switchingFrequency"]["nominal"] = fsw;
    dr["efficiency"] = 0.9;
    json op;
    op["name"] = "op";
    op["inputVoltage"] = vop;
    op["ambientTemperature"] = 25.0;
    for (size_t i = 0; i < outs.size(); ++i) {
        json o;
        o["name"] = "out" + std::to_string(i);
        o["voltage"]["nominal"] = outs[i].first;
        o["power"]["nominal"] = outs[i].second;
        dr["outputs"].push_back(o);
        json oo;
        oo["name"] = o["name"];
        oo["voltage"] = outs[i].first;
        oo["power"] = outs[i].second;
        op["outputs"].push_back(oo);
    }
    for (const auto& [k, v] : extraRequirements.items()) dr[k] = v;
    s["operatingPoints"] = json::array({op});
    if (!config.empty()) s["config"] = config;
    return s;
}

// The customer's active-clamp forward: Vin 24..125 V (74.5 nominal), Np:Ns = 1, Lm = 50 µH, 13 V / 24 A, 140 kHz.
json customer_acf(double vop, const json& config) {
    json pinned;
    pinned["magnetizingInductance"]["nominal"] = 50e-6;
    pinned["turnsRatios"] = json::array({json{{"nominal", 1.0}}});
    return spec(24.0, 74.5, 125.0, vop, {{13.0, 13.0 * 24.0}}, 140e3, pinned, config);
}

json process(const std::string& topology, const json& s) {
    const std::string out = Kirchhoff::api::process_converter(topology, s.dump(), "analytical");
    INFO(out.substr(0, 400));
    REQUIRE(out.rfind("Exception", 0) != 0);
    return json::parse(out);
}

std::string refusal(const std::string& topology, const json& s) {
    const std::string out = Kirchhoff::api::process_converter(topology, s.dump(), "analytical");
    INFO(out.substr(0, 400));
    REQUIRE(out.rfind("Exception", 0) == 0);
    return out;
}

double stimulus_duty(const json& result, const std::string& component) {
    for (const auto& st : result.at("tas").at("simulation").at("stimulus"))
        if (st.at("component") == component) return st.at("waveform").at("dutyCycle").get<double>();
    FAIL("no stimulus for " << component);
    return -1.0;
}

// The duty the model ran the switch at, as diagnostics reports it: the analytical solver's exact duty for the
// main magnetic, not a measurement of the waveform (every waveform in the result is resampled to 128 points,
// which quantised a measured 0.5 to 0.5039).
double analytical_primary_duty(const json& result) {
    return result.at("diagnostics").at("dutyCycle").get<double>();
}

const json& component(const json& result, const std::string& name) {
    for (const auto& stage : result.at("tas").at("topology").at("stages"))
        if (stage.contains("circuit") && stage.at("circuit").contains("components"))
            for (const auto& c : stage.at("circuit").at("components"))
                if (c.at("name") == name) return c;
    FAIL("no component " << name);
    static const json none;
    return none;
}

}  // namespace

TEST_CASE("Customer ACF at a pinned duty of 0.5 runs the deck and the model at 0.5, Vo = D·Vin/n − Vd",
          "[duty-pin][acf]") {
    const double R = 13.0 * 13.0 / 312.0;
    const double vd = 24.0 * 0.01;   // synchronous-rectifier drop Io·Rds of the requested load (design_acf)
    for (const double vin : {24.0, 74.5, 125.0}) {
        CAPTURE(vin);
        const json r = process("acf", customer_acf(vin, json{{"forcedDutyCycle", 0.5}}));
        CHECK(stimulus_duty(r, "Q1") == 0.5);
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(0.5, 1e-9));
        const json& d = r.at("diagnostics");
        CHECK(d.at("forcedDutyCycle").get<double>() == 0.5);
        REQUIRE(d.at("outputs").size() == 1);
        const json& o = d.at("outputs").at(0);
        CHECK(o.at("name") == "out0");
        CHECK(o.at("requestedVoltage").get<double>() == 13.0);
        const double vo = 0.5 * vin / 1.0 - vd;
        CHECK_THAT(o.at("voltage").get<double>(), WithinRel(vo, 1e-9));
        CHECK_THAT(o.at("power").get<double>(), WithinRel(vo * vo / R, 1e-9));
        // The TAS carries the delivered rail, so the deck's synthesized load stays the requested R.
        const json& tasOut = r.at("tas").at("inputs").at("operatingPoints").at(0).at("outputs").at(0);
        CHECK_THAT(tasOut.at("power").get<double>(), WithinRel(vo * vo / R, 1e-9));
    }
}

TEST_CASE("Without the pin the ACF still derives its duty from the requested rail", "[duty-pin][acf]") {
    const json r = process("acf", customer_acf(74.5, json::object()));
    CHECK_THAT(stimulus_duty(r, "Q1"), WithinRel((13.0 + 0.24) * 1.0 / 74.5, 1e-12));
    CHECK_FALSE(r.at("diagnostics").contains("outputs"));
    CHECK_FALSE(r.at("diagnostics").contains("forcedDutyCycle"));
}

TEST_CASE("A pinned ACF rates the main switch for the heavier of the nominal and Vin_min corners",
          "[duty-pin][acf]") {
    // At 125 V and D = 0.5 the open-loop rail is ~62 V into the 0.54 Ω load: far more primary current than
    // the regulated 13 V Vin_min corner the ratings were taken at before.
    const json r = process("acf", customer_acf(125.0, json{{"forcedDutyCycle", 0.5}}));
    const json& wf = r.at("operatingPoint").at("excitationsPerWinding").at(0).at("current").at("waveform");
    double peak = 0.0;
    for (double v : wf.at("data").get<std::vector<double>>()) peak = std::max(peak, v);
    const double rated = component(r, "Q1").at("data").at("inputs").at("designRequirements")
                             .at("ratedContinuousDrainCurrent").get<double>();
    CHECK(rated >= peak * (1.0 - 1e-9));
}

TEST_CASE("ACF forced dead time sets the clamp-switch timing", "[duty-pin][acf]") {
    const double D = 0.45, td = 200e-9, fsw = 140e3;
    const json r = process("acf", customer_acf(24.0, json{{"forcedDutyCycle", D}, {"forcedDeadTime", td}}));
    CHECK(stimulus_duty(r, "Q1") == D);
    CHECK_THAT(stimulus_duty(r, "Sc"), WithinRel((1.0 - D) - 2.0 * td * fsw, 1e-12));
    // Dead time alone (no duty pin) is honoured too.
    const json r2 = process("acf", customer_acf(74.5, json{{"forcedDeadTime", td}}));
    const double D2 = stimulus_duty(r2, "Q1");
    CHECK_THAT(stimulus_duty(r2, "Sc"), WithinRel((1.0 - D2) - 2.0 * td * fsw, 1e-12));
}

TEST_CASE("Every duty-pinnable topology runs its deck and its model at the pinned duty", "[duty-pin]") {
    struct Case { std::string topology; double duty; std::vector<std::pair<double, double>> outs; json config; };
    const std::vector<Case> cases{
        {"forward", 0.3, {{12.0, 60.0}}, json::object()},
        {"forward", 0.3, {{12.0, 60.0}, {5.0, 10.0}}, json::object()},
        {"two_switch_forward", 0.3, {{12.0, 60.0}}, json::object()},
        {"acf", 0.3, {{12.0, 60.0}, {5.0, 10.0}}, json::object()},
        {"push_pull", 0.3, {{12.0, 60.0}}, json::object()},
        {"push_pull", 0.3, {{12.0, 60.0}, {5.0, 10.0}}, json::object()},
        {"flyback", 0.3, {{12.0, 60.0}}, json::object()},
        {"flyback", 0.3, {{12.0, 60.0}, {5.0, 10.0}}, json::object()},
        {"flyback", 0.3, {{12.0, 20.0}}, json{{"mode", "dcm"}}},
        {"isolated_buck_boost", 0.3, {{12.0, 20.0}, {12.0, 10.0}}, json::object()},
        {"isolated_buck_boost", 0.3, {{12.0, 20.0}, {12.0, 10.0}, {5.0, 5.0}}, json::object()},
    };
    for (const auto& c : cases) {
        CAPTURE(c.topology, c.outs.size(), c.config.dump());
        json config = c.config;
        config["forcedDutyCycle"] = c.duty;
        const json r = process(c.topology, spec(36.0, 48.0, 72.0, 48.0, c.outs, 100e3, json::object(), config));
        const std::string sw = (c.topology == "isolated_buck_boost") ? "QS1" : "Q1";
        CHECK(stimulus_duty(r, sw) == c.duty);
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(c.duty, 1e-9));
        const json& d = r.at("diagnostics");
        CHECK(d.at("forcedDutyCycle").get<double>() == c.duty);
        REQUIRE(d.at("outputs").size() == c.outs.size());
        for (size_t i = 0; i < c.outs.size(); ++i) {
            const json& o = d.at("outputs").at(i);
            CHECK(o.at("requestedVoltage").get<double>() == c.outs[i].first);
            CHECK(o.at("voltage").get<double>() > 0.0);
            // The requested load resistance is held: P = V²/R.
            const double R = c.outs[i].first * c.outs[i].first / c.outs[i].second;
            CHECK_THAT(o.at("power").get<double>(),
                       WithinRel(o.at("voltage").get<double>() * o.at("voltage").get<double>() / R, 1e-9));
        }
    }
}

TEST_CASE("Forward-family extra rails follow the primary volt-seconds at the pinned duty", "[duty-pin]") {
    const json r = process("forward", spec(36.0, 48.0, 72.0, 48.0, {{12.0, 60.0}, {5.0, 10.0}}, 100e3,
                                           json::object(), json{{"forcedDutyCycle", 0.3}}));
    const json& t1 = component(r, "T1").at("data").at("inputs").at("designRequirements").at("turnsRatios");
    const double n1 = t1.at(2).contains("nominal") ? t1.at(2).at("nominal").get<double>()
                                                   : t1.at(2).at("maximum").get<double>();
    const double v1 = r.at("diagnostics").at("outputs").at(1).at("voltage").get<double>();
    // Vo_1 = D·Vin/n_1 − Vd_1, so v1 + Vd_1 = D·Vin/n_1 with a small positive drop.
    const double drop = 0.3 * 48.0 / n1 - v1;
    CHECK(drop > 0.0);
    CHECK(drop < 1.5);
}

TEST_CASE("Duty and dead-time pins are refused where they cannot be honoured", "[duty-pin][errors]") {
    const json fwd = spec(36.0, 48.0, 72.0, 48.0, {{12.0, 60.0}}, 100e3, json::object(), json::object());
    auto with = [](json s, const json& config) { s["config"] = config; return s; };

    CHECK_THAT(refusal("buck", with(fwd, json{{"forcedDutyCycle", 0.3}})), ContainsSubstring("Duty Cycle"));
    CHECK_THAT(refusal("llc", with(fwd, json{{"forcedDutyCycle", 0.3}})), ContainsSubstring("Duty Cycle"));
    CHECK_THAT(refusal("forward", with(fwd, json{{"forcedDeadTime", 100e-9}})), ContainsSubstring("Dead Time"));
    CHECK_THAT(refusal("forward", with(fwd, json{{"forcedDutyCycle", 1.2}})), ContainsSubstring("Duty Cycle"));
    CHECK_THAT(refusal("forward", with(fwd, json{{"forcedDutyCycle", 0.0}})), ContainsSubstring("Duty Cycle"));
    CHECK_THAT(refusal("forward", with(fwd, json{{"forcedDutyCycle", "0.4"}})), ContainsSubstring("Duty Cycle"));
    CHECK_THAT(refusal("forward", with(fwd, json{{"forcedDutyCycle", 0.6}})), ContainsSubstring("0.5"));
    CHECK_THAT(refusal("two_switch_forward", with(fwd, json{{"forcedDutyCycle", 0.6}})), ContainsSubstring("0.5"));
    CHECK_THAT(refusal("push_pull", with(fwd, json{{"forcedDutyCycle", 0.5}})), ContainsSubstring("0.5"));
    CHECK_THAT(refusal("flyback", with(fwd, json{{"forcedDutyCycle", 0.3}, {"mode", "qrm"}})),
               ContainsSubstring("quasi-resonant"));
    CHECK_THAT(refusal("acf", customer_acf(24.0, json{{"forcedDeadTime", 100e-9}, {"deadTimeFraction", 0.01}})),
               ContainsSubstring("Dead Time"));
    CHECK_THAT(refusal("acf", customer_acf(24.0, json{{"forcedDeadTime", -1e-9}})), ContainsSubstring("Dead Time"));
    // 0.95 leaves the clamp switch 0.05 of the period minus two 200 ns dead times at 140 kHz (0.056): none.
    CHECK_THAT(refusal("acf", customer_acf(24.0, json{{"forcedDutyCycle", 0.95}, {"forcedDeadTime", 200e-9}})),
               ContainsSubstring("clamp switch"));
    const json ibb = spec(36.0, 48.0, 72.0, 48.0, {{12.0, 20.0}, {12.0, 10.0}}, 100e3, json::object(),
                          json{{"forcedDutyCycle", 0.95}});
    CHECK_THAT(refusal("isolated_buck_boost", ibb), ContainsSubstring("Duty Cycle"));
}

TEST_CASE("Isolated buck runs its deck and its model at the pinned duty, the rails it delivers are results",
          "[duty-pin][isolated-buck]") {
    // Vpri follows the solver's own law D = Vpri/Vin·η, so Vpri = D·Vin/η; each isolated rail is the winding's
    // flyback voltage Vpri/N_k less its rectifier drop.
    const double D = 0.3, vin = 48.0, eta = 0.9;
    for (const auto& outs : {std::vector<std::pair<double, double>>{{12.0, 60.0}, {5.0, 10.0}},
                             std::vector<std::pair<double, double>>{{12.0, 60.0}, {5.0, 10.0}, {3.3, 5.0}}}) {
        CAPTURE(outs.size());
        const json r = process("isolated_buck", spec(36.0, 48.0, 72.0, vin, outs, 100e3, json::object(),
                                                     json{{"forcedDutyCycle", D}}));
        CHECK(stimulus_duty(r, "QS1") == D);
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(D, 1e-9));
        const json& d = r.at("diagnostics");
        CHECK(d.at("forcedDutyCycle").get<double>() == D);
        REQUIRE(d.at("outputs").size() == outs.size());
        const double vpri = d.at("outputs").at(0).at("voltage").get<double>();
        CHECK_THAT(vpri, WithinRel(D * vin / eta, 1e-12));
        const json& ratios = component(r, "T1").at("data").at("inputs").at("designRequirements").at("turnsRatios");
        for (size_t i = 0; i < outs.size(); ++i) {
            CAPTURE(i);
            const json& o = d.at("outputs").at(i);
            CHECK(o.at("requestedVoltage").get<double>() == outs[i].first);
            const double R = outs[i].first * outs[i].first / outs[i].second;
            const double v = o.at("voltage").get<double>();
            CHECK_THAT(o.at("power").get<double>(), WithinRel(v * v / R, 1e-9));
            if (i == 0) continue;
            const json& n = ratios.at(i - 1);
            const double ni = n.contains("nominal") ? n.at("nominal").get<double>() : n.at("maximum").get<double>();
            const double drop = vpri / ni - v;   // the rectifier drop: small and positive
            CHECK(drop >= 0.0);
            CHECK(drop < 1.5);
        }
    }
}

TEST_CASE("Isolated buck duty pins that cannot be honoured are refused", "[duty-pin][isolated-buck][errors]") {
    // η = 0.9: D = 0.95 needs the primary rail above the input voltage.
    CHECK_THAT(refusal("isolated_buck", spec(36.0, 48.0, 72.0, 48.0, {{12.0, 60.0}, {5.0, 10.0}}, 100e3,
                                             json::object(), json{{"forcedDutyCycle", 0.95}})),
               ContainsSubstring("Duty Cycle"));
}

// ── Isolated buck dead time (Alf 2026-11-01: "Kirchhoff takes the dead time", "Use the spec's diode drop") ──────
// config.forcedDeadTime sets the QS1/QS2 dead time on both edges, in the deck and in the model. During a dead time
// the primary current freewheels through a body diode at the spec's config.diodeVoltageDrop: with the magnetizing
// current positive the primary sees -(Vpri + Vf) instead of -Vpri, with it negative the high-side diode conducts and
// the on-time effectively starts td early. The duty moves with those volt-seconds, so the deck still delivers the rail.
namespace {
// 36-72 V (48 V operating point) -> 12 V / 24 W primary rail + 12 V / 12 W isolated rail, 200 kHz, no stated
// efficiency (the model and the deck then share one duty law).
json isolated_buck_dead_time_spec(const json& config) {
    json s = spec(36.0, 48.0, 72.0, 48.0, {{12.0, 24.0}, {12.0, 12.0}}, 200e3, json::object(), config);
    s["designRequirements"].erase("efficiency");
    return s;
}
// Time-weighted mean of a sampled waveform {data, time}.
double waveform_mean(const json& waveform) {
    const auto& v = waveform.at("data");
    const auto& t = waveform.at("time");
    REQUIRE(v.size() == t.size());
    REQUIRE(v.size() >= 2);
    double area = 0.0;
    for (size_t k = 1; k < v.size(); ++k)
        area += 0.5 * (v[k].get<double>() + v[k - 1].get<double>()) * (t[k].get<double>() - t[k - 1].get<double>());
    return area / (t.back().get<double>() - t.front().get<double>());
}
double waveform_rms(const json& waveform) {
    const auto& v = waveform.at("data");
    const auto& t = waveform.at("time");
    double area = 0.0;
    for (size_t k = 1; k < v.size(); ++k) {
        const double a = v[k - 1].get<double>(), b = v[k].get<double>();
        area += (a * a + a * b + b * b) / 3.0 * (t[k].get<double>() - t[k - 1].get<double>());
    }
    return std::sqrt(area / (t.back().get<double>() - t.front().get<double>()));
}
// The value of a sampled waveform at time t (the segment to the right of a jump at t; t == the last sample time is
// that sample).
double waveform_at(const MAS::Waveform& w, double t) {
    const auto& v = w.get_data();
    const std::vector<double> tt = w.get_time().value();   // get_time() returns a copy: no reference into it
    for (size_t k = 0; k + 1 < tt.size(); ++k)
        if (t >= tt[k] && t < tt[k + 1] && tt[k + 1] > tt[k])
            return v[k] + (v[k + 1] - v[k]) * (t - tt[k]) / (tt[k + 1] - tt[k]);
    if (!tt.empty() && t == tt.back()) return v.back();
    FAIL("t = " << t << " is outside the waveform");
    return 0.0;
}
}  // namespace

TEST_CASE("Isolated buck with a dead time: the deck and the model agree, and the deck delivers the rail",
          "[duty-pin][isolated-buck][dead-time][ngspice]") {
    // 250 ns at 200 kHz: 5 % of the period on each edge. The deck's body diodes are DIDEAL (0.83 V at 1 A), so the
    // spec's drop is 0.8 V. Two magnetizing-current regimes: the default 40 % ripple keeps it positive at both dead
    // times; a ripple ratio of 3 takes it negative before the on-time (the high-side diode conducts there).
    for (const double ripple : {0.4, 3.0}) {
        CAPTURE(ripple);
        const json s = isolated_buck_dead_time_spec(
            json{{"forcedDeadTime", 250e-9}, {"diodeVoltageDrop", 0.8}, {"inductorRippleRatio", ripple}});
        const json an = process("isolated_buck", s);
        CHECK_THAT(stimulus_duty(an, "QS2"), WithinAbs(1.0 - stimulus_duty(an, "QS1") - 0.1, 1e-12));
        const double D = stimulus_duty(an, "QS1");
        // The deck is driven at the duty the model runs. At 40 % ripple both dead times freewheel low:
        // 0.25 + 2·0.05·0.8/48. At 300 % the one before the on-time freewheels high for x and idles for td - x (the
        // current reaches zero inside it): 0.25 + (0.8·td - (48 + 0.8)·x - 12·(td - x))·fs/48, x from the model.
        CHECK_THAT(analytical_primary_duty(an), WithinAbs(D, 1e-12));
        const Kirchhoff::IsolatedBuckDesign design = Kirchhoff::design_isolated_buck(s);
        const Kirchhoff::analytical::IsolatedBuckDeadTimes regime = Kirchhoff::analytical::isolated_buck_dead_times(
            0.25, 48.0, 12.0, 2.0 + 1.0, 200e3, design.magnetizingInductance, 250e-9, 0.8);
        CHECK(regime.negative[1] == (ripple > 1.0));
        CHECK((regime.conduction[1] < 250e-9) == (ripple > 1.0));
        const double x = regime.conduction[1];
        CHECK_THAT(D, WithinAbs(ripple < 1.0 ? 0.25 + 2.0 * 0.05 * 0.8 / 48.0
                                             : 0.25 + (0.8 * 250e-9 - 48.8 * x - 12.0 * (250e-9 - x)) * 200e3 / 48.0,
                                1e-12));

        const std::string raw = Kirchhoff::api::process_converter("isolated_buck", s.dump(), "ngspice");
        INFO(raw.substr(0, 400));
        REQUIRE(raw.rfind("Exception:", 0) != 0);
        const json sim = json::parse(raw);
        const auto& simPri = sim.at("operatingPoint").at("excitationsPerWinding").at(0);
        const auto& anPri = sim.at("analyticalWaveforms").at("T1").at("excitationsPerWinding").at(0);
        // The deck delivers the 12 V rail into its 6 ohm load: the primary winding carries the rail current 2 A on
        // average (its capacitor none).
        const double iSim = waveform_mean(simPri.at("current").at("waveform"));
        INFO("deck primary mean " << iSim << " A, rms " << waveform_rms(simPri.at("current").at("waveform"))
             << " A; model rms " << waveform_rms(anPri.at("current").at("waveform")) << " A");
        CHECK_THAT(iSim * 6.0, WithinRel(12.0, 0.02));
        // Alf 2026-11-01, "Drop that check, ticket the model": the primary-current rms is not compared at ripple 3.
        // Deck 2.786 A against model 3.213 A (-13.3 %), and the base model without a dead time is already -9.7 % on
        // the primary and +32 % on the secondary there: with the magnetizing current negative the deck's secondary
        // spikes to -6.7 A and decays, while the model ramps up from Iout. A model gap, not a dead-time one.
        if (ripple < 1.0)
            CHECK_THAT(waveform_rms(simPri.at("current").at("waveform")),
                       WithinRel(waveform_rms(anPri.at("current").at("waveform")), 0.05));
        CHECK_THAT(waveform_rms(simPri.at("voltage").at("waveform")),
                   WithinRel(waveform_rms(anPri.at("voltage").at("waveform")), 0.05));
    }
}

TEST_CASE("Isolated buck model with a dead time against none: the freewheel voltage and the duty move as they should",
          "[isolated-buck][dead-time]") {
    namespace AN = Kirchhoff::analytical;
    const double Vin = 48.0, Vpri = 12.0, Ipri = 2.0, Vsec = 12.0, Isec = 1.0, fs = 200e3, n = 1.0, Vf = 0.8;
    const double T = 1.0 / fs, td = 250e-9;
    // Lm for a 40 % ripple (positive magnetizing current throughout) and for a 300 % one (negative at the end of
    // the low-side interval).
    for (const double L : {(Vin - Vpri) * 0.25 * T / (0.4 * 3.0), (Vin - Vpri) * 0.25 * T / (3.0 * 3.0)}) {
        CAPTURE(L);
        const MAS::OperatingPoint none = AN::analytical_isolated_buck(Vin, Vpri, Ipri, Vsec, Isec, fs, L, n, Vf, 1.0);
        const MAS::OperatingPoint dead = AN::analytical_isolated_buck(Vin, Vpri, Ipri, Vsec, Isec, fs, L, n, Vf, 1.0,
                                                                      td, Vf);
        const double D0 = AN::model_duty_of(none)->dutyCycle, D = AN::model_duty_of(dead)->dutyCycle;
        CHECK_THAT(D0, WithinAbs(0.25, 1e-12));
        // By value: get_current() / get_voltage() return copies, so a reference into them dangles.
        const MAS::Waveform im = dead.get_excitations_per_winding().at(0).get_current()->get_waveform().value();
        const MAS::Waveform v = dead.get_excitations_per_winding().at(0).get_voltage()->get_waveform().value();
        const MAS::Waveform is = dead.get_excitations_per_winding().at(1).get_current()->get_waveform().value();
        const std::vector<double> vt = v.get_time().value();
        const double tOn = D * T, tLast = vt.back();
        // The regime the model solved: which body diode each dead time freewheels through, and for how long.
        const AN::IsolatedBuckDeadTimes regime = AN::isolated_buck_dead_times(D0, Vin, Vpri, Ipri + Isec / n, fs, L, td, Vf);
        CHECK_THAT(D, WithinAbs(D0 + regime.dutyShift, 1e-15));
        const bool negativeBeforeOn = regime.negative[1];
        CHECK(negativeBeforeOn == (L < 1e-5));
        CHECK_FALSE(regime.negative[0]);
        CHECK(regime.conduction[0] == td);
        // After the on-time the magnetizing current is at its peak, positive: the low-side diode clamps the switch
        // node at -Vf, so the primary sees -(Vpri + Vf), where it saw -Vpri with no dead time.
        CHECK_THAT(waveform_at(v, tOn + 0.5 * td), WithinAbs(-(Vpri + Vf), 1e-12));
        CHECK_THAT(waveform_at(v, 0.5 * (tOn + td + T - td)), WithinAbs(-Vpri, 1e-12));
        // The primary winding carries the primary rail's current on average and the secondary the isolated rail's
        // (source convention: -Isec), whatever the dead times do. The samples are 128 per period, so the mean of a
        // waveform with jumps is good to its jumps times half a sample.
        const auto mean_and_bound = [&](const MAS::Waveform& w) {
            const auto& d = w.get_data();
            const double dt = T / d.size();
            double area = 0.0, bound = 0.0;
            for (size_t k = 0; k < d.size(); ++k) {
                const double a = d[k], b = d[(k + 1) % d.size()];
                area += 0.5 * (a + b) * dt;
                bound += 0.5 * std::abs(b - a) * dt;
            }
            return std::make_pair(area / T, bound / T);
        };
        const auto [iPriMean, iPriBound] = mean_and_bound(im);
        const auto [iSecMean, iSecBound] = mean_and_bound(is);
        const auto [vMean, vBound] = mean_and_bound(v);
        CHECK_THAT(iPriMean, WithinAbs(Ipri, iPriBound));
        CHECK_THAT(iSecMean, WithinAbs(-Isec, iSecBound));
        // Volt-second balance on the primary over the period (efficiency 1).
        CHECK_THAT(vMean, WithinAbs(0.0, vBound));
        if (!negativeBeforeOn) {
            CHECK(regime.conduction[1] == td);
            CHECK_THAT(waveform_at(v, T - 0.5 * td), WithinAbs(-(Vpri + Vf), 1e-12));
            CHECK_THAT(D - D0, WithinAbs(2.0 * td * fs * Vf / Vin, 1e-12));
        } else {
            // Negative before the on-time: the high-side diode conducts and the primary sees the on-time voltage
            // Vin + Vf - Vpri. In this design the current reaches zero inside that dead time, after x = conduction[1]:
            // both diodes then block and the winding idles at zero voltage and current until QS1 turns on. The duty
            // moves by the volt-seconds of the two dead times: +Vf·td after the on-time, -(Vin + Vf)·x for the
            // freewheel and -Vpri·(td - x) for the idle (a full freewheel, x = td, would move it by -td·fs).
            const double x = regime.conduction[1];
            CHECK(x > 0.0);
            CHECK(x < td);
            CHECK_THAT(D - D0, WithinAbs((Vf * td - (Vin + Vf) * x - Vpri * (td - x)) * fs / Vin, 1e-12));
            CHECK_THAT(waveform_at(v, T - td + 0.5 * x), WithinAbs(Vin + Vf - Vpri, 1e-12));
            REQUIRE(tLast > T - td + x);   // the last sample falls in the idle
            CHECK_THAT(waveform_at(v, tLast), WithinAbs(0.0, 1e-12));
            CHECK_THAT(waveform_at(im, tLast), WithinAbs(0.0, 1e-12));
            CHECK_THAT(waveform_at(is, tLast), WithinAbs(0.0, 1e-12));
            CHECK(waveform_at(im, T - td + 0.5 * x) < 0.0);
        }
    }
}

TEST_CASE("Isolated buck dead time: refused without the spec's diode drop, or with a dead-time fraction beside it",
          "[duty-pin][isolated-buck][dead-time][errors]") {
    CHECK_THAT(refusal("isolated_buck", isolated_buck_dead_time_spec(json{{"forcedDeadTime", 250e-9}})),
               ContainsSubstring("Diode Voltage Drop"));
    CHECK_THAT(refusal("isolated_buck", isolated_buck_dead_time_spec(
                                            json{{"forcedDeadTime", 250e-9}, {"diodeVoltageDrop", 0.8},
                                                 {"deadTimeFraction", 0.01}})),
               ContainsSubstring("both a dead time and a dead-time fraction"));
    // 2.6 us at 200 kHz is 52 % of the period on the two edges: no low-side time is left.
    CHECK_THAT(refusal("isolated_buck", isolated_buck_dead_time_spec(
                                            json{{"forcedDeadTime", 2.6e-6}, {"diodeVoltageDrop", 0.8}})),
               ContainsSubstring("Lower the Dead Time"));
    // The isolated buck-boost keeps refusing it: one switch and a diode.
    const json ibb = spec(36.0, 48.0, 72.0, 48.0, {{12.0, 20.0}, {12.0, 10.0}}, 100e3, json::object(),
                          json{{"forcedDeadTime", 100e-9}});
    CHECK_THAT(refusal("isolated_buck_boost", ibb),
               ContainsSubstring("the isolated buck-boost has no complementary switch, so a dead time does not apply"));
}

// The web wizard's "I know the design" default, exactly as the page sends it: Vin 36..72 V (54 V for the design
// call), 750 kHz, Lm 10 uH, n = 5, 10 V / 0.02 A primary and 10 V / 0.1 A isolated rails, efficiency 0.9, 100 ns
// dead time, Vf 0.7 V. The magnetizing current averages only 0.04 A against a ~1 A ripple, so it is negative before
// the on-time and reaches zero inside that dead time: the high-side diode conducts for x < td, then the converter
// idles. Below efficiency 1 the duty law D = Vpri·eta/Vin does not balance the volt-seconds with a low-side slope of
// -Vpri/L; the model without an idle lets the low-side interval close the period, and the idle regime must too, or
// its freewheel length (solved with -Vpri/L) and its waveform (closed at zero) disagree and every input voltage threw
// "no freewheel length gives the average 0.040000 A".
json isolated_buck_web_default_spec(double inputVoltage) {
    json s = json::parse(R"({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":36,"maximum":72},
        "switchingFrequency":{"nominal":750000},"outputs":[{"name":"out","voltage":{"nominal":10},"regulation":"voltage"},
        {"name":"out2","voltage":{"nominal":10},"regulation":"voltage"}],"efficiency":0.9,
        "magnetizingInductance":{"nominal":0.00001},"turnsRatios":[{"nominal":5},{"nominal":5}]},
        "operatingPoints":[{"name":"op","ambientTemperature":25,"outputs":[{"name":"out","power":0.2},
        {"name":"out2","power":1}]}],"config":{"diodeVoltageDrop":0.7,"forcedDeadTime":1e-7}})");
    s["operatingPoints"][0]["inputVoltage"] = inputVoltage;
    return s;
}

TEST_CASE("Isolated buck dead time below efficiency 1: the current that reaches zero before the on-time idles",
          "[duty-pin][isolated-buck][dead-time]") {
    namespace AN = Kirchhoff::analytical;
    const double Vpri = 10.0, eta = 0.9, fs = 750e3, L = 10e-6, td = 100e-9, Vf = 0.7;
    const double imAverage = 0.02 + 0.1 / 5.0;
    for (const double Vin : {54.0, 36.0, 72.0}) {
        CAPTURE(Vin);
        const json an = process("isolated_buck", isolated_buck_web_default_spec(Vin));
        const double D = analytical_primary_duty(an);
        const AN::IsolatedBuckDeadTimes regime =
            AN::isolated_buck_dead_times(Vpri / Vin * eta, Vin, Vpri, imAverage, fs, L, td, Vf);
        CHECK_THAT(D, WithinAbs(Vpri / Vin * eta + regime.dutyShift, 1e-12));
        CHECK_FALSE(regime.negative[0]);
        CHECK(regime.negative[1]);
        const double x = regime.conduction[1];
        INFO("freewheel before the on-time " << x * 1e9 << " ns of " << td * 1e9 << " ns, duty " << D);
        CHECK(x > 0.0);
        CHECK(x <= td);
        // The model's waveforms: the period averages the magnetizing current, so the primary winding carries the
        // primary rail's 0.02 A and the secondary the isolated rail's 0.1 A (source convention -0.1 A), whatever the
        // dead times do. The samples are 128 per period, so the mean of a waveform with jumps is good to its jumps
        // times half a sample (as in the model test above).
        const MAS::OperatingPoint op =
            AN::analytical_isolated_buck(Vin, Vpri, 0.02, 10.0, 0.1, fs, L, 5.0, 0.7, eta, td, Vf);
        CHECK_THAT(AN::model_duty_of(op)->dutyCycle, WithinAbs(D, 1e-12));
        const double T = 1.0 / fs;
        const auto mean_and_bound = [&](const MAS::Waveform& w) {
            const auto& d = w.get_data();
            const double dt = T / d.size();
            double area = 0.0, bound = 0.0;
            for (size_t k = 0; k < d.size(); ++k) {
                const double a = d[k], b = d[(k + 1) % d.size()];
                area += 0.5 * (a + b) * dt;
                bound += 0.5 * std::abs(b - a) * dt;
            }
            return std::make_pair(area / T, bound / T);
        };
        const auto [iPriMean, iPriBound] =
            mean_and_bound(op.get_excitations_per_winding().at(0).get_current()->get_waveform().value());
        const auto [iSecMean, iSecBound] =
            mean_and_bound(op.get_excitations_per_winding().at(1).get_current()->get_waveform().value());
        CHECK_THAT(iPriMean, WithinAbs(0.02, iPriBound));
        CHECK_THAT(iSecMean, WithinAbs(-0.1, iSecBound));
    }
}

// Inputs that have exactly one steady state, which the fixed point missed: a 5616-case sweep (eta 0.8..1, Vin
// 30..80 V, Lm 2..20 uH, ImAvg 0..1 A, td 50..200 ns at 750 kHz, Vpri 10 V, Vf 0.7 V) against a brute-force
// enumeration of the four diode regimes, with and without the idle. The expected numbers are the brute force's.
// (a) The iterate without an idle ends the period with the current positive before the on-time (low-side diode),
//     but the idle's freewheel is through the high-side diode; solving the idle only with the iterate's sign threw
//     "no freewheel length in (0, td] gives the average".
// (b) An intermediate iterate's duty fell below zero (-0.000x) and was refused as "no high-side switch on-time",
//     although the settled regime's duty is 0.07.
// (c) As (a), with a freewheel of only 0.15 ns of the 200 ns dead time (a brute force sampling the freewheel from
//     td/400 up misses it; sampled from ~0 it finds this one solution).
TEST_CASE("Isolated buck dead time: the regimes the fixed point missed", "[duty-pin][isolated-buck][dead-time]") {
    namespace AN = Kirchhoff::analytical;
    const double Vpri = 10.0, fs = 750e3, Vf = 0.7;
    struct Case { double eta, Vin, L, imAverage, td, duty, freewheel; };
    for (const Case& c : {Case{0.8, 30.0, 2e-6, 1.0, 200e-9, 0.20949758728737683, 2.0616578510704986e-08},
                          Case{0.8, 54.0, 2e-6, 0.0, 200e-9, 0.07146111774848649, 8.19119952746228e-08},
                          Case{0.8, 76.0, 10e-6, 0.3, 200e-9, 0.08681020058765276, 1.4842089631963602e-10}}) {
        DYNAMIC_SECTION("eta " << c.eta << ", Vin " << c.Vin << " V, Lm " << c.L << " H, ImAvg " << c.imAverage << " A") {
            const double base = Vpri / c.Vin * c.eta;
            const AN::IsolatedBuckDeadTimes r = AN::isolated_buck_dead_times(base, c.Vin, Vpri, c.imAverage, fs, c.L, c.td, Vf);
            CHECK_THAT(base + r.dutyShift, WithinAbs(c.duty, 1e-9));
            CHECK_FALSE(r.negative[0]);
            CHECK(r.negative[1]);
            CHECK(r.conduction[0] == c.td);
            CHECK_THAT(r.conduction[1], WithinRel(c.freewheel, 1e-6));
        }
    }
    // Still refused: two dead times longer than the period leave no on-time in any regime.
    CHECK_THROWS_WITH(AN::isolated_buck_dead_times(10.0 / 54.0 * 0.8, 54.0, Vpri, 0.0, fs, 2e-6, 700e-9, Vf),
                      ContainsSubstring("Lower the Dead Time"));
}

TEST_CASE("Isolated buck dead time below efficiency 1: the web default runs in ngspice",
          "[duty-pin][isolated-buck][dead-time][ngspice]") {
    // As the page's Simulated button sends it: the design's turns ratio pinned, 50 + 2 periods of transient.
    for (const double Vin : {36.0, 72.0}) {
        CAPTURE(Vin);
        json s = isolated_buck_web_default_spec(Vin);
        s["designRequirements"]["turnsRatios"] = json::array({json{{"nominal", 5}}});
        s["config"]["tranStopTime"] = 52.0 / 750e3;
        const std::string raw = Kirchhoff::api::process_converter("isolated_buck", s.dump(), "ngspice");
        INFO(raw.substr(0, 400));
        REQUIRE(raw.rfind("Exception", 0) != 0);
    }
}

TEST_CASE("Push-pull reports the per-switch duty, which falls as the input voltage rises", "[duty-pin][push-pull]") {
    // Help mode (no pin). diagnostics.dutyCycle once measured the dot-reference primary half's current, about
    // 1 − D: 0.709 / 0.811 / 0.858 at 36 / 54 / 72 V. The model's duty is per switch: t1/T = N(Vo+Vd)/(2·Vin) < 0.5.
    for (const auto& [vmin, vnom, vmax, vops] :
         {std::tuple<double, double, double, std::vector<double>>{36.0, 54.0, 72.0, {36.0, 54.0, 72.0}},
          std::tuple<double, double, double, std::vector<double>>{20.0, 25.0, 30.0, {20.0, 30.0}}}) {
        double previous = 1.0;
        for (const double vop : vops) {
            CAPTURE(vmin, vmax, vop);
            const json r = process("push_pull", spec(vmin, vnom, vmax, vop, {{12.0, 60.0}}, 100e3, json::object(),
                                                     json::object()));
            const double duty = analytical_primary_duty(r);
            CHECK(duty > 0.0);
            CHECK(duty < 0.5);
            CHECK(duty < previous);
            previous = duty;
        }
    }
}

TEST_CASE("A pinned push-pull reports the same per-switch duty its switches run at", "[duty-pin][push-pull]") {
    for (const double D : {0.2, 0.3, 0.45}) {
        CAPTURE(D);
        const json r = process("push_pull", spec(36.0, 48.0, 72.0, 48.0, {{12.0, 60.0}}, 100e3, json::object(),
                                                 json{{"forcedDutyCycle", D}}));
        CHECK(stimulus_duty(r, "Q1") == D);
        CHECK(stimulus_duty(r, "Q2") == D);
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(D, 1e-9));
    }
}

TEST_CASE("Phase-shifted bridges report the effective duty: the phase fraction less the duty-cycle loss",
          "[duty-pin][psfb][pshb]") {
    // diagnostics.dutyCycle of a phase-shifted bridge is the fraction of the period the primary sees the bridge
    // voltage (Vout = Vbridge·Deff/n, TI SLUA107): phase/180 less the Sabate loss 4·Lr·Io·fs/(n·Vbridge).
    for (const std::string topology : {"psfb", "pshb"}) {
        for (const double vop : {380.0, 400.0, 420.0}) {
            CAPTURE(topology, vop);
            const json s = spec(380.0, 400.0, 420.0, vop, {{48.0, 500.0}}, 100e3, json::object(), json::object());
            const json r = process(topology, s);
            double phaseDeg, Lr, n, Vbridge, Io, fs;
            if (topology == "psfb") {
                const Kirchhoff::PsfbDesign d = Kirchhoff::design_psfb(s);
                phaseDeg = d.phaseDeg; Lr = d.seriesInductance; n = d.turnsRatio; Vbridge = d.inputVoltage;
                Io = d.outputPower / d.outputVoltage; fs = d.switchingFrequency;
            } else {
                const Kirchhoff::PshbDesign d = Kirchhoff::design_pshb(s);
                phaseDeg = d.phaseDeg; Lr = d.seriesInductance; n = d.turnsRatio; Vbridge = 0.5 * d.inputVoltage;
                Io = d.outputPower / d.outputVoltage; fs = d.switchingFrequency;
            }
            const double commanded = phaseDeg / 180.0;
            const double loss = 4.0 * Lr * Io * fs / (n * Vbridge);
            REQUIRE(loss > 0.0);
            CHECK_THAT(analytical_primary_duty(r), WithinAbs(commanded - loss, 1e-12));
            CHECK(analytical_primary_duty(r) < commanded);
            CHECK_FALSE(r.at("diagnostics").contains("boostLegDutyCycle"));
        }
    }
}

TEST_CASE("Resonant and dual-active bridges report the 50 % square-wave duty", "[duty-pin][resonant]") {
    for (const std::string topology : {"llc", "cllc", "clllc", "src", "dab", "lcc"}) {
        CAPTURE(topology);
        if (topology == "lcc") {
            // Alf 2026-10-30: a design that cannot run at an end of its own input range throws (ABT #2171). The LCC
            // design solved at 400 V does not solve at the 380 V end of its 380-420 V range (solve_lcc refuses it).
            const std::string out = refusal(topology, spec(380.0, 400.0, 420.0, 400.0, {{48.0, 500.0}}, 100e3,
                                                           json::object(), json::object()));
            CHECK_THAT(out, Catch::Matchers::ContainsSubstring("cannot be evaluated at the input voltage 380 V"));
            continue;
        }
        const json r = process(topology, spec(380.0, 400.0, 420.0, 400.0, {{48.0, 500.0}}, 100e3, json::object(),
                                              json::object()));
        CHECK(analytical_primary_duty(r) == 0.5);
        CHECK_FALSE(r.at("diagnostics").contains("boostLegDutyCycle"));
    }
}

TEST_CASE("Reverse-flow CLLC and CLLLC report the driving bridge's duty", "[duty-pin][resonant][reverse]") {
    // The reverse solve reorders the windings onto a new operating point; diagnostics must still find the
    // model duty (it threw "registered no switch duty" when the reordered point was not registered).
    for (const std::string topology : {"cllc", "clllc"}) {
        CAPTURE(topology);
        const json r = process(topology, spec(380.0, 400.0, 420.0, 400.0, {{48.0, 480.0}}, 100e3, json::object(),
                                              json{{"powerFlowDirection", "reverse"}}));
        CHECK(analytical_primary_duty(r) == 0.5);
    }
}

TEST_CASE("Four-switch buck-boost reports the duty of the leg that switches, and both legs in buck-boost",
          "[duty-pin][fsbb]") {
    const double eta = 0.9;
    SECTION("buck region: the buck leg's duty") {
        const json r = process("fsbb", spec(36.0, 48.0, 72.0, 48.0, {{12.0, 60.0}}, 100e3, json::object(),
                                            json::object()));
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(12.0 / (48.0 * eta), 1e-12));
        CHECK_FALSE(r.at("diagnostics").contains("boostLegDutyCycle"));
    }
    SECTION("boost region: the boost leg's duty") {
        const json r = process("fsbb", spec(10.0, 12.0, 14.0, 12.0, {{48.0, 60.0}}, 100e3, json::object(),
                                            json::object()));
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(1.0 - 12.0 * eta / 48.0, 1e-12));
        CHECK_FALSE(r.at("diagnostics").contains("boostLegDutyCycle"));
    }
    SECTION("buck-boost split-PWM: buck leg t2, boost leg t1") {
        // Vin = Vo = 24 V: D = Vo/(Vin+Vo) = 0.5, t1 = κ·D = 0.25 (κ = 0.5), t2 = Vo·(1 − t1)/Vin = 0.75.
        const json r = process("fsbb", spec(22.0, 24.0, 26.0, 24.0, {{24.0, 60.0}}, 100e3, json::object(),
                                            json{{"fsbbSplitRatio", 0.5}}));
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(0.75, 1e-12));
        REQUIRE(r.at("diagnostics").contains("boostLegDutyCycle"));
        CHECK_THAT(r.at("diagnostics").at("boostLegDutyCycle").get<double>(), WithinAbs(0.25, 1e-12));
    }
    SECTION("buck-boost simultaneous: both legs at D = Vo/(Vin+Vo)") {
        const json r = process("fsbb", spec(22.0, 24.0, 26.0, 24.0, {{24.0, 60.0}}, 100e3, json::object(),
                                            json{{"transitionMode", "simultaneous"}}));
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(0.5, 1e-12));
        REQUIRE(r.at("diagnostics").contains("boostLegDutyCycle"));
        CHECK_THAT(r.at("diagnostics").at("boostLegDutyCycle").get<double>(), WithinAbs(0.5, 1e-12));
    }
}

TEST_CASE("Vienna reports phase A's boost duty at the line-voltage peak in every sampling strategy",
          "[duty-pin][vienna]") {
    // Phase A at the line peak: D = 1 − V_phase,peak/(Vout/2). Peak-of-line sampling threw "registered no switch
    // duty" before; the full line-cycle envelope reported no duty at all.
    const double vLine = 230.94010767585033, vout = 800.0;
    for (const std::string sampling : {"peakOfLineOnly", "peakOfLinePlusSectors", "fullLineCycle"}) {
        CAPTURE(sampling);
        json s;
        s["designRequirements"] = json::parse(R"({"inputType":"acThreePhase","inputVoltage":{"minimum":207.85,
            "nominal":230.94010767585033,"maximum":254.03},"switchingFrequency":{"nominal":20000},
            "outputs":[{"name":"out","voltage":{"nominal":800},"regulation":"voltage"}],"efficiency":0.97,
            "lineFrequency":{"nominal":50}})");
        s["operatingPoints"] = json::parse(R"([{"name":"full_load","inputVoltage":230.94010767585033,
            "ambientTemperature":40,"outputs":[{"name":"out","power":10000}]}])");
        s["config"] = {{"samplingStrategy", sampling}};
        const json r = process("vienna", s);
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(1.0 - std::sqrt(2.0) * vLine / (vout / 2.0), 1e-12));
    }
}

TEST_CASE("Boost PFC reports the duty at the line-voltage peak", "[duty-pin][pfc]") {
    // CCM boost at the line peak: D = 1 − Vin,pk/(Vout + Vd), the minimum duty of the line cycle. Before, the
    // line-frequency envelope reported no duty.
    for (const double vrms : {115.0, 230.0}) {
        CAPTURE(vrms);
        json s;
        s["designRequirements"] = json::parse(R"({"inputType":"acSinglePhase","inputVoltage":{"minimum":90,
            "maximum":265},"switchingFrequency":{"nominal":65000},
            "outputs":[{"name":"out","voltage":{"nominal":400},"regulation":"voltage"}],"efficiency":0.95,
            "lineFrequency":{"nominal":50}})");
        s["designRequirements"]["inputVoltage"]["nominal"] = vrms;   // the model runs at the nominal line
        s["operatingPoints"] = json::array({json{{"name", "op"}, {"inputVoltage", vrms}, {"ambientTemperature", 25},
                                                 {"outputs", json::array({json{{"name", "out"}, {"power", 300}}})}}});
        const json r = process("pfc", s);
        const double D = analytical_primary_duty(r);
        // Vd is the model's diode drop (≤ 2 V): D sits within 2/400 of 1 − Vpk/Vout and below it is not.
        const double ideal = 1.0 - std::sqrt(2.0) * vrms / 400.0;
        CHECK(D >= ideal);
        CHECK(D < ideal + 2.0 * std::sqrt(2.0) * vrms / (400.0 * 400.0) + 1e-12);
    }
}

// The spec the web PFC wizard sends for its SEPIC / Ćuk / interleaved-boost variants (captured from the browser,
// 2026-11-01): only config.topologyVariant (and numberOfPhases for the interleaved boost) differs between them.
static json web_pfc_variant_spec(const std::string& variant) {
    json s = json::parse(R"({"designRequirements":{"inputType":"acSinglePhase","inputVoltage":{"minimum":85,
        "maximum":265},"switchingFrequency":{"nominal":65000},"outputs":[{"name":"out","voltage":{"nominal":400},
        "regulation":"voltage"}],"efficiency":0.95,"lineFrequency":{"nominal":50}},"operatingPoints":[{"name":
        "full_load","inputVoltage":175,"ambientTemperature":25,"outputs":[{"name":"out","power":300}]}],"config":
        {"currentRippleFraction":0.3,"diodeVoltageDrop":0.6,"mode":"ccm","wideBandgapSwitch":true}})");
    s["config"]["topologyVariant"] = variant;
    if (variant == "interleavedBoost") s["config"]["numberOfPhases"] = 2;
    return s;
}

TEST_CASE("The web's SEPIC, Cuk and interleaved PFC capture their inductors under the TAS names",
          "[duty-pin][pfc][pfc-variants]") {
    // The variant builders captured their inductor excitation as "L" while their TAS names the inductors L1, L2:
    // process_converter then found no operating point for the main magnetic L1 and threw, and before the duty
    // report the analyticalWaveforms registry keyed a component the TAS does not have.
    for (const std::string variant : {"sepic", "cuk", "interleavedBoost"}) {
        CAPTURE(variant);
        const json r = process("pfc", web_pfc_variant_spec(variant));
        std::vector<std::string> magnetics;
        for (const auto& m : r.at("diagnostics").at("magnetics")) magnetics.push_back(m.at("name").get<std::string>());
        const json& waveforms = r.at("analyticalWaveforms");
        REQUIRE(waveforms.contains("L1"));
        for (const auto& [name, op] : waveforms.items()) {
            CAPTURE(name);
            CHECK(std::find(magnetics.begin(), magnetics.end(), name) != magnetics.end());
        }
        // Both interleaved phases carry the per-phase inductor excitation.
        if (variant == "interleavedBoost") CHECK(waveforms.contains("L2"));
        // The TAS operating point of L1 is the captured one.
        const json& l1 = component(r, "L1").at("data").at("inputs").at("operatingPoints").at(0)
                             .at("excitationsPerWinding").at(0).at("current").at("processed");
        const json& captured = waveforms.at("L1").at("excitationsPerWinding").at(0).at("current").at("processed");
        CHECK(l1.at("peak").get<double>() == captured.at("peak").get<double>());
        CHECK(l1.at("rms").get<double>() == captured.at("rms").get<double>());
    }
}

TEST_CASE("SEPIC and Cuk PFC report the buck-boost duty at the line peak, the interleaved boost the boost duty",
          "[duty-pin][pfc][pfc-variants]") {
    // At the line peak a SEPIC / Ćuk input inductor is ON at +Vpk and OFF at −(Vout + Vd) (the energy-transfer
    // capacitor holds the line voltage), so D = (Vout + Vd)/(Vout + Vd + Vpk) — the duty design_pfc sizes L1 with.
    // A boost leg is OFF at Vpk − (Vout + Vd): D = 1 − Vpk/(Vout + Vd).
    const double vpk = 175.0 * std::sqrt(2.0), vo = 400.0 + 0.6;
    for (const std::string variant : {"sepic", "cuk"}) {
        CAPTURE(variant);
        const json r = process("pfc", web_pfc_variant_spec(variant));
        CHECK_THAT(analytical_primary_duty(r), WithinAbs(vo / (vo + vpk), 1e-12));
    }
    const json r = process("pfc", web_pfc_variant_spec("interleavedBoost"));
    CHECK_THAT(analytical_primary_duty(r), WithinAbs(1.0 - vpk / vo, 1e-12));
}

TEST_CASE("A current-doubler PSFB reports the effective duty its inline excitation is built at", "[duty-pin][psfb]") {
    // The current doubler is not modelled by the analytical solver: T1's excitation is written inline at the
    // design's effective duty, declared for diagnostics (it threw "captured no operating point" without it).
    const json s = spec(380.0, 400.0, 420.0, 400.0, {{48.0, 500.0}}, 100e3, json::object(),
                        json{{"rectifierType", "currentDoubler"}});
    const json r = process("psfb", s);
    const Kirchhoff::PsfbDesign d = Kirchhoff::design_psfb(s);
    CHECK(analytical_primary_duty(r) == d.effectiveDuty);
    CHECK(analytical_primary_duty(r) < d.phaseDeg / 180.0);
}

