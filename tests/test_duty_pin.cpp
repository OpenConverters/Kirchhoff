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
