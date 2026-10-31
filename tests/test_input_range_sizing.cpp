// One design for the whole input-voltage range (ABT #2160). The web designs a converter once over
// designRequirements.inputVoltage {minimum, nominal, maximum} and then evaluates THAT converter at every input
// voltage the user entered (operatingPoints[0].inputVoltage). Every part, and every rating the parts are sourced
// with, must therefore be the same at each of those voltages:
//   - parts the design sizes are sized at the range's worst corner (output inductor ripple at Vin_max, SRC turns
//     ratio at Vin_min, ...), never at the operating point;
//   - parts the web pins back from the design call (config.outputInductance, config.outputCapacitance, ...) are
//     honoured, and a malformed pin throws;
//   - semiconductor / capacitor / resistor ratings are the worst case over the range (api layer).
// Checked through the api layer every caller uses (process_converter), and on the typed designs where the value
// is a single field.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "KirchhoffApi.hpp"
#include "Acf.hpp"
#include "Forward.hpp"
#include "TwoSwitchForward.hpp"
#include "Pfc.hpp"
#include "Src.hpp"
#include "DimensionJson.hpp"
#include "ConverterAnalytical.hpp"

using json = nlohmann::json;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinRel;

namespace {

json ranged_spec(double vmin, double vnom, double vmax, double vop, const std::vector<std::pair<double, double>>& outs,
                 double fsw, double efficiency) {
    json s;
    json& dr = s["designRequirements"];
    dr["inputVoltage"] = {{"minimum", vmin}, {"nominal", vnom}, {"maximum", vmax}};
    dr["switchingFrequency"]["nominal"] = fsw;
    dr["efficiency"] = efficiency;
    json op;
    op["name"] = "op";
    op["inputVoltage"] = vop;
    op["ambientTemperature"] = 25.0;
    for (size_t i = 0; i < outs.size(); ++i) {
        json o;
        o["name"] = "out" + std::to_string(i);
        o["voltage"]["nominal"] = outs[i].first;
        o["power"]["nominal"] = outs[i].first * outs[i].second;
        dr["outputs"].push_back(o);
        json oo;
        oo["name"] = o["name"];
        oo["voltage"] = outs[i].first;
        oo["power"] = outs[i].first * outs[i].second;
        op["outputs"].push_back(oo);
    }
    s["operatingPoints"] = json::array({op});
    return s;
}

json at_input_voltage(json spec, double vin) {
    spec["operatingPoints"][0]["inputVoltage"] = vin;
    return spec;
}

json run(const std::string& topology, const json& spec) {
    const std::string r = Kirchhoff::api::process_converter(topology, spec.dump(), "analytical");
    INFO(r.substr(0, 400));
    REQUIRE(r.rfind("Exception", 0) != 0);
    return json::parse(r);
}

std::string run_error(const std::string& topology, const json& spec) {
    const std::string r = Kirchhoff::api::process_converter(topology, spec.dump(), "analytical");
    REQUIRE(r.rfind("Exception", 0) == 0);
    return r;
}

// Every part value the design reports (magnetics, capacitors, extra inductors).
std::map<std::string, json> part_values(const json& result) {
    std::map<std::string, json> m;
    const json& d = result.at("diagnostics");
    for (const auto& x : d.value("magnetics", json::array())) {
        m["magnetic " + x.at("name").get<std::string>() + " L"] = x.at("magnetizingInductance");
        m["magnetic " + x.at("name").get<std::string>() + " n"] = x.value("turnsRatios", json::array());
    }
    for (const auto& x : d.value("capacitors", json::array()))
        m["capacitor " + x.at("name").get<std::string>()] = x;
    if (d.contains("computed") && d.at("computed").contains("extraInductors"))
        for (const auto& x : d.at("computed").at("extraInductors"))
            m["inductor " + x.at("name").get<std::string>()] = x.at("inductance");
    return m;
}

// What every non-magnetic TAS component is sourced with: its designRequirements (values AND ratings).
std::map<std::string, json> sourcing(const json& result) {
    std::map<std::string, json> m;
    for (const auto& st : result.at("tas").at("topology").at("stages"))
        for (const auto& c : st.at("circuit").value("components", json::array())) {
            const json& data = c.at("data");
            if (data.contains("magnetic") || !data.contains("inputs")) continue;
            if (!data.at("inputs").contains("designRequirements")) continue;
            m[st.at("name").get<std::string>() + "/" + c.at("name").get<std::string>()] =
                data.at("inputs").at("designRequirements");
        }
    return m;
}

// `operatingFrequencyKey`: a frequency-regulated converter's controller is seeded with the frequency the bridge runs
// at, which moves with the input voltage by design; that one entry is compared without its switchingFrequency.
void require_same_design(const json& a, const json& b, const std::string& operatingFrequencyKey = "") {
    const auto pa = part_values(a), pb = part_values(b);
    REQUIRE_FALSE(pa.empty());
    REQUIRE(pa.size() == pb.size());
    for (const auto& [k, v] : pa) {
        INFO(k << ": " << v.dump() << " vs " << (pb.count(k) ? pb.at(k).dump() : std::string("missing")));
        REQUIRE(pb.count(k) == 1);
        CHECK(pb.at(k) == v);
    }
    const auto sa = sourcing(a), sb = sourcing(b);
    REQUIRE_FALSE(sa.empty());
    REQUIRE(sa.size() == sb.size());
    for (const auto& [k, v] : sa) {
        INFO(k << ": " << v.dump() << " vs " << (sb.count(k) ? sb.at(k).dump() : std::string("missing")));
        REQUIRE(sb.count(k) == 1);
        if (k == operatingFrequencyKey) {
            json x = v, y = sb.at(k);
            REQUIRE(x.contains("switchingFrequency"));
            x.erase("switchingFrequency");
            y.erase("switchingFrequency");
            CHECK(x == y);
            continue;
        }
        CHECK(sb.at(k) == v);
    }
}

// The customer's active clamp forward: 24-125 V in, 13 V / 24 A out, n = 1, Lm = 50 uH, 140 kHz, Vd = 0.7 V,
// efficiency 92 %.
json customer_acf(double vop) {
    json s = ranged_spec(24.0, 74.5, 125.0, vop, {{13.0, 24.0}}, 140e3, 0.92);
    s["designRequirements"]["magnetizingInductance"]["nominal"] = 50e-6;
    s["designRequirements"]["turnsRatios"] = json::array({{{"nominal", 1.0}}});
    s["config"]["diodeVoltageDrop"] = 0.7;
    return s;
}

}  // namespace

TEST_CASE("Customer ACF: the same parts and ratings at 24 V and at 125 V", "[input-range][acf]") {
    const json at24 = run("acf", customer_acf(24.0));
    const json at125 = run("acf", customer_acf(125.0));
    require_same_design(at24, at125);
    // The evaluations really are at two different input voltages (the waveforms differ).
    CHECK(at24.at("diagnostics").at("dutyCycle").get<double>() > at125.at("diagnostics").at("dutyCycle").get<double>());
}

TEST_CASE("Customer ACF: the web's design pins reproduce the design call at every input voltage",
          "[input-range][acf]") {
    const json design = run("acf", customer_acf(74.5));
    double lout = 0.0;
    for (const auto& x : design.at("diagnostics").at("computed").at("extraInductors"))
        if (x.at("name") == "Lout") lout = x.at("inductance").get<double>();
    REQUIRE(lout > 0.0);
    for (const double vin : {24.0, 125.0}) {
        json s = customer_acf(vin);
        s["config"]["outputInductance"] = lout;
        INFO("evaluated at " << vin << " V");
        require_same_design(design, run("acf", s));
    }
}

TEST_CASE("Forward-family output inductor: Lout = Vout(1-D)/(fs * rippleRatio * Iout), D at Vin_max",
          "[input-range][lout]") {
    // The ripple ratio is relative to the output current (the same convention forward_timing's DCM test uses),
    // and the ripple's worst corner is the highest input, where the duty is smallest.
    const double vo = 13.0, io = 24.0, fs = 140e3, ripple = 0.4, vinMax = 125.0;
    for (const double vop : {24.0, 74.5, 125.0}) {
        INFO("operating point " << vop << " V");
        const json s = customer_acf(vop);
        {
            const auto d = Kirchhoff::design_acf(s);
            const double D = (vo + d.diodeDrop) * d.turnsRatio / vinMax;
            CHECK_THAT(d.outputInductance, WithinRel(vo * (1.0 - D) / (fs * ripple * io), 1e-12));
        }
        {
            const auto d = Kirchhoff::design_forward(s);
            const double D = d.turnsRatio * (vo + d.diodeDrop) / vinMax;
            CHECK_THAT(d.outputInductance, WithinRel(vo * (1.0 - D) / (fs * ripple * io), 1e-12));
        }
        {
            json s2 = s;   // the two-switch forward cannot reach 13 V at 24 V with n = 1: let it size n itself
            s2["designRequirements"].erase("turnsRatios");
            const auto d = Kirchhoff::design_two_switch_forward(s2);
            const double D = d.turnsRatio * (vo + d.diodeDrop) / vinMax;
            CHECK_THAT(d.outputInductance, WithinRel(vo * (1.0 - D) / (fs * ripple * io), 1e-12));
        }
    }
}

TEST_CASE("SRC: the turns ratio is sized at the lowest input of the range", "[input-range][src]") {
    const json base = ranged_spec(36.0, 48.0, 72.0, 48.0, {{12.0, 5.0}}, 100e3, 0.9);
    // A spec whose only input voltage is the range minimum: its turns ratio is the one sized at Vin_min.
    json atMin = ranged_spec(36.0, 36.0, 36.0, 36.0, {{12.0, 5.0}}, 100e3, 0.9);
    const auto dMin = Kirchhoff::design_src(atMin);
    for (const double vop : {36.0, 48.0, 72.0}) {
        INFO("operating point " << vop << " V");
        const auto d = Kirchhoff::design_src(at_input_voltage(base, vop));
        CHECK(d.turnsRatio == dMin.turnsRatio);
        CHECK(d.resonantInductance == dMin.resonantInductance);
        CHECK(d.resonantCapacitance == dMin.resonantCapacitance);
    }
    // Sized at the nominal (48 V) the ratio would be a third larger: the test separates the two.
    json atNom = ranged_spec(48.0, 48.0, 48.0, 48.0, {{12.0, 5.0}}, 100e3, 0.9);
    CHECK(Kirchhoff::design_src(atNom).turnsRatio > dMin.turnsRatio * 1.2);
}

TEST_CASE("SRC: frequency regulation delivers the output at every input of the range with the same parts",
          "[input-range][src][regulation]") {
    // 36-72 V -> 12 V / 5 A. The turns ratio is sized at 36 V (the tank only steps down from its unity peak), so
    // above 36 V the bridge runs above resonance, where the series tank's exact steady-state gain (state plane,
    // ABT #2173) falls to Vin_min/Vin.
    const double vinMin = 36.0, vo = 12.0, eta = 0.9, k = 0.5;
    const json base = ranged_spec(vinMin, 48.0, 72.0, vinMin, {{vo, 5.0}}, 100e3, eta);
    std::map<double, Kirchhoff::SrcDesign> d;
    std::map<double, json> r;
    for (const double vop : {36.0, 54.0, 72.0}) {
        d[vop] = Kirchhoff::design_src(at_input_voltage(base, vop));
        r[vop] = run("src", at_input_voltage(base, vop));
    }
    // One converter: identical parts and sourcing at both ends of the range.
    require_same_design(r.at(36.0), r.at(72.0), "control/U1");
    for (const double vop : {54.0, 72.0}) {
        CHECK(d.at(vop).turnsRatio == d.at(36.0).turnsRatio);
        CHECK(d.at(vop).resonantInductance == d.at(36.0).resonantInductance);
        CHECK(d.at(vop).resonantCapacitance == d.at(36.0).resonantCapacitance);
    }
    // At Vin_min the converter runs exactly at the series resonance (today's operating point, gain 1).
    CHECK(d.at(36.0).switchingFrequency == 100e3);
    CHECK(d.at(36.0).tankGain == 1.0);
    CHECK_THAT(1.0 / (2.0 * M_PI * std::sqrt(d.at(36.0).resonantInductance * d.at(36.0).resonantCapacitance)),
               WithinRel(100e3, 1e-12));
    // The operating frequency rises with the input voltage; the TAS, its stimulus and the diagnostics carry it.
    CHECK(d.at(54.0).switchingFrequency > d.at(36.0).switchingFrequency * 1.05);
    CHECK(d.at(72.0).switchingFrequency > d.at(54.0).switchingFrequency * 1.05);
    for (const double vop : {36.0, 54.0, 72.0}) {
        INFO("operating point " << vop << " V");
        const json& res = r.at(vop);
        CHECK(res.at("diagnostics").at("switchingFrequency").get<double>() == d.at(vop).switchingFrequency);
        CHECK(PEAS::resolve_dimensional_values(
                  res.at("tas").at("inputs").at("designRequirements").at("switchingFrequency")) ==
              d.at(vop).switchingFrequency);
        CHECK(PEAS::resolve_dimensional_values(sourcing(res).at("control/U1").at("switchingFrequency")) ==
              d.at(vop).switchingFrequency);
        const json stimuli = res.at("tas").at("simulation").at("stimulus");
        REQUIRE(stimuli.size() == 2);
        for (const auto& st : stimuli)
            CHECK(st.at("waveform").at("frequency").get<double>() == d.at(vop).switchingFrequency);
        // The output reaches its target at every input: the series tank's exact gain at the operating frequency into
        // the reflected load (the rectifier's DC resistance referred to the primary, n²·R) delivers the same rail as
        // the resonant point at 36 V (n·(Vo+Vd) = M·η·k·Vin), and that rail is at least the 12 V target (the 1.08
        // headroom above it is left to the closed loop).
        const auto& dd = d.at(vop);
        const double n = dd.turnsRatio, Vd = dd.outputs.at(0).diodeDrop;
        const double R0 = std::sqrt(dd.resonantInductance / dd.resonantCapacitance);
        const double f0 = 1.0 / (2.0 * M_PI * std::sqrt(dd.resonantInductance * dd.resonantCapacitance));
        const auto tank = Kirchhoff::analytical::src_exact_tank(dd.switchingFrequency / f0,
                                                                R0 / (n * n * dd.loadResistance));
        const double M = tank.gain;
        const double delivered = M * eta * k * vop / n - Vd;
        const double deliveredAtMin = eta * k * vinMin / n - Vd;
        CHECK_THAT(M, WithinRel(vinMin / vop, 1e-9));
        CHECK_THAT(delivered, WithinRel(deliveredAtMin, 1e-9));
        CHECK(delivered >= vo);
        CHECK(dd.outputVoltage == vo);
        // Regulated, the primary carries the same rectified (load) current at every input (unregulated at resonance it
        // doubled from 36 V to 72 V: 6.07 A -> 12.15 A rms); its rms is the exact tank current's, a sine at 36 V and
        // two arcs per half period above (6.07 / 6.17 / 6.27 A at 36 / 54 / 72 V). 1 %: the 256-sample waveform.
        CHECK_THAT(res.at("diagnostics").at("primaryRmsCurrent").get<double>(),
                   WithinRel(tank.currentRms() * k * vop / R0, 0.01));
        CHECK_THAT(res.at("diagnostics").at("primaryRmsCurrent").get<double>(),
                   WithinRel(r.at(36.0).at("diagnostics").at("primaryRmsCurrent").get<double>(), 0.04));
    }
    // Below the range minimum no frequency delivers the output: refused, naming the input and what it reaches.
    {
        const std::string e = run_error("src", at_input_voltage(base, 30.0));
        CHECK_THAT(e, ContainsSubstring("at the input voltage 30 V"));
        CHECK_THAT(e, ContainsSubstring("range minimum 36 V"));
    }
    // A switching band whose maximum lies below the frequency 72 V needs is refused, with the output it reaches.
    {
        json banded = at_input_voltage(base, 72.0);
        banded["config"]["maxSwitchingFrequency"] = 200e3;
        const std::string e = run_error("src", banded);
        CHECK_THAT(e, ContainsSubstring("at the input voltage 72 V"));
        CHECK_THAT(e, ContainsSubstring("above the switching band maximum 200000 Hz"));
    }
}

TEST_CASE("SRC: a pinned tank runs at its own resonance when that lies a few ppm above the design frequency",
          "[input-range][src][regulation]") {
    // 50 uH / 50.66 nF resonate at 100000.58 Hz: within design_src's 1 % acceptance of the 100 kHz design frequency,
    // but 5.8 ppm above it, which analytical_src refuses as below-resonance. The reference point is the tank's own
    // resonance (gain 1), single- and multi-output.
    for (const auto& outs : {std::vector<std::pair<double, double>>{{48.0, 500.0 / 48.0}},
                             std::vector<std::pair<double, double>>{{48.0, 500.0 / 48.0}, {12.0, 50.0 / 12.0}}}) {
        json s = ranged_spec(300.0, 300.0, 300.0, 300.0, outs, 100e3, 0.9);
        s["designRequirements"]["desiredResonantInductance"] = 5e-5;
        s["designRequirements"]["desiredResonantCapacitance"] = 5.066e-8;
        const auto d = Kirchhoff::design_src(s);
        const double ftank = 1.0 / (2.0 * M_PI * std::sqrt(5e-5 * 5.066e-8));
        CHECK(d.resonantInductance == 5e-5);
        CHECK(d.resonantCapacitance == 5.066e-8);
        CHECK(d.switchingFrequency == ftank);
        const json res = run("src", s);
        CHECK(res.at("diagnostics").at("switchingFrequency").get<double>() == ftank);
    }
}

TEST_CASE("PFC: designed at the nominal line, run at the operating point's line voltage", "[input-range][pfc]") {
    json s;
    json& dr = s["designRequirements"];
    dr["efficiency"] = 0.95;
    dr["inputType"] = "acSinglePhase";
    dr["inputVoltage"] = {{"minimum", 90.0}, {"nominal", 230.0}, {"maximum", 265.0}};
    dr["lineFrequency"]["nominal"] = 50.0;
    dr["switchingFrequency"]["nominal"] = 65e3;
    dr["outputs"] = json::array({{{"name", "out"}, {"regulation", "voltage"}, {"voltage", {{"nominal", 400.0}}}}});
    s["operatingPoints"] = json::array(
        {{{"name", "op"}, {"inputVoltage", 230.0}, {"ambientTemperature", 25.0},
          {"outputs", json::array({{{"name", "out"}, {"power", 300.0}}})}}});
    const auto nominal = Kirchhoff::design_pfc(s);
    const auto lowLine = Kirchhoff::design_pfc(at_input_voltage(s, 115.0));
    CHECK(lowLine.inputVoltageRms == 115.0);
    CHECK(lowLine.designInputVoltageRms == 230.0);
    CHECK(lowLine.boostInductance == nominal.boostInductance);
    CHECK(lowLine.outputCapacitance == nominal.outputCapacitance);
    // The current reference follows the operating line: half the line voltage, twice the emulated conductance.
    CHECK_THAT(lowLine.referenceGain, WithinRel(4.0 * nominal.referenceGain, 1e-12));
    // Through the api: the same inductor, and the line current of the lower line.
    const json r230 = run("pfc", s), r115 = run("pfc", at_input_voltage(s, 115.0));
    CHECK(part_values(r230) == part_values(r115));
    CHECK(r115.at("diagnostics").at("primaryRmsCurrent").get<double>() >
          1.8 * r230.at("diagnostics").at("primaryRmsCurrent").get<double>());
    // A boost PFC cannot run with the line peak above its output: refused at the operating point.
    CHECK_THAT(run_error("pfc", at_input_voltage(s, 290.0)), ContainsSubstring("at the operating line voltage"));
}

TEST_CASE("Design pins: honoured, and a malformed pin throws", "[input-range][pins]") {
    SECTION("config.outputCapacitance (flyback)") {
        json s = ranged_spec(36.0, 48.0, 72.0, 72.0, {{12.0, 2.0}}, 100e3, 0.9);
        s["config"]["outputCapacitance"] = 123e-6;
        bool found = false;
        const json r = run("flyback", s);
        for (const auto& c : r.at("diagnostics").at("capacitors"))
            if (c.at("name") == "Cout") { found = true; CHECK(c.at("capacitance").get<double>() == 123e-6); }
        CHECK(found);
        s["config"]["outputCapacitance"] = "123u";
        CHECK_THAT(run_error("flyback", s), ContainsSubstring("config.outputCapacitance must be a number"));
        s["config"]["outputCapacitance"] = json::array({1e-4, 1e-4});
        CHECK_THAT(run_error("flyback", s), ContainsSubstring("has 2 values but the design has 1 outputs"));
    }
    SECTION("config.outputInductance (forward family)") {
        json s = customer_acf(74.5);
        s["config"]["outputInductance"] = -5e-6;
        CHECK_THAT(run_error("acf", s), ContainsSubstring("config.outputInductance must be a positive, finite value"));
        s["config"]["outputInductance"] = 0.0;
        CHECK_THAT(run_error("forward", s), ContainsSubstring("config.outputInductance must be a positive, finite value"));
    }
    SECTION("config.inductanceL2 / config.couplingCapacitance (SEPIC, Cuk, Zeta)") {
        json s = ranged_spec(9.0, 12.0, 15.0, 12.0, {{12.0, 1.0}}, 100e3, 0.9);
        s["config"]["inductanceL2"] = -1.0;
        CHECK_THAT(run_error("sepic", s), ContainsSubstring("config.inductanceL2"));
        s["config"].erase("inductanceL2");
        s["config"]["couplingCapacitance"] = "big";
        CHECK_THAT(run_error("cuk", s), ContainsSubstring("config.couplingCapacitance"));
        CHECK_THAT(run_error("zeta", s), ContainsSubstring("config.couplingCapacitance"));
    }
    SECTION("config.snubberCap (energy-budget snubbers)") {
        json s = ranged_spec(36.0, 48.0, 72.0, 48.0, {{12.0, 5.0}}, 100e3, 0.9);
        s["config"]["snubberCap"] = 0.0;
        CHECK_THAT(run_error("push_pull", s), ContainsSubstring("config.snubberCap must be a positive, finite"));
        s["config"]["snubberCap"] = 1e-9;
        const json r = run("push_pull", s);
        int snubbers = 0;
        for (const auto& c : r.at("diagnostics").at("capacitors"))
            if (c.at("name").get<std::string>().rfind("Csn", 0) == 0) {
                ++snubbers;
                CHECK(c.at("capacitance").get<double>() == 1e-9);
            }
        CHECK(snubbers > 0);
    }
}

TEST_CASE("Sourcing ratings are the worst case over the input range", "[input-range][ratings]") {
    // Without pins, every topology whose parts are sized from the range gives one part list; the ratings follow
    // the worst corner even where the operating point is the mildest one.
    for (const std::string topology : {"flyback", "forward", "boost", "push_pull", "full_bridge"}) {
        INFO(topology);
        const bool boostClass = topology == "boost";
        const json s = boostClass ? ranged_spec(9.0, 12.0, 15.0, 12.0, {{24.0, 1.0}}, 100e3, 0.9)
                                  : ranged_spec(36.0, 48.0, 72.0, 48.0, {{12.0, 5.0}}, 100e3, 0.9);
        const json lo = run(topology, at_input_voltage(s, boostClass ? 9.0 : 36.0));
        const json hi = run(topology, at_input_voltage(s, boostClass ? 15.0 : 72.0));
        const auto a = sourcing(lo), b = sourcing(hi);
        REQUIRE(a.size() == b.size());
        for (const auto& [k, v] : a) {
            INFO(k << ": " << v.dump() << " vs " << b.at(k).dump());
            CHECK(b.at(k) == v);
        }
    }
    // A spec without a range is rated at its one input voltage (the corners are that voltage).
    const json single = ranged_spec(48.0, 48.0, 48.0, 48.0, {{12.0, 5.0}}, 100e3, 0.9);
    const json ranged = ranged_spec(36.0, 48.0, 72.0, 48.0, {{12.0, 5.0}}, 100e3, 0.9);
    const auto a = sourcing(run("flyback", single)), b = sourcing(run("flyback", ranged));
    bool higher = false;
    for (const auto& [k, v] : b)
        if (v.contains("ratedDrainSourceVoltage") &&
            v.at("ratedDrainSourceVoltage").get<double>() > a.at(k).at("ratedDrainSourceVoltage").get<double>())
            higher = true;
    CHECK(higher);   // the 72 V corner rates the switch above what 48 V alone would
}

TEST_CASE("Forward family: the duty-sized turns ratio runs at the range minimum", "[input-range][forward]") {
    // 43.2-52.8 V -> 12 V / 5 A. n = η·Vin_min·0.5/(Vout+Vd) rounded to 2 dp can round UP, putting D(Vin_min) just
    // past the 0.5 reset limit; capped only at the operating point (or the range nominal) the design then could not
    // run at its own range minimum and refused to be rated over the range. The cap is at Vin_min.
    const json s = ranged_spec(43.2, 48.0, 52.8, 48.0, {{12.0, 5.0}}, 100e3, 1.0);
    {
        const auto d = Kirchhoff::design_forward(s);
        CHECK(d.turnsRatio * (d.outputVoltage + d.diodeDrop) / 43.2 <= 0.5);
    }
    {
        const auto d = Kirchhoff::design_two_switch_forward(s);
        CHECK(d.turnsRatio * (d.outputVoltage + d.diodeDrop) / 43.2 <= 0.5);
    }
    for (const char* topo : {"forward", "two_switch_forward"}) {
        INFO(topo);
        require_same_design(run(topo, s), run(topo, at_input_voltage(s, 43.2)));
    }
}

TEST_CASE("A component without ratings may exist at only some corners: it is not folded", "[input-range][ratings]") {
    // The web PFC wizard (85-265 V, run at 175 V, 250 uH pinned): at 175 V the inductance is too small for
    // continuous conduction at the line peak and the control stage runs average-current control (oscillator
    // 'Osc'); at 85 V it runs the hysteretic law. 'Osc' carries no rating, so the fold does not look for it at 85 V.
    json s;
    json& dr = s["designRequirements"];
    dr["inputType"] = "acSinglePhase";
    dr["inputVoltage"] = {{"minimum", 85.0}, {"maximum", 265.0}};
    dr["switchingFrequency"]["nominal"] = 65e3;
    dr["outputs"] = json::array({{{"name", "out"}, {"voltage", {{"nominal", 400.0}}}, {"regulation", "voltage"}}});
    dr["efficiency"] = 0.95;
    dr["lineFrequency"]["nominal"] = 50.0;
    dr["magnetizingInductance"]["nominal"] = 250e-6;
    s["operatingPoints"] = json::array({{{"name", "full_load"}, {"inputVoltage", 175.0}, {"ambientTemperature", 25.0},
                                         {"outputs", json::array({{{"name", "out"}, {"power", 300.0}}})}}});
    s["config"] = {{"currentRippleFraction", 0.3}, {"diodeVoltageDrop", 0.6}, {"topologyVariant", "boost"}};
    const json r = run("pfc", s);
    bool hasOsc = false;
    for (const auto& st : r.at("tas").at("topology").at("stages"))
        for (const auto& c : st.at("circuit").value("components", json::array()))
            if (c.value("name", std::string()) == "Osc") hasOsc = true;
    CHECK(hasOsc);   // premise: the operating point really runs the law with the oscillator
}

TEST_CASE("A design that cannot run at a corner of its range is refused, naming the corner",
          "[input-range][ratings]") {
    // n = 1 two-switch forward: 13 V needs D = 0.57 at 24 V, past the reset limit of 0.5. The operating point
    // (74.5 V) runs, but the parts cannot be rated over a range the design does not cover.
    json s = customer_acf(74.5);
    const std::string e = run_error("two_switch_forward", s);
    CHECK_THAT(e, ContainsSubstring("rating the parts over the input range"));
    CHECK_THAT(e, ContainsSubstring("cannot be evaluated at the input voltage 24 V"));
    // A spec refused at its own operating point keeps its own message.
    const std::string own = run_error("two_switch_forward", at_input_voltage(s, 24.0));
    CHECK_THAT(own, !ContainsSubstring("rating the parts over the input range"));
}
