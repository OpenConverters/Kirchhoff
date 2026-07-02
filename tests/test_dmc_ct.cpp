// Differential-mode choke + current-transformer designers — the requirement-synthesis halves ported
// from MKF converter_models/{DifferentialModeChoke,CurrentTransformer}.{h,cpp} (deleted in 3e0261fd),
// built on the SHARED ChokeDesign core (impedance_to_inductance / filter_choke_requirements / make_inputs)
// alongside CMC. Covers the Kirchhoff::api facade (design_dmc / propose_dmc_design /
// design_current_transformer / design_magnetic_inputs) and MKF design-requirements parity.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Dmc.hpp"
#include "CurrentTransformer.hpp"
#include "ChokeDesign.hpp"
#include "KirchhoffApi.hpp"

#include <cmath>
#include <string>

using json = nlohmann::json;
using Catch::Approx;

namespace {

// DmcWizard flat aux payload, impedance (help) mode.
json dmc_spec() {
    return json{
        {"configuration", "singlePhase"}, {"inputVoltage", 230.0}, {"operatingCurrent", 8.0},
        {"lineFrequency", 50.0},          {"switchingFrequency", 100000.0},
        {"ambientTemperature", 40.0},
        {"minimumImpedance", json::array({{{"frequency", 150000.0}, {"impedance", 800.0}},
                                          {{"frequency", 500000.0}, {"impedance", 1500.0}}})},
    };
}

// process_current_transformer flat payload. turnsRatio is MKF's Np/Ns factor mapping primary→secondary
// current (I_sec = I_pri·turnsRatio); a 1:100 step-down CT is turnsRatio = 0.01.
json ct_spec() {
    return json{
        {"waveformLabel", "sinusoidal"}, {"maximumPrimaryCurrentPeak", 20.0},
        {"frequency", 100000.0},         {"turnsRatio", 0.01},
        {"burdenResistor", 10.0},        {"ambientTemperature", 25.0},
    };
}

} // namespace

TEST_CASE("design_dmc: L_min taken at the LOWEST impedance-spec frequency (MKF parity)", "[dmc][design]") {
    Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(dmc_spec());
    // MKF sizes L_min from Z at the LOWEST frequency (150 kHz, 800 Ω) — NOT the max-L rule CMC uses.
    CHECK(d.computedInductance == Approx(Kirchhoff::impedance_to_inductance(800.0, 150000.0)));
    CHECK(d.computedMinFrequency == Approx(150000.0));
    CHECK(d.computedMaxFrequency == Approx(500000.0));
    CHECK(d.computedImpedanceAtMinFreq == Approx(800.0));
    CHECK(d.numberOfWindings == 1);   // singlePhase
    CHECK(d.impedancePoints.size() == 2);
}

TEST_CASE("design_dmc: winding count from configuration; minimumInductance mode", "[dmc][design]") {
    json spec = dmc_spec();
    spec["configuration"] = "threePhaseWithNeutral";
    spec.erase("minimumImpedance");
    spec["minimumInductance"] = 1.2e-3;
    Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(spec);
    CHECK(d.numberOfWindings == 4);
    CHECK(d.computedInductance == Approx(1.2e-3));
    CHECK(d.impedancePoints.empty());
}

TEST_CASE("design_dmc throws when no inductance target is derivable", "[dmc][design]") {
    json spec = dmc_spec();
    spec.erase("minimumImpedance");   // no minimumInductance either
    CHECK_THROWS(Kirchhoff::design_dmc(spec));
    json missing = dmc_spec();
    missing.erase("operatingCurrent");
    CHECK_THROWS(Kirchhoff::design_dmc(missing));
}

TEST_CASE("build_dmc_inputs: MKF-parity design requirements", "[dmc][inputs]") {
    json spec = dmc_spec();
    spec["configuration"] = "threePhase";
    Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(spec);
    MAS::Inputs in = Kirchhoff::build_dmc_inputs(d);
    const auto& dr = in.get_design_requirements();

    CHECK(dr.get_application() == std::string("interferenceSuppression"));
    CHECK(dr.get_sub_application() == std::string("differentialModeNoiseFiltering"));
    // DMC DID set the topology in MKF (unlike CMC) — MagneticFilterSaturation routes off it.
    CHECK(dr.get_topology() == MAS::Topology::DIFFERENTIAL_MODE_CHOKE);
    // 3 windings → 2 exactly-1:1 turns ratios, all-primary isolation.
    REQUIRE(dr.get_turns_ratios().size() == 2);
    for (const auto& tr : dr.get_turns_ratios()) CHECK(tr.get_nominal() == Approx(1.0));
    const auto sides = dr.get_isolation_sides();
    REQUIRE(sides.has_value());
    CHECK(sides->size() == 3);
    for (auto s : *sides) CHECK(s == MAS::IsolationSide::PRIMARY);
    // L_min bound, impedance points forwarded.
    CHECK(dr.get_magnetizing_inductance().get_minimum().value() == Approx(d.computedInductance));
    CHECK_FALSE(dr.get_magnetizing_inductance().get_nominal().has_value());
    REQUIRE(dr.get_minimum_impedance().has_value());
    CHECK(dr.get_minimum_impedance()->size() == 2);
    // One operating point, one excitation per winding.
    REQUIRE(in.get_operating_points().size() == 1);
    CHECK(in.get_operating_points()[0].get_excitations_per_winding().size() == 3);
}

TEST_CASE("dmc_required_inductance + propose_dmc_design synthesize a consistent LC pair", "[dmc][propose]") {
    // L = 1/(4π²fc²C) with fc = f/10^(A/40).
    const double L = Kirchhoff::dmc_required_inductance(40.0, 100000.0, 1e-6);
    const double fc = 100000.0 / std::pow(10.0, 40.0 / 40.0);
    CHECK(L == Approx(1.0 / (4.0 * M_PI * M_PI * fc * fc * 1e-6)));

    json spec = dmc_spec();
    spec.erase("minimumImpedance");   // help mode: no impedance target yet
    json p = Kirchhoff::propose_dmc_design(spec);
    CHECK(p["inductance"].get<double>() > 0.0);
    CHECK(p["capacitance"].get<double>() > 0.0);
    CHECK(p["numberOfWindings"].get<int>() == 1);
    CHECK(p["configuration"].get<std::string>() == "singlePhase");
    // Re-derive the cutoff and check the LC pair reproduces it: fc = 1/(2π√LC).
    const double Lp = p["inductance"].get<double>(), Cp = p["capacitance"].get<double>();
    const double fcPair = 1.0 / (2.0 * M_PI * std::sqrt(Lp * Cp));
    CHECK(fcPair == Approx(p["cutoffFrequency"].get<double>()).epsilon(1e-6));
}

TEST_CASE("design_current_transformer: MKF-parity requirements + sensing operating point", "[ct][inputs]") {
    MAS::Inputs in = Kirchhoff::design_current_transformer(ct_spec());
    const auto& dr = in.get_design_requirements();

    CHECK(dr.get_topology() == MAS::Topology::CURRENT_TRANSFORMER);
    REQUIRE(dr.get_turns_ratios().size() == 1);
    CHECK(dr.get_turns_ratios()[0].get_nominal() == Approx(0.01));   // round(turnsRatio, 2)
    CHECK(dr.get_magnetizing_inductance().get_minimum().value() == Approx(1e-6));  // CT Lm floor
    const auto sides = dr.get_isolation_sides();
    REQUIRE(sides.has_value());
    REQUIRE(sides->size() == 2);
    CHECK((*sides)[0] == MAS::IsolationSide::PRIMARY);
    CHECK((*sides)[1] == MAS::IsolationSide::SECONDARY);
    // Primary + secondary excitations.
    REQUIRE(in.get_operating_points().size() == 1);
    const auto& op = in.get_operating_points()[0];
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    CHECK(op.get_conditions().get_ambient_temperature() == Approx(25.0));
    // Secondary current = primary·turnsRatio (MKF multiply convention): 20 A × 0.01 = 0.2 A.
    auto secCur = op.get_excitations_per_winding()[1].get_current();
    REQUIRE(secCur.has_value());
    REQUIRE(secCur->get_processed().has_value());
    CHECK(secCur->get_processed()->get_peak().value() == Approx(20.0 * 0.01).epsilon(0.02));
}

TEST_CASE("design_current_transformer rejects bad spec", "[ct][design]") {
    json bad = ct_spec();
    bad["turnsRatio"] = 0.0;
    CHECK_THROWS(Kirchhoff::design_current_transformer(bad));
    json unsup = ct_spec();
    unsup["waveformLabel"] = "bipolarRectangular";   // not a CT-supported label
    CHECK_THROWS(Kirchhoff::design_current_transformer(unsup));
    json missing = ct_spec();
    missing.erase("burdenResistor");
    CHECK_THROWS(Kirchhoff::design_current_transformer(missing));
}

TEST_CASE("api::design_dmc / design_current_transformer envelopes + schema-clean Inputs", "[dmc][ct][api]") {
    json dmcOut = json::parse(Kirchhoff::api::design_dmc(dmc_spec().dump()));
    REQUIRE(dmcOut.contains("inputs"));
    REQUIRE(dmcOut.contains("dmcDiagnostics"));
    CHECK_FALSE(dmcOut["inputs"].contains("dmcDiagnostics"));
    CHECK(dmcOut["inputs"]["designRequirements"]["subApplication"] == "differentialModeNoiseFiltering");
    CHECK(dmcOut["dmcDiagnostics"]["computedInductance"].get<double>() > 0.0);
    CHECK(dmcOut["dmcDiagnostics"]["numberWindings"].get<int>() == 1);

    json propose = json::parse(Kirchhoff::api::propose_dmc_design(dmc_spec().dump()));
    CHECK(propose["inductance"].get<double>() > 0.0);

    json ctOut = json::parse(Kirchhoff::api::design_current_transformer(ct_spec().dump()));
    CHECK(ctOut["designRequirements"]["turnsRatios"][0]["nominal"].get<double>() == Approx(0.01));
    CHECK(ctOut["operatingPoints"][0]["excitationsPerWinding"].size() == 2);

    // Errors surface as the facade's "Exception: " string.
    CHECK(Kirchhoff::api::design_dmc("{}").rfind("Exception: ", 0) == 0);
    CHECK(Kirchhoff::api::design_current_transformer("{}").rfind("Exception: ", 0) == 0);
}

TEST_CASE("api::design_magnetic_inputs routes the DMC + CT particular cases", "[dmc][ct][api]") {
    // DMC alias → bare MAS::Inputs identical to design_dmc's "inputs".
    json viaAlias = json::parse(Kirchhoff::api::design_magnetic_inputs("differential_mode_choke", dmc_spec().dump()));
    json viaDmc = json::parse(Kirchhoff::api::design_dmc(dmc_spec().dump()))["inputs"];
    CHECK(viaAlias == viaDmc);
    CHECK(json::parse(Kirchhoff::api::design_magnetic_inputs("dmc", dmc_spec().dump())) == viaAlias);

    // CT alias → bare MAS::Inputs.
    json ctAlias = json::parse(Kirchhoff::api::design_magnetic_inputs("current_transformer", ct_spec().dump()));
    CHECK(ctAlias["designRequirements"]["topology"] == "currentTransformer");
}

TEST_CASE("api::simulate_dmc_waveforms + verify_dmc_attenuation run the LC decks", "[dmc][api][ngspice]") {
    json spec = dmc_spec();   // has minimumImpedance at 150 kHz + 500 kHz
    Kirchhoff::DmcDesign d = Kirchhoff::design_dmc(spec);
    const double L = d.computedInductance;

    json wf = json::parse(Kirchhoff::api::simulate_dmc_waveforms(spec.dump(), L));
    json ver = json::parse(Kirchhoff::api::verify_dmc_attenuation(spec.dump(), L, 0.0));
    if (!wf.value("success", false)) {
        WARN("ngspice unavailable: " + wf.value("error", std::string("(no error)")));
        CHECK(wf.contains("error"));
        return;
    }
    // One waveform entry per spec frequency, each with a finite DM attenuation.
    REQUIRE(wf["converterWaveforms"].is_array());
    CHECK(wf["converterWaveforms"].size() == 2);
    for (const auto& cw : wf["converterWaveforms"]) {
        CHECK(cw["time"].is_array());
        CHECK(std::isfinite(cw["dmAttenuation"].get<double>()));
    }
    // verify returns one row per test point, with a pass/fail and a message.
    REQUIRE(ver.is_array());
    CHECK(ver.size() == 2);
    for (const auto& r : ver) {
        CHECK(r.contains("passed"));
        CHECK(std::isfinite(r["measuredAttenuation"].get<double>()));
        CHECK(r["message"].get<std::string>().find("kHz") != std::string::npos);
    }
}
