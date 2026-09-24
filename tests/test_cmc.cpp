// Common-mode choke designer — the requirement-synthesis half ported from MKF
// converter_models/CommonModeChoke.{h,cpp} (spec modes → impedance points → required L_cm) plus the
// MAS::Inputs assembly over analytical_common_mode_choke, and the Kirchhoff::api facade entries
// (design_cmc / design_magnetic_inputs) the OpenMagnetics Wizard & PyOM consume.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Cmc.hpp"
#include "ChokeDesign.hpp"
#include "KirchhoffApi.hpp"

#include <cmath>
#include <regex>
#include <string>

using json = nlohmann::json;
using Catch::Approx;

namespace {

// The CmcWizard's flat aux payload, noise-estimation mode (its defaults).
json wizard_spec() {
    return json{
        {"operatingVoltage", 230.0}, {"operatingCurrent", 6.0},
        {"lineFrequency", 50.0},     {"ambientTemperature", 25.0},
        {"lineImpedance", 50.0},     {"numberOfWindings", 2},
        {"parasiticCap_pF", 100.0},  {"dvdt_V_ns", 5.0},
        {"safetyMargin_dB", 6.0},    {"regulatoryStandard", "CISPR 32 Class B"},
    };
}

} // namespace

TEST_CASE("cmc spec-conversion helpers match the MKF formulas", "[cmc][helpers]") {
    // L = Z/(2πf) and its inverse round-trip (shared reactance core, ChokeDesign.hpp).
    const double L = Kirchhoff::impedance_to_inductance(1000.0, 150e3);
    CHECK(L == Approx(1000.0 / (2.0 * M_PI * 150e3)));
    CHECK(Kirchhoff::inductance_to_impedance(L, 150e3) == Approx(1000.0));

    // IL → Z: 20 dB over 50 Ω → 50·(10^1 − 1) = 450 Ω.
    CHECK(Kirchhoff::cmc_insertion_loss_to_impedance(20.0, 50.0) == Approx(450.0));

    // Noise params: 100 pF · 5 V/ns = 0.5 A CM; V = 0.5·25 = 12.5 V = 141.94 dBµV;
    // atten = 141.94 − 66 + 6 = 81.94 dB; Z = 25·10^(81.94/20).
    const double icm = 100e-12 * 5e9;
    const double vDbuv = 20.0 * std::log10(icm * 25.0 / 1e-6);
    const double expectedZ = 25.0 * std::pow(10.0, (vDbuv - 66.0 + 6.0) / 20.0);
    CHECK(Kirchhoff::cmc_noise_params_to_impedance(100.0, 5.0, 50.0, 6.0) == Approx(expectedZ));

    CHECK(Kirchhoff::cmc_emissions_limit_dbuv("CISPR 32 Class B") == 66.0);
    CHECK(Kirchhoff::cmc_emissions_limit_dbuv("EN 55032 Class B") == 66.0);  // EN alias of CISPR 32
    CHECK(Kirchhoff::cmc_emissions_limit_dbuv("FCC Part 15 Class A") == 79.0);
    CHECK_THROWS(Kirchhoff::cmc_emissions_limit_dbuv("EN 12345"));  // no silent 66 fallback
}

TEST_CASE("design_cmc: minimumImpedance mode — hardest point wins", "[cmc][design]") {
    json spec = wizard_spec();
    // 1000 Ω @ 150 kHz → 1.06 mH beats 2000 Ω @ 1 MHz → 0.318 mH.
    spec["minimumImpedance"] = json::array({
        {{"frequency", 150e3}, {"impedance", 1000.0}},
        {{"frequency", 1e6},   {"impedance", 2000.0}},
    });
    Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(spec);
    CHECK(d.impedancePoints.size() == 2);
    CHECK(d.computedInductance == Approx(1000.0 / (2.0 * M_PI * 150e3)));
    CHECK(d.dominantFrequency == Approx(150e3));
    CHECK(d.dominantImpedance == Approx(1000.0));
    // Explicit spec present → NO noise-synthesized third point, but the noise params still set the
    // excitation amplitude.
    CHECK(d.parasiticCapPf == Approx(100.0));
    CHECK(d.dvdtVPerNs == Approx(5.0));
}

TEST_CASE("design_cmc: targetInsertionLoss converts through the line impedance", "[cmc][design]") {
    json spec = wizard_spec();
    spec["targetInsertionLoss"] = json::array({{{"frequency", 150e3}, {"insertionLoss", 20.0}}});
    Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(spec);
    REQUIRE(d.impedancePoints.size() == 1);
    CHECK(d.impedancePoints[0].get_impedance().get_magnitude() == Approx(450.0));
    CHECK(d.computedInductance == Approx(450.0 / (2.0 * M_PI * 150e3)));
}

TEST_CASE("design_cmc: noise-estimation mode synthesizes the 150 kHz point", "[cmc][design]") {
    Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(wizard_spec());
    REQUIRE(d.impedancePoints.size() == 1);
    CHECK(d.impedancePoints[0].get_frequency() == Approx(150e3));
    const double expectedZ = Kirchhoff::cmc_noise_params_to_impedance(100.0, 5.0, 50.0, 6.0);
    CHECK(d.impedancePoints[0].get_impedance().get_magnitude() == Approx(expectedZ));
    CHECK(d.dominantFrequency == Approx(150e3));
}

TEST_CASE("design_cmc throws on missing/invalid required data", "[cmc][design]") {
    CHECK_THROWS(Kirchhoff::design_cmc(json::object()));            // everything missing
    json noSpec = wizard_spec();
    noSpec.erase("parasiticCap_pF");                                 // no mode left at all
    CHECK_THROWS(Kirchhoff::design_cmc(noSpec));
    json badWindings = wizard_spec();
    badWindings["numberOfWindings"] = 5;
    CHECK_THROWS(Kirchhoff::design_cmc(badWindings));
    json badPoint = wizard_spec();
    badPoint["minimumImpedance"] = json::array({{{"frequency", -1.0}, {"impedance", 100.0}}});
    CHECK_THROWS(Kirchhoff::design_cmc(badPoint));
}

TEST_CASE("design_cmc rejects a partial or non-positive noise spec (no silent 10 mA fallback)",
          "[cmc][design]") {
    // Half a noise spec would silently zero I_cm = C·dV/dt and undersize the excitation 50× —
    // it must throw EVEN when another spec mode (minimumImpedance) makes the design viable.
    json half = wizard_spec();
    half["minimumImpedance"] = json::array({{{"frequency", 150e3}, {"impedance", 1000.0}}});
    half.erase("dvdt_V_ns");
    CHECK_THROWS(Kirchhoff::design_cmc(half));
    json negative = wizard_spec();
    negative["minimumImpedance"] = json::array({{{"frequency", 150e3}, {"impedance", 1000.0}}});
    negative["dvdt_V_ns"] = -5.0;
    CHECK_THROWS(Kirchhoff::design_cmc(negative));
}

TEST_CASE("build_cmc_inputs: designRequirements carry the full CMC contract", "[cmc][inputs]") {
    json spec = wizard_spec();
    spec["numberOfWindings"] = 3;
    spec["minimumImpedance"] = json::array({{{"frequency", 150e3}, {"impedance", 1000.0}}});
    Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(spec);
    MAS::Inputs in = Kirchhoff::build_cmc_inputs(d);

    const auto& dr = in.get_design_requirements();
    CHECK(dr.get_application() == std::string("interferenceSuppression"));
    CHECK(dr.get_sub_application() == std::string("commonModeNoiseFiltering"));
    // MKF parity: CommonModeChoke::process_design_requirements did NOT set the topology field (DMC/CT
    // did). Preserved so the MagneticAdviser sees the exact requirements it saw pre-cutover.
    CHECK_FALSE(dr.get_topology().has_value());
    // 3 windings → 2 turns ratios, all exactly 1:1.
    REQUIRE(dr.get_turns_ratios().size() == 2);
    for (const auto& tr : dr.get_turns_ratios()) CHECK(tr.get_nominal() == Approx(1.0));
    // L_cm from the impedance spec is a MINIMUM bound (nominal/maximum unset), rounded to 10
    // decimals exactly as MKF's roundFloat(computedInductance, 10) emitted it.
    CHECK(dr.get_magnetizing_inductance().get_minimum().value()
          == Approx(std::round(d.computedInductance * 1e10) / 1e10).epsilon(1e-12));
    CHECK_FALSE(dr.get_magnetizing_inductance().get_nominal().has_value());
    REQUIRE(dr.get_minimum_impedance().has_value());
    CHECK(dr.get_minimum_impedance()->size() == 1);
    // NB: MAS getters return std::optional BY VALUE — copy it before iterating (never range over
    // *getter(): the temporary optional dies and the loop reads freed memory).
    const auto sides = dr.get_isolation_sides();
    REQUIRE(sides.has_value());
    CHECK(sides->size() == 3);
    for (auto s : *sides) CHECK(s == MAS::IsolationSide::PRIMARY);

    // One operating point, one excitation per winding, at the dominant frequency. The CM ripple rides on each
    // winding's DIFFERENTIAL-mode level: MAS convention (2026-09-24) — line and return currents flow in
    // opposite directions through the dotted windings, so the three-phase snapshot at phase A's peak is
    // +6, −3, −3 A (Σ = 0: the DM flux cancels). Before, every winding carried +6 A, i.e. 18 A of DM
    // ampere-turns that a common-mode choke never sees.
    REQUIRE(in.get_operating_points().size() == 1);
    const auto& op = in.get_operating_points()[0];
    REQUIRE(op.get_excitations_per_winding().size() == 3);
    const double dm[] = {6.0, -3.0, -3.0};
    for (size_t w = 0; w < 3; ++w) {
        const auto& exc = op.get_excitations_per_winding()[w];
        CHECK(exc.get_frequency() == Approx(150e3));
        auto cur = exc.get_current();
        REQUIRE(cur.has_value());
        REQUIRE(cur->get_processed().has_value());
        CHECK(cur->get_processed()->get_offset() == Approx(dm[w]).margin(1e-6));
    }
    CHECK(op.get_conditions().get_ambient_temperature() == Approx(25.0));
}

TEST_CASE("build_cmc_inputs: advanced mode pins L nominal and excites at designFrequency",
          "[cmc][inputs][advanced]") {
    json spec = wizard_spec();
    spec["desiredInductance"] = 2e-3;
    spec["designFrequency"] = 250e3;
    Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(spec);
    MAS::Inputs in = Kirchhoff::build_cmc_inputs(d);

    const auto& lmSpec = in.get_design_requirements().get_magnetizing_inductance();
    CHECK(lmSpec.get_nominal().value() == Approx(2e-3));
    CHECK_FALSE(lmSpec.get_minimum().has_value());
    // Excitation frequency follows designFrequency, not the noise-synthesized 150 kHz point.
    CHECK(in.get_operating_points()[0].get_excitations_per_winding()[0].get_frequency()
          == Approx(250e3));
    // V = L·ω·(Σ I_cm) = n·L·ω·I_cm: the pinned L shapes the voltage amplitude (I_cm = C·dV/dt = 0.5 A at
    // 230 V per winding). MAS convention (2026-09-24): v_k = L·d(i_m)/dt with i_m = Σ_k i_k for the fully
    // coupled CM windings — n times the former per-winding L·ω·I_cm.
    auto vol = in.get_operating_points()[0].get_excitations_per_winding()[0].get_voltage();
    REQUIRE(vol.has_value());
    REQUIRE(vol->get_processed().has_value());
    const double iCm = 100.0 * 5.0 * 1e-3;  // C·dV/dt in A, 230 V scaling is a no-op
    CHECK(vol->get_processed()->get_peak().value()
          == Approx(d.numberOfWindings * 2e-3 * 2.0 * M_PI * 250e3 * iCm).epsilon(0.02));
}

TEST_CASE("api::design_cmc returns {inputs, cmcDiagnostics} and stays schema-clean",
          "[cmc][api]") {
    json out = json::parse(Kirchhoff::api::design_cmc(wizard_spec().dump()));
    REQUIRE(out.contains("inputs"));
    REQUIRE(out.contains("cmcDiagnostics"));
    // The Inputs object itself carries ONLY schema keys (diagnostics live outside it).
    CHECK_FALSE(out["inputs"].contains("cmcDiagnostics"));
    CHECK(out["inputs"]["designRequirements"]["subApplication"] == "commonModeNoiseFiltering");
    CHECK(out["cmcDiagnostics"]["computedInductance"].get<double>() > 0.0);
    CHECK(out["cmcDiagnostics"]["dominantFrequency"].get<double>() == Approx(150e3));

    // Errors surface as the facade's "Exception: " string, never a throw across the boundary.
    const std::string err = Kirchhoff::api::design_cmc("{}");
    CHECK(err.rfind("Exception: ", 0) == 0);
}

TEST_CASE("api::design_magnetic_inputs serves every topology AND the CMC particular case",
          "[cmc][api]") {
    // CMC aliases route to the component designer (bare MAS::Inputs, same as design_cmc's inputs).
    json viaAlias = json::parse(
        Kirchhoff::api::design_magnetic_inputs("common_mode_choke", wizard_spec().dump()));
    json viaCmc = json::parse(Kirchhoff::api::design_cmc(wizard_spec().dump()))["inputs"];
    CHECK(viaAlias == viaCmc);
    CHECK(json::parse(Kirchhoff::api::design_magnetic_inputs("cmc", wizard_spec().dump()))
          == viaAlias);

    // A switching topology goes through design_tas + main_magnetic_inputs (existing logic).
    json buck;
    buck["designRequirements"]["efficiency"] = 1.0;
    buck["designRequirements"]["inputVoltage"] = {{"minimum", 43.2}, {"nominal", 48.0}, {"maximum", 52.8}};
    buck["designRequirements"]["switchingFrequency"]["nominal"] = 100e3;
    buck["designRequirements"]["outputs"] =
        json::array({ {{"name", "out"}, {"voltage", {{"nominal", 12.0}}}} });
    buck["operatingPoints"] =
        json::array({ {{"inputVoltage", 48.0}, {"outputs", json::array({{{"power", 24.0}}})}} });
    json buckInputs = json::parse(Kirchhoff::api::design_magnetic_inputs("buck", buck.dump()));
    REQUIRE(buckInputs.contains("designRequirements"));
    REQUIRE(buckInputs.contains("operatingPoints"));
    CHECK(buckInputs["designRequirements"]["magnetizingInductance"]["nominal"].get<double>() > 0.0);

    // Unknown topology surfaces loudly.
    CHECK(Kirchhoff::api::design_magnetic_inputs("teleporter", "{}").rfind("Exception: ", 0) == 0);
}

TEST_CASE("api::simulate_cmc_* run the ngspice decks (or report success:false without libngspice)",
          "[cmc][api][ngspice]") {
    // Impedance-mode spec so the LISN sweep has a frequency (150 kHz) to test at.
    json spec = wizard_spec();
    spec["minimumImpedance"] = json::array({{{"frequency", 150000.0}, {"impedance", 1000.0}}});
    const double L = 1.061e-3;   // ~ the computed L for 1 kΩ @ 150 kHz

    json ideal = json::parse(Kirchhoff::api::simulate_cmc_ideal_waveforms(spec.dump(), L, 100.0, 5.0));
    json lisn = json::parse(Kirchhoff::api::simulate_cmc_lisn_waveforms(spec.dump(), L));
    if (!ideal.value("success", false)) {
        // Surfaced, not silently passed — built without libngspice.
        WARN("ngspice unavailable: " + ideal.value("error", std::string("(no error)")));
        CHECK(ideal.contains("error"));
        CHECK(lisn.value("success", false) == false);
        return;
    }
    // Ideal sim: one operating point with a per-winding V/I waveform each.
    REQUIRE(ideal["inputs"]["operatingPoints"].is_array());
    REQUIRE(ideal["inputs"]["operatingPoints"].size() == 1);
    auto excs = ideal["inputs"]["operatingPoints"][0]["excitationsPerWinding"];
    CHECK(excs.size() == 2);   // Line + Neutral
    CHECK(ideal["cmcDiagnostics"]["computedInductance"].get<double>() > 0.0);

    // ABT #1356: each excitation is ONE steady-state period of the 150 kHz sine drive. The deck runs
    // numberOfPeriods = 2 by default; handing both to complete_excitation (which reads a timed
    // waveform's span as one period) put the fundamental at 300 kHz with ~266 % THD.
    const double excFreq = 150000.0;
    for (const auto& e : excs) {
        CHECK(e["frequency"].get<double>() == Approx(excFreq));
        for (const char* side : {"current", "voltage"}) {
            INFO("winding " << e["name"].get<std::string>() << ", " << side);
            const auto& t = e[side]["waveform"]["time"];
            REQUIRE(t.size() >= 2);
            // The stored waveform spans one period (128 points on [0, T), so last = T·127/128).
            CHECK(t.back().get<double>() - t.front().get<double>() == Approx(1.0 / excFreq).epsilon(0.02));
            const auto& p = e[side]["processed"];
            CHECK(p["acEffectiveFrequency"].get<double>() == Approx(excFreq).epsilon(0.02));
            CHECK(p["thd"].get<double>() < 0.05);
            CHECK(p["label"].get<std::string>() == "sinusoidal");
        }
    }

    // LISN sim: one converterWaveforms entry per spec frequency, with a finite attenuation.
    REQUIRE(lisn.value("success", false));
    REQUIRE(lisn["converterWaveforms"].is_array());
    REQUIRE(lisn["converterWaveforms"].size() == 1);
    const auto& cw = lisn["converterWaveforms"][0];
    CHECK(cw["frequency"].get<double>() == Approx(150000.0));
    CHECK(cw["theoreticalImpedance"].get<double>() == Approx(2.0 * M_PI * 150000.0 * L));
    CHECK(std::isfinite(cw["commonModeAttenuation"].get<double>()));
    CHECK(cw["time"].is_array());
    CHECK(cw["windingCurrents"].size() == 2);
}

TEST_CASE("api::generate_cmc_ngspice_circuit returns the ideal CM deck as netlist text", "[cmc][api][netlist]") {
    // The wizard's SPICE button: a real ngspice netlist (the deck simulate_cmc_ideal_waveforms runs), not
    // the design JSON the web runtime used to hand back.
    json spec = wizard_spec();
    spec["minimumImpedance"] = json::array({{{"frequency", 150000.0}, {"impedance", 1000.0}}});
    spec["parasiticCap_pF"] = 100.0;
    spec["dvdt_V_ns"] = 5.0;
    const std::string net = Kirchhoff::api::generate_cmc_ngspice_circuit(spec.dump());
    INFO(net);
    REQUIRE(net.rfind("Exception", 0) != 0);
    CHECK(std::regex_search(net, std::regex(R"(^\s*\.tran\b)", std::regex::multiline)));
    CHECK(std::regex_search(net, std::regex(R"(^\s*\.end\b)", std::regex::multiline)));
    // One CM source + one inductor per winding (Line + Neutral), at the design's inductance and 150 kHz.
    CHECK(std::regex_search(net, std::regex(R"(^Icm0 .*SIN\(.* 150000 )", std::regex::multiline)));
    CHECK(std::regex_search(net, std::regex(R"(^Icm1 )", std::regex::multiline)));
    CHECK(std::regex_search(net, std::regex(R"(^Lcmc0 )", std::regex::multiline)));
    CHECK(std::regex_search(net, std::regex(R"(^Lcmc1 )", std::regex::multiline)));
    const Kirchhoff::CmcDesign d = Kirchhoff::design_cmc(spec);
    std::smatch m;
    REQUIRE(std::regex_search(net, m, std::regex(R"(^Lcmc0 \S+ \S+ (\S+))", std::regex::multiline)));
    CHECK(std::stod(m[1]) == Approx(d.computedInductance).epsilon(1e-6));

    // Without the noise spec the CM source has no amplitude: loud error, never a DC-only deck.
    json noNoise = wizard_spec();
    noNoise["minimumImpedance"] = spec["minimumImpedance"];
    noNoise.erase("parasiticCap_pF");
    noNoise.erase("dvdt_V_ns");
    CHECK(Kirchhoff::api::generate_cmc_ngspice_circuit(noNoise.dump()).rfind("Exception: ", 0) == 0);
}
