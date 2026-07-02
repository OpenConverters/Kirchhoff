// Integration-surface test — Kirchhoff::extract_operating_point / topology_waveforms.
//
// These are the topology-AGNOSTIC replacements for MKF's per-topology simulate_and_extract trio. They
// operate purely on the assembled TAS document, so one test drives them across an isolated resonant
// topology (LLC: transformer + resonant inductor as separate magnetics), a simple isolated topology
// (flyback), and a non-isolated one (buck). We assert:
//   * topology_waveforms(tas) returns every magnetic with a populated MAS::Inputs and exactly one `isMain`.
//   * extract_operating_point(tas, ANALYTICAL) returns the main magnetic's per-winding excitations.
//   * extract_operating_point(tas, NGSPICE) runs the deck and returns real per-winding currents
//     (skipped — surfaced, not silently passed — when built without libngspice).

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "KirchhoffApi.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

json spec_for(double vin, double vout, double power, double fsw, double eta = 1.0) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name", "out"}, {"voltage", {{"nominal", vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}} });
    return s;
}

// RMS of the current-side processed data — used to assert a winding actually carries current after
// extraction. NB: MAS getters return std::optional BY VALUE — never bind a reference to *getter()
// (it dangles: the temporary optional dies at the end of the full expression).
double current_rms(const MAS::OperatingPointExcitation& exc) {
    auto cur = exc.get_current();
    if (!cur) return 0.0;
    auto proc = cur->get_processed();
    if (!proc) return 0.0;
    auto rms = proc->get_rms();
    return rms ? *rms : 0.0;
}

void check_topology_waveforms(const json& tas, size_t expectedMagnetics) {
    auto mags = Kirchhoff::topology_waveforms(tas);
    REQUIRE(mags.size() == expectedMagnetics);
    size_t mains = 0;
    for (const auto& m : mags) {
        CHECK_FALSE(m.name.empty());
        // every magnetic carries a MAS::Inputs with at least one operating point and one winding
        REQUIRE(m.inputs.get_operating_points().size() >= 1);
        REQUIRE(m.inputs.get_operating_points().at(0).get_excitations_per_winding().size() >= 1);
        if (m.isMain) ++mains;
    }
    CHECK(mains == 1);   // exactly one main magnetic
}

void check_analytical(const json& tas) {
    MAS::OperatingPoint op = Kirchhoff::extract_operating_point(tas, Kirchhoff::ExtractEngine::ANALYTICAL);
    const auto& excs = op.get_excitations_per_winding();
    REQUIRE(excs.size() >= 1);
    // the main magnetic's primary carries current
    CHECK(current_rms(excs.front()) > 0.0);
    for (const auto& e : excs) {
        REQUIRE(e.get_current().has_value());
        CHECK(e.get_frequency() > 0.0);
    }
}

// Number of time-domain samples on a winding's current waveform (0 if none). The analytical TAS carries
// processed-only excitations (no waveform); the NGSPICE extract resamples the simulated branch onto a
// 128-point grid — so a populated waveform PROVES the sim actually replaced the value (not a fallback).
size_t current_waveform_points(const MAS::OperatingPointExcitation& exc) {
    auto cur = exc.get_current();
    if (!cur) return 0;
    auto wf = cur->get_waveform();
    if (!wf) return 0;
    return wf->get_data().size();
}

void check_ngspice(const json& tas) {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — NGSPICE extract skipped");
        return;
    }
    MAS::OperatingPoint op = Kirchhoff::extract_operating_point(tas, Kirchhoff::ExtractEngine::NGSPICE);
    const auto& excs = op.get_excitations_per_winding();
    REQUIRE(excs.size() >= 1);
    // Non-vacuousness is guaranteed by the extract itself: it THROWS if any winding fails to match a
    // simulated inductor branch, so a successful return means every winding was rebuilt from the sim.
    // Each therefore carries a real 128-sample simulated current waveform.
    for (size_t w = 0; w < excs.size(); ++w) {
        CHECK(current_rms(excs[w]) > 0.0);
        CHECK(current_waveform_points(excs[w]) == 128);   // sim populated the waveform
    }
}

}  // namespace

TEST_CASE("extract: LLC (transformer + resonant inductor)", "[extract][llc]") {
    json tas = Kirchhoff::build_llc_tas(Kirchhoff::design_llc(spec_for(400, 12, 120, 100000)));
    auto mags = Kirchhoff::topology_waveforms(tas);
    // LLC carries at least the main transformer; resonant Lr may be a separate magnetic.
    REQUIRE(mags.size() >= 1);
    check_topology_waveforms(tas, mags.size());
    check_analytical(tas);
    check_ngspice(tas);
}

TEST_CASE("extract: flyback (single coupled inductor)", "[extract][flyback]") {
    json tas = Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(spec_for(48, 12, 30, 100000)));
    auto mags = Kirchhoff::topology_waveforms(tas);
    REQUIRE(mags.size() >= 1);
    check_topology_waveforms(tas, mags.size());
    check_analytical(tas);
    check_ngspice(tas);
}

TEST_CASE("extract: buck (non-isolated inductor)", "[extract][buck]") {
    json tas = Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(24, 12, 60, 100000)));
    auto mags = Kirchhoff::topology_waveforms(tas);
    REQUIRE(mags.size() >= 1);
    check_topology_waveforms(tas, mags.size());
    check_analytical(tas);
    check_ngspice(tas);
}

TEST_CASE("extract: bad TAS throws (no silent fallback)", "[extract][errors]") {
    json empty = json::object();
    CHECK_THROWS(Kirchhoff::topology_waveforms(empty));
    CHECK_THROWS(Kirchhoff::extract_operating_point(empty, Kirchhoff::ExtractEngine::ANALYTICAL));
    CHECK_THROWS(Kirchhoff::main_magnetic_inputs(empty));
    // named magnetic that doesn't exist
    json tas = Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(24, 12, 60, 100000)));
    CHECK_THROWS(Kirchhoff::extract_operating_point(tas, Kirchhoff::ExtractEngine::ANALYTICAL, "no_such_magnetic"));
}

TEST_CASE("api: full analytical waveforms captured out-of-band", "[extract][api][waveforms]") {
    // The builders strip waveforms when baking the TAS (excitations_processed); the named variant
    // captures the FULL operating point per magnetic, exposed via api::design_tas_full /
    // api::process_converter. LLC is the acid test: its labels are `custom`, so WITHOUT this capture
    // no time-domain data exists anywhere for it analytically.
    const std::string spec = spec_for(400, 12, 120, 100000).dump();
    const std::string out = Kirchhoff::api::design_tas_full("llc", spec);
    REQUIRE(out.rfind("Exception:", 0) != 0);
    const json j = json::parse(out);
    REQUIRE(j.contains("tas"));
    REQUIRE(j.contains("analyticalWaveforms"));
    REQUIRE(j.at("analyticalWaveforms").contains("T1"));

    const json& op = j.at("analyticalWaveforms").at("T1");
    const auto& excs = op.at("excitationsPerWinding");
    REQUIRE(excs.size() == 3);   // LLC: primary + 2 center-tapped secondary halves
    for (const auto& e : excs) {
        // full waveform arrays present — the whole point of the capture
        REQUIRE(e.at("current").contains("waveform"));
        const auto& wf = e.at("current").at("waveform");
        REQUIRE(wf.at("data").size() >= 3);
        REQUIRE(wf.at("data").size() == wf.at("time").size());
    }

    // the TAS itself stays stripped (minimal, schema-valid): no waveform key inside baked excitations
    // (tas.simulation.stimulus[].waveform — the PWM gate drive — is a different, legitimate key)
    std::function<void(const json&)> checkStripped = [&](const json& node) {
        if (node.is_object()) {
            if (node.contains("excitationsPerWinding"))
                for (const auto& e : node.at("excitationsPerWinding")) {
                    if (e.contains("current")) CHECK_FALSE(e.at("current").contains("waveform"));
                    if (e.contains("voltage")) CHECK_FALSE(e.at("voltage").contains("waveform"));
                }
            for (const auto& [k, v] : node.items()) checkStripped(v);
        } else if (node.is_array()) {
            for (const auto& v : node) checkStripped(v);
        }
    };
    checkStripped(j.at("tas"));

    // a second build must not leak the first build's captures (registry cleared per build)
    const json j2 = json::parse(Kirchhoff::api::design_tas_full("buck", spec_for(24, 12, 60, 100000).dump()));
    REQUIRE(j2.at("analyticalWaveforms").contains("L1"));
    CHECK_FALSE(j2.at("analyticalWaveforms").contains("T1"));
}

TEST_CASE("extract: main_magnetic_inputs = the adviser's MAS::Inputs", "[extract][legacy]") {
    json tas = Kirchhoff::build_llc_tas(Kirchhoff::design_llc(spec_for(400, 12, 120, 100000)));

    // main_magnetic_inputs = the transformer (3-winding) MAS::Inputs the adviser designs around.
    MAS::Inputs main = Kirchhoff::main_magnetic_inputs(tas);
    REQUIRE(main.get_operating_points().size() >= 1);
    CHECK(main.get_operating_points().at(0).get_excitations_per_winding().size() == 3);
    // In HS the non-main components (Lr, caps) are already in the TAS — reachable via topology_waveforms
    // and a direct cap walk — so there is no separate extra_components_inputs to test.
    auto mags = Kirchhoff::topology_waveforms(tas);
    bool lr = false;
    for (const auto& m : mags) if (m.name == "Lr") lr = true;
    CHECK(lr);
}
