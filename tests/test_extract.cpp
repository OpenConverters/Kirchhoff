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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
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
        // ABT #3: the winding VOLTAGE is now reconstructed from the simulated node difference too — the
        // extract throws if either terminal node is unresolvable, so a return means every winding carries a
        // real 128-sample simulated voltage (not the analytical one riding through).
        auto vol = excs[w].get_voltage();
        REQUIRE(vol);
        auto vwf = vol->get_waveform();
        REQUIRE(vwf);
        CHECK(vwf->get_data().size() == 128);
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
    // The named excitations_processed(op, component) variant captures the FULL operating point per
    // magnetic, exposed via api::design_tas_full / api::process_converter. LLC is the acid test: its
    // labels are `custom`, so WITHOUT this capture no time-domain data exists anywhere for it
    // analytically. (What the TAS itself embeds is asserted further down.)
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

    // The TAS's OWN baked excitations. This block used to assert they carried NO waveform ("the TAS
    // stays stripped"), which was true when this test was written (96482b9, 2026-07-02) but was
    // superseded three weeks later by 332619c: emit_processed now serializes the sampled waveform
    // whenever the solver produced one, because MKF reconstructs a STANDARD-label waveform from
    // `processed` alone but a CUSTOM one cannot be — calculate_advised_magnetics_fast rejects the seed
    // with "Waveform must have at least 2 data points". Every one of the 24 topologies emits samples
    // now, so the old assertion was simply stale (it only ever FAILED here because LLC is the topology
    // this test builds). What still matters, and is what this asserts, is that whatever the TAS carries
    // is USABLE by MKF: stats always present, and any embedded waveform a real ≥2-point signal with
    // matching data/time — never a stub. The FULL operating point (harmonics, conditions) still lives
    // out-of-band in analyticalWaveforms, which the block above proves.
    std::function<void(const json&)> checkUsable = [&](const json& node) {
        if (node.is_object()) {
            if (node.contains("excitationsPerWinding"))
                for (const auto& e : node.at("excitationsPerWinding"))
                    for (const char* side : {"current", "voltage"}) {
                        if (!e.contains(side)) continue;
                        CHECK(e.at(side).contains("processed"));
                        if (!e.at(side).contains("waveform")) continue;
                        const auto& wf = e.at(side).at("waveform");
                        CHECK(wf.at("data").size() >= 2);
                        if (wf.contains("time")) CHECK(wf.at("data").size() == wf.at("time").size());
                    }
            for (const auto& [k, v] : node.items()) checkUsable(v);
        } else if (node.is_array()) {
            for (const auto& v : node) checkUsable(v);
        }
    };
    checkUsable(j.at("tas"));

    // a second build must not leak the first build's captures (registry cleared per build)
    const json j2 = json::parse(Kirchhoff::api::design_tas_full("buck", spec_for(24, 12, 60, 100000).dump()));
    REQUIRE(j2.at("analyticalWaveforms").contains("L1"));
    CHECK_FALSE(j2.at("analyticalWaveforms").contains("T1"));
}

TEST_CASE("api: weinberg captures both magnetics into the registry", "[extract][api][waveforms][weinberg]") {
    // Regression (ABT #4): Weinberg used to bake its TAS via the UNNAMED excitations_processed overload,
    // so it registered NOTHING — the only topology absent from analyticalWaveforms, forcing the wizard onto
    // its synthesis fallback. The 6-winding solver op is now sliced into a 2-winding L1 op + 4-winding T1
    // op, each captured under its TAS component name with full waveforms + harmonics.
    const std::string out = Kirchhoff::api::design_tas_full("weinberg", spec_for(48, 12, 60, 100000).dump());
    REQUIRE(out.rfind("Exception:", 0) != 0);
    const json j = json::parse(out);
    REQUIRE(j.at("analyticalWaveforms").contains("L1"));
    REQUIRE(j.at("analyticalWaveforms").contains("T1"));
    CHECK(j.at("analyticalWaveforms").at("L1").at("excitationsPerWinding").size() == 2);
    CHECK(j.at("analyticalWaveforms").at("T1").at("excitationsPerWinding").size() == 4);
    for (const char* mag : {"L1", "T1"}) {
        const json& op = j.at("analyticalWaveforms").at(mag);
        for (const auto& e : op.at("excitationsPerWinding")) {
            REQUIRE(e.at("current").contains("waveform"));    // full waveform arrays (the point of the capture)
            REQUIRE(e.at("current").contains("harmonics"));   // and harmonics, like the other 23 topologies
            REQUIRE(e.at("voltage").contains("harmonics"));
        }
    }
}

TEST_CASE("api: captured operating points carry a sane ambient temperature", "[extract][api][waveforms]") {
    // Regression (ABT #5): the analytical solvers build only excitations, so the registry op's
    // conditions.ambientTemperature was uninitialized (denormal garbage). It is now stamped at capture time.
    for (const char* topo : {"buck", "llc", "weinberg"}) {
        const json j = json::parse(Kirchhoff::api::design_tas_full(topo, spec_for(48, 12, 60, 100000).dump()));
        for (const auto& [name, op] : j.at("analyticalWaveforms").items()) {
            REQUIRE(op.contains("conditions"));
            REQUIRE(op.at("conditions").contains("ambientTemperature"));
            const double amb = op.at("conditions").at("ambientTemperature").get<double>();
            INFO(topo << " / " << name << " ambient=" << amb);
            CHECK(amb == Catch::Approx(25.0));   // the documented emitted default, not garbage
        }
    }
}

TEST_CASE("api: spec ambient temperature threads to registry + magnetic conditions", "[extract][api][ambient]") {
    // Broader ABT #5: a non-default operatingPoints[0].ambientTemperature must flow to BOTH the captured
    // registry ops (PyOM/HS) AND every magnetic's TAS operating-point conditions (what MKF's adviser designs
    // the core against) — not the hardcoded 25 C the builders stamp. Threaded centrally in build_tas_for.
    for (const char* topo : {"buck", "llc", "weinberg"}) {
        json spec = spec_for(48, 12, 60, 100000);
        spec["operatingPoints"][0]["ambientTemperature"] = 85.0;
        const json j = json::parse(Kirchhoff::api::process_converter(topo, spec.dump(), "analytical"));
        // registry
        for (const auto& [name, op] : j.at("analyticalWaveforms").items()) {
            INFO(topo << " registry/" << name);
            CHECK(op.at("conditions").at("ambientTemperature").get<double>() == Catch::Approx(85.0));
        }
        // every magnetic's TAS operating-point conditions
        size_t magsChecked = 0;
        for (const auto& st : j.at("tas").at("topology").at("stages")) {
            if (!st.contains("circuit") || !st.at("circuit").is_object()) continue;
            for (const auto& c : st.at("circuit").at("components")) {
                if (!c.contains("data") || !c.at("data").contains("magnetic")) continue;
                for (const auto& op : c.at("data").at("inputs").at("operatingPoints")) {
                    INFO(topo << " magnetic " << c.value("name", std::string{}));
                    CHECK(op.at("conditions").at("ambientTemperature").get<double>() == Catch::Approx(85.0));
                    ++magsChecked;
                }
            }
        }
        CHECK(magsChecked >= 1);
    }
}

TEST_CASE("api: ngspice extract reconstructs winding voltage across topologies", "[extract][api][ngspice][voltage]") {
    // ABT #3 de-risk: the winding-voltage reconstruction THROWS if a winding terminal node can't be
    // resolved to a sim vector, so it must be exercised broadly — not just on buck/llc/flyback — to prove
    // the pin convention (primary_/secondary<i>_{start,end}) holds for every topology whose main magnetic
    // the wizard extracts. Runs the real process_converter(ngspice) path per topology.
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — ngspice voltage extract skipped");
        return;
    }
    // (vin, vout): boost steps up; everyone else here steps down. Power 60 W, 100 kHz.
    // psfb converges now that its ideal deck opts into the rshunt DC-reference aid (its full-bridge secondary
    // floats when all four rectifier diodes are reverse-biased) — see build_psfb_tas / TasAssembler.
    const std::vector<std::pair<std::string, std::pair<double, double>>> topos = {
        {"buck", {48, 12}}, {"boost", {12, 48}}, {"fsbb", {24, 12}}, {"flyback", {48, 12}},
        {"forward", {48, 12}}, {"two_switch_forward", {48, 12}}, {"acf", {48, 12}},
        {"push_pull", {48, 12}}, {"ahb", {48, 12}}, {"psfb", {400, 24}}, {"pshb", {400, 24}},
        {"dab", {48, 12}}, {"llc", {400, 12}}, {"src", {400, 12}},
    };
    for (const auto& [topo, vv] : topos) {
        const std::string out = Kirchhoff::api::process_converter(
            topo, spec_for(vv.first, vv.second, 60, 100000).dump(), "ngspice");
        INFO("topology=" << topo << " out=" << out.substr(0, 240));
        REQUIRE(out.rfind("Exception:", 0) != 0);
        const json j = json::parse(out);
        const auto& excs = j.at("operatingPoint").at("excitationsPerWinding");
        REQUIRE(excs.size() >= 1);
        for (const auto& e : excs) {
            REQUIRE(e.at("current").at("waveform").at("data").size() == 128);
            REQUIRE(e.at("voltage").at("waveform").at("data").size() == 128);   // simulated V, not analytical
        }
    }
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



TEST_CASE("the deck's transient never stops inside a gate edge", "[extract][tran][isolatedbuck]") {
    // The isolated-buck wizard default asks for 52 switching periods at 750 kHz. That stop time falls on
    // the rising gate edge at every period start, and the browser's ngspice (45.2 WASM) never returned on
    // it. The assembler moves the stop 1 ns past the transition; the averaging window follows it.
    const double fsw = 750000.0, period = 1.0 / fsw, requested = 52.0 / fsw;
    json spec = json::parse(R"({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":36,"maximum":72},
        "switchingFrequency":{"nominal":750000},"outputs":[{"name":"out","voltage":{"nominal":10},"regulation":"voltage"},
        {"name":"out2","voltage":{"nominal":10},"regulation":"voltage"}],"efficiency":0.9},
        "operatingPoints":[{"name":"full_load","inputVoltage":54,"ambientTemperature":25,
        "outputs":[{"name":"out","power":0.2},{"name":"out2","power":1}]}],"config":{"rippleRatio":0.4}})");
    spec["config"]["tranStopTime"] = requested;
    const std::string tas = Kirchhoff::api::design_tas("isolated_buck", spec.dump());
    REQUIRE(tas.rfind("Exception", 0) != 0);
    const std::string deck = Kirchhoff::api::generate_ngspice_circuit(tas, R"({"origin":"REQUIREMENTS"})");
    INFO(deck);
    const auto tranPos = deck.find("\n.tran ");
    REQUIRE(tranPos != std::string::npos);
    double tstep = 0, tstop = 0;
    REQUIRE(std::sscanf(deck.c_str() + tranPos + 7, "%lf %lf", &tstep, &tstop) == 2);
    // Moved just past the edge at the requested time: [requested + 1 ns, requested + 2 ns].
    CHECK(tstop > requested + 1e-9 - 1e-15);
    CHECK(tstop < requested + 2e-9 + 1e-15);
    CHECK(std::fmod(tstop, period) > 1e-9);
    // The averaging window ends at the same, moved stop time (same printed token as the .tran stop).
    std::istringstream tranLine(deck.substr(tranPos + 7));
    std::string stepToken, stopToken;
    tranLine >> stepToken >> stopToken;
    CHECK(deck.find("to=" + stopToken + "\n") != std::string::npos);
    // A stop time clear of every edge is left exactly as requested.
    spec["config"]["tranStopTime"] = 52.5 / fsw;
    const std::string deck2 = Kirchhoff::api::generate_ngspice_circuit(
        Kirchhoff::api::design_tas("isolated_buck", spec.dump()), R"({"origin":"REQUIREMENTS"})");
    const auto p2 = deck2.find("\n.tran ");
    REQUIRE(std::sscanf(deck2.c_str() + p2 + 7, "%lf %lf", &tstep, &tstop) == 2);
    CHECK(tstop == Catch::Approx(52.5 / fsw).epsilon(1e-9));
}


TEST_CASE("initial conditions reach a net inside a stage's subcircuit", "[extract][initial-conditions]") {
    // The single-secondary isolated buck's isolated rail is internal to the flybuck cell; its precharge
    // is declared as "<stage>.<net>" and must land on X<stage>.<net> in the deck.
    const std::string tas = Kirchhoff::api::design_tas("isolated_buck",
        R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":36,"maximum":72},"switchingFrequency":{"nominal":750000},"outputs":[{"name":"out","voltage":{"nominal":10},"regulation":"voltage"},{"name":"out2","voltage":{"nominal":10},"regulation":"voltage"}],"efficiency":0.9},"operatingPoints":[{"name":"full_load","inputVoltage":54,"ambientTemperature":25,"outputs":[{"name":"out","power":0.2},{"name":"out2","power":1}]}],"config":{"rippleRatio":0.4,"tranStopTime":0.00006933333333333333}})KH");
    const std::string deck = Kirchhoff::api::generate_ngspice_circuit(tas, R"({"origin":"REQUIREMENTS"})");
    INFO(deck.substr(deck.find(".ic"), 200));
    CHECK(deck.find(".ic v(XflybuckCell.vout_sec)=10") != std::string::npos);
    CHECK(deck.find(".ic v(Vout)=10") != std::string::npos);
}


TEST_CASE("extract(NGSPICE): an AC-input operating point is read at the line peak, not the zero crossing",
          "[extract][ac-input]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — AC-input extract skipped");
        return;
    }
    // The web Vienna wizard's spec, forced to stop after exactly two 50 Hz cycles: phase A's zero crossing.
    // Reading the last switching period there returned a phase-A current of microamps against ~23 A
    // analytical (both are the peak-of-line operating point).
    const std::string spec = R"KH({"designRequirements":{"inputType":"acThreePhase","inputVoltage":{"minimum":207.84609690826528,"nominal":230.94010767585033,"maximum":254.03411844343535},"switchingFrequency":{"nominal":20000},"outputs":[{"name":"out","voltage":{"nominal":800},"regulation":"voltage"}],"efficiency":0.97,"lineFrequency":{"nominal":50}},"operatingPoints":[{"name":"full_load","inputVoltage":230.94010767585033,"ambientTemperature":40,"outputs":[{"name":"out","power":10000}]}],"config":{"rippleRatio":0.25,"samplingStrategy":"peakOfLineOnly","phaseCount":1,"tranStopTime":0.04}})KH";
    const std::string ana = Kirchhoff::api::process_converter("vienna", spec, "analytical");
    const std::string sim = Kirchhoff::api::process_converter("vienna", spec, "ngspice");
    INFO(sim.substr(0, 300));
    REQUIRE(ana.rfind("Exception:", 0) != 0);
    REQUIRE(sim.rfind("Exception:", 0) != 0);
    auto peak_of = [](const std::string& out) {
        const json j = json::parse(out);
        const json& op = j.contains("operatingPoint") ? j.at("operatingPoint") : j.at("inputs").at("operatingPoints").at(0);
        double m = 0.0;
        for (double v : op.at("excitationsPerWinding").at(0).at("current").at("waveform").at("data").get<std::vector<double>>())
            m = std::max(m, std::abs(v));
        return m;
    };
    const double analytical = peak_of(ana), simulated = peak_of(sim);
    INFO("analytical peak=" << analytical << " simulated peak=" << simulated);
    CHECK(simulated == Catch::Approx(analytical).epsilon(0.25));
}


// ─── Steady state before extraction ─────────────────────────────────────────────────────────────────────
// The web's Simulated button sends a short settle window (the wizard's "steady-state periods": 50 for the
// forward, 5 for Weinberg). From a cold start those decks were still in start-up when the run ended, and
// the extracted operating point carried several times the design currents (two-switch forward 7.1 A peak
// against 3.1 A; Weinberg 10.8 A against 5.2 A) — the advisers then found no core, silently. The
// extraction must now verify steady state, extending the run until it holds; these are the web wizards'
// own specs, verbatim.
namespace {
double peak_abs(const json& signal) {
    double m = 0.0;
    for (double v : signal.at("waveform").at("data").get<std::vector<double>>()) m = std::max(m, std::abs(v));
    return m;
}
double primary_peak(const std::string& topo, const std::string& spec, const std::string& engine) {
    const std::string out = Kirchhoff::api::process_converter(topo, spec, engine);
    INFO("topology=" << topo << " engine=" << engine << " out=" << out.substr(0, 300));
    REQUIRE(out.rfind("Exception:", 0) != 0);
    const json j = json::parse(out);
    const json& op = j.contains("operatingPoint") ? j.at("operatingPoint") : j.at("inputs").at("operatingPoints").at(0);
    return peak_abs(op.at("excitationsPerWinding").at(0).at("current"));
}
// rms / mean of one winding's current or voltage over the extracted period.
double rms_of(const json& signal) {
    const auto d = signal.at("waveform").at("data").get<std::vector<double>>();
    double a = 0.0; for (double v : d) a += v * v;
    return std::sqrt(a / d.size());
}
double mean_of(const json& signal) {
    const auto d = signal.at("waveform").at("data").get<std::vector<double>>();
    double a = 0.0; for (double v : d) a += v;
    return a / d.size();
}
// Correlation of two one-period signals sampled on their own grids (the analytical one is resampled onto the
// simulated one's time points).
double correlation(const json& sim, const json& ref) {
    const auto sd = sim.at("waveform").at("data").get<std::vector<double>>();
    const auto st = sim.at("waveform").at("time").get<std::vector<double>>();
    const auto rd = ref.at("waveform").at("data").get<std::vector<double>>();
    const auto rt = ref.at("waveform").at("time").get<std::vector<double>>();
    double dot = 0.0, ns = 0.0, nr = 0.0;
    for (size_t k = 0; k < sd.size(); ++k) {
        const size_t j = static_cast<size_t>(std::upper_bound(rt.begin(), rt.end(), st[k]) - rt.begin());
        const double rv = (j == 0) ? rd.front() : (j >= rt.size()) ? rd.back()
                        : rd[j - 1] + (st[k] - rt[j - 1]) / (rt[j] - rt[j - 1]) * (rd[j] - rd[j - 1]);
        dot += sd[k] * rv; ns += sd[k] * sd[k]; nr += rv * rv;
    }
    return dot / std::sqrt(ns * nr);
}
}  // namespace

TEST_CASE("extract(NGSPICE): a run that ends inside start-up is extended to steady state",
          "[extract][steady-state]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — steady-state extract skipped");
        return;
    }
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"two_switch_forward", R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":36,"maximum":72},"switchingFrequency":{"nominal":200000},"outputs":[{"name":"out","voltage":{"nominal":12},"regulation":"voltage"}],"efficiency":0.92},"operatingPoints":[{"name":"full_load","inputVoltage":54,"ambientTemperature":25,"outputs":[{"name":"out","power":36}]}],"config":{"rippleRatio":0.3,"tranStopTime":0.00026}})KH"},
        {"weinberg",           R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":20,"maximum":30},"switchingFrequency":{"nominal":200000},"outputs":[{"name":"out","voltage":{"nominal":48},"regulation":"voltage"}],"efficiency":0.9,"turnsRatios":[{"nominal":1},{"nominal":0.5}]},"operatingPoints":[{"name":"full_load","inputVoltage":25,"ambientTemperature":25,"outputs":[{"name":"out","power":96}]}],"config":{"rippleRatio":0.4,"tranStopTime":0.000035}})KH"},
    };
    for (const auto& [topo, spec] : cases) {
        const double analytical = primary_peak(topo, spec, "analytical");
        const double simulated  = primary_peak(topo, spec, "ngspice");
        INFO("topology=" << topo << " analytical peak=" << analytical << " simulated peak=" << simulated);
        CHECK(simulated == Catch::Approx(analytical).epsilon(0.15));
    }
}

TEST_CASE("extract(NGSPICE): a deck that never settles is an error, not an operating point",
          "[extract][steady-state]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — steady-state extract skipped");
        return;
    }
    // The web isolated-buck default WITHOUT its initial conditions: its output LC (240 uH, 100 uF,
    // Q ~ 300) rings for ~0.3 s from a cold start, far beyond any bounded run. It used to come back as a
    // 3.1 A primary peak against 0.14 A analytical; it must be refused.
    json tas = json::parse(Kirchhoff::api::design_tas("isolated_buck", R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":36,"maximum":72},"switchingFrequency":{"nominal":750000},"outputs":[{"name":"out","voltage":{"nominal":10},"regulation":"voltage"},{"name":"out2","voltage":{"nominal":10},"regulation":"voltage"}],"efficiency":0.9},"operatingPoints":[{"name":"full_load","inputVoltage":54,"ambientTemperature":25,"outputs":[{"name":"out","power":0.2},{"name":"out2","power":1}]}],"config":{"rippleRatio":0.4,"tranStopTime":0.00006933333333333333}})KH"));
    REQUIRE(tas.at("simulation").contains("initialConditions"));
    tas["simulation"].erase("initialConditions");
    const std::string out = Kirchhoff::api::extract_operating_point(tas.dump(), "ngspice", "");
    INFO(out.substr(0, 300));
    REQUIRE(out.rfind("Exception:", 0) == 0);
    CHECK(out.find("did not reach steady state") != std::string::npos);
}

TEST_CASE("extract(NGSPICE): the isolated buck reaches its own periodic steady state", "[extract][steady-state]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — steady-state extract skipped");
        return;
    }
    // The web isolated-buck default. Its output LC (240 uH, 100 uF, Q ~ 300) rings for ~0.3 s from any
    // start that is not the deck's exact equilibrium, and the lossless deck, driven at the loss-compensated
    // duty D = Vpri/(Vin*eta), settles at ~11.15 V rather than the 10 V design point. The extractor must
    // solve for the deck's periodic steady state (shooting on the declared initial conditions) and extract
    // that period: the secondary then carries exactly the isolated rail's load current on average (charge
    // balance holds only in steady state), and each winding correlates with its analytical current.
    const std::string spec = R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":36,"maximum":72},"switchingFrequency":{"nominal":750000},"outputs":[{"name":"out","voltage":{"nominal":10},"regulation":"voltage"},{"name":"out2","voltage":{"nominal":10},"regulation":"voltage"}],"efficiency":0.9},"operatingPoints":[{"name":"full_load","inputVoltage":54,"ambientTemperature":25,"outputs":[{"name":"out","power":0.2},{"name":"out2","power":1}]}],"config":{"rippleRatio":0.4,"tranStopTime":0.00006933333333333333}})KH";
    const std::string a1 = Kirchhoff::api::process_converter("isolated_buck", spec, "ngspice");
    INFO(a1.substr(0, 300));
    REQUIRE(a1.rfind("Exception:", 0) != 0);
    const json op = json::parse(a1).at("operatingPoint");
    const auto& excs = op.at("excitationsPerWinding");
    REQUIRE(excs.size() == 2);
    const double pk0 = peak_abs(excs.at(0).at("current")), pk1 = peak_abs(excs.at(1).at("current"));
    INFO("primary peak=" << pk0 << " secondary peak=" << pk1);
    // The start-up transient came back as a 3.1 A primary peak; the settled deck carries tenths of an amp.
    CHECK(pk0 < 0.3);
    CHECK(pk1 < 0.3);
    CHECK(pk1 > 0.05);   // the isolated rail is loaded (1 W at ~10 V)
    // Charge balance on the isolated rail: the diode's average current is the load's, V/R with R = 100 ohm.
    // MAS excitation convention: the secondary's voltage is in the dot reference and it delivers its power
    // in the off-time, while that voltage is NEGATIVE, so in the source convention its average current is
    // -I_out (as the analytical isolated buck's, [convention]).
    const json j = json::parse(a1);
    const double iSecMean = mean_of(excs.at(1).at("current"));
    INFO("secondary mean current=" << iSecMean);
    CHECK(iSecMean < -0.095);
    CHECK(iSecMean > -0.115);   // 1 W on a rail that the lossless deck puts at ~10.4 V: ~0.104 A
    const json& ana = j.at("analyticalWaveforms").at("T1").at("excitationsPerWinding");
    CHECK(correlation(excs.at(0).at("current"), ana.at(0).at("current")) > 0.8);
    CHECK(correlation(excs.at(1).at("current"), ana.at(1).at("current")) > 0.8);
}


TEST_CASE("extract(NGSPICE): the DAB reaches its own periodic steady state", "[extract][steady-state]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — steady-state extract skipped");
        return;
    }
    // The web DAB default. A DAB is a current source into its output capacitor (tau = R*C = 16 ms here) and
    // its magnetizing / Lr DC offsets decay over tens of ms; from a cold start the web's 52-period window
    // extracted a primary current of ~10 A at the cycle start. Precharged and refined, it settles on the
    // deck's own equilibrium (~393 V; the snubbers and dead times make it ~20 % below the lossless
    // analytical 4 A peak, reported separately), and a second extraction agrees with the first.
    const std::string spec = R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"nominal":400,"tolerance":0.1},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":400},"regulation":"voltage"}],"efficiency":0.97,"magnetizingInductance":{"nominal":0.001},"turnsRatios":[{"nominal":1}]},"operatingPoints":[{"name":"full_load","inputVoltage":400,"ambientTemperature":25,"outputs":[{"name":"out","power":1000}]}],"config":{"dabPhaseShiftDeg":30,"tranStopTime":0.00052}})KH";
    const double analytical = primary_peak("dab", spec, "analytical");
    const double s1 = primary_peak("dab", spec, "ngspice");
    INFO("analytical peak=" << analytical << " simulated peak=" << s1);
    CHECK(s1 > 0.6 * analytical);
    CHECK(s1 < 1.2 * analytical);
}



TEST_CASE("extract(NGSPICE): the AHB's simulated transformer matches its analytical one, winding by winding", "[extract][steady-state][ahb]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — AHB extract skipped");
        return;
    }
    // 400 V -> 12 V, 200 W, both forward rectifiers. Three defects made the simulated T1 disagree with the
    // analytical one: the deck wired T1's primary the other way round (+(Vin−Vcb) while Q1 conducts is the
    // analytical orientation; the deck gave −Vcb), the full-bridge variant needed a steady-state solve that
    // started the blocking cap from −V(cb_mid) because the uic deck never stated V(Vin), and a switching
    // commutation spike sampled onto one grid point read as a 41–100 A peak. Every winding's rms and peak
    // must now agree with the analytical transformer, and every simulated signal must correlate positively
    // with its analytical counterpart.
    for (const std::string rect : {"centerTapped", "fullBridge"}) {
        const std::string spec = R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"nominal":400,"tolerance":0.1},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":12},"regulation":"voltage"}],"efficiency":0.95},"operatingPoints":[{"name":"full_load","inputVoltage":400,"ambientTemperature":25,"outputs":[{"name":"out","power":200}]}],"config":{"rectifierType":")KH" + rect + R"KH("}})KH";
        const std::string out = Kirchhoff::api::process_converter("ahb", spec, "ngspice");
        INFO("rectifier=" << rect << " out=" << out.substr(0, 300));
        REQUIRE(out.rfind("Exception:", 0) != 0);
        const json j = json::parse(out);
        const json& sim = j.at("operatingPoint").at("excitationsPerWinding");
        const json& ana = j.at("analyticalWaveforms").at("T1").at("excitationsPerWinding");
        REQUIRE(sim.size() == ana.size());
        REQUIRE(sim.size() == (rect == "centerTapped" ? 3u : 2u));
        for (size_t w = 0; w < sim.size(); ++w) {
            for (const char* q : {"current", "voltage"}) {
                const double rs = rms_of(sim[w].at(q)), ra = rms_of(ana[w].at(q));
                const double ps = peak_abs(sim[w].at(q)), pa = peak_abs(ana[w].at(q));
                const double c = correlation(sim[w].at(q), ana[w].at(q));
                INFO("winding " << w << " " << q << ": rms sim=" << rs << " ana=" << ra << " peak sim=" << ps
                     << " ana=" << pa << " corr=" << c);
                CHECK(std::abs(rs - ra) <= 0.10 * ra);
                CHECK(std::abs(ps - pa) <= 0.10 * pa);
                CHECK(c > 0.9);
            }
        }
    }
}

TEST_CASE("TasAssembler: a uic deck states its DC source levels in .ic", "[initial-conditions]") {
    // Under uic ngspice computes no operating point and starts every capacitor from the .ic values of its two
    // nodes, counting an unstated node as 0 V — a DC source node included. The AHB's blocking cap runs from
    // Vin to cb_mid; without v(Vin) it started at −V(cb_mid) and the steady-state solve converged on a
    // spurious state.
    const std::string spec = R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"nominal":400,"tolerance":0.1},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":12},"regulation":"voltage"}],"efficiency":0.95},"operatingPoints":[{"name":"full_load","inputVoltage":400,"ambientTemperature":25,"outputs":[{"name":"out","power":200}]}],"config":{"rectifierType":"fullBridge"}})KH";
    const json tas = json::parse(Kirchhoff::api::process_converter("ahb", spec, "analytical")).at("tas");
    REQUIRE(tas.at("simulation").contains("initialConditions"));
    const std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    INFO(deck.substr(deck.find(".ic"), 300));
    CHECK(deck.find(".ic v(Vin)=400") != std::string::npos);
    CHECK(deck.find(".ic v(XahbCell.cb_mid)=") != std::string::npos);
    CHECK(deck.find(" uic") != std::string::npos);
}

// ─── Web wizard inputs that the steady-state work broke (2026-09-25) ─────────────────────────────────
// Each is the spec the web wizard sends (the PFC one without the phase count the wizard used to send on
// every variant; Kirchhoff rejects numberOfPhases outside interleavedBoost). Four-Switch Buck-Boost and PFC were rejected with "cannot orient
// the simulated voltage" (a zero-shift correlation of 0.18 / -0.05: a phase offset in one case, a whole-
// line-cycle reference in the other). CLLLC "I know the design" declares only a rail precharge; shooting it
// stranded the SR at t = 0 ("Timestep too small" natively, a run the browser engine never returned from).
TEST_CASE("extract(NGSPICE): the web wizards' simulated operating points extract", "[extract][web-wizards]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("libngspice not linked — web-wizard extract skipped");
        return;
    }
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"fsbb", R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":9,"maximum":18},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":12},"regulation":"voltage"}],"efficiency":0.92},"operatingPoints":[{"name":"full_load","inputVoltage":13.5,"ambientTemperature":25,"outputs":[{"name":"out","power":24}]}],"config":{"rippleRatio":0.4,"tranStopTime":0.00052}})KH"},
        {"pfc", R"KH({"designRequirements":{"inputType":"acSinglePhase","inputVoltage":{"minimum":85,"maximum":265},"switchingFrequency":{"nominal":65000},"outputs":[{"name":"out","voltage":{"nominal":400},"regulation":"voltage"}],"efficiency":0.95,"lineFrequency":{"nominal":50}},"operatingPoints":[{"name":"full_load","inputVoltage":175,"ambientTemperature":25,"outputs":[{"name":"out","power":300}]}],"config":{"rippleRatio":0.3,"topologyVariant":"boost"}})KH"},
        {"clllc", R"KH({"designRequirements":{"inputType":"dc","inputVoltage":{"minimum":380,"maximum":420},"switchingFrequency":{"nominal":120000},"outputs":[{"name":"out","voltage":{"nominal":48},"regulation":"voltage"}],"efficiency":0.95,"magnetizingInductance":{"nominal":0.0005},"turnsRatios":[{"nominal":8}]},"operatingPoints":[{"name":"full_load","inputVoltage":400,"ambientTemperature":25,"outputs":[{"name":"out","power":240}]}],"config":{"resonantBandMin":90000,"resonantBandMax":150000,"tranStopTime":0.0004333333333333333}})KH"},
    };
    for (const auto& [topology, spec] : cases) {
        INFO(topology);
        const std::string out = Kirchhoff::api::process_converter(topology, spec, "ngspice");
        INFO(out.substr(0, 300));
        REQUIRE(out.rfind("Exception:", 0) != 0);
    }
}
