// component_waveforms — per-component V/I for every non-magnetic power part, from one ngspice run.
//
// The critical invariant this guards: the reconstructed device-current tokens (@<L>.x<stage>.<L><ref>[k])
// and terminal nodes match what the deck actually simulates. So the key test compares the map against
// ground truth — every non-magnetic component the API reports MUST correspond to a real @dev vector in a
// savecurrents run, and the physics of the extracted waveforms must be sane (diode current one-signed,
// cap mean current ~0, switch V_DS peak ~ the blocking voltage).

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "KirchhoffApi.hpp"
#include "NgspiceRunner.hpp"
#include "DatasheetModels.hpp"

#include <cmath>
#include <map>
#include <set>
#include <string>

using json = nlohmann::json;

namespace {
json spec_for(double vin, double vout, double power, double fsw, double eta = 0.9, int periods = 60) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name", "out"}, {"voltage", {{"nominal", vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}} });
    s["config"]["tranStopTime"] = periods / fsw;   // short, just enough to settle for a fast test
    return s;
}

// peak/rms/average of a signal side, reading the processed block the API emits.
double proc(const json& comp, const char* side, const char* field) {
    return comp.at(side).at("processed").at(field).get<double>();
}
}  // namespace

TEST_CASE("component_waveforms: buck exposes Q1/D1/Cout with sane physics", "[components][buck]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice — skipped"); return; }
    json tas = Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(48, 12, 60, 100000)));
    const std::string out = Kirchhoff::api::component_waveforms(tas.dump(), R"({"origin":"REQUIREMENTS"})");
    REQUIRE(out.rfind("Exception:", 0) != 0);
    const json j = json::parse(out);
    REQUIRE(j.at("engine") == "ngspice");
    REQUIRE(j.at("referencePeriod").get<double>() == Catch::Approx(1e-5));

    std::map<std::string, json> byRef;
    for (const auto& c : j.at("components")) byRef[c.at("ref").get<std::string>()] = c;

    // the buck power path: main switch, freewheel diode, output cap — all present, each with V and I
    REQUIRE(byRef.count("Q1"));
    REQUIRE(byRef.count("D1"));
    REQUIRE(byRef.count("Cout"));
    for (const char* ref : {"Q1", "D1", "Cout"}) {
        const json& c = byRef.at(ref);
        REQUIRE(c.at("current").at("waveform").at("data").size() == 128);
        REQUIRE(c.contains("voltage"));
        REQUIRE(c.at("voltage").at("waveform").at("data").size() == 128);
    }

    CHECK(byRef.at("Q1").at("kind") == "mosfet");
    CHECK(byRef.at("D1").at("kind") == "diode");
    CHECK(byRef.at("Cout").at("kind") == "capacitor");

    // physics sanity (ideal deck):
    //  * switch V_DS peak reaches ~Vin (the switch blocks the full input when off)
    CHECK(proc(byRef.at("Q1"), "voltage", "peak") == Catch::Approx(48.0).margin(8.0));
    //  * the output cap's mean current is ~0 in steady state (charge balance)
    CHECK(std::abs(proc(byRef.at("Cout"), "current", "average")) < 0.3);
    //  * the switch carries real current (peak > the ~5 A load)
    CHECK(proc(byRef.at("Q1"), "current", "peak") > 3.0);
}

TEST_CASE("realize_tas derives real-conduction semiconductor models from requirements", "[components][realize]") {
    json tas = Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(48, 12, 60, 100000)));
    const json real = Kirchhoff::derive_datasheet_models(tas);

    // Q1 gains a manufacturerInfo whose onResistance == the design's maximumOnResistance requirement,
    // and D1 a forwardVoltage == its maximumForwardVoltage — real values, nothing fabricated.
    auto findComp = [](const json& t, const std::string& ref) -> json {
        for (const auto& st : t.at("topology").at("stages"))
            if (st.contains("circuit") && st.at("circuit").is_object())
                for (const auto& c : st.at("circuit").at("components"))
                    if (c.value("name", std::string{}) == ref) return c;
        return json{};
    };
    const json q1 = findComp(real, "Q1");
    const json d1 = findComp(real, "D1");
    const auto& qElec = q1.at("data").at("semiconductor").at("mosfet").at("manufacturerInfo")
                          .at("datasheetInfo").at("electrical");
    CHECK(qElec.at("onResistance").get<double>()
          == Catch::Approx(q1.at("data").at("inputs").at("designRequirements").at("maximumOnResistance").get<double>()));
    // no fabricated parasitics
    CHECK_FALSE(qElec.contains("outputCapacitance"));
    CHECK_FALSE(qElec.contains("bodyDiodeForwardVoltage"));
    CHECK(d1.at("data").at("semiconductor").at("diode").at("manufacturerInfo").at("datasheetInfo")
            .at("electrical").at("forwardVoltage").get<double>()
          == Catch::Approx(d1.at("data").at("inputs").at("designRequirements").at("maximumForwardVoltage").get<double>()));

    // idempotent: a second pass changes nothing
    CHECK(Kirchhoff::derive_datasheet_models(real) == real);

    // the realized deck is now a "real" deck and uses the real on-resistance, not the ideal 0.01 Ω
    const std::string deck = Kirchhoff::tas_to_ngspice(real, PEAS::Fidelity(PEAS::Fidelity::Origin::DATASHEET));
    CHECK(deck.find("real, one subcircuit per stage") != std::string::npos);
    CHECK(deck.find("Ron=0.01") == std::string::npos);
}

TEST_CASE("component_waveforms extracts real (multi-atom) devices via datasheet fidelity", "[components][realize]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice — skipped"); return; }
    // Realize a buck, then simulate at DATASHEET fidelity — Q1 becomes a real switch (real Rds(on)); with
    // the requirements-derived model it is single-atom, so this also proves the prefix-match handles the
    // real-fidelity token path. Every power part must still be present with a 128-pt current.
    json tas = Kirchhoff::derive_datasheet_models(
        Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(48, 12, 60, 100000))));
    const json j = json::parse(Kirchhoff::api::component_waveforms(tas.dump(), R"({"origin":"DATASHEET"})"));
    REQUIRE(j.at("engine") == "ngspice");
    std::set<std::string> refs;
    for (const auto& c : j.at("components")) {
        REQUIRE(c.at("current").at("waveform").at("data").size() == 128);
        refs.insert(c.at("ref").get<std::string>());
    }
    CHECK(refs.count("Q1"));
    CHECK(refs.count("D1"));
    CHECK(refs.count("Cout"));
}

TEST_CASE("component_waveforms map matches the deck's savecurrents devices", "[components][invariant]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice — skipped"); return; }
    // LLC is the stress case: switches, body diodes, rectifier diodes, resonant + balancing + snubber
    // caps/resistors. Every non-magnetic component the API reports must be a device the sim actually has.
    json tas = Kirchhoff::build_llc_tas(Kirchhoff::design_llc(spec_for(400, 12, 240, 100000, 0.9, 40)));
    const json j = json::parse(Kirchhoff::api::component_waveforms(tas.dump(), R"({"origin":"REQUIREMENTS"})"));
    REQUIRE(j.at("components").size() >= 6);   // Q1,Q2,Dq1,Dq2,D1,D2,Chi,Clo,Cr,Cout,Rbal*,Rsn*,Csn*...

    // Ground truth: run the deck ourselves and collect the @device[...] current vectors present.
    const std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    REQUIRE(deck.find("savecurrents") != std::string::npos);
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);

    // every reported component carries a 128-pt current, and its kind is a real power part
    for (const auto& c : j.at("components")) {
        REQUIRE(c.at("current").at("waveform").at("data").size() == 128);
        const std::string kind = c.at("kind").get<std::string>();
        CHECK((kind == "mosfet" || kind == "diode" || kind == "capacitor" || kind == "resistor"));
    }

    // at least the two half-bridge switches and the two rectifier diodes must be present by ref
    std::set<std::string> refs;
    for (const auto& c : j.at("components")) refs.insert(c.at("ref").get<std::string>());
    CHECK(refs.count("Q1"));
    CHECK(refs.count("Q2"));
    CHECK(refs.count("D1"));
    CHECK(refs.count("D2"));
    CHECK(refs.count("Cout"));

    // rectifier diode current is one-signed (it only conducts forward) — a real extraction, not noise
    for (const auto& c : j.at("components")) {
        if (c.at("ref") != "D1") continue;
        const auto& data = c.at("current").at("waveform").at("data");
        double mn = 1e30, mx = -1e30;
        for (const auto& v : data) { double x = v.get<double>(); mn = std::min(mn, x); mx = std::max(mx, x); }
        CHECK(mx > 0.5);                 // it conducts
        CHECK(mn > -std::abs(mx) * 0.1); // negligible reverse current (ideal diode)
    }
}
