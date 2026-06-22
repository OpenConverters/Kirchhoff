// MKF_MODEL magnetic integration (Phase 4): a designed magnetic carries an MKF-exported ngspice
// subcircuit in magnetic.modelOutputs.spiceSubcircuit. The assembler must HOIST that .subckt to the
// deck top level (global, deduped) and the CIAS emitter must replace the ideal L+K with an X-instance
// that maps the winding terminals to the subckt's per-winding ports P<i>+/-.
//
// This test pins the integration logic with a hand-written subcircuit (no MKF link). The real
// MKF-designed-magnetic round-trip is exercised by tests/reference/{gen_real_magnetic,
// validate_real_magnetic}.cpp (which link MKF).

#include "Boost.hpp"
#include "Flyback.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

using nlohmann::json;

namespace {

std::string run_ngspice(const std::string& deck) {
    { std::ofstream f("/tmp/kf_real_magnetic.cir"); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen("ngspice -b /tmp/kf_real_magnetic.cir 2>&1", "r");
    if (!p) throw std::runtime_error("failed to launch ngspice");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}

// Attach a real-magnetic subcircuit to the first magnetic component found in the TAS.
bool bind_magnetic_subckt(json& node, const std::string& text, const std::string& ref) {
    if (node.is_object()) {
        if (node.contains("data") && node["data"].is_object() && node["data"].contains("magnetic")) {
            node["data"]["magnetic"]["modelOutputs"]["spiceSubcircuit"] = {{"text", text}, {"reference", ref}};
            return true;
        }
        for (auto& [k, v] : node.items()) if (bind_magnetic_subckt(v, text, ref)) return true;
    } else if (node.is_array()) {
        for (auto& e : node) if (bind_magnetic_subckt(e, text, ref)) return true;
    }
    return false;
}

json boost_inputs() {
    return json::parse(R"({
        "designRequirements": { "efficiency": 1.0,
            "inputVoltage": { "minimum": 11.4, "nominal": 12, "maximum": 12.6 },
            "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "out", "voltage": { "nominal": 24 }, "regulation": "voltage" } ] },
        "operatingPoints": [ { "name": "full", "inputVoltage": 12, "ambientTemperature": 25,
            "outputs": [ { "name": "out", "power": 24 } ] } ] })");
}

double settle_and_measure(const std::string& assembledDeck) {
    const double period = 1.0 / 100000.0, settle = 2400 * period, tstep = period / 200.0;
    std::ostringstream f; f.precision(12);
    auto F = [&](double v) { std::ostringstream o; o.precision(12); o << v; return o.str(); };
    std::string deck = std::regex_replace(assembledDeck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                          ".tran " + F(tstep) + " " + F(settle) + " 0 " + F(tstep));
    auto cpos = deck.rfind("\n.control"); if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\nmeas tran vout AVG v(Vout) from=" + F(settle - period) + " to=" + F(settle) +
            "\nprint vout\n.endc\n.end\n";
    std::string out = run_ngspice(deck);
    std::smatch m;
    if (!std::regex_search(out, m, std::regex(R"(vout\s*=\s*([-0-9.eE+]+))"))) {
        INFO("ngspice output:\n" << out);
        REQUIRE(false);
    }
    return std::stod(m[1].str());
}

} // namespace

TEST_CASE("Real magnetic: MKF_MODEL subcircuit is hoisted + instantiated and simulates", "[magnetic][real]") {
    json di = boost_inputs();
    Kirchhoff::BoostDesign d = Kirchhoff::design_boost(di);

    // Ideal baseline.
    double voutIdeal = settle_and_measure(
        Kirchhoff::tas_to_ngspice(Kirchhoff::build_boost_tas(d),
                                  PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS)));

    // Bind a hand-written real magnetic: winding Rdc + magnetizing L (single winding -> ports P1+ P1-).
    const std::string ref = "TESTMAG_PQ20";
    std::ostringstream sk;
    sk << ".subckt " << ref << " P1+ P1-\n"
       << "Rdc1 P1+ nmid 0.08\n"
       << "Lmag1 nmid P1- 150u\n"
       << ".ends " << ref << "\n";
    json tas = Kirchhoff::build_boost_tas(d);
    REQUIRE(bind_magnetic_subckt(tas, sk.str(), ref));

    std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::MKF_MODEL));

    // The subckt def is hoisted to the deck top level, and instantiated via an X line referencing it.
    CHECK(deck.find(".subckt " + ref) != std::string::npos);
    CHECK(std::regex_search(deck, std::regex(R"(\nX\S+[^\n]* )" + ref + R"(\n)")));
    // The ideal coupled-inductor card for the magnetic must NOT be emitted in the real path.
    CHECK(deck.find("_pri ") == std::string::npos);

    double voutReal = settle_and_measure(deck);
    INFO("Vout ideal=" << voutIdeal << "  real=" << voutReal);
    CHECK(voutReal > 0.8 * 24.0);        // boost still steps up and regulates near target
    CHECK(voutReal < voutIdeal + 1e-6);  // real magnetic (winding Rdc) is no less lossy than ideal
}

TEST_CASE("Real magnetic: a 2-winding (transformer) subcircuit wires P1+/-,P2+/- and simulates", "[magnetic][real]") {
    // Multi-winding integration: the flyback transformer (primary + secondary) must map to the
    // subckt's P1+/P1- (primary) and P2+/P2- (secondary) ports, in order.
    json di = json::parse(R"({
        "designRequirements": { "efficiency": 0.9, "inputType": "dc",
            "inputVoltage": { "minimum": 45.6, "nominal": 48, "maximum": 50.4 },
            "switchingFrequency": { "nominal": 100000 }, "isolationVoltage": 1500,
            "outputs": [ { "name": "out", "voltage": { "nominal": 12 }, "regulation": "voltage" } ] },
        "operatingPoints": [ { "name": "full", "inputVoltage": 48, "ambientTemperature": 25,
            "outputs": [ { "name": "out", "power": 24 } ] } ] })");
    Kirchhoff::FlybackDesign d = Kirchhoff::design_flyback(di);

    double voutIdeal = settle_and_measure(
        Kirchhoff::tas_to_ngspice(Kirchhoff::build_flyback_tas(d),
                                  PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS)));

    // Hand-written 2-winding transformer: Lp + Ls=Lp/n^2, each with a winding Rdc, well coupled.
    const std::string ref = "FLYMAG2";
    const double Lp = d.magnetizingInductance, Ls = Lp / (d.turnsRatio * d.turnsRatio);
    std::ostringstream sk;
    sk.precision(12);
    sk << ".subckt " << ref << " P1+ P1- P2+ P2-\n"
       << "Rdcp P1+ pmid 0.10\n"
       << "Lmp pmid P1- " << Lp << "\n"
       << "Rdcs P2+ smid 0.03\n"
       << "Lms smid P2- " << Ls << "\n"
       << "Kps Lmp Lms 0.999\n"
       << ".ends " << ref << "\n";
    json tas = Kirchhoff::build_flyback_tas(d);
    REQUIRE(bind_magnetic_subckt(tas, sk.str(), ref));

    std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::MKF_MODEL));
    CHECK(deck.find(".subckt " + ref) != std::string::npos);
    // X-instance with 4 winding nodes (primary +/-, secondary +/-) then the subckt reference.
    CHECK(std::regex_search(deck, std::regex(R"(\nX\S+ \S+ \S+ \S+ \S+ )" + ref + R"(\n)")));

    double voutReal = settle_and_measure(deck);
    INFO("flyback Vout ideal=" << voutIdeal << "  real=" << voutReal);
    CHECK(voutReal > 0.5 * voutIdeal);   // transformer still transfers energy to the output
    CHECK(voutReal < voutIdeal + 1e-6);  // real windings (Rdc + leakage) are no less lossy than ideal
}
