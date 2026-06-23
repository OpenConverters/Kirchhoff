// PFC capability test (NOT an MKF-equivalence test — there is no MKF reference for PFC). Validates the
// first AC-input topology end to end: a single-phase CLOSED-LOOP boost PFC with BOTH loops.
//   • INNER current loop: a hysteretic AAS comparator + inductor-current sense + a |Vac|-shaped
//     reference make the input current track the rectified line → near-unity power factor.
//   • OUTER voltage loop: an AAS integrator drives the current-loop gain from the bus error (via an AAS
//     multiplier that forms vth = V(busP)·gv), so the bus is ACTIVELY regulated to its target.
// All of it is expressed in CIAS (comparator + multiplier + integrator analog blocks + resistors) as a
// swappable control stage — see [[control-in-cias]].
//
// We assert: (1) the deck is genuinely AC-input + closed-loop (floating SIN, no DC source, no PWM
// stimulus) and carries the voltage-loop blocks; (2) at the design load it regulates the bus to ~400 V
// at near-unity PF; and (3) ACTIVE regulation — with a deliberately MISMATCHED load (200 W into a system
// designed for 300 W) the bus stays near 400 V, whereas a fixed-gain (no voltage loop) controller would
// float to √(300·800) ≈ 490 V.
//
// Runs at 400 Hz mains (real aircraft power, identical control mechanism) so a line cycle is 2.5 ms and
// the stiff-diode sim is fast; the demo uses a realistic 50 Hz line.
//
// Run directly:  ./build/test_pfc
#include "Pfc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

using nlohmann::json;

namespace {
std::string fmt(double v) { std::ostringstream os; os.precision(12); os << v; return os.str(); }

std::string run_ngspice(const std::string& deck, const std::string& tag) {
    const std::string path = "/tmp/kirchhoff_pfc_" + tag + ".cir";
    { std::ofstream f(path); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    if (!p) throw std::runtime_error("failed to launch ngspice");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}
bool meas(const std::string& out, const std::string& name, double& v) {
    std::smatch m; std::regex re(name + R"(\s*=\s*([-0-9.eE+]+))");
    if (!std::regex_search(out, m, re)) return false;
    v = std::stod(m[1].str()); return true;
}
// Build a runnable deck (transient + a one-line-cycle measurement block), optionally overriding the load.
std::string make_deck(const std::string& base, double rloadOverride) {
    std::string deck = base;
    const double tstop = 12.5e-3, tstep = 1e-6, t0 = 10e-3, t1 = 12.5e-3;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(tstop) + " 0 " + fmt(tstep));
    if (rloadOverride > 0)
        deck = std::regex_replace(deck, std::regex(R"(Rload Vout 0 \S+)"),
                                  "Rload Vout 0 " + fmt(rloadOverride));
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\n"
            "let vac = v(AcLine)-v(AcNeutral)\nlet iac = -i(Vac)\nlet pinst = vac*iac\n"
            "meas tran pavg AVG pinst from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran vrms RMS vac from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran irms RMS iac from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran voavg AVG v(Vout) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "print pavg vrms irms voavg\n.endc\n.end\n";
    return deck;
}
}  // namespace

TEST_CASE("single-phase boost PFC: dual-loop (unity PF + ACTIVELY regulated bus)", "[pfc][ac]") {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;   // RMS (Vpeak ≈ 170)
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;  // aircraft mains (fast to simulate)
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=300.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(di);
    json tas = Kirchhoff::build_pfc_tas(d);

    // (1) AC-input, closed-loop, and carries the voltage-loop blocks (multiplier + integrator).
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acSinglePhase");
    CHECK(tas.at("simulation").contains("stimulus") == false);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(base.find("Vac AcLine AcNeutral SIN(") != std::string::npos);
    REQUIRE(base.find("Vstim") == std::string::npos);
    REQUIRE(base.find("BMv") != std::string::npos);     // multiplier (vth = busP·gv)
    REQUIRE(base.find("BIv_i") != std::string::npos);   // integrator (voltage-loop compensator)

    // (2) design load: regulated bus + near-unity PF.
    {
        std::string out = run_ngspice(make_deck(base, 0.0), "nom");
        double pavg=0, vrms=0, irms=0, voavg=0;
        REQUIRE(meas(out, "pavg", pavg)); REQUIRE(meas(out, "vrms", vrms));
        REQUIRE(meas(out, "irms", irms)); REQUIRE(meas(out, "voavg", voavg));
        const double pf = pavg / (vrms * irms);
        INFO("nominal: Vout=" << voavg << " PF=" << pf << " Pin=" << pavg);
        CHECK(voavg > 388.0);
        CHECK(voavg < 412.0);     // tightly regulated to the 400 V target
        CHECK(pf > 0.95);
        CHECK(pf <= 1.01);
    }
    // (3) ACTIVE regulation: a 200 W load (Rload=800) — designed for 300 W. The voltage loop holds the
    //     bus near 400 V; a fixed-gain controller would float to √(300·800) ≈ 490 V.
    {
        std::string out = run_ngspice(make_deck(base, 800.0), "mismatch");
        double voavg=0;
        REQUIRE(meas(out, "voavg", voavg));
        INFO("mismatched 200 W load: Vout=" << voavg << " V (fixed-gain would be ~490 V)");
        CHECK(voavg > 380.0);
        CHECK(voavg < 440.0);     // regulated near target, NOT floated up to ~490 V
    }
}
