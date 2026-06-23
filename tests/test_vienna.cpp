// Vienna capability test (NOT an MKF-equivalence test — no MKF reference). Validates the first
// THREE-phase AC-input topology: a Vienna 3-level rectifier. The assembler emits three sinusoidal
// sources 120° apart; the power cell (per phase: boost inductor + two rail diodes + a bidirectional
// switch to the split-bus midpoint) rectifies + boosts them to a split DC bus (±Vdc/2 about the grounded
// midpoint = source neutral). We assert the SOLID, open-loop facts:
//   (1) genuinely 3-phase AC input — three SIN sources at 0/−120/−240°, no DC source;
//   (2) it converges to a BOOSTED bus (full bus above the line-to-line peak), BALANCED about the
//       grounded midpoint (busP ≈ −busN), drawing real power from all three phases.
//
// NOT claimed: unity power factor. Open-loop fixed-duty switching does not shape the phase currents
// (measured PF ≈ 0.84). The full closed-loop Vienna control — polarity-dependent per-phase current
// shaping (the gating polarity flips with the sign of the phase voltage) + midpoint balancing — is the
// documented refinement (it needs a difference/summer block in CIAS on top of the
// multiplier/integrator/comparator already there). See [[kirchhoff-ac-topologies]].
//
// Runs at 400 Hz mains (fast to simulate); the mechanism is identical at 50 Hz.
//
// Run directly:  ./build/test_vienna
#include "Vienna.hpp"
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

std::string run_ngspice(const std::string& deck) {
    const std::string path = "/tmp/kirchhoff_vienna_test.cir";
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
}  // namespace

TEST_CASE("three-phase Vienna rectifier: 3-phase AC input -> boosted, balanced split DC bus", "[vienna][ac][3ph]") {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acThreePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;   // per-phase (L-N) RMS, Vpeak ≈ 170
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=600.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::ViennaDesign d = Kirchhoff::design_vienna(di);
    json tas = Kirchhoff::build_vienna_tas(d);

    // (1) genuinely 3-phase AC input: three SIN sources at 0 / −120 / −240°, no DC source.
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acThreePhase");
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(deck.find("Vpha ") != std::string::npos);
    REQUIRE(deck.find("Vphb ") != std::string::npos);
    REQUIRE(deck.find("Vphc ") != std::string::npos);
    REQUIRE(deck.find(" -120)") != std::string::npos);
    REQUIRE(deck.find(" -240)") != std::string::npos);
    REQUIRE(deck.find(" DC ") == std::string::npos);

    // (2) run to ~steady state (bus precharged) and measure over one 400 Hz line cycle.
    const double tstop = 10e-3, tstep = 5e-7, t0 = 7.5e-3, t1 = 10e-3;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(tstop) + " 0 " + fmt(tstep));
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\n"
            "let vbus = v(busP)-v(busN)\n"
            "let pin = v(PhaseA)*(-i(Vpha)) + v(PhaseB)*(-i(Vphb)) + v(PhaseC)*(-i(Vphc))\n"
            "meas tran vbus_avg AVG vbus from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran vbus_min MIN vbus from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran busp_avg AVG v(busP) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran busn_avg AVG v(busN) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran pin_avg AVG pin from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "print vbus_avg vbus_min busp_avg busn_avg pin_avg\n.endc\n.end\n";

    std::string out = run_ngspice(deck);
    double vbus=0, vbusMin=0, busP=0, busN=0, pin=0;
    REQUIRE(meas(out, "vbus_avg", vbus));
    REQUIRE(meas(out, "vbus_min", vbusMin));
    REQUIRE(meas(out, "busp_avg", busP));
    REQUIRE(meas(out, "busn_avg", busN));
    REQUIRE(meas(out, "pin_avg", pin));

    const double vllPeak = 120.0 * std::sqrt(3.0) * std::sqrt(2.0);  // line-to-line peak ≈ 294 V
    INFO("Vienna: full bus=" << vbus << " V (min " << vbusMin << "), busP=" << busP << " busN=" << busN
         << ", Pin=" << pin << " W");
    // boosted: full bus above the line-to-line peak, and stable (not collapsing)
    CHECK(vbus > vllPeak);
    CHECK(vbus < 480.0);
    CHECK(vbusMin > 0.9 * vbus);             // low ripple / not collapsing
    // balanced split about the grounded midpoint: busP ≈ −busN
    CHECK(std::abs(busP + busN) < 40.0);
    CHECK(busP > 150.0);
    // real power drawn from the three phases
    CHECK(pin > 300.0);
}
