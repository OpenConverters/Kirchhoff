// Vienna capability test (NOT an MKF-equivalence test — no MKF reference). Validates the first
// THREE-phase AC-input topology: a CLOSED-LOOP Vienna 3-level rectifier. The assembler emits three
// sinusoidal sources 120° apart; the power cell (per phase: boost inductor + two rail diodes + a
// bidirectional switch to the split-bus midpoint) rectifies + boosts them to a split DC bus (±Vdc/2
// about the grounded midpoint = source neutral); and a per-phase CURRENT-SHAPING control stage (in CIAS:
// a summer + multiplier + comparator per phase) gates each switch on V(phase)·(iref − iL) > 0, which
// handles the Vienna polarity flip (the gating polarity follows the sign of the phase voltage), so each
// phase current tracks its phase voltage → near-unity 3-phase power factor.
//
// An ACTIVE balancing loop (an integrator on busP+busN injecting a common term into the phase
// references) keeps the two rail voltages equal under unbalanced half-loads.
//
// We assert: (1) genuinely 3-phase AC + closed-loop (three SIN sources, a control stage, no DC source,
// no PWM stimulus); (2) a BOOSTED (above the line-to-line peak), BALANCED (busP ≈ −busN) split DC bus at
// NEAR-UNITY 3-phase power factor; (3) ACTIVE balancing — under an asymmetric (top-rail-only) load the
// balancing loop holds busP ≈ −busN far tighter than with the loop disabled.
//
// Runs at 400 Hz mains (fast); the mechanism is identical at 50 Hz. No MKF ref.
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

TEST_CASE("three-phase Vienna rectifier: closed-loop -> boosted balanced bus at near-unity PF",
          "[vienna][ac][3ph]") {
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

    // (1) genuinely 3-phase AC + closed-loop.
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acThreePhase");
    CHECK(tas.at("simulation").contains("stimulus") == false);
    bool hasControl = false;
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.value("role", "") == "control") hasControl = true;
    CHECK(hasControl);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(deck.find("Vpha ") != std::string::npos);
    REQUIRE(deck.find(" -120)") != std::string::npos);
    REQUIRE(deck.find(" -240)") != std::string::npos);
    REQUIRE(deck.find("Vstim") == std::string::npos);

    // (2) run to ~steady state (bus precharged) and measure over one 400 Hz line cycle.
    const std::string base = deck;   // unmodified deck, reused by the balancing checks below
    const double tstop = 10e-3, tstep = 5e-7, t0 = 7.5e-3, t1 = 10e-3;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(tstop) + " 0 " + fmt(tstep));
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\n"
            "let vbus = v(busP)-v(busN)\n"
            "let pin = v(PhaseA)*(-i(Vpha)) + v(PhaseB)*(-i(Vphb)) + v(PhaseC)*(-i(Vphc))\n"
            "meas tran vbus_avg AVG vbus from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran busp_avg AVG v(busP) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran busn_avg AVG v(busN) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran pin_avg AVG pin from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran var RMS v(PhaseA) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran iar RMS i(Vpha) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "print vbus_avg busp_avg busn_avg pin_avg var iar\n.endc\n.end\n";

    std::string out = run_ngspice(deck);
    double vbus=0, busP=0, busN=0, pin=0, var=0, iar=0;
    REQUIRE(meas(out, "vbus_avg", vbus));
    REQUIRE(meas(out, "busp_avg", busP));
    REQUIRE(meas(out, "busn_avg", busN));
    REQUIRE(meas(out, "pin_avg", pin));
    REQUIRE(meas(out, "var", var));
    REQUIRE(meas(out, "iar", iar));
    const double vllPeak = 120.0 * std::sqrt(3.0) * std::sqrt(2.0);   // line-to-line peak ≈ 294 V
    const double pf = pin / (3.0 * var * iar);                        // balanced 3-phase power factor

    INFO("Vienna: bus=" << vbus << " (busP=" << busP << " busN=" << busN << "), Pin=" << pin
         << " W, Vph=" << var << " Iph=" << iar << ", PF=" << pf);
    // boosted bus, above the line-to-line peak
    CHECK(vbus > vllPeak);
    CHECK(vbus < 460.0);
    // balanced split about the grounded midpoint
    CHECK(std::abs(busP + busN) < 40.0);
    CHECK(busP > 150.0);
    // near-unity 3-phase power factor (current shaping working) + real power drawn
    CHECK(pin > 400.0);
    CHECK(pf > 0.95);
    CHECK(pf <= 1.02);

    // (3) ACTIVE balancing — add an asymmetric (top-rail-only) load and compare the rail imbalance
    // |busP+busN| WITH the balancing loop vs WITH it disabled (bal forced to 0). The loop should hold
    // the rails markedly tighter.
    auto rail_imbalance = [&](bool balancingOn) {
        std::string dk = base;
        dk = std::regex_replace(dk, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                ".tran " + fmt(tstep) + " 12e-3 0 " + fmt(tstep));
        auto cp = dk.rfind("\n.control");
        if (cp != std::string::npos) dk = dk.substr(0, cp);
        dk += "\nRasym busP 0 600\n";                                  // top-rail-only load
        if (!balancingOn)                                              // force the balancing term to 0
            dk = std::regex_replace(dk, std::regex(R"(BIbal bal 0 V=[^\n]*)"), "BIbal bal 0 V=0");
        dk += "\n.control\nrun\n"
              "meas tran bp AVG v(busP) from=9.5e-3 to=12e-3\n"
              "meas tran bn AVG v(busN) from=9.5e-3 to=12e-3\n"
              "print bp bn\n.endc\n.end\n";
        std::string o = run_ngspice(dk);
        double bp=0, bn=0;
        REQUIRE(meas(o, "bp", bp));
        REQUIRE(meas(o, "bn", bn));
        return std::abs(bp + bn);
    };
    const double imbOn  = rail_imbalance(true);
    const double imbOff = rail_imbalance(false);
    INFO("rail imbalance |busP+busN|: balancing ON=" << imbOn << " V, OFF=" << imbOff << " V");
    CHECK(imbOn < 3.0);             // tightly balanced with the loop
    CHECK(imbOn < 0.5 * imbOff);    // and markedly tighter than with the loop disabled
}
