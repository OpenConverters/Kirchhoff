// PFC capability test (NOT an MKF-equivalence test — there is no MKF reference for PFC). Validates the
// first AC-input topology end to end: a single-phase CLOSED-LOOP boost PFC. The assembler emits a
// FLOATING mains SIN source (the acSinglePhase path); a diode bridge whose DC return is ground + a boost
// stage feed a HV DC bus; and a CURRENT-MODE control stage (expressed in CIAS — a hysteretic AAS
// comparator + an inductor-current sense + a |Vac|-shaped reference divider) gates the boost switch so
// the input current tracks the rectified line. We assert the real PFC result:
//   (1) the deck is genuinely AC-input (a floating sinusoidal source, no DC source), driven CLOSED-LOOP
//       (no open-loop PWM stimulus — the controller drives the switch), and
//   (2) running it converts AC -> a regulated, boosted DC bus at NEAR-UNITY power factor (the current
//       loop makes the boost emulate a resistor across the line).
//
// The current loop forces unity PF; the bus regulates to its target because the reference gain is matched
// to the design load (a fixed-gain power balance). An outer voltage loop (analog multiplier scaling the
// reference by a bus-error integrator) for active regulation against load steps is the next refinement.
// Runs at 400 Hz mains (real aircraft power, identical control mechanism) purely so a line cycle is
// 2.5 ms and the stiff-diode sim is fast; the demo uses a realistic 50 Hz line.
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

std::string run_ngspice(const std::string& deck) {
    const std::string path = "/tmp/kirchhoff_pfc_test.cir";
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

TEST_CASE("single-phase closed-loop boost PFC: AC input -> regulated bus at near-unity PF", "[pfc][ac]") {
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

    // (1) genuinely AC-input AND closed-loop: floating SIN source, a `control`-role stage, no DC source,
    //     and no open-loop PWM stimulus (the controller drives the switch).
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acSinglePhase");
    bool hasControl = false;
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.value("role", "") == "control") hasControl = true;
    CHECK(hasControl);
    CHECK(tas.at("simulation").contains("stimulus") == false);   // no open-loop gate drive
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(deck.find("Vac AcLine AcNeutral SIN(") != std::string::npos);
    REQUIRE(deck.find(" DC ") == std::string::npos);
    REQUIRE(deck.find("Vstim") == std::string::npos);

    // (2) run a few line cycles (bus precharged so it settles fast) and measure over one 400 Hz cycle.
    const double tstop = 7.5e-3, tstep = 1e-6, t0 = 5e-3, t1 = 7.5e-3;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(tstop) + " 0 " + fmt(tstep));
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\n"
            "let vac = v(AcLine)-v(AcNeutral)\n"
            "let iac = -i(Vac)\n"
            "let pinst = vac*iac\n"
            "meas tran pavg AVG pinst from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran vrms RMS vac from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran irms RMS iac from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "meas tran voavg AVG v(Vout) from=" + fmt(t0) + " to=" + fmt(t1) + "\n"
            "print pavg vrms irms voavg\n.endc\n.end\n";

    std::string out = run_ngspice(deck);
    double pavg = 0, vrms = 0, irms = 0, voavg = 0;
    REQUIRE(meas(out, "pavg", pavg));
    REQUIRE(meas(out, "vrms", vrms));
    REQUIRE(meas(out, "irms", irms));
    REQUIRE(meas(out, "voavg", voavg));
    const double vpeak = vrms * std::sqrt(2.0);
    const double pf = pavg / (vrms * irms);

    INFO("PFC: Vout=" << voavg << " V (Vpeak=" << vpeak << "), Vrms=" << vrms << " Irms=" << irms
         << " Pavg=" << pavg << " W, PF=" << pf);
    // regulated, boosted DC bus near the design target (400 V), well above the line peak.
    CHECK(voavg > 1.5 * vpeak);
    CHECK(voavg > 370.0);
    CHECK(voavg < 430.0);
    // the current loop draws ~the design power and emulates a resistor across the line: NEAR-UNITY PF.
    CHECK(pavg > 250.0);
    CHECK(pf > 0.95);
    CHECK(pf <= 1.01);
}
