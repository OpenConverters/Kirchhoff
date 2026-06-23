// PFC capability test (NOT an MKF-equivalence test — there is no MKF reference for PFC). Validates the
// FIRST AC-input topology end to end: a single-phase boost PFC front end. The assembler emits a
// FLOATING mains SIN source (the new acSinglePhase path); a diode bridge whose DC return is ground +
// a DCM boost pump |Vac| up to a boosted DC bus. We assert the solid, robust facts:
//   (1) the deck is genuinely AC-input (a sinusoidal source across two floating terminals, not DC), and
//   (2) running it converts AC -> a BOOSTED DC bus (Vout well above the line peak) while drawing real,
//       largely-in-phase power from the mains (the fundamental input current emulates a resistor — the
//       PFC mechanism).
//
// Notes on what is and isn't claimed: a fixed-duty DCM boost has INHERENT (open-loop) power-factor
// correction, but (a) its power factor is ratio-dependent and sub-unity, and (b) the *raw* line-current
// RMS here includes the 20 kHz DCM switching ripple (a real PFC's input EMI filter removes it), so the
// measured PF understates the true line-frequency PF. Unity PF + a regulated bus need an average-current
// PFC controller — a CTAS controller stage on top (control-in-CIAS), the documented next step. We run at
// 400 Hz mains (a real aircraft-power line) purely so a line cycle is 2.5 ms and the stiff-diode sim is
// fast; the DCM shaping mechanism is identical at 50/60 Hz (see pfc_demo, which uses 50 Hz).
//
// Run directly:  ./build/test_pfc
#include "Pfc.hpp"
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

TEST_CASE("single-phase boost PFC: floating AC input -> boosted DC bus, real-power draw", "[pfc][ac]") {
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

    // (1) genuinely AC-input: the TAS declares acSinglePhase + lineFrequency, and the deck carries a
    //     FLOATING sinusoidal source across two terminals (not a DC source to ground).
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acSinglePhase");
    CHECK(tas.at("inputs").at("designRequirements").contains("lineFrequency"));
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(deck.find("Vac AcLine AcNeutral SIN(") != std::string::npos);
    REQUIRE(deck.find(" DC ") == std::string::npos);     // no DC input source

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
         << " Pavg=" << pavg << " W, raw PF=" << pf);
    // AC -> a genuinely BOOSTED DC bus, well above the line peak (active boost, not mere rectification).
    CHECK(voavg > 1.5 * vpeak);
    CHECK(voavg < 520.0);
    // real power is drawn from the mains and delivered (a working converter, not a reactive/idle load).
    CHECK(pavg > 150.0);
    // the fundamental input current emulates a resistor (in phase): raw PF is sub-unity (switching ripple
    // is unfiltered here) but well above ~0 — the current is NOT purely reactive/random.
    CHECK(pf > 0.40);
    CHECK(pf <= 1.01);
}
