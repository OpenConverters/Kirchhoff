// Control-in-CIAS capability test (NOT an MKF-equivalence test — this is where Kirchhoff intentionally
// improves on MKF). Validates the end-to-end pipeline for a CLOSED-LOOP, current-aware controller
// expressed entirely in CIAS:
//
//   AAS comparator (CIAS analog atom) --> CIAS->ngspice behavioural source -->
//   a separate, swappable TAS "control" stage --> drives the SR switches of the power stage.
//
// The demonstrator is CLLLC with a diode-emulating synchronous rectifier: four comparators sense each
// SR switch's drain-source and gate it like its body diode. We assert (1) the converter settles to a
// stable, sensible output (symmetric CLLLC at resonance ~ unity-gain DC transformer), and (2) the SR
// gates are ACTIVELY switching (the comparators are doing real work, not idle) — which is the actual
// thing under test: control/analog living in CIAS and driving the power stage from a control stage.
//
// Run directly:  ./build/test_control_stage

#include "Clllc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <regex>
#include <string>

using nlohmann::json;

namespace {
std::string fmt(double v) { std::ostringstream os; os.precision(12); os << v; return os.str(); }

std::string run_ngspice(const std::string& deck, const std::string& tag) {
    const std::string path = "/tmp/kirchhoff_ctrl_" + tag + ".cir";
    { std::ofstream f(path); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    if (!p) throw std::runtime_error("failed to launch ngspice");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}
bool parse_meas(const std::string& out, const std::string& name, double& v) {
    std::smatch m; std::regex re(name + R"(\s*=\s*([-0-9.eE+]+))");
    if (!std::regex_search(out, m, re)) return false;
    v = std::stod(m[1].str()); return true;
}
json clllc_inputs() {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputVoltage"] = {{"nominal",400.0},{"minimum",380.0},{"maximum",420.0}};
    di["designRequirements"]["switchingFrequency"]["nominal"] = 350e3;
    json o; o["name"]="out"; o["voltage"]["nominal"]=400.0;
    di["designRequirements"]["outputs"] = json::array({o});
    json op; op["inputVoltage"]=400.0; json oo; oo["power"]=400.0*16.5; op["outputs"]=json::array({oo});
    di["operatingPoints"] = json::array({op});
    return di;
}
}  // namespace

TEST_CASE("CLLLC closed-loop SR: control stage expressed in CIAS drives the rectifier",
          "[control][cias][clllc]") {
    json di = clllc_inputs();
    Kirchhoff::ClllcDesign d = Kirchhoff::design_clllc(di);
    json tas = Kirchhoff::build_clllc_tas(d);

    // The TAS carries TWO stages — a power switchingCell and a separate "control" stage — and the
    // control stage's brick contains AAS comparators (the control/analog block lives in CIAS).
    bool hasControlStage = false, hasComparator = false;
    for (const auto& st : tas.at("topology").at("stages")) {
        if (st.value("role", "") == "control") {
            hasControlStage = true;
            for (const auto& c : st.at("circuit").at("components"))
                if (c.at("data").contains("analog") && c.at("data").at("analog").contains("comparator"))
                    hasComparator = true;
        }
    }
    CHECK(hasControlStage);
    CHECK(hasComparator);

    // Assemble -> deck. The CIAS->ngspice converter must realise the comparator (it threw on `analog`
    // before this work); the deck carries .ic + uic for the resonant start.
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(deck.find("BCmpE") != std::string::npos);   // a comparator behavioural source got emitted
    REQUIRE(deck.find(" uic") != std::string::npos);     // initial-condition transient

    // Run to steady state (the 100µF LV bus + resonant loop; output precharged so it settles quickly).
    const double fsw = 350e3, period = 1.0 / fsw, tstep = period / 200.0, tstop = 12e-3;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(tstop) + " 0 " + fmt(tstep));
    // keep the trailing " uic" that the regex leaves in place
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\n"
            "meas tran vo AVG v(Vout) from=" + fmt(tstop - period) + " to=" + fmt(tstop) + "\n"
            "meas tran ge_avg AVG v(driveE) from=" + fmt(tstop - period) + " to=" + fmt(tstop) + "\n"
            "meas tran ge_hi MAX v(driveE) from=" + fmt(tstop - 2*period) + " to=" + fmt(tstop) + "\n"
            "print vo ge_avg ge_hi\n.endc\n.end\n";

    std::string out = run_ngspice(deck, "clllc");
    double vo = 0, geAvg = 0, geHi = 0;
    REQUIRE(parse_meas(out, "vo", vo));
    parse_meas(out, "ge_avg", geAvg);
    parse_meas(out, "ge_hi", geHi);

    INFO("CLLLC closed-loop SR: Vout=" << vo << " V, SR gate avg=" << geAvg << " V, gate max=" << geHi);
    // (1) settles to a stable, sensible output — symmetric CLLLC at resonance is ~unity-gain DC
    //     transformer (Vout ~ Vin = 400 V). Wide band: this is a physical-sanity gate, not an
    //     MKF match (we deliberately diverge from MKF's solver-timed-SR 187 V).
    CHECK(vo > 300.0);
    CHECK(vo < 460.0);
    // (2) the SR gate is genuinely being SWITCHED by the comparator (the point of the test): the gate
    //     reaches its high rail and spends a non-trivial fraction of the period both on and off.
    CHECK(geHi > 4.0);
    CHECK(geAvg > 0.5);
    CHECK(geAvg < 4.5);
}
