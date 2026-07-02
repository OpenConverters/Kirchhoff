// Validate the in-process libngspice runner (src/NgspiceRunner.cpp) against the ngspice CLI: the SAME
// deck, run both ways, must measure the same output. This is the native half of MKF-migration item 4.
//
// Built/run only when Kirchhoff was configured with libngspice (-DENABLE_NGSPICE=ON, auto-on when the
// library is present). Without it the runner stub throws and this test is a no-op skip (surfaced, not
// silently passed).

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "NgspiceRunner.hpp"

#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

using nlohmann::json;

namespace {

// std::to_string uses %f (6 decimals) and collapses small values like 5e-8 to "0.000000" — useless for
// SPICE timesteps. Format with full precision instead.
std::string num(double v) { std::ostringstream o; o.precision(12); o << v; return o.str(); }

// A minimal, self-contained RC step deck with a known DC settling value — proves the runner loads,
// runs, and reads back a node vector independently of any topology. 10 V source, R=1k, C=1u to gnd;
// after ~10 RC (10 ms) v(out) -> 10 V.
std::string rc_deck() {
    return
        "Vsrc in 0 DC 10\n"
        "R1 in out 1k\n"
        "C1 out 0 1u\n"
        ".options reltol=1e-4\n"
        ".tran 1u 10m 0\n"
        ".end\n";
}

// Remove an existing `.control … .endc` block (and trailing `.end`) so we can append our own single
// measurement block — otherwise the deck would carry two control blocks.
std::string strip_control_and_end(std::string deck) {
    auto cpos = deck.find(".control");
    if (cpos != std::string::npos) {
        auto epos = deck.find(".endc", cpos);
        if (epos != std::string::npos) deck = deck.substr(0, cpos) + deck.substr(epos + 5);
    }
    auto endpos = deck.rfind("\n.end");
    if (endpos != std::string::npos) deck = deck.substr(0, endpos);
    return deck;
}

// Run a deck through the ngspice CLI, returning the AVG of v(node) over [from,to] via `meas`.
double cli_average(const std::string& circuit, const std::string& node, double from, double to) {
    std::string deck = strip_control_and_end(circuit);
    std::ostringstream m; m.precision(12);
    deck += "\n.control\nrun\nmeas tran val AVG v(" + node + ") from=" + num(from) +
            " to=" + num(to) + "\nprint val\n.endc\n.end\n";
    const std::string path = "/tmp/kirchhoff_runner_cli.cir";
    { std::ofstream f(path); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    REQUIRE(p != nullptr);
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    std::smatch sm;
    REQUIRE(std::regex_search(out, sm, std::regex(R"(val\s*=\s*([-0-9.eE+]+))")));
    return std::stod(sm[1].str());
}

} // namespace

TEST_CASE("in-process libngspice runner reads back a simple RC deck", "[ngspice][runner]") {
    if (!Kirchhoff::ngspice_in_process_available())
        SKIP("Kirchhoff built without libngspice (ENABLE_NGSPICE off) — in-process runner not exercised");
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(rc_deck());
    REQUIRE(r.success);
    REQUIRE_FALSE(r.time.empty());
    auto vout = r.average("v(out)", 9.5e-3, 10e-3);   // last 0.5 ms — settled
    REQUIRE(vout.has_value());
    CHECK(*vout == Catch::Approx(10.0).margin(0.05));   // RC charged to the 10 V rail
}

TEST_CASE("in-process runner matches the ngspice CLI on a converter deck", "[ngspice][runner]") {
    if (!Kirchhoff::ngspice_in_process_available())
        SKIP("Kirchhoff built without libngspice — in-process vs CLI equivalence not exercised");
    // Design a boost converter, emit its ideal deck, and run it both ways over the same window.
    json spec;
    spec["designRequirements"]["efficiency"] = 1.0;
    spec["designRequirements"]["inputVoltage"] = {{"minimum",10.8},{"nominal",12},{"maximum",13.2}};
    spec["designRequirements"]["switchingFrequency"]["nominal"] = 100000;
    spec["designRequirements"]["outputs"] = json::array({ {{"name","out"},{"voltage",{{"nominal",24}}}} });
    spec["operatingPoints"] = json::array({ {{"inputVoltage",12},{"outputs",json::array({{{"power",24}}})}} });

    Kirchhoff::BoostDesign d = Kirchhoff::design_boost(spec);
    json tas = Kirchhoff::build_boost_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);

    // Settle window: a few RC of the output filter, expressed in whole switching periods.
    const double period = 1.0 / 100000.0;
    const double rc = d.loadResistance * d.outputCapacitance;
    const double tstop = std::ceil(30.0 * rc / period) * period;
    const double tstep = period / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    const double from = tstop - period;

    double cli = cli_average(deck, "Vout", from, tstop);
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);
    auto inproc = r.average("v(Vout)", from, tstop);
    REQUIRE(inproc.has_value());
    INFO("boost Vout: in-process=" << *inproc << "  CLI=" << cli);
    // Same deck, same engine — they must agree closely (small numeric diff from meas-vs-trapezoid).
    CHECK(*inproc == Catch::Approx(cli).epsilon(0.02));
}
