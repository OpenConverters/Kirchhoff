// Vienna capability test (NOT an MKF-equivalence test — no MKF reference). Validates the first
// THREE-phase AC-input topology: a CLOSED-LOOP Vienna 3-level rectifier. The assembler emits three
// sinusoidal sources 120° apart; the power cell (per phase: boost inductor + two rail diodes + a
// bidirectional switch to the split-bus midpoint) rectifies + boosts them to a split DC bus (±Vdc/2
// about the grounded midpoint = source neutral); a per-phase CURRENT-SHAPING control stage (in CIAS: a
// summer + multiplier + comparator per phase) gates each switch on V(phase)·(iref − iL) > 0, handling the
// Vienna polarity flip → near-unity 3-phase power factor. An ACTIVE balancing loop (an integrator on
// busP+busN injecting a common term into the phase references — with a DERIVED gain, see design_vienna)
// keeps the two rail voltages equal under unbalanced half-loads.
//
// Every operating point is cross-checked against an INDEPENDENT oracle (tests/averaged_model.py): a
// Python cycle-averaged model + a different solver. ngspice must match it (bus voltage, drawn power) and
// satisfy energy balance Pin == Vbus²/Rload — the AC analogue of grading the DC topologies vs MKF.
//
// We sweep LINE and LOAD (re-designing each corner) so "works" means "works across the envelope". Runs at
// 400 Hz mains (fast); the mechanism is identical at 50 Hz.
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

#ifndef KIRCHHOFF_TEST_DIR
#define KIRCHHOFF_TEST_DIR "tests"
#endif

namespace {
std::string fmt(double v) { std::ostringstream os; os.precision(12); os << v; return os.str(); }

std::string run_ngspice(const std::string& deck, const std::string& tag) {
    const std::string path = "/tmp/kirchhoff_vienna_" + tag + ".cir";
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

struct Oracle { double vbus, pin, pout, vllPeak, pf; };
Oracle oracle_vienna(const Kirchhoff::ViennaDesign& d, double rload, double tstop, double t0, double t1) {
    json p;
    p["inputVoltageRms"] = d.inputVoltageRms; p["outputVoltage"] = d.outputVoltage;
    p["busCapacitance"] = d.busCapacitance;   p["loadResistance"] = rload;
    p["outputPower"] = d.outputPower;
    p["rippleFraction"] = 0.30;   // matches the 0.3 inductor-ripple fraction in Vienna.cpp
    p["tstop"] = tstop; p["windowStart"] = t0; p["windowEnd"] = t1; p["precharge"] = d.outputVoltage;
    const std::string ppath = "/tmp/kirchhoff_vienna_oracle_in.json";
    { std::ofstream f(ppath); f << p.dump(); }
    const std::string cmd = "python3 " KIRCHHOFF_TEST_DIR "/averaged_model.py vienna < " + ppath + " 2>&1";
    std::string out; char buf[4096];
    FILE* fp = popen(cmd.c_str(), "r");
    if (!fp) throw std::runtime_error("failed to launch the averaged-model oracle");
    while (fgets(buf, sizeof(buf), fp)) out += buf;
    pclose(fp);
    json r = json::parse(out);
    return {r.at("vbus_pred").get<double>(), r.at("pin_pred").get<double>(),
            r.at("pout_pred").get<double>(), r.at("vll_peak").get<double>(), r.at("pf_pred").get<double>()};
}

// Append a measurement block (one 400 Hz line cycle) to a Vienna deck and set the transient.
std::string make_deck(const std::string& base, double tstop, double tstep, double t0, double t1) {
    std::string deck = std::regex_replace(base, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
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
    return deck;
}

struct Point { double vbus, busP, busN, pin, pf; };
Point run_vienna(double vrms, double vout, double fline, double pout, const std::string& tag) {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acThreePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = vrms;
    di["designRequirements"]["lineFrequency"]["nominal"] = fline;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=vout; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=vrms; json o; o["power"]=pout; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::ViennaDesign d = Kirchhoff::design_vienna(di);
    json tas = Kirchhoff::build_vienna_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);

    const double tstop = 10e-3, tstep = 5e-7, t0 = 7.5e-3, t1 = 10e-3;
    std::string out = run_ngspice(make_deck(base, tstop, tstep, t0, t1), tag);
    double vbus=0, busP=0, busN=0, pin=0, var=0, iar=0;
    REQUIRE(meas(out, "vbus_avg", vbus)); REQUIRE(meas(out, "busp_avg", busP));
    REQUIRE(meas(out, "busn_avg", busN)); REQUIRE(meas(out, "pin_avg", pin));
    REQUIRE(meas(out, "var", var));       REQUIRE(meas(out, "iar", iar));
    const double pf = pin / (3.0 * var * iar);
    const double rload = vout * vout / pout;

    Oracle o = oracle_vienna(d, rload, tstop, t0, t1);
    INFO("[" << tag << "] Vrms=" << vrms << " f=" << fline << " Pout=" << pout
         << "  ngspice: bus=" << vbus << " (busP=" << busP << " busN=" << busN << ") Pin=" << pin
         << " PF=" << pf << "  | oracle: bus=" << o.vbus << " Pin=" << o.pin << " vllPk=" << o.vllPeak);
    // (a) boosted above the line-to-line peak (the independent oracle supplies the peak)
    CHECK(vbus > o.vllPeak);
    // (b) bus matches the independent prediction (no voltage loop -> power-balance bus)
    CHECK(std::abs(vbus - o.vbus) / vout < 0.05);
    // (c) drawn power matches the prediction, and energy balance Pin == Vbus^2/Rload holds
    CHECK(std::abs(pin - o.pin) / o.pin < 0.10);
    CHECK(std::abs(pin - vbus * vbus / rload) / pin < 0.10);
    // (d) balanced split about the grounded midpoint
    CHECK(std::abs(busP + busN) < 0.05 * vout);
    CHECK(busP > 0.30 * vout);
    // (e) PF vs the MODEL. The oracle's ripple ceiling (o.pf ≈ 0.993) bounds the raw measured PF from
    // ABOVE (a measurement can't beat the ripple-limited PF). The floor stays at 0.93 because Vienna's
    // m-band hysteresis makes the effective current ripple larger and cycle-dependent (worse at low line /
    // high boost ratio), and the raw cycle-RMS also folds in unfiltered switching ripple — both only LOWER
    // the measured PF below the ceiling. PF > 0.93 still corresponds to a true displacement PF ~0.99.
    CHECK(pf < o.pf + 0.02);
    CHECK(pf > 0.93);
    return {vbus, busP, busN, pin, pf};
}
}  // namespace

TEST_CASE("three-phase Vienna rectifier: structural — 3-phase AC, closed loop, derived balancing", "[vienna][ac][3ph]") {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acThreePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=600.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::ViennaDesign d = Kirchhoff::design_vienna(di);
    json tas = Kirchhoff::build_vienna_tas(d);
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acThreePhase");
    CHECK(tas.at("simulation").contains("stimulus") == false);
    bool hasControl = false;
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.value("role", "") == "control") hasControl = true;
    CHECK(hasControl);
    CHECK(d.balanceGain > 0.0);        // derived, not the old hardcoded -2
    CHECK(d.balanceClamp > 0.0);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(deck.find("Vpha ") != std::string::npos);
    REQUIRE(deck.find(" -120)") != std::string::npos);
    REQUIRE(deck.find(" -240)") != std::string::npos);
    REQUIRE(deck.find("Vstim") == std::string::npos);
    REQUIRE(deck.find("BIbal_i") != std::string::npos);   // the balancing integrator
}

TEST_CASE("three-phase Vienna rectifier: line/load sweep vs the independent oracle", "[vienna][ac][3ph][sweep]") {
    // Re-design each corner (line into a 400 V bus kept above the line-to-line peak; a range of power) and
    // check the boosted, balanced, near-unity-PF bus matches the independent oracle + energy balance.
    run_vienna(100.0, 400.0, 400.0, 600.0, "line_100");
    run_vienna(120.0, 400.0, 400.0, 400.0, "load_400");
    run_vienna(120.0, 400.0, 400.0, 600.0, "nominal");
    run_vienna(120.0, 400.0, 400.0, 800.0, "load_800");
}

TEST_CASE("three-phase Vienna rectifier: active rail balancing under an asymmetric load", "[vienna][ac][3ph][balance]") {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acThreePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=600.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }
    Kirchhoff::ViennaDesign d = Kirchhoff::design_vienna(di);
    json tas = Kirchhoff::build_vienna_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);

    // Add an asymmetric (top-rail-only) load and compare the rail imbalance |busP+busN| WITH the balancing
    // loop vs WITH it disabled (bal forced to 0). The derived loop should hold the rails markedly tighter.
    const double tstep = 5e-7;
    auto rail_imbalance = [&](bool balancingOn) {
        std::string dk = std::regex_replace(base, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                            ".tran " + fmt(tstep) + " 12e-3 0 " + fmt(tstep));
        auto cp = dk.rfind("\n.control");
        if (cp != std::string::npos) dk = dk.substr(0, cp);
        dk += "\nRasym busP 0 600\n";
        if (!balancingOn)
            dk = std::regex_replace(dk, std::regex(R"(BIbal bal 0 V=[^\n]*)"), "BIbal bal 0 V=0");
        dk += "\n.control\nrun\n"
              "meas tran bp AVG v(busP) from=9.5e-3 to=12e-3\n"
              "meas tran bn AVG v(busN) from=9.5e-3 to=12e-3\n"
              "print bp bn\n.endc\n.end\n";
        std::string o = run_ngspice(dk, balancingOn ? "bal_on" : "bal_off");
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
