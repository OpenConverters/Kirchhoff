// Requirements-conformance suite for EVERY Kirchhoff topology (the regression gate).
//
// Stronger than the MKF-equivalence suite: that one only checks Kirchhoff and MKF AGREE; this one checks
// each converter actually DELIVERS ITS SPEC — the simulated steady-state output VOLTAGE and POWER match
// the design REQUIREMENTS (Vout, Pout from the operating point). Every CIAS / Kirchhoff change must keep
// all of these green. Each topology is designed from its requirements (the same fixture inputs the
// MKF-equivalence suite uses), assembled, simulated to steady state in ngspice, and the measured Vout /
// Pout are compared to the requirement.
//
// Scope = all 21 DC topologies. (MKF's "PtP" point-to-point reference designs are not replicated here —
// they pin to MKF-internal analytical models we don't carry; this suite pins to the user-facing
// requirements instead, which is the contract that matters.)
//
// Run directly:  ./build/test_requirements   (or a single one: ./build/test_requirements "[boost]")
#include "Boost.hpp"
#include "Buck.hpp"
#include "Flyback.hpp"
#include "Forward.hpp"
#include "TwoSwitchForward.hpp"
#include "Sepic.hpp"
#include "Cuk.hpp"
#include "Zeta.hpp"
#include "PushPull.hpp"
#include "Psfb.hpp"
#include "Ahb.hpp"
#include "Acf.hpp"
#include "Fsbb.hpp"
#include "Pshb.hpp"
#include "Dab.hpp"
#include "IsolatedBuck.hpp"
#include "IsolatedBuckBoost.hpp"
#include "Weinberg.hpp"
#include "Llc.hpp"
#include "Src.hpp"
#include "Cllc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using nlohmann::json;

#ifndef KIRCHHOFF_TEST_DIR
#define KIRCHHOFF_TEST_DIR "tests"
#endif

namespace {

// ── Harness (mirrors tests/test_mkf_equivalence.cpp: design → TAS → deck → settle → measure) ──
const std::string kRefDir = std::string(KIRCHHOFF_TEST_DIR) + "/reference";
constexpr double kReqTol = 0.05;    // 5 % on Vout
constexpr double kPowerTol = 0.10;  // ~2x on power (P proportional V^2, so a 5% Vout band IS a 10% power band)

std::string fmt(double v) { std::ostringstream os; os.precision(12); os << v; return os.str(); }

std::string run_ngspice(const std::string& deck, const std::string& tag) {
    const std::string path = "/tmp/kirchhoff_req_" + tag + ".cir";
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
json load_fixture(const std::string& topo) {
    std::ifstream f(kRefDir + "/" + topo + ".mkf.json");
    if (!f) throw std::runtime_error("missing fixture " + topo + ".mkf.json");
    json fx; f >> fx; return fx;
}
// Build the Kirchhoff design-input doc from the fixture requirements (η=1 ideal, ±5 % Vin).
json kirchhoff_inputs(const json& in) {
    json d;
    d["designRequirements"]["efficiency"] = 1.0;
    const double vin = in.at("inputVoltage").get<double>();
    d["designRequirements"]["inputVoltage"]["nominal"] = vin;
    d["designRequirements"]["inputVoltage"]["minimum"] = vin * 0.95;
    d["designRequirements"]["inputVoltage"]["maximum"] = vin * 1.05;
    d["designRequirements"]["switchingFrequency"]["nominal"] = in.at("switchingFrequency").get<double>();
    json o; o["name"] = "out"; o["voltage"]["nominal"] = in.at("outputVoltage").get<double>();
    d["designRequirements"]["outputs"] = json::array({o});
    json op; op["inputVoltage"] = vin;
    json oo; oo["power"] = in.at("outputPower").get<double>();
    op["outputs"] = json::array({oo});
    d["operatingPoints"] = json::array({op});
    d["simStimulusFsw"] = json::array({in.at("switchingFrequency").get<double>()});
    return d;
}

// Steady-state Vout from the Kirchhoff deck (≥30 output-RC time constants, 400-period floor).
double simulate_vout(const json& tasInputs, const json& tas, double loadResistance,
                     double settleCap, const std::string& tag) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    // Cuk's resonant coupling loop fails ideal-diode startup convergence ("timestep too small") at the
    // drop-compensated operating point; a tiny node-to-ground shunt cap fixes it (negligible vs the µF
    // power-stage caps). Cuk ONLY — a global cshunt would detune the resonant tanks (LLC/SRC).
    if (tag == "cuk")
        deck = std::regex_replace(deck, std::regex(R"((\.options [^\n]*method=gear))"), "$1 cshunt=1e-9");
    double fsw = 100000.0;
    for (const auto& st : tasInputs.value("simStimulusFsw", json::array())) fsw = st.get<double>();
    const double period = 1.0 / fsw;
    const double rc = loadResistance * settleCap;
    const double settleTime = std::max(400.0 * period, std::ceil(30.0 * rc / period) * period);
    const double tstep = period / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(settleTime) + " 0 " + fmt(tstep));
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\nmeas tran vout AVG v(Vout) from=" + fmt(settleTime - period) +
            " to=" + fmt(settleTime) + "\nprint vout\n.endc\n.end\n";
    std::string out = run_ngspice(deck, tag);
    double vout = 0;
    if (!parse_meas(out, "vout", vout)) throw std::runtime_error("could not parse Vout for " + tag);
    return vout;
}

// One topology: design-from-fixture builder -> {tas, loadResistance, settleCap, inputVoltage}.
struct Built { json tas; double loadR, settleCap, vin; };
template <class DesignFn, class BuildFn>
Built build_topology(DesignFn designFn, BuildFn buildFn, const json& di, double settleCapOverride = 0.0) {
    auto d = designFn(di);
    const double cap = settleCapOverride > 0.0 ? settleCapOverride : d.outputCapacitance;
    return {buildFn(d), d.loadResistance, cap, d.inputVoltage};
}

struct Topology { std::string name; std::function<Built(const json&)> build; };

const std::vector<Topology>& topologies() {
    static const std::vector<Topology> kTopos = {
        {"boost",   [](const json& di){ return build_topology(Kirchhoff::design_boost,   Kirchhoff::build_boost_tas,   di); }},
        {"buck",    [](const json& di){ return build_topology(Kirchhoff::design_buck,    Kirchhoff::build_buck_tas,    di); }},
        {"flyback", [](const json& di){ return build_topology(Kirchhoff::design_flyback, Kirchhoff::build_flyback_tas, di); }},
        {"forward", [](const json& di){ return build_topology(Kirchhoff::design_forward, Kirchhoff::build_forward_tas, di); }},
        {"two_switch_forward", [](const json& di){ return build_topology(Kirchhoff::design_two_switch_forward, Kirchhoff::build_two_switch_forward_tas, di); }},
        {"sepic",   [](const json& di){ return build_topology(Kirchhoff::design_sepic,   Kirchhoff::build_sepic_tas,   di); }},
        {"cuk",     [](const json& di){ return build_topology(Kirchhoff::design_cuk,     Kirchhoff::build_cuk_tas,     di); }},
        {"zeta",    [](const json& di){ return build_topology(Kirchhoff::design_zeta,    Kirchhoff::build_zeta_tas,    di); }},
        {"push_pull",[](const json& di){ return build_topology(Kirchhoff::design_push_pull, Kirchhoff::build_push_pull_tas, di); }},
        {"psfb",    [](const json& di){ return build_topology(Kirchhoff::design_psfb,    Kirchhoff::build_psfb_tas,    di); }},
        {"ahb",     [](const json& di){ return build_topology(Kirchhoff::design_ahb,     Kirchhoff::build_ahb_tas,     di); }},
        {"acf",     [](const json& di){ return build_topology(Kirchhoff::design_acf,     Kirchhoff::build_acf_tas,     di); }},
        {"fsbb",    [](const json& di){ return build_topology(Kirchhoff::design_fsbb,    Kirchhoff::build_fsbb_tas,    di); }},
        {"pshb",    [](const json& di){ return build_topology(Kirchhoff::design_pshb,    Kirchhoff::build_pshb_tas,    di); }},
        {"dab",     [](const json& di){ return build_topology(Kirchhoff::design_dab,     Kirchhoff::build_dab_tas,     di); }},
        {"isolated_buck",      [](const json& di){ return build_topology(Kirchhoff::design_isolated_buck,      Kirchhoff::build_isolated_buck_tas,      di); }},
        {"isolated_buck_boost",[](const json& di){ return build_topology(Kirchhoff::design_isolated_buck_boost,Kirchhoff::build_isolated_buck_boost_tas,di); }},
        {"weinberg",[](const json& di){ return build_topology(Kirchhoff::design_weinberg, Kirchhoff::build_weinberg_tas, di, 200e-6); }},  // slow current-fed loop
        {"llc",     [](const json& di){ return build_topology(Kirchhoff::design_llc,     Kirchhoff::build_llc_tas,     di); }},
        {"src",     [](const json& di){ return build_topology(Kirchhoff::design_src,     Kirchhoff::build_src_tas,     di); }},
        {"cllc",    [](const json& di){ return build_topology(Kirchhoff::design_cllc,    Kirchhoff::build_cllc_tas,    di); }},
    };
    return kTopos;
}

// Topologies whose open-loop output is NOT pinned to the requirement (so a spec gate cannot apply):
//  • llc — resonant gain set by fsw vs the tank, not a clean ratio;
//  • dab — output set by the bridge phase shift, not a ratio;
//  • cuk — the drop-compensated operating point is CORRECT but won't converge with ideal diodes in its
//    resonant coupling loop (the design is right; the ngspice sim isn't). Left at Vd=0 (matches MKF).
// These are REPORTED loudly (WARN), never silently dropped, but not asserted.
const std::set<std::string> kNotPinned = {};  // NO exclusions: every topology is gated to its spec
// Flybuck-family: the MEASURED output is the primary buck rail; the design needs a second (internal
// isolated secondary) output spec, taken from the fixture's design section.
const std::set<std::string> kDualOutput = {"isolated_buck", "isolated_buck_boost"};

void check_meets_requirements(const std::string& name) {
    json fx = load_fixture(name);
    const json& in = fx.at("inputs");
    const double vReq = in.at("outputVoltage").get<double>();
    const double pReq = in.at("outputPower").get<double>();

    json di = kirchhoff_inputs(in);
    if (kDualOutput.count(name)) {   // append the internal isolated-secondary output the design requires
        const json& des = fx.at("design");
        json os; os["name"] = "vsec"; os["voltage"]["nominal"] = des.at("secondaryVoltage").get<double>();
        di["designRequirements"]["outputs"].push_back(os);
        json osp; osp["power"] = des.at("secondaryPower").get<double>();
        di["operatingPoints"][0]["outputs"].push_back(osp);
    }
    Built b;
    for (const auto& t : topologies())
        if (t.name == name) { b = t.build(di); break; }

    if (kNotPinned.count(name)) {
        WARN(name << ": open-loop output is not requirement-pinned (resonant/phase) or non-convergent "
             "with ideal diodes (cuk) -- REPORTED, not gated (req " << vReq << " V).");
        return;
    }
    const double vout = std::fabs(simulate_vout(di, b.tas, b.loadR, b.settleCap, name));  // |.| : Cuk/Zeta invert
    const double pout = vout * vout / b.loadR;
    const double vErr = std::fabs(vout - vReq) / vReq, pErr = std::fabs(pout - pReq) / pReq;
    INFO(name << ": Vout=" << vout << " V (req " << vReq << ", err " << 100.0 * vErr << " %), "
         << "Pout=" << pout << " W (req " << pReq << ", err " << 100.0 * pErr << " %)");
    CHECK(vErr <= kReqTol);     // delivers the required output VOLTAGE
    CHECK(pErr <= kPowerTol);   // ...and the required POWER
}

}  // namespace

// One TEST_CASE per topology (each tagged) so a failure names the offending converter directly.
TEST_CASE("Boost meets requirements", "[requirements][boost]")                     { check_meets_requirements("boost"); }
TEST_CASE("Buck meets requirements", "[requirements][buck]")                       { check_meets_requirements("buck"); }
TEST_CASE("Flyback meets requirements", "[requirements][flyback]")                 { check_meets_requirements("flyback"); }
TEST_CASE("Forward meets requirements", "[requirements][forward]")                 { check_meets_requirements("forward"); }
TEST_CASE("Two-switch forward meets requirements", "[requirements][tsf]")          { check_meets_requirements("two_switch_forward"); }
TEST_CASE("SEPIC meets requirements", "[requirements][sepic]")                     { check_meets_requirements("sepic"); }
TEST_CASE("Cuk meets requirements", "[requirements][cuk]")                         { check_meets_requirements("cuk"); }
TEST_CASE("Zeta meets requirements", "[requirements][zeta]")                       { check_meets_requirements("zeta"); }
TEST_CASE("Push-pull meets requirements", "[requirements][pushpull]")              { check_meets_requirements("push_pull"); }
TEST_CASE("PSFB meets requirements", "[requirements][psfb]")                       { check_meets_requirements("psfb"); }
TEST_CASE("AHB meets requirements", "[requirements][ahb]")                         { check_meets_requirements("ahb"); }
TEST_CASE("ACF meets requirements", "[requirements][acf]")                         { check_meets_requirements("acf"); }
TEST_CASE("4SBB meets requirements", "[requirements][fsbb]")                       { check_meets_requirements("fsbb"); }
TEST_CASE("PSHB meets requirements", "[requirements][pshb]")                       { check_meets_requirements("pshb"); }
TEST_CASE("DAB meets requirements", "[requirements][dab]")                         { check_meets_requirements("dab"); }
TEST_CASE("IsolatedBuck meets requirements", "[requirements][isolated_buck]")      { check_meets_requirements("isolated_buck"); }
TEST_CASE("IsolatedBuckBoost meets requirements", "[requirements][isolated_buck_boost]") { check_meets_requirements("isolated_buck_boost"); }
TEST_CASE("Weinberg meets requirements", "[requirements][weinberg]")              { check_meets_requirements("weinberg"); }
TEST_CASE("LLC meets requirements", "[requirements][llc]")                         { check_meets_requirements("llc"); }
TEST_CASE("SRC meets requirements", "[requirements][src]")                         { check_meets_requirements("src"); }
TEST_CASE("CLLC meets requirements", "[requirements][cllc]")                       { check_meets_requirements("cllc"); }
