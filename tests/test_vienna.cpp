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
#include "ConverterAnalytical.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

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
    // (b) bus matches the independent prediction. The closed-loop bus-voltage PI now REGULATES the bus to
    // the target (= the oracle's power-balance bus at unit efficiency), so this also confirms regulation.
    CHECK(std::abs(vbus - o.vbus) / vout < 0.05);
    // (c) drawn power matches the prediction, and energy balance Pin == Vbus^2/Rload holds
    CHECK(std::abs(pin - o.pin) / o.pin < 0.10);
    CHECK(std::abs(pin - vbus * vbus / rload) / pin < 0.10);
    // (d) balanced split about the grounded midpoint
    CHECK(std::abs(busP + busN) < 0.05 * vout);
    CHECK(busP > 0.30 * vout);
    // (e) PF vs the MODEL. The oracle's ripple ceiling (o.pf ≈ 0.993) bounds the raw measured PF from
    // ABOVE (a measurement can't beat the ripple-limited PF). The floor is 0.92: Vienna's m-band hysteresis
    // makes the effective current ripple larger and cycle-dependent (worse at low line / high boost ratio),
    // the raw cycle-RMS folds in unfiltered switching ripple, AND — now that the closed-loop bus PI
    // REGULATES the bus to the exact target (rather than the old open-loop float a few % ABOVE it) — the
    // boost margin (half-bus vs phase peak) is tighter, which slightly raises the current ripple and lowers
    // the measured PF by ~0.5 pp. All only LOWER the measured value below the ceiling; PF > 0.92 still
    // corresponds to a true displacement PF ~0.99 (the end-to-end DATASHEET run measures PF ≈ 0.976).
    CHECK(pf < o.pf + 0.02);
    CHECK(pf > 0.92);
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

// ─── analytical_vienna: interleaving (numberOfChannels) + peakOfLinePlusSectors sampling (ABT #93) ──────
// PURE-ANALYTICAL tests (no ngspice): the analytical solver's interleaving current-split and the six-sector
// DPWM sampling strategy. Design point: 230 V L-N → 800 V split bus, 10 kW, 50 Hz line, 70 kHz sw, L=500 µH.
namespace {
constexpr double kVph = 230.0, kVdc = 800.0, kPo = 10000.0, kFline = 50.0, kFsw = 70000.0, kL = 500e-6;
const double kVpk  = std::sqrt(2.0) * kVph;                 // 325.27 V
const double kMod  = kVpk / (kVdc / 2.0);                   // 0.8132
const double kIpk  = std::sqrt(2.0) * kPo / (3.0 * kVph);   // 20.50 A per-phase line-current peak
const double kDIpp = kVpk * (1.0 - kMod) / kFsw / kL;       // 1.736 A peak-of-line ripple

// processed-current accessor for a winding (get_current()/get_processed() return std::optional by value).
MAS::ProcessedWaveform vienna_processed_current(const MAS::OperatingPoint& op, size_t w) {
    return *op.get_excitations_per_winding().at(w).get_current()->get_processed();
}
}  // namespace

TEST_CASE("analytical_vienna interleaving: per-channel peak = single-channel peak / N_ch", "[vienna][analytical][interleave]") {
    using Kirchhoff::analytical::analytical_vienna;

    // Single-channel baseline (default N_ch = 1): three windings, full line-cycle envelope of peak ≈ I_pk.
    MAS::OperatingPoint op1 = analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL);
    REQUIRE(op1.get_excitations_per_winding().size() == 3);
    const double peakSingle = *vienna_processed_current(op1, 0).get_peak();
    CHECK(peakSingle == Catch::Approx(kIpk).margin(kDIpp));

    // N_ch = 2: each phase splits into TWO channel inductors (3·2 = 6 windings). Each channel carries half
    // the phase current → its envelope peak is HALF the single-channel peak (the switching ripple, set by L,
    // is not divided, hence the ΔI_pp margin).
    MAS::OperatingPoint op2 = analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL, 1.0, 1.0, true, 2);
    REQUIRE(op2.get_excitations_per_winding().size() == 6);
    const double peakCh = *vienna_processed_current(op2, 0).get_peak();
    CHECK(peakCh == Catch::Approx(peakSingle / 2.0).margin(kDIpp));

    // The exact DC/envelope split is cleaner in the peak-of-line snapshot: the triangular current is centred
    // on the per-channel average I_pk/N_ch (no ripple in the offset). N_ch = 4 → 12 windings, offset = I_pk/4.
    MAS::OperatingPoint op4 = analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL, 1.0, 1.0, /*fullLineCycle=*/false, 4);
    REQUIRE(op4.get_excitations_per_winding().size() == 12);
    CHECK(*vienna_processed_current(op4, 0).get_average() == Catch::Approx(kIpk / 4.0).margin(0.1));

    // numberOfChannels < 1 is rejected (no silent clamp to 1).
    CHECK_THROWS(analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL, 1.0, 1.0, false, 0));
    CHECK_THROWS(analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL, 1.0, 1.0, false, -2));
}

TEST_CASE("analytical_vienna peakOfLinePlusSectors: peak-of-line + six DPWM sector operating points", "[vienna][analytical][sectors]") {
    using Kirchhoff::analytical::analytical_vienna;

    // peakOfLinePlusSectors (fullLineCycle=false, plusSectors=true): per phase → 1 peak-of-line snapshot +
    // 6 sector snapshots = 7 windings; three phases → 21 windings total.
    MAS::OperatingPoint op = analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL,
                                               1.0, 1.0, /*fullLineCycle=*/false, /*numberOfChannels=*/1,
                                               /*peakOfLinePlusSectors=*/true);
    REQUIRE(op.get_excitations_per_winding().size() == 3 * 7);

    // Winding 0 is Phase A's peak-of-line snapshot: triangular current centred on the line-current peak I_pk.
    CHECK(*vienna_processed_current(op, 0).get_average() == Catch::Approx(kIpk).margin(0.2));

    // The six Phase-A sectors follow at windings 1..6, sampled at line angles 30°+k·60° (k=0..5). Their
    // per-angle average current is I_pk·sin(angle): {0.5, 1, 0.5, −0.5, −1, −0.5}·I_pk. Sector k=1 (90°)
    // coincides with the peak; k=4 (270°) is the negative peak.
    const double kSin[6] = {0.5, 1.0, 0.5, -0.5, -1.0, -0.5};
    for (int k = 0; k < 6; ++k) {
        CHECK(*vienna_processed_current(op, 1 + k).get_average()
              == Catch::Approx(kIpk * kSin[k]).margin(0.3));
    }

    // Interleaving composes with the sector sampling: N_ch = 2 → 3 phases × 2 channels × 7 = 42 windings,
    // and every sector current is halved.
    MAS::OperatingPoint opN = analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL,
                                                1.0, 1.0, false, /*numberOfChannels=*/2, true);
    REQUIRE(opN.get_excitations_per_winding().size() == 3 * 2 * 7);
    CHECK(*vienna_processed_current(opN, 0).get_average() == Catch::Approx(kIpk / 2.0).margin(0.2));

    // The default (fullLineCycle) path is unaffected by the plusSectors flag → still exactly three windings.
    MAS::OperatingPoint opFull = analytical_vienna(kVph, kVdc, kPo, kFline, kFsw, kL,
                                                   1.0, 1.0, /*fullLineCycle=*/true, 1, /*plusSectors=*/true);
    CHECK(opFull.get_excitations_per_winding().size() == 3);
}
