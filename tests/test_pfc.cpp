// PFC capability test (NOT an MKF-equivalence test — there is no MKF reference for PFC). Validates the
// first AC-input topology end to end: a single-phase CLOSED-LOOP boost PFC with a DESIGNED dual loop.
//   • INNER current loop: a hysteretic AAS comparator + inductor-current sense + a |Vac|-shaped
//     reference make the input current track the rectified line → near-unity power factor.
//   • OUTER voltage loop: a DESIGNED PI compensator (proportional path via an AAS summer, integral path
//     via an AAS integrator; gains derived from the bus-cap plant for a ripple-safe crossover and ~90°
//     phase margin — see design_pfc), so the bus is ACTIVELY regulated to its target.
// All of it is expressed in CIAS (comparator + multiplier + integrator + summer + resistors) as a
// swappable control stage — see [[control-in-cias]].
//
// Beyond the structural checks, every operating point is cross-checked against an INDEPENDENT oracle
// (tests/averaged_model.py): a Python cycle-averaged large-signal model that re-derives the same control
// law and integrates the plant with a different solver. ngspice's switched result must match it (bus
// voltage, drawn power, energy balance). This is the AC analogue of grading the DC topologies against
// MKF — the AC topologies finally have an external second source of truth instead of self-graded numbers.
//
// We sweep LINE, LOAD and FREQUENCY (not a single spot check) so "works" means "works across the
// envelope, stably". The bulk runs at 400 Hz (aircraft mains, fast to simulate); a tagged [slow] case
// verifies the realistic 50 Hz path actually runs and matches the oracle (it previously lived only in an
// un-run demo).
//
// Run directly:        ./build/test_pfc
// Skip the slow case:  ./build/test_pfc ~[slow]
#include "Pfc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

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

// INDEPENDENT oracle: run the Python averaged model with the design's derived gains + this run's plant
// (Rload) and measurement window, and return its prediction. Different language, different solver,
// different model class (cycle-averaged, not switched) → a genuine cross-check on the ngspice result.
struct Oracle { double vbus, pin, pout, pf; };
Oracle oracle_pfc(const Kirchhoff::PfcDesign& d, double rload, double tstop, double t0, double t1,
                  double precharge) {
    json p;
    p["inputVoltageRms"] = d.inputVoltageRms;   p["outputVoltage"] = d.outputVoltage;
    p["senseResistance"] = d.senseResistance;   p["outputCapacitance"] = d.outputCapacitance;
    p["loadResistance"] = rload;                p["referenceGain"] = d.referenceGain;
    p["proportionalGain"] = d.proportionalGain; p["integralGain"] = d.integralGain;
    p["outputDividerGain"] = d.outputDividerGain;
    p["rippleFraction"] = 0.30;   // matches kRippleFraction in Pfc.cpp (the hysteretic current band)
    p["tstop"] = tstop; p["windowStart"] = t0; p["windowEnd"] = t1; p["precharge"] = precharge;
    const std::string ppath = "/tmp/kirchhoff_pfc_oracle_in.json";
    { std::ofstream f(ppath); f << p.dump(); }
    const std::string cmd = "python3 " KIRCHHOFF_TEST_DIR "/averaged_model.py pfc < " + ppath + " 2>&1";
    std::string out; char buf[4096];
    FILE* fp = popen(cmd.c_str(), "r");
    if (!fp) throw std::runtime_error("failed to launch the averaged-model oracle");
    while (fgets(buf, sizeof(buf), fp)) out += buf;
    pclose(fp);
    json r = json::parse(out);   // throws (loudly) if the oracle failed — we do NOT swallow it
    return {r.at("vbus_pred").get<double>(), r.at("pin_pred").get<double>(),
            r.at("pout_pred").get<double>(), r.at("pf_pred").get<double>()};
}

// MEASURE the voltage-loop phase margin in ngspice. The switched PFC has no static operating point to
// linearise (the hysteretic switch is time-varying), so a black-box .ac on the full deck is meaningless;
// the correct measurement is .ac on the SMALL-SIGNAL loop the design targets. We realise the loop gain
//   T(s) = kv·K0·(kp + ki/s)/(s + wp),  K0 = Vrms²/(Rsense·C·Vbus),  wp = 2/(Rload·C)
// (the inner current loop emulates a conductance, so the plant is the single-pole bus cap; PI controller
// from design_pfc) as a linear ngspice circuit and read the unity-gain crossover + phase there. This
// MEASURES the margin the IMPLEMENTED gains actually produce (not the derivation's claim), and is run
// across line/load to confirm the ONE designed controller keeps margin across the envelope.
struct Margin { double fcHz, pmDeg; };
Margin measure_phase_margin(double kp, double ki, double kv, double K0, double wp, const std::string& tag) {
    const double kvK0 = kv * K0, rp = 1.0 / wp;
    std::ostringstream d;
    d << "* PFC voltage-loop linear loop gain T(s)=kv*K0*(kp+ki/s)/(s+wp)\n"
      << "Vin n0 0 DC 0 AC 1\n"
      << "Bint 0 nint I=" << fmt(ki) << "*V(n0)\n"     // integrator: ki*Vin into 1F cap
      << "Cint nint 0 1\nRint nint 0 1e12\n"
      << "Bu nu 0 V=" << fmt(kp) << "*V(n0)+V(nint)\n"  // PI: kp*Vin + ki*integral
      << "Bpl 0 nt I=" << fmt(kvK0) << "*V(nu)\n"        // plant kv*K0/(s+wp)
      << "Rp nt 0 " << fmt(rp) << "\nCp nt 0 1\n"
      << ".control\nac dec 200 0.01 1e5\n"
      << "meas ac fc when vdb(nt)=0\n"
      << "meas ac ph find vp(nt) when vdb(nt)=0\n"
      << "print fc ph\n.endc\n.end\n";
    const std::string path = "/tmp/kirchhoff_pfc_pm_" + tag + ".cir";
    { std::ofstream f(path); f << d.str(); }
    std::string out; char buf[4096];
    FILE* p = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    double fc = 0, ph = 0;
    REQUIRE(meas(out, "fc", fc));
    REQUIRE(meas(out, "ph", ph));   // radians
    return {fc, 180.0 + ph * 180.0 / 3.14159265358979323846};
}

// Build a runnable deck (transient + a one-line-cycle measurement block), optionally overriding the load.
std::string make_deck(const std::string& base, double tstop, double tstep, double t0, double t1,
                      double rloadOverride) {
    std::string deck = base;
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

// A measured PFC operating point + the matching independent prediction, with the core assertions shared
// by every swept corner.
struct Point { double vout, pf, pin, rload, vtarget; };
Point run_pfc(double vrms, double vout, double fline, double pout, double rloadOverride,
              double tstop, double tstep, double t0, double t1, const std::string& tag) {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = vrms;
    di["designRequirements"]["lineFrequency"]["nominal"] = fline;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=vout; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=vrms; json o; o["power"]=pout; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(di);
    json tas = Kirchhoff::build_pfc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);

    const double rload = rloadOverride > 0 ? rloadOverride : d.loadResistance;
    std::string out = run_ngspice(make_deck(base, tstop, tstep, t0, t1, rloadOverride), tag);
    double pavg=0, vrms_m=0, irms=0, voavg=0;
    REQUIRE(meas(out, "pavg", pavg));  REQUIRE(meas(out, "vrms", vrms_m));
    REQUIRE(meas(out, "irms", irms));  REQUIRE(meas(out, "voavg", voavg));
    const double pf = pavg / (vrms_m * irms);

    // Cross-check against the independent averaged-model oracle (same window + precharge).
    Oracle o = oracle_pfc(d, rload, tstop, t0, t1, vout);
    INFO("[" << tag << "] Vrms=" << vrms << " f=" << fline << " Pout=" << pout << " Rload=" << rload
         << "  ngspice: Vout=" << voavg << " PF=" << pf << " Pin=" << pavg
         << "  | oracle: Vout=" << o.vbus << " Pin=" << o.pin << " PFceil=" << o.pf);
    // (a) bus matches the independent prediction, and is regulated near target
    CHECK(std::abs(voavg - o.vbus) / vout < 0.03);
    CHECK(std::abs(voavg - vout) / vout < 0.06);
    // (b) drawn power matches the independent prediction
    CHECK(std::abs(pavg - o.pin) / o.pin < 0.10);
    // (c) energy balance (lossless): Pin == Vout^2/Rload — an invariant ngspice does not enforce a priori
    CHECK(std::abs(pavg - voavg * voavg / rload) / pavg < 0.10);
    // (d) PF checked against the MODEL: the raw measured PF cannot exceed the ripple-derived ceiling
    //     (o.pf ≈ 0.993), and stays within a distortion allowance below it — no hand-picked PF bound.
    CHECK(pf < o.pf + 0.01);
    CHECK(pf > o.pf - 0.03);
    return {voavg, pf, pavg, rload, vout};
}
}  // namespace

namespace {
// Build PFC design inputs, optionally with a `config` object (ABT #92 mode/variant knobs).
json pfc_inputs(double vrms, double vout, double fline, double fsw, double pout, json config = json()) {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = vrms;
    di["designRequirements"]["lineFrequency"]["nominal"] = fline;
    di["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=vout; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=vrms; json o; o["power"]=pout; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }
    if (!config.is_null()) di["config"] = config;
    return di;
}
}  // namespace

TEST_CASE("PFC conduction mode drives the boost-inductor sizing (CCM/DCM/CrM/Transition)", "[pfc][mode]") {
    // Unit-check the per-mode inductance formulas (MKF PowerFactorCorrection::calculate_inductance_
    // ccm/dcm/crcm) against an INDEPENDENT recomputation — no ngspice needed for pure sizing. All three
    // formulas evaluate at the line-voltage peak.
    const double vrms = 120.0, vout = 400.0, fline = 400.0, fsw = 20e3, pout = 300.0;
    const double vpeak = vrms * std::sqrt(2.0);
    const double pin   = pout;                       // efficiency = 1
    const double D     = 1.0 - vpeak / vout;         // boost duty at the line peak
    const double iPeak = std::sqrt(2.0) * pin / vrms;
    const double ripple = 0.30;                       // kRippleFraction

    const double Lccm_ref = vpeak * D / (ripple * iPeak * fsw);
    const double Ldcm_ref = vpeak * vpeak * D * D / (2.0 * pin * fsw);
    const double Lcrm_ref = vpeak * D / (2.0 * iPeak * fsw);

    auto L_of = [&](const char* mode) {
        json cfg; if (mode) cfg["mode"] = mode;
        return Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout,
                                                mode ? cfg : json())).boostInductance;
    };

    // Default (no mode) == explicit CCM == the original formula (regression: unchanged).
    CHECK(L_of(nullptr) == Catch::Approx(Lccm_ref).epsilon(1e-9));
    CHECK(L_of("ccm")   == Catch::Approx(Lccm_ref).epsilon(1e-9));
    CHECK(L_of("dcm")   == Catch::Approx(Ldcm_ref).epsilon(1e-9));
    CHECK(L_of("crm")   == Catch::Approx(Lcrm_ref).epsilon(1e-9));
    // Transition mode == CrM sizing (MKF folds them together).
    CHECK(L_of("transition") == Catch::Approx(Lcrm_ref).epsilon(1e-9));

    // Sanity: the modes are genuinely different, and CrM (boundary) is well below a 30%-ripple CCM design.
    CHECK(Lcrm_ref < Lccm_ref);
    // L_ccm/L_crm = (2·iPeak)/(ripple·iPeak) = 2/ripple.
    CHECK(std::abs(Lccm_ref / Lcrm_ref - 2.0 / ripple) < 1e-6);
    CHECK(L_of("dcm") != Catch::Approx(Lccm_ref));

    // The comparator hysteresis band tracks the ACTUAL peak ripple of the sized inductor, so a smaller L
    // (DCM/CrM) gives a proportionally larger band. For CCM the band is byte-identical to the original.
    Kirchhoff::PfcDesign dccm = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout));
    Kirchhoff::PfcDesign dcrm = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout,
                                                                 json{{"mode","crm"}}));
    CHECK(dcrm.currentHysteresis > dccm.currentHysteresis);

    // Unknown mode / unknown variant THROW (no silent fallback).
    CHECK_THROWS(Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout, json{{"mode","turbo"}})));
    CHECK_THROWS(Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout,
                                                  json{{"topologyVariant","flyback"}})));
    // SEPIC/Ćuk (buck-boost class) are sized in CCM ONLY — a non-CCM request throws (no wrong-topology
    // formula), but the default (CCM) designs successfully.
    for (const char* v : {"sepic", "cuk"}) {
        CHECK_NOTHROW(Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout,
                                                       json{{"topologyVariant", v}})));
        CHECK_THROWS(Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout,
                                                      json{{"topologyVariant", v}, {"mode", "dcm"}})));
    }
}

TEST_CASE("interleaved boost PFC: per-phase sizing + the deck holds the bus", "[pfc][interleaved]") {
    const double vrms = 120.0, vout = 400.0, fline = 400.0, fsw = 20e3, pout = 300.0;
    // Single-phase reference inductance (boost).
    const double L1 = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout)).boostInductance;

    for (int N : {2, 3}) {
        json cfg; cfg["topologyVariant"] = "interleaved"; cfg["numberOfPhases"] = N;
        Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout, cfg));
        // Each phase carries 1/N the current at the same ripple RATIO, so its inductor is N× larger; the
        // per-phase reference gain is 1/N of the single-phase value (design_pfc).
        CHECK(d.numberOfPhases == N);
        CHECK(d.boostInductance == Catch::Approx(N * L1).epsilon(1e-9));

        json tas = Kirchhoff::build_pfc_tas(d);
        PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
        const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);
        // Structural: N boost legs (L1..LN, SW1..SWN) + N phase comparators.
        for (int p = 1; p <= N; ++p) {
            REQUIRE(base.find("L" + std::to_string(p) + " ") != std::string::npos);
        }

        const double tstop = 12.5e-3, tstep = 1e-6, t0 = 10e-3, t1 = 12.5e-3;
        std::string out = run_ngspice(make_deck(base, tstop, tstep, t0, t1, /*rload*/0.0),
                                      "il_" + std::to_string(N));
        double pavg=0, vrms_m=0, irms=0, voavg=0;
        REQUIRE(meas(out, "pavg", pavg));  REQUIRE(meas(out, "vrms", vrms_m));
        REQUIRE(meas(out, "irms", irms));  REQUIRE(meas(out, "voavg", voavg));
        const double pf = pavg / (vrms_m * irms);
        const double rload = d.loadResistance;
        INFO("N=" << N << " Vout=" << voavg << " PF=" << pf << " Pin=" << pavg
             << " (target Vout=" << vout << ", Pout=" << pout << ", Rload=" << rload << ")");
        // Bus regulated near target, power balance holds, near-unity PF: the N phases share the load.
        CHECK(std::abs(voavg - vout) / vout < 0.06);
        CHECK(std::abs(pavg - voavg * voavg / rload) / pavg < 0.12);
        CHECK(pf > 0.95);
    }
}

TEST_CASE("totem-pole bridgeless PFC: bipolar true-sine deck holds the bus with high PF", "[pfc][totempole]") {
    const double vrms = 120.0, vout = 400.0, fline = 400.0, fsw = 20e3, pout = 300.0;
    // Same boost-class inductor sizing as the bridged boost (totem-pole IS a boost cell).
    const double Lboost = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout)).boostInductance;

    json cfg; cfg["topologyVariant"] = "totemPole";
    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout, cfg));
    CHECK(d.topologyVariant == "totemPole");
    CHECK(d.bipolar == true);
    CHECK(d.numberOfPhases == 1);
    CHECK(d.boostInductance == Catch::Approx(Lboost).epsilon(1e-9));

    json tas = Kirchhoff::build_pfc_tas(d);
    // Bridgeless: an AC input, no diode BRIDGE (four rectifier diodes) — the fast leg + a slow-diode return.
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acSinglePhase");
    CHECK(tas.at("simulation").contains("stimulus") == false);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);
    // Structural: two fast MOSFETs (Q1 high, Q2 low) + their free-wheel diodes + the slow return diodes.
    REQUIRE(base.find("SQ1 ") != std::string::npos);
    REQUIRE(base.find("SQ2 ") != std::string::npos);
    REQUIRE(base.find("DDQ1 ") != std::string::npos);
    REQUIRE(base.find("DDQ2 ") != std::string::npos);
    REQUIRE(base.find("DDa ") != std::string::npos);
    REQUIRE(base.find("DDb ") != std::string::npos);
    REQUIRE(base.find("Vac AcLine AcNeutral SIN(") != std::string::npos);

    // LOAD-ROBUSTNESS sweep: the ONE designed controller (300 W) must hold the bus near target across a
    // 0.5×–2× load range (150–600 W). The bipolar true-sine current loop keeps near-unity PF at every load.
    const double tstop = 12.5e-3, tstep = 5e-7, t0 = 10e-3, t1 = 12.5e-3;
    for (double pload : {150.0, 300.0, 600.0}) {
        const double rload = vout * vout / pload;
        std::string out = run_ngspice(make_deck(base, tstop, tstep, t0, t1, rload),
                                      "totempole_" + std::to_string((int)pload));
        double pavg=0, vrms_m=0, irms=0, voavg=0;
        if (!(meas(out, "pavg", pavg) && meas(out, "voavg", voavg)))
            WARN("totem-pole ngspice output:\n" << out.substr(0, out.size() > 4000 ? 4000 : out.size()));
        REQUIRE(meas(out, "pavg", pavg));  REQUIRE(meas(out, "vrms", vrms_m));
        REQUIRE(meas(out, "irms", irms));  REQUIRE(meas(out, "voavg", voavg));
        const double pf = pavg / (vrms_m * irms);
        INFO("totem-pole Pload=" << pload << ": Vout=" << voavg << " PF=" << pf << " Pin=" << pavg
             << " (target Vout=" << vout << ", Rload=" << rload << ")");
        CHECK(std::abs(voavg - vout) / vout < 0.06);
        CHECK(std::abs(pavg - voavg * voavg / rload) / pavg < 0.12);
        CHECK(pf > 0.95);
    }
}

TEST_CASE("SEPIC / Ćuk PFC: buck-boost front end holds the bus with high PF", "[pfc][buckboost]") {
    // A buck-boost-class PFC (Vout may sit above or below the line peak). SEPIC is non-inverting (bus +Vout);
    // Ćuk is inverting (bus −Vout, sensed via the summer negator). Both reuse the boost current + voltage
    // loops. The input inductor L1 is sized in CCM from the buck-boost duty D = Vout/(Vout+Vpk); L2 = L1 and
    // the energy-transfer cap Cs places the L2–Cs resonance a decade below fsw.
    const double vrms = 120.0, vout = 400.0, fline = 400.0, fsw = 20e3, pout = 300.0;
    const double vpeak = vrms * std::sqrt(2.0), D = vout / (vout + vpeak);
    const double iPeak = std::sqrt(2.0) * pout / vrms, ripple = 0.30;
    const double L1_ref = vpeak * D / (ripple * iPeak * fsw);   // independent CCM buck-boost sizing check

    for (const char* variant : {"sepic", "cuk"}) {
        const bool inverting = std::string(variant) == "cuk";
        const double vsign = inverting ? -1.0 : 1.0;
        json cfg; cfg["topologyVariant"] = variant;
        Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(pfc_inputs(vrms, vout, fline, fsw, pout, cfg));
        CHECK(d.topologyVariant == variant);
        CHECK(d.boostInductance == Catch::Approx(L1_ref).epsilon(1e-9));   // buck-boost duty, not boost duty
        CHECK(d.coupledInductance == Catch::Approx(d.boostInductance).epsilon(1e-9));
        CHECK(d.couplingCapacitance > 0.0);

        json tas = Kirchhoff::build_pfc_tas(d);
        PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
        const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);
        // Structural: the SEPIC/Ćuk cell has TWO inductors (L1, L2) + a coupling cap Cs + the switch/diode.
        REQUIRE(base.find("LL1_pri") != std::string::npos);   // single-winding magnetic emits as L<name>_pri
        REQUIRE(base.find("LL2_pri") != std::string::npos);
        REQUIRE(base.find("CCs ") != std::string::npos);
        REQUIRE(base.find("SSW ") != std::string::npos);
        REQUIRE(base.find("Vac AcLine AcNeutral SIN(") != std::string::npos);

        const double tstop = 12.5e-3, tstep = 5e-7, t0 = 10e-3, t1 = 12.5e-3;
        std::string out = run_ngspice(make_deck(base, tstop, tstep, t0, t1, /*rload*/0.0),
                                      std::string("bb_") + variant);
        double pavg=0, vrms_m=0, irms=0, voavg=0;
        if (!(meas(out, "pavg", pavg) && meas(out, "voavg", voavg)))
            WARN(variant << " ngspice output:\n" << out.substr(0, out.size() > 4000 ? 4000 : out.size()));
        REQUIRE(meas(out, "pavg", pavg));  REQUIRE(meas(out, "vrms", vrms_m));
        REQUIRE(meas(out, "irms", irms));  REQUIRE(meas(out, "voavg", voavg));
        const double pf = pavg / (vrms_m * irms);
        const double rload = d.loadResistance, vtarget = vsign * vout;
        INFO(variant << ": Vout=" << voavg << " PF=" << pf << " Pin=" << pavg
             << " (target Vout=" << vtarget << ", Pout=" << pout << ", Rload=" << rload << ")");
        // Bus regulated near its (signed) target, power balance holds (Pin ≈ Vout²/Rload), near-unity PF.
        CHECK(std::abs(voavg - vtarget) / vout < 0.06);
        CHECK(std::abs(pavg - voavg * voavg / rload) / pavg < 0.12);
        CHECK(pf > 0.95);
    }
}

TEST_CASE("single-phase boost PFC: structural — AC input, closed loop, designed PI blocks", "[pfc][ac]") {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=300.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(di);
    json tas = Kirchhoff::build_pfc_tas(d);
    CHECK(tas.at("inputs").at("designRequirements").at("inputType") == "acSinglePhase");
    CHECK(tas.at("simulation").contains("stimulus") == false);
    // Gains are DERIVED (positive, finite) — not the old hardcoded ki=3.
    CHECK(d.proportionalGain > 0.0);
    CHECK(d.integralGain > 0.0);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string base = Kirchhoff::tas_to_ngspice(tas, ideal);
    REQUIRE(base.find("Vac AcLine AcNeutral SIN(") != std::string::npos);
    REQUIRE(base.find("Vstim") == std::string::npos);
    REQUIRE(base.find("BMv") != std::string::npos);     // multiplier (vth = busP·gv)
    REQUIRE(base.find("BIv_i") != std::string::npos);   // integrator (voltage-loop integral path)
    REQUIRE(base.find("BSgv") != std::string::npos);    // summer (voltage-loop proportional path)
}

TEST_CASE("single-phase boost PFC: 400 Hz line/load sweep vs the independent oracle", "[pfc][ac][sweep]") {
    // 400 Hz mains: one cycle is 2.5 ms, so the (stiff-diode) sim is fast. Measure the last ~line cycle.
    const double f = 400.0, tstop = 12.5e-3, tstep = 1e-6, t0 = 10e-3, t1 = 12.5e-3;

    // (1) DESIGN-FORMULA sweep across LINE voltage: each corner is designed by the formula and run at its
    //     design load. Proves the derived gains produce a working, regulated design across the line range.
    //     NOTE on envelope: this simple hysteretic boost needs a healthy boost ratio M = Vout/Vpeak. When
    //     M → 1 (e.g. the 230 V → 400 V universal-input corner, M ≈ 1.23) the bus can sag below the line
    //     peak into uncontrolled diode conduction — a documented limitation of THIS topology (a real
    //     230 V PFC uses average-current control + a higher bus), NOT of the control derivation. So the
    //     line sweep stays in the valid-M region (≤ 150 V into a 400 V bus, M ≥ 1.9); see the [slow] case.
    for (double vrms : {100.0, 120.0, 150.0}) {
        Point p = run_pfc(vrms, 400.0, f, 300.0, /*rloadOverride*/0.0, tstop, tstep, t0, t1,
                          "line_" + std::to_string((int)vrms));
        (void)p;
    }

    // (2) LOAD-ROBUSTNESS sweep: ONE fixed design (120 V / 300 W), run across a 0.5×–2× load range. The
    //     fixed controller must hold the bus near target at every load (it was DESIGNED, not tuned to one
    //     operating point) — each load cross-checked against the oracle. Rload(P) = Vout^2 / P.
    for (double pout : {150.0, 300.0, 600.0}) {
        const double rload = 400.0 * 400.0 / pout;
        Point p = run_pfc(120.0, 400.0, f, 300.0, rload, tstop, tstep, t0, t1,
                          "load_" + std::to_string((int)pout));
        (void)p;
    }
}

TEST_CASE("single-phase boost PFC: MEASURED voltage-loop phase margin across the envelope", "[pfc][ac][margin]") {
    // Design ONE controller at nominal, then MEASURE the loop phase margin (ngspice .ac on the small-signal
    // loop) as the plant varies with line and load — the realistic "fixed controller, varying operating
    // point" robustness question the review flagged ("a different power/voltage could ring and nothing
    // would catch it"). The derived PI targets ~90° PM; we require it stays comfortably stable across the
    // whole envelope, and that the crossover sits a decade below the line at nominal.
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=300.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }
    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(di);
    const double kp = d.proportionalGain, ki = d.integralGain, kv = d.outputDividerGain;
    const double Rsense = d.senseResistance, C = d.outputCapacitance, Vbus = d.outputVoltage;

    // Nominal: PM should be ~90° and the crossover a decade below the 400 Hz line (~40 Hz).
    {
        const double K0 = 120.0 * 120.0 / (Rsense * C * Vbus), wp = 2.0 / (d.loadResistance * C);
        Margin m = measure_phase_margin(kp, ki, kv, K0, wp, "nom");
        INFO("nominal: PM=" << m.pmDeg << " deg, crossover=" << m.fcHz << " Hz");
        CHECK(m.pmDeg > 80.0);  CHECK(m.pmDeg < 100.0);
        CHECK(m.fcHz > 25.0);   CHECK(m.fcHz < 55.0);
    }
    // Fixed controller, varying plant: line scales K0 (∝ Vrms²), load scales the bus pole wp (∝ 1/Rload).
    int idx = 0;
    for (double lineScale : {0.8, 1.0, 1.4})
        for (double loadScale : {0.5, 1.0, 2.0}) {
            const double Vrms  = 120.0 * lineScale;
            const double K0    = Vrms * Vrms / (Rsense * C * Vbus);
            const double Rload = d.loadResistance / loadScale;   // higher power -> lower R -> higher wp
            const double wp    = 2.0 / (Rload * C);
            Margin m = measure_phase_margin(kp, ki, kv, K0, wp, "c" + std::to_string(idx++));
            INFO("line×" << lineScale << " load×" << loadScale << ": PM=" << m.pmDeg
                 << " deg, crossover=" << m.fcHz << " Hz");
            CHECK(m.pmDeg > 45.0);    // comfortably stable everywhere
            CHECK(m.pmDeg < 135.0);
        }
}

TEST_CASE("CIAS integrator: back-calculation anti-windup speeds recovery from saturation", "[pfc][antiwindup]") {
    // Exercise the integrator realisation (CiasCircuitConverter) directly: gain=1, ref=0, clamp [0,1],
    // IC=0.5. Drive the input hard into the HIGH rail for 2 s, then reverse it. WITHOUT the bleed the state
    // (the 1 F cap) winds far past the rail (to ~2.5) and the output stays pinned at 1 long after the input
    // reverses; WITH the bleed the state is held at the rail (1), so the output releases immediately. At
    // t=3 s the difference is unambiguous: state-with-bleed has fully unwound (out≈0) while the wound-up
    // one is still saturated (out≈1). (The full PFC loop's integrator is so gentle that windup needs
    // >100 ms to matter — so the mechanism is verified here at the block level, fast and cleanly.)
    auto out_at_3s = [&](bool antiWindup) {
        std::ostringstream d;
        d << "* integrator anti-windup unit test\n"
          << "Vdrv in 0 PWL(0 1  2 1  2.0001 -1  4 -1)\n"   // +1 over [0,2] s, -1 over [2,4] s
          << "Bi 0 raw I=1*(V(in)-(0))";
        if (antiWindup)   // the same back-calculation bleed the converter emits (Kaw=1e4, clamp [0,1])
            d << "-10000*(((V(raw)>(1))?(V(raw)-(1)):(0))+((V(raw)<(0))?(V(raw)-(0)):(0)))";
        d << "\nCi raw 0 1 IC=0.5\n"
          << "Bo out 0 V=(V(raw)<(0))?(0):((V(raw)>(1))?(1):(V(raw)))\n"
          << ".control\ntran 1e-3 4 0 1e-3 uic\n"
          << "meas tran o3 FIND v(out) AT=3\nprint o3\n.endc\n.end\n";
        std::string out = run_ngspice(d.str(), antiWindup ? "intaw_on" : "intaw_off");
        double o3 = 0; REQUIRE(meas(out, "o3", o3));
        return o3;
    };
    const double onAt3  = out_at_3s(true);
    const double offAt3 = out_at_3s(false);
    INFO("integrator output at t=3 s (1 s after the input reversed): bleed ON=" << onAt3
         << " (recovered), OFF=" << offAt3 << " (still wound up)");
    CHECK(onAt3  < 0.5);    // bleed: state was pinned at the rail, has unwound -> output released
    CHECK(offAt3 > 0.5);    // no bleed: state wound past the rail, output still saturated
}

TEST_CASE("single-phase boost PFC: realistic 50 Hz mains runs and matches the oracle", "[pfc][ac][mains50]") {
    // The realistic 50 Hz path (a line cycle is 20 ms; the voltage loop crosses over a decade below the
    // line, ~5 Hz). The bus is precharged, so two line cycles confirm the mechanism holds at 50 Hz and
    // tracks the independent oracle over the SAME window. This previously existed only in an un-run demo.
    //
    // TIMESTEP NOTE: the 20 kHz hysteretic current loop must be resolved finely (5e-7 ≈ 100 pts/switching
    // period). A coarse step lets the sensed current overshoot the hysteresis band, mis-averaging the
    // emulated conductance; over the long 50 Hz window the slow voltage loop then integrates that error
    // into a FALSE bus collapse — which the oracle catches as a divergence. With the resolved step the run
    // is both correct AND fast (~1 s), so it runs by default (the [mains50] tag just lets you select it).
    const double f = 50.0, tstop = 40e-3, tstep = 5e-7, t0 = 20e-3, t1 = 40e-3;
    // 120 V → 400 V (M ≈ 2.35, the realistic demo point with a healthy boost ratio).
    Point p = run_pfc(120.0, 400.0, f, 300.0, /*rloadOverride*/0.0, tstop, tstep, t0, t1, "mains50");
    INFO("50 Hz mains: Vout=" << p.vout << " PF=" << p.pf << " Pin=" << p.pin);
    CHECK(p.pf > 0.95);      // near-unity PF sustained over the realistic 50 Hz line cycle
}
