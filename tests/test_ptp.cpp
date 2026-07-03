// PtP cross-validation: for each topology's reference operating point, run BOTH Kirchhoff run engines —
// the analytical operating-point predictor (Analytical.hpp) and the in-process ngspice simulation
// (NgspiceRunner.hpp) — and assert they BEHAVE THE SAME: they agree on the converter's output and the
// power balance is physical. This ports the spirit of MKF's *ReferenceDesignsPtp tests (which likewise
// cross-check the analytical and SPICE engines), reframed as scalar gates because KH's analytical engine
// yields scalar operating-point quantities (Vout/Iout/Pin/per-winding RMS) rather than full waveforms.
//
// Reference operating points are the real, MKF-derived points captured per topology in
// tests/reference/<topo>.mkf.json (Vin/Vout/P/fsw). The whole path is ideal-coupling (magnetics-free),
// exactly KH's scope. Skipped without libngspice (surfaced, not silently passed).
//
// What the two engines actually compute (important for reading the tolerances): the analytical engine
// returns the design REGULATION SETPOINT (the spec target Vout — the lossless design intent), while the
// ngspice deck settles the REAL OPEN-LOOP LOSSY circuit. For most topologies the open-loop deck lands on
// target (<2%). A few carry a small, understood, INVESTIGATED open-loop offset — these are NOT engine
// disagreements or bugs, and each is widened past the tight default ONLY with a cited root cause:
//   • weinberg (−3..5%): real open-loop boost droop — a high-duty current-fed boost loses ~5% of ideal
//     gain to leakage-commutation; design_weinberg compensates only the diode drop (η passed as 1.0).
//     [independent root-cause investigation, June 2026 — NOT a settle artifact: flat 400→16000 periods]
//   • push_pull SN6501 (+5%): the fixed 2.2 nF node-snubber rings on the ideal lossless (k=0.9999)
//     transformer, injecting load-independent charge — worst at the lowest current (0.35 A); at a
//     right-sized snubber → <0.3%. [independent investigation — ideal-deck artifact, not a bug]
//   • pshb (+3%): the deliberate kOuterTrim=0.01 reproduces MKF's +4% reference over-delivery; at
//     outerTrim=0 the deck lands <0.7% on target. [independent investigation — design margin, passes default]
// (The deeper reconciliation — make the analytical engine predict the lossy open-loop point instead of
// echoing the setpoint — is tracked as a follow-up; the per-topology source fixes all trade off KH's
// deliberate MKF-reference matching, so they are left as explicit decisions, not auto-applied here.)
//
// Gates per design:
//   G1 Vout consistency  — ngspice-settled |Vout| ≈ analytical-predicted |Vout| · expectedRatio (±tol)
//   G2 Iout consistency  — ngspice Iout (|Vout|/Rload) ≈ analytical Iout (±tol)
//   G3 power balance     — 0 ≤ (Pin − Pout)/Pin ≤ lossMax on the ngspice result (energy conservation)

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "Analytical.hpp"
#include "NgspiceRunner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

using nlohmann::json;

namespace {

std::string num(double v) { std::ostringstream o; o.precision(12); o << v; return o.str(); }

// A single-output DC spec at a reference operating point. `eta` is the design efficiency the converter
// is SIZED for — 1.0 (idealized) for most topologies, but a realistic value where the duty/turns sizing
// must account for loss to land the open-loop deck on target (the high-duty current-fed Weinberg boost).
json spec_for(double vin, double vout, double power, double fsw, double eta = 1.0) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name", "out"}, {"voltage", {{"nominal", vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}} });
    return s;
}

// ngspice run of an assembled TAS: settle to steady state, return averaged Vout + input current.
struct SimResult { double vout; double iin; bool ok; };
SimResult run_spice(const json& tas, double fsw, double rc) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    const double period = 1.0 / fsw;
    const double tstop = std::max(400.0 * period, std::ceil(30.0 * rc / period) * period);
    const double tstep = period / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    if (!r.success) return {0, 0, false};
    // Average the DC quantities over MANY periods, not one: a single-period average of the input-source
    // current is sensitive to switching ripple and any slow settling ring (e.g. the ACF's 10 uF clamp cap,
    // which settles over ~2000 cycles — far past the output RC that sizes tstop). Averaging ~40 periods
    // rejects that and yields the true DC power draw; for an already-settled deck it equals the 1-period
    // average. tstop >= 400 periods, so the window stays safely in the post-settling region.
    const double navg = std::min(0.5 * tstop, 40.0 * period);
    auto v = r.average("v(Vout)", tstop - navg, tstop);
    auto iin = r.average("i(VVin)", tstop - navg, tstop);
    return {v.value_or(0.0), iin.value_or(0.0), v.has_value()};
}

struct Ref {
    std::string topo;            // topology
    std::string design;          // reference-design name (datasheet/app-note)
    std::function<json()> tas;   // design + assemble at this reference point
    double vin, vout, power, fsw;
    double voutTol;              // G1/G2 tolerance
    double expectedRatio;        // ngspice/analytical Vout ratio expected by design (1.0 except SRC headroom)
};

// Reference designs = real datasheet / app-note operating points, transcribed from MKF's
// *ReferenceDesignsPtp suites (3 per topology where MKF has them; cross-checked here through BOTH KH
// engines). expectedRatio: SRC carries a deliberate ~8% open-loop gain headroom (abt #62); all else
// hits target (ratio 1).
std::vector<Ref> refs() {
    using namespace Kirchhoff;
    std::vector<Ref> v;
    // Default G1/G2 tolerance: 5% — the open-loop deck lands on the regulation setpoint to <2% for most
    // topologies; 5% covers the small understood margins (e.g. pshb's outerTrim ~3%). Anything wider is
    // an explicit per-design exception with a cited root-cause investigation (see header).
    auto add = [&](const char* topo, const char* design, auto designFn, auto buildFn,
                   double vin, double vout, double iout, double fs, double tol = 0.05, double ratio = 1.0,
                   double eta = 1.0) {
        json spec = spec_for(vin, vout, vout * iout, fs, eta);
        v.push_back({topo, design, [=]{ return buildFn(designFn(spec)); }, vin, vout, vout * iout, fs, tol, ratio});
    };
    // ── non-isolated ──
    add("buck","TPS54202EVM", design_buck, build_buck_tas, 12,5,2,500000);
    add("buck","LMR33630",    design_buck, build_buck_tas, 12,5,3,400000);
    add("buck","LM5146-Q1",   design_buck, build_buck_tas, 24,12,8,400000);
    add("boost","TPS61089",   design_boost, build_boost_tas, 5,9,2,400000);
    add("boost","TPS61178",   design_boost, build_boost_tas, 7.2,16,2,300000);
    add("boost","LM5122",     design_boost, build_boost_tas, 12,24,4.5,250000);
    add("sepic","SNVA168E",   design_sepic, build_sepic_tas, 5,12,0.5,600000);
    add("sepic","LTC1871",    design_sepic, build_sepic_tas, 3.3,5,1,250000);
    add("sepic","TIDA-00781", design_sepic, build_sepic_tas, 12,12,1,250000);
    add("zeta","PMP9581",     design_zeta, build_zeta_tas, 12,5,1,600000);
    add("zeta","LM5085",      design_zeta, build_zeta_tas, 12,12,1,300000);
    add("zeta","step-up",     design_zeta, build_zeta_tas, 5,12,0.5,600000);
    add("cuk","fixture",      design_cuk, build_cuk_tas, 12,12,2,100000);   // MKF Cuk specs encode Vout in titles
    add("fsbb","fixture",     design_fsbb, build_fsbb_tas, 12,12,2,100000); // MKF FSBB: 8 multi-region specs
    // ── isolated single/two-switch ──
    add("flyback","PMP30817", design_flyback, build_flyback_tas, 24,6,0.2,250000);
    add("flyback","LM5180",   design_flyback, build_flyback_tas, 24,15,0.2,200000);
    add("flyback","TIDA-00709",design_flyback, build_flyback_tas, 120,12,2.75,70000);
    add("forward","fixture",  design_forward, build_forward_tas, 48,12,2,100000);
    add("two_switch","fixture",design_two_switch_forward, build_two_switch_forward_tas, 48,12,2,100000);
    // push-pull now sizes node snubbers from the energy budget (src/PushPull.cpp), so SN6501 (which the
    // fixed 2.2 nF lifted +5% via ringing at 0.35 A) lands on target too — all three at the default band.
    add("push_pull","SN6501", design_push_pull, build_push_pull_tas, 5,3.3,0.35,410000);
    add("push_pull","SN6505B",design_push_pull, build_push_pull_tas, 5,3.3,1,420000);
    add("push_pull","SN6507", design_push_pull, build_push_pull_tas, 12,5,1,200000);
    add("acf","UCC2897A",     design_acf, build_acf_tas, 48,3.3,30,250000);
    add("acf","Erickson-50W", design_acf, build_acf_tas, 28,5,10,200000);
    add("acf","AN1023-200W",  design_acf, build_acf_tas, 48,12,16,250000);
    // ── isolated bridge / phase-shift ──
    add("ahb","SLUP223",      design_ahb, build_ahb_tas, 100,5,20,200000);
    add("ahb","AN4153",       design_ahb, build_ahb_tas, 100,12,16,100000);
    add("ahb","AN2852",       design_ahb, build_ahb_tas, 90,19,4.7,100000);
    add("psfb","Telecom-600W",design_psfb, build_psfb_tas, 400,12,50,100000);
    add("psfb","Server-1.2kW",design_psfb, build_psfb_tas, 400,24,50,100000);
    add("psfb","EV-Aux-1kW",  design_psfb, build_psfb_tas, 400,48,21,100000);
    add("pshb","Telecom-600W",design_pshb, build_pshb_tas, 400,12,50,100000);
    add("pshb","Server-1.2kW",design_pshb, build_pshb_tas, 400,24,50,100000);
    add("pshb","EV-Aux-1kW",  design_pshb, build_pshb_tas, 400,48,21,100000);
    // weinberg: a high-duty current-fed boost has real open-loop droop (leakage-commutation loss,
    // amplified by 1/(1−D)²). design_weinberg already folds the design efficiency η into the boost duty
    // (duty_boost(…, η)); the droop only appeared because the PtP idealized η=1.0. Passing the realistic
    // η that real Weinberg designs run at engages that existing compensation and lands the open-loop deck
    // on target. (The droop is current-dependent, so one η can't perfectly centre all three; residual is
    // within the default band.) [root cause: independent investigation — real droop, not a settle artifact]
    add("weinberg","Schreuders",design_weinberg, build_weinberg_tas, 50,150,10,50000, 0.05, 1.0, 0.95);
    add("weinberg","Yadav",   design_weinberg, build_weinberg_tas, 42,100,5,100000, 0.05, 1.0, 0.95);
    add("weinberg","IJRTE",   design_weinberg, build_weinberg_tas, 24,50,5,100000, 0.05, 1.0, 0.95);
    // ── resonant ──
    add("llc","Telecom-120W", design_llc, build_llc_tas, 400,12,10,100000);
    add("llc","ATX-240W",     design_llc, build_llc_tas, 400,24,10,100000);
    add("llc","EV-1kW",       design_llc, build_llc_tas, 400,48,20,100000);
    add("src","500W-FB",      design_src, build_src_tas, 400,48,10,100000, 0.07, 1.08); // 8% headroom
    return v;
    // Excluded (need special specs/handling, validated in test_mkf_equivalence): isolated_buck /
    // isolated_buck_boost (2-output), DAB (output floats), CLLC / CLLLC (initial-conditions + active SR),
    // PFC / Vienna (AC-input).
}

} // namespace

TEST_CASE("PtP: ngspice and analytical engines agree per topology", "[ptp]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping ngspice-vs-analytical PtP cross-validation");
        return;
    }
    for (const auto& d : refs()) {
        INFO("topology: " << d.topo << "  design: " << d.design);
        json tas = d.tas();

        // --- analytical engine ---
        Kirchhoff::AnalyticalOperatingPoint a = Kirchhoff::analytical_operating_point(tas);
        const double aVout = std::abs(a.outputVoltage);
        const double aIout = std::abs(a.outputCurrent);

        // --- ngspice engine ---
        const double rload = (d.vout * d.vout) / d.power;       // nominal load
        SimResult s = run_spice(tas, d.fsw, rload * 47e-6 /*rough RC for settle*/);
        REQUIRE(s.ok);
        const double sVout = std::abs(s.vout);
        const double sIout = sVout / rload;
        const double pin  = std::abs(s.iin) * d.vin;
        const double pout = sVout * sIout;

        INFO("analytical Vout=" << aVout << " Iout=" << aIout
             << " | ngspice Vout=" << sVout << " Iout=" << sIout
             << " | Pin=" << pin << " Pout=" << pout);

        // G1 — output voltage: the two engines agree (modulo the design's expected open-loop ratio).
        CHECK(sVout == Catch::Approx(aVout * d.expectedRatio).epsilon(d.voutTol));
        // G2 — output current consistency.
        CHECK(sIout == Catch::Approx(aIout * d.expectedRatio).epsilon(d.voutTol));
        // G3 — power balance / energy conservation on the ngspice result. The decisive check is that the
        // sim does not MANUFACTURE energy (loss >= 0). The "ideal" decks still dissipate a real amount —
        // near-ideal diode Vf (~0.8 V, ~7%/diode at a 12 V rail; doubled for full-bridge rectifiers),
        // switch RON=0.01, and dV/dt snubbers — so the upper bound is a generous sanity ceiling, not a
        // tight efficiency claim (e.g. the NPC PSHB stack runs ~25% on an ideal deck).
        REQUIRE(pin > 0.0);
        const double loss = (pin - pout) / pin;
        CHECK(loss >= -0.02);          // no manufactured energy
        CHECK(loss <= 0.35);           // sane ceiling for an ideal-device deck (catches gross bugs)
    }
}

TEST_CASE("PtP: DAB EPS/DPS deliver spec through inner phase shift", "[ptp][dab][eps]") {
    // End-to-end validation that the EPS/DPS deck gate timing matches the analytical design: with an inner
    // phase shift, design_dab re-sizes the series inductance from the general power model so the OPEN-LOOP
    // ngspice deck still lands on the target output. If the leg-shift gate convention (QA@D1 / QE@D3+D2)
    // disagreed with the solver's Vab_at/Vcd_at, Vout would miss badly.
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping DAB EPS/DPS PtP");
        return;
    }
    // Power kept moderate (100 W): the IDEAL non-SPS deck's zero-voltage plateaus commutate the rectifier
    // hard, and above ~100 W the ideal-switch transient stops converging ("timestep too small"). That is a
    // sim-robustness limit of the ideal deck, NOT the design — design_dab sizes L from the general power
    // model at any power (see the analytical round-trip test), and the SPS deck converges at full power.
    const double P = 100.0, Vout = 24.0;
    struct Mod { const char* name; double d1; double d2; };
    for (const Mod m : {Mod{"SPS", 0.0, 0.0}, Mod{"EPS", 30.0, 0.0}, Mod{"DPS", 25.0, 25.0}, Mod{"TPS", 40.0, 20.0}}) {
        INFO("modulation: " << m.name << " D1=" << m.d1 << " D2=" << m.d2);
        json spec = spec_for(400, Vout, P, 100000);
        spec["config"]["dabInnerPhaseShift1Deg"] = m.d1;
        spec["config"]["dabInnerPhaseShift2Deg"] = m.d2;
        json tas = Kirchhoff::build_dab_tas(Kirchhoff::design_dab(spec));
        SimResult s = run_spice(tas, 100000, (Vout * Vout / P) * 100e-6 /* DAB output cap ~100u */);
        REQUIRE(s.ok);
        INFO("ngspice Vout=" << s.vout);
        CHECK(s.vout == Catch::Approx(Vout).epsilon(0.12));   // open-loop within 12% of the 24 V target
    }
}

TEST_CASE("PtP: Sepic synchronous rectifier delivers spec", "[ptp][sepic][syncrect]") {
    // MKF Sepic V4 variant: config.rectifier="synchronous" swaps D1 for a low-side sync MOSFET Q2 driven
    // complementary to Q1. Verify the emitted deck converges and lands on the target output (proves the Q2
    // orientation + complementary gate timing are correct), and that a diode-rectifier build still works.
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice — skipping"); return; }
    for (const char* rect : {"diode", "synchronous"}) {
        INFO("rectifier=" << rect);
        json sepicSpec = spec_for(12, 5, 5, 500000);
        sepicSpec["config"]["rectifier"] = rect;
        SimResult sp = run_spice(Kirchhoff::build_sepic_tas(Kirchhoff::design_sepic(sepicSpec)), 500000, (5.0 * 5.0 / 5.0) * 1e-4);
        INFO("sepic Vout=" << sp.vout);
        REQUIRE(sp.ok);
        CHECK(sp.vout == Catch::Approx(5.0).epsilon(0.12));

        json zetaSpec = spec_for(12, 5, 5, 500000);
        zetaSpec["config"]["rectifier"] = rect;
        SimResult zt = run_spice(Kirchhoff::build_zeta_tas(Kirchhoff::design_zeta(zetaSpec)), 500000, (5.0 * 5.0 / 5.0) * 1e-4);
        INFO("zeta Vout=" << zt.vout);
        REQUIRE(zt.ok);
        CHECK(zt.vout == Catch::Approx(5.0).epsilon(0.12));

        // Cuk is inverting → |Vout| ~ target.
        json cukSpec = spec_for(12, 12, 24, 100000);
        cukSpec["config"]["rectifier"] = rect;
        SimResult ck = run_spice(Kirchhoff::build_cuk_tas(Kirchhoff::design_cuk(cukSpec)), 100000, (12.0 * 12.0 / 24.0) * 1e-4);
        INFO("cuk Vout=" << ck.vout);
        REQUIRE(ck.ok);
        CHECK(std::abs(ck.vout) == Catch::Approx(12.0).epsilon(0.12));
    }
}

TEST_CASE("PtP: FSBB operating regions deliver spec (buck / boost / buck-boost)", "[ptp][fsbb][region]") {
    // MKF FourSwitchBuckBoost runs only the buck leg when Vin>Vo (BUCK), only the boost leg when Vo>Vin
    // (BOOST), and all four switches in the transition band (BUCK_BOOST). KH previously ran SIMULTANEOUS for
    // every point. Verify each region's deck converges and lands on target, and that the region is classified.
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice — skipping"); return; }
    struct Case { const char* name; double vin; double vout; const char* region; };
    for (const Case c : {Case{"buck", 24, 12, "buck"}, Case{"boost", 12, 24, "boost"}, Case{"buckboost", 12, 12, "buckBoost"}}) {
        INFO("case " << c.name << " " << c.vin << "->" << c.vout);
        json spec = spec_for(c.vin, c.vout, c.vout * 2.0, 100000);
        Kirchhoff::FsbbDesign d = Kirchhoff::design_fsbb(spec);
        CHECK(d.region == std::string(c.region));
        SimResult s = run_spice(Kirchhoff::build_fsbb_tas(d), 100000, (c.vout * c.vout / (c.vout * 2.0)) * 1e-4);
        INFO("region=" << d.region << " Vout=" << s.vout);
        REQUIRE(s.ok);
        CHECK(s.vout == Catch::Approx(c.vout).epsilon(0.12));
    }
}

// ── ABT #94: split-PWM sub-mode measurably lowers the inductor-current ripple ─────────────────────
// In the buck-boost transition band the MKF-default SPLIT_PWM scheme phase-shifts the buck and boost
// legs (different per-leg duties) so the inductor sees a mild (Vin−Vo) freewheel interval instead of
// the full ±Vin/∓Vo swing of SIMULTANEOUS. Build BOTH decks at the SAME operating point / same L and
// verify in ngspice: each lands Vout on target, and SPLIT_PWM's inductor-current ripple is strictly
// lower. (The inductor is emitted as "LL_pri"; its branch current is captured as ll_pri#branch.)
namespace {
struct FsbbMeas { double vout; double iLpp; bool ok; };
FsbbMeas measure_fsbb(const json& tas, double fsw) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    const double period = 1.0 / fsw;
    const double tstop = 500.0 * period;      // ~8 output RC (Rload=6Ω, Co=100uF) → settled
    const double tstep = period / 400.0;      // fine enough to resolve the triangular ripple
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
    Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    if (!r.success) return {0, 0, false};
    const double from = tstop - 3.0 * period, to = tstop;   // a few settled periods
    auto v = r.average("v(Vout)", from, to);
    // Locate the inductor branch-current vector (raw key contains "ll_pri") and take its peak-to-peak.
    const std::vector<double>* iL = nullptr;
    for (const auto& kv : r.vectors) {
        std::string key = kv.first;
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c){ return std::tolower(c); });
        if (key.find("ll_pri") != std::string::npos) { iL = &kv.second; break; }
    }
    if (!iL || !v) return {v.value_or(0.0), 0.0, false};
    double lo = 1e300, hi = -1e300;
    for (size_t i = 0; i < r.time.size() && i < iL->size(); ++i) {
        if (r.time[i] < from || r.time[i] > to) continue;
        lo = std::min(lo, (*iL)[i]);
        hi = std::max(hi, (*iL)[i]);
    }
    return {*v, hi - lo, hi >= lo};
}
}  // namespace

TEST_CASE("PtP: FSBB split-PWM lowers inductor ripple vs simultaneous (ABT #94)", "[ptp][fsbb][splitpwm]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice — skipping"); return; }
    const double fsw = 100000;
    // 12 V -> 12 V lands in the buck-boost transition band, where the sub-mode matters.
    json base = spec_for(12, 12, 24, fsw);
    json splitSpec = base;  splitSpec["config"]["transitionMode"] = "splitPwm";      // MKF default
    json simulSpec = base;  simulSpec["config"]["transitionMode"] = "simultaneous";
    Kirchhoff::FsbbDesign dS = Kirchhoff::design_fsbb(splitSpec);
    Kirchhoff::FsbbDesign dM = Kirchhoff::design_fsbb(simulSpec);
    REQUIRE(dS.region == "buckBoost");
    REQUIRE(dM.region == "buckBoost");
    CHECK(dS.inductance == Catch::Approx(dM.inductance));   // same L → a fair ripple comparison

    FsbbMeas s = measure_fsbb(Kirchhoff::build_fsbb_tas(dS), fsw);
    FsbbMeas m = measure_fsbb(Kirchhoff::build_fsbb_tas(dM), fsw);
    REQUIRE(s.ok);
    REQUIRE(m.ok);
    INFO("splitPwm  Vout=" << s.vout << " ΔiL=" << s.iLpp
         << " | simultaneous Vout=" << m.vout << " ΔiL=" << m.iLpp);
    CHECK(s.vout == Catch::Approx(12.0).epsilon(0.12));     // splitPwm lands on target
    CHECK(m.vout == Catch::Approx(12.0).epsilon(0.12));     // simultaneous lands on target
    CHECK(s.iLpp < m.iLpp * 0.9);                           // split PWM measurably cuts the ripple
}

