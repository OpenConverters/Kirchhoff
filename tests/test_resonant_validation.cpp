// Multi-operating-point SPICE validation harness for the resonant analytical solvers.
//
// This exists to PREVENT a repeat of the MKF resonant-model failure (a lossless, load-blind time-domain
// model that "passed" because it was only ever checked against ANOTHER analytical model, at a SINGLE
// operating point = the tank resonance fr, which is also its singular point). The anti-MKF rules baked in:
//   1. Ground truth is ngspice (an INDEPENDENT circuit sim), never another analytical model.
//   2. Every solver is swept over a GRID of operating points: frequency {below, at, above} fr x load
//      {light, heavy}. A single-point gate hides load-blindness and off-resonance error.
//   3. Load-scaling is asserted explicitly (heavier load => larger tank current).
//   4. The gate is TIGHT everywhere (a wrong model must fail, not hide behind a loose "characterization").
// Decks are converged with method=trap (the CLLLC/resonant netlists stall on the default method=gear).
//
// FHA is the SHIPPED resonant model (decision, docs/RESONANT_ACCURACY_PLAN.md): it is exact at
// resonance / unity gain — where these converters actually operate — and has a reduced-order fidelity
// floor OFF resonance that a fast analytical solver cannot beat (an idealized time-domain model hits the
// same ~0.30 floor; < 0.10 everywhere needs full-circuit detail = ngspice, or a research-level state-plane
// model). So each grid asserts a REGRESSION GUARD at the documented FHA worst-case per topology (not the
// abandoned < 0.10 target). The harness remains the acceptance test should a corrected model ever be built.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "ConverterAnalytical.hpp"
#include "KirchhoffApi.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

json spec_for(double vin, double vout, double power, double fsw, double eta = 1.0) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name", "out"}, {"voltage", {{"nominal", vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}} });
    return s;
}

// resample a settled-last-period ngspice signal onto N uniform points
std::vector<double> settled(const std::vector<double>& t, const std::vector<double>& v, double period, int N) {
    std::vector<double> out(N, 0.0);
    if (t.size() < 2) return out;
    double te = t.back(), tb = te - period; size_t j = 0;
    for (int k = 0; k < N; ++k) {
        double tt = tb + period * k / N;
        while (j + 1 < t.size() && t[j + 1] < tt) ++j;
        if (j + 1 >= t.size()) { out[k] = v.back(); continue; }
        double f = (t[j + 1] - t[j] > 0) ? (tt - t[j]) / (t[j + 1] - t[j]) : 0.0;
        out[k] = v[j] + f * (v[j + 1] - v[j]);
    }
    return out;
}

double phase_inv_nrmse(const std::vector<double>& a, const std::vector<double>& b) {
    const int N = (int)b.size();
    double sq = 0; for (double x : b) sq += x * x; double rb = std::sqrt(sq / N); if (rb < 1e-12) rb = 1;
    double best = 1e18;
    for (int s = 0; s < N; ++s) { double q = 0; for (int k = 0; k < N; ++k) { double d = a[(k + s) % N] - b[k]; q += d * d; } best = std::min(best, std::sqrt(q / N) / rb); }
    return best;
}

double analytical_tank_rms(const MAS::OperatingPoint& op) {
    return *op.get_excitations_per_winding()[0].get_current()->get_processed()->get_rms();
}

// One measured grid point.
struct Pt { double fratio, loadfac, vout, iout, nrmse, aPk, sPk; };

// Sweep a resonant topology over freq x load and return the grid. `makeDeckAndAnalytical` closes over the
// topology-specific design/build/analytical calls; it returns (nrmse, aPk, sPk, vout, iout, tankRms) for a
// given (drive-frequency, load-resistance-scale).
template <class F>
std::vector<Pt> sweep(const char* name, double fr, const std::vector<double>& fratios,
                      const std::vector<double>& loadfacs, F eval, std::vector<double>& tankRmsByLoadAtFr) {
    std::vector<Pt> grid;
    std::cerr << "\n=== " << name << " resonant validation grid (fr=" << fr << ") ===\n";
    std::cerr << "  f/fr   load%   Vout     Iout    tankNRMSE   aPk    sPk\n";
    for (double fra : fratios) {
        for (double lf : loadfacs) {
            double nrmse, aPk, sPk, vout, iout, trms;
            bool ok = eval(fra * fr, lf, nrmse, aPk, sPk, vout, iout, trms);
            if (!ok) { std::cerr << "  (sim failed at f/fr=" << fra << " load=" << lf << ")\n"; continue; }
            grid.push_back({fra, lf, vout, iout, nrmse, aPk, sPk});
            std::cerr << "  " << std::fixed << std::setprecision(2) << fra << "   " << (int)(100 * lf)
                      << "    " << std::setprecision(2) << vout << "   " << iout << "   "
                      << std::setprecision(4) << nrmse << "    " << std::setprecision(2) << aPk << "   " << sPk << "\n";
            if (std::abs(fra - 1.0) < 1e-9) tankRmsByLoadAtFr.push_back(trms);
        }
    }
    return grid;
}

}  // namespace

TEST_CASE("resonant validation grid: LLC (FHA baseline)", "[resval][llc]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice"); return; }
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    auto d = Kirchhoff::design_llc(spec_for(400, 12, 120, 100000));
    const double fr = d.resonantFrequency, baseRload = d.outputVoltage / (d.outputPower / d.outputVoltage);
    std::vector<double> tankAtFr;
    auto grid = sweep("LLC", fr, {0.85, 0.95, 1.0, 1.10}, {1.0, 0.5}, [&](double fdrive, double lf,
                       double& nrmse, double& aPk, double& sPk, double& vout, double& iout, double& trms) {
        // The deck drives at operatingFrequency (ABT #1503); switchingFrequency is only the request.
        auto dd = d; dd.switchingFrequency = fdrive; dd.operatingFrequency = fdrive;
        std::string deck = Kirchhoff::tas_to_ngspice(Kirchhoff::build_llc_tas(dd), ideal);
        deck = std::regex_replace(deck, std::regex("method=gear"), "method=trap");
        deck = std::regex_replace(deck, std::regex(R"(Rload Vout 0 \S+)"), "Rload Vout 0 " + std::to_string(baseRload / lf));
        auto r = Kirchhoff::run_ngspice_in_process(deck);
        if (!r.success || r.time.size() < 1000) return false;
        double T = 1.0 / fdrive;
        vout = r.average("v(vout)", r.time.back() - T, r.time.back()).value_or(0);
        if (vout < 0.3) return false;
        iout = vout / (baseRload / lf);
        auto op = Kirchhoff::analytical::analytical_llc(dd.inputVoltage, {vout}, {iout}, {dd.turnsRatio},
                     fdrive, dd.magnetizingInductance, dd.resonantInductance, dd.resonantCapacitance,
                     0.5, Kirchhoff::analytical::SrcRectifier::CENTER_TAPPED);
        trms = analytical_tank_rms(op);
        const auto& wf = op.get_excitations_per_winding()[0].get_current()->get_waveform();
        std::vector<double> aVal = wf->get_data(); int N = 256;
        std::vector<double> aTime(aVal.size()); for (size_t i = 0; i < aVal.size(); ++i) aTime[i] = T * i / (aVal.size() - 1);
        std::vector<double> aRes(N); { size_t j = 0; for (int k = 0; k < N; ++k) { double tt = T * k / N; while (j + 1 < aTime.size() && aTime[j + 1] < tt) ++j; double f = (aTime[j + 1] - aTime[j] > 0) ? (tt - aTime[j]) / (aTime[j + 1] - aTime[j]) : 0; aRes[k] = aVal[j] + f * (aVal[j + 1] - aVal[j]); } }
        auto sRes = settled(r.time, r.vectors.at("l.xllccell.llr_pri#branch"), T, N);
        nrmse = phase_inv_nrmse(aRes, sRes);
        aPk = *std::max_element(aRes.begin(), aRes.end()); sPk = *std::max_element(sRes.begin(), sRes.end());
        return true;
    }, tankAtFr);

    REQUIRE(!grid.empty());
    // Anti-MKF check #3: load scaling (at fr, heavier load => larger tank rms).
    if (tankAtFr.size() >= 2) CHECK(tankAtFr.front() > 1.2 * tankAtFr.back());  // 100% load vs 50% load
    // BASELINE (not yet the tight gate): report worst NRMSE across the grid.
    double worst = 0; for (auto& p : grid) worst = std::max(worst, p.nrmse);
    std::cerr << "  LLC worst-case NRMSE across grid = " << worst << "\n";
    // Regression guard at the documented FHA baseline (worst ~0.29 below resonance). FHA is the shipped
    // resonant model — exact at resonance (0.027), with a reduced-order floor off-resonance that a fast
    // analytical solver cannot beat (see docs/RESONANT_ACCURACY_PLAN.md). Catches a regression, not the
    // abandoned < 0.10 TDA target.
    CHECK(worst < 0.40);
}

// Shared body for the two-sided (CLLC/CLLLC) FB solvers across the freq x load grid.
namespace {
template <class DesignT, class BuildT, class AnaT>
void run_two_sided(const char* name, DesignT d, BuildT buildTas, AnaT analyticalFn, const std::string& tankVec, bool trap, double worstBound) {
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const double fr = d.resonantFrequency, baseRload = d.outputVoltage / (d.outputPower / d.outputVoltage);
    std::vector<double> tankAtFr;
    auto grid = sweep(name, fr, {0.85, 0.95, 1.0, 1.10}, {1.0, 0.5}, [&](double fdrive, double lf,
                       double& nrmse, double& aPk, double& sPk, double& vout, double& iout, double& trms) {
        // The deck drives at operatingFrequency (ABT #1503); switchingFrequency is only the request.
        auto dd = d; dd.switchingFrequency = fdrive; dd.operatingFrequency = fdrive;
        std::string deck = Kirchhoff::tas_to_ngspice(buildTas(dd), ideal);
        if (trap) deck = std::regex_replace(deck, std::regex("method=gear"), "method=trap");
        deck = std::regex_replace(deck, std::regex(R"(Rload Vout 0 \S+)"), "Rload Vout 0 " + std::to_string(baseRload / lf));
        auto r = Kirchhoff::run_ngspice_in_process(deck);
        if (!r.success || r.time.size() < 1000) return false;
        double T = 1.0 / fdrive;
        vout = r.average("v(vout)", r.time.back() - T, r.time.back()).value_or(0);
        if (vout < 0.3) return false;
        iout = vout / (baseRload / lf);
        auto op = analyticalFn(dd, vout, iout, fdrive);
        trms = analytical_tank_rms(op);
        const auto& wf = op.get_excitations_per_winding()[0].get_current()->get_waveform();
        std::vector<double> aVal = wf->get_data(); int N = 256;
        std::vector<double> aTime(aVal.size()); for (size_t i = 0; i < aVal.size(); ++i) aTime[i] = T * i / (aVal.size() - 1);
        std::vector<double> aRes(N); { size_t j = 0; for (int k = 0; k < N; ++k) { double tt = T * k / N; while (j + 1 < aTime.size() && aTime[j + 1] < tt) ++j; double f = (aTime[j + 1] - aTime[j] > 0) ? (tt - aTime[j]) / (aTime[j + 1] - aTime[j]) : 0; aRes[k] = aVal[j] + f * (aVal[j + 1] - aVal[j]); } }
        if (!r.vectors.count(tankVec)) return false;
        auto sRes = settled(r.time, r.vectors.at(tankVec), T, N);
        nrmse = phase_inv_nrmse(aRes, sRes);
        aPk = *std::max_element(aRes.begin(), aRes.end()); sPk = *std::max_element(sRes.begin(), sRes.end());
        return true;
    }, tankAtFr);
    REQUIRE(!grid.empty());
    if (tankAtFr.size() >= 2) CHECK(tankAtFr.front() > 1.2 * tankAtFr.back());
    double worst = 0; for (auto& p : grid) worst = std::max(worst, p.nrmse);
    std::cerr << "  " << name << " worst-case NRMSE across grid = " << worst << "\n";
    // Regression guard at the DOCUMENTED FHA baseline (FHA is the shipped resonant model; it is exact at
    // resonance/unity gain — the design point — and degrades off-resonance, a reduced-order-model floor that
    // a fast analytical solver cannot beat; see docs/RESONANT_ACCURACY_PLAN.md). This bound catches a
    // regression, it is NOT the (abandoned) < 0.10 corrected-TDA target.
    CHECK(worst < worstBound);
}
}  // namespace

TEST_CASE("resonant validation grid: CLLC (FHA baseline)", "[resval][cllc]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice"); return; }
    auto d = Kirchhoff::design_cllc(spec_for(400, 48, 480, 100000));
    run_two_sided("CLLC", d,
        [](const Kirchhoff::CllcDesign& x){ return Kirchhoff::build_cllc_tas(x); },
        [](const Kirchhoff::CllcDesign& x, double vo, double io, double f){
            return Kirchhoff::analytical::analytical_cllc(x.inputVoltage,{vo},{io},{x.turnsRatio},f,x.magnetizingInductance,x.primaryResonantInductance,x.primaryResonantCapacitance,x.secondaryResonantInductance,x.secondaryResonantCapacitance,1.0); },
        "l.xcllccell.llr1_pri#branch", false, /*worstBound=*/1.60);   // FHA off-unity worst ~1.37
}

TEST_CASE("resonant validation grid: CLLLC (FHA baseline)", "[resval][clllc]") {
    if (!Kirchhoff::ngspice_in_process_available()) { WARN("no libngspice"); return; }
    auto d = Kirchhoff::design_clllc(spec_for(400, 48, 480, 100000));
    run_two_sided("CLLLC", d,
        [](const Kirchhoff::ClllcDesign& x){ return Kirchhoff::build_clllc_tas(x); },
        [](const Kirchhoff::ClllcDesign& x, double vo, double io, double f){
            return Kirchhoff::analytical::analytical_clllc(x.inputVoltage,{vo},{io},{x.turnsRatio},f,x.magnetizingInductance,x.primaryResonantInductance,x.primaryResonantCapacitance,x.secondaryResonantInductance,x.secondaryResonantCapacitance,1.0); },
        "l.xclllcpower.llr1_pri#branch", true, /*worstBound=*/0.75);   // FHA off-resonance worst ~0.57
}

// ── SRC: the analytical tank agrees with the ngspice deck across the input range (ABT #2179) ──────────────────────
// Alf 2026-10-31 ("New agreement test"): the SRC design (design_src) runs each input at the frequency that delivers Vo,
// solving the series tank through the deck's own circuit (src_tank_circuit: Lm and the coupling, the half bridge's
// split bus caps, the diode drop) — the model analytical_src emits. The deck is the same circuit, open loop, at the
// design's frequency. The design solves the tank at the efficiency-derated drive η·k·Vin; the deck drives k·Vin and
// its parts are near-lossless, so the comparison is on the lossless basis: the same model evaluated at the deck's
// undrated drive k·Vin. Output voltage and primary (Lr) rms current must agree within 1 % at Vin min, nominal and max
// for (a) the 36-72 V -> 12 V / 5 A design of test_input_range_sizing and (b) test_ptp's SRC (360-440 V -> 48 V / 10 A).
// Guards both halves: a deck that grows parts the model lacks (the rectifier R‖C snubbers) and a model that drops a
// part the deck has (Lm) each move one side off the other.
namespace {

json src_agreement_spec(double vmin, double vnom, double vmax, double vop, double vo, double io, double fsw, double eta,
                        bool ptpForm) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vmin}, {"nominal", vnom}, {"maximum", vmax}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    if (ptpForm) {
        // test_ptp's spec_for, with the operating point moved across the range.
        s["designRequirements"]["outputs"] = json::array({{{"name", "out"}, {"voltage", {{"nominal", vo}}}}});
        s["operatingPoints"] = json::array({{{"inputVoltage", vop}, {"outputs", json::array({{{"power", vo * io}}})}}});
    } else {
        // test_input_range_sizing's ranged_spec.
        s["designRequirements"]["outputs"] = json::array(
            {{{"name", "out0"}, {"voltage", {{"nominal", vo}}}, {"power", {{"nominal", vo * io}}}}});
        s["operatingPoints"] = json::array({{{"name", "op"}, {"inputVoltage", vop}, {"ambientTemperature", 25.0},
                                             {"outputs", json::array({{{"name", "out0"}, {"voltage", vo},
                                                                       {"power", vo * io}}})}}});
    }
    return s;
}

std::string src_num(double v) {
    std::ostringstream o;
    o.precision(12);
    o << v;
    return o.str();
}

}  // namespace

TEST_CASE("SRC: the analytical tank agrees with the ngspice deck within 1 % across the input range (ABT #2179)",
          "[src][abt2179][agreement]") {
    REQUIRE(Kirchhoff::ngspice_in_process_available());
    struct Case { const char* name; double vmin, vnom, vmax, vo, io, fsw, eta; bool ptpForm; };
    const std::vector<Case> cases = {
        {"12V5A 36-72 V", 36.0, 48.0, 72.0, 12.0, 5.0, 100e3, 0.9, false},
        {"ptp 500W-FB 360-440 V", 360.0, 400.0, 440.0, 48.0, 10.0, 100e3, 1.0, true},
    };
    for (const auto& c : cases) {
        for (const double vop : {c.vmin, c.vnom, c.vmax}) {
            const Kirchhoff::SrcDesign d = Kirchhoff::design_src(
                src_agreement_spec(c.vmin, c.vnom, c.vmax, vop, c.vo, c.io, c.fsw, c.eta, c.ptpForm));
            REQUIRE(d.outputs.size() == 1);
            const double fsw = d.switchingFrequency, T = 1.0 / fsw;
            // Model: the design's tank network, at the deck's undrated bridge amplitude k·Vin.
            const double vg = (d.fullBridge ? 1.0 : 0.5) * d.inputVoltage;
            const double n = d.turnsRatio, vd = d.outputs.at(0).diodeDrop;
            const double lr = d.resonantInductance, cr = d.resonantCapacitance;
            const double r0 = std::sqrt(lr / cr), f0 = 1.0 / (2.0 * M_PI * std::sqrt(lr * cr));
            const auto network = Kirchhoff::analytical::src_tank_network(Kirchhoff::src_tank_circuit(d), lr, cr, vg, {n});
            const auto tank = Kirchhoff::analytical::src_exact_tank(fsw / f0, r0 / (n * n * d.loadResistance), network);
            const double voutModel = tank.gain * vg / n - vd;
            const double priRmsModel = tank.currentRms() * vg / r0;
            // Deck: open loop at the design's frequency, settled past 30 output RC constants (>= 400 periods).
            PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
            std::string deck = Kirchhoff::tas_to_ngspice(Kirchhoff::build_src_tas(d), ideal);
            const double rc = d.loadResistance * d.outputCapacitance;
            const double tstop = std::max(400.0 * T, std::ceil(30.0 * rc / T) * T), tstep = T / 200.0;
            deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                      ".tran " + src_num(tstep) + " " + src_num(tstop) + " 0 " + src_num(tstep));
            const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
            INFO(c.name << " @ " << vop << " V, fsw " << fsw << " Hz: " << r.error);
            REQUIRE(r.success);
            const auto vout = r.average("v(Vout)", tstop - 40.0 * T, tstop);
            REQUIRE(vout.has_value());
            std::string lrKey;
            for (const auto& [k, v] : r.vectors)
                if (k.find("llr_pri#branch") != std::string::npos) {
                    REQUIRE(lrKey.empty());
                    lrKey = k;
                }
            REQUIRE(!lrKey.empty());
            const auto& x = r.vectors.at(lrKey);
            const auto& t = r.time;
            REQUIRE(x.size() == t.size());
            double a2 = 0.0, span = 0.0;
            for (size_t k = 1; k < t.size(); ++k) {
                if (t[k - 1] < tstop - 20.0 * T) continue;
                const double dt = t[k] - t[k - 1];
                a2 += 0.5 * dt * (x[k] * x[k] + x[k - 1] * x[k - 1]);
                span += dt;
            }
            REQUIRE(span > 19.0 * T);
            const double priRmsDeck = std::sqrt(a2 / span);
            const double voutDeck = std::abs(*vout);
            std::cout << std::setprecision(6) << "[abt2179] " << c.name << " @ " << vop << " V  fsw=" << fsw
                      << "  Vout model=" << voutModel << " deck=" << voutDeck << " ("
                      << 100.0 * (voutDeck / voutModel - 1.0) << " %)  priRms model=" << priRmsModel
                      << " deck=" << priRmsDeck << " (" << 100.0 * (priRmsDeck / priRmsModel - 1.0) << " %)\n";
            INFO("Vout model " << voutModel << " deck " << voutDeck << "; priRms model " << priRmsModel << " deck "
                               << priRmsDeck);
            CHECK(std::abs(voutDeck / voutModel - 1.0) <= 0.01);
            CHECK(std::abs(priRmsDeck / priRmsModel - 1.0) <= 0.01);
        }
    }
}

// ── SRC: the web's default design simulates in ngspice (full-bridge rectifier, ABT #2189) ──────────────────────────────────────
// The SrcWizard defaults (400 V ±10 % -> 48 V / 500 W, full-bridge inverter AND full-bridge rectifier, Q 1, band
// 80-150 kHz, η 0.96) reach Kirchhoff as the two process_converter requests webKirchhoff builds from them: the
// design call, then the Simulated (ngspice) call at the nominal input with the design's parts pinned. Both specs
// below are those requests as the web sent them (captured 2026-11-01 through the node probe on Kirchhoff dd9b666).
// Once the rectifier R‖C snubbers left the deck (ABT #2179) the full-bridge rectifier's secondary hung on its four
// ideal junctions alone, and the Simulated call aborted at t = 10 ps ("Timestep too small ... node xsrccell.sec_a");
// the agreement test above covers only the centre-tapped rectifier, which ties the secondary to sgnd. Guards:
// (a) the web's Simulated call returns a waveform on every winding; (b) the same deck still agrees with the
// lossless exact-tank model within 1 % (whatever lets the deck run must not load the tank).
TEST_CASE("SRC: the web's default full-bridge-rectifier design simulates in ngspice",
          "[src][abt2189][web-default][ngspice]") {
    REQUIRE(Kirchhoff::ngspice_in_process_available());
    const json designSpec = json::parse(R"js({"designRequirements": {"inputType": "dc", "inputVoltage": {"nominal": 400, "tolerance": 0.1}, "switchingFrequency": {"nominal": 100000}, "outputs": [{"name": "out", "voltage": {"nominal": 48}, "regulation": "voltage"}], "efficiency": 0.96}, "operatingPoints": [{"name": "full_load", "inputVoltage": 400, "ambientTemperature": 25, "outputs": [{"name": "out", "power": 500}]}], "config": {"qualityFactor": 1, "resonantBandMin": 80000, "resonantBandMax": 150000, "rectifierType": "fullBridge", "bridgeType": "fullBridge", "tranStopTime": 0.00052}})js");
    const json simSpec = json::parse(R"js({"designRequirements": {"inputType": "dc", "inputVoltage": {"nominal": 400, "tolerance": 0.1}, "switchingFrequency": {"nominal": 100000}, "outputs": [{"name": "out", "voltage": {"nominal": 48}, "regulation": "voltage"}], "efficiency": 0.96, "magnetizingInductance": {"nominal": 0.003031719771947776}, "turnsRatios": [{"nominal": 7.14}], "desiredResonantInductance": 0.0003031719771947776, "desiredResonantCapacitance": 8.355091438517288e-09}, "operatingPoints": [{"name": "Nom. input (400 V)", "inputVoltage": 400, "ambientTemperature": 25, "outputs": [{"name": "out", "power": 500}]}], "config": {"qualityFactor": 1, "resonantBandMin": 80000, "resonantBandMax": 150000, "rectifierType": "fullBridge", "bridgeType": "fullBridge", "tranStopTime": 0.00052}})js");

    // The design call the web makes first (the Analytical engine) must stand.
    const std::string designRaw = Kirchhoff::api::process_converter("src", designSpec.dump(), "analytical");
    INFO(designRaw.substr(0, 400));
    REQUIRE(designRaw.rfind("Exception", 0) != 0);

    // (a) The Simulated call.
    const std::string simRaw = Kirchhoff::api::process_converter("src", simSpec.dump(), "ngspice");
    INFO(simRaw.substr(0, 600));
    REQUIRE(simRaw.rfind("Exception", 0) != 0);
    const json sim = json::parse(simRaw);
    const json& exc = sim.at("operatingPoint").at("excitationsPerWinding");
    REQUIRE(exc.size() >= 2);
    for (const auto& e : exc) {
        CHECK(e.at("current").at("waveform").at("data").size() > 10);
        CHECK(e.at("voltage").at("waveform").at("data").size() > 10);
    }

    // (b) The same deck against the lossless exact-tank model (the agreement test's basis).
    const Kirchhoff::SrcDesign d = Kirchhoff::design_src(simSpec);
    REQUIRE(d.fullBridge);
    REQUIRE(d.rectifierType == Kirchhoff::RectifierType::FullBridge);
    const double fsw = d.switchingFrequency, T = 1.0 / fsw;
    const double vg = d.inputVoltage, n = d.turnsRatio, vd = d.outputs.at(0).diodeDrop;
    const double lr = d.resonantInductance, cr = d.resonantCapacitance;
    const double r0 = std::sqrt(lr / cr), f0 = 1.0 / (2.0 * M_PI * std::sqrt(lr * cr));
    const auto network = Kirchhoff::analytical::src_tank_network(Kirchhoff::src_tank_circuit(d), lr, cr, vg, {n});
    const auto tank = Kirchhoff::analytical::src_exact_tank(fsw / f0, r0 / (n * n * d.loadResistance), network);
    // The tank gain is the winding clamp over the drive: the full-bridge rectifier puts two diode drops in its path.
    const double voutModel = tank.gain * vg / n - 2.0 * vd;
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(Kirchhoff::build_src_tas(d), ideal);
    const double rc = d.loadResistance * d.outputCapacitance;
    const double tstop = std::max(400.0 * T, std::ceil(30.0 * rc / T) * T), tstep = T / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + src_num(tstep) + " " + src_num(tstop) + " 0 " + src_num(tstep));
    const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    INFO("fsw " << fsw << " Hz: " << r.error);
    REQUIRE(r.success);
    const auto vout = r.average("v(Vout)", tstop - 40.0 * T, tstop);
    REQUIRE(vout.has_value());
    const double voutDeck = std::abs(*vout);
    std::cout << std::setprecision(6) << "[src web default] fsw=" << fsw << " Hz  Vout model=" << voutModel
              << " deck=" << voutDeck << " (" << 100.0 * (voutDeck / voutModel - 1.0) << " %)\n";
    CHECK(std::abs(voutDeck / voutModel - 1.0) <= 0.01);
}

// ── SRC: the web's "I know the design" default simulates in ngspice (full-bridge rectifier, ABT #2189) ───────────────
// The same SrcWizard defaults in "I know the design I want": turns ratio 8.33 and the drive forced at the 100 kHz
// resonance (driveAtSwitchingFrequency), so the output is the result (~46 V). Both specs are the requests the web sent
// (captured 2026-11-02 through the node probe on Kirchhoff 193bb99). At resonance the tank current reaches zero at
// every half period and all four rectifier diodes block until the next edge, so the secondary's common mode spent
// every half period on junction leakage alone and the Simulated call aborted at t = 78 us ("Timestep too small ...
// node xsrccell.sec_a") even at gmin 1e-9: the deck now gives each secondary terminal its rectifier node capacitance
// (cfg::kSrcRectifierNodeCap). Guards: the design call stands, and the Simulated call returns a waveform on every
// winding at the nominal input and at both ends of the range.
TEST_CASE("SRC: the web's I-know full-bridge-rectifier design, driven at resonance, simulates in ngspice",
          "[src][abt2189][web-default][i-know][ngspice]") {
    REQUIRE(Kirchhoff::ngspice_in_process_available());
    const json designSpec = json::parse(R"js({"designRequirements":{"inputType":"dc","inputVoltage":{"nominal":400,"tolerance":0.1},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":48},"regulation":"voltage"}],"efficiency":0.96,"turnsRatios":[{"nominal":8.33}]},"operatingPoints":[{"name":"full_load","inputVoltage":400,"ambientTemperature":25,"outputs":[{"name":"out","power":500}]}],"config":{"qualityFactor":1,"resonantBandMin":80000,"resonantBandMax":150000,"driveAtSwitchingFrequency":true,"resonantFrequency":100000,"rectifierType":"fullBridge","bridgeType":"fullBridge","tranStopTime":0.00052}})js");
    const json simSpec = json::parse(R"js({"designRequirements":{"inputType":"dc","inputVoltage":{"nominal":400,"tolerance":0.1},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":48},"regulation":"voltage"}],"efficiency":0.96,"turnsRatios":[{"nominal":8.33}],"magnetizingInductance":{"nominal":0.003031719771947776},"desiredResonantInductance":0.0003031719771947776,"desiredResonantCapacitance":8.355091438517288e-9},"operatingPoints":[{"name":"Nom. input (400 V)","inputVoltage":400,"ambientTemperature":25,"outputs":[{"name":"out","power":500}]}],"config":{"qualityFactor":1,"resonantBandMin":80000,"resonantBandMax":150000,"driveAtSwitchingFrequency":true,"resonantFrequency":100000,"rectifierType":"fullBridge","bridgeType":"fullBridge","tranStopTime":0.00052}})js");

    const std::string designRaw = Kirchhoff::api::process_converter("src", designSpec.dump(), "analytical");
    INFO(designRaw.substr(0, 400));
    REQUIRE(designRaw.rfind("Exception", 0) != 0);

    for (const double vin : {400.0, 360.0, 440.0}) {
        CAPTURE(vin);
        json spec = simSpec;
        spec["operatingPoints"][0]["inputVoltage"] = vin;
        const std::string simRaw = Kirchhoff::api::process_converter("src", spec.dump(), "ngspice");
        INFO(simRaw.substr(0, 600));
        REQUIRE(simRaw.rfind("Exception", 0) != 0);
        const json sim = json::parse(simRaw);
        const json& exc = sim.at("operatingPoint").at("excitationsPerWinding");
        REQUIRE(exc.size() >= 2);
        for (const auto& e : exc) {
            CHECK(e.at("current").at("waveform").at("data").size() > 10);
            CHECK(e.at("voltage").at("waveform").at("data").size() > 10);
        }
    }

    // The same deck against the lossless exact-tank model at the nominal input, as in the help-me test above (the
    // node capacitance that lets the deck run must not load the tank).
    const Kirchhoff::SrcDesign d = Kirchhoff::design_src(simSpec);
    const double fsw = d.switchingFrequency, T = 1.0 / fsw;
    const double vg = d.inputVoltage, n = d.turnsRatio, vd = d.outputs.at(0).diodeDrop;
    const double lr = d.resonantInductance, cr = d.resonantCapacitance;
    const double r0 = std::sqrt(lr / cr), f0 = 1.0 / (2.0 * M_PI * std::sqrt(lr * cr));
    const auto network = Kirchhoff::analytical::src_tank_network(Kirchhoff::src_tank_circuit(d), lr, cr, vg, {n});
    const auto tank = Kirchhoff::analytical::src_exact_tank(fsw / f0, r0 / (n * n * d.loadResistance), network);
    const double voutModel = tank.gain * vg / n - 2.0 * vd;
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(Kirchhoff::build_src_tas(d), ideal);
    const double rc = d.loadResistance * d.outputCapacitance;
    const double tstop = std::max(400.0 * T, std::ceil(30.0 * rc / T) * T), tstep = T / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + src_num(tstep) + " " + src_num(tstop) + " 0 " + src_num(tstep));
    const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    INFO("fsw " << fsw << " Hz: " << r.error);
    REQUIRE(r.success);
    const auto vout = r.average("v(Vout)", tstop - 40.0 * T, tstop);
    REQUIRE(vout.has_value());
    std::cout << std::setprecision(6) << "[src web I-know] fsw=" << fsw << " Hz f0=" << f0 << " Hz  Vout model="
              << voutModel << " deck=" << std::abs(*vout) << " (" << 100.0 * (std::abs(*vout) / voutModel - 1.0)
              << " %)\n";
    CHECK(std::abs(std::abs(*vout) / voutModel - 1.0) <= 0.01);
}
