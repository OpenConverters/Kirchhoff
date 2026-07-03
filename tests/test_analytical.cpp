// Validate the ANALYTICAL run engine (src/Analytical.cpp, MKF-migration item 3): the simulator-free
// operating-point prediction from an assembled TAS doc. Two kinds of check:
//  1. Self-consistency for many topologies — Vout = target, Iout = P/Vout, Pin·η = Pout, and per-winding
//     stresses present with positive RMS currents (no ngspice run at all).
//  2. Cross-validation vs SPICE — for a couple of topologies, the analytically-predicted Vout matches
//     the Vout the deck actually settles to (run here through the in-process libngspice runner when
//     available, else skipped), proving the prediction is physically sound.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "Analytical.hpp"
#include "NgspiceRunner.hpp"
#include "Topology.hpp"      // the single MAS::Topology enum (Kirchhoff::Topology names it)
#include <type_traits>

#include <cmath>
#include <functional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

using nlohmann::json;

// Kirchhoff::Topology IS the single generated MAS::Topology enum (a name in the Kirchhoff namespace,
// not a second type). The enum is used exactly as quicktype emits it — no post-processing.
static_assert(std::is_same<Kirchhoff::Topology, MAS::Topology>::value,
              "Kirchhoff::Topology must be the single MAS::Topology enum");

namespace {

std::string num(double v) { std::ostringstream o; o.precision(12); o << v; return o.str(); }

// A generic single-output DC spec.
json spec_for(double vin, double vout, double power, double fsw = 100000.0) {
    json s;
    s["designRequirements"]["efficiency"] = 1.0;
    s["designRequirements"]["inputVoltage"] = {{"minimum",vin*0.9},{"nominal",vin},{"maximum",vin*1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name","out"},{"voltage",{{"nominal",vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage",vin},{"outputs",json::array({{{"power",power}}})}} });
    return s;
}

struct Case { std::string name; std::function<json()> tas; double vout; };

// Each entry designs + assembles one topology and reports its expected output voltage.
std::vector<Case> all_cases() {
    using namespace Kirchhoff;
    return {
        {"buck",        []{ return build_buck_tas(design_buck(spec_for(24,12,24))); }, 12},
        {"boost",       []{ return build_boost_tas(design_boost(spec_for(12,24,24))); }, 24},
        {"flyback",     []{ return build_flyback_tas(design_flyback(spec_for(48,12,24))); }, 12},
        {"forward",     []{ return build_forward_tas(design_forward(spec_for(48,12,24))); }, 12},
        {"two_switch",  []{ return build_two_switch_forward_tas(design_two_switch_forward(spec_for(48,12,24))); }, 12},
        {"push_pull",   []{ return build_push_pull_tas(design_push_pull(spec_for(48,12,24))); }, 12},
        {"sepic",       []{ return build_sepic_tas(design_sepic(spec_for(12,12,12))); }, 12},
        {"cuk",         []{ return build_cuk_tas(design_cuk(spec_for(12,12,12))); }, 12},
        {"zeta",        []{ return build_zeta_tas(design_zeta(spec_for(12,12,12))); }, 12},
        {"fsbb",        []{ return build_fsbb_tas(design_fsbb(spec_for(12,12,12))); }, 12},
        {"psfb",        []{ return build_psfb_tas(design_psfb(spec_for(48,12,24))); }, 12},
        {"ahb",         []{ return build_ahb_tas(design_ahb(spec_for(48,12,24))); }, 12},
        {"acf",         []{ return build_acf_tas(design_acf(spec_for(48,12,24))); }, 12},
        {"llc",         []{ return build_llc_tas(design_llc(spec_for(400,48,240))); }, 48},
        {"weinberg",    []{ return build_weinberg_tas(design_weinberg(spec_for(48,12,24))); }, 12},
    };
}

} // namespace

TEST_CASE("analytical operating point is self-consistent across topologies", "[analytical]") {
    for (const auto& c : all_cases()) {
        INFO("topology: " << c.name);
        json tas = c.tas();
        Kirchhoff::AnalyticalOperatingPoint op = Kirchhoff::analytical_operating_point(tas);

        // Output voltage magnitude = the design target. (The engine reports SIGNED voltage; inverting
        // topologies like Ćuk correctly come back negative, so compare magnitudes.)
        CHECK(std::abs(op.outputVoltage) == Catch::Approx(c.vout).epsilon(0.001));
        // Iout = Pout / Vout.
        CHECK(op.outputCurrent == Catch::Approx(op.outputPower / op.outputVoltage).epsilon(1e-9));
        // Power balance: Pin · η = Pout.
        CHECK(op.inputPower * op.efficiency == Catch::Approx(op.outputPower).epsilon(1e-9));
        CHECK(op.switchingFrequency > 0.0);
        // At least one magnetic winding, and every winding carries a positive RMS current.
        CHECK_FALSE(op.windings.empty());
        for (const auto& w : op.windings) {
            INFO("winding " << w.component << "[" << w.winding << "]");
            CHECK(w.currentRms > 0.0);
            CHECK(w.currentPeak >= w.currentRms * 0.99);   // peak >= rms for any real waveform
        }
    }
}

TEST_CASE("analytical operating point throws on a malformed doc", "[analytical]") {
    CHECK_THROWS(Kirchhoff::analytical_operating_point(json::object()));
}

TEST_CASE("analytical Vout matches the SPICE deck (in-process)", "[analytical][ngspice]") {
    if (!Kirchhoff::ngspice_in_process_available()) {
        WARN("Kirchhoff built without libngspice — skipping analytical-vs-SPICE cross-check");
        return;
    }
    struct X { std::string name; std::function<json()> tas; double rc_cap; double rload; double vin; };
    // boost + buck: design, predict analytically, then settle the deck in-process and compare.
    std::vector<X> xs = {
        {"boost", []{ return Kirchhoff::build_boost_tas(Kirchhoff::design_boost(spec_for(12,24,24))); }, 0,0,12},
        {"buck",  []{ return Kirchhoff::build_buck_tas(Kirchhoff::design_buck(spec_for(24,12,24))); }, 0,0,24},
    };
    for (auto& x : xs) {
        INFO("topology: " << x.name);
        json tas = x.tas();
        Kirchhoff::AnalyticalOperatingPoint op = Kirchhoff::analytical_operating_point(tas);

        PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
        std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
        const double period = 1.0 / op.switchingFrequency;
        // Settle to ~30 RC. Pull Rload·Cout from the deck is overkill; use a generous fixed window.
        const double tstop = std::max(400.0 * period, 0.02);
        const double tstep = period / 200.0;
        deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                  ".tran " + num(tstep) + " " + num(tstop) + " 0 " + num(tstep));
        Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
        REQUIRE(r.success);
        auto spiceVout = r.average("v(Vout)", tstop - period, tstop);
        REQUIRE(spiceVout.has_value());
        INFO("analytical Vout=" << op.outputVoltage << "  SPICE Vout=" << *spiceVout);
        CHECK(*spiceVout == Catch::Approx(op.outputVoltage).epsilon(0.05));
    }
}

// ABT #80 — flyback conduction-mode variant: CCM (default) / DCM / BCM / QRM size the magnetizing
// inductance at or below the CCM–DCM boundary. DCM = 0.6·critical, BCM/QRM = critical.
TEST_CASE("design_flyback conduction modes size L at/below the critical boundary", "[analytical][design][flyback]") {
    using Kirchhoff::design_flyback;
    auto withMode = [](const char* m) { json s = spec_for(48, 12, 24); s["config"]["mode"] = m; return design_flyback(s); };
    auto dcm = withMode("dcm");
    auto bcm = withMode("bcm");
    auto qrm = withMode("qrm");
    // DCM sits solidly below the boundary; DCM = 0.6·BCM by construction.
    CHECK(dcm.magnetizingInductance < bcm.magnetizingInductance);
    CHECK(dcm.magnetizingInductance == Catch::Approx(0.6 * bcm.magnetizingInductance).epsilon(1e-6));
    // QRM shares BCM's boundary inductance at the design point (valley switching is a control refinement).
    CHECK(qrm.magnetizingInductance == Catch::Approx(bcm.magnetizingInductance).epsilon(1e-9));
    // Non-CCM duty follows the energy-balance law and stays a valid (0,1) duty.
    CHECK(dcm.dutyCycle > 0.0);
    CHECK(dcm.dutyCycle < 1.0);
    // CCM (default, no config.mode) is a distinct, larger inductance (continuous conduction).
    auto ccm = design_flyback(spec_for(48, 12, 24));
    CHECK(ccm.magnetizingInductance != Catch::Approx(bcm.magnetizingInductance));
    // Unknown mode fails loud.
    { json s = spec_for(48, 12, 24); s["config"]["mode"] = "bogus"; CHECK_THROWS(design_flyback(s)); }
}
