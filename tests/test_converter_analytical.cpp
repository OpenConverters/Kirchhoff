// Validate the per-topology analytical solvers (src/ConverterAnalytical.cpp) — Phase 2 of the MKF
// analytical-converter-solver port. Checks the computed winding excitation (current/voltage waveforms +
// processed stresses) against the closed-form expectations.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "ConverterAnalytical.hpp"
#include "Buck.hpp"    // design_buck — ABT #95 maximumSwitchCurrent sizing tests
#include "Boost.hpp"   // design_boost
#include "Zeta.hpp"    // design_zeta
#include "Pshb.hpp"
#include "PushPull.hpp" // design_push_pull — pinned turns-ratio feasibility    // design_pshb / build_pshb_tas — CURRENT_DOUBLER output-inductor split
#include "KirchhoffApi.hpp"  // design_tas_full — ABT #102 AHB RMS repro

#include <nlohmann/json.hpp>
#include <cmath>
#include <cctype>
#include <sstream>
#include <vector>
#include <functional>

using Kirchhoff::analytical::analytical_buck;

// Convenience: processed-current / processed-voltage accessors for a winding (by value —
// get_current()/get_processed() return std::optional by value).
static MAS::ProcessedWaveform processed_current(const MAS::OperatingPoint& op, size_t winding) {
    return *op.get_excitations_per_winding().at(winding).get_current()->get_processed();
}
static double voltage_average(const MAS::OperatingPoint& op, size_t winding) {
    return *op.get_excitations_per_winding().at(winding).get_voltage()->get_processed()->get_average();
}
static MAS::ProcessedWaveform processed_voltage(const MAS::OperatingPoint& op, size_t winding) {
    return *op.get_excitations_per_winding().at(winding).get_voltage()->get_processed();
}
static MAS::ProcessedWaveform processed_magnetizing(const MAS::OperatingPoint& op, size_t winding) {
    return *op.get_excitations_per_winding().at(winding).get_magnetizing_current()->get_processed();
}

TEST_CASE("analytical_buck CCM inductor excitation matches closed form", "[analytical][solver][buck]") {
    // 12 V -> 5 V, 2 A, 100 kHz, L = 10 uH (ideal: Vd=0, eta=1).
    const double vin = 12, vout = 5, iout = 2, fsw = 100000, L = 10e-6;
    MAS::OperatingPoint op = analytical_buck(vin, vout, iout, fsw, L);

    REQUIRE(op.get_excitations_per_winding().size() == 1);
    const auto& exc = op.get_excitations_per_winding()[0];
    REQUIRE(exc.get_current().has_value());
    REQUIRE(exc.get_current()->get_processed().has_value());
    const auto& cur = *exc.get_current()->get_processed();

    // Closed form: D = Vout/Vin = 0.4167; tOn = D/fsw; ripple ΔIL = (Vin-Vout)·tOn/L.
    const double D = vout / vin;
    const double ripple = (vin - vout) * (D / fsw) / L;          // ~2.917 A
    const double peak = iout + ripple / 2.0;                      // ~3.458 A
    const double rms = std::sqrt(iout * iout + ripple * ripple / 12.0);  // triangle RMS ~2.17 A

    REQUIRE(cur.get_average().has_value());
    CHECK(*cur.get_average() == Catch::Approx(iout).margin(0.02));      // inductor avg = load current
    CHECK(*cur.get_peak() == Catch::Approx(peak).margin(0.05));
    CHECK(*cur.get_rms() == Catch::Approx(rms).margin(0.03));
    CHECK(*cur.get_peak_to_peak() == Catch::Approx(ripple).margin(0.05));

    // Voltage excitation present; the inductor sees +/- with zero average (volt-second balance).
    REQUIRE(exc.get_voltage().has_value());
    REQUIRE(exc.get_voltage()->get_processed().has_value());
    CHECK(*exc.get_voltage()->get_processed()->get_average() == Catch::Approx(0.0).margin(0.1));
}

TEST_CASE("analytical_buck enters DCM at light load", "[analytical][solver][buck]") {
    // Light load + small L -> DCM (minimum inductor current would go negative).
    const double vin = 12, vout = 5, iout = 0.1, fsw = 100000, L = 47e-6;
    MAS::OperatingPoint op = analytical_buck(vin, vout, iout, fsw, L);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    const auto& cur = *op.get_excitations_per_winding()[0].get_current()->get_processed();
    // In DCM the current returns to zero each cycle: average stays ~the load current, min ~0.
    REQUIRE(cur.get_average().has_value());
    CHECK(*cur.get_average() == Catch::Approx(iout).margin(0.03));
    REQUIRE(cur.get_negative_peak().has_value());
    CHECK(*cur.get_negative_peak() >= -0.05);   // does not go meaningfully negative (discontinuous)
}

TEST_CASE("analytical_buck rejects Vout >= Vin (duty >= 1)", "[analytical][solver][buck]") {
    CHECK_THROWS(analytical_buck(12, 12, 2, 100000, 10e-6));
}

TEST_CASE("analytical_boost CCM inductor excitation matches closed form", "[analytical][solver][boost]") {
    using Kirchhoff::analytical::analytical_boost;
    // 12 V -> 24 V, 1 A out, 100 kHz, L = 20 uH (ideal). D = 1 - Vin/Vout = 0.5.
    const double vin = 12, vout = 24, iout = 1, fsw = 100000, L = 20e-6;
    MAS::OperatingPoint op = analytical_boost(vin, vout, iout, fsw, L);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    const auto& cur = *op.get_excitations_per_winding()[0].get_current()->get_processed();

    const double D = 1 - vin / vout;                            // 0.5
    const double ripple = vin * (D / fsw) / L;                  // 3 A
    const double iAvg = iout * vout / vin;                      // input current 2 A
    const double peak = iAvg + ripple / 2.0;                    // 3.5 A
    const double rms = std::sqrt(iAvg * iAvg + ripple * ripple / 12.0);  // ~2.18 A

    CHECK(*cur.get_average() == Catch::Approx(iAvg).margin(0.03));   // INPUT current, not load
    CHECK(*cur.get_peak() == Catch::Approx(peak).margin(0.05));
    CHECK(*cur.get_rms() == Catch::Approx(rms).margin(0.03));
    CHECK(*cur.get_peak_to_peak() == Catch::Approx(ripple).margin(0.05));
}

TEST_CASE("analytical_boost rejects Vin >= Vout (duty <= 0)", "[analytical][solver][boost]") {
    using Kirchhoff::analytical::analytical_boost;
    CHECK_THROWS(analytical_boost(24, 12, 1, 100000, 20e-6));
}

// ─── Phase 3: PWM converter family ──────────────────────────────────────────

TEST_CASE("analytical_flyback CCM: primary=input current, secondary=load current", "[analytical][solver][flyback]") {
    using Kirchhoff::analytical::analytical_flyback;
    // 48 V -> 12 V, 2 A, 100 kHz, n=Np/Ns=2, Lp=200 uH (>> Lcrit ~53 uH -> CCM).
    const double vin = 48, vout = 12, iout = 2, fsw = 100000, n = 2, Lp = 200e-6;
    MAS::OperatingPoint op = analytical_flyback(vin, {vout}, {iout}, {n}, fsw, Lp);

    REQUIRE(op.get_excitations_per_winding().size() == 2);          // Primary + Secondary 0
    const double D = n * vout / (n * vout + vin);                   // 0.3333
    // Primary winding conducts only during D: <i_pri> = reflected input current = Iout*Vout/Vin.
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(iout * vout / vin).margin(0.05));   // 0.5 A
    // Secondary winding integrates to the load current. MAS excitation convention (2026-09-24): the secondary
    // current is SOURCE-referenced to the dotted terminal and a flyback rectifier conducts while the dot-
    // reference secondary voltage is negative, so the delivered current is NEGATIVE (was +2 A before the
    // convention; the magnitude is unchanged).
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-iout).margin(0.1));                // -2 A
    // Inductor volt-second balance on both windings.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.5));
    (void)D;
}

TEST_CASE("analytical_flyback DCM preserves load current", "[analytical][solver][flyback]") {
    using Kirchhoff::analytical::analytical_flyback;
    // Small Lp -> DCM (Lp=10 uH < Lcrit).
    MAS::OperatingPoint op = analytical_flyback(48, {12}, {2}, {2}, 100000, 10e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    // |secondary avg| = Iout; negative in the MAS source convention (see the CCM case above).
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-2.0).margin(0.15));
}

TEST_CASE("analytical_flyback rejects mismatched vector sizes", "[analytical][solver][flyback]") {
    using Kirchhoff::analytical::analytical_flyback;
    CHECK_THROWS(analytical_flyback(48, {12}, {2, 1}, {2}, 100000, 200e-6));
}

TEST_CASE("analytical_forward CCM: 3 windings, secondary peak = Iout(1+ripple/2)", "[analytical][solver][forward]") {
    using Kirchhoff::analytical::analytical_forward;
    // 48 V -> 5 V, 10 A, 100 kHz, turnsRatios=[demag=1, sec=4], Lmag=1 mH, Lout=10 uH, ripple=0.3.
    const double vin = 48, vout = 5, iout = 10, fsw = 100000, ripple = 0.3;
    MAS::OperatingPoint op = analytical_forward(vin, {vout}, {iout}, {1, 4}, fsw, 1e-3, 10e-6, ripple);

    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Primary, Demag, Secondary 0
    // Secondary winding current peak = max output-inductor current = Iout + ripple*Iout/2.
    CHECK(*processed_current(op, 2).get_peak() == Catch::Approx(iout * (1 + ripple / 2)).margin(0.3));   // 11.5 A
    // Primary & demag windings see zero-average (reset) voltage.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.6));
    CHECK(voltage_average(op, 1) == Catch::Approx(0.0).margin(0.6));
}

TEST_CASE("analytical_forward rejects t_on > T/2", "[analytical][solver][forward]") {
    using Kirchhoff::analytical::analytical_forward;
    // nSec=8 -> required t1 = 0.83*T > T/2.
    CHECK_THROWS(analytical_forward(48, {5}, {10}, {1, 8}, 100000, 1e-3, 10e-6, 0.3));
}

TEST_CASE("analytical_two_switch_forward CCM: 2 windings, secondary peak", "[analytical][solver][twoswitchforward]") {
    using Kirchhoff::analytical::analytical_two_switch_forward;
    const double vin = 48, vout = 5, iout = 10, fsw = 100000, ripple = 0.3;
    MAS::OperatingPoint op = analytical_two_switch_forward(vin, {vout}, {iout}, {4}, fsw, 1e-3, 10e-6, ripple);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0
    CHECK(*processed_current(op, 1).get_peak() == Catch::Approx(iout * (1 + ripple / 2)).margin(0.3));   // 11.5 A
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.6));
}

TEST_CASE("analytical_two_switch_forward rejects t_on > T/2", "[analytical][solver][twoswitchforward]") {
    using Kirchhoff::analytical::analytical_two_switch_forward;
    CHECK_THROWS(analytical_two_switch_forward(48, {5}, {10}, {8}, 100000, 1e-3, 10e-6, 0.3));
}

TEST_CASE("analytical_push_pull CCM: 4 windings, symmetric halves, primary peak", "[analytical][solver][pushpull]") {
    using Kirchhoff::analytical::analytical_push_pull;
    // 24 V -> 5 V, 10 A, 100 kHz, n(sec)=4, Lmag=1 mH, Lout=10 uH, ripple=0.3.
    const double vin = 24, vout = 5, iout = 10, fsw = 100000, n = 4, ripple = 0.3;
    MAS::OperatingPoint op = analytical_push_pull(vin, vout, iout, fsw, n, 1e-3, 10e-6, ripple);

    REQUIRE(op.get_excitations_per_winding().size() == 4);   // 2 primary halves + 2 secondary halves
    // Closed form: t1 = (T/2)*Vout/(Vin/n); magCurrent = Vin*t1/Lmag.
    const double period = 1.0 / fsw;
    const double t1 = period / 2 * vout / (vin / n);
    const double magCurrent = vin * t1 / 1e-3;
    const double maxSec = iout + ripple * iout / 2;            // 11.5
    const double maxPri = maxSec / n + magCurrent / 2;         // primary peak
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(maxPri).margin(0.2));
    // The two primary halves are mirror images -> equal RMS.
    CHECK(*processed_current(op, 0).get_rms() == Catch::Approx(*processed_current(op, 1).get_rms()).margin(0.05));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.4));
}

TEST_CASE("analytical_push_pull rejects t_on > T/2", "[analytical][solver][pushpull]") {
    using Kirchhoff::analytical::analytical_push_pull;
    // n=6 -> t1 = (T/2)*1.25 > T/2.
    CHECK_THROWS(analytical_push_pull(24, 5, 10, 100000, 6, 1e-3, 10e-6, 0.3));
}

TEST_CASE("analytical_weinberg boost regime: 6 windings, input-current magnitude", "[analytical][solver][weinberg]") {
    using Kirchhoff::analytical::analytical_weinberg;
    // 24 V -> 72 V (M=3, boost regime D=0.833), 2 A, 100 kHz, L1=50 uH, n=1. Weinberg has TWO magnetics,
    // so the solver emits all 6 windings: [L1a, L1b, T1_pri_a, T1_pri_b, T1_sec_a, T1_sec_b]. Power balance
    // gives Iin = Iout*M = 6 A (the earlier Iout/M = 0.667 was an INVERTED-magnitude bug — this test used
    // to pin it). The current-fed front end splits Iin/2 per L1 / push-pull-primary winding (avg ~3).
    const double vin = 24, vout = 72, iout = 2, fsw = 100000, L1 = 50e-6, n = 1;
    MAS::OperatingPoint op = analytical_weinberg(vin, vout, iout, fsw, L1, n);

    REQUIRE(op.get_excitations_per_winding().size() == 6);
    const double M = vout / vin;                              // 3
    const double D = 1.0 - 1.0 / (2.0 * n * M);               // 0.8333
    const double inputCurrent = iout * M;                     // 6  (Iin = Iout*M, power balance)
    // All four primary-side windings carry iL/2 during the overlaps and the whole iL during their own single
    // conduction: avg magnitude Iin/2, peak = Iin + ΔI/2 with the L1 ripple ΔI = Vin·tOv/L1 (tOv = (2D−1)·T/2)
    // — 6.8 A here. (Before 2026-09-24 the pulses ramped from 0 across each overlap and this pinned a peak of
    // ~Iin; that shape broke the T1 ampere-turn balance, see test_winding_convention [convention].)
    const double tOv = (2.0 * D - 1.0) / (2.0 * fsw);
    const double peak = inputCurrent + vin * tOv / L1 / 2.0;
    for (size_t w = 0; w < 4; ++w) {
        CHECK(std::abs(*processed_current(op, w).get_average()) == Catch::Approx(inputCurrent / 2.0).margin(0.6));
        CHECK(*processed_current(op, w).get_peak() == Catch::Approx(peak).margin(0.1));
    }
    // L1's two windings share sense (both +Iin/2); the T1 push-pull halves are opposite-wound, so their
    // DC offsets take OPPOSITE sign — net transformer DC-MMF ~0 (both primary halves AND both secondary
    // halves cancel), as measured on the ngspice deck. This is the physical push-pull flux balance.
    CHECK(*processed_current(op, 0).get_average() > 0.0);                                   // L1a +
    CHECK(*processed_current(op, 1).get_average() > 0.0);                                   // L1b +
    CHECK(*processed_current(op, 2).get_average() * *processed_current(op, 3).get_average() < 0.0);  // T1 pri opposite
    CHECK(*processed_current(op, 4).get_average() * *processed_current(op, 5).get_average() < 0.0);  // T1 sec opposite
    CHECK((*processed_current(op, 2).get_average() + *processed_current(op, 3).get_average())
          == Catch::Approx(0.0).margin(0.2));                                              // net primary DC-MMF ~0
    CHECK((*processed_current(op, 4).get_average() + *processed_current(op, 5).get_average())
          == Catch::Approx(0.0).margin(0.2));                                              // net secondary DC-MMF ~0
    // Secondary halves conduct only during single-conduction (1-D)*T: |avg| ~ Iin*n*(1-D).
    CHECK(std::abs(*processed_current(op, 4).get_average())
          == Catch::Approx(inputCurrent * n * (1.0 - D)).margin(0.3));
    CHECK(voltage_average(op, 2) == Catch::Approx(0.0).margin(0.5));   // T1 primary bipolar rectangular
}

TEST_CASE("analytical_sepic: 1 winding, IL1avg and zero-mean voltage", "[analytical][solver][sepic]") {
    using Kirchhoff::analytical::analytical_sepic;
    // 12 V -> 12 V (D=0.5), 1 A, 100 kHz, L1=47 uH.
    MAS::OperatingPoint op = analytical_sepic(12, 12, 1, 100000, 47e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    const double D = 0.5, IL1avg = 1.0 * D / (1.0 - D);   // 1.0
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(IL1avg).margin(0.05));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));
}

TEST_CASE("analytical_cuk: 1 winding, IL1avg, VC1=Vin/(1-D) swing", "[analytical][solver][cuk]") {
    using Kirchhoff::analytical::analytical_cuk;
    MAS::OperatingPoint op = analytical_cuk(12, 12, 1, 100000, 47e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(1.0).margin(0.05));   // IL1avg
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.5));
    // pp voltage = VC1 = Vin/(1-D) = 24 -> half-amplitude ~ sqrt of swing; just confirm peak-to-peak.
    CHECK(*op.get_excitations_per_winding()[0].get_voltage()->get_processed()->get_peak_to_peak() == Catch::Approx(24.0).margin(1.0));
}

TEST_CASE("analytical_zeta: 1 winding, IL1avg", "[analytical][solver][zeta]") {
    using Kirchhoff::analytical::analytical_zeta;
    MAS::OperatingPoint op = analytical_zeta(12, 12, 1, 100000, 47e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(1.0).margin(0.05));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));
}

TEST_CASE("analytical_fsbb buck region: inductor avg = Iout", "[analytical][solver][fsbb]") {
    using Kirchhoff::analytical::analytical_fsbb;
    // 12 V -> 5 V (buck), 2 A, 100 kHz, L=10 uH.
    MAS::OperatingPoint op = analytical_fsbb(12, 5, 2, 100000, 10e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 1);   // "Inductor"
    const double D = 5.0 / 12.0;
    const double dIL = (12 - 5) * (D / 100000) / 10e-6;       // 2.917
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(2.0).margin(0.05));
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(2.0 + dIL / 2).margin(0.1));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.2));
}

TEST_CASE("analytical_fsbb boost region: inductor avg = Iout/(1-D)", "[analytical][solver][fsbb]") {
    using Kirchhoff::analytical::analytical_fsbb;
    // 12 V -> 24 V (boost, D=0.5), 1 A, 100 kHz, L=20 uH.
    MAS::OperatingPoint op = analytical_fsbb(12, 24, 1, 100000, 20e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(2.0).margin(0.05));   // Iout/(1-D)=2
}

TEST_CASE("analytical_fsbb SIMULTANEOUS: regular at Vo==Vin (buck-boost mode)", "[analytical][solver][fsbb]") {
    using Kirchhoff::analytical::analytical_fsbb;
    using Kirchhoff::analytical::FsbbMode;
    // 24 V -> 24 V, 5 A, 100 kHz, L=20 uH — the transition point that BUCK_BOOST_AUTO throws on.
    // Simultaneous mode: D = Vo/(Vin+Vo) = 0.5, iL_avg = Iout/(1-D) = 10 A, ΔiL = Vin*D*T/L.
    MAS::OperatingPoint op = analytical_fsbb(24, 24, 5, 100000, 20e-6, 1.0, FsbbMode::SIMULTANEOUS);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    const double D = 0.5, dIL = 24.0 * (D / 100000) / 20e-6;   // = 6.0 A pk-pk
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(10.0).margin(0.1));   // Iout/(1-D)
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(10.0 + dIL / 2).margin(0.2));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));                     // volt-second balance
    // Voltage swings +Vin (charge) / -Vo (discharge): peak-to-peak = Vin+Vo = 48.
    CHECK(*processed_voltage(op, 0).get_peak_to_peak() == Catch::Approx(48.0).margin(1.0));
    // And it must NOT throw at the transition the AUTO mode rejects.
    CHECK_NOTHROW(analytical_fsbb(24, 24, 5, 100000, 20e-6, 1.0, FsbbMode::SIMULTANEOUS));
}

TEST_CASE("analytical_fsbb AUTO routes the transition band to SIMULTANEOUS", "[analytical][solver][fsbb]") {
    using Kirchhoff::analytical::analytical_fsbb;
    using Kirchhoff::analytical::FsbbMode;
    // Vo == Vin: no pure buck/boost duty exists — AUTO now produces the SIMULTANEOUS waveform
    // (previously threw "transition region, not ported").
    MAS::OperatingPoint atEqual = analytical_fsbb(24, 24, 5, 100000, 20e-6);
    MAS::OperatingPoint simul   = analytical_fsbb(24, 24, 5, 100000, 20e-6, 1.0, FsbbMode::SIMULTANEOUS);
    CHECK(*processed_current(atEqual, 0).get_average() ==
          Catch::Approx(*processed_current(simul, 0).get_average()).margin(1e-9));
    CHECK(*processed_current(atEqual, 0).get_peak_to_peak() ==
          Catch::Approx(*processed_current(simul, 0).get_peak_to_peak()).margin(1e-9));
    // Saturated-buck window (the FSBB wizard defaults that hit this live): 13.5 V -> 12 V at
    // eta=0.92 gives D_buck = 12/(13.5*0.92) = 0.966 >= maxDuty — a real 4-switch controller
    // runs its buck-boost region here, so AUTO must produce the SIMULTANEOUS waveform, not throw.
    MAS::OperatingPoint window = analytical_fsbb(13.5, 12, 2, 100000, 47e-6, 0.92);
    const double D = 12.0 / (13.5 + 12.0);
    CHECK(*processed_current(window, 0).get_average() == Catch::Approx(2.0 / (1.0 - D)).margin(0.05));
    CHECK(*processed_voltage(window, 0).get_peak_to_peak() == Catch::Approx(13.5 + 12.0).margin(0.5));
    // Comfortably-buck stays pure buck: avg = Iout, not Iout/(1-D).
    MAS::OperatingPoint buck = analytical_fsbb(18, 12, 2, 100000, 47e-6, 0.92);
    CHECK(*processed_current(buck, 0).get_average() == Catch::Approx(2.0).margin(0.05));
    // Deep boost beyond maximumDutyCycle remains a hard error.
    CHECK_THROWS(analytical_fsbb(1, 24, 1, 100000, 20e-6));
}

TEST_CASE("analytical_fsbb SPLIT_PWM: lower inductor ripple than SIMULTANEOUS, on-target VSB (ABT #94)",
          "[analytical][solver][fsbb][splitpwm]") {
    using Kirchhoff::analytical::analytical_fsbb;
    using Kirchhoff::analytical::FsbbMode;
    // 24 V -> 24 V, 5 A, 100 kHz, L=20 uH — the transition point. SIMULTANEOUS: D=0.5, iL_avg=10 A,
    // ΔiL = Vin*D*T/L = 6 A pk-pk. SPLIT_PWM κ=0.5: t1=κ*D=0.25 (charge), t2=Vo(1-t1)/Vin=0.75, so the
    // +Vin charge interval is halved → ΔiL ≈ Vin*t1*T/L = 3 A pk-pk (half), and iL_avg = Iout/(1-t1)=6.67 A.
    const double Vin = 24, Vo = 24, Io = 5, fsw = 100000, L = 20e-6;
    MAS::OperatingPoint simul = analytical_fsbb(Vin, Vo, Io, fsw, L, 1.0, FsbbMode::SIMULTANEOUS);
    MAS::OperatingPoint split = analytical_fsbb(Vin, Vo, Io, fsw, L, 1.0, FsbbMode::SPLIT_PWM, 0.5);
    const double simulPP = *processed_current(simul, 0).get_peak_to_peak();
    const double splitPP = *processed_current(split, 0).get_peak_to_peak();
    INFO("simul ΔiL=" << simulPP << " A, split ΔiL=" << splitPP << " A");
    CHECK(simulPP == Catch::Approx(6.0).margin(0.2));
    CHECK(splitPP == Catch::Approx(3.0).margin(0.2));            // κ=0.5 halves the charge interval
    CHECK(splitPP < simulPP);                                    // the whole point of split PWM
    CHECK(*processed_current(split, 0).get_average() == Catch::Approx(6.667).margin(0.1));   // Iout/(1-t1)
    CHECK(voltage_average(split, 0) == Catch::Approx(0.0).margin(0.3));                      // volt-second balance
    // κ=1 collapses to SIMULTANEOUS (state 2 vanishes): same ripple as SIMULTANEOUS.
    MAS::OperatingPoint atUnity = analytical_fsbb(Vin, Vo, Io, fsw, L, 1.0, FsbbMode::SPLIT_PWM, 1.0);
    CHECK(*processed_current(atUnity, 0).get_peak_to_peak() == Catch::Approx(simulPP).margin(0.2));
    // Regular at Vo==Vin (no transition singularity), and bad splitRatio throws.
    CHECK_NOTHROW(analytical_fsbb(24, 24, 5, 100000, 20e-6, 1.0, FsbbMode::SPLIT_PWM, 0.5));
    CHECK_THROWS(analytical_fsbb(24, 24, 5, 100000, 20e-6, 1.0, FsbbMode::SPLIT_PWM, 0.0));   // κ out of (0,1]
    CHECK_THROWS(analytical_fsbb(24, 24, 5, 100000, 20e-6, 1.0, FsbbMode::SPLIT_PWM, 1.5));
}

TEST_CASE("analytical_isolated_buck: primary avg=Ipri, secondary avg=Isec", "[analytical][solver][isolatedbuck]") {
    using Kirchhoff::analytical::analytical_isolated_buck;
    // 12 V; primary rail 3.3 V @ 1 A; isolated secondary 5 V @ 0.5 A; n=0.5, L=22 uH.
    MAS::OperatingPoint op = analytical_isolated_buck(12, 3.3, 1.0, 5.0, 0.5, 100000, 22e-6, 0.5);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(1.0).margin(0.05));   // Ipri (KCL)
    // Isec, negative in the MAS source convention (the rectifier conducts while the dot-reference secondary
    // voltage is negative; was +0.5 before the 2026-09-24 convention).
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-0.5).margin(0.05));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));
}

TEST_CASE("analytical_isolated_buck_boost: magnetizing avg = (Ipri+Isec/n)/(1-D), shared in the off-time",
          "[analytical][solver][isolatedbuckboost]") {
    using Kirchhoff::analytical::analytical_isolated_buck_boost;
    // 12 V; primary rail 5 V @ 1 A; isolated secondary 12 V @ 0.5 A; n=0.5, L=22 uH.
    MAS::OperatingPoint op = analytical_isolated_buck_boost(12, 5, 1.0, 12, 0.5, 100000, 22e-6, 0.5);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    const double D = 5.0 / (12.0 + 5.0);                      // 0.294
    const double magAvg = (1.0 + 0.5 / 0.5) / (1.0 - D);      // (1+1)/0.706 = 2.833
    // The primary WINDING carries the whole magnetizing current during t_on and its (1-S) share (its own
    // rail's Ipri) during the off-time: <i_p> = D·<i_m> + Ipri = 1.833 A. (Before 2026-09-24 the model
    // emitted the magnetizing triangle itself as the primary current, 2.833 A, and a secondary that rose
    // during the off-time — ampere-turns not conserved.)
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(D * magAvg + 1.0).margin(0.05));
    // Secondary average = Isec, negative in the MAS source convention.
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-0.5).margin(0.05));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));
}

TEST_CASE("analytical_active_clamp_forward CCM: 2 windings, secondary peak, clamp-balanced primary",
          "[analytical][solver][acf]") {
    using Kirchhoff::analytical::analytical_active_clamp_forward;
    // 48 V -> 5 V, 10 A, 100 kHz, n=4, Lmag=1 mH, Lout=10 uH, ripple=0.3.
    const double vin = 48, vout = 5, iout = 10, fsw = 100000, ripple = 0.3;
    MAS::OperatingPoint op = analytical_active_clamp_forward(vin, {vout}, {iout}, {4}, fsw, 1e-3, 10e-6, ripple);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0
    // Secondary winding current peak = max output-inductor current = Iout + ripple*Iout/2.
    CHECK(*processed_current(op, 1).get_peak() == Catch::Approx(iout * (1 + ripple / 2)).margin(0.3));   // 11.5 A
    // Primary volt-second balance: +Vin during t1, -Vclamp during t2, Vclamp = D/(1-D)*Vin -> zero mean.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.6));
}

TEST_CASE("analytical_active_clamp_forward rejects t1 > T/2", "[analytical][solver][acf]") {
    using Kirchhoff::analytical::analytical_active_clamp_forward;
    // n=8 -> t1 = period*5/(48/8) = 0.83*period > period/2.
    CHECK_THROWS(analytical_active_clamp_forward(48, {5}, {10}, {8}, 100000, 1e-3, 10e-6, 0.3));
}

// ─── Phase 4: phase-shifted bridge family (structural invariants) ───────────
// NOTE: these check the invariants that hold by construction (antisymmetric primary
// => zero-mean V and I; winding counts; throw guards). The secondary-winding current
// FIDELITY (freewheel attribution, ZVS freewheel-tau constants) is NOT asserted here
// pending the independent MKF-fidelity review + the ngspice NRMSE cross-check.

TEST_CASE("analytical_psfb: antisymmetric primary, 3 windings", "[analytical][solver][psfb]") {
    using Kirchhoff::analytical::analytical_psfb;
    // 400 V -> 12 V, 20 A, 100 kHz, n=27, Lm=1 mH, Lr=5 uH, Lo=5 uH, phase=144 deg (D_cmd=0.8).
    MAS::OperatingPoint op = analytical_psfb(400, {12}, {20}, {27}, 100000, 1e-3, 5e-6, 5e-6, 144);
    REQUIRE(op.get_excitations_per_winding().size() == 3);            // Primary + Secondary 0a + 0b
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.5));   // antisymmetric
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(5.0));                    // volt-second balance
}

TEST_CASE("analytical_psfb FULL_BRIDGE: one bipolar secondary, 2 windings", "[analytical][solver][psfb]") {
    using Kirchhoff::analytical::analytical_psfb;
    using Kirchhoff::analytical::SrcRectifier;
    MAS::OperatingPoint op = analytical_psfb(400, {12}, {20}, {27}, 100000, 1e-3, 5e-6, 5e-6, 144, 0.0,
                                             SrcRectifier::FULL_BRIDGE);
    REQUIRE(op.get_excitations_per_winding().size() == 2);             // Primary + one Secondary
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.5));   // antisymmetric primary
    // Full-bridge secondary is a single bipolar winding: full-wave (zero-mean current), carrying the
    // reflected output-inductor current only during the active fraction Deff (~0 during freewheel), so
    // RMS = sqrt(Deff)*Io_rms < Io (= 20 A) — the exact form the validated inline build uses.
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(0.0).margin(1.0));
    CHECK(*processed_current(op, 1).get_rms() > 12.0);
    CHECK(*processed_current(op, 1).get_rms() < 20.5);
}

TEST_CASE("analytical_psfb rejects zero phase shift / bad inputs", "[analytical][solver][psfb]") {
    using Kirchhoff::analytical::analytical_psfb;
    CHECK_THROWS(analytical_psfb(400, {12}, {20}, {27}, 100000, 1e-3, 5e-6, 5e-6, 0));    // D_cmd=0
    CHECK_THROWS(analytical_psfb(0,   {12}, {20}, {27}, 100000, 1e-3, 5e-6, 5e-6, 144));  // Vin=0
}

TEST_CASE("analytical_pshb: antisymmetric primary (+/-Vin/2), 3 windings", "[analytical][solver][pshb]") {
    using Kirchhoff::analytical::analytical_pshb;
    MAS::OperatingPoint op = analytical_pshb(400, {12}, {20}, {14}, 100000, 1e-3, 5e-6, 5e-6, 144);
    REQUIRE(op.get_excitations_per_winding().size() == 3);
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.5));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(5.0));
}

TEST_CASE("analytical_pshb FULL_BRIDGE: one bipolar secondary, 2 windings", "[analytical][solver][pshb]") {
    using Kirchhoff::analytical::analytical_pshb;
    using Kirchhoff::analytical::SrcRectifier;
    MAS::OperatingPoint op = analytical_pshb(400, {12}, {20}, {14}, 100000, 1e-3, 5e-6, 5e-6, 144, 0.0,
                                             SrcRectifier::FULL_BRIDGE);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.5));
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(0.0).margin(1.0));
    CHECK(*processed_current(op, 1).get_rms() > 12.0);   // sqrt(Deff)*Io, full-wave with freewheel gaps
    CHECK(*processed_current(op, 1).get_rms() < 20.5);
}

TEST_CASE("analytical_pshb CURRENT_DOUBLER: one bipolar secondary carrying ~Io/2", "[analytical][pshb]") {
    using Kirchhoff::analytical::analytical_pshb;
    using Kirchhoff::analytical::SrcRectifier;
    // Same operating point as the FULL_BRIDGE case (Io = 20 A). CURRENT_DOUBLER splits the load across
    // the two output inductors, so the single bipolar transformer secondary is centered at Io/2 — its
    // RMS is ~half the FULL_BRIDGE secondary. Winding set is Primary + ONE secondary (no half-windings).
    MAS::OperatingPoint opFb = analytical_pshb(400, {12}, {20}, {14}, 100000, 1e-3, 5e-6, 5e-6, 144, 0.0,
                                               SrcRectifier::FULL_BRIDGE);
    MAS::OperatingPoint opCd = analytical_pshb(400, {12}, {20}, {14}, 100000, 1e-3, 5e-6, 5e-6, 144, 0.0,
                                               SrcRectifier::CURRENT_DOUBLER);
    REQUIRE(opCd.get_excitations_per_winding().size() == 2);              // Primary + ONE secondary
    CHECK(*processed_current(opCd, 0).get_average() == Catch::Approx(0.0).margin(0.5));   // zero-mean primary
    CHECK(*processed_current(opCd, 1).get_average() == Catch::Approx(0.0).margin(1.0));   // bipolar secondary
    double rmsFb = *processed_current(opFb, 1).get_rms();
    double rmsCd = *processed_current(opCd, 1).get_rms();
    // Secondary carries ~Io/2 (the DC center halves; the ripple term is common), so its RMS is ~half
    // the FULL_BRIDGE secondary — a little under 0.5 because the (unchanged) ripple weighs relatively
    // more against the smaller DC. Bound it around the physical ~0.45 ratio (not tighter than the model).
    CHECK(rmsCd > 0.40 * rmsFb);
    CHECK(rmsCd < 0.55 * rmsFb);
    CHECK(rmsCd > 6.0);
    CHECK(rmsCd < 10.5);
}

TEST_CASE("build_pshb_tas CURRENT_DOUBLER: two output inductors each DC-biased at Io/2", "[analytical][pshb]") {
    // The current doubler feeds the load through TWO output inductors, each carrying the average Io/2.
    // Verify the built TAS has both Lout and Lo2 as magnetics with a DC bias (current offset) of Io/2.
    nlohmann::json di;
    di["designRequirements"]["outputs"][0]["voltage"]["nominal"] = 12.0;
    di["designRequirements"]["outputs"][0]["power"]["nominal"]   = 600.0;    // Io = 50 A
    di["designRequirements"]["switchingFrequency"]["nominal"]    = 100000.0;
    di["designRequirements"]["inputVoltage"] = {{"nominal", 400.0}, {"minimum", 380.0}, {"maximum", 420.0}};
    di["config"]["rectifierType"] = "currentDoubler";
    Kirchhoff::PshbDesign d = Kirchhoff::design_pshb(di);
    REQUIRE(d.rectifierType == Kirchhoff::RectifierType::CurrentDoubler);
    const double Io = d.outputPower / d.outputVoltage;                       // 50 A
    nlohmann::json tas = Kirchhoff::build_pshb_tas(d);

    // Locate the switching-cell stage components.
    const nlohmann::json* comps = nullptr;
    for (const auto& st : tas.at("topology").at("stages"))
        if (st.value("name", "") == "pshbCell") comps = &st.at("circuit").at("components");
    REQUIRE(comps != nullptr);

    auto inductor_offset = [&](const char* name) -> double {
        for (const auto& c : *comps)
            if (c.value("name", "") == name)
                return c.at("data").at("inputs").at("operatingPoints").at(0)
                        .at("excitationsPerWinding").at(0).at("current").at("processed").at("offset").get<double>();
        throw std::runtime_error(std::string("component not found: ") + name);
    };
    CHECK(inductor_offset("Lout") == Catch::Approx(Io / 2.0).epsilon(0.01));   // 25 A each
    CHECK(inductor_offset("Lo2")  == Catch::Approx(Io / 2.0).epsilon(0.01));
}

TEST_CASE("analytical_asymmetric_half_bridge: zero-mean primary (Cb blocks DC), 3 windings",
          "[analytical][solver][ahb]") {
    using Kirchhoff::analytical::analytical_asymmetric_half_bridge;
    // 48 V -> 12 V, 5 A, 100 kHz, n=2, Lm=200 uH, D=0.4, ripple=0.3.
    MAS::OperatingPoint op = analytical_asymmetric_half_bridge(48, {12}, {5}, {2}, 100000, 200e-6, 0.4, 0.3);
    REQUIRE(op.get_excitations_per_winding().size() == 3);            // Primary + Secondary 0a + 0b
    // Series blocking cap forces zero-mean primary current; primary voltage volt-second balanced
    // ((1-D)*Vin during D*T, -D*Vin during (1-D)*T).
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.3));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.5));
}

TEST_CASE("AHB GUI preset: primary current stored-waveform RMS == processed.rms",
          "[analytical][solver][ahb]") {
    // Regression for the time-weighted-RMS fix: the AHB primary current is non-uniformly sampled
    // (interval A denser than C, plus a coincident-time discontinuity), so an equal-weight mean of
    // squares over-reads its RMS (~1.78) vs the true time-weighted value (~1.51). processed.rms must
    // match the stored (resampled) waveform's RMS.
    // Exactly the spec the web GUI sends for the AHB preset (Vin=400, Vout=12, Po=240).
    const std::string spec = R"({
      "designRequirements": {
        "efficiency": 0.9, "inputType": "dc",
        "inputVoltage": {"nominal": 400},
        "switchingFrequency": {"nominal": 100000},
        "outputs": [{"name":"out","voltage":{"nominal":12},"regulation":"voltage"}],
        "isolationVoltage": 3000
      },
      "operatingPoints": [{"name":"full_load","inputVoltage":400,"ambientTemperature":25,
        "outputs":[{"name":"out","power":240}]}]
    })";
    const nlohmann::json out = nlohmann::json::parse(Kirchhoff::api::design_tas_full("ahb", spec));
    const nlohmann::json& awf = out.at("analyticalWaveforms");
    // main magnetic is "T1"
    const nlohmann::json& cur = awf.at("T1").at("excitationsPerWinding").at(0).at("current");
    double processedRms = cur.at("processed").at("rms");
    std::vector<double> d = cur.at("waveform").at("data");
    std::vector<double> t = cur.at("waveform").at("time");
    double integ = 0, T = t.back() - t.front();
    for (size_t i = 0; i + 1 < d.size(); ++i) {
        double a = d[i], b = d[i+1], dt = t[i+1] - t[i];
        integ += (a*a + a*b + b*b) / 3.0 * dt;
    }
    double waveformRms = std::sqrt(integ / T);
    CHECK(waveformRms == Catch::Approx(processedRms).epsilon(0.03));
}

TEST_CASE("analytical_asymmetric_half_bridge FULL_BRIDGE: one DC-biased secondary, 2 windings",
          "[analytical][solver][ahb]") {
    using Kirchhoff::analytical::analytical_asymmetric_half_bridge;
    using Kirchhoff::analytical::SrcRectifier;
    // Full-bridge rectifier: ONE secondary winding, i_sec = sign(v_pri)*i_Lo. AHB has no freewheel
    // (complementary duty) so the winding conducts the whole period => RMS ~ Io (5 A). The asymmetric
    // duty D=0.4 gives a REAL DC bias Io*(2D-1) = 5*(-0.2) = -1.0 A (the gapped AHB transformer carries
    // it) — confirmed against the ngspice deck. Primary stays zero-mean (series Cb blocks primary DC).
    MAS::OperatingPoint op = analytical_asymmetric_half_bridge(48, {12}, {5}, {2}, 100000, 200e-6, 0.4, 0.3, 0.0,
                                                              SrcRectifier::FULL_BRIDGE);
    REQUIRE(op.get_excitations_per_winding().size() == 2);            // Primary + one Secondary
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.3));   // primary zero-mean
    // |Io*(2D-1)| DC bias; +1.0 A since 2026-09-24: the winding is emitted in the TAS pin (dot) orientation,
    // which the [convention][orientation] cross-check confirms against the deck (the sign was inverted before).
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(1.0).margin(0.4));
    CHECK(*processed_current(op, 1).get_rms() > 4.0);                 // full-period conduction ~ Io
    CHECK(*processed_current(op, 1).get_rms() < 6.0);
}

TEST_CASE("analytical_asymmetric_half_bridge rejects duty outside (0,1)", "[analytical][solver][ahb]") {
    using Kirchhoff::analytical::analytical_asymmetric_half_bridge;
    CHECK_THROWS(analytical_asymmetric_half_bridge(48, {12}, {5}, {2}, 100000, 200e-6, 1.0, 0.3));
    CHECK_THROWS(analytical_asymmetric_half_bridge(48, {12}, {5}, {2}, 100000, 200e-6, 0.0, 0.3));
}

TEST_CASE("analytical_dab SPS: antisymmetric tank, 2 windings, bipolar bridge voltages",
          "[analytical][solver][dab]") {
    using Kirchhoff::analytical::analytical_dab;
    // 400 V -> 100 V, 5 A, 100 kHz, n=4 (matched: n*V2 = V1), Lm=1 mH, Lr=5 uH, SPS (D1=D2=0), D3=30 deg.
    MAS::OperatingPoint op = analytical_dab(400, {100}, {5}, {4}, 100000, 1e-3, 5e-6, 0, 0, 30);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0
    // Half-wave antisymmetry x(pi) = -x(0) => zero-mean tank current; bipolar bridge voltages zero-mean.
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.3));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(2.0));   // primary Vab
    CHECK(voltage_average(op, 1) == Catch::Approx(0.0).margin(2.0));   // secondary Vcd
}

TEST_CASE("analytical_dab power-flow direction flips with D3 sign", "[analytical][solver][dab]") {
    using Kirchhoff::analytical::analytical_dab;
    // The tank current (hence primary RMS) is the same magnitude for +/- D3 (symmetric transfer),
    // but the primary peak current sign window flips. Just check it runs and stays finite/antisymmetric.
    MAS::OperatingPoint opPos = analytical_dab(400, {100}, {5}, {4}, 100000, 1e-3, 5e-6, 0, 0,  30);
    MAS::OperatingPoint opNeg = analytical_dab(400, {100}, {5}, {4}, 100000, 1e-3, 5e-6, 0, 0, -30);
    CHECK(*processed_current(opPos, 0).get_rms() == Catch::Approx(*processed_current(opNeg, 0).get_rms()).margin(0.05));
    CHECK(*processed_current(opPos, 0).get_rms() > 0.0);
}

TEST_CASE("dab_series_inductance_for_power round-trips the general power model", "[analytical][solver][dab][eps]") {
    using Kirchhoff::analytical::dab_power_transfer;
    using Kirchhoff::analytical::dab_series_inductance_for_power;
    // For SPS and every inner-shift modulation (EPS/DPS/TPS), the L sized for a target power must, fed back
    // through the SAME kernel the waveforms use, reproduce that power — so an EPS/DPS/TPS design delivers spec.
    const double V1 = 400, V2 = 100, N = 4, Fs = 100000, Ptar = 500;
    const double D3 = 25.0 * M_PI / 180.0, deg = M_PI / 180.0;
    const std::vector<std::pair<double, double>> mods = {
        {0, 0}, {30 * deg, 0}, {30 * deg, 30 * deg}, {40 * deg, 20 * deg}};   // SPS, EPS, DPS, TPS
    for (const auto& [D1, D2] : mods) {
        const double L = dab_series_inductance_for_power(V1, V2, N, D3, D1, D2, Fs, Ptar);
        CHECK(L > 0.0);
        CHECK(dab_power_transfer(V1, V2, N, D3, D1, D2, Fs, L) == Catch::Approx(Ptar).epsilon(0.005));
    }
    // Adding an inner shift at a FIXED L reduces the transferred power (the zero plateaus shrink the
    // volt-second area) — the reactive-power / ZVS trade EPS/DPS/TPS exist for.
    const double Lfix = 5e-6;
    CHECK(dab_power_transfer(V1, V2, N, D3, 30 * deg, 0, Fs, Lfix)
          < dab_power_transfer(V1, V2, N, D3, 0, 0, Fs, Lfix));
}

TEST_CASE("analytical_dab rejects bad inputs", "[analytical][solver][dab]") {
    using Kirchhoff::analytical::analytical_dab;
    CHECK_THROWS(analytical_dab(400, {100}, {5}, {4}, 0,      1e-3, 5e-6, 0, 0, 30));   // Fs=0
    CHECK_THROWS(analytical_dab(400, {100}, {5}, {4}, 100000, 1e-3, 0,    0, 0, 30));   // Lr=0
    CHECK_THROWS(analytical_dab(400, {100}, {5}, {4}, 100000, 0,    5e-6, 0, 0, 30));   // Lm=0
}

// ─── Phase 5: resonant family (FHA) ─────────────────────────────────────────

TEST_CASE("analytical_src FHA above resonance: sinusoidal tank, 2 windings", "[analytical][solver][src]") {
    using Kirchhoff::analytical::analytical_src;
    // Lr=40 uH, Cr=63.3 nF -> fr ~ 100 kHz; fsw=120 kHz (Lambda=1.2, above res); 400 V -> 48 V, 5 A, n=8.
    const double Lr = 40e-6, Cr = 63.3e-9;
    MAS::OperatingPoint op = analytical_src(400, {48}, {5}, {8}, 120000, Lr, Cr);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0 (full-bridge)
    // FHA tank current is a pure sinusoid: zero-mean, RMS = Ipk/sqrt(2).
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.2));
    const double ipk = *op.get_excitations_per_winding()[0].get_current()->get_processed()->get_peak();
    CHECK(ipk > 0.0);
    CHECK(*processed_current(op, 0).get_rms() == Catch::Approx(ipk / std::sqrt(2.0)).margin(0.08 * ipk));
    // Square bridge voltage +/- Vin: zero-mean.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(2.0));
}

TEST_CASE("analytical_src center-tapped rectifier: 3 windings", "[analytical][solver][src]") {
    using Kirchhoff::analytical::analytical_src;
    using Kirchhoff::analytical::SrcRectifier;
    MAS::OperatingPoint op = analytical_src(400, {48}, {5}, {8}, 120000, 40e-6, 63.3e-9, 1.0,
                                            SrcRectifier::CENTER_TAPPED);
    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Primary + 2 half-windings
    // Each half-winding only conducts its half-cycle => non-negative current.
    // MAS convention (2026-09-24): both halves in the dot reference, so Half 2 (the negative polarity) carries
    // a NON-POSITIVE source current (it was emitted non-negative in its own reference before).
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_positive_peak() <= 0.05);
}

TEST_CASE("analytical_src rejects below-resonance and bad tank", "[analytical][solver][src]") {
    using Kirchhoff::analytical::analytical_src;
    CHECK_THROWS(analytical_src(400, {48}, {5}, {8}, 80000, 40e-6, 63.3e-9));   // Lambda=0.8 < 1
    CHECK_THROWS(analytical_src(400, {48}, {5}, {8}, 120000, 0, 63.3e-9));      // Lr=0
    CHECK_THROWS(analytical_src(400, {48}, {5}, {8}, 120000, 40e-6, 0));        // Cr=0
}

// ─── Phase 5: LLC resonant converter (Runo Nielsen TDA) ─────────────────────
// Structural invariants of the time-domain tank solver: the symmetric half-bridge yields an
// antisymmetric primary tank current (zero-mean); the winding set is Primary + the rectifier's
// secondaries; and the throw guards fire on non-positive fsw / Lm / Ls / Cr. Driven BELOW resonance
// (fsw < fr) where the multi-start Newton converges cleanly (see the [nrmse][llc] gate for the
// at-resonance singularity characterization). Follows the SRC/DAB structural-test style.

TEST_CASE("analytical_llc center-tapped: antisymmetric tank current, 3 windings",
          "[analytical][solver][llc]") {
    using Kirchhoff::analytical::analytical_llc;
    using Kirchhoff::analytical::SrcRectifier;
    // 400 V -> 12 V, 10 A, half-bridge (k=0.5), n=16, Lm=589 uH, Ls=118 uH, Cr=13.4 nF.
    // fsw=100 kHz is below fr=1/(2*pi*sqrt(Ls*Cr))~=126 kHz, so the TDA solver converges.
    MAS::OperatingPoint op = analytical_llc(400, {12}, {10}, {16}, 100000, 589e-6, 118e-6, 13.4e-9,
                                            0.5, SrcRectifier::CENTER_TAPPED);
    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Primary + Secondary 0 Half 1/2
    // Symmetric bridge -> half-wave-antisymmetric tank current -> zero mean (small vs the RMS).
    const double cur = *processed_current(op, 0).get_average();   // copy: processed_current returns a temporary
    const double rms = *processed_current(op, 0).get_rms();
    CHECK(rms > 0.0);
    CHECK(std::abs(cur) < 0.15 * rms + 0.05);
    // Each center-tapped half-winding conducts only one polarity -> non-negative current.
    // MAS convention (2026-09-24): both halves in the dot reference, so Half 2 (the negative polarity) carries
    // a NON-POSITIVE source current (it was emitted non-negative in its own reference before).
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_positive_peak() <= 0.05);
}

TEST_CASE("analytical_llc full-bridge rectifier: 2 windings, bipolar secondary",
          "[analytical][solver][llc]") {
    using Kirchhoff::analytical::analytical_llc;
    using Kirchhoff::analytical::SrcRectifier;
    MAS::OperatingPoint op = analytical_llc(400, {12}, {10}, {16}, 100000, 589e-6, 118e-6, 13.4e-9,
                                            0.5, SrcRectifier::FULL_BRIDGE);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0
    CHECK(std::abs(*processed_current(op, 0).get_average()) <
          0.15 * (*processed_current(op, 0).get_rms()) + 0.05);
    // Full-bridge secondary carries the bipolar reflected diode current (swings both signs).
    CHECK(*processed_current(op, 1).get_peak() > 0.0);
    CHECK(*processed_current(op, 1).get_negative_peak() < 0.0);
}

TEST_CASE("analytical_llc rejects non-positive fsw / Lm / Ls / Cr / turns ratio",
          "[analytical][solver][llc]") {
    using Kirchhoff::analytical::analytical_llc;
    CHECK_THROWS(analytical_llc(400, {12}, {10}, {16}, 0,      589e-6, 118e-6, 13.4e-9));  // fsw=0
    CHECK_THROWS(analytical_llc(400, {12}, {10}, {16}, 100000, 0,      118e-6, 13.4e-9));  // Lm=0
    CHECK_THROWS(analytical_llc(400, {12}, {10}, {16}, 100000, 589e-6, 0,      13.4e-9));  // Ls=0
    CHECK_THROWS(analytical_llc(400, {12}, {10}, {16}, 100000, 589e-6, 118e-6, 0));        // Cr=0
    CHECK_THROWS(analytical_llc(400, {12}, {10}, {0},  100000, 589e-6, 118e-6, 13.4e-9));  // n=0
    // Vector length mismatch.
    CHECK_THROWS(analytical_llc(400, {12}, {10, 1}, {16}, 100000, 589e-6, 118e-6, 13.4e-9));
}

// ─── Phase 5: CLLC bidirectional resonant converter (4-state Sun et al. TDA) ──
// Structural invariants of the 4-state time-domain tank solver: the symmetric full bridge yields an
// antisymmetric primary tank current (zero-mean); the winding set is Primary + the rectifier's
// secondaries; the throw guards fire on non-positive fsw / Lm / tank values / turns ratio and on an
// infeasible conversion gain. Driven BELOW resonance (fsw < fr) where the damped-Picard steady-state
// solve converges cleanly (residual < 0.5 A) — see the [nrmse][cllc] gate for the at-resonance
// singularity characterization. Tank params below are a design_cllc 400 V→48 V/480 W design (n≈7.72,
// Lm=492 µH, Lr1=110.6 µH, Cr1=22.9 nF, Lr2=1.86 µH, Cr2=1.364 µF; fr≈100 kHz). Follows the LLC style.
namespace {
constexpr double kCllcVin = 400, kCllcVout = 48, kCllcIout = 10, kCllcN = 7.71605;
constexpr double kCllcLm = 492.179e-6, kCllcLr1 = 110.602e-6, kCllcCr1 = 22.9022e-9;
constexpr double kCllcLr2 = 1.85769e-6, kCllcCr2 = 1.36354e-6;
constexpr double kCllcFsub = 85000;   // below fr → the Picard solve converges cleanly
}

TEST_CASE("analytical_cllc full-bridge: antisymmetric tank current, 2 windings",
          "[analytical][solver][cllc]") {
    using Kirchhoff::analytical::analytical_cllc;
    MAS::OperatingPoint op = analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {kCllcN}, kCllcFsub,
                                             kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0 (full-wave)
    // Symmetric full bridge → half-wave-antisymmetric tank current → zero mean (small vs the RMS).
    const double mean = *processed_current(op, 0).get_average();
    const double rms  = *processed_current(op, 0).get_rms();
    CHECK(rms > 0.0);
    CHECK(std::abs(mean) < 0.15 * rms + 0.05);
    // Full-wave secondary carries the bipolar transferred rectifier current n·(iLs−iLm): swings both signs.
    CHECK(*processed_current(op, 1).get_peak() > 0.0);
    CHECK(*processed_current(op, 1).get_negative_peak() < 0.0);
    // Primary voltage (magnetizing VLm, clamped to ±n·Vout during power delivery) is zero-mean.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(2.0));
}

TEST_CASE("analytical_cllc center-tapped rectifier: 3 windings", "[analytical][solver][cllc]") {
    using Kirchhoff::analytical::analytical_cllc;
    using Kirchhoff::analytical::SrcRectifier;
    MAS::OperatingPoint op = analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {kCllcN}, kCllcFsub,
                                             kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2,
                                             1.0, SrcRectifier::CENTER_TAPPED);
    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Primary + Secondary 0 Half 1/2
    // Each center-tapped half-winding conducts only one polarity → non-negative current.
    // MAS convention (2026-09-24): both halves in the dot reference, so Half 2 (the negative polarity) carries
    // a NON-POSITIVE source current (it was emitted non-negative in its own reference before).
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_positive_peak() <= 0.05);
}

TEST_CASE("analytical_cllc rejects bad inputs", "[analytical][solver][cllc]") {
    using Kirchhoff::analytical::analytical_cllc;
    // fsw / Lm / turns ratio / tank values non-positive.
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {kCllcN}, 0,
                                 kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2));               // fsw=0
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {kCllcN}, kCllcFsub,
                                 0, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2));                     // Lm=0
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {0.0}, kCllcFsub,
                                 kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2));               // n=0
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {kCllcN}, kCllcFsub,
                                 kCllcLm, 0, kCllcCr1, kCllcLr2, kCllcCr2));                      // Lr1=0
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {kCllcN}, kCllcFsub,
                                 kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, 0));                      // Cr2=0
    // Vector length mismatch.
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout, 1}, {kCllcN}, kCllcFsub,
                                 kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2));
    // (The old TDA also threw on "infeasible gain" n=1 → M_req=0.12; the load-aware FHA has no such
    // convergence limitation — it computes that off-design point fine — so that assertion is dropped.)
}

// ─── Phase 5: CLLLC bidirectional resonant converter (4-state RK4 affine-propagator TDA) ──────────────
// Structural invariants of the RK4 affine-propagator tank solver: the symmetric full bridge yields a
// half-wave-antisymmetric primary tank current (zero-mean by construction); the winding set is Primary +
// the rectifier's secondaries; and the throw guards fire on non-positive fsw / Lm / tank values / turns
// ratio / bus voltages. There is NO CLLLC reference design, so these are STRUCTURAL checks only (no NRMSE
// gate). Driven a few % BELOW resonance (fsw < fr): the RK4 solver's 1 mΩ ESR keeps (M+I) non-singular at
// any frequency, but below-resonance operation gives a healthy, clearly non-trivial tank current so the
// zero-mean assertion is meaningful. Symmetric 400 V(HV)→48 V(LV) tank, n≈8.33, Lm=490 µH, Lr1=110 µH,
// Cr1=23 nF, Lr2=Lr1/n²≈1.58 µH, Cr2=Cr1·n²≈1.60 µF; fr = 1/(2π√(Lr1·Cr1)) ≈ 100.1 kHz.
namespace {
constexpr double kClllcVhv = 400, kClllcVlv = 48, kClllcIout = 10, kClllcN = 8.33333;
constexpr double kClllcLm = 490e-6, kClllcLr1 = 110e-6, kClllcCr1 = 23e-9;
constexpr double kClllcLr2 = 1.5840e-6, kClllcCr2 = 1.5972e-6;
constexpr double kClllcFsub = 90000;   // ~10% below fr (≈100.1 kHz) → clearly non-trivial tank current
}

TEST_CASE("analytical_clllc full-bridge: antisymmetric tank current, 2 windings",
          "[analytical][solver][clllc]") {
    using Kirchhoff::analytical::analytical_clllc;
    MAS::OperatingPoint op = analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {kClllcN}, kClllcFsub,
                                              kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0 (full-wave)
    // Symmetric full bridge → half-wave-antisymmetric primary tank current → zero mean (tiny vs the RMS).
    const double mean = *processed_current(op, 0).get_average();
    const double rms  = *processed_current(op, 0).get_rms();
    CHECK(rms > 0.0);
    CHECK(std::abs(mean) < 0.15 * rms + 0.05);
    // Full-wave secondary carries the bipolar LV-side tank current i_Lr2 (≈ n·i_Lr1): swings both signs.
    CHECK(*processed_current(op, 1).get_peak() > 0.0);
    CHECK(*processed_current(op, 1).get_negative_peak() < 0.0);
    // Primary winding voltage (v_pri = Lm·di_Lm/dt, half-wave antisymmetric) is zero-mean.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(2.0));
}

TEST_CASE("analytical_clllc center-tapped rectifier: 3 windings", "[analytical][solver][clllc]") {
    using Kirchhoff::analytical::analytical_clllc;
    using Kirchhoff::analytical::SrcRectifier;
    MAS::OperatingPoint op = analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {kClllcN}, kClllcFsub,
                                              kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2,
                                              1.0, SrcRectifier::CENTER_TAPPED);
    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Primary + Secondary 0 Half 1/2
    // Each center-tapped half-winding conducts only one polarity → non-negative current.
    // MAS convention (2026-09-24): both halves in the dot reference, so Half 2 (the negative polarity) carries
    // a NON-POSITIVE source current (it was emitted non-negative in its own reference before).
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_positive_peak() <= 0.05);
    // Primary tank current is still zero-mean (antisymmetry is independent of the secondary rectifier).
    CHECK(std::abs(*processed_current(op, 0).get_average()) <
          0.15 * (*processed_current(op, 0).get_rms()) + 0.05);
}

TEST_CASE("analytical_clllc rejects non-positive fsw / Lm / tank / turns ratio / bus voltage",
          "[analytical][solver][clllc]") {
    using Kirchhoff::analytical::analytical_clllc;
    CHECK_THROWS(analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {kClllcN}, 0,
                                  kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2));      // fsw=0
    CHECK_THROWS(analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {kClllcN}, kClllcFsub,
                                  0, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2));             // Lm=0
    CHECK_THROWS(analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {0.0}, kClllcFsub,
                                  kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2));      // n=0
    CHECK_THROWS(analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {kClllcN}, kClllcFsub,
                                  kClllcLm, 0, kClllcCr1, kClllcLr2, kClllcCr2));              // Lr1=0
    CHECK_THROWS(analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout}, {kClllcN}, kClllcFsub,
                                  kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, 0));              // Cr2=0
    CHECK_THROWS(analytical_clllc(kClllcVhv, {0.0}, {kClllcIout}, {kClllcN}, kClllcFsub,
                                  kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2));      // Vlv=0
    // Vector length mismatch.
    CHECK_THROWS(analytical_clllc(kClllcVhv, {kClllcVlv}, {kClllcIout, 1}, {kClllcN}, kClllcFsub,
                                  kClllcLm, kClllcLr1, kClllcCr1, kClllcLr2, kClllcCr2));
}

// ─── Phase 6: three-phase AC-input PFC — Vienna rectifier (structural) ──────────
// STRUCTURAL tests only. Vienna is a 3-phase AC-input PFC; the PtP/ngspice cross-check suite EXCLUDES
// it (no clean single-vector boost-inductor mapping — the SPICE side is a single-phase peak-of-line
// boost emulation), so there is NO NRMSE gate. We assert the winding count (3 per-phase boost inductors),
// the current-envelope shape/magnitude (fullLineCycle: bipolar full-sine of amplitude ≈ the per-phase
// line-current peak I_pk, zero mean over the line cycle; peakOfLine: triangular about I_pk), and the
// throw guards. Design point: 230 V L-N (=400 V L-L) → 800 V split bus, 10 kW, 50 Hz line, 70 kHz sw,
// L = 500 µH, η = pf = 1.
namespace {
constexpr double kVienVph = 230.0, kVienVdc = 800.0, kVienPo = 10000.0;
constexpr double kVienFline = 50.0, kVienFsw = 70000.0, kVienL = 500e-6;
// Closed form (MKF Vienna::compute_*): V_phase_peak = √2·Vph; M = Vpk/(Vdc/2); I_pk = √2·P/(3·Vph);
// ΔI_pp(peak) = V_phase_peak·(1−M)·Tsw/L.
const double kVienVpk   = std::sqrt(2.0) * kVienVph;                 // 325.27 V
const double kVienM     = kVienVpk / (kVienVdc / 2.0);              // 0.8132
const double kVienIpk   = std::sqrt(2.0) * kVienPo / (3.0 * kVienVph);   // 20.50 A
const double kVienDIpp  = kVienVpk * (1.0 - kVienM) / kVienFsw / kVienL; // 1.736 A
}

TEST_CASE("analytical_vienna fullLineCycle: 3 windings, bipolar sine envelope peak = I_pk",
          "[analytical][solver][vienna]") {
    using Kirchhoff::analytical::analytical_vienna;
    MAS::OperatingPoint op = analytical_vienna(kVienVph, kVienVdc, kVienPo, kVienFline, kVienFsw, kVienL);

    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Phase A / B / C boost inductors
    const auto& cur = *processed_current(op, 0).get_average();  // (forces the optional; unused directly)
    (void)cur;

    // The line-cycle envelope is a bipolar full sine of amplitude I_pk + the local switching ripple:
    // positive peak ≈ +I_pk, negative peak ≈ −I_pk (MKF builds it bipolar), zero mean over the cycle.
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(kVienIpk).margin(kVienDIpp));
    REQUIRE(processed_current(op, 0).get_negative_peak().has_value());
    CHECK(*processed_current(op, 0).get_negative_peak() == Catch::Approx(-kVienIpk).margin(kVienDIpp));
    CHECK(*processed_current(op, 0).get_negative_peak() < 0.0);                 // bipolar, as MKF builds it
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.5));   // full sine → zero mean

    // The three phases are the same envelope shifted ±120° → identical magnitude statistics.
    CHECK(*processed_current(op, 1).get_peak() == Catch::Approx(*processed_current(op, 0).get_peak()).margin(0.3));
    CHECK(*processed_current(op, 2).get_peak() == Catch::Approx(*processed_current(op, 0).get_peak()).margin(0.3));

    // Voltage excitation present on every winding.
    REQUIRE(op.get_excitations_per_winding()[0].get_voltage().has_value());
    REQUIRE(op.get_excitations_per_winding()[0].get_voltage()->get_processed().has_value());
}

TEST_CASE("analytical_vienna peakOfLine: 3 windings, triangular about I_pk, zero-mean voltage",
          "[analytical][solver][vienna]") {
    using Kirchhoff::analytical::analytical_vienna;
    // fullLineCycle=false → the peak-of-line switching-period snapshot.
    MAS::OperatingPoint op = analytical_vienna(kVienVph, kVienVdc, kVienPo, kVienFline, kVienFsw, kVienL,
                                               1.0, 1.0, /*fullLineCycle=*/false);
    REQUIRE(op.get_excitations_per_winding().size() == 3);
    // Triangular inductor current about the per-phase line-current peak I_pk, ripple = ΔI_pp.
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(kVienIpk).margin(0.2));
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(kVienIpk + kVienDIpp / 2.0).margin(0.2));
    CHECK(*processed_current(op, 0).get_peak_to_peak() == Catch::Approx(kVienDIpp).margin(0.2));
    CHECK(*processed_current(op, 0).get_negative_peak() > 0.0);   // I_pk − ΔI_pp/2 > 0 (non-negative snapshot)
    // Inductor voltage is volt-second balanced (V_on = V_phase_peak during D, V_off = V_phase_peak − Vdc/2).
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.5));
}

TEST_CASE("analytical_vienna rejects non-positive line/bus/fsw/L and over-modulation",
          "[analytical][solver][vienna]") {
    using Kirchhoff::analytical::analytical_vienna;
    CHECK_THROWS(analytical_vienna(0.0,      kVienVdc, kVienPo, kVienFline, kVienFsw, kVienL));  // Vph=0
    CHECK_THROWS(analytical_vienna(kVienVph, 0.0,      kVienPo, kVienFline, kVienFsw, kVienL));  // Vdc=0
    CHECK_THROWS(analytical_vienna(kVienVph, kVienVdc, 0.0,     kVienFline, kVienFsw, kVienL));  // Po=0
    CHECK_THROWS(analytical_vienna(kVienVph, kVienVdc, kVienPo, kVienFline, 0.0,      kVienL));  // fsw=0
    CHECK_THROWS(analytical_vienna(kVienVph, kVienVdc, kVienPo, kVienFline, kVienFsw, 0.0));     // L=0
    CHECK_THROWS(analytical_vienna(kVienVph, kVienVdc, kVienPo, 0.0,        kVienFsw, kVienL));  // fLine=0
    // Over-modulation: Vdc = 400 V gives M = 325.27/200 = 1.63 > 1.
    CHECK_THROWS(analytical_vienna(kVienVph, 400.0,    kVienPo, kVienFline, kVienFsw, kVienL));
    // efficiency / power factor out of (0,1].
    CHECK_THROWS(analytical_vienna(kVienVph, kVienVdc, kVienPo, kVienFline, kVienFsw, kVienL, 1.5, 1.0));
    CHECK_THROWS(analytical_vienna(kVienVph, kVienVdc, kVienPo, kVienFline, kVienFsw, kVienL, 1.0, 0.0));
}

// ─── Phase 7: single-phase AC-input PFC — boost front end (structural) ──────────
// STRUCTURAL tests only. A boost PFC is AC-input + closed-loop, so it does NOT map to a single settled
// ngspice vector (same reasoning as Vienna) — there is NO NRMSE gate. We assert: the winding count (1
// boost inductor), the current-envelope shape/magnitude (a RECTIFIED-sine |sin| envelope of amplitude
// ≈ the line-current peak I_pk = √2·Pin/Vrms, so peak ≈ I_pk, NON-negative min ≈ 0, and a NON-zero
// rectified mean ≈ (2/π)·I_pk — distinct from a bipolar full-sine which would be zero-mean), the
// volt-second-balanced (zero-mean) inductor voltage, and the throw guards. Design point: 230 V RMS line
// → 400 V bus, 3 kW, 50 Hz line, 65 kHz sw, L = 500 µH, η = 1.
namespace {
constexpr double kPfcVrms = 230.0, kPfcVout = 400.0, kPfcPo = 3000.0;
constexpr double kPfcFline = 50.0, kPfcFsw = 65000.0, kPfcL = 500e-6;
// Closed form (MKF PowerFactorCorrection::process_operating_points): Pin = Po/η;
// iLinePeak = √2·Pin/Vrms; boost duty at line peak D = 1 − √2·Vrms/Vout; ΔI_pp = √2·Vrms·D/(L·fsw).
const double kPfcIpk   = std::sqrt(2.0) * kPfcPo / kPfcVrms;               // √2·Pin/Vrms ≈ 18.45 A
const double kPfcVpk   = std::sqrt(2.0) * kPfcVrms;                        // 325.27 V
const double kPfcDpeak = 1.0 - kPfcVpk / kPfcVout;                         // 0.1868
const double kPfcDIpp  = kPfcVpk * kPfcDpeak / (kPfcL * kPfcFsw);          // ≈ 1.87 A ripple pk-pk at peak
const double kPfcMean  = (2.0 / M_PI) * kPfcIpk;                           // rectified-sine mean ≈ 11.74 A
}

TEST_CASE("analytical_pfc: 1 winding, rectified-sine envelope peak = I_pk, non-zero rectified mean",
          "[analytical][solver][pfc]") {
    using Kirchhoff::analytical::analytical_pfc;
    MAS::OperatingPoint op = analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, kPfcFline, kPfcFsw, kPfcL);

    REQUIRE(op.get_excitations_per_winding().size() == 1);   // single boost inductor
    const auto& exc = op.get_excitations_per_winding()[0];
    REQUIRE(exc.get_current().has_value());
    REQUIRE(exc.get_current()->get_processed().has_value());
    const auto cur = processed_current(op, 0);

    // Envelope peak = line-current peak I_pk (+ up to ΔI_pp/2 of switching ripple at the line peak).
    REQUIRE(cur.get_peak().has_value());
    CHECK(*cur.get_peak() == Catch::Approx(kPfcIpk).margin(kPfcDIpp));
    // Rectified (|sin|) → NON-negative, so the trough sits at ≈ 0 (bounded below by 0).
    REQUIRE(cur.get_negative_peak().has_value());
    CHECK(*cur.get_negative_peak() == Catch::Approx(0.0).margin(0.3));
    CHECK(*cur.get_negative_peak() >= -0.3);
    // Rectified-sine mean ≈ (2/π)·I_pk > 0 — a boost PFC inductor carries a unipolar current
    // (distinct from a bipolar full-sine, which would be zero-mean).
    REQUIRE(cur.get_average().has_value());
    CHECK(*cur.get_average() == Catch::Approx(kPfcMean).margin(0.8));
    CHECK(*cur.get_average() > 5.0);

    // Inductor voltage present: ON-time reaches +Vin_peak (√2·Vrms); OFF-time swings strongly negative
    // (Vin−Vout, the inductor discharging into the boost bus). NOTE: we do NOT assert a zero (volt-second-
    // balanced) mean — MKF synthesises the ON/OFF voltage with only 4 samples per switching cycle
    // (the discrete `switchPhase < D` threshold, PowerFactorCorrection.cpp:570-585), so that coarse
    // quantization biases the discrete voltage mean away from zero (≈ +44 V at this design point) even
    // though the physical inductor voltage is volt-second balanced. The CURRENT ripple uses the
    // continuous duty and is correct; this bias is a faithful artifact of MKF's voltage synthesis.
    REQUIRE(exc.get_voltage().has_value());
    REQUIRE(exc.get_voltage()->get_processed().has_value());
    const auto vlt = *exc.get_voltage()->get_processed();
    REQUIRE(vlt.get_peak().has_value());
    CHECK(*vlt.get_peak() == Catch::Approx(kPfcVpk).margin(1.0));   // ON-time = +Vin_peak
    REQUIRE(vlt.get_negative_peak().has_value());
    CHECK(*vlt.get_negative_peak() < -100.0);                       // OFF-time = Vin−Vout (boost discharge)
}

TEST_CASE("analytical_pfc bipolar (totem-pole): TRUE sine inductor current, zero mean, ±I_pk",
          "[analytical][solver][pfc]") {
    // Bridgeless TOTEM-POLE branch (ABT #92, bipolar=true): the inductor sits on the AC line with no
    // rectifier, so it carries a TRUE bipolar sine — reaching BOTH +I_pk and −I_pk with a ~zero mean —
    // instead of the bridged boost's rectified-sine (unipolar, (2/π)·I_pk mean). Same design point.
    using Kirchhoff::analytical::analytical_pfc;
    MAS::OperatingPoint op = analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, kPfcFline, kPfcFsw, kPfcL,
                                            /*efficiency*/1.0, /*Vd*/0.0, /*numberOfPeriods*/2,
                                            /*bipolar*/true);
    REQUIRE(op.get_excitations_per_winding().size() == 1);
    const auto cur = processed_current(op, 0);
    REQUIRE(cur.get_peak().has_value());
    REQUIRE(cur.get_negative_peak().has_value());
    REQUIRE(cur.get_average().has_value());
    // Reaches both rails: +I_pk and −I_pk (± up to ΔI_pp/2 of switching ripple).
    CHECK(*cur.get_peak()          == Catch::Approx(+kPfcIpk).margin(kPfcDIpp));
    CHECK(*cur.get_negative_peak() == Catch::Approx(-kPfcIpk).margin(kPfcDIpp));
    // Zero-mean (bipolar), NOT the rectified-sine (2/π)·I_pk of the bridged boost.
    CHECK(std::abs(*cur.get_average()) < 1.0);
    CHECK(*cur.get_average() < 0.5 * kPfcMean);   // clearly distinct from the unipolar mean
}

TEST_CASE("analytical_pfc rejects non-positive line/bus/power/fsw/L and infeasible step-up",
          "[analytical][solver][pfc]") {
    using Kirchhoff::analytical::analytical_pfc;
    CHECK_THROWS(analytical_pfc(0.0,      kPfcVout, kPfcPo, kPfcFline, kPfcFsw, kPfcL));  // Vrms=0
    CHECK_THROWS(analytical_pfc(kPfcVrms, 0.0,      kPfcPo, kPfcFline, kPfcFsw, kPfcL));  // Vout=0
    CHECK_THROWS(analytical_pfc(kPfcVrms, kPfcVout, 0.0,    kPfcFline, kPfcFsw, kPfcL));  // Po=0
    CHECK_THROWS(analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, 0.0,       kPfcFsw, kPfcL));  // fLine=0
    CHECK_THROWS(analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, kPfcFline, 0.0,     kPfcL));  // fsw=0
    CHECK_THROWS(analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, kPfcFline, kPfcFsw, 0.0));    // L=0
    // Infeasible step-up: Vout = 300 V < √2·230 = 325 V peak line (boost can only step up).
    CHECK_THROWS(analytical_pfc(kPfcVrms, 300.0,    kPfcPo, kPfcFline, kPfcFsw, kPfcL));
    // efficiency out of (0,1].
    CHECK_THROWS(analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, kPfcFline, kPfcFsw, kPfcL, 1.5));
    CHECK_THROWS(analytical_pfc(kPfcVrms, kPfcVout, kPfcPo, kPfcFline, kPfcFsw, kPfcL, 0.0));
}

// ─── Phase 8: magnetic-COMPONENT operating-point models (CT / DMC / CMC) ────────
// STRUCTURAL tests only — these are magnetic COMPONENTS (a current-sense transformer, a differential-
// mode choke, a common-mode choke), NOT gated switching converters, so there is NO NRMSE gate. We assert
// the winding count, the per-winding current/voltage shape + magnitude against the closed form, and the
// throw guards (mirroring each MKF model's own guards).

// Current-sense transformer. Ported from MKF converter_models/CurrentTransformer.cpp:42.
TEST_CASE("analytical_current_transformer: 2 windings, secondary = primary*turnsRatio, burden voltage",
          "[analytical][component][ct]") {
    using Kirchhoff::analytical::analytical_current_transformer;
    // 100 A sensed line current, 100 kHz, turnsRatio Np/Ns = 0.01 (1:100 step-up turns → 1:100 step-down
    // current), burden = 10 Ω, secondary DC resistance 0.5 Ω, diode drop 0.7 V.
    const double ipk = 100, freq = 100000, n = 0.01, burden = 10, rdc = 0.5, vd = 0.7;
    MAS::OperatingPoint op = analytical_current_transformer(
        MAS::WaveformLabel::SINUSOIDAL, ipk, freq, n, burden, rdc, 0.5, vd);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary
    // Primary winding carries the sensed line current (zero-mean sine of peak I_pk).
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(ipk).margin(0.5));
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.05));
    // Secondary current = primary × turnsRatio (Ip·Np = Is·Ns).
    CHECK(*processed_current(op, 1).get_peak() == Catch::Approx(ipk * n).margin(0.05));   // 1.0 A
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(0.0).margin(0.05));
    // Secondary voltage = Is·(burden + Rsec_dc) + Vdiode: a zero-mean sine of peak Is_pk·(burden+Rsec_dc)
    // riding on the diode-drop DC term. The winding DCR drop i·Rsec_dc is zero-mean (rides on the AC
    // current), so it does NOT enter the DC average — only the rectifier drop does.
    CHECK(voltage_average(op, 1) == Catch::Approx(vd).margin(0.05));                       // 0.7 V DC (diode only)
    CHECK(*processed_voltage(op, 1).get_peak() == Catch::Approx(ipk * n * (burden + rdc) + vd).margin(0.1)); // 11.2
    // Primary winding voltage is the secondary voltage reflected back: V_pri = V_sec × turnsRatio.
    CHECK(*processed_voltage(op, 0).get_peak() ==
          Catch::Approx((ipk * n * (burden + rdc) + vd) * n).margin(0.02));                // 0.112 V
}

TEST_CASE("analytical_current_transformer rejects bad waveform label and non-positive inputs",
          "[analytical][component][ct]") {
    using Kirchhoff::analytical::analytical_current_transformer;
    // Only SINUSOIDAL / UNIPOLAR_RECTANGULAR / UNIPOLAR_TRIANGULAR are allowed (MKF CT guard).
    CHECK_THROWS(analytical_current_transformer(MAS::WaveformLabel::BIPOLAR_TRIANGULAR, 100, 100000, 0.01, 10));
    CHECK_THROWS(analytical_current_transformer(MAS::WaveformLabel::TRIANGULAR, 100, 100000, 0.01, 10));
    // Non-positive required inputs.
    CHECK_THROWS(analytical_current_transformer(MAS::WaveformLabel::SINUSOIDAL, 0.0, 100000, 0.01, 10)); // peak
    CHECK_THROWS(analytical_current_transformer(MAS::WaveformLabel::SINUSOIDAL, 100, 0.0,    0.01, 10)); // freq
    CHECK_THROWS(analytical_current_transformer(MAS::WaveformLabel::SINUSOIDAL, 100, 100000, 0.0,  10)); // n
    // A valid unipolar label works (no throw).
    CHECK_NOTHROW(analytical_current_transformer(MAS::WaveformLabel::UNIPOLAR_TRIANGULAR, 100, 100000, 0.01, 10));
}

// Differential-mode choke. Ported from MKF converter_models/DifferentialModeChoke.cpp:145.
TEST_CASE("analytical_differential_mode_choke single-phase: 1 winding, current RMS = line current",
          "[analytical][component][dmc]") {
    using Kirchhoff::analytical::analytical_differential_mode_choke;
    // 10 A line current, 230 V, 50 Hz line, 100 kHz switching ripple, single phase (default peak → 20% ripple).
    const double iop = 10, vin = 230, fline = 50, fsw = 100000;
    MAS::OperatingPoint op = analytical_differential_mode_choke(100e-6, iop, fline, fsw);

    REQUIRE(op.get_excitations_per_winding().size() == 1);   // single winding
    // Line-frequency sinusoid of amplitude √2·Iop (RMS→peak) + a switching ripple of amplitude
    // (peak−operating)=0.2·Iop → RMS ≈ Iop, zero mean, peak ≈ √2·Iop + 0.2·Iop.
    const double ripple = 0.2 * iop;   // default peakCurrent = 1.2·Iop
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(0.0).margin(0.3));
    CHECK(*processed_current(op, 0).get_rms() == Catch::Approx(iop).margin(0.3));
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(iop * std::sqrt(2.0) + ripple).margin(0.5));
    // The magnetizing current (Σ of all winding currents) is set on the first winding's excitation.
    REQUIRE(op.get_excitations_per_winding()[0].get_magnetizing_current().has_value());
    CHECK(*processed_magnetizing(op, 0).get_rms() > 0.0);
}

TEST_CASE("analytical_differential_mode_choke three-phase: 3 windings, identical, DM currents cancel in core",
          "[analytical][component][dmc]") {
    using Kirchhoff::analytical::analytical_differential_mode_choke;
    using Kirchhoff::analytical::DmcConfiguration;
    const double iop = 10;
    MAS::OperatingPoint op = analytical_differential_mode_choke(
        100e-6, iop, 50, 100000, DmcConfiguration::THREE_PHASE);

    REQUIRE(op.get_excitations_per_winding().size() == 3);   // Phase A / B / C
    // The three windings carry the same-magnitude line current (120° apart) → identical RMS ≈ Iop.
    CHECK(*processed_current(op, 0).get_rms() == Catch::Approx(iop).margin(0.3));
    CHECK(*processed_current(op, 1).get_rms() == Catch::Approx(*processed_current(op, 0).get_rms()).margin(0.05));
    CHECK(*processed_current(op, 2).get_rms() == Catch::Approx(*processed_current(op, 0).get_rms()).margin(0.05));
    // Magnetizing current = point-by-point sum: the balanced sines cancel, leaving only the common ripple,
    // so its peak is far below a single winding's peak (the DM current does NOT saturate the core).
    REQUIRE(op.get_excitations_per_winding()[0].get_magnetizing_current().has_value());
    CHECK(*processed_magnetizing(op, 0).get_peak() < *processed_current(op, 0).get_peak());
}

TEST_CASE("analytical_differential_mode_choke rejects non-positive frequencies / missing current",
          "[analytical][component][dmc]") {
    using Kirchhoff::analytical::analytical_differential_mode_choke;
    CHECK_THROWS(analytical_differential_mode_choke(100e-6, 10, 50, 0.0));     // switchingFrequency = 0
    CHECK_THROWS(analytical_differential_mode_choke(100e-6, 10, 0.0, 100000)); // lineFrequency = 0
    // Neither peakCurrent (NaN default) nor a positive operatingCurrent → cannot size the choke.
    CHECK_THROWS(analytical_differential_mode_choke(100e-6, 0.0, 50, 100000));
    CHECK_THROWS(analytical_differential_mode_choke(0.0, 10, 50, 100000));    // magnetizing inductance = 0
}

// Common-mode choke. Ported from MKF converter_models/CommonModeChoke.cpp:327 (scalar-arg overload).
TEST_CASE("analytical_common_mode_choke: 2 windings, DM line/return levels cancel, small identical CM ripple",
          "[analytical][component][cmc]") {
    using Kirchhoff::analytical::analytical_common_mode_choke;
    // Lm = 1 mH, 5 A line current, 230 V mains (scaling no-op), 150 kHz dominant impedance frequency, 2 windings.
    const double Lm = 1e-3, iop = 5, vop = 230, fexc = 150000;
    MAS::OperatingPoint op = analytical_common_mode_choke(Lm, iop, vop, fexc);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Line + Neutral
    // Default (no parasitics): I_cm = 0.01 A × (230/230) — the post-Y-cap RESIDUAL CM current
    // (fd8d56f lowered the fallback from 100 mA raw injection, which false-saturated
    // high-permeability CM cores). Every winding = CM ripple (pp = 2·I_cm) on the line-current DC
    // bias → average = line current, small ripple.
    const double iCmPeak = 0.01;
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(iop).margin(0.05));           // DC = line I
    CHECK(*processed_current(op, 0).get_peak_to_peak() == Catch::Approx(2.0 * iCmPeak).margin(0.005));
    // MAS convention (2026-09-24): the return winding carries the line current back, so in the common dot
    // reference its DM level is the NEGATED line current (the DM ampere-turns cancel in the core); both
    // windings carry the identical CM ripple, so their rms (what MKF's can_be_common_mode_choke compares) match.
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-*processed_current(op, 0).get_average()).margin(0.01));
    CHECK(*processed_current(op, 1).get_rms() == Catch::Approx(*processed_current(op, 0).get_rms()).margin(0.01));
    // CM voltage present, peak = L·ω·Σ I_cm = n·L·ω·I_cm (v = L·d(i_m)/dt, i_m = Σ i_k; was L·ω·I_cm).
    REQUIRE(op.get_excitations_per_winding()[0].get_voltage().has_value());
    const double vCmPeak = 2.0 * Lm * 2.0 * M_PI * fexc * iCmPeak;                                // ≈ 18.85 V
    CHECK(*processed_voltage(op, 0).get_peak() == Catch::Approx(vCmPeak).margin(0.2));
}

TEST_CASE("analytical_common_mode_choke honors winding count and C·dV/dt CM current",
          "[analytical][component][cmc]") {
    using Kirchhoff::analytical::analytical_common_mode_choke;
    // 4 windings; explicit parasitics 100 pF × 10 V/ns → I_cm = 100·10·1e-3 = 1.0 A (× 230/230 = 1.0).
    MAS::OperatingPoint op = analytical_common_mode_choke(1e-3, 5, 230, 150000, 4, 100.0, 10.0);
    REQUIRE(op.get_excitations_per_winding().size() == 4);   // Phase A/B/C + Neutral
    CHECK(*processed_current(op, 0).get_peak_to_peak() == Catch::Approx(2.0).margin(0.05));   // 2·I_cm = 2.0
    // DM snapshot at phase A's peak (MAS convention, 2026-09-24): A = +5, B = C = −2.5, balanced neutral 0 —
    // Σ = 0 (the DM flux cancels). Was +5 A on every winding, incl. the neutral.
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(5.0).margin(0.05));
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-2.5).margin(0.05));
    CHECK(*processed_current(op, 2).get_average() == Catch::Approx(-2.5).margin(0.05));
    CHECK(*processed_current(op, 3).get_average() == Catch::Approx(0.0).margin(0.05));
}

TEST_CASE("analytical_common_mode_choke rejects bad winding count / non-positive inputs",
          "[analytical][component][cmc]") {
    using Kirchhoff::analytical::analytical_common_mode_choke;
    CHECK_THROWS(analytical_common_mode_choke(1e-3, 5, 230, 150000, 1));   // numberOfWindings < 2
    CHECK_THROWS(analytical_common_mode_choke(1e-3, 5, 230, 150000, 5));   // numberOfWindings > 4
    CHECK_THROWS(analytical_common_mode_choke(0.0,  5, 230, 150000));      // Lm = 0
    CHECK_THROWS(analytical_common_mode_choke(1e-3, 0, 230, 150000));      // operatingCurrent = 0
    CHECK_THROWS(analytical_common_mode_choke(1e-3, 5, 0.0, 150000));      // operatingVoltage = 0
    CHECK_THROWS(analytical_common_mode_choke(1e-3, 5, 230, 0.0));         // excitationFrequency = 0
}

// ─── Load-scaling regression guard ──────────────────────────────────────────
// The resonant solvers were once LOAD-BLIND (a faithful port of MKF's lossless TDA emitted a tank current
// independent of Iout — ~5x the SPICE value). That defect is now caught here: every current-carrying
// solver's primary current MUST increase with load. A future regression to a load-independent model fails.
TEST_CASE("all solvers scale with load (no load-blind regression)", "[analytical][scaling]") {
    using namespace Kirchhoff::analytical;
    auto rms0 = [](const MAS::OperatingPoint& op) {
        return *op.get_excitations_per_winding()[0].get_current()->get_processed()->get_rms();
    };
    const double n = 8.0, f = 95000, Lm = 4.9e-4, Lr1 = 1.1e-4, Cr1 = 2.3e-8, Lr2 = Lr1/(n*n), Cr2 = Cr1*n*n;
    // Resonant: primary tank rms at 4x load must be clearly larger than at 1x (magnetizing is load-
    // independent, reflected load scales — so > 1.3x is a conservative, always-true-if-load-aware bound).
    {
        double lo = rms0(analytical_llc(400,{12},{2.5},{n},f,Lm,Lr1,Cr1,0.5,SrcRectifier::CENTER_TAPPED));
        double hi = rms0(analytical_llc(400,{12},{10.0},{n},f,Lm,Lr1,Cr1,0.5,SrcRectifier::CENTER_TAPPED));
        CHECK(hi > 1.3 * lo);
    }
    {
        double lo = rms0(analytical_cllc(400,{48},{2.5},{n},f,Lm,Lr1,Cr1,Lr2,Cr2,1.0));
        double hi = rms0(analytical_cllc(400,{48},{10.0},{n},f,Lm,Lr1,Cr1,Lr2,Cr2,1.0));
        CHECK(hi > 1.3 * lo);
    }
    {
        double lo = rms0(analytical_clllc(400,{48},{2.5},{n},f,Lm,Lr1,Cr1,Lr2,Cr2,1.0));
        double hi = rms0(analytical_clllc(400,{48},{10.0},{n},f,Lm,Lr1,Cr1,Lr2,Cr2,1.0));
        CHECK(hi > 1.3 * lo);
    }
    // SRC (FHA, already load-aware).
    {
        double lo = rms0(analytical_src(400,{48},{2.5},{n},120000,40e-6,63.3e-9));
        double hi = rms0(analytical_src(400,{48},{10.0},{n},120000,40e-6,63.3e-9));
        CHECK(hi > 1.3 * lo);
    }
    // Vienna / PFC: boost-inductor peak scales with power.
    {
        double lo = *analytical_vienna(230,800,5000,50,70000,500e-6).get_excitations_per_winding()[0].get_current()->get_processed()->get_peak();
        double hi = *analytical_vienna(230,800,10000,50,70000,500e-6).get_excitations_per_winding()[0].get_current()->get_processed()->get_peak();
        CHECK(hi > 1.5 * lo);
    }
    {
        double lo = *analytical_pfc(230,400,1500,50,65000,500e-6).get_excitations_per_winding()[0].get_current()->get_processed()->get_peak();
        double hi = *analytical_pfc(230,400,3000,50,65000,500e-6).get_excitations_per_winding()[0].get_current()->get_processed()->get_peak();
        CHECK(hi > 1.5 * lo);
    }
    // Current transformer: secondary scales with primary (ampere-turn balance).
    {
        double lo = *analytical_current_transformer(MAS::WaveformLabel::SINUSOIDAL,50,100000,0.01,10.0).get_excitations_per_winding()[1].get_current()->get_processed()->get_peak();
        double hi = *analytical_current_transformer(MAS::WaveformLabel::SINUSOIDAL,100,100000,0.01,10.0).get_excitations_per_winding()[1].get_current()->get_processed()->get_peak();
        CHECK(hi > 1.8 * lo);
    }
    // Differential-mode choke: winding rms scales with the line current.
    {
        double lo = rms0(analytical_differential_mode_choke(100e-6,5,50,100000));
        double hi = rms0(analytical_differential_mode_choke(100e-6,10,50,100000));
        CHECK(hi > 1.8 * lo);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ABT #95 — configurable maximumDutyCycle gate (item 1) + maximumSwitchCurrent
// alternative inductance sizing (item 2).
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("analytical maximumDutyCycle gate throws when exceeded, byte-identical at default",
          "[analytical][solver][maxduty][abt95]") {
    using Kirchhoff::analytical::analytical_boost;
    using Kirchhoff::analytical::analytical_zeta;
    using Kirchhoff::analytical::analytical_fsbb;
    using Kirchhoff::analytical::analytical_weinberg;

    // Buck 12->5 (D=0.4167). Default (1.0) preserves the historical singularity-only guard: no throw.
    CHECK_NOTHROW(analytical_buck(12, 5, 2, 100000, 10e-6));                       // default maxDuty = 1.0
    CHECK_NOTHROW(analytical_buck(12, 5, 2, 100000, 10e-6, 0.0, 1.0, 0.5));        // 0.4167 < 0.5 → ok
    CHECK_THROWS(analytical_buck(12, 5, 2, 100000, 10e-6, 0.0, 1.0, 0.3));         // 0.4167 > 0.3 → throw

    // Boost 12->24 (D=0.5).
    CHECK_NOTHROW(analytical_boost(12, 24, 1, 100000, 20e-6));                     // default 1.0
    CHECK_THROWS(analytical_boost(12, 24, 1, 100000, 20e-6, 0.0, 1.0, 0.4));       // 0.5 > 0.4 → throw

    // Zeta 12->24 (D = 24/(24+12) = 0.667); default cap 0.95 lets it through, 0.6 rejects it.
    CHECK_NOTHROW(analytical_zeta(12, 24, 1, 100000, 47e-6));                      // default 0.95
    CHECK_THROWS(analytical_zeta(12, 24, 1, 100000, 47e-6, 0.0, 1.0, 0.6));        // 0.667 > 0.606 → throw

    // FSBB buck region 12->5 (D_buck=0.4167). Unlike the single-mode topologies above, a
    // saturated buck duty does not make the conversion infeasible for a 4-switch converter —
    // AUTO routes it to the SIMULTANEOUS (buck-boost) waveform, whose duty D = 5/17 = 0.29
    // sits under the same gate. So maxDuty=0.4 now yields the transition-mode waveform
    // (iL_avg = Iout/(1-D)), not a throw.
    CHECK_NOTHROW(analytical_fsbb(12, 5, 2, 100000, 10e-6));                       // default 0.95
    {
        MAS::OperatingPoint routed = analytical_fsbb(12, 5, 2, 100000, 10e-6, 1.0,
                                     Kirchhoff::analytical::FsbbMode::BUCK_BOOST_AUTO, 0.5, 0.4);  // 0.4167 >= 0.39 → SIMULTANEOUS
        CHECK(*processed_current(routed, 0).get_average() == Catch::Approx(2.0 / (1.0 - 5.0 / 17.0)).margin(0.05));
    }

    // Weinberg boost regime 24->72 (D=0.833); default 0.95 ok, 0.8 rejects.
    CHECK_NOTHROW(analytical_weinberg(24, 72, 2, 100000, 50e-6, 1));               // default 0.95
    CHECK_THROWS(analytical_weinberg(24, 72, 2, 100000, 50e-6, 1, 0.0, 1.0, false, 0.8));  // 0.833 >= 0.79
}

namespace {
// Minimal Kirchhoff design-input doc (η=1 ideal, ±5% Vin) for the sizing tests.
static nlohmann::json sizing_inputs(double vin, double vout, double pout, double fsw,
                                    const nlohmann::json& config = nlohmann::json::object()) {
    nlohmann::json d;
    d["designRequirements"]["efficiency"] = 1.0;
    d["designRequirements"]["inputVoltage"]["nominal"] = vin;
    d["designRequirements"]["inputVoltage"]["minimum"] = vin * 0.95;
    d["designRequirements"]["inputVoltage"]["maximum"] = vin * 1.05;
    d["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    nlohmann::json o; o["name"] = "out"; o["voltage"]["nominal"] = vout;
    d["designRequirements"]["outputs"] = nlohmann::json::array({o});
    nlohmann::json op; op["inputVoltage"] = vin;
    nlohmann::json oo; oo["power"] = pout;
    op["outputs"] = nlohmann::json::array({oo});
    d["operatingPoints"] = nlohmann::json::array({op});
    if (!config.empty()) d["config"] = config;
    return d;
}
}  // namespace

TEST_CASE("design_buck maximumSwitchCurrent sizes L so the peak inductor current hits the cap (ABT #95)",
          "[requirements][buck][maxswitch][abt95]") {
    const double vin = 24, vout = 12, pout = 24, fsw = 100000;   // iout = 2 A
    const double iout = pout / vout, vinMax = vin * 1.05;

    // No config → the ripple-ratio rule (byte-identical to before): ΔIL = 0.4·iout.
    auto dRef = Kirchhoff::design_buck(sizing_inputs(vin, vout, pout, fsw));
    const double refRipple = vout * (vinMax - vout) / (dRef.inductance * fsw * vinMax);
    CHECK(refRipple == Catch::Approx(0.4 * iout).epsilon(1e-6));

    // With the cap → L sized so peak = iout + ΔIL/2 lands exactly on maximumSwitchCurrent.
    const double cap = 3.0;   // > iout (2 A)
    auto dCap = Kirchhoff::design_buck(sizing_inputs(vin, vout, pout, fsw, {{"maximumSwitchCurrent", cap}}));
    const double capRipple = vout * (vinMax - vout) / (dCap.inductance * fsw * vinMax);
    CHECK(iout + capRipple / 2.0 == Catch::Approx(cap).epsilon(1e-6));   // the peak hits the cap
    CHECK(dCap.inductance != Catch::Approx(dRef.inductance));            // and differs from the ripple rule

    // A cap at/below the DC current is unreachable → loud throw (no silent fallback).
    CHECK_THROWS(Kirchhoff::design_buck(sizing_inputs(vin, vout, pout, fsw, {{"maximumSwitchCurrent", 1.5}})));
}

TEST_CASE("design_boost maximumSwitchCurrent sizes L so the peak inductor current hits the cap (ABT #95)",
          "[requirements][boost][maxswitch][abt95]") {
    const double vin = 12, vout = 24, pout = 24, fsw = 100000;   // step-up
    const double vinMax = vin * 1.05;
    const double iLavg = pout / (1.0 * vinMax);                  // input-inductor avg at the sizing corner (η=1)

    const double cap = 3.0;   // > iLavg (~1.905 A)
    auto dCap = Kirchhoff::design_boost(sizing_inputs(vin, vout, pout, fsw, {{"maximumSwitchCurrent", cap}}));
    const double capRipple = vinMax * (vout - vinMax) / (dCap.inductance * fsw * vout);
    CHECK(iLavg + capRipple / 2.0 == Catch::Approx(cap).epsilon(1e-6));

    // Ripple-ratio path unchanged when the cap is absent.
    auto dRef = Kirchhoff::design_boost(sizing_inputs(vin, vout, pout, fsw));
    const double refRipple = vinMax * (vout - vinMax) / (dRef.inductance * fsw * vout);
    CHECK(refRipple == Catch::Approx(0.4 * (pout / vout)).epsilon(1e-6));   // rippleRatio·Iout, as before

    CHECK_THROWS(Kirchhoff::design_boost(sizing_inputs(vin, vout, pout, fsw, {{"maximumSwitchCurrent", 1.0}})));
}

TEST_CASE("design_zeta maximumSwitchCurrent sizes L1 so its peak current hits the cap (ABT #95)",
          "[requirements][zeta][maxswitch][abt95]") {
    const double vin = 12, vout = 12, pout = 12, fsw = 100000;   // iout = 1 A
    const double iout = pout / vout, vinMax = vin * 1.05;
    // Synchronous rectifier → diodeDrop = 0, so the duty is the clean D = Vo/(Vin+Vo).
    nlohmann::json cfg = {{"rectifier", "synchronous"}, {"maximumSwitchCurrent", 2.0}};
    auto dCap = Kirchhoff::design_zeta(sizing_inputs(vin, vout, pout, fsw, cfg));
    const double dMax = vout / (vinMax + vout);
    const double iL1avg = iout * dMax / (1.0 - dMax);
    const double capRipple = vinMax * dMax / (dCap.inductanceL1 * fsw);
    CHECK(iL1avg + capRipple / 2.0 == Catch::Approx(2.0).epsilon(1e-6));

    // Without the cap, the L1 ripple follows the l1RippleRatio rule (0.4·iL1avg) — byte-identical.
    auto dRef = Kirchhoff::design_zeta(sizing_inputs(vin, vout, pout, fsw, {{"rectifier", "synchronous"}}));
    const double refRipple = vinMax * dMax / (dRef.inductanceL1 * fsw);
    CHECK(refRipple == Catch::Approx(0.4 * iL1avg).epsilon(1e-6));
}

TEST_CASE("analytical_flyback secondary voltage follows the dot/start-end convention (ABT #101)", "[analytical][solver][flyback]") {
    using Kirchhoff::analytical::analytical_flyback;
    // The ngspice extraction reads every winding as V(start)-V(end): the flyback secondary is
    // +Vin/n while the switch conducts and -(Vout+Vd) while the rectifier does. The analytical
    // view must agree so the web engine toggle no longer flips the trace sign.
    const double vin = 48, vout = 12, n = 2;
    // NB: MAS getters return optionals BY VALUE — copy the waveform out (a reference would
    // dangle) — and complete_excitation stores the RESAMPLED equidistant waveform, so sample
    // by index fraction, not by the original PWL corner times.
    auto secondary_v_at = [](const MAS::OperatingPoint& op, double frac) {
        const MAS::Waveform w = *op.get_excitations_per_winding().at(1).get_voltage()->get_waveform();
        const auto& data = w.get_data();
        REQUIRE(data.size() >= 8);
        return data.at(std::min(data.size() - 1, static_cast<size_t>(frac * data.size())));
    };
    // CCM (Lp = 200 uH)
    MAS::OperatingPoint ccm = analytical_flyback(vin, {vout}, {2}, {n}, 100000, 200e-6);
    const double dCcm = n * vout / (n * vout + vin);
    CHECK(secondary_v_at(ccm, dCcm * 0.5) == Catch::Approx(vin / n).epsilon(0.1));    // switch on: +24 V
    CHECK(secondary_v_at(ccm, dCcm + 0.5 * (1 - dCcm)) < 0.0);                        // rectifier on: -(Vout+Vd)
    // DCM (Lp = 10 uH): same signs during t_on / t_reset
    MAS::OperatingPoint dcm = analytical_flyback(vin, {vout}, {2}, {n}, 100000, 10e-6);
    CHECK(secondary_v_at(dcm, 0.05) == Catch::Approx(vin / n).epsilon(0.1));
    CHECK(secondary_v_at(dcm, 0.35) < 0.0);   // reset phase early in the period for this deep-DCM point
}

// Push-pull with a PINNED turns ratio (della-Pollock Pass 2): the centre-tapped push-pull needs a per-switch
// duty N·(Vo+Vd)/(2·Vin_min) <= 0.5. An infeasible pin must fail at design time naming the needed duty and
// the maximum turns ratio (it used to surface as the solver's "T1 cannot be larger than period/2"); a pinned
// maxDutyCycle that contradicts the pinned ratio throws too.
TEST_CASE("design_push_pull: pinned turns ratio feasibility and duty agreement", "[analytical][pushpull]") {
    nlohmann::json s;
    s["designRequirements"]["efficiency"] = 0.9;
    s["designRequirements"]["inputVoltage"] = {{"minimum", 36.0}, {"nominal", 48.0}, {"maximum", 60.0}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = 100000.0;
    s["designRequirements"]["outputs"] = nlohmann::json::array({{{"name", "out"}, {"voltage", {{"nominal", 12.0}}}}});
    s["operatingPoints"] = nlohmann::json::array({{{"inputVoltage", 48.0},
                                                   {"outputs", nlohmann::json::array({{{"power", 60.0}}})}}});
    CHECK_NOTHROW(Kirchhoff::design_push_pull(s));
    // N = 4: needs D = 4·(12+Vd)/72 > 0.5 at Vin_min = 36 V.
    nlohmann::json bad = s;
    bad["designRequirements"]["turnsRatios"] = nlohmann::json::array({1.0, 4.0});
    try {
        Kirchhoff::design_push_pull(bad);
        FAIL("an infeasible pinned turns ratio was accepted");
    } catch (const std::invalid_argument& e) {
        const std::string m = e.what();
        CHECK(m.find("per-switch duty") != std::string::npos);
        CHECK(m.find("maximum turns ratio") != std::string::npos);
    }
    // N = 2 needs D ~ 0.35 at 36 V; a pinned maxDutyCycle of 0.45 contradicts it.
    nlohmann::json clash = s;
    clash["designRequirements"]["turnsRatios"] = nlohmann::json::array({1.0, 2.0});
    CHECK_NOTHROW(Kirchhoff::design_push_pull(clash));
    clash["config"]["maxDutyCycle"] = 0.45;
    CHECK_THROWS_WITH(Kirchhoff::design_push_pull(clash), Catch::Matchers::ContainsSubstring("disagree"));
}

// ─── ABT #1503: LLC with a pinned turns ratio regulates by frequency, or refuses ─────────────────────────────
// A user's "I know the design" LLC: Q 0.3, Ln 4.8, n 3, Lm 29 uH, integrated Lr, full-bridge rectifier,
// full-bridge primary, band 100-300 kHz, fr 180 kHz, 425 V -> 90 V / 3.3 kW, efficiency 97 %. The engine drove
// every pinned-ratio LLC at fr, where the gain is 1 whatever the ratio: the transformer was clamped to ±n·Vout
// while the tank current came from a drive that delivers n=3 -> 142 V, so the primary v·i (5.2 kW) and the
// secondary (3.3 kW) described two different converters. It must now either solve the frequency at which the
// FHA gain meets the output, or throw.
#include "Llc.hpp"
#include "ComponentRequirements.hpp"

namespace {
nlohmann::json llc_1503_spec(double turnsRatio, const char* bridgeType) {
    nlohmann::json s;
    auto& dr = s["designRequirements"];
    dr["inputType"] = "dc";
    dr["inputVoltage"] = {{"nominal", 425.0}, {"minimum", 350.0}};
    dr["switchingFrequency"] = {{"nominal", 180e3}};
    dr["outputs"] = nlohmann::json::array({{{"name", "out"}, {"voltage", {{"nominal", 90.0}}}, {"regulation", "voltage"}}});
    dr["efficiency"] = 0.97;
    dr["magnetizingInductance"] = {{"nominal", 29e-6}};
    dr["turnsRatios"] = nlohmann::json::array({{{"nominal", turnsRatio}}});
    s["operatingPoints"] = nlohmann::json::array({{{"name", "full_load"}, {"inputVoltage", 425.0},
        {"ambientTemperature", 25.0}, {"outputs", nlohmann::json::array({{{"name", "out"}, {"power", 3300.0}}})}}});
    s["config"] = {{"qualityFactor", 0.3}, {"inductanceRatio", 4.8}, {"resonantBandMin", 100e3},
                   {"resonantBandMax", 300e3}, {"resonantFrequency", 180e3}, {"rectifierType", "fullBridge"},
                   {"bridgeType", bridgeType}, {"integratedResonantInductor", true}};
    return s;
}
// Cycle-average of v·i and rms of i over one winding's waveforms (both sampled on the same time grid).
struct WindingPower { double power, currentRms; };
WindingPower winding_power(const nlohmann::json& exc) {
    const auto& t = exc.at("current").at("waveform").at("time");
    const auto& i = exc.at("current").at("waveform").at("data");
    const auto& v = exc.at("voltage").at("waveform").at("data");
    REQUIRE(t.size() == i.size());
    REQUIRE(v.size() == i.size());
    double p = 0.0, i2 = 0.0;
    for (size_t k = 1; k < t.size(); ++k) {
        const double dt = t[k].get<double>() - t[k - 1].get<double>();
        p  += 0.5 * dt * (v[k].get<double>() * i[k].get<double>() + v[k - 1].get<double>() * i[k - 1].get<double>());
        i2 += 0.5 * dt * (i[k].get<double>() * i[k].get<double>() + i[k - 1].get<double>() * i[k - 1].get<double>());
    }
    const double T = t.back().get<double>() - t.front().get<double>();
    return {p / T, std::sqrt(i2 / T)};
}
}  // namespace

TEST_CASE("LLC pinned n=3, full bridge, 425 V -> 90 V: no frequency in the band reaches it, so it throws (ABT #1503)",
          "[analytical][llc][abt1503]") {
    // Needed gain n·(Vout+2Vd)/(η·Vin) ≈ 0.67; the tank reaches only ≈[0.83, 1.40] over 100–300 kHz.
    const std::string out = Kirchhoff::api::process_converter("llc", llc_1503_spec(3.0, "fullBridge").dump(), "analytical");
    INFO(out.substr(0, 600));
    REQUIRE(out.rfind("Exception:", 0) == 0);
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("pinned turns ratio 3 cannot deliver 90 V from 425 V"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("needs a tank gain of 0.668"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("only reaches [0.830"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("[100000, 300000] Hz"));
}

TEST_CASE("LLC pinned turns ratio: the operating point is solved in the band, meets Vout and balances power (ABT #1503)",
          "[analytical][llc][abt1503]") {
    // Two reachable variants of the user's design: the same n=3 behind a HALF-bridge primary (needs M≈1.34,
    // below resonance) and n=4 behind the full bridge (needs M≈0.89, above resonance).
    struct Case { double n; const char* bridge; double k; bool belowResonance; };
    for (const Case c : {Case{3.0, "halfBridge", 0.5, true}, Case{4.0, "fullBridge", 1.0, false}}) {
        DYNAMIC_SECTION("n=" << c.n << " " << c.bridge) {
            const nlohmann::json spec = llc_1503_spec(c.n, c.bridge);
            const Kirchhoff::LlcDesign d = Kirchhoff::design_llc(spec);
            const double Vin = 425.0, Vout = 90.0, Pout = 3300.0, eta = 0.97, Iout = Pout / Vout;
            const double Vd = Kirchhoff::req::rectifier_drop(spec.at("config"), Iout);

            // The tank resonates at the requested 180 kHz: Lr = Lm/Ln, Cr = 1/((2π·180k)²·Lr) = 129.4 nF.
            CHECK(d.resonantFrequency == Catch::Approx(180e3));
            CHECK(d.resonantCapacitance == Catch::Approx(129.4e-9).epsilon(0.002));
            // The operating frequency is inside the band, on the correct side of resonance ...
            CHECK(d.operatingFrequency >= 100e3);
            CHECK(d.operatingFrequency <= 300e3);
            CHECK((d.operatingFrequency < d.resonantFrequency) == c.belowResonance);
            // ... and the FHA gain there delivers the output: Vout = M·η·k·Vin/n − 2·Vd.
            const double Rac = 8.0 / (M_PI * M_PI) * c.n * c.n * Vout / Iout;
            const double M = Kirchhoff::analytical::llc_fha_tank(d.operatingFrequency, d.magnetizingInductance,
                                                                 d.resonantInductance, d.resonantCapacitance, Rac).gain();
            CHECK(M * eta * c.k * Vin / c.n - 2.0 * Vd == Catch::Approx(Vout).epsilon(0.001));

            const std::string raw = Kirchhoff::api::process_converter("llc", spec.dump(), "analytical");
            INFO(raw.substr(0, 400));
            REQUIRE(raw.rfind("Exception:", 0) != 0);
            const nlohmann::json out = nlohmann::json::parse(raw);
            // The diagnostics report the frequency the converter runs at, and the 180 kHz tank.
            CHECK(out.at("diagnostics").at("switchingFrequency").get<double>() == Catch::Approx(d.operatingFrequency));
            CHECK(out.at("diagnostics").at("computed").at("resonantCapacitance").get<double>() ==
                  Catch::Approx(129.4e-9).epsilon(0.002));

            const auto& exc = out.at("operatingPoint").at("excitationsPerWinding");
            REQUIRE(exc.size() == 2);   // primary + one full-bridge secondary
            const WindingPower pri = winding_power(exc.at(0));
            const WindingPower sec = winding_power(exc.at(1));
            INFO("fsw " << d.operatingFrequency << " Hz, P_pri " << pri.power << " W, P_sec " << sec.power
                 << " W, I_sec,rms " << sec.currentRms << " A, Vd " << Vd << " V");
            // Secondary: the rectifier delivers Pout (its current is scaled to Iout; the residual is the part of
            // the reflected current that FHA puts outside the voltage's polarity — measured ≈1 %).
            CHECK(sec.power == Catch::Approx(Pout).epsilon(0.02));
            // Primary: the transformer carries Pout/η plus the rectifier conduction loss (the gain was solved
            // for Vout + 2·Vd), i.e. Pout/η·(1 + 2·Vd/Vout); FHA's sinusoidal current against the square winding
            // voltage and the triangular magnetizing current leave ≈1 % — 2 % tolerance.
            CHECK(pri.power == Catch::Approx(Pout / eta * (1.0 + 2.0 * Vd / Vout)).epsilon(0.02));
            // A full-bridge rectifier's winding current is ≈ the half-sine family: rms = π/(2√2)·Iout ≈ 40.7 A.
            CHECK(sec.currentRms == Catch::Approx(M_PI / (2.0 * std::sqrt(2.0)) * Iout).epsilon(0.03));
        }
    }
}

// ─── ABT #1503 (CLLC / CLLLC): the two-sided resonant tanks regulate by frequency too, or refuse ──────────────
// Both were always driven at fr = the requested switching frequency, where a symmetric tank's FHA gain is 1 for any
// ratio. The CLLC's engine-sized ratio carries a 1.08 gain headroom (at fr it would deliver 1.08·Vout) and a pinned
// ratio of either is whatever the magnetic realised, so the transformer was clamped to ±n·Vout while the tank
// current came from a drive delivering a different voltage. Now the operating frequency is solved from the FHA gain
// inside the switching band, or the design throws naming the required gain, the reachable range and a working ratio.
#include "Cllc.hpp"
#include "Clllc.hpp"

namespace {
// 400 V -> 48 V / 480 W, tank resonance 100 kHz, efficiency 95 %, band 70-200 kHz. turnsRatio <= 0: engine-sized.
nlohmann::json two_sided_1503_spec(double turnsRatio) {
    nlohmann::json s;
    auto& dr = s["designRequirements"];
    dr["inputType"] = "dc";
    dr["inputVoltage"] = {{"nominal", 400.0}, {"minimum", 380.0}, {"maximum", 420.0}};
    dr["switchingFrequency"] = {{"nominal", 100e3}};
    dr["outputs"] = nlohmann::json::array({{{"name", "out"}, {"voltage", {{"nominal", 48.0}}}, {"regulation", "voltage"}}});
    dr["efficiency"] = 0.95;
    if (turnsRatio > 0) dr["turnsRatios"] = nlohmann::json::array({{{"nominal", turnsRatio}}});
    s["operatingPoints"] = nlohmann::json::array({{{"name", "full_load"}, {"inputVoltage", 400.0},
        {"ambientTemperature", 25.0}, {"outputs", nlohmann::json::array({{{"name", "out"}, {"power", 480.0}}})}}});
    s["config"] = {{"minSwitchingFrequency", 70e3}, {"maxSwitchingFrequency", 200e3}};
    return s;
}

// The design's FHA gain at its operating frequency (forward power flow), from the shared tank model.
template <class D>
double two_sided_gain_at_operating_frequency(const D& d) {
    const double N = d.turnsRatio;
    const double Rac = 8.0 / (M_PI * M_PI) * N * N * d.outputVoltage * d.outputVoltage / d.outputPower;
    return Kirchhoff::analytical::cllc_fha_tank(d.operatingFrequency, d.magnetizingInductance,
                                                d.primaryResonantInductance, d.primaryResonantCapacitance,
                                                d.secondaryResonantInductance * N * N,
                                                d.secondaryResonantCapacitance / (N * N), Rac).gain();
}

// Solve checks shared by both topologies: in band, on the expected side of fr, gain meets Vout; then the emitted
// transformer excitations balance power and the secondary rms is the full-bridge rectifier's.
template <class D>
void check_two_sided_operating_point(const char* topology, const D& d, const nlohmann::json& spec, bool belowResonance) {
    const double Vin = 400.0, Vout = 48.0, Pout = 480.0, eta = 0.95, Iout = Pout / Vout;
    CHECK(d.resonantFrequency == Catch::Approx(100e3));
    CHECK(d.operatingFrequency >= 70e3);
    CHECK(d.operatingFrequency <= 200e3);
    CHECK((d.operatingFrequency < d.resonantFrequency) == belowResonance);
    // Vout = M·η·Vin/N (full bridge both sides, synchronous rectifier: no diode drop).
    const double M = two_sided_gain_at_operating_frequency(d);
    CHECK(M == Catch::Approx(d.requiredGain).epsilon(1e-6));
    CHECK(M * eta * Vin / d.turnsRatio == Catch::Approx(Vout).epsilon(0.001));

    const std::string raw = Kirchhoff::api::process_converter(topology, spec.dump(), "analytical");
    INFO(raw.substr(0, 400));
    REQUIRE(raw.rfind("Exception:", 0) != 0);
    const nlohmann::json out = nlohmann::json::parse(raw);
    CHECK(out.at("diagnostics").at("switchingFrequency").get<double>() == Catch::Approx(d.operatingFrequency));
    const auto& exc = out.at("operatingPoint").at("excitationsPerWinding");
    REQUIRE(exc.size() == 2);   // primary + one full-bridge secondary
    const WindingPower pri = winding_power(exc.at(0));
    const WindingPower sec = winding_power(exc.at(1));
    INFO(topology << ": n " << d.turnsRatio << ", fsw " << d.operatingFrequency << " Hz (fr " << d.resonantFrequency
         << "), M " << M << ", P_pri " << pri.power << " W, P_sec " << sec.power << " W, I_sec,rms " << sec.currentRms);
    // Secondary: the synchronous rectifier delivers Pout (its current is scaled to Iout).
    CHECK(sec.power == Catch::Approx(Pout).epsilon(0.02));
    // Primary: the gain was solved for Vout from η·Vin, so the transformer carries Pout/η. The residual is FHA's
    // sinusoidal current against the square winding voltage (second order in the secondary-tank reactance,
    // Rac/|Zsec| ≈ 1 − (Xsec/Rac)²/2) and the triangular magnetizing current — 2 % tolerance, as the LLC.
    CHECK(pri.power == Catch::Approx(Pout / eta).epsilon(0.02));
    // A full-bridge rectifier's winding current is ≈ a sinusoid of mean |i| = Iout: rms = π/(2√2)·Iout ≈ 11.1 A.
    CHECK(sec.currentRms == Catch::Approx(M_PI / (2.0 * std::sqrt(2.0)) * Iout).epsilon(0.03));
}
}  // namespace

TEST_CASE("CLLC: the operating frequency is solved from the FHA gain in the band, meets Vout and balances power "
          "(ABT #1503)", "[analytical][cllc][abt1503]") {
    SECTION("engine-sized ratio with the 1.08 gain headroom runs above resonance at gain 1/1.08") {
        const nlohmann::json spec = two_sided_1503_spec(0.0);
        const Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(spec);
        CHECK(d.turnsRatio == Catch::Approx(0.95 * 400.0 / (1.08 * 48.0)));
        CHECK(d.requiredGain == Catch::Approx(1.0 / 1.08));
        check_two_sided_operating_point("cllc", d, spec, false);
    }
    SECTION("pinned n=8.5 needs gain 1.07, below resonance") {
        const nlohmann::json spec = two_sided_1503_spec(8.5);
        const Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(spec);
        CHECK(d.requiredGain == Catch::Approx(8.5 * 48.0 / (0.95 * 400.0)));
        check_two_sided_operating_point("cllc", d, spec, true);
    }
}

TEST_CASE("CLLC pinned n=12, 400 V -> 48 V: no frequency in the band reaches gain 1.52, so it throws (ABT #1503)",
          "[analytical][cllc][abt1503]") {
    const std::string out = Kirchhoff::api::process_converter("cllc", two_sided_1503_spec(12.0).dump(), "analytical");
    INFO(out.substr(0, 800));
    REQUIRE(out.rfind("Exception:", 0) == 0);
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("pinned turns ratio 12 cannot deliver 48 V from 400 V"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("needs a tank gain of 1.51579"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("[70000, 200000] Hz only reaches ["));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("A turns ratio of 7.91667 gives the output at the tank resonance (100000 Hz)"));
}

TEST_CASE("CLLLC: the unity-gain ratio stays at resonance; a pinned ratio solves its frequency in the band (ABT #1503)",
          "[analytical][clllc][abt1503]") {
    SECTION("engine-sized ratio N = η·Vin/Vout is the unity-gain ratio: runs at fr") {
        const Kirchhoff::ClllcDesign d = Kirchhoff::design_clllc(two_sided_1503_spec(0.0));
        CHECK(d.requiredGain == Catch::Approx(1.0));
        CHECK(d.operatingFrequency == d.resonantFrequency);
        check_two_sided_operating_point("clllc", d, two_sided_1503_spec(0.0), false);
    }
    SECTION("pinned N=7.5 needs gain 0.947, above resonance") {
        const nlohmann::json spec = two_sided_1503_spec(7.5);
        const Kirchhoff::ClllcDesign d = Kirchhoff::design_clllc(spec);
        CHECK(d.requiredGain == Catch::Approx(7.5 * 48.0 / (0.95 * 400.0)));
        check_two_sided_operating_point("clllc", d, spec, false);
    }
    SECTION("pinned N=8.05 needs gain 1.017, below resonance (this Q=0.4, K=6 tank peaks at ~1.03)") {
        const nlohmann::json spec = two_sided_1503_spec(8.05);
        check_two_sided_operating_point("clllc", Kirchhoff::design_clllc(spec), spec, true);
    }
}

TEST_CASE("CLLLC pinned N=4, 400 V -> 48 V: no frequency in the band reaches gain 0.505, so it throws (ABT #1503)",
          "[analytical][clllc][abt1503]") {
    const std::string out = Kirchhoff::api::process_converter("clllc", two_sided_1503_spec(4.0).dump(), "analytical");
    INFO(out.substr(0, 800));
    REQUIRE(out.rfind("Exception:", 0) == 0);
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("pinned turns ratio 4 cannot deliver 48 V from 400 V"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("needs a tank gain of 0.505263"));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("[70000, 200000] Hz only reaches ["));
    CHECK_THAT(out, Catch::Matchers::ContainsSubstring("A turns ratio of 7.91667 gives the output at the tank resonance (100000 Hz)"));
}

TEST_CASE("CLLC reverse power flow: the Vout-side drive solves its own frequency and balances power (ABT #1503)",
          "[analytical][cllc][reverse][abt1503]") {
    // Reverse: the 48 V side drives Lr2/Cr2 with Lm/N² and delivers 400 V through 1/N. The engine-sized ratio
    // (N = η·Vin/(1.08·Vout)) needs gain Vin/(η·N·Vout) = 1.08/η² there — below resonance.
    nlohmann::json spec = two_sided_1503_spec(0.0);
    spec["config"]["powerFlowDirection"] = "reverse";
    const Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(spec);
    const double Vin = 400.0, Vout = 48.0, P = 480.0, eta = 0.95, N = d.turnsRatio;
    CHECK(d.requiredGain == Catch::Approx(Vin / (eta * N * Vout)));
    CHECK(d.requiredGain == Catch::Approx(1.08 / (eta * eta)));
    CHECK(d.operatingFrequency >= 70e3);
    CHECK(d.operatingFrequency < d.resonantFrequency);
    const double RacLv = 8.0 / (M_PI * M_PI) * Vin * Vin / (N * N * P);
    const auto tank = Kirchhoff::analytical::cllc_fha_tank(d.operatingFrequency, d.magnetizingInductance / (N * N),
                                                          d.secondaryResonantInductance, d.secondaryResonantCapacitance,
                                                          d.primaryResonantInductance / (N * N),
                                                          d.primaryResonantCapacitance * N * N, RacLv);
    const double M = tank.gain();
    // Receiving-tank power factor: the model's square winding voltage is in phase with Lm's voltage, the rectifier
    // current with Rac; they differ by arg(Zsec), negligible near fr but not 28 % below it.
    const double pfRecv = tank.zsecRe / std::hypot(tank.zsecRe, tank.zsecIm);
    CHECK(M * eta * N * Vout == Catch::Approx(Vin).epsilon(0.001));

    const std::string raw = Kirchhoff::api::process_converter("cllc", spec.dump(), "analytical");
    INFO(raw.substr(0, 400));
    REQUIRE(raw.rfind("Exception:", 0) != 0);
    const nlohmann::json out = nlohmann::json::parse(raw);
    CHECK(out.at("diagnostics").at("switchingFrequency").get<double>() == Catch::Approx(d.operatingFrequency));
    const auto& exc = out.at("operatingPoint").at("excitationsPerWinding");
    REQUIRE(exc.size() == 2);
    const WindingPower hv = winding_power(exc.at(0));   // receiver (Vin winding)
    const WindingPower lv = winding_power(exc.at(1));   // driver (Vout winding)
    INFO("fsw " << d.operatingFrequency << " Hz, M " << M << ", P_hv " << hv.power << " W, P_lv " << lv.power
         << " W, I_hv,rms " << hv.currentRms);
    // The driven winding carries P/η. The receiving winding's v·i is P·cos(arg Zsec) (its current is scaled to the
    // delivered DC current; its square voltage leads it by the receiving tank's angle). Solved ~28 % below fr the
    // receiving tank is off resonance, so that factor is ~0.988 here (≈1 at the forward points above).
    INFO("receiving-tank power factor " << pfRecv);
    CHECK(std::abs(lv.power) == Catch::Approx(P / eta).epsilon(0.02));
    CHECK(std::abs(hv.power) == Catch::Approx(P * pfRecv).epsilon(0.02));
    CHECK(hv.currentRms == Catch::Approx(M_PI / (2.0 * std::sqrt(2.0)) * P / Vin).epsilon(0.03));
}

// ── ABT #1525: the CLLC/CLLLC ngspice deck must rectify on the receiving bridge, at any operating frequency ──
namespace {
nlohmann::json two_sided_1525_spec(double turnsRatio, double efficiency) {
    nlohmann::json s;
    auto& dr = s["designRequirements"];
    dr["inputType"] = "dc";
    dr["inputVoltage"] = {{"nominal", 400.0}, {"minimum", 380.0}, {"maximum", 420.0}};
    dr["switchingFrequency"] = {{"nominal", 120e3}};
    dr["outputs"] = nlohmann::json::array({{{"name", "out"}, {"voltage", {{"nominal", 400.0}}}, {"regulation", "voltage"}}});
    dr["efficiency"] = efficiency;
    if (turnsRatio > 0) dr["turnsRatios"] = nlohmann::json::array({{{"nominal", turnsRatio}}});
    s["operatingPoints"] = nlohmann::json::array({{{"name", "full_load"}, {"inputVoltage", 400.0},
        {"ambientTemperature", 25.0}, {"outputs", nlohmann::json::array({{{"name", "out"}, {"power", 3300.0}}})}}});
    s["config"] = {{"resonantBandMin", 80e3}, {"resonantBandMax", 200e3}, {"qualityFactor", 0.4}, {"inductanceRatio", 5.0}};
    return s;
}
}  // namespace

#include "TasAssembler.hpp"
#include "Cllc.hpp"
#include "Clllc.hpp"
#include "NgspiceRunner.hpp"

namespace {
// The design's own deck, run in-process (libngspice): output voltage and the source / load power averaged over
// the last 20 switching periods. `deliveredNode` is Vout forward, Vin in reverse (the HV rail is the load).
struct TwoSidedDeckRun { double vDelivered, pSource, pLoad; };
TwoSidedDeckRun run_two_sided_deck(const nlohmann::json& tas, double fsw, double vSource, const char* sourceBranch,
                                   const char* deliveredNode, double loadResistance, double stopTime = 0.016) {
    // 16 ms: past the output capacitor's start-up ring against the tank (~0.5 kHz, decaying over ~10 ms), so the
    // averages are the steady state rather than a phase of that ring.
    nlohmann::json settled = tas;
    for (auto& an : settled.at("simulation").at("analyses"))
        if (an.value("type", "") == "transient") an["stopTime"] = stopTime;
    const std::string deck = Kirchhoff::tas_to_ngspice(settled, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    const Kirchhoff::NgspiceRunResult r = Kirchhoff::run_ngspice_in_process(deck);
    REQUIRE(r.success);
    const double tEnd = r.time.back(), from = tEnd - 20.0 / fsw;
    const auto v = r.average(deliveredNode, from, tEnd);
    const auto i = r.average(sourceBranch, from, tEnd);
    REQUIRE(v.has_value());
    REQUIRE(i.has_value());
    // Load power as the average of v·i: in reverse the delivered HV rail has no filter capacitor, so its voltage
    // is a rectified waveform and V_avg²/R understates what the load takes.
    const auto& t = r.time;
    const std::vector<double>* vl = nullptr;
    for (const auto& kv : r.vectors) if (kv.first == deliveredNode) vl = &kv.second;
    REQUIRE(vl != nullptr);
    double pLoad = 0.0, span = 0.0;
    for (size_t k = 1; k < t.size(); ++k) {
        if (t[k - 1] < from) continue;
        const double dt = t[k] - t[k - 1];
        pLoad += 0.5 * dt * ((*vl)[k] * (*vl)[k] + (*vl)[k - 1] * (*vl)[k - 1]) / loadResistance;
        span += dt;
    }
    // A DC source's branch current is positive INTO its + terminal: a sourcing supply reads negative.
    return {*v, -*i * vSource, pLoad / span};
}

// ABT #1525 regression: the ngspice path of a CLLC/CLLLC design, at whatever frequency the design runs.
//  - the deck regulates to the spec open loop (within the equivalence suite's kSpecTol, 6 %),
//  - power flows forward through the transformer on both windings, and balances with the analytical operating
//    point taken to the deck's output voltage (5 %: the deck's receiving bridge rectifies through ideal diodes
//    where the analytical model has an ideal SR, and the FHA sinusoid stands in for the simulated tank current).
//    The analytical point sits at the spec Vout with the design's efficiency losses; the ideal-part deck has
//    none of those losses and settles where its tank gain puts it (e.g. the unity-gain CLLLC at Vin/N = 412 V
//    for a 400 V spec at η = 0.97), so the analytical winding powers are scaled by (Vout_deck / Vout_spec)²,
//    the load power ratio of the resistive load at the two voltages,
//  - the deck's source-to-load efficiency is plausible (ideal parts: above 90 %, and no energy created).
template <class D>
void check_two_sided_ngspice(const char* topology, const D& d, const nlohmann::json& spec, const nlohmann::json& tas,
                             double stopTime = 0.016) {
    const double Vin = 400.0, Vout = 400.0;
    const TwoSidedDeckRun deck = run_two_sided_deck(tas, d.operatingFrequency, Vin, "vvin#branch", "vout",
                                                    d.loadResistance, stopTime);
    const std::string rawSim = Kirchhoff::api::process_converter(topology, spec.dump(), "ngspice");
    const std::string rawAn  = Kirchhoff::api::process_converter(topology, spec.dump(), "analytical");
    INFO(rawSim.substr(0, 400));
    REQUIRE(rawSim.rfind("Exception:", 0) != 0);
    REQUIRE(rawAn.rfind("Exception:", 0) != 0);
    const nlohmann::json simOut = nlohmann::json::parse(rawSim), anOut = nlohmann::json::parse(rawAn);
    const auto& simExc = simOut.at("operatingPoint").at("excitationsPerWinding");
    const auto& anExc  = anOut.at("operatingPoint").at("excitationsPerWinding");
    REQUIRE(simExc.size() == 2);
    REQUIRE(anExc.size() == 2);
    const double simPri = winding_power(simExc.at(0)).power, simSec = winding_power(simExc.at(1)).power;
    const double anPri  = winding_power(anExc.at(0)).power,  anSec  = winding_power(anExc.at(1)).power;
    INFO(topology << ": n " << d.turnsRatio << ", fsw " << d.operatingFrequency << " Hz (fr " << d.resonantFrequency
         << "), deck Vout " << deck.vDelivered << " V, Psource " << deck.pSource << " W, Pload " << deck.pLoad
         << " W; ngspice P_pri " << simPri << " W, P_sec " << simSec << " W; analytical P_pri " << anPri
         << " W, P_sec " << anSec << " W");
    CHECK(deck.vDelivered == Catch::Approx(Vout).epsilon(0.06));
    CHECK(simPri > 0.0);
    CHECK(simSec > 0.0);
    const double loadScale = (deck.vDelivered / Vout) * (deck.vDelivered / Vout);
    CHECK(simPri == Catch::Approx(anPri * loadScale).epsilon(0.05));
    CHECK(simSec == Catch::Approx(anSec * loadScale).epsilon(0.05));
    CHECK(simSec == Catch::Approx(simPri).epsilon(0.03));   // an ideal transformer passes the power through
    const double eff = deck.pLoad / deck.pSource;
    CHECK(eff > 0.90);
    CHECK(eff <= 1.0);
}
}  // namespace

TEST_CASE("CLLC ngspice: the receiving bridge rectifies, so power is forward and balanced off resonance (ABT #1525)",
          "[ngspice][cllc][abt1525]") {
    SECTION("engine-sized ratio: runs above resonance") {
        const auto spec = two_sided_1525_spec(0.0, 0.97);
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.operatingFrequency > 1.1 * d.resonantFrequency);
        check_two_sided_ngspice("cllc", d, spec, Kirchhoff::build_cllc_tas(d));
    }
    SECTION("pinned n=0.9: runs above resonance") {
        const auto spec = two_sided_1525_spec(0.9, 0.97);
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.operatingFrequency > 1.1 * d.resonantFrequency);
        check_two_sided_ngspice("cllc", d, spec, Kirchhoff::build_cllc_tas(d));
    }
    SECTION("n=1, efficiency 1: runs at resonance") {
        const auto spec = two_sided_1525_spec(1.0, 1.0);
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.operatingFrequency == d.resonantFrequency);
        check_two_sided_ngspice("cllc", d, spec, Kirchhoff::build_cllc_tas(d));
    }
}

TEST_CASE("CLLLC ngspice: the receiving bridge rectifies, forward power balances (ABT #1525)",
          "[ngspice][clllc][abt1525]") {
    SECTION("engine-sized ratio (unity gain, at resonance)") {
        const auto spec = two_sided_1525_spec(0.0, 0.97);
        const auto d = Kirchhoff::design_clllc(spec);
        check_two_sided_ngspice("clllc", d, spec, Kirchhoff::build_clllc_tas(d));
    }
    SECTION("pinned N=0.9: off resonance") {
        const auto spec = two_sided_1525_spec(0.9, 0.97);
        const auto d = Kirchhoff::design_clllc(spec);
        CHECK(std::abs(d.operatingFrequency - d.resonantFrequency) > 0.05 * d.resonantFrequency);
        check_two_sided_ngspice("clllc", d, spec, Kirchhoff::build_clllc_tas(d));
    }
}

TEST_CASE("CLLC / CLLLC reverse ngspice: the HV bridge rectifies what the LV bridge drives (ABT #1525)",
          "[ngspice][cllc][clllc][reverse][abt1525]") {
    // n = 1: the engine-sized ratio (sized for forward, with the 1.08 headroom) cannot reach 400 V in reverse in
    // this band, and design_cllc says so (ABT #1503); n = 1 needs gain 1/0.97, just below resonance.
    auto reverse_spec = [] { auto s = two_sided_1525_spec(1.0, 0.97);
        s["config"]["powerFlowDirection"] = "reverse"; return s; };
    auto check = [](double fsw, double fr, double loadR, const nlohmann::json& tas) {
        // Reverse: the Vout rail (400 V here) sources, the Vin rail (400 V) is delivered.
        const TwoSidedDeckRun r = run_two_sided_deck(tas, fsw, 400.0, "vvout#branch", "vin", loadR);
        INFO("fsw " << fsw << " (fr " << fr << "), delivered " << r.vDelivered << " V, Psource " << r.pSource
             << " W, Pload " << r.pLoad << " W");
        // The reverse deck has no capacitor on the delivered HV rail (its builders put the only output capacitor
        // on the LV side), so the HV rail carries the rectified tank voltage straight into the load and its
        // average is not the FHA's filtered output: check genuine delivery (as the ABT #85 equivalence test
        // does) and the energy balance, which a bridge conducting against the current breaks.
        CHECK(r.vDelivered > 0.6 * 400.0);
        CHECK(r.vDelivered < 1.15 * 400.0);
        CHECK(r.pSource > 0.5 * 3300.0);
        CHECK(r.pLoad / r.pSource > 0.90);
        CHECK(r.pLoad / r.pSource <= 1.0);
    };
    const double loadR = 400.0 * 400.0 / 3300.0;
    SECTION("CLLC") {
        const auto d = Kirchhoff::design_cllc(reverse_spec());
        REQUIRE(d.reverse);
        check(d.operatingFrequency, d.resonantFrequency, loadR, Kirchhoff::build_cllc_tas(d));
    }
    SECTION("CLLLC") {
        const auto d = Kirchhoff::design_clllc(reverse_spec());
        REQUIRE(d.reverse);
        check(d.operatingFrequency, d.resonantFrequency, loadR, Kirchhoff::build_clllc_tas(d));
    }
}

// ── ABT #1537 / #1538: a pinned turns ratio and pinned tank values reach the design, the analytical point and the
// ngspice deck ──────────────────────────────────────────────────────────────────────────────────────────────────
namespace {
// The value an element of the ngspice deck carries: the first line whose element name is `name` (SPICE letter
// prefix optional, a single-winding magnetic's "_pri" winding suffix optional, case-insensitive), last numeric
// token with its SPICE scale suffix.
double deck_element_value(const std::string& deck, const std::string& name) {
    auto lower = [](std::string s) { for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c))); return s; };
    const std::string want = lower(name);
    std::istringstream in(deck);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string head;
        if (!(ls >> head)) continue;
        const std::string h = lower(head);
        // A capacitor is "C<name>"; a single-winding magnetic is its primary winding "L<name>_pri".
        if (h != want && h.substr(1) != want && h.substr(1) != want + "_pri") continue;
        std::vector<std::string> toks;
        for (std::string t; ls >> t;) toks.push_back(t);
        for (auto it = toks.rbegin(); it != toks.rend(); ++it) {
            std::string t = lower(*it);
            size_t pos = 0;
            double v = 0.0;
            try { v = std::stod(t, &pos); } catch (...) { continue; }
            const std::string suf = t.substr(pos);
            double scale = 1.0;
            if (suf.rfind("meg", 0) == 0) scale = 1e6;
            else if (!suf.empty()) switch (suf[0]) {
                case 'f': scale = 1e-15; break; case 'p': scale = 1e-12; break; case 'n': scale = 1e-9; break;
                case 'u': scale = 1e-6; break;  case 'm': scale = 1e-3; break;  case 'k': scale = 1e3; break;
                case 'g': scale = 1e9; break;   case 't': scale = 1e12; break;  default: continue;
            }
            return v * scale;
        }
    }
    std::string near;
    { std::istringstream all(deck); std::string l;
      while (std::getline(all, l)) if (lower(l).find(want) != std::string::npos) near += "\n  " + l; }
    FAIL("deck has no element " << name << "; lines mentioning it:" << near);
    return 0.0;
}

// The ABT #1537 web case: 400 V -> 400 V / 3.3 kW, fr 120 kHz, band 80-200 kHz, Q 0.4, k 5, efficiency 0.97.
nlohmann::json cllc_web_spec(double turnsRatio, double pinnedLm) {
    nlohmann::json s = two_sided_1525_spec(turnsRatio, 0.97);
    if (pinnedLm > 0) s["designRequirements"]["magnetizingInductance"] = {{"nominal", pinnedLm}};
    return s;
}

std::string deck_of(const nlohmann::json& tas) {
    return Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
}

// The tank values of a design reach the deck verbatim.
template <class D>
void check_tank_in_deck(const D& d, const nlohmann::json& tas) {
    const std::string deck = deck_of(tas);
    CHECK(deck_element_value(deck, "Lr1") == Catch::Approx(d.primaryResonantInductance).epsilon(1e-4));
    CHECK(deck_element_value(deck, "Cr1") == Catch::Approx(d.primaryResonantCapacitance).epsilon(1e-4));
    CHECK(deck_element_value(deck, "Lr2") == Catch::Approx(d.secondaryResonantInductance).epsilon(1e-4));
    CHECK(deck_element_value(deck, "Cr2") == Catch::Approx(d.secondaryResonantCapacitance).epsilon(1e-4));
}

// The design's FHA gain at its operating frequency meets the requirement, and the analytical operating point
// balances power: the secondary delivers Pout, the primary carries Pout/η (2 %, as the #1503 checks).
template <class D>
void check_two_sided_solved(const char* topology, const D& d, const nlohmann::json& spec, double Pout, double eta) {
    CHECK(two_sided_gain_at_operating_frequency(d) == Catch::Approx(d.requiredGain).epsilon(1e-6));
    const std::string raw = Kirchhoff::api::process_converter(topology, spec.dump(), "analytical");
    INFO(raw.substr(0, 600));
    REQUIRE(raw.rfind("Exception:", 0) != 0);
    const nlohmann::json out = nlohmann::json::parse(raw);
    CHECK(out.at("diagnostics").at("switchingFrequency").get<double>() == Catch::Approx(d.operatingFrequency));
    const auto& exc = out.at("operatingPoint").at("excitationsPerWinding");
    REQUIRE(exc.size() == 2);
    const WindingPower pri = winding_power(exc.at(0)), sec = winding_power(exc.at(1));
    INFO(topology << ": fsw " << d.operatingFrequency << " Hz (fr " << d.resonantFrequency << "), P_pri " << pri.power
         << " W, P_sec " << sec.power << " W");
    // The primary carries Pout/η times the receiving tank's power factor Rac/|Zsec|: the model's square winding
    // voltage is in phase with Lm's voltage, the rectifier current with Rac, and they differ by arg(Zsec) — ≈1 for a
    // symmetric tank near fr, 0.976 for the asymmetric pinned tank below (Lr2 referred 40 uH against Cr2 47 nF).
    const double N = d.turnsRatio;
    const auto tank = Kirchhoff::analytical::cllc_fha_tank(
        d.operatingFrequency, d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
        d.secondaryResonantInductance * N * N, d.secondaryResonantCapacitance / (N * N),
        8.0 / (M_PI * M_PI) * N * N * d.outputVoltage * d.outputVoltage / d.outputPower);
    const double pf = tank.zsecRe / std::hypot(tank.zsecRe, tank.zsecIm);
    INFO("receiving-tank power factor " << pf);
    CHECK(sec.power == Catch::Approx(Pout).epsilon(0.02));
    CHECK(pri.power == Catch::Approx(Pout / eta * pf).epsilon(0.02));
}
}  // namespace

TEST_CASE("CLLC with a pinned turns ratio sizes Ro and the secondary tank with THAT ratio: a = b = 1 (ABT #1537)",
          "[analytical][cllc][abt1537]") {
    // (n = 1 needs a tank gain of 1.031 at this efficiency, beyond the band: design_cllc throws for it, ABT #1503.)
    for (const double n : {0.9, 0.95, 0.97}) {
        for (const double Lm : {0.0, 200e-6}) {
            INFO("pinned n " << n << ", pinned Lm " << Lm);
            const auto d = Kirchhoff::design_cllc(cllc_web_spec(n, Lm));
            REQUIRE(d.turnsRatio == Catch::Approx(n));
            // Symmetric tank at the REQUESTED ratios: a = n²·Lr2/Lr1 = 1, b = Cr2/(n²·Cr1) = 1.
            CHECK(n * n * d.secondaryResonantInductance / d.primaryResonantInductance == Catch::Approx(1.0).epsilon(1e-12));
            CHECK(d.secondaryResonantCapacitance / (n * n * d.primaryResonantCapacitance) == Catch::Approx(1.0).epsilon(1e-12));
            if (Lm == 0.0) {
                // Q sizing against the load reflected through the pinned ratio: Ro = 8n²/π²·Rload, Cr1 = 1/(2π·Q·fr·Ro).
                const double Ro = 8.0 * n * n / (M_PI * M_PI) * (400.0 * 400.0 / 3300.0);
                CHECK(d.primaryResonantCapacitance == Catch::Approx(1.0 / (2.0 * M_PI * 0.4 * 120e3 * Ro)).epsilon(1e-12));
            }
        }
    }
    SECTION("the ticket's web case: n 0.97, Lm 200 uH, k 5 -> Lr1 40 uH, Lr2 = 40/0.97^2 = 42.51 uH (was 49.59)") {
        const auto d = Kirchhoff::design_cllc(cllc_web_spec(0.97, 200e-6));
        CHECK(d.primaryResonantInductance == Catch::Approx(40e-6));
        CHECK(d.secondaryResonantInductance == Catch::Approx(40e-6 / (0.97 * 0.97)));
        check_tank_in_deck(d, Kirchhoff::build_cllc_tas(d));
    }
    SECTION("an asymmetric tank ratio is applied against the pinned ratio too") {
        auto spec = cllc_web_spec(0.97, 0.0);
        spec["config"]["resonantInductorRatio"] = 0.95;
        spec["config"]["resonantCapacitorRatio"] = 1.05;
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.secondaryResonantInductance == Catch::Approx(0.95 * d.primaryResonantInductance / (0.97 * 0.97)));
        CHECK(d.secondaryResonantCapacitance == Catch::Approx(1.05 * 0.97 * 0.97 * d.primaryResonantCapacitance));
    }
    SECTION("the #1503 solve still meets Vout and balances power at the pinned ratio") {
        const auto spec = cllc_web_spec(0.97, 200e-6);
        check_two_sided_solved("cllc", Kirchhoff::design_cllc(spec), spec, 3300.0, 0.97);
    }
}

TEST_CASE("CLLLC with a pinned turns ratio keeps a symmetric tank at that ratio (ABT #1537)",
          "[analytical][clllc][abt1537]") {
    for (const double n : {0.9, 0.97, 1.0}) {
        INFO("pinned N " << n);
        const auto d = Kirchhoff::design_clllc(cllc_web_spec(n, 0.0));
        CHECK(n * n * d.secondaryResonantInductance / d.primaryResonantInductance == Catch::Approx(1.0).epsilon(1e-12));
        CHECK(d.secondaryResonantCapacitance / (n * n * d.primaryResonantCapacitance) == Catch::Approx(1.0).epsilon(1e-12));
    }
}

TEST_CASE("CLLC ngspice with a pinned turns ratio: the symmetric tank reaches the deck, power is forward and balanced "
          "(ABT #1537)", "[ngspice][cllc][abt1537]") {
    const auto spec = cllc_web_spec(0.97, 200e-6);
    const auto d = Kirchhoff::design_cllc(spec);
    const auto tas = Kirchhoff::build_cllc_tas(d);
    check_tank_in_deck(d, tas);
    CHECK(deck_element_value(deck_of(tas), "Lr2") == Catch::Approx(40e-6 / (0.97 * 0.97)).epsilon(1e-4));
    check_two_sided_ngspice("cllc", d, spec, tas);
}

TEST_CASE("CLLC honours pinned tank values and solves the operating frequency from them, or throws (ABT #1538)",
          "[analytical][cllc][abt1538]") {
    SECTION("Lr1, Cr1 and Lr2 pinned, engine-sized ratio: verbatim, resonance from Lr1-Cr1, frequency solved") {
        auto spec = cllc_web_spec(0.0, 0.0);
        auto& dr = spec["designRequirements"];
        dr["desiredResonantInductance"] = {{"nominal", 36e-6}};
        dr["desiredResonantCapacitance"] = {{"nominal", 47e-9}};
        dr["desiredSecondaryResonantInductance"] = {{"nominal", 50e-6}};
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.primaryResonantInductance == 36e-6);
        CHECK(d.primaryResonantCapacitance == 47e-9);
        CHECK(d.secondaryResonantInductance == 50e-6);
        // Unpinned Cr2 follows the pinned primary through b = 1 and the ratio; unpinned Lm is k·Lr1.
        CHECK(d.secondaryResonantCapacitance == Catch::Approx(d.turnsRatio * d.turnsRatio * 47e-9));
        CHECK(d.magnetizingInductance == Catch::Approx(5.0 * 36e-6));
        CHECK(d.resonantFrequency == Catch::Approx(1.0 / (2.0 * M_PI * std::sqrt(36e-6 * 47e-9))));
        CHECK(d.operatingFrequency >= 80e3);
        CHECK(d.operatingFrequency <= 200e3);
        check_two_sided_solved("cllc", d, spec, 3300.0, 0.97);
        check_tank_in_deck(d, Kirchhoff::build_cllc_tas(d));
    }
    SECTION("pinned n 0.97 and Lm 200 uH with Lr1 and Cr1 pinned: the tank pins win over the Lm re-size") {
        auto spec = cllc_web_spec(0.97, 200e-6);
        spec["designRequirements"]["desiredResonantInductance"] = {{"nominal", 45e-6}};
        spec["designRequirements"]["desiredResonantCapacitance"] = {{"nominal", 40e-9}};
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.primaryResonantInductance == 45e-6);
        CHECK(d.primaryResonantCapacitance == 40e-9);
        CHECK(d.magnetizingInductance == 200e-6);
        CHECK(d.secondaryResonantInductance == Catch::Approx(45e-6 / (0.97 * 0.97)));
        check_two_sided_solved("cllc", d, spec, 3300.0, 0.97);
    }
    SECTION("only Lr1 pinned: Cr1 follows so the tank still resonates at fr") {
        auto spec = cllc_web_spec(0.0, 0.0);
        spec["designRequirements"]["desiredResonantInductance"] = 30e-6;
        const auto d = Kirchhoff::design_cllc(spec);
        CHECK(d.primaryResonantInductance == 30e-6);
        CHECK(d.resonantFrequency == Catch::Approx(120e3));
        CHECK(1.0 / (2.0 * M_PI * std::sqrt(d.primaryResonantInductance * d.primaryResonantCapacitance)) ==
              Catch::Approx(120e3));
    }
    SECTION("a pinned tank that cannot reach the output in the band throws, naming the tank") {
        auto spec = cllc_web_spec(0.97, 0.0);
        spec["designRequirements"]["desiredResonantInductance"] = 40e-6;
        spec["designRequirements"]["desiredResonantCapacitance"] = 44e-9;
        spec["designRequirements"]["desiredSecondaryResonantInductance"] = 400e-6;   // a = 9.4: gain collapses
        const std::string out = Kirchhoff::api::process_converter("cllc", spec.dump(), "analytical");
        INFO(out.substr(0, 800));
        REQUIRE(out.rfind("Exception:", 0) == 0);
        CHECK_THAT(out, Catch::Matchers::ContainsSubstring("pinned turns ratio 0.97 cannot deliver 400 V from 400 V"));
        CHECK_THAT(out, Catch::Matchers::ContainsSubstring("Lr1 4e-05 H, Cr1 4.4e-08 F"));
        CHECK_THAT(out, Catch::Matchers::ContainsSubstring("The tank is asymmetric"));
    }
    SECTION("a pinned secondary element that contradicts a stated tank ratio throws") {
        auto spec = cllc_web_spec(0.97, 0.0);
        spec["designRequirements"]["desiredSecondaryResonantInductance"] = 50e-6;
        spec["config"]["resonantInductorRatio"] = 1.0;
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(spec), Catch::Matchers::ContainsSubstring("but config.resonantInductorRatio is 1"));
    }
    SECTION("a non-positive pin throws") {
        auto spec = cllc_web_spec(0.0, 0.0);
        spec["designRequirements"]["desiredResonantCapacitance"] = 0.0;
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(spec), Catch::Matchers::ContainsSubstring("desiredResonantCapacitance must be > 0"));
    }
}

TEST_CASE("CLLLC honours pinned primarySeriesInductance and primaryResonantCapacitance off fr and solves from them "
          "(ABT #1538)", "[analytical][clllc][abt1538]") {
    SECTION("the web I-know seed: 400 V -> 48 V, N 8, Lm 500 uH, Lr1 50 uH, Cr1 33 nF (123.9 kHz against 120 kHz)") {
        nlohmann::json spec = two_sided_1503_spec(8.0);
        spec["designRequirements"]["switchingFrequency"] = {{"nominal", 120e3}};
        spec["designRequirements"]["magnetizingInductance"] = {{"nominal", 500e-6}};
        spec["config"] = {{"minSwitchingFrequency", 90e3}, {"maxSwitchingFrequency", 150e3},
                          {"primarySeriesInductance", 50e-6}, {"primaryResonantCapacitance", 33e-9}};
        const auto d = Kirchhoff::design_clllc(spec);
        CHECK(d.primaryResonantInductance == 50e-6);
        CHECK(d.primaryResonantCapacitance == 33e-9);
        CHECK(d.magnetizingInductance == 500e-6);
        CHECK(d.secondaryResonantInductance == Catch::Approx(50e-6 / 64.0));
        CHECK(d.secondaryResonantCapacitance == Catch::Approx(64.0 * 33e-9));
        CHECK(d.resonantFrequency == Catch::Approx(1.0 / (2.0 * M_PI * std::sqrt(50e-6 * 33e-9))));
        CHECK(d.operatingFrequency >= 90e3);
        CHECK(d.operatingFrequency <= 150e3);
        check_two_sided_solved("clllc", d, spec, 480.0, 0.95);
        check_tank_in_deck(d, Kirchhoff::build_clllc_tas(d));
    }
    SECTION("a stated primaryResonantFrequency must agree with the pinned tank") {
        nlohmann::json spec = two_sided_1503_spec(8.0);
        spec["config"]["primarySeriesInductance"] = 50e-6;
        spec["config"]["primaryResonantCapacitance"] = 33e-9;
        spec["config"]["primaryResonantFrequency"] = 100e3;
        CHECK_THROWS_WITH(Kirchhoff::design_clllc(spec), Catch::Matchers::ContainsSubstring("differs from the tank resonance"));
    }
}

TEST_CASE("CLLC / CLLLC ngspice with pinned tank values: the pins reach the deck and the deck balances off resonance "
          "(ABT #1538)", "[ngspice][cllc][clllc][abt1538]") {
    SECTION("CLLC: Lr1 36 uH, Cr1 47 nF, Lr2 50 uH, engine-sized ratio") {
        auto spec = cllc_web_spec(0.0, 0.0);
        spec["designRequirements"]["desiredResonantInductance"] = 36e-6;
        spec["designRequirements"]["desiredResonantCapacitance"] = 47e-9;
        spec["designRequirements"]["desiredSecondaryResonantInductance"] = 50e-6;
        const auto d = Kirchhoff::design_cllc(spec);
        const auto tas = Kirchhoff::build_cllc_tas(d);
        const std::string deck = deck_of(tas);
        CHECK(deck_element_value(deck, "Lr1") == Catch::Approx(36e-6).epsilon(1e-4));
        CHECK(deck_element_value(deck, "Cr1") == Catch::Approx(47e-9).epsilon(1e-4));
        CHECK(deck_element_value(deck, "Lr2") == Catch::Approx(50e-6).epsilon(1e-4));
        CHECK(std::abs(d.operatingFrequency - d.resonantFrequency) > 0.05 * d.resonantFrequency);
        // 30 ms: the pinned tank (characteristic impedance ~39 ohm against a ~32-39 ohm reflected load, Q ~1) rings
        // against the output capacitor longer than the engine-sized one; at 16 ms the output is still settling and
        // the source-to-load average reads a few 0.01 % above 1.
        check_two_sided_ngspice("cllc", d, spec, tas, 0.030);
    }
    SECTION("CLLLC: Lr1 50 uH, Cr1 33 nF, pinned N 0.95") {
        auto spec = cllc_web_spec(0.95, 0.0);
        spec["config"]["primarySeriesInductance"] = 50e-6;
        spec["config"]["primaryResonantCapacitance"] = 33e-9;
        const auto d = Kirchhoff::design_clllc(spec);
        const auto tas = Kirchhoff::build_clllc_tas(d);
        const std::string deck = deck_of(tas);
        CHECK(deck_element_value(deck, "Lr1") == Catch::Approx(50e-6).epsilon(1e-4));
        CHECK(deck_element_value(deck, "Cr1") == Catch::Approx(33e-9).epsilon(1e-4));
        CHECK(d.operatingFrequency != d.resonantFrequency);
        // 30 ms: the pinned tank (characteristic impedance ~39 ohm against a ~32-39 ohm reflected load, Q ~1) rings
        // against the output capacitor longer than the engine-sized one; at 16 ms the output is still settling and
        // the source-to-load average reads a few 0.01 % above 1.
        check_two_sided_ngspice("clllc", d, spec, tas, 0.030);
    }
}

// ── ABT #1539: "I know the design I want" forces the drive frequency; the output voltage is a result ────────────
// config.driveAtSwitchingFrequency drives the tank at exactly designRequirements.switchingFrequency. The LLC used to
// embed the TARGET Vout at that frequency (a power-inconsistent point off resonance) and the CLLC had no such key at
// all (the wizard's Op. Frequency was silently ignored, ABT #1503 solved its own). Now the rail is what the FHA tank
// delivers there into the design load, the deck is driven at that frequency, and only a capacitive (non-ZVS) point
// throws.
namespace {
double stimulus_frequency(const nlohmann::json& tas) {
    const auto& st = tas.at("simulation").at("stimulus");
    REQUIRE(!st.empty());
    const double f = st.at(0).at("waveform").at("frequency").get<double>();
    for (const auto& s : st) CHECK(s.at("waveform").at("frequency").get<double>() == f);
    return f;
}
nlohmann::json llc_1539_spec(double fsw) {
    nlohmann::json s = llc_1503_spec(3.0, "halfBridge");   // n 3 half bridge, fr 180 kHz, band 100-300 kHz
    s["designRequirements"]["switchingFrequency"] = {{"nominal", fsw}};
    s["config"]["driveAtSwitchingFrequency"] = true;
    return s;
}
// The forced point's analytical operating point. The winding powers follow the model the solved #1503 points use:
// P_sec = Pout*pf and P_pri = Pout/eta*pf, pf = Rac/|Zsec| the receiving tank's power factor. KNOWN MODEL LIMIT
// (ABT #1546, open): physically both windings carry the load power; analytical_cllc places the rectifier square at
// the Lm phase, so off resonance the excitations fall short by pf (0.939 at 150 kHz here). Near fr pf ~1 and this
// is the #1503 check. When #1546 is fixed the pf factor goes and these become P_sec = P_pri = Pout.
void check_two_sided_forced(const char* topology, const Kirchhoff::CllcDesign& d, const nlohmann::json& spec, double eta) {
    const std::string raw = Kirchhoff::api::process_converter(topology, spec.dump(), "analytical");
    INFO(raw.substr(0, 400));
    REQUIRE(raw.rfind("Exception:", 0) != 0);
    const nlohmann::json out = nlohmann::json::parse(raw);
    CHECK(out.at("diagnostics").at("switchingFrequency").get<double>() == Catch::Approx(d.operatingFrequency));
    const auto& exc = out.at("operatingPoint").at("excitationsPerWinding");
    REQUIRE(exc.size() == 2);
    const WindingPower pri = winding_power(exc.at(0)), sec = winding_power(exc.at(1));
    const double N = d.turnsRatio;
    const auto tank = Kirchhoff::analytical::cllc_fha_tank(
        d.operatingFrequency, d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
        d.secondaryResonantInductance * N * N, d.secondaryResonantCapacitance / (N * N),
        8.0 / (M_PI * M_PI) * N * N * d.loadResistance);
    const double pf = tank.zsecRe / std::hypot(tank.zsecRe, tank.zsecIm);
    INFO(topology << ": forced fsw " << d.operatingFrequency << " Hz (fr " << d.resonantFrequency << "), Pout "
         << d.outputPower << " W, P_pri " << pri.power << " W, P_sec " << sec.power << " W, pf " << pf);
    CHECK(sec.power == Catch::Approx(d.outputPower * pf).epsilon(0.02));
    CHECK(pri.power == Catch::Approx(d.outputPower / eta * pf).epsilon(0.02));
}
nlohmann::json cllc_1539_spec(double fsw) {
    nlohmann::json s = cllc_web_spec(0.97, 200e-6);        // 400 V -> 400 V / 3.3 kW, eff 0.97, band 80-200 kHz
    s["designRequirements"]["switchingFrequency"] = {{"nominal", fsw}};
    s["config"]["resonantFrequency"] = 120e3;
    s["config"]["driveAtSwitchingFrequency"] = true;
    return s;
}
}  // namespace

TEST_CASE("LLC driveAtSwitchingFrequency: the forced frequency reaches the deck and Vout is the tank's result "
          "(ABT #1539)", "[analytical][llc][abt1539]") {
    const double Vin = 425.0, eta = 0.97, k = 0.5, n = 3.0, Rload = 90.0 * 90.0 / 3300.0;
    struct Case { double fsw; int side; };   // side: -1 below fr, 0 at fr, +1 above
    for (const Case c : {Case{180e3, 0}, Case{140e3, -1}, Case{240e3, +1}}) {
        DYNAMIC_SECTION("fsw " << c.fsw) {
            const nlohmann::json spec = llc_1539_spec(c.fsw);
            const Kirchhoff::LlcDesign d = Kirchhoff::design_llc(spec);
            const double Vd = d.outputs.at(0).diodeDrop;
            CHECK(d.resonantFrequency == Catch::Approx(180e3));
            CHECK(d.operatingFrequency == c.fsw);
            CHECK(d.loadResistance == Catch::Approx(Rload));
            // The rail is the tank's: n·(Vo + 2·Vd) = M·η·k·Vin (full-bridge rectifier), power V²/R.
            const double Rac = 8.0 / (M_PI * M_PI) * n * n * Rload;
            const double M = Kirchhoff::analytical::llc_fha_tank(c.fsw, d.magnetizingInductance, d.resonantInductance,
                                                                 d.resonantCapacitance, Rac).gain();
            CHECK(d.requiredGain == Catch::Approx(M));
            CHECK(d.outputVoltage == Catch::Approx(M * eta * k * Vin / n - 2.0 * Vd).epsilon(1e-9));
            CHECK(d.outputPower == Catch::Approx(d.outputVoltage * d.outputVoltage / Rload).epsilon(1e-9));
            if (c.side == 0) CHECK(M == Catch::Approx(1.0).epsilon(1e-9));
            if (c.side < 0) CHECK(M > 1.02);   // below resonance the LLC boosts
            if (c.side > 0) CHECK(M < 0.98);   // above resonance it bucks
            // The 90 V target is NOT met (n 3 half bridge at 180 kHz gives ~68 V) and nothing throws or re-solves.
            CHECK(std::abs(d.outputVoltage - 90.0) > 5.0);

            const nlohmann::json tas = Kirchhoff::build_llc_tas(d);
            CHECK(stimulus_frequency(tas) == c.fsw);
            CHECK(tas.at("inputs").at("designRequirements").at("outputs").at(0).at("voltage").at("nominal").get<double>() ==
                  Catch::Approx(d.outputVoltage));

            const std::string raw = Kirchhoff::api::process_converter("llc", spec.dump(), "analytical");
            INFO(raw.substr(0, 400));
            REQUIRE(raw.rfind("Exception:", 0) != 0);
            const nlohmann::json out = nlohmann::json::parse(raw);
            CHECK(out.at("diagnostics").at("switchingFrequency").get<double>() == c.fsw);
            const auto& exc = out.at("operatingPoint").at("excitationsPerWinding");
            REQUIRE(exc.size() == 2);
            const WindingPower pri = winding_power(exc.at(0)), sec = winding_power(exc.at(1));
            INFO("fsw " << c.fsw << ": M " << M << ", Vout " << d.outputVoltage << " V, Pout " << d.outputPower
                 << " W, P_pri " << pri.power << " W, P_sec " << sec.power << " W");
            // Power balance of the analytical point at the delivered rail (as the #1503 checks).
            CHECK(sec.power == Catch::Approx(d.outputPower).epsilon(0.02));
            CHECK(pri.power == Catch::Approx(d.outputPower / eta * (1.0 + 2.0 * Vd / d.outputVoltage)).epsilon(0.02));
        }
    }
}

TEST_CASE("LLC driveAtSwitchingFrequency: the ngspice deck switches at the forced frequency and lands on the FHA "
          "rail (ABT #1539)", "[ngspice][llc][abt1539]") {
    // Discrete resonant inductor: the integrated-Lr deck realises T1's leakage through a K coupling that shrinks
    // the effective turns ratio and lands ~13% high at EVERY frequency, forced or not (ABT #1545, open). The
    // forced drive is what is under test here, so the deck carries Lr as its own part.
    // Measured deck vs FHA Vout: 150 kHz 77.30 vs 72.91 V (FHA's below-resonance underestimate, ABT #1548), 220 kHz
    // 61.54 vs 61.77 V.
    for (const double fsw : {150e3, 220e3}) {
        DYNAMIC_SECTION("fsw " << fsw) {
            nlohmann::json spec = llc_1539_spec(fsw);
            spec["config"]["integratedResonantInductor"] = false;
            const Kirchhoff::LlcDesign d = Kirchhoff::design_llc(spec);
            const nlohmann::json tas = Kirchhoff::build_llc_tas(d);
            const TwoSidedDeckRun r = run_two_sided_deck(tas, fsw, 425.0, "vvin#branch", "vout", d.loadResistance);
            INFO("fsw " << fsw << ": FHA Vout " << d.outputVoltage << " V, deck Vout " << r.vDelivered << " V, P_src "
                 << r.pSource << " W, P_load " << r.pLoad << " W");
            CHECK(r.vDelivered == Catch::Approx(d.outputVoltage).epsilon(0.08));
            CHECK(r.pSource > 0.0);
            CHECK(r.pLoad / r.pSource > 0.85);
            CHECK(r.pLoad / r.pSource <= 1.0);
        }
    }
}

TEST_CASE("LLC driveAtSwitchingFrequency: a capacitive or out-of-band point throws (ABT #1539)",
          "[analytical][llc][abt1539]") {
    SECTION("below the no-load resonance fr/sqrt(1+Ln) the tank input is capacitive") {
        nlohmann::json spec = llc_1539_spec(60e3);
        spec["config"]["resonantBandMin"] = 40e3;
        CHECK_THROWS_WITH(Kirchhoff::design_llc(spec), Catch::Matchers::ContainsSubstring("tank input is capacitive"));
    }
    SECTION("outside the switching band") {
        CHECK_THROWS_WITH(Kirchhoff::design_llc(llc_1539_spec(350e3)), Catch::Matchers::ContainsSubstring("lies outside the band"));
    }
}

TEST_CASE("CLLC driveAtSwitchingFrequency: the tank resonates at config.resonantFrequency, the bridges switch at the "
          "forced frequency and Vout is the tank's result (ABT #1539)", "[analytical][cllc][abt1539]") {
    const double Vin = 400.0, eta = 0.97, n = 0.97, Rload = 400.0 * 400.0 / 3300.0;
    struct Case { double fsw; int side; };
    // The tank input turns capacitive below ~110.5 kHz at this load (independent FHA Zin: 110 kHz -0.6 deg, 112 kHz
    // +2.3 deg, 115 kHz +6.6 deg), so the below-resonance point is 115 kHz. This Q ~0.8 tank boosts little there
    // (M 1.008): the check is M > 1, i.e. Vout above the unity-gain 400 V.
    for (const Case c : {Case{120e3, 0}, Case{115e3, -1}, Case{150e3, +1}}) {
        DYNAMIC_SECTION("fsw " << c.fsw) {
            const nlohmann::json spec = cllc_1539_spec(c.fsw);
            const Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(spec);
            CHECK(d.resonantFrequency == Catch::Approx(120e3));
            CHECK(1.0 / (2.0 * M_PI * std::sqrt(d.primaryResonantInductance * d.primaryResonantCapacitance)) ==
                  Catch::Approx(120e3));
            CHECK(d.operatingFrequency == c.fsw);
            CHECK(d.loadResistance == Catch::Approx(Rload));
            const double M = two_sided_gain_at_operating_frequency(d);
            CHECK(d.requiredGain == Catch::Approx(M));
            CHECK(d.outputVoltage == Catch::Approx(M * eta * Vin / n).epsilon(1e-9));
            CHECK(d.outputPower == Catch::Approx(d.outputVoltage * d.outputVoltage / Rload).epsilon(1e-9));
            if (c.side == 0) CHECK(d.outputVoltage == Catch::Approx(400.0).epsilon(1e-9));   // symmetric tank: M(fr) = 1
            if (c.side < 0) CHECK(d.outputVoltage > 400.0);
            if (c.side > 0) CHECK(d.outputVoltage < 0.99 * 400.0);

            const nlohmann::json tas = Kirchhoff::build_cllc_tas(d);
            CHECK(stimulus_frequency(tas) == c.fsw);
            CHECK(tas.at("inputs").at("designRequirements").at("outputs").at(0).at("voltage").at("nominal").get<double>() ==
                  Catch::Approx(d.outputVoltage));
            check_two_sided_forced("cllc", d, spec, eta);
        }
    }
}

TEST_CASE("CLLC driveAtSwitchingFrequency: the ngspice deck switches at the forced frequency and lands on the FHA "
          "rail with forward, balanced power (ABT #1539)", "[ngspice][cllc][abt1539]") {
    // 115 kHz: below fr inside the inductive region (110 kHz is capacitive, see above); 140 kHz above fr.
    for (const double fsw : {115e3, 140e3}) {
        DYNAMIC_SECTION("fsw " << fsw) {
            const Kirchhoff::CllcDesign d = Kirchhoff::design_cllc(cllc_1539_spec(fsw));
            const nlohmann::json tas = Kirchhoff::build_cllc_tas(d);
            const TwoSidedDeckRun r = run_two_sided_deck(tas, fsw, 400.0, "vvin#branch", "vout", d.loadResistance, 0.030);
            INFO("fsw " << fsw << ": FHA Vout " << d.outputVoltage << " V, deck Vout " << r.vDelivered << " V, P_src "
                 << r.pSource << " W, P_load " << r.pLoad << " W");
            CHECK(r.vDelivered == Catch::Approx(d.outputVoltage).epsilon(0.08));
            CHECK(r.pSource > 0.0);
            CHECK(r.pLoad / r.pSource > 0.90);
            CHECK(r.pLoad / r.pSource <= 1.0);
        }
    }
}

TEST_CASE("CLLC driveAtSwitchingFrequency: contradictions, reverse flow and capacitive points throw (ABT #1539)",
          "[analytical][cllc][abt1539]") {
    SECTION("a resonantFrequency different from switchingFrequency without the flag states two resonances") {
        nlohmann::json spec = cllc_1539_spec(140e3);
        spec["config"].erase("driveAtSwitchingFrequency");
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(spec), Catch::Matchers::ContainsSubstring("differs from designRequirements.switchingFrequency"));
    }
    SECTION("reverse power flow is not modelled under the flag") {
        nlohmann::json spec = cllc_1539_spec(140e3);
        spec["config"]["powerFlowDirection"] = "reverse";
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(spec), Catch::Matchers::ContainsSubstring("not modelled"));
    }
    SECTION("outside the switching band") {
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(cllc_1539_spec(210e3)), Catch::Matchers::ContainsSubstring("lies outside the band"));
    }
    SECTION("far below resonance the tank input is capacitive") {
        nlohmann::json spec = cllc_1539_spec(50e3);
        spec["config"]["resonantBandMin"] = 30e3;
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(spec), Catch::Matchers::ContainsSubstring("tank input is capacitive"));
    }
    SECTION("just below the inductive boundary (~110.5 kHz here) it throws, just above it designs") {
        CHECK_THROWS_WITH(Kirchhoff::design_cllc(cllc_1539_spec(110e3)), Catch::Matchers::ContainsSubstring("tank input is capacitive"));
        CHECK_NOTHROW(Kirchhoff::design_cllc(cllc_1539_spec(112e3)));
    }
}
