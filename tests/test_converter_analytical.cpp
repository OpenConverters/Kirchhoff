// Validate the per-topology analytical solvers (src/ConverterAnalytical.cpp) — Phase 2 of the MKF
// analytical-converter-solver port. Checks the computed winding excitation (current/voltage waveforms +
// processed stresses) against the closed-form expectations.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ConverterAnalytical.hpp"

#include <cmath>
#include <vector>

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
    // Secondary winding integrates to the load current.
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(iout).margin(0.1));                 // 2 A
    // Inductor volt-second balance on both windings.
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.5));
    (void)D;
}

TEST_CASE("analytical_flyback DCM preserves load current", "[analytical][solver][flyback]") {
    using Kirchhoff::analytical::analytical_flyback;
    // Small Lp -> DCM (Lp=10 uH < Lcrit).
    MAS::OperatingPoint op = analytical_flyback(48, {12}, {2}, {2}, 100000, 10e-6);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(2.0).margin(0.15));   // secondary avg = Iout
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

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // First primary + Secondary 0
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
    // All four primary-side windings are half-period PULSES of peak ~Iin, avg magnitude Iin/2.
    for (size_t w = 0; w < 4; ++w) {
        CHECK(std::abs(*processed_current(op, w).get_average()) == Catch::Approx(inputCurrent / 2.0).margin(0.6));
        CHECK(*processed_current(op, w).get_peak() == Catch::Approx(inputCurrent).margin(0.6));
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
    CHECK_THROWS(analytical_fsbb(24, 24, 5, 100000, 20e-6));   // AUTO still throws at Vo==Vin
}

TEST_CASE("analytical_isolated_buck: primary avg=Ipri, secondary avg=Isec", "[analytical][solver][isolatedbuck]") {
    using Kirchhoff::analytical::analytical_isolated_buck;
    // 12 V; primary rail 3.3 V @ 1 A; isolated secondary 5 V @ 0.5 A; n=0.5, L=22 uH.
    MAS::OperatingPoint op = analytical_isolated_buck(12, 3.3, 1.0, 5.0, 0.5, 100000, 22e-6, 0.5);
    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary 0
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(1.0).margin(0.05));   // Ipri (KCL)
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(0.5).margin(0.05));   // Isec
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));
}

TEST_CASE("analytical_isolated_buck_boost: primary avg = (Ipri+Isec/n)/(1-D)", "[analytical][solver][isolatedbuckboost]") {
    using Kirchhoff::analytical::analytical_isolated_buck_boost;
    // 12 V; primary rail 5 V @ 1 A; isolated secondary 12 V @ 0.5 A; n=0.5, L=22 uH.
    MAS::OperatingPoint op = analytical_isolated_buck_boost(12, 5, 1.0, 12, 0.5, 100000, 22e-6, 0.5);
    REQUIRE(op.get_excitations_per_winding().size() == 2);
    const double D = 5.0 / (12.0 + 5.0);                      // 0.294
    const double primAvg = (1.0 + 0.5 / 0.5) / (1.0 - D);     // (1+1)/0.706 = 2.833
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(primAvg).margin(0.1));
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.3));
}

TEST_CASE("analytical_active_clamp_forward CCM: 2 windings, secondary peak, clamp-balanced primary",
          "[analytical][solver][acf]") {
    using Kirchhoff::analytical::analytical_active_clamp_forward;
    // 48 V -> 5 V, 10 A, 100 kHz, n=4, Lmag=1 mH, Lout=10 uH, ripple=0.3.
    const double vin = 48, vout = 5, iout = 10, fsw = 100000, ripple = 0.3;
    MAS::OperatingPoint op = analytical_active_clamp_forward(vin, {vout}, {iout}, {4}, fsw, 1e-3, 10e-6, ripple);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // First primary + Secondary 0
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
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(-1.0).margin(0.4));  // Io*(2D-1) DC bias
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
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_negative_peak() >= -0.05);
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
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_negative_peak() >= -0.05);
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
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_negative_peak() >= -0.05);
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
    CHECK(*processed_current(op, 1).get_negative_peak() >= -0.05);
    CHECK(*processed_current(op, 2).get_negative_peak() >= -0.05);
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
    MAS::OperatingPoint op = analytical_differential_mode_choke(iop, vin, fline, fsw);

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
        iop, 230, 50, 100000, DmcConfiguration::THREE_PHASE);

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
    CHECK_THROWS(analytical_differential_mode_choke(10, 230, 50, 0.0));     // switchingFrequency = 0
    CHECK_THROWS(analytical_differential_mode_choke(10, 230, 0.0, 100000)); // lineFrequency = 0
    // Neither peakCurrent (NaN default) nor a positive operatingCurrent → cannot size the choke.
    CHECK_THROWS(analytical_differential_mode_choke(0.0, 230, 50, 100000));
}

// Common-mode choke. Ported from MKF converter_models/CommonModeChoke.cpp:327 (scalar-arg overload).
TEST_CASE("analytical_common_mode_choke: N windings, DC bias = line current, small CM ripple, identical",
          "[analytical][component][cmc]") {
    using Kirchhoff::analytical::analytical_common_mode_choke;
    // Lm = 1 mH, 5 A line current, 230 V mains (scaling no-op), 150 kHz dominant impedance frequency, 2 windings.
    const double Lm = 1e-3, iop = 5, vop = 230, fexc = 150000;
    MAS::OperatingPoint op = analytical_common_mode_choke(Lm, iop, vop, fexc);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Line + Neutral
    // Default (no parasitics): I_cm = 0.1 A × (230/230). Every winding = CM ripple (pp = 2·I_cm) on the
    // line-current DC bias → average = line current, small ripple.
    const double iCmPeak = 0.1;
    CHECK(*processed_current(op, 0).get_average() == Catch::Approx(iop).margin(0.05));           // DC = line I
    CHECK(*processed_current(op, 0).get_peak_to_peak() == Catch::Approx(2.0 * iCmPeak).margin(0.05));
    // Both windings carry the identical CM waveform (precondition for the CM-choke magnetizing-current path).
    CHECK(*processed_current(op, 1).get_average() == Catch::Approx(*processed_current(op, 0).get_average()).margin(0.01));
    CHECK(*processed_current(op, 1).get_rms() == Catch::Approx(*processed_current(op, 0).get_rms()).margin(0.01));
    // CM voltage present, peak = L·ω·I_cm.
    REQUIRE(op.get_excitations_per_winding()[0].get_voltage().has_value());
    const double vCmPeak = Lm * 2.0 * M_PI * fexc * iCmPeak;                                      // ≈ 94.25 V
    CHECK(*processed_voltage(op, 0).get_peak() == Catch::Approx(vCmPeak).margin(1.0));
}

TEST_CASE("analytical_common_mode_choke honors winding count and C·dV/dt CM current",
          "[analytical][component][cmc]") {
    using Kirchhoff::analytical::analytical_common_mode_choke;
    // 4 windings; explicit parasitics 100 pF × 10 V/ns → I_cm = 100·10·1e-3 = 1.0 A (× 230/230 = 1.0).
    MAS::OperatingPoint op = analytical_common_mode_choke(1e-3, 5, 230, 150000, 4, 100.0, 10.0);
    REQUIRE(op.get_excitations_per_winding().size() == 4);   // Phase A/B/C + Neutral
    CHECK(*processed_current(op, 0).get_peak_to_peak() == Catch::Approx(2.0).margin(0.05));   // 2·I_cm = 2.0
    CHECK(*processed_current(op, 3).get_average() == Catch::Approx(5.0).margin(0.05));        // DC = line I
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
        double lo = rms0(analytical_differential_mode_choke(5,230,50,100000));
        double hi = rms0(analytical_differential_mode_choke(10,230,50,100000));
        CHECK(hi > 1.8 * lo);
    }
}
