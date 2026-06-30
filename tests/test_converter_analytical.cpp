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

TEST_CASE("analytical_weinberg boost regime: 2 windings, primary peak", "[analytical][solver][weinberg]") {
    using Kirchhoff::analytical::analytical_weinberg;
    // 24 V -> 72 V (M=3, boost regime D=0.833), 2 A, 100 kHz, L1=50 uH, n=1.
    const double vin = 24, vout = 72, iout = 2, fsw = 100000, L1 = 50e-6, n = 1;
    MAS::OperatingPoint op = analytical_weinberg(vin, vout, iout, fsw, L1, n);

    REQUIRE(op.get_excitations_per_winding().size() == 2);   // Primary + Secondary
    const double M = vout / vin;                              // 3
    const double D = 1.0 - 1.0 / (2.0 * n * M);               // 0.8333
    const double overlap = 2.0 * D - 1.0;                     // 0.6667
    const double inputCurrent = iout / M;                     // 0.6667
    const double deltaIL1 = vin * std::max(overlap, D) / (L1 * fsw);   // 4.0
    const double iL1_high = inputCurrent + deltaIL1 / 2.0;    // 2.667
    CHECK(*processed_current(op, 0).get_peak() == Catch::Approx(iL1_high).margin(0.15));
    CHECK(*processed_current(op, 1).get_peak() == Catch::Approx(iL1_high * n).margin(0.15));   // secondary = primary*n
    CHECK(voltage_average(op, 0) == Catch::Approx(0.0).margin(0.5));   // bipolar rectangular
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

TEST_CASE("analytical_asymmetric_half_bridge rejects duty outside (0,1)", "[analytical][solver][ahb]") {
    using Kirchhoff::analytical::analytical_asymmetric_half_bridge;
    CHECK_THROWS(analytical_asymmetric_half_bridge(48, {12}, {5}, {2}, 100000, 200e-6, 1.0, 0.3));
    CHECK_THROWS(analytical_asymmetric_half_bridge(48, {12}, {5}, {2}, 100000, 200e-6, 0.0, 0.3));
}
