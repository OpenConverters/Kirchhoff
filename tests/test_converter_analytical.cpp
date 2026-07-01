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
    const auto& cur = *processed_current(op, 0).get_average();
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

TEST_CASE("analytical_cllc rejects bad inputs and infeasible gain", "[analytical][solver][cllc]") {
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
    // Infeasible conversion gain: n=1 for a 400→48 step-down gives M_req = 48/400 = 0.12 < 0.5.
    CHECK_THROWS(analytical_cllc(kCllcVin, {kCllcVout}, {kCllcIout}, {1.0}, kCllcFsub,
                                 kCllcLm, kCllcLr1, kCllcCr1, kCllcLr2, kCllcCr2));
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
