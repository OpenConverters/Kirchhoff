// Validate the per-topology analytical solvers (src/ConverterAnalytical.cpp) — Phase 2 of the MKF
// analytical-converter-solver port. Checks the computed winding excitation (current/voltage waveforms +
// processed stresses) against the closed-form expectations.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ConverterAnalytical.hpp"

#include <cmath>

using Kirchhoff::analytical::analytical_buck;

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
