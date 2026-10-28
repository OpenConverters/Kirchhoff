// PFC boost-inductor current in critical and discontinuous conduction (ABT #2143).
//
// analytical_pfc used to ride a zero-mean switching triangle on the line envelope in every switching cycle, so
// whenever the ripple exceeded twice the local average (a CrM or DCM design, or a pinned inductance too small
// for continuous conduction) the boost-inductor current went NEGATIVE — impossible for a boost whose diode
// blocks reverse current — and its peak/rms were those of a CCM triangle, not of the real waveform.
//
// The oracle here is the CLOSED FORM of each conduction mode, not a simulation:
//   * CrM (critical / transition): constant on-time, every cycle a triangle 0 → 2·I_line·|sin| → 0, so the
//     peak is 2·I_linePeak, each cycle averages its line current (mean 2/π·I_linePeak) and the rms is
//     I_linePeak·√(2/3).
//   * DCM: fixed frequency, ONE on-time over the line cycle from the power balance
//     Pin = t_on²/(2·L·Tsw)·⟨Vin²·Vo'/(Vo'−|Vin|)⟩ (Vo' = Vout + Vd); peak Vpk·t_on/L at the line peak.
//   * CCM-mode control with a pinned L too small for continuous conduction ("I know the design", 250 µH):
//     every cycle averages the line envelope (mean 2/π·I_linePeak); where the CCM triangle would dip below
//     zero the cycle is discontinuous with t_on = √(2·L·Tsw·(I_linePeak/Vpk)·(Vo'−|Vin|)/Vo').
// Line-cycle averages of the per-cycle closed forms are integrated numerically on a grid 64x finer than the
// waveform's (they are smooth, so that integral is exact to far below the tolerances).
//
// The design is the openmagnetics.com PFC wizard's default: 175 Vrms operating point (85-265 Vac range), 400 V,
// 300 W, 65 kHz, 50 Hz, efficiency 0.95, Vd 0.6 V; CrM-sized L (284.4 µH), DCM-sized L (216.9 µH), and a pinned
// 250 µH in CCM mode.
//
// SAMPLING GRID. The waveform is emitted on a uniform power-of-two grid (h = T/(N−1), 16384 points here) that
// MKF's WaveformProcessor consumes as-is; switching edges fall between samples. The current is piecewise
// linear with exact sample values, so:
//   * the stored waveform is MKF's resample (onto T/N spacing) of the model's T/(N−1) grid, i.e. points on
//     the model's sample polyline;
//   * the largest model SAMPLE misses the true apex by at most h·sOn·sOff/(sOn+sOff) (the two samples
//     straddling the apex, sOn/sOff the rising/falling slopes), and the resample can miss that polyline's apex
//     by as much again: twice that is asserted as a bound on the raw sampled peak; and
//   * the apex itself is RECONSTRUCTED exactly from the samples (intersection of the rising and falling
//     straight segments through the samples beside it) and asserted against the closed form to the only
//     remaining model approximation: a cycle takes the line voltage at its midpoint, so the cycle nearest the
//     line peak sits at most one switching period from it (relative error ≤ 1 − cos(ω_line·Tc)).
//   * the trapezoidal mean errs only in cells holding a slope change (≤ Δs·h²/8 each, of alternating sign:
//     concave apex vs convex zero-current corner, so they cancel to the phase spread); measured ≤ 0.05 %,
//     asserted at 0.5 %.
//   * the exact-for-linear rms integral misses the cut-off apex triangle in each apex cell (twice: the model
//     grid and MKF's resample): rms² falls short by at most ⟨Ipk/Tc⟩·(Vo'/L)·h²/2 (computed below, ~7 % of
//     rms² for these designs; measured 1-3.5 %), never exceeds the closed form; asserted one-sided.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ConverterAnalytical.hpp"
#include "Kirchhoff.hpp"
#include "Pfc.hpp"

using nlohmann::json;
namespace AN = Kirchhoff::analytical;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDiodeDrop = 0.6;

// The spec the web PFC wizard sends for its default design (WebSharedComponents buildKhConverterSpec).
json web_pfc_spec(const std::string& mode, double pinnedInductance, const std::string& variant = "boost") {
    json s;
    auto& dr = s["designRequirements"];
    dr["inputType"] = "acSinglePhase";
    dr["inputVoltage"]["minimum"] = 85.0;
    dr["inputVoltage"]["maximum"] = 265.0;
    dr["switchingFrequency"]["nominal"] = 65000.0;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = 400.0; o["regulation"] = "voltage";
      dr["outputs"] = json::array({o}); }
    dr["efficiency"] = 0.95;
    dr["lineFrequency"]["nominal"] = 50.0;
    if (pinnedInductance > 0) dr["magnetizingInductance"]["nominal"] = pinnedInductance;
    { json op; op["name"] = "full_load"; op["inputVoltage"] = 175.0; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = 300.0; op["outputs"] = json::array({o});
      s["operatingPoints"] = json::array({op}); }
    s["config"]["currentRippleFraction"] = 0.3;
    s["config"]["diodeVoltageDrop"] = kDiodeDrop;
    s["config"]["topologyVariant"] = variant;
    if (variant == "interleaved") s["config"]["numberOfPhases"] = 2;
    s["config"]["mode"] = mode;
    s["config"]["wideBandgapSwitch"] = true;
    return s;
}

struct Sampled { std::vector<double> t, i, v; };

Sampled sampled_of(const MAS::OperatingPoint& op) {
    const auto& exc = op.get_excitations_per_winding().at(0);
    const MAS::Waveform ci = *exc.get_current()->get_waveform();
    const MAS::Waveform vi = *exc.get_voltage()->get_waveform();
    return {*ci.get_time(), ci.get_data(), vi.get_data()};
}

MAS::OperatingPoint analytical_of(const Kirchhoff::PfcDesign& d) {
    // Each interleaved phase carries 1/N of the power (build_pfc_interleaved_tas); N = 1 otherwise.
    return AN::analytical_pfc(d.inputVoltageRms, d.outputVoltage, d.outputPower / d.numberOfPhases, d.lineFrequency,
                              d.switchingFrequency, d.boostInductance, AN::pfc_conduction_mode(d.mode),
                              d.efficiency, kDiodeDrop, d.bipolar);
}

struct Stats { double peak, rms, min, mean; };

// Time-weighted statistics of a sampled piecewise-linear waveform (trapezoid mean, exact-for-linear rms).
Stats stats_of(const std::vector<double>& t, const std::vector<double>& y) {
    double peak = -std::numeric_limits<double>::max(), mn = std::numeric_limits<double>::max();
    double area = 0.0, area2 = 0.0;
    for (size_t k = 1; k < t.size(); ++k) {
        const double dt = t[k] - t[k - 1];
        area  += 0.5 * (y[k] + y[k - 1]) * dt;
        area2 += (y[k] * y[k] + y[k] * y[k - 1] + y[k - 1] * y[k - 1]) / 3.0 * dt;
        peak = std::max({peak, y[k], y[k - 1]});
        mn   = std::min({mn, y[k], y[k - 1]});
    }
    const double span = t.back() - t.front();
    return {peak, std::sqrt(area2 / span), mn, area / span};
}

// The exact apex of the piecewise-linear current, reconstructed from the samples. The stored waveform is
// MKF's resample (WaveformProcessor::calculate_sampled_waveform, onto T/N spacing) of the T/(N−1) grid the
// model emits, i.e. points ON the model's sample polyline: they lie on the true current wherever both
// neighbouring model samples do, and fall below it in the few cells around a kink. So around every local
// maximum sample j in the top 5 % of the waveform: the rising ramp is the line through the last three
// COLLINEAR samples at or before j (collinear to 1e-9 relative ⇒ on one straight piece of the current), the
// falling ramp the line through the first three collinear samples at or after j; their intersection is the
// apex, accepted when it lies between the two ramps and they are at most 4 samples apart. Returns the
// largest accepted apex; throws if none is (the grid does not resolve the cycle).
double reconstructed_apex(const std::vector<double>& t, const std::vector<double>& y) {
    const double top = *std::max_element(y.begin(), y.end());
    double best = -std::numeric_limits<double>::max();
    size_t accepted = 0;
    auto slope = [&](size_t a) { return (y[a + 1] - y[a]) / (t[a + 1] - t[a]); };
    auto same = [](double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(std::abs(a), std::abs(b)); };
    for (size_t j = 8; j + 8 < y.size(); ++j) {
        if (!(y[j] >= y[j - 1] && y[j] >= y[j + 1] && y[j] >= 0.95 * top)) continue;
        // rising ramp: samples r−2, r−1, r collinear, rising, r ≤ j as late as possible
        size_t r = j;
        while (r + 4 > j && !(slope(r - 1) > 0 && same(slope(r - 2), slope(r - 1)))) --r;
        // falling ramp: samples f, f+1, f+2 collinear, falling, f ≥ j as early as possible
        size_t f = j;
        while (f < j + 4 && !(slope(f) < 0 && same(slope(f), slope(f + 1)))) ++f;
        if (!(slope(r - 1) > 0 && same(slope(r - 2), slope(r - 1))) || !(slope(f) < 0 && same(slope(f), slope(f + 1))))
            continue;
        if (f < r || f - r > 4) continue;
        const double sR = slope(r - 1), sF = slope(f);
        // y[r] + sR·(x − t[r]) = y[f] + sF·(x − t[f])
        const double x = (y[f] - y[r] + sR * t[r] - sF * t[f]) / (sR - sF);
        if (x < t[r] - 1e-15 || x > t[f] + 1e-15) continue;
        best = std::max(best, y[r] + sR * (x - t[r]));
        ++accepted;
    }
    if (accepted == 0) throw std::runtime_error("reconstructed_apex: no apex resolved by the sampling grid");
    return best;
}

// Line-cycle average of f(|sin θ|) over one line period, midpoint rule with n points.
double line_average(const std::function<double(double)>& f, size_t n) {
    double s = 0.0;
    for (size_t k = 0; k < n; ++k) s += f(std::abs(std::sin(2.0 * kPi * (static_cast<double>(k) + 0.5) / static_cast<double>(n))));
    return s / static_cast<double>(n);
}

// Closed form of each mode.
struct ClosedForm {
    double peak, mean, rms;
    double peakRisingSlope, peakFallingSlope;   // the slopes at the line-peak cycle's apex [A/s]
    double peakCyclePeriod;                     // that cycle's period [s]
    double apexRate;                            // ⟨Ipk/Tc⟩: Σ over the line cycle's apexes of Ipk, per second [A/s]
};

ClosedForm closed_form(const Kirchhoff::PfcDesign& d, size_t gridPoints) {
    const double L = d.boostInductance, Tsw = 1.0 / d.switchingFrequency;
    const double pin = d.outputPower / d.efficiency;
    const double vpk = d.inputVoltageRms * std::sqrt(2.0), ilpk = pin / d.inputVoltageRms * std::sqrt(2.0);
    const double vo = d.outputVoltage + kDiodeDrop;
    const size_t n = 64 * gridPoints;
    ClosedForm c{};
    c.peakRisingSlope  = vpk / L;
    c.peakFallingSlope = (vo - vpk) / L;
    if (d.mode == "crm" || d.mode == "transition") {
        const double tOn = 2.0 * L * ilpk / vpk;
        c.peak = 2.0 * ilpk;
        c.mean = 2.0 / kPi * ilpk;
        c.rms  = ilpk * std::sqrt(2.0 / 3.0);
        c.peakCyclePeriod = tOn * vo / (vo - vpk);
        c.apexRate = line_average([&](double s) { return 2.0 * ilpk * s * (vo - vpk * s) / (tOn * vo); }, n);
        return c;
    }
    // Discontinuous cycles from zero: peak Ip = |Vin|·tOn/L, conducting tOn·Vo'/(Vo'−|Vin|) of Tsw;
    // cycle mean Ip·cond/(2·Tsw), cycle mean square Ip²·cond/(3·Tsw).
    std::function<double(double)> tOnAt;
    if (d.mode == "dcm") {
        const double shape = line_average([&](double s) { const double v = vpk * s; return v * v * vo / (vo - v); }, n);
        const double tOn = std::sqrt(2.0 * L * Tsw * pin / shape);
        tOnAt = [tOn](double) { return tOn; };
    } else {   // ccm control with an L too small for continuous conduction at the line peak (premise below)
        tOnAt = [&](double s) { return std::sqrt(2.0 * L * Tsw * (ilpk / vpk) * (vo - vpk * s) / vo); };
    }
    auto ip   = [&](double s) { return vpk * s * tOnAt(s) / L; };
    auto cond = [&](double s) { return tOnAt(s) * vo / (vo - vpk * s); };
    c.peak = ip(1.0);
    c.mean = line_average([&](double s) { return ip(s) * cond(s) / (2.0 * Tsw); }, n);
    c.rms  = std::sqrt(line_average([&](double s) { return ip(s) * ip(s) * cond(s) / (3.0 * Tsw); }, n));
    c.peakCyclePeriod = Tsw;
    c.apexRate = line_average(ip, n) / Tsw;
    return c;
}

uint64_t fnv1a(const std::vector<double>& v) {
    uint64_t h = 1469598103934665603ULL;
    for (double x : v) {
        unsigned char b[sizeof(double)];
        std::memcpy(b, &x, sizeof(double));
        for (unsigned char ch : b) { h ^= ch; h *= 1099511628211ULL; }
    }
    return h;
}

}  // namespace

TEST_CASE("PFC CrM / DCM / small pinned L: boost-inductor current against each mode's closed form",
          "[pfc][conduction][closedform]") {
    struct Case { const char* name; const char* mode; double pinnedL; double expectedL; };
    // expectedL: the inductance the design must carry for the case to be the one described above.
    const Case cases[] = {{"crm", "crm", 0.0, 284.4e-6},
                          {"transition", "transition", 0.0, 284.4e-6},
                          {"dcm", "dcm", 0.0, 216.9e-6},
                          {"iknow250uH", "ccm", 250e-6, 250e-6}};
    for (const Case& c : cases) {
        INFO("case " << c.name);
        const Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(web_pfc_spec(c.mode, c.pinnedL));
        REQUIRE(d.boostInductance == Catch::Approx(c.expectedL).epsilon(5e-4));
        const Sampled s = sampled_of(analytical_of(d));
        const Stats st = stats_of(s.t, s.i);
        const ClosedForm cf = closed_form(d, s.t.size());
        const double h = s.t[1] - s.t[0];
        const double vpk = d.inputVoltageRms * std::sqrt(2.0);
        const double vo = d.outputVoltage + kDiodeDrop;
        const double ilpk = d.outputPower / d.efficiency / d.inputVoltageRms * std::sqrt(2.0);
        if (std::string(c.mode) == "ccm") {
            // Premise: the pinned L is too small for continuous conduction at the line peak (CCM ripple/2 > I_line).
            const double halfRipple = 0.5 * vpk * (1.0 - vpk / vo) / (d.boostInductance * d.switchingFrequency);
            REQUIRE(halfRipple > ilpk);
        }
        // Premise of the apex reconstruction: the rising and falling ramps each span several samples.
        const double tOnPeak = cf.peak / cf.peakRisingSlope, tOffPeak = cf.peak / cf.peakFallingSlope;
        REQUIRE(tOnPeak > 4.0 * h);
        REQUIRE(tOffPeak > 4.0 * h);

        // Never negative: the boost diode blocks reverse current.
        CHECK(st.min >= 0.0);

        // Peak: exact apex vs closed form (only the cycle-midpoint line voltage separates them) ...
        const double apex = reconstructed_apex(s.t, s.i);
        const double midpointBound = 1.0 - std::cos(2.0 * kPi * d.lineFrequency * cf.peakCyclePeriod);
        CHECK(apex <= cf.peak * (1.0 + 1e-9));
        CHECK(apex >= cf.peak * (1.0 - midpointBound));
        // ... and the raw sampled peak within the straddle bound of the grid, once for the model grid and once
        // for MKF's resample of it.
        const double straddle = h * cf.peakRisingSlope * cf.peakFallingSlope / (cf.peakRisingSlope + cf.peakFallingSlope);
        CHECK(st.peak <= cf.peak * (1.0 + 1e-9));
        CHECK(st.peak >= cf.peak - 2.0 * straddle);

        // Mean (each cycle's average; ±0.5 %, see the header) and rms (one-sided apex-cell deficit).
        CHECK(st.mean == Catch::Approx(cf.mean).epsilon(5e-3));
        // Each apex cell cuts a triangle of area ≤ Δs·h²/8 off the current (Δs = Vo'/L, the slope change at
        // the apex), once on the model grid and once more in MKF's resample: ≤ 2·Ipk·Δs·h²/4 of i²·t per apex.
        const double rms2Deficit = cf.apexRate * (vo / d.boostInductance) * h * h / 2.0;
        CHECK(st.rms <= cf.rms * (1.0 + 1e-3));
        CHECK(st.rms * st.rms >= cf.rms * cf.rms - rms2Deficit);

        // Power balance: the current delivers the input power, ⟨|Vin|·i⟩ = Pin (1 %).
        double energy = 0.0;
        for (size_t k = 1; k < s.t.size(); ++k) {
            auto p = [&](size_t m) { return vpk * std::abs(std::sin(2.0 * kPi * d.lineFrequency * s.t[m])) * s.i[m]; };
            energy += 0.5 * (p(k) + p(k - 1)) * (s.t[k] - s.t[k - 1]);
        }
        CHECK(energy / (s.t.back() - s.t.front()) == Catch::Approx(d.outputPower / d.efficiency).epsilon(1e-2));
        std::printf("%-11s L=%.1f uH  closed pk/mean/rms %.4f/%.4f/%.4f  apex %.4f  sampled pk/mean/rms/min %.4f/%.4f/%.4f/%.4g"
                    "  P %.2f W\n", c.name, d.boostInductance * 1e6, cf.peak, cf.mean, cf.rms, apex, st.peak, st.mean,
                    st.rms, st.min, energy / (s.t.back() - s.t.front()));
    }
}

TEST_CASE("PFC DCM: an inductance too large for discontinuous conduction throws", "[pfc][conduction]") {
    const Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(web_pfc_spec("dcm", 1e-3));
    REQUIRE(d.boostInductance == 1e-3);
    try {
        (void) analytical_of(d);
        FAIL("analytical_pfc accepted DCM with 1 mH");
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("discontinuous conduction is impossible") != std::string::npos);
    }
}

TEST_CASE("PFC CrM / DCM: totem-pole and interleaved designs build, the current follows the line polarity",
          "[pfc][conduction][variants]") {
    for (const char* variant : {"totemPole", "interleaved"}) {
        for (const char* mode : {"crm", "dcm"}) {
            INFO(variant << " " << mode);
            const Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(web_pfc_spec(mode, 0.0, variant));
            CHECK_NOTHROW(Kirchhoff::build_pfc_tas(d));
            const Sampled s = sampled_of(analytical_of(d));
            // Unipolar: never negative. Bipolar: the current takes the sign of the line voltage, except in the
            // one switching cycle that straddles a line zero crossing: the model holds a cycle's line voltage
            // (and so its polarity) at the cycle midpoint, and a variable-period CrM cycle can start before the
            // crossing and end after it (fixed-frequency cycles tile the half line period exactly here). That
            // cycle is shorter than one nominal switching period (its |Vin| is at the 5 % floor).
            bool polarityHeld = true;
            const double halfLine = 0.5 / d.lineFrequency;
            for (size_t k = 0; k < s.t.size(); ++k) {
                const double line = d.bipolar ? std::sin(2.0 * kPi * d.lineFrequency * s.t[k]) : 1.0;
                const double toCrossing = std::abs(s.t[k] - std::round(s.t[k] / halfLine) * halfLine);
                if (d.bipolar && toCrossing < 1.0 / d.switchingFrequency) continue;
                if (s.i[k] * line < -1e-12 * std::abs(s.i[k])) {
                    polarityHeld = false;
                    UNSCOPED_INFO("t=" << s.t[k] << " i=" << s.i[k]);
                }
            }
            CHECK(polarityHeld);
        }
    }
}

TEST_CASE("PFC CCM: analytical line cycle unchanged by the conduction-mode work", "[pfc][conduction][ccm]") {
    // Fingerprints of origin/main 737f2a6 (before ABT #2143): voltage and time are byte-identical; the current
    // differs only in the switching cycles adjacent to a line zero crossing (≤ 54 of 16384 samples, ≤ 45 mA),
    // where the continuous triangle around the moving envelope dipped below zero and now holds the cycle
    // average instead. Peak is bit-identical; rms/mean move by < 1e-7 / 2e-6 relative; the minimum is no
    // longer negative. The bipolar totem-pole cell is byte-identical (it carries reverse current).
    struct Fp { const char* name; uint64_t v, t, i; double pk, rms, mean; bool bipolar; };
    const Fp base[] = {
        {"webccm",   0xd485860eb79fcdc9ULL, 0x0a0c1afb8f834f33ULL, 0, 2.9047588188686762, 1.8145392254900623, 1.6247308468132637, false},
        {"vs85",     0x69d4b9873c56eb7cULL, 0x084b3468499fd630ULL, 0, 12.491129260866002, 8.0647227569659758, 7.247567859296618, false},
        {"c230",     0x0849ad6801fa3fecULL, 0x0a0c1afb8f834f33ULL, 0, 19.33412422523406, 13.060464100402051, 11.743976109943864, false},
        {"bip230",   0x72b26e4d60606d88ULL, 0x0a0c1afb8f834f33ULL, 0xdff8dd12704aaba9ULL, 19.237892394416551, 13.060476589565742, 6.6201611090636681e-06, true},
        {"c230half", 0x0849ad6801fa3fecULL, 0x0a0c1afb8f834f33ULL, 0, 10.111060245657045, 6.5550486198151203, 5.8719897112932458, false},
    };
    const Kirchhoff::PfcDesign web = Kirchhoff::design_pfc(web_pfc_spec("ccm", 0.0));
    const auto C = AN::PfcConductionMode::CONTINUOUS;
    const MAS::OperatingPoint ops[] = {
        AN::analytical_pfc(web.inputVoltageRms, web.outputVoltage, web.outputPower, web.lineFrequency,
                           web.switchingFrequency, web.boostInductance, C, web.efficiency, kDiodeDrop),
        AN::analytical_pfc(85, 370, 650, 60, 65000, 536e-6, C, 0.95, 0.8),
        AN::analytical_pfc(230, 400, 3000, 50, 65000, 500e-6, C),
        AN::analytical_pfc(230, 400, 3000, 50, 65000, 500e-6, C, 1.0, 0.0, true),
        AN::analytical_pfc(230, 400, 1500, 50, 65000, 500e-6, C),
    };
    for (size_t k = 0; k < 5; ++k) {
        INFO(base[k].name);
        const Sampled s = sampled_of(ops[k]);
        const Stats st = stats_of(s.t, s.i);
        CHECK(fnv1a(s.v) == base[k].v);
        CHECK(fnv1a(s.t) == base[k].t);
        if (base[k].bipolar) {
            CHECK(fnv1a(s.i) == base[k].i);
            continue;
        }
        CHECK(st.peak == base[k].pk);
        CHECK(st.rms == Catch::Approx(base[k].rms).epsilon(1e-7));
        CHECK(st.mean == Catch::Approx(base[k].mean).epsilon(2e-6));
        CHECK(st.min >= 0.0);
    }
}
