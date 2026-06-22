// MKF reference generator for the Kirchhoff ↔ MKF equivalence test.
//
// For each topology Kirchhoff supports, this links MKF (libMKF.so) and runs MKF's
// OWN design + ideal-component ngspice simulation, then writes the design
// parameters, the ideal deck text, and the simulator outputs
// (Vout/Iout/Pin/Pout/efficiency/ripple) to a golden JSON fixture under
// tests/reference/<topology>.mkf.json.
//
// "Ideal components" here = MKF's lossless analytical reference settings:
// diode_voltage_drop = 0, efficiency = 1, synchronous switch — exactly the
// configuration MKF's own PtP reference-design tests use
// (TestBoostReferenceDesignsPtp.cpp / TestFlybackReferenceDesignsPtp.cpp).
//
// This is a ONE-OFF generator, not part of the Kirchhoff test build. Regenerate
// the fixtures with tests/reference/build_and_run.sh whenever MKF's converter
// models change. The Kirchhoff test (tests/test_mkf_equivalence.cpp) re-runs the
// stored MKF deck through the same ngspice and compares Kirchhoff against it.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "converter_models/Boost.h"
#include "converter_models/Flyback.h"
#include "converter_models/Buck.h"
#include "converter_models/SingleSwitchForward.h"
#include "converter_models/TwoSwitchForward.h"
#include "converter_models/Sepic.h"
#include "converter_models/Cuk.h"
#include "converter_models/Zeta.h"
#include "converter_models/PushPull.h"
#include "converter_models/PhaseShiftedFullBridge.h"
#include "converter_models/AsymmetricHalfBridge.h"
#include "converter_models/ActiveClampForward.h"
#include "converter_models/FourSwitchBuckBoost.h"
#include "converter_models/PhaseShiftedHalfBridge.h"
#include "converter_models/Dab.h"

using namespace MAS;
using namespace OpenMagnetics;
using nlohmann::json;

namespace {

// Trapezoidal time-weighted helpers (copied from MKF tests/ConverterPortChecks.h
// to avoid pulling in catch2). ngspice grids are non-uniform, so a naive mean
// over-weights the dense switching regions — the trapezoid is required.
namespace C {
inline double time_weighted_mean(const std::vector<double>& t, const std::vector<double>& v) {
    if (t.size() < 2 || t.size() != v.size()) return 0.0;
    double area = 0.0;
    for (size_t i = 1; i < t.size(); ++i) area += 0.5 * (v[i] + v[i-1]) * (t[i] - t[i-1]);
    return area / (t.back() - t.front());
}
inline double time_weighted_mean_product(const std::vector<double>& t,
                                         const std::vector<double>& a,
                                         const std::vector<double>& b) {
    if (t.size() < 2 || t.size() != a.size() || t.size() != b.size()) return 0.0;
    double area = 0.0;
    for (size_t i = 1; i < t.size(); ++i)
        area += 0.5 * (a[i]*b[i] + a[i-1]*b[i-1]) * (t[i] - t[i-1]);
    return area / (t.back() - t.front());
}
}  // namespace C

// Settling periods before the steady-state period is extracted. This must cover several
// output-filter time constants (RC = Rload·Cout) or the captured operating point is a
// transient, not steady state. The boost EVM-style default (Cout=100µF, Rload=24Ω ->
// RC=2.4ms) needs ~2400 periods @100kHz (=24ms ≈ 10·RC) to settle; the flyback
// (Cout=10µF, Rload=6Ω -> RC=60µs) settles in well under 400. (MKF's own PtP tests use
// 400, which under-settles the 100µF boost — surfaced via Kirchhoff/tests/test_mkf_equivalence.)
constexpr size_t kBoostSettlingPeriods   = 2400;
constexpr size_t kFlybackSettlingPeriods = 400;
constexpr size_t kBuckSettlingPeriods    = 2400;
constexpr size_t kForwardSettlingPeriods = 2400;
constexpr size_t kTsfSettlingPeriods     = 2400;
constexpr size_t kSepicSettlingPeriods   = 2400;
constexpr size_t kCukSettlingPeriods     = 2400;
constexpr size_t kZetaSettlingPeriods    = 2400;
constexpr size_t kPushPullSettlingPeriods= 2400;
constexpr size_t kPsfbSettlingPeriods    = 2400;
constexpr size_t kAhbSettlingPeriods     = 2400;
constexpr size_t kAcfSettlingPeriods     = 2400;
constexpr size_t kFsbbSettlingPeriods    = 2400;
constexpr size_t kPshbSettlingPeriods    = 2400;
constexpr size_t kDabSettlingPeriods     = 2400;

struct SimRead {
    double voutMean = 0, ioutMean = 0, vinMean = 0, iinMean = 0;
    double pin = 0, pout = 0, efficiency = 0;
    double voutRipplePkPk = 0;
};

SimRead read_waveforms(const ConverterWaveforms& w) {
    SimRead r;
    const auto& voutW = w.get_output_voltages().at(0);
    const auto& ioutW = w.get_output_currents().at(0);
    const auto& vinW  = w.get_input_voltage();
    const auto& iinW  = w.get_input_current();

    const auto& voutD = voutW.get_data();
    const auto& ioutD = ioutW.get_data();
    const auto& vinD  = vinW.get_data();
    const auto& iinD  = iinW.get_data();
    const auto voutT  = voutW.get_time().value();
    const auto ioutT  = ioutW.get_time().value();
    const auto vinT   = vinW.get_time().value();

    r.voutMean = C::time_weighted_mean(voutT, voutD);
    r.ioutMean = C::time_weighted_mean(ioutT, ioutD);
    r.vinMean  = C::time_weighted_mean(vinT, vinD);
    r.iinMean  = C::time_weighted_mean(vinT, iinD);
    r.pin      = C::time_weighted_mean_product(vinT, vinD, iinD);
    r.pout     = C::time_weighted_mean_product(voutT, voutD, ioutD);
    r.efficiency = std::fabs(r.pin) > 1e-12 ? r.pout / r.pin : 0.0;
    auto [mn, mx] = std::minmax_element(voutD.begin(), voutD.end());
    r.voutRipplePkPk = *mx - *mn;
    return r;
}

void write_fixture(const std::string& path, json fixture) {
    std::ofstream f(path);
    f << fixture.dump(2) << "\n";
    std::cout << "wrote " << path << "\n";
}

// ── Boost ──────────────────────────────────────────────────────────────
void gen_boost(const std::string& outPath) {
    const double Vin = 12.0, Vout = 24.0, Iout = 1.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::Boost b;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    b.set_input_voltage(iv);
    b.set_diode_voltage_drop(0.0);   // ideal / synchronous
    b.set_efficiency(1.0);           // lossless reference
    b.set_current_ripple_ratio(ripple);
    BaseOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    b.set_operating_points({op});

    auto dr = b.process_design_requirements();
    const double L = dr.get_magnetizing_inductance().get_minimum().value();
    const double D = b.calculate_duty_cycle(Vin, Vout, 0.0, 1.0);

    b.set_num_steady_state_periods(kBoostSettlingPeriods);
    b.set_num_periods_to_extract(1);
    const std::string deck = b.generate_ngspice_circuit(L);
    auto wfs = b.simulate_and_extract_topology_waveforms(L, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "boost";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs},
                      {"currentRippleRatio", ripple}};
    fx["design"]   = {{"inductance", L}, {"dutyCycle", D}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    // probes: how to re-measure this deck live (the extracted steady-state period =
    // [settling·T, (settling+1)·T], matching MKF's .tran extraction window).
    const double bT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout"},
                      {"measFrom", kBoostSettlingPeriods * bT},
                      {"measTo", (kBoostSettlingPeriods + 1) * bT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  boost: L=" << L * 1e6 << "uH D=" << D
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean
              << " eff=" << r.efficiency << "\n";
}

// ── Flyback ────────────────────────────────────────────────────────────
void gen_flyback(const std::string& outPath) {
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::Flyback f;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    f.set_input_voltage(iv);
    f.set_diode_voltage_drop(0.0);   // ideal
    f.set_efficiency(1.0);
    f.set_current_ripple_ratio(ripple);
    f.set_maximum_duty_cycle(0.5);
    OpenMagnetics::FlybackOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    f.set_operating_points(std::vector<OpenMagnetics::FlybackOperatingPoint>{op});

    auto dr = f.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_nominal().value();
    std::vector<double> turnsRatios;
    for (const auto& tr : dr.get_turns_ratios())
        turnsRatios.push_back(tr.get_nominal().value());

    f.set_num_steady_state_periods(kFlybackSettlingPeriods);
    f.set_num_periods_to_extract(1);
    const std::string deck = f.generate_ngspice_circuit(turnsRatios, Lm);
    auto wfs = f.simulate_and_extract_topology_waveforms(turnsRatios, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "flyback";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs},
                      {"currentRippleRatio", ripple}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", turnsRatios},
                      {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout0"},
                      {"measFrom", kFlybackSettlingPeriods * fT},
                      {"measTo", (kFlybackSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  flyback: Lm=" << Lm * 1e6 << "uH n=" << (turnsRatios.empty() ? 0 : turnsRatios[0])
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean
              << " eff=" << r.efficiency << "\n";
}

// ── Buck ───────────────────────────────────────────────────────────────
void gen_buck(const std::string& outPath) {
    const double Vin = 12.0, Vout = 5.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::Buck b;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    b.set_input_voltage(iv);
    b.set_diode_voltage_drop(0.0);
    b.set_efficiency(1.0);
    b.set_current_ripple_ratio(ripple);
    BaseOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    b.set_operating_points({op});

    auto dr = b.process_design_requirements();
    const double L = dr.get_magnetizing_inductance().get_minimum().value();
    const double D = b.calculate_duty_cycle(Vin, Vout, 0.0, 1.0);

    b.set_num_steady_state_periods(kBuckSettlingPeriods);
    b.set_num_periods_to_extract(1);
    const std::string deck = b.generate_ngspice_circuit(L);
    auto wfs = b.simulate_and_extract_topology_waveforms(L, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "buck";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs},
                      {"currentRippleRatio", ripple}};
    fx["design"]   = {{"inductance", L}, {"dutyCycle", D}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double bT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout"},
                      {"measFrom", kBuckSettlingPeriods * bT},
                      {"measTo", (kBuckSettlingPeriods + 1) * bT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  buck: L=" << L * 1e6 << "uH D=" << D
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean
              << " eff=" << r.efficiency << "\n";
}

// ── Single-switch forward ──────────────────────────────────────────────
void gen_forward(const std::string& outPath) {
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4, Dmax = 0.5;

    OpenMagnetics::SingleSwitchForward f;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    f.set_input_voltage(iv);
    f.set_diode_voltage_drop(0.0);
    f.set_efficiency(1.0);
    f.set_current_ripple_ratio(ripple);
    f.set_duty_cycle(Dmax);
    OpenMagnetics::ForwardOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    f.set_operating_points({op});

    auto dr = f.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_minimum().value();
    std::vector<double> turnsRatios;
    for (const auto& tr : dr.get_turns_ratios()) turnsRatios.push_back(tr.get_nominal().value());

    f.set_num_steady_state_periods(kForwardSettlingPeriods);
    f.set_num_periods_to_extract(1);
    const std::string deck = f.generate_ngspice_circuit(turnsRatios, Lm);
    auto wfs = f.simulate_and_extract_topology_waveforms(turnsRatios, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "forward";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs},
                      {"currentRippleRatio", ripple}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", turnsRatios},
                      {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout0"},
                      {"measFrom", kForwardSettlingPeriods * fT},
                      {"measTo", (kForwardSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  forward: Lm=" << Lm * 1e6 << "uH n=" << (turnsRatios.size() > 1 ? turnsRatios[1] : 0.0)
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean
              << " eff=" << r.efficiency << "\n";
}

// ── Two-switch forward ─────────────────────────────────────────────────
void gen_tsf(const std::string& outPath) {
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4, Dmax = 0.5;

    OpenMagnetics::TwoSwitchForward f;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    f.set_input_voltage(iv);
    f.set_diode_voltage_drop(0.0);
    f.set_efficiency(1.0);
    f.set_current_ripple_ratio(ripple);
    f.set_duty_cycle(Dmax);
    OpenMagnetics::ForwardOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    f.set_operating_points({op});

    auto dr = f.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_minimum().value();
    std::vector<double> turnsRatios;
    for (const auto& tr : dr.get_turns_ratios()) turnsRatios.push_back(tr.get_nominal().value());

    f.set_num_steady_state_periods(kTsfSettlingPeriods);
    f.set_num_periods_to_extract(1);
    const std::string deck = f.generate_ngspice_circuit(turnsRatios, Lm);
    auto wfs = f.simulate_and_extract_topology_waveforms(turnsRatios, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "two_switch_forward";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", turnsRatios}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout0"}, {"measFrom", kTsfSettlingPeriods * fT},
                      {"measTo", (kTsfSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  two_switch_forward: Lm=" << Lm * 1e6 << "uH n=" << (turnsRatios.empty() ? 0.0 : turnsRatios[0])
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── SEPIC ──────────────────────────────────────────────────────────────
void gen_sepic(const std::string& outPath) {
    const double Vin = 12.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::Sepic c;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    c.set_input_voltage(iv);
    c.set_diode_voltage_drop(0.0);
    c.set_efficiency(1.0);
    c.set_current_ripple_ratio(ripple);
    MAS::TopologyExcitation op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    c.set_operating_points({op});

    auto dr = c.process_design_requirements();
    const double L1 = dr.get_magnetizing_inductance().get_minimum().value();

    c.set_num_steady_state_periods(kSepicSettlingPeriods);
    c.set_num_periods_to_extract(1);
    const std::string deck = c.generate_ngspice_circuit(L1);
    auto wfs = c.simulate_and_extract_topology_waveforms(L1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "sepic";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"inductanceL1", L1}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout"}, {"measFrom", kSepicSettlingPeriods * fT},
                      {"measTo", (kSepicSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  sepic: L1=" << L1 * 1e6 << "uH Vout=" << r.voutMean
              << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Cuk (inverting) ────────────────────────────────────────────────────
void gen_cuk(const std::string& outPath) {
    const double Vin = 12.0, VoutMag = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::Cuk c;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    c.set_input_voltage(iv);
    c.set_diode_voltage_drop(0.0);
    c.set_efficiency(1.0);
    c.set_current_ripple_ratio(ripple);
    MAS::CukOperatingPoint op;
    op.set_output_voltages({VoutMag});   // magnitude — Cuk treats as |Vo|
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    c.set_operating_points({op});

    auto dr = c.process_design_requirements();
    const double L1 = dr.get_magnetizing_inductance().get_minimum().value();

    c.set_num_steady_state_periods(kCukSettlingPeriods);
    c.set_num_periods_to_extract(1);
    const std::string deck = c.generate_ngspice_circuit(L1);
    auto wfs = c.simulate_and_extract_topology_waveforms(L1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "cuk";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", VoutMag}, {"outputCurrent", Iout},
                      {"outputPower", VoutMag * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"inductanceL1", L1}, {"loadResistance", VoutMag / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout_load_node"}, {"measFrom", kCukSettlingPeriods * fT},
                      {"measTo", (kCukSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  cuk: L1=" << L1 * 1e6 << "uH Vout=" << r.voutMean
              << " Iout=" << r.ioutMean << " eff=" << r.efficiency << " (inverting)\n";
}

// ── Zeta ───────────────────────────────────────────────────────────────
void gen_zeta(const std::string& outPath) {
    const double Vin = 12.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::Zeta c;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    c.set_input_voltage(iv);
    c.set_diode_voltage_drop(0.0);
    c.set_efficiency(1.0);
    c.set_current_ripple_ratio(ripple);
    MAS::TopologyExcitation op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    c.set_operating_points({op});

    auto dr = c.process_design_requirements();
    const double L1 = dr.get_magnetizing_inductance().get_minimum().value();

    c.set_num_steady_state_periods(kZetaSettlingPeriods);
    c.set_num_periods_to_extract(1);
    const std::string deck = c.generate_ngspice_circuit(L1);
    auto wfs = c.simulate_and_extract_topology_waveforms(L1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "zeta";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"inductanceL1", L1}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout"}, {"measFrom", kZetaSettlingPeriods * fT},
                      {"measTo", (kZetaSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  zeta: L1=" << L1 * 1e6 << "uH Vout=" << r.voutMean
              << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Push-pull ──────────────────────────────────────────────────────────
void gen_push_pull(const std::string& outPath) {
    const double Vin = 24.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::PushPull pp;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    pp.set_input_voltage(iv);
    pp.set_diode_voltage_drop(0.0);
    pp.set_efficiency(1.0);
    pp.set_current_ripple_ratio(ripple);
    pp.set_duty_cycle(0.48);   // matches Kirchhoff's kMaxDuty
    PushPullOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    pp.set_operating_points(std::vector<PushPullOperatingPoint>{op});

    auto dr = pp.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_minimum().value();
    std::vector<double> tr;
    for (const auto& t : dr.get_turns_ratios()) tr.push_back(t.get_nominal().value());

    pp.set_num_steady_state_periods(kPushPullSettlingPeriods);
    pp.set_num_periods_to_extract(1);
    const std::string deck = pp.generate_ngspice_circuit(tr, Lm);
    auto wfs = pp.simulate_and_extract_topology_waveforms(tr, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "push_pull";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", tr}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout"}, {"measFrom", kPushPullSettlingPeriods * fT},
                      {"measTo", (kPushPullSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  push_pull: Lm=" << Lm * 1e6 << "uH N=" << (tr.size() > 1 ? tr[1] : 0.0)
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Phase-Shifted Full Bridge ──────────────────────────────────────────
void gen_psfb(const std::string& outPath) {
    // 48->12V/24W, Fs=100kHz, phase shift 126 deg (commanded effective duty D_cmd = 126/180 = 0.7).
    // FULL_BRIDGE rectifier: MKF's center-tapped PSFB deck is a fake CT (one full secondary with the
    // "center tap" pinned at one end) -> effectively half-wave -> delivers ~half the designed Vout
    // (5.68V here). The full-bridge rectifier deck is correct (11.84V). (Surfaced to the user; the
    // Kirchhoff port mirrors the working full-bridge variant.)
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, phi = 126.0;

    OpenMagnetics::Psfb p;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    p.set_input_voltage(iv);
    p.set_rectifier_type(BRectifierType::FULL_BRIDGE);
    PsfbOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_phase_shift(phi);
    op.set_ambient_temperature(25.0);
    p.set_operating_points(std::vector<PsfbOperatingPoint>{op});

    auto dr = p.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_nominal().value();
    std::vector<double> tr;
    for (const auto& t : dr.get_turns_ratios()) tr.push_back(t.get_nominal().value());

    p.set_num_steady_state_periods(kPsfbSettlingPeriods);
    p.set_num_periods_to_extract(1);
    const std::string deck = p.generate_ngspice_circuit(tr, Lm, 0, 0);
    auto wfs = p.simulate_and_extract_topology_waveforms(tr, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "psfb";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", 0.3}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", tr},
                      {"seriesInductance", p.get_computed_series_inductance()},
                      {"outputInductance", p.get_computed_output_inductance()},
                      {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "out_node_o1"}, {"measFrom", kPsfbSettlingPeriods * fT},
                      {"measTo", (kPsfbSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  psfb: n=" << (tr.empty() ? 0.0 : tr[0]) << " Lm=" << Lm * 1e6 << "uH"
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Asymmetric Half-Bridge ─────────────────────────────────────────────
void gen_ahb(const std::string& outPath) {
    // 48->12V/24W, Fs=100kHz, operating duty D=0.45. FULL_BRIDGE rectifier (the center-tapped AHB deck
    // fails to converge here; full-bridge is correct, 11.80V). Gain Vo=2*D*(1-D)*Vin/n.
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, D = 0.45;

    OpenMagnetics::AsymmetricHalfBridge p;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    p.set_input_voltage(iv);
    p.set_rectifier_type(AhbRectifierType::FULL_BRIDGE);
    p.set_efficiency(1.0);
    AhbOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_duty_cycle(D);
    op.set_ambient_temperature(25.0);
    p.set_operating_points(std::vector<AhbOperatingPoint>{op});

    auto dr = p.process_design_requirements();
    const double Lm = p.get_computed_magnetizing_inductance();
    std::vector<double> tr;
    for (const auto& t : dr.get_turns_ratios()) tr.push_back(t.get_nominal().value());

    p.set_num_steady_state_periods(kAhbSettlingPeriods);
    p.set_num_periods_to_extract(1);
    const std::string deck = p.generate_ngspice_circuit(tr, Lm, 0, 0);
    auto wfs = p.simulate_and_extract_topology_waveforms(tr, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "ahb";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", 0.3}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", tr},
                      {"dcBlockingCapacitance", p.get_computed_dc_blocking_capacitance()},
                      {"outputInductance", p.get_computed_output_inductance()},
                      {"dutyCycle", D}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "out_node"}, {"measFrom", kAhbSettlingPeriods * fT},
                      {"measTo", (kAhbSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  ahb: n=" << (tr.empty() ? 0.0 : tr[0]) << " Lm=" << Lm * 1e6 << "uH"
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Active-Clamp Forward ───────────────────────────────────────────────
void gen_acf(const std::string& outPath) {
    // 48->12V/24W, Fs=100kHz, D=0.45. Forward with active-clamp reset; single forward output diode, so
    // (like the forward) MKF designs with diodeVoltageDrop=0 and both decks share the same ideal diode.
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, D = 0.45, ripple = 0.4;

    OpenMagnetics::ActiveClampForward p;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    p.set_input_voltage(iv);
    p.set_diode_voltage_drop(0.0);
    p.set_efficiency(1.0);
    p.set_current_ripple_ratio(ripple);
    p.set_duty_cycle(D);
    ForwardOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    p.set_operating_points(std::vector<ForwardOperatingPoint>{op});

    auto dr = p.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_minimum().value_or(
                      dr.get_magnetizing_inductance().get_nominal().value_or(0));
    std::vector<double> tr;
    for (const auto& t : dr.get_turns_ratios()) tr.push_back(t.get_nominal().value());

    p.set_num_steady_state_periods(kAcfSettlingPeriods);
    p.set_num_periods_to_extract(1);
    const std::string deck = p.generate_ngspice_circuit(tr, Lm, 0, 0);
    auto wfs = p.simulate_and_extract_topology_waveforms(tr, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "acf";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", tr}, {"dutyCycle", D},
                      {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout0"}, {"measFrom", kAcfSettlingPeriods * fT},
                      {"measTo", (kAcfSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  acf: n=" << (tr.empty() ? 0.0 : tr[0]) << " Lm=" << Lm * 1e6 << "uH"
              << " Vout=" << r.voutMean << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Four-Switch Buck-Boost (buck-boost region, simultaneous mode) ───────
void gen_fsbb(const std::string& outPath) {
    // Vin=Vout=12 -> BUCK_BOOST region; SIMULTANEOUS mode (M=D/(1-D), D=Vo/(Vin+Vo)=0.5). Non-isolated,
    // all four switches synchronous (no rectifier diodes) -> high efficiency (~0.98).
    const double Vin = 12.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, ripple = 0.4;

    OpenMagnetics::FourSwitchBuckBoost p;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    p.set_input_voltage(iv);
    p.set_efficiency(1.0);
    p.set_current_ripple_ratio(ripple);
    p.set_transition_mode(MAS::TransitionMode::SIMULTANEOUS);
    TopologyExcitation op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    p.set_operating_points(std::vector<TopologyExcitation>{op});

    auto dr = p.process_design_requirements();
    const double L = dr.get_magnetizing_inductance().get_minimum().value_or(
                     dr.get_magnetizing_inductance().get_nominal().value_or(0));

    p.set_num_steady_state_periods(kFsbbSettlingPeriods);
    p.set_num_periods_to_extract(1);
    const std::string deck = p.generate_ngspice_circuit(L, 0, 0);
    auto wfs = p.simulate_and_extract_topology_waveforms(L, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "fsbb";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", ripple}};
    fx["design"]   = {{"inductance", L}, {"dutyCycle", 0.5}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean},
                      {"vinMean", r.vinMean}, {"iinMean", r.iinMean},
                      {"pin", r.pin}, {"pout", r.pout}, {"efficiency", r.efficiency},
                      {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout"}, {"measFrom", kFsbbSettlingPeriods * fT},
                      {"measTo", (kFsbbSettlingPeriods + 1) * fT}};
    fx["deck"]     = deck;
    write_fixture(outPath, fx);

    std::cout << "  fsbb: L=" << L * 1e6 << "uH Vout=" << r.voutMean
              << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Phase-Shifted Half-Bridge (3-level NPC) ────────────────────────────
void gen_pshb(const std::string& outPath) {
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3, phi = 126.0;
    OpenMagnetics::Pshb p;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    p.set_input_voltage(iv);
    p.set_rectifier_type(BRectifierType::FULL_BRIDGE);
    PshbOperatingPoint op;
    op.set_output_voltages({Vout}); op.set_output_currents({Iout});
    op.set_switching_frequency(Fs); op.set_phase_shift(phi); op.set_ambient_temperature(25.0);
    p.set_operating_points(std::vector<PshbOperatingPoint>{op});
    auto dr = p.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_nominal().value();
    std::vector<double> tr; for (const auto& t : dr.get_turns_ratios()) tr.push_back(t.get_nominal().value());
    p.set_num_steady_state_periods(kPshbSettlingPeriods); p.set_num_periods_to_extract(1);
    const std::string deck = p.generate_ngspice_circuit(tr, Lm, 0, 0);
    auto wfs = p.simulate_and_extract_topology_waveforms(tr, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));
    json fx;
    fx["topology"] = "pshb";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", 0.3}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", tr}, {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean}, {"vinMean", r.vinMean},
                      {"iinMean", r.iinMean}, {"pin", r.pin}, {"pout", r.pout},
                      {"efficiency", r.efficiency}, {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "out_node_o1"}, {"measFrom", kPshbSettlingPeriods * fT},
                      {"measTo", (kPshbSettlingPeriods + 1) * fT}};
    fx["deck"] = deck;
    write_fixture(outPath, fx);
    std::cout << "  pshb: n=" << (tr.empty()?0.0:tr[0]) << " Vout=" << r.voutMean
              << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

// ── Dual Active Bridge (SPS) ───────────────────────────────────────────
void gen_dab(const std::string& outPath) {
    // 48->12V/24W, Fs=100kHz, SPS modulation (D1=D2=0); MKF picks D3≈25° and solves L. Two ACTIVE
    // bridges (8 switches) phase-shifted by D3; no output inductor, so the settled Vout (~10.56V)
    // droops below the lossless 12V target by the deck's snubber/switch conduction loss.
    const double Vin = 48.0, Vout = 12.0, Iout = 2.0, Fs = 100e3;

    OpenMagnetics::Dab p;
    DimensionWithTolerance iv;
    iv.set_nominal(Vin); iv.set_minimum(Vin * 0.95); iv.set_maximum(Vin * 1.05);
    p.set_input_voltage(iv);
    p.set_efficiency(1.0);
    DabOperatingPoint op;
    op.set_output_voltages({Vout});
    op.set_output_currents({Iout});
    op.set_switching_frequency(Fs);
    op.set_ambient_temperature(25.0);
    op.set_modulation_type(MAS::ModulationType::SPS);
    p.set_operating_points(std::vector<DabOperatingPoint>{op});

    auto dr = p.process_design_requirements();
    const double Lm = dr.get_magnetizing_inductance().get_minimum().value();
    std::vector<double> tr;
    for (const auto& t : dr.get_turns_ratios()) tr.push_back(t.get_nominal().value());

    p.set_num_steady_state_periods(kDabSettlingPeriods);
    p.set_num_periods_to_extract(1);
    const std::string deck = p.generate_ngspice_circuit(tr, Lm, 0, 0);
    auto wfs = p.simulate_and_extract_topology_waveforms(tr, Lm, 1);
    SimRead r = read_waveforms(wfs.at(0));

    json fx;
    fx["topology"] = "dab";
    fx["inputs"]   = {{"inputVoltage", Vin}, {"outputVoltage", Vout}, {"outputCurrent", Iout},
                      {"outputPower", Vout * Iout}, {"switchingFrequency", Fs}, {"currentRippleRatio", 0.3}};
    fx["design"]   = {{"magnetizingInductance", Lm}, {"turnsRatios", tr},
                      {"seriesInductance", p.get_computed_series_inductance()},
                      {"phaseShiftDeg", p.get_computed_d3_rad() * 180.0 / M_PI},
                      {"loadResistance", Vout / Iout}};
    fx["sim"]      = {{"voutMean", r.voutMean}, {"ioutMean", r.ioutMean}, {"vinMean", r.vinMean},
                      {"iinMean", r.iinMean}, {"pin", r.pin}, {"pout", r.pout},
                      {"efficiency", r.efficiency}, {"voutRipplePkPk", r.voutRipplePkPk}};
    const double fT = 1.0 / Fs;
    fx["probes"]   = {{"voutNode", "vout_cap_o1"}, {"measFrom", kDabSettlingPeriods * fT},
                      {"measTo", (kDabSettlingPeriods + 1) * fT}};
    fx["deck"] = deck;
    write_fixture(outPath, fx);
    std::cout << "  dab: n=" << (tr.empty()?0.0:tr[0]) << " D3=" << p.get_computed_d3_rad()*180.0/M_PI
              << "deg Vout=" << r.voutMean << " Iout=" << r.ioutMean << " eff=" << r.efficiency << "\n";
}

}  // namespace

// Optional topology-name filter: `gen_mkf_reference <dir> [name1 name2 ...]` regenerates only the
// named fixtures (so a single new topology doesn't require re-running the whole multi-minute suite).
int main(int argc, char** argv) {
    std::string dir = argc > 1 ? argv[1] : ".";
    std::vector<std::string> only(argv + std::min(argc, 2), argv + argc);  // names after <dir>
    auto want = [&](const std::string& name) {
        return only.empty() || std::find(only.begin(), only.end(), name) != only.end();
    };
    try {
        if (want("boost"))              gen_boost(dir + "/boost.mkf.json");
        if (want("flyback"))            gen_flyback(dir + "/flyback.mkf.json");
        if (want("buck"))               gen_buck(dir + "/buck.mkf.json");
        if (want("forward"))            gen_forward(dir + "/forward.mkf.json");
        if (want("two_switch_forward")) gen_tsf(dir + "/two_switch_forward.mkf.json");
        if (want("sepic"))              gen_sepic(dir + "/sepic.mkf.json");
        if (want("cuk"))                gen_cuk(dir + "/cuk.mkf.json");
        if (want("zeta"))               gen_zeta(dir + "/zeta.mkf.json");
        if (want("push_pull"))          gen_push_pull(dir + "/push_pull.mkf.json");
        if (want("psfb"))               gen_psfb(dir + "/psfb.mkf.json");
        if (want("ahb"))                gen_ahb(dir + "/ahb.mkf.json");
        if (want("acf"))                gen_acf(dir + "/acf.mkf.json");
        if (want("fsbb"))               gen_fsbb(dir + "/fsbb.mkf.json");
        if (want("pshb"))               gen_pshb(dir + "/pshb.mkf.json");
        if (want("dab"))                gen_dab(dir + "/dab.mkf.json");
    } catch (const std::exception& e) {
        std::cerr << "MKF reference generation FAILED: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
