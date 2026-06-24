#pragma once

// Kirchhoff design configuration — NO bare magic literals in topology code.
//
// Every otherwise-hardcoded design/auxiliary value is read from the TAS input doc's optional `config`
// object via cfg::get(...). When the user supplies no override, the value comes from either a PRINCIPLED
// RULE derived from the circuit's own quantities (power, blocking voltage, switching frequency, load) or a
// named, documented engineering default — never an unexplained constant scattered in the code.
//
//   tasInputs["config"] = {                 // all optional; omit any to take the principled default
//     "snubberCap":        2.2e-9,          // force an explicit snubber capacitance (F)
//     "snubberEnergyFrac": 1e-3,            // or just retune the rule's loss budget
//     "vDerate":           0.8, ...
//   }
//
// The dimensionless rule factors (energy/loss budgets, damping fraction) ARE design parameters, not magic:
// each has a documented meaning and is itself overridable. The rules make the auxiliary values SCALE with
// the converter (a snubber on an 800 V bus must differ from one on a 5 V rail) instead of a fixed literal.

#include <nlohmann/json.hpp>
#include <cmath>
#include <algorithm>
#include <string>

namespace Kirchhoff {
namespace cfg {

using nlohmann::json;

// The config object is tasInputs["config"] (may be absent/empty). Designs extract it once and carry it on
// the Design struct so the build stage can read it too.
inline json object_of(const json& tasInputs) {
    return tasInputs.is_object() ? tasInputs.value("config", json::object()) : json::object();
}

// Read config[key] if the user supplied it; otherwise return the principled/derived fallback.
inline double get(const json& config, const char* key, double fallback) {
    if (config.is_object() && config.contains(key) && config.at(key).is_number())
        return config.at(key).get<double>();
    return fallback;
}

// ── Documented, overridable dimensionless design-parameter defaults ──────────────────────────────────
// (These are the *rule* knobs. A user who wants a different policy sets them in config; nothing in a
//  topology hardcodes them.)
constexpr double kSnubberEnergyFrac = 5e-3;   // ACROSS-DEVICE snubber may store <= this fraction of throughput/cycle
constexpr double kSnubberDampFrac   = 0.01;   // series-RC / R∥C reset time R*C = this fraction of the switching period
// Numerical commutation-snubber caps (see kNodeSnubberCap note — solver constants, not formulas; a P/V²
// derivation over-sizes them at the rectifier's low blocking voltage). Overridable per design via config.
constexpr double kRectifierSnubberCap = 100e-12;  // R∥C across resonant/rectifier diodes (LLC/SRC/Weinberg)
constexpr double kDiodeSnubberCap     = 1e-9;     // series-RC across a hard-commutated freewheel diode (Cuk)
// Bridge midpoint node cap — a NUMERICAL solver constant, NOT a formula. It tames the ideal-switch floating-
// midpoint dV/dt; empirically the same small value works across the whole bridge family (5 V → 800 V). An
// operating-point derivation (C ∝ P/V²) was tried and REJECTED: it over-sizes at low Vin and shifts the
// commutation droop enough to fail points. So it is a named, documented, overridable constant — never a
// scattered literal. Override per design via config "nodeSnubberCap".
constexpr double kNodeSnubberCap    = 2.2e-9;
constexpr double kBiasLossFrac      = 1e-3;   // a DC-bias/bleed resistor dissipates <= this fraction of rated P
constexpr double kLoopBreakerFrac   = 1e-4;   // numerical loop-breaker R as a fraction of the reflected load R
constexpr double kVoltageDerate     = 0.8;    // operate devices at <= this fraction of rating (IPC-9592)

// ── Principled auxiliary-component rules (for the ideal-switch ngspice decks) ─────────────────────────

// Snubber capacitance. Sized from an ENERGY BUDGET: the cap may store/return at most `eps` of the energy
// the stage moves per switching cycle, so it neither dissipates meaningfully nor detunes the power train:
//     C = eps · P / (V_block^2 · f_sw).
// Scales correctly with power, blocking voltage and frequency (unlike a fixed 100 pF / 1 nF / 2.2 nF).
inline double snubber_cap(const json& in, double P, double Vblock, double fsw) {
    const double eps = get(in, "snubberEnergyFrac", kSnubberEnergyFrac);
    return get(in, "snubberCap", eps * P / (Vblock * Vblock * fsw));
}

// Numerical commutation-snubber capacitances — documented constants above, each overridable. (Not
// operating-point-derived; see kNodeSnubberCap for why a formula is wrong for these.)
inline double node_snubber_cap(const json& in)      { return get(in, "nodeSnubberCap",      kNodeSnubberCap); }
inline double rectifier_snubber_cap(const json& in) { return get(in, "rectifierSnubberCap", kRectifierSnubberCap); }
inline double diode_snubber_cap(const json& in)     { return get(in, "diodeSnubberCap",     kDiodeSnubberCap); }

// Series-RC (or R∥C) snubber resistance. The snubber's RC reset time is a small fraction `zeta` of the
// switching period — long enough to damp the commutation transient, short enough to reset every cycle:
//     R = zeta / (f_sw · C).   (Needs only the period and C, not an ill-defined ideal-deck parasitic L.)
inline double snubber_res(const json& in, double fsw, double Csnub) {
    const double zeta = get(in, "snubberDampFrac", kSnubberDampFrac);
    return get(in, "snubberRes", zeta / (fsw * Csnub));
}

// DC-bias / bleed resistor (a node that needs a defined DC path — e.g. a DAB floating midpoint). Sized to
// dissipate at most `delta` of rated power at the blocking voltage:  R = V_block^2 / (delta · P).
inline double bias_res(const json& in, double Vblock, double P) {
    const double delta = get(in, "biasLossFrac", kBiasLossFrac);
    return get(in, "biasRes", Vblock * Vblock / (delta * P));
}

// Numerical loop-breaker resistance for an otherwise-singular all-inductor mesh: the smallest R that keeps
// ngspice non-singular without dissipating — a tiny fraction of the reflected load resistance.
inline double loop_breaker_res(const json& in, double Rload) {
    const double frac = get(in, "loopBreakerFrac", kLoopBreakerFrac);
    return get(in, "loopBreakerRes", frac * Rload);
}

// Device voltage derating (required rating = peak stress / derate).
inline double v_derate(const json& in) { return get(in, "vDerate", kVoltageDerate); }

} // namespace cfg
} // namespace Kirchhoff
