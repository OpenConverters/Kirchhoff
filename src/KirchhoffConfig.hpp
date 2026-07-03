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
#include <cctype>
#include <stdexcept>
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

// String variant of get(...) — for categorical design knobs supplied as strings (e.g. "rectifierType":
// "centerTapped" | "fullBridge" | "currentDoubler" | "voltageDoubler"). Same no-magic policy: the
// topology passes its principled default; the user may override via the config object. Comparison is
// done case-insensitively by the consumer (see rectifierType parsing in the resonant/bridge topologies).
inline std::string get_str(const json& config, const char* key, const char* fallback) {
    if (config.is_object() && config.contains(key) && config.at(key).is_string())
        return config.at(key).get<std::string>();
    return fallback;
}

// Boolean variant of get(...) — for on/off design-variant flags (e.g. "coupledInductor", "bidirectional",
// "isolated"). Accepts a JSON boolean; anything else keeps the topology's principled default.
inline bool get_bool(const json& config, const char* key, bool fallback) {
    if (config.is_object() && config.contains(key) && config.at(key).is_boolean())
        return config.at(key).get<bool>();
    return fallback;
}

// Primary-bridge topology flag (ABT #91). Parses config[key] = "halfBridge" (default) | "fullBridge"
// (case/space/underscore/hyphen-insensitive) into "is it a full bridge?". Half-bridge is a split-cap leg
// driving the tank at ±Vin/2 (bridge factor 0.5); full-bridge is a 4-MOSFET primary driving at ±Vin
// (factor 1.0). An ABSENT/empty value keeps the topology's principled default (half-bridge for LLC/SRC); a
// PRESENT but unrecognized string is malformed input and THROWS (no silent default — that would build a
// different primary than the user asked for, mirroring parse_rectifier_type).
inline bool full_bridge_selected(const json& config, const char* key = "bridgeType",
                                 bool fallback = false) {
    std::string s = get_str(config, key, fallback ? "fullBridge" : "halfBridge"), t;
    for (char c : s)
        if (!std::isspace(static_cast<unsigned char>(c)) && c != '_' && c != '-')
            t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (t.empty()) return fallback;
    if (t == "halfbridge" || t == "hb") return false;
    if (t == "fullbridge" || t == "fb") return true;
    throw std::invalid_argument("Kirchhoff: unknown bridgeType '" + s + "'");
}

// ── Documented, overridable dimensionless design-parameter defaults ──────────────────────────────────
// (These are the *rule* knobs. A user who wants a different policy sets them in config; nothing in a
//  topology hardcodes them.)
constexpr double kSnubberEnergyFrac = 5e-3;   // ACROSS-DEVICE snubber may store <= this fraction of throughput/cycle
constexpr double kSnubberRes        = 100.0;  // series-RC / R∥C snubber damping R [Ω] — like the snubber CAP, a
                                              // numerical constant (a √(L/C) or RC-reset derivation gives different
                                              // values per C and shifts the LLC tank's input-current/efficiency
                                              // artifact); empirically 100 Ω damps the whole family. Overridable.
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
constexpr double kNodeShuntCap      = 1e-12;  // ngspice cshunt: tiny node-to-ground cap, REAL decks only (see below)
constexpr double kNodeShuntRes      = 1e9;    // ngspice rshunt: large node-to-ground R, REAL decks only (see below)
constexpr double kTranIterLimit     = 100;    // ngspice itl4: transient Newton iterations per timestep (default 10)

// ── Principled auxiliary-component rules (for the ideal-switch ngspice decks) ─────────────────────────

// Snubber capacitance. Sized from an ENERGY BUDGET: the cap may store/return at most `eps` of the energy
// the stage moves per switching cycle, so it neither dissipates meaningfully nor detunes the power train:
//     C = eps · P / (V_block^2 · f_sw).
// Scales correctly with power, blocking voltage and frequency (unlike a fixed 100 pF / 1 nF / 2.2 nF).
inline double snubber_cap(const json& in, double P, double Vblock, double fsw) {
    const double eps = get(in, "snubberEnergyFrac", kSnubberEnergyFrac);
    return get(in, "snubberCap", eps * P / (Vblock * Vblock * fsw));
}

// Explicit numerical-aid marker (ABT #96). A numerical convergence snubber / loop-breaker exists ONLY to
// tame the infinite dV/dt of an IDEAL switch so ngspice converges; at real fidelity the switch's own Coss
// does that physically, so the assembler STRIPS it (see TasAssembler::is_numerical_aid). That strip used to
// be inferred from the refdes prefix (Csn*/Rsn*/Csw*), which silently deleted any REAL, sourceable part
// whose refdes happened to match (e.g. a real "Csnubber"). Instead we tag each numerical aid EXPLICITLY, on
// a schema-legal field: inputs.designRequirements.name == kNumericalAidName. It is a DEDICATED marker, never
// the electrical refdes, so a real part can never be stripped by accident — a missed tag merely leaves a
// redundant snubber in the deck (safe over-damping), never removes a real component. `name` is the only free
// string the closed CAS/RAS designRequirements schemas expose (role is a fixed enum; extra keys are rejected
// by unevaluatedProperties:false), so the marker lives there.
inline const char* kNumericalAidName = "__kh_numerical_aid__";

// Tag a numerical-aid component SEED (a capacitor/resistor `data` json, i.e. {capacitor|resistor, inputs})
// so the real-fidelity strip removes it explicitly. Idempotent; overwrites any prior designRequirements.name.
inline void mark_numerical_aid(json& seed) {
    seed["inputs"]["designRequirements"]["name"] = kNumericalAidName;
}

// True iff a component `data` json carries the explicit numerical-aid marker.
inline bool is_numerical_aid(const json& data) {
    return data.is_object() && data.contains("inputs") && data.at("inputs").is_object()
        && data.at("inputs").contains("designRequirements") && data.at("inputs").at("designRequirements").is_object()
        && data.at("inputs").at("designRequirements").contains("name")
        && data.at("inputs").at("designRequirements").at("name") == kNumericalAidName;
}

// Numerical commutation-snubber capacitances — documented constants above, each overridable. (Not
// operating-point-derived; see kNodeSnubberCap for why a formula is wrong for these.)
inline double node_snubber_cap(const json& in)      { return get(in, "nodeSnubberCap",      kNodeSnubberCap); }
inline double rectifier_snubber_cap(const json& in) { return get(in, "rectifierSnubberCap", kRectifierSnubberCap); }
inline double diode_snubber_cap(const json& in)     { return get(in, "diodeSnubberCap",     kDiodeSnubberCap); }

// Series-RC / R∥C snubber damping resistance — the documented numerical constant above, overridable.
inline double snubber_res(const json& in) {
    return get(in, "snubberRes", kSnubberRes);
}

// ngspice `cshunt` — a tiny capacitance the solver places from every node to ground, giving each node a
// defined dV/dt so a stiff node (e.g. a resonant tank whose switch body diode was stripped on a real-device
// deck) does not go singular ("timestep too small"). Emitted on REAL-semiconductor decks only: at 1e-12 F a
// real deck's loose real-loss bands absorb it, but it would detune the tightly-pinned IDEAL decks past their
// 5% tolerance. A solver setting like reltol/abstol, but circuit-affecting, so it lives here and is overridable.
inline double node_shunt_cap(const json& in) { return get(in, "nodeShuntCap", kNodeShuntCap); }

// ngspice `rshunt` — a large resistance from every node to ground. Where cshunt gives a node a defined dV/dt,
// rshunt gives it a DC reference, which is what a STIFF MKF_MODEL core needs: a real-core subcircuit (coupled
// inductors near unity coupling, or a frequency-dependent-loss R-L ladder) presents a near-singular branch the
// DC operating point / transient cannot resolve from cshunt alone, so it diverges "across the whole bracket"
// (ABT #33). At 1e9 Ohm the leakage (~nA) is negligible vs any converter load, so it breaks the singularity
// without detuning Vout. REAL decks only (same gating as cshunt). Overridable.
inline double node_shunt_res(const json& in) { return get(in, "nodeShuntRes", kNodeShuntRes); }

// ngspice `itl4` — max transient Newton iterations per timestep. ngspice cuts the timestep when a point does
// not converge in itl4 iterations; for a stiff core that bottoms out at "timestep too small". Raising it lets
// the solver work harder at a hard point instead of collapsing the step. REAL decks only. Overridable.
inline double tran_iter_limit(const json& in) { return get(in, "tranIterLimit", kTranIterLimit); }

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

// Device voltage derating (required rating = peak stress / derate). Default kVoltageDerate (0.8,
// IPC-9592 1.25x). The general "vDerate" sets all device classes at once.
inline double v_derate(const json& in) { return get(in, "vDerate", kVoltageDerate); }

// Per-device-class voltage derating, so a consumer (e.g. HS's realism gate) can drive a DIFFERENT
// derating per class — FET 1.5x, diode 1.3x, capacitor 1.5x — instead of one uniform factor. Each
// per-class knob falls back to the general "vDerate", then to the class default, so existing configs
// that set only "vDerate" (or nothing) are unchanged. required rating = peak stress / derate, so a
// SMALLER derate fraction => a LARGER required rating (stricter).
inline double v_derate_mosfet(const json& in)    { return get(in, "vDerateMosfet",    v_derate(in)); }
inline double v_derate_diode(const json& in)     { return get(in, "vDerateDiode",     v_derate(in)); }
inline double v_derate_capacitor(const json& in) { return get(in, "vDerateCapacitor", v_derate(in)); }

// ── Per-topology design knobs (externally configurable so HS can drive the design) ────────────────────
// Each takes the topology's principled default and lets config override it. HS sets these in
// tasInputs["config"] to steer Kirchhoff's sizing (e.g. to match its own realism-gate policy).

// Inductor current-ripple ratio ΔI_L/I (sizes the magnetizing inductance). Default differs per topology
// (buck/boost/forward 0.4, push/half-bridge 0.3), passed in as `dflt`; config "rippleRatio" overrides.
inline double ripple_ratio(const json& in, double dflt) { return get(in, "rippleRatio", dflt); }

// Output-voltage ripple as a fraction of Vout (sizes the output capacitor). config "outputRippleFraction".
inline double output_ripple_fraction(const json& in, double dflt = 0.01) { return get(in, "outputRippleFraction", dflt); }

// Switch Rds(on) loss budget as a fraction of rated power (the maxOnResistance requirement). config "rdsOnLossFraction".
inline double rds_on_loss_fraction(const json& in, double dflt = 0.01) { return get(in, "rdsOnLossFraction", dflt); }

// Ideal-deck transient analysis window/step (the real-deck path overrides these in regulate). config
// "tranStopTime" / "tranMaxTimeStep".
inline double tran_stop_time(const json& in, double dflt = 0.004)   { return get(in, "tranStopTime", dflt); }
inline double tran_max_timestep(const json& in, double dflt = 5e-8) { return get(in, "tranMaxTimeStep", dflt); }

} // namespace cfg
} // namespace Kirchhoff
