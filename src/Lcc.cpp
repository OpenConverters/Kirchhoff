#include "Lcc.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include "processors/WaveformProcessor.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace analytical {
namespace {

using State = std::array<double, 4>;   // tank current i, series-cap voltage vs, parallel-cap voltage vp, magnetizing im
enum Mode { kOpen = 0, kClampPos = 1, kClampNeg = -1 };
constexpr int kStepsPerHalf = 800;

State derivative(const LccCircuit& c, const State& x, int mode, double vb) {
    State d;
    d[0] = (vb - x[1] - x[2]) / c.seriesInductance;
    d[1] = x[0] / c.seriesCapacitance;
    d[2] = (mode == kOpen) ? (x[0] - x[3]) / c.parallelCapacitance : 0.0;
    d[3] = x[2] / c.magnetizingInductance;
    return d;
}

State rk4(const LccCircuit& c, const State& x, int mode, double vb, double h) {
    auto add = [](const State& a, const State& b, double s) { State r; for (int k = 0; k < 4; ++k) r[k] = a[k] + s * b[k]; return r; };
    const State k1 = derivative(c, x, mode, vb);
    const State k2 = derivative(c, add(x, k1, h / 2), mode, vb);
    const State k3 = derivative(c, add(x, k2, h / 2), mode, vb);
    const State k4 = derivative(c, add(x, k3, h), mode, vb);
    State r;
    for (int k = 0; k < 4; ++k) r[k] = x[k] + h / 6.0 * (k1[k] + 2 * k2[k] + 2 * k3[k] + k4[k]);
    return r;
}

// The conduction state a point implies: clamped while the parallel cap sits at ±Vc and the rectifier current
// i − im flows into the clamp; open otherwise.
int mode_of(const LccCircuit& c, State& x) {
    const double ir = x[0] - x[3];
    if (x[2] >= c.clampVoltage && ir > 0) { x[2] = c.clampVoltage; return kClampPos; }
    if (x[2] <= -c.clampVoltage && ir < 0) { x[2] = -c.clampVoltage; return kClampNeg; }
    return kOpen;
}

// The event a step crosses, as a signed function that rises through zero at the event.
double event_fn(const LccCircuit& c, const State& x, int mode, int which) {
    if (mode == kOpen) return which > 0 ? x[2] - c.clampVoltage : -(x[2] + c.clampVoltage);
    return mode == kClampPos ? -(x[0] - x[3]) : (x[0] - x[3]);
}

struct Sample { double t; State x; int mode; };

// Integrate the +Vb half period from x0. Records every step end and every event when `rec` is given.
State half_period(const LccCircuit& c, State x, std::vector<Sample>* rec) {
    const double Th = 0.5 / c.switchingFrequency, h = Th / kStepsPerHalf, vb = c.busVoltage;
    int mode = mode_of(c, x);
    double t = 0.0;
    if (rec) rec->push_back({t, x, mode});
    for (int s = 0; s < kStepsPerHalf; ++s) {
        double remaining = h;
        for (int guard = 0; remaining > 0 && guard < 8; ++guard) {
            const State xn = rk4(c, x, mode, vb, remaining);
            int hit = 0;
            double gOld = 0;
            for (int which : (mode == kOpen ? std::array<int, 2>{1, -1} : std::array<int, 2>{1, 1})) {
                const double g0 = event_fn(c, x, mode, which), g1 = event_fn(c, xn, mode, which);
                if (g0 < 0 && g1 >= 0) { hit = which; gOld = g0; break; }
            }
            if (!hit) { x = xn; t += remaining; remaining = 0; break; }
            (void)gOld;
            double lo = 0, hi = remaining;
            for (int it = 0; it < 60; ++it) {
                const double mid = 0.5 * (lo + hi);
                if (event_fn(c, rk4(c, x, mode, vb, mid), mode, hit) < 0) lo = mid; else hi = mid;
            }
            x = rk4(c, x, mode, vb, hi);
            t += hi;
            remaining -= hi;
            // the event instant in the old mode too: entering a clamp, the rectifier current steps from zero to
            // the capacitor current, and a single post-switch sample would smear that step over a whole interval
            if (rec) rec->push_back({t, x, mode});
            if (mode == kOpen) { mode = hit > 0 ? kClampPos : kClampNeg; x[2] = hit > 0 ? c.clampVoltage : -c.clampVoltage; }
            else mode = kOpen;
            if (rec) rec->push_back({t, x, mode});
        }
        if (rec) rec->push_back({t, x, mode});
    }
    return x;
}

// Half-wave antisymmetry: the steady state satisfies x(T/2) = −x(0).
State residual(const LccCircuit& c, const State& x0) {
    const State x1 = half_period(c, x0, nullptr);
    State r;
    for (int k = 0; k < 4; ++k) r[k] = x1[k] + x0[k];
    return r;
}

bool solve4(std::array<std::array<double, 4>, 4> A, State b, State& out) {
    for (int col = 0; col < 4; ++col) {
        int p = col;
        for (int r = col + 1; r < 4; ++r) if (std::fabs(A[r][col]) > std::fabs(A[p][col])) p = r;
        if (std::fabs(A[p][col]) < 1e-300) return false;
        std::swap(A[p], A[col]); std::swap(b[p], b[col]);
        for (int r = col + 1; r < 4; ++r) {
            const double f = A[r][col] / A[col][col];
            for (int k = col; k < 4; ++k) A[r][k] -= f * A[col][k];
            b[r] -= f * b[col];
        }
    }
    for (int r = 3; r >= 0; --r) {
        double s = b[r];
        for (int k = r + 1; k < 4; ++k) s -= A[r][k] * out[k];
        out[r] = s / A[r][r];
    }
    return true;
}

// Steady-state initial condition: settle by repeated half periods (the rectifier is the only damping, so the
// tank rings down slowly), then Newton on the antisymmetry residual with a finite-difference Jacobian.
State periodic_state(const LccCircuit& c, State x, int settle) {
    const double iScale = c.busVoltage / std::sqrt(c.seriesInductance / c.seriesCapacitance);
    const State scale{iScale, c.busVoltage, c.busVoltage, iScale};
    auto norm = [&](const State& r) { double m = 0; for (int k = 0; k < 4; ++k) m = std::max(m, std::fabs(r[k]) / scale[k]); return m; };
    for (int k = 0; k < settle; ++k) {
        State y = half_period(c, x, nullptr);
        for (double& v : y) v = -v;
        x = y;
    }
    State r = residual(c, x);
    for (int it = 0; it < 40 && norm(r) > 1e-10; ++it) {
        std::array<std::array<double, 4>, 4> J{};
        for (int j = 0; j < 4; ++j) {
            State xp = x;
            const double dx = 1e-7 * scale[j];
            xp[j] += dx;
            const State rp = residual(c, xp);
            for (int i = 0; i < 4; ++i) J[i][j] = (rp[i] - r[i]) / dx;
        }
        State step{};
        State neg; for (int k = 0; k < 4; ++k) neg[k] = -r[k];
        if (!solve4(J, neg, step)) break;
        double lambda = 1.0;
        for (int ls = 0; ls < 20; ++ls) {
            State xt; for (int k = 0; k < 4; ++k) xt[k] = x[k] + lambda * step[k];
            const State rt = residual(c, xt);
            if (norm(rt) < norm(r)) { x = xt; r = rt; break; }
            lambda *= 0.5;
        }
        if (lambda < 1e-5) break;
    }
    if (norm(r) > 1e-7)
        throw std::runtime_error("solve_lcc: the periodic steady state did not converge (residual " +
                                 std::to_string(norm(r)) + " of the tank scale)");
    return x;
}

} // namespace

LccSteadyState solve_lcc(const LccCircuit& c, const std::array<double, 4>* guess) {
    if (!(c.busVoltage > 0) || !(c.clampVoltage > 0) || !(c.seriesInductance > 0) || !(c.seriesCapacitance > 0) ||
        !(c.parallelCapacitance > 0) || !(c.magnetizingInductance > 0) || !(c.switchingFrequency > 0))
        throw std::invalid_argument("solve_lcc: every element, Vb, Vc and fs must be > 0");
    State x0;
    try {
        x0 = guess ? periodic_state(c, *guess, 0) : periodic_state(c, State{0, 0, 0, 0}, 300);
    } catch (const std::runtime_error&) {
        if (!guess) throw;
        x0 = periodic_state(c, State{0, 0, 0, 0}, 300);   // the warm start was too far: settle from rest
    }
    std::vector<Sample> rec;
    half_period(c, x0, &rec);
    LccSteadyState s;
    s.period = 1.0 / c.switchingFrequency;
    const double Th = 0.5 * s.period;
    for (int half = 0; half < 2; ++half) {
        const double sign = half ? -1.0 : 1.0;
        for (size_t k = (half ? 1 : 0); k < rec.size(); ++k) {
            const Sample& p = rec[k];
            s.time.push_back(p.t + half * Th);
            s.bridgeVoltage.push_back(sign * c.busVoltage);
            s.tankCurrent.push_back(sign * p.x[0]);
            s.seriesCapacitorVoltage.push_back(sign * p.x[1]);
            s.parallelCapacitorVoltage.push_back(sign * p.x[2]);
            s.magnetizingCurrent.push_back(sign * p.x[3]);
            s.rectifierCurrent.push_back(p.mode == kOpen ? 0.0 : sign * (p.x[0] - p.x[3]));
        }
    }
    double acc = 0;
    for (size_t k = 1; k < s.time.size(); ++k)
        acc += 0.5 * (std::fabs(s.rectifierCurrent[k]) + std::fabs(s.rectifierCurrent[k - 1])) * (s.time[k] - s.time[k - 1]);
    s.rectifierCurrentMean = acc / s.period;
    s.tankCurrentAtTurnOff = rec.back().x[0];
    s.initialState = x0;
    return s;
}

LccLoadedState solve_lcc_loaded(LccCircuit c, double R) {
    if (!(R > 0)) throw std::invalid_argument("solve_lcc_loaded: the load resistance must be > 0");
    // f(Vc) = R·mean|irect|(Vc) − Vc falls monotonically: positive near 0, negative once Vc is above the voltage the
    // tank can ring Cp up to (no conduction at all). Bracket, then Illinois regula falsi.
    std::optional<std::array<double, 4>> warm;
    auto f = [&](double vc, LccSteadyState* out) {
        c.clampVoltage = vc;
        LccSteadyState s = solve_lcc(c, warm ? &*warm : nullptr);
        warm = s.initialState;
        const double v = R * s.rectifierCurrentMean - vc;
        if (out) *out = std::move(s);
        return v;
    };
    double lo = 1e-3 * c.busVoltage, hi = c.busVoltage;
    double flo = f(lo, nullptr), fhi = f(hi, nullptr);
    if (!(flo > 0)) throw std::runtime_error("solve_lcc_loaded: no rectifier current even at a near-zero output");
    for (int k = 0; k < 30 && fhi > 0; ++k) { lo = hi; flo = fhi; hi *= 2.0; fhi = f(hi, nullptr); }
    if (fhi > 0) throw std::runtime_error("solve_lcc_loaded: the output is not bounded (the tank gain does not fall)");
    int side = 0;
    double vc = lo;
    for (int it = 0; it < 80; ++it) {
        vc = (lo * fhi - hi * flo) / (fhi - flo);
        const double fv = f(vc, nullptr);
        if (std::fabs(fv) < 1e-9 * vc || (hi - lo) < 1e-12 * hi) break;
        if (fv > 0) { lo = vc; flo = fv; if (side == 1) fhi *= 0.5; side = 1; }
        else { hi = vc; fhi = fv; if (side == -1) flo *= 0.5; side = -1; }
    }
    LccLoadedState out;
    out.clampVoltage = vc;
    f(vc, &out.state);
    return out;
}

} // namespace analytical

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kQualityFactor = 1.0;
constexpr double kCapacitanceRatio = 1.0;
constexpr double kResonanceRatio = 1.6;
constexpr double kInductanceRatio = 10.0;
constexpr double kDeadTime = 0.02;
constexpr double kOutputRipple = 0.01;
constexpr double kNodeCapFraction = 0.005;   // each rectifier-node numerical cap, as a fraction of Cp (secondary-referred)

// The tank normalised to a unit clamp-side load (R_eff' = 1 Ω) and a unit bus: every impedance scales with
// R_eff', so the gain M = Vc/Vb at a frequency is a pure number of (Q, Cp/Cs, fs/fr, Lm/Lr).
struct Normalised { double Lr, Cs, Cp, Lm; };
Normalised normalised_tank(double Q, double A, double lmRatio, double fr) {
    const double Rac = 8.0 / (M_PI * M_PI), Zr = Q * Rac, w0 = 2.0 * M_PI * fr;
    return {Zr / w0, 1.0 / (w0 * Zr), A / (w0 * Zr), lmRatio * Zr / w0};
}
analytical::LccLoadedState unit_gain(const Normalised& t, double f) {
    analytical::LccCircuit c{1.0, 0.0, t.Lr, t.Cs, t.Cp, t.Lm, f};
    return analytical::solve_lcc_loaded(c, 1.0);
}

// The frequency at which the normalised tank delivers gain M, searched on the falling (inductive) side of the
// gain peak: step down from `fStart` while the gain rises; bracket M; regula falsi. Throws if the peak is
// below M (the input range asks for more gain than the tank has).
double frequency_for_gain(const Normalised& t, double M, double fStart, double fFloor, const char* who) {
    double fHi = fStart, mHi = unit_gain(t, fHi).clampVoltage;
    if (mHi > M) {   // need a higher frequency: step up until the gain falls below M
        double fLo = fHi, mLo = mHi;
        for (int k = 0; k < 60 && mHi > M; ++k) { fLo = fHi; mLo = mHi; fHi *= 1.05; mHi = unit_gain(t, fHi).clampVoltage; }
        if (mHi > M) throw std::invalid_argument(std::string(who) + "the gain stays above " + std::to_string(M) + " up to " + std::to_string(fHi) + " Hz");
        for (int it = 0; it < 60; ++it) {
            const double f = fLo + (fHi - fLo) * (mLo - M) / (mLo - mHi);
            const double m = unit_gain(t, f).clampVoltage;
            if (std::fabs(m - M) < 1e-7 * M) return f;
            if (m > M) { fLo = f; mLo = m; } else { fHi = f; mHi = m; }
        }
        return 0.5 * (fLo + fHi);
    }
    double fLo = fHi, mLo = mHi;
    while (mLo < M) {
        const double fn = fLo / 1.03;
        if (fn < fFloor)
            throw std::invalid_argument(std::string(who) + "the tank cannot deliver gain " + std::to_string(M) +
                                        " above its series resonance " + std::to_string(fFloor) + " Hz (it peaks at " +
                                        std::to_string(mLo) + "); raise qualityFactor's gain margin, lower resonanceRatio "
                                        "or narrow the input range");
        const double mn = unit_gain(t, fn).clampVoltage;
        if (mn <= mLo)
            throw std::invalid_argument(std::string(who) + "the tank's gain peaks at " + std::to_string(mLo) + " (" +
                                        std::to_string(fLo) + " Hz), below the " + std::to_string(M) +
                                        " the minimum input needs; lower resonanceRatio or narrow the input range");
        fHi = fLo; mHi = mLo; fLo = fn; mLo = mn;
    }
    for (int it = 0; it < 60; ++it) {
        const double f = fLo + (fHi - fLo) * (mLo - M) / (mLo - mHi);
        const double m = unit_gain(t, f).clampVoltage;
        if (std::fabs(m - M) < 1e-7 * M) return f;
        if (m > M) { fLo = f; mLo = m; } else { fHi = f; mHi = m; }
    }
    return 0.5 * (fLo + fHi);
}
} // namespace

LccDesign design_lcc(const json& tasInputs) {
    const char* who = "design_lcc: ";
    const json& dr = tasInputs.at("designRequirements");
    LccDesign d{};
    d.config = cfg::object_of(tasInputs);
    if (dr.at("outputs").size() != 1)
        throw std::invalid_argument(std::string(who) + "one output only");
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    if (!(d.outputVoltage > 0)) throw std::invalid_argument(std::string(who) + "the output voltage must be positive");
    const double fSpec = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 1.0);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }
    const json& iv = dr.at("inputVoltage");
    d.inputVoltageMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    for (const char* k : {"desiredResonantInductance", "desiredResonantCapacitance", "magnetizingInductance"})
        if (dr.contains(k))
            throw std::invalid_argument(std::string(who) + "a pinned " + k + " is not supported: the LCC tank is sized "
                                        "as a whole from qualityFactor / capacitanceRatio / resonanceRatio");

    const std::string side = cfg::get_str(d.config, "parallelCapacitorSide", "secondary");
    if (side != "secondary" && side != "primary")
        throw std::invalid_argument(std::string(who) + "parallelCapacitorSide must be 'secondary' or 'primary', got '" + side + "'");
    d.parallelOnSecondary = (side == "secondary");
    d.fullBridge = cfg::full_bridge_selected(d.config);
    d.deadTimeFraction = cfg::get(d.config, "deadTimeFraction", kDeadTime);
    if (!(d.deadTimeFraction > 0 && d.deadTimeFraction < 0.1))
        throw std::invalid_argument(std::string(who) + "deadTimeFraction must be in (0, 0.1)");
    const double Q = cfg::get(d.config, "qualityFactor", kQualityFactor);
    const double A = cfg::get(d.config, "capacitanceRatio", kCapacitanceRatio);
    const double ratio = cfg::get(d.config, "resonanceRatio", kResonanceRatio);
    const double lmRatio = cfg::get(d.config, "inductanceRatio", kInductanceRatio);
    if (!(Q > 0) || !(A > 0) || !(ratio > 1.0) || !(lmRatio > 0))
        throw std::invalid_argument(std::string(who) + "qualityFactor, capacitanceRatio and inductanceRatio must be > 0 and "
                                    "resonanceRatio > 1 (above the series resonance)");

    const double Vo = d.outputVoltage, Io = d.outputPower / Vo, k = d.fullBridge ? 1.0 : 0.5;
    d.diodeDrop = req::rectifier_drop(d.config, Io);
    const double voPlusVd = Vo + 2.0 * d.diodeDrop;   // full-bridge rectifier: two diodes in the path
    d.loadResistance = Vo * Vo / d.outputPower;
    const double fr = fSpec / ratio;
    const Normalised t = normalised_tank(Q, A, lmRatio, fr);

    // Turns ratio from the gain at the spec frequency, or — pinned — the frequency that gives its gain.
    const std::optional<double> pinnedN = req::provided_turns_ratio(dr, 0);
    const double gainAtSpec = unit_gain(t, fSpec).clampVoltage;
    if (pinnedN) {
        if (!(*pinnedN > 0)) throw std::invalid_argument(std::string(who) + "a pinned turns ratio must be > 0");
        d.turnsRatio = *pinnedN;
        d.switchingFrequency = frequency_for_gain(t, d.turnsRatio * voPlusVd / (k * d.inputVoltage), fSpec, fr, who);
    } else {
        d.turnsRatio = gainAtSpec * k * d.inputVoltage / voPlusVd;
        d.switchingFrequency = fSpec;
    }
    const double n = d.turnsRatio;
    // Scale to the real load: the clamp sees R_eff' = n²·R·(Vo + Vd_path)/Vo (the diode drop as part of the load).
    const double Reff = n * n * d.loadResistance * voPlusVd / Vo;
    d.seriesInductance = t.Lr * Reff;
    d.seriesCapacitance = t.Cs / Reff;
    d.magnetizingInductance = t.Lm * Reff;
    const double cpPrimary = t.Cp / Reff;
    d.parallelCapacitance = d.parallelOnSecondary ? cpPrimary * n * n : cpPrimary;
    // The rectifier's junction capacitance is part of the tank (ABT #1678): a real 2 kV bridge's tens of pF are tens
    // of percent of a 157 pF secondary Cp. Fold it out of the discrete part, or refuse a rectifier that alone
    // exceeds the tank's Cp.
    {
        double cj = 0;
        if (d.config.is_object() && d.config.contains("rectifierJunctionCapacitance")) {
            const json& v = d.config.at("rectifierJunctionCapacitance");
            if (!v.is_number() || !(v.get<double>() >= 0))
                throw std::invalid_argument(std::string(who) + "config.rectifierJunctionCapacitance must be a number >= 0 (F, per diode)");
            cj = v.get<double>();
        }
        // Full bridge: two series pairs in parallel across the winding -> Cj; referred to the primary by 1/n².
        d.rectifierCapacitance = d.parallelOnSecondary ? cj : cj / (n * n);
        if (!(d.rectifierCapacitance < d.parallelCapacitance))
            throw std::invalid_argument(std::string(who) + "the rectifier's junction capacitance (" +
                std::to_string(d.rectifierCapacitance * 1e12) + " pF on Cp's side) alone reaches the tank's Cp (" +
                std::to_string(d.parallelCapacitance * 1e12) + " pF): pick lower-capacitance diodes or a larger capacitanceRatio");
        d.discreteParallelCapacitance = d.parallelCapacitance - d.rectifierCapacitance;
    }
    d.seriesResonantFrequency = fr;

    // Frequency control across the input range at full load, and ZVS at each corner.
    d.frequencyAtMinimumInput = frequency_for_gain(t, n * voPlusVd / (k * d.inputVoltageMin), d.switchingFrequency, fr, who);
    d.frequencyAtMaximumInput = frequency_for_gain(t, n * voPlusVd / (k * d.inputVoltageMax), d.switchingFrequency, fr, who);
    for (double f : {d.frequencyAtMinimumInput, d.switchingFrequency, d.frequencyAtMaximumInput}) {
        const auto s = unit_gain(t, f);
        if (!(s.state.tankCurrentAtTurnOff > 0))
            throw std::invalid_argument(std::string(who) + "at " + std::to_string(f) + " Hz the tank current at turn-off is " +
                                        std::to_string(s.state.tankCurrentAtTurnOff) + " (capacitive): the bridge would hard-switch. "
                                        "Raise resonanceRatio");
    }
    if (!d.fullBridge) {
        // Split capacitors: the bus midpoint carries the tank current; size them stiff against it (2 % ripple).
        const auto s = unit_gain(t, d.frequencyAtMinimumInput);
        double qMax = 0, q = 0;
        for (size_t i = 1; i < s.state.time.size(); ++i) {
            q += 0.5 * (s.state.tankCurrent[i] + s.state.tankCurrent[i - 1]) * (s.state.time[i] - s.state.time[i - 1]);
            qMax = std::max(qMax, std::fabs(q));
        }
        // normalised charge (unit bus, unit load) → real: current scales Vb/Reff, time 1/f is already real
        const double qReal = qMax * (k * d.inputVoltageMin) / Reff;
        d.splitCapacitance = qReal / (2.0 * 0.02 * 0.5 * d.inputVoltageMin);
        d.balancingResistance = cfg::bias_res(d.config, 0.5 * d.inputVoltageMax, d.outputPower);
    }
    const double ripple = cfg::get(d.config, "outputVoltageRipple", kOutputRipple);
    if (!(ripple > 0)) throw std::invalid_argument(std::string(who) + "outputVoltageRipple must be > 0");
    // The rectifier delivers two pulses per period; between them Cout alone carries Io for up to half a period.
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", Io / (2.0 * d.frequencyAtMaximumInput * ripple * Vo));
    return d;
}

json build_lcc_tas(const LccDesign& d) {
    auto port = [](const char* n){ json p; p["name"]=n; return p; };
    auto pin  = [](const char* c, const char* p){ json e; e["component"]=c; e["pin"]=p; return e; };
    auto prt  = [](const char* p){ json e; e["port"]=p; return e; };
    auto conn = [](const char* name, std::vector<json> eps){ json c; c["name"]=name; c["endpoints"]=eps; return c; };
    auto comp = [](const char* name, json data){ json c; c["name"]=name; c["data"]=data; return c; };
    auto bind = [](const char* p, const char* t){ json b; b["port"]=p; b["type"]=t; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb){
        json s; s["name"]=name; s["role"]=role; s["circuit"]=brick; s["inputPort"]=inb; s["outputPort"]=outb; return s; };
    auto sp = [](const char* st, const char* po){ json e; e["stage"]=st; e["port"]=po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps){
        json c; c["name"]=name; c["kind"]=kind; if(dir[0]) c["direction"]=dir; c["endpoints"]=eps; return c; };
    using Lbl = MAS::WaveformLabel;
    using WP = OpenMagnetics::WaveformProcessor;
    namespace AN = Kirchhoff::analytical;

    const double n = d.turnsRatio, f = d.switchingFrequency, T = 1.0 / f;
    const double Vo = d.outputVoltage, k = d.fullBridge ? 1.0 : 0.5;
    const double voPlusVd = Vo + 2.0 * d.diodeDrop;
    const double cpPrimary = d.parallelOnSecondary ? d.parallelCapacitance / (n * n) : d.parallelCapacitance;
    auto circuit = [&](double vin, double fs) {
        return AN::LccCircuit{k * vin, n * voPlusVd, d.seriesInductance, d.seriesCapacitance, cpPrimary,
                              d.magnetizingInductance, fs};
    };
    // The operating point (embedded) and the Vin_min corner (lowest frequency, largest tank current: ratings).
    const AN::LccSteadyState ss = AN::solve_lcc(circuit(d.inputVoltage, f));
    const AN::LccSteadyState worst = AN::solve_lcc(circuit(d.inputVoltageMin, d.frequencyAtMinimumInput));
    auto peak = [](const std::vector<double>& v) { double m = 0; for (double x : v) m = std::max(m, std::fabs(x)); return m; };
    auto rms = [](const std::vector<double>& t, const std::vector<double>& v) {
        double a = 0; for (size_t i = 1; i < t.size(); ++i) a += 0.5 * (v[i] * v[i] + v[i - 1] * v[i - 1]) * (t[i] - t[i - 1]);
        return std::sqrt(a / (t.back() - t.front())); };
    const double iTankPk = peak(worst.tankCurrent), iTankRms = rms(worst.time, worst.tankCurrent);
    cfg::check_maximum_switch_current(d.config, iTankPk, "build_lcc_tas");
    cfg::check_maximum_drain_source_voltage(d.config, d.inputVoltageMax, "build_lcc_tas");

    auto wave = [&](const std::vector<double>& data) {
        MAS::Waveform w; w.set_ancillary_label(Lbl::CUSTOM); w.set_data(data); w.set_time(ss.time); return w; };
    // Lr: the tank current, and the voltage across it (bridge − Cs − Cp).
    std::vector<double> vLr(ss.time.size());
    for (size_t i = 0; i < ss.time.size(); ++i) vLr[i] = ss.bridgeVoltage[i] - ss.seriesCapacitorVoltage[i] - ss.parallelCapacitorVoltage[i];
    MAS::OperatingPoint opLr;
    opLr.get_mutable_excitations_per_winding().push_back(WP::complete_excitation(wave(ss.tankCurrent), wave(vLr), f, "Primary"));
    json lr; lr["magnetic"] = json::object();
    lr["inputs"] = req::magnetic_inputs(d.seriesInductance, 0.05, {}, {"primary"}, std::nullopt, 25.0,
                                        AN::excitations_processed(opLr, "Lr"));
    // T1: with Cp on the secondary the whole tank current enters the primary; with Cp on the primary only the
    // magnetizing and rectifier currents do. The secondary carries n·(primary − magnetizing).
    std::vector<double> iPri(ss.time.size()), iSec(ss.time.size()), vSec(ss.time.size());
    for (size_t i = 0; i < ss.time.size(); ++i) {
        iPri[i] = d.parallelOnSecondary ? ss.tankCurrent[i] : ss.magnetizingCurrent[i] + ss.rectifierCurrent[i];
        iSec[i] = n * (iPri[i] - ss.magnetizingCurrent[i]);
        vSec[i] = ss.parallelCapacitorVoltage[i] / n;
    }
    MAS::OperatingPoint opT1;
    opT1.get_mutable_excitations_per_winding().push_back(WP::complete_excitation(wave(iPri), wave(ss.parallelCapacitorVoltage), f, "Primary"));
    opT1.get_mutable_excitations_per_winding().push_back(WP::complete_excitation(wave(iSec), wave(vSec), f, "Secondary 0"));
    json t1; t1["magnetic"] = json::object();
    // Lm is part of the tank (it shunts Cp), so it is a nominal value with a tolerance, not a floor.
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, {n}, {"primary", "secondary"}, std::nullopt, 25.0,
                                        AN::excitations_processed(opT1, "T1"));

    auto cap = [](double c, double v, const char* role) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = v;
        if (role[0]) j["inputs"]["designRequirements"]["role"] = role;
        return j; };
    // Resonant caps (role resonant: sourced near nominal), rated from the Vin_min corner's peaks.
    const json cs = cap(d.seriesCapacitance, peak(worst.seriesCapacitorVoltage) / cfg::v_derate_capacitor(d.config), "resonant");
    const double vCpPeak = peak(worst.parallelCapacitorVoltage) / (d.parallelOnSecondary ? n : 1.0);
    const json cp = cap(d.discreteParallelCapacitance, vCpPeak / cfg::v_derate_capacitor(d.config), "resonant");

    const double ratedVds = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    json mosfetReq; mosfetReq["semiconductor"]["mosfet"] = json::object();
    mosfetReq["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, iTankPk,
        cfg::rds_on_loss_fraction(d.config) * d.outputPower / (iTankRms * iTankRms), 125.0);
    auto bodyDiode = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltageMax, iTankPk); return j; };
    // Rectifier: each diode blocks the clamp (Vo) and carries the rectified current pulses.
    const double vSecPk = Vo + 2.0 * d.diodeDrop;
    const double ratedVr = vSecPk / cfg::v_derate_diode(d.config);
    json diodeReq; diodeReq["semiconductor"]["diode"] = json::object();
    diodeReq["inputs"]["designRequirements"] = req::diode(ratedVr, n * peak(worst.rectifierCurrent),
                                                          (ratedVr < 100.0) ? 0.6 : 1.2, 0.05 * T);
    // Numerical aids (ABT #96): a small cap on the bridge node(s) and on each rectifier node.
    const double cpSecondary = d.parallelOnSecondary ? d.parallelCapacitance : d.parallelCapacitance * n * n;
    auto aid = [&](double c, double v) { json j = cap(c, v, ""); cfg::mark_numerical_aid(j); return j; };

    std::vector<json> comps, conns, gndEps, sgndEps;
    std::vector<json> cports{port("vin"), port("gnd"), port("sgnd"), port("vout")};
    if (!d.fullBridge) {
        json rb; rb["resistor"] = json::object();
        rb["inputs"]["designRequirements"] = req::resistor(d.balancingResistance,
            2.0 * std::pow(0.5 * d.inputVoltageMax, 2) / d.balancingResistance, "bleed");
        const json csplit = cap(d.splitCapacitance, d.inputVoltageMax / cfg::v_derate_capacitor(d.config), "");
        comps = {comp("CsHi", csplit), comp("CsLo", csplit), comp("RbHi", rb), comp("RbLo", rb),
                 comp("Q1", mosfetReq), comp("Q2", mosfetReq), comp("Dq1", bodyDiode()), comp("Dq2", bodyDiode()),
                 comp("Csw1", aid(100e-12, 2.0 * d.inputVoltageMax))};
        conns = {
            conn("vin_net", {pin("CsHi","1"), pin("RbHi","1"), pin("Q1","drain"), pin("Dq1","cathode"), prt("vin")}),
            conn("sw", {pin("Q1","source"), pin("Q2","drain"), pin("Dq1","anode"), pin("Dq2","cathode"),
                        pin("Lr","primary_start"), pin("Csw1","1")}),
            conn("g1_net", {pin("Q1","gate"), prt("g1")}),
            conn("g2_net", {pin("Q2","gate"), prt("g2")})};
        gndEps = {pin("CsLo","2"), pin("RbLo","2"), pin("Q2","source"), pin("Dq2","anode"), pin("Csw1","2")};
        cports.push_back(port("g1")); cports.push_back(port("g2"));
    } else {
        comps = {comp("Q1", mosfetReq), comp("Q2", mosfetReq), comp("Q3", mosfetReq), comp("Q4", mosfetReq),
                 comp("Dq1", bodyDiode()), comp("Dq2", bodyDiode()), comp("Dq3", bodyDiode()), comp("Dq4", bodyDiode()),
                 comp("Csw1", aid(100e-12, 2.0 * d.inputVoltageMax)), comp("Csw2", aid(100e-12, 2.0 * d.inputVoltageMax))};
        conns = {
            conn("vin_net", {pin("Q1","drain"), pin("Dq1","cathode"), pin("Q3","drain"), pin("Dq3","cathode"), prt("vin")}),
            conn("leg_a", {pin("Q1","source"), pin("Q2","drain"), pin("Dq1","anode"), pin("Dq2","cathode"),
                           pin("Lr","primary_start"), pin("Csw1","1")}),
            conn("g1_net", {pin("Q1","gate"), prt("g1")}), conn("g2_net", {pin("Q2","gate"), prt("g2")}),
            conn("g3_net", {pin("Q3","gate"), prt("g3")}), conn("g4_net", {pin("Q4","gate"), prt("g4")})};
        gndEps = {pin("Q2","source"), pin("Dq2","anode"), pin("Q4","source"), pin("Dq4","anode"), pin("Csw1","2"), pin("Csw2","2")};
        for (const char* g : {"g1", "g2", "g3", "g4"}) cports.push_back(port(g));
    }
    // Tank: Lr → Cs → the primary; Cp across the primary or the secondary.
    comps.insert(comps.end(), {comp("Lr", lr), comp("Cs", cs), comp("T1", t1), comp("Cp", cp),
                               comp("Dr1", diodeReq), comp("Dr2", diodeReq), comp("Dr3", diodeReq), comp("Dr4", diodeReq),
                               comp("CsnSA", aid(kNodeCapFraction * cpSecondary, 3.0 * Vo)),
                               comp("CsnSB", aid(kNodeCapFraction * cpSecondary, 3.0 * Vo)),
                               comp("Rcm", [&] { json r; r["resistor"] = json::object();
                                   r["inputs"]["designRequirements"] = req::resistor(cfg::get(d.config, "commonModeBleed", 1e8),
                                       Vo * Vo / cfg::get(d.config, "commonModeBleed", 1e8), "bleed");
                                   cfg::mark_numerical_aid(r); return r; }())});
    conns.push_back(conn("tank", {pin("Lr","primary_end"), pin("Cs","1")}));
    std::vector<json> priTop{pin("Cs","2"), pin("T1","primary_start")};
    std::vector<json> priBot{pin("T1","primary_end")};
    // The rectifier nodes' numerical caps are a fixed small FRACTION of Cp (referred to the secondary), not the 100 pF
    // rectifier constant: in a step-up LCC that constant is not small — referred to the primary it added ~30 % to Cp
    // and cut the output by a fifth (1625 V of 2000 V). Without any, ngspice finds the near-unity-coupled secondary
    // singular whenever every diode is off.
    std::vector<json> secA{pin("T1","secondary1_start"), pin("Dr1","anode"), pin("Dr3","cathode"), pin("CsnSA","1")};
    // The secondary's common mode still needs a DC reference while every diode is off: a bleed from sec_b to the
    // return (a numerical aid; 100 MΩ dissipates ~0.01 % of the output at a 2 kV rail).
    std::vector<json> secB{pin("T1","secondary1_end"), pin("Dr2","anode"), pin("Dr4","cathode"), pin("CsnSB","1"), pin("Rcm","1")};
    if (d.parallelOnSecondary) { secA.push_back(pin("Cp","1")); secB.push_back(pin("Cp","2")); }
    else { priTop.push_back(pin("Cp","1")); priBot.push_back(pin("Cp","2")); }
    conns.push_back(conn("pri_top", priTop));
    conns.push_back(conn("sec_a", secA));
    conns.push_back(conn("sec_b", secB));
    if (!d.fullBridge) {
        priBot.insert(priBot.end(), {pin("CsHi","2"), pin("CsLo","1"), pin("RbHi","2"), pin("RbLo","1")});
        conns.push_back(conn("mid", priBot));
    } else {
        priBot.insert(priBot.end(), {pin("Q3","source"), pin("Q4","drain"), pin("Dq3","anode"), pin("Dq4","cathode"), pin("Csw2","1")});
        conns.push_back(conn("leg_b", priBot));
    }
    conns.push_back(conn("vout_net", {pin("Dr1","cathode"), pin("Dr2","cathode"), prt("vout")}));
    sgndEps = {pin("Dr3","anode"), pin("Dr4","anode"), pin("CsnSA","2"), pin("CsnSB","2"), pin("Rcm","2"), prt("sgnd")};
    gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));
    conns.push_back(conn("sgnd_net", sgndEps));

    json cell; cell["name"] = "lcc-cell";
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;
    json filt; filt["name"] = "output-filter";
    filt["ports"] = json::array({port("in"), port("rtn")});
    filt["components"] = json::array({comp("Cout", cap(d.outputCapacitance, 2.0 * Vo, ""))});
    filt["connections"] = json::array({conn("out", {pin("Cout","1"), prt("in")}), conn("ret", {pin("Cout","2"), prt("rtn")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency; dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = f;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = Vo; o["regulation"] = "voltage"; dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }
    tas["topology"]["stages"] = json::array({
        req::control_stage("llcController"),
        pstage("lccCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "pulsatingDc")),
        pstage("filter", "outputFilter", filt, bind("in", "pulsatingDc"), bind("in", "dcOutput"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("Vin", "externalPort", "input", {sp("lccCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("lccCell", "gnd")}),
        isc("SGND", "externalPort", "input", {sp("lccCell", "sgnd"), sp("filter", "rtn")}),
        isc("Vout", "externalPort", "output", {sp("lccCell", "vout"), sp("filter", "in")})});
    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // 50 % complementary drive less the dead time; the (+) pair at 0°, the (−) pair at 180°.
    const double duty = 0.5 - d.deadTimeFraction;
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "lccCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = f;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg; return st; };
    if (d.fullBridge)
        tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q4", 0.0), stim("Q3", 180.0), stim("Q2", 180.0)});
    else
        tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    req::finalize_control_seeds(tas, Topology::LCC_RESONANT_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
