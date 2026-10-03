#include "ActiveClampFlyback.hpp"
#include "processors/WaveformProcessor.h"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "KirchhoffConfig.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace analytical {
namespace {

// What holds the switch node. Q1: GND (Q1's channel, or Dq1 carrying a negative primary current). QC: the clamp,
// Vin + Vc, so the primary sees −Vc (Qc's channel, or Dqc carrying a positive current). FLOAT: neither — the
// node rides on the switch-node capacitance Csw, which the primary current charges or discharges; this is the
// dead-time transition (the ZVS swing) and, with the transformer idle, the Lp–Csw valley ring.
enum class Pri { Q1, QC, FLOAT };

// Primary (Lk) current, magnetizing current, clamp voltage, switch-node voltage, rectifier conducting.
struct State { double ik, im, vc, vsw; bool sec; };

struct Cell {
    double vin, vr, a, lm, lk, cc, csw, T, dt, tp;
    bool comp;
    double iTol, vTol;
};

struct Topo {
    Pri pri; bool sec;
    bool operator==(const Topo& o) const { return pri == o.pri && sec == o.sec; }
    bool operator!=(const Topo& o) const { return !(*this == o); }
};

// Voltage across the primary winding terminals (Lk + Lm), dot end positive.
double series_voltage(Pri p, const State& s, const Cell& c) {
    if (p == Pri::Q1) return c.vin;
    if (p == Pri::QC) return -s.vc;
    return c.vin - s.vsw;
}

Topo topology(const State& s, bool g1, bool gc, const Cell& c) {
    Pri p;
    if (g1) p = Pri::Q1;
    else if (gc) p = Pri::QC;
    else if (s.vsw <= c.vTol && s.ik < -c.iTol) p = Pri::Q1;                        // Dq1
    else if (s.vsw >= c.vin + s.vc - c.vTol && s.ik > c.iTol) p = Pri::QC;           // Dqc
    else p = Pri::FLOAT;
    bool sec;
    if (s.sec) {
        sec = (s.im - s.ik) > -c.iTol;                  // conducts until the rectifier current reverses
    } else {
        const double vLm = series_voltage(p, s, c) * c.lm / (c.lm + c.lk);
        sec = vLm < -c.vr * (1.0 + 1e-12);              // the magnetizing voltage reaches the reflected output
    }
    return {p, sec};
}

// Exact state after h seconds in a fixed topology.
State advance(const State& s, Topo t, double h, const Cell& c) {
    State o = s; o.sec = t.sec;
    // (i, v) of an L–C pair resonating about v = vCentre:  dv/dt = i/C,  di/dt = −(v − vCentre)/L.
    auto rotate = [&](double L, double C, double vCentre, double& i, double& v) {
        const double w = 1.0 / std::sqrt(L * C), Z = std::sqrt(L / C);
        const double u0 = v - vCentre, i0 = i;
        const double cs = std::cos(w * h), sn = std::sin(w * h);
        i = i0 * cs - (u0 / Z) * sn;
        v = vCentre + u0 * cs + Z * i0 * sn;
    };
    switch (t.pri) {
    case Pri::Q1:
        if (t.sec) { o.ik = s.ik + (c.vin + c.vr) / c.lk * h; o.im = s.im - c.vr / c.lm * h; }
        else       { o.ik = s.ik + c.vin / (c.lk + c.lm) * h; o.im = o.ik; }
        o.vsw = 0.0;
        break;
    case Pri::QC:
        // The clamp: Lk (rectifier on) or Lk + Lm (off) against Cc, about Vr (on) or 0 (off).
        if (t.sec) { rotate(c.lk, c.cc, c.vr, o.ik, o.vc); o.im = s.im - c.vr / c.lm * h; }
        else       { rotate(c.lk + c.lm, c.cc, 0.0, o.ik, o.vc); o.im = o.ik; }
        o.vsw = c.vin + o.vc;
        break;
    case Pri::FLOAT:
        // The node: dVsw/dt = ik/Csw; the winding sees Vin − Vsw. Lk against Csw about Vin + Vr (on), Lk + Lm
        // about Vin (off).
        if (t.sec) { rotate(c.lk, c.csw, c.vin + c.vr, o.ik, o.vsw); o.im = s.im - c.vr / c.lm * h; }
        else       { rotate(c.lk + c.lm, c.csw, c.vin, o.ik, o.vsw); o.im = o.ik; }
        break;
    }
    return o;
}

bool gate1(double t, double D, const Cell& c) { return t < D * c.T; }
bool gatec(double t, double D, const Cell& c) {
    if (c.comp) return t >= D * c.T + c.dt && t < c.T - c.dt;
    return t >= c.T - c.dt - c.tp && t < c.T - c.dt;
}

struct Sample { double t, ik, im, vc, vsw, vpri, isec, vsec; };

Sample sample_of(double t, const State& s, Topo tp, const Cell& c) {
    const double vser = series_voltage(tp.pri, s, c);
    const double vLm = tp.sec ? -c.vr : vser * c.lm / (c.lm + c.lk);
    return {t, s.ik, s.im, s.vc, s.vsw, vser, tp.sec ? c.a * (s.im - s.ik) : 0.0, vLm / c.a};
}

// One period from Q1's turn-on. Returns the end state; `charge` is the rectifier charge ∫ i_sec dt.
// With `out`, samples the period on a uniform grid of `nGrid` points (both ends included).
// With tEnd < T, stops there and returns the state at tEnd.
State run_period(State s, double D, const Cell& c, double& charge, std::vector<Sample>* out, int nGrid,
                 double tEnd = -1.0) {
    if (tEnd < 0.0) tEnd = c.T;
    std::vector<double> edges{0.0, D * c.T, c.T};
    if (c.comp) { edges.push_back(D * c.T + c.dt); edges.push_back(c.T - c.dt); }
    else        { edges.push_back(c.T - c.dt - c.tp); edges.push_back(c.T - c.dt); }
    if (out) for (int k = 0; k < nGrid; ++k) edges.push_back(c.T * k / (nGrid - 1));
    edges.push_back(tEnd);
    edges.erase(std::remove_if(edges.begin(), edges.end(), [&](double e) { return e > tEnd; }), edges.end());
    std::sort(edges.begin(), edges.end());
    const double hMax = c.T / 2000.0;
    charge = 0.0;
    std::size_t nextGrid = 0;
    auto record = [&](double t, const State& x) {
        if (!out) return;
        while (nextGrid < static_cast<std::size_t>(nGrid) && c.T * nextGrid / (nGrid - 1) <= t + 1e-15 * c.T) {
            const bool g1 = gate1(std::min(t, c.T * (1 - 1e-12)), D, c), gc = gatec(std::min(t, c.T * (1 - 1e-12)), D, c);
            out->push_back(sample_of(c.T * nextGrid / (nGrid - 1), x, topology(x, g1, gc, c), c));
            ++nextGrid;
        }
    };
    record(0.0, s);
    auto name = [](Topo x) { return std::string(x.pri == Pri::Q1 ? "Q1" : x.pri == Pri::QC ? "QC" : "FLOAT")
                                    + (x.sec ? "+sec" : ""); };
    for (std::size_t e = 0; e + 1 < edges.size(); ++e) {
        const double tA = edges[e], tB = edges[e + 1];
        if (!(tB > tA)) continue;
        const double tMid = 0.5 * (tA + tB);
        const bool g1 = gate1(tMid, D, c), gc = gatec(tMid, D, c);
        double t = tA;
        int events = 0;
        while (t < tB - 1e-18) {
            const Topo cur = topology(s, g1, gc, c);
            s.sec = cur.sec;
            if (!cur.sec) s.im = s.ik;
            // A switch that turns on into a node away from its rail hard-switches it: Q1 dumps Csw; Qc charges
            // Csw from Cc (charge conserved between the two: the node and the clamp settle together).
            if (cur.pri == Pri::Q1) s.vsw = 0.0;
            if (cur.pri == Pri::QC && std::abs(c.vin + s.vc - s.vsw) > c.vTol) {
                const double step = (c.vin + s.vc - s.vsw) / (1.0 + c.csw / c.cc);
                s.vc -= c.csw * step / c.cc;
                s.vsw = c.vin + s.vc;
            }
            const double h = std::min(hMax, tB - t);
            State end = advance(s, cur, h, c);
            double hUse = h;
            if (topology(end, g1, gc, c) != cur) {
                // First instant the topology changes: bisect on the exact in-topology solution.
                double lo = 0.0, hi = h;
                for (int it = 0; it < 80; ++it) {
                    const double mid = 0.5 * (lo + hi);
                    if (topology(advance(s, cur, mid, c), g1, gc, c) != cur) hi = mid; else lo = mid;
                }
                hUse = hi;
                end = advance(s, cur, hUse, c);
                // Land the event exactly: a rectifier that stops carries zero current; a node that reaches a rail
                // sits on it.
                const Topo nxt = topology(end, g1, gc, c);
                if (cur.sec && !nxt.sec) { const double m = 0.5 * (end.ik + end.im); end.ik = m; end.im = m; }
                if (nxt.pri == Pri::Q1) end.vsw = 0.0;
                if (nxt.pri == Pri::QC) end.vsw = c.vin + end.vc;
                end.sec = nxt.sec;   // advance() carried the old topology's rectifier state
                if (++events > 200)
                    throw std::runtime_error("solve_active_clamp_flyback: the primary keeps changing state inside one "
                        "interval (chattering at t = " + std::to_string(t / c.T) + " T, " + name(cur) + " -> " + name(nxt)
                        + ", ik " + std::to_string(end.ik) + " A, im " + std::to_string(end.im) + " A, vc "
                        + std::to_string(end.vc) + " V, vsw " + std::to_string(end.vsw) + " V)");
            }
            if (cur.sec) {
                // Exact rectifier charge a·∫(im − ik): im is linear in every rectifying topology; ik is linear (Q1),
                // or the charge of the capacitor it rotates against: ∫ik = Cc·ΔVc (QC), Csw·ΔVsw (FLOAT).
                const State raw = advance(s, cur, hUse, c);
                const double qIk = (cur.pri == Pri::QC)    ? c.cc * (raw.vc - s.vc)
                                 : (cur.pri == Pri::FLOAT) ? c.csw * (raw.vsw - s.vsw)
                                                           : 0.5 * (s.ik + raw.ik) * hUse;
                charge += c.a * (0.5 * (s.im + raw.im) * hUse - qIk);
            }
            s = end;
            t += hUse;
            record(t, s);
        }
    }
    return s;
}

// Dense 4×4 solve (partial pivoting).
std::array<double, 4> solve4(std::array<std::array<double, 4>, 4> A, std::array<double, 4> b) {
    for (int col = 0; col < 4; ++col) {
        int piv = col;
        for (int r = col + 1; r < 4; ++r) if (std::abs(A[r][col]) > std::abs(A[piv][col])) piv = r;
        if (std::abs(A[piv][col]) < 1e-300)
            throw std::runtime_error("solve_active_clamp_flyback: singular steady-state Jacobian");
        std::swap(A[piv], A[col]); std::swap(b[piv], b[col]);
        for (int r = col + 1; r < 4; ++r) {
            const double f = A[r][col] / A[col][col];
            for (int k = col; k < 4; ++k) A[r][k] -= f * A[col][k];
            b[r] -= f * b[col];
        }
    }
    std::array<double, 4> x{};
    for (int r = 3; r >= 0; --r) {
        double acc = b[r];
        for (int k = r + 1; k < 4; ++k) acc -= A[r][k] * x[k];
        x[r] = acc / A[r][r];
    }
    return x;
}

} // namespace

AcfFlybackSteadyState solve_active_clamp_flyback(const AcfFlybackCircuit& ci, double outputCurrent) {
    const char* who = "solve_active_clamp_flyback";
    for (double v : {ci.inputVoltage, ci.outputVoltage, ci.turnsRatio, ci.magnetizingInductance,
                     ci.leakageInductance, ci.clampCapacitance, ci.switchNodeCapacitance, ci.switchingFrequency,
                     outputCurrent})
        if (!(v > 0.0) || !std::isfinite(v))
            throw std::invalid_argument(std::string(who) + ": every circuit value and the output current must be > 0");
    if (!(ci.diodeDrop >= 0.0)) throw std::invalid_argument(std::string(who) + ": diodeDrop must be >= 0");
    if (!(ci.deadTime >= 0.0)) throw std::invalid_argument(std::string(who) + ": deadTime must be >= 0");
    if (!ci.complementary && !(ci.clampPulse > 0.0))
        throw std::invalid_argument(std::string(who) + ": the non-complementary drive needs a clamp pulse > 0");

    Cell c{};
    c.vin = ci.inputVoltage; c.lm = ci.magnetizingInductance; c.lk = ci.leakageInductance; c.cc = ci.clampCapacitance;
    c.csw = ci.switchNodeCapacitance;
    c.T = 1.0 / ci.switchingFrequency; c.dt = ci.deadTime; c.tp = ci.complementary ? 0.0 : ci.clampPulse;
    c.comp = ci.complementary;
    // Cantilever ratio of the coupled transformer: a = k·n with k = sqrt(Lm / (Lm + Lk)).
    c.a = ci.turnsRatio * std::sqrt(c.lm / (c.lm + c.lk));
    c.vr = c.a * (ci.outputVoltage + ci.diodeDrop);
    const double iScale = outputCurrent / c.a;
    c.iTol = 1e-9 * iScale;
    c.vTol = 1e-9 * c.vin;
    if (!c.comp && c.tp + 2.0 * c.dt >= c.T)
        throw std::invalid_argument(std::string(who) + ": clamp pulse plus dead times fill the period");

    // Starting point: the leakage-free converter (complementary: volt-second balance; NC: energy balance).
    double D0, im0, vc0;
    const double lp = c.lm + c.lk, pout = c.vr / c.a * outputCurrent;
    if (c.comp) {
        D0 = c.vr / (c.vin + c.vr);
        const double iAvg = iScale / (1.0 - D0), dI = c.vin * D0 * c.T / lp;
        im0 = iAvg - dI / 2.0;
        vc0 = c.vr;
    } else {
        D0 = std::sqrt(2.0 * pout * lp * ci.switchingFrequency) / c.vin;
        im0 = -c.vr * c.tp / c.lm;
        // Clamp charge balance: turn-off pushes ½·Ipk·t_reset into Cc with t_reset = Lk·Ipk/(Vc − Vr); the pulse
        // returns ½·(Vc − Vr)/Lk·tp². Equal when Vc − Vr = Ipk·Lk/tp.
        const double ipk = c.vin * D0 * c.T / lp;
        vc0 = c.vr + std::min(ipk * c.lk / c.tp, c.vr);
    }
    if (!(D0 > 0.0 && D0 < 1.0))
        throw std::invalid_argument(std::string(who) + ": no duty cycle in (0, 1) delivers this output");
    std::array<double, 4> z{im0, im0, vc0, D0};   // ik0, im0, vc0, D

    const double dMax = 1.0 - (c.comp ? 2.0 * c.dt : 2.0 * c.dt + c.tp) / c.T;
    auto residual = [&](const std::array<double, 4>& x) {
        State s{x[0], x[1], x[2], 0.0, (x[1] - x[0]) > c.iTol};
        double q = 0.0;
        const State e = run_period(s, x[3], c, q, nullptr, 0);
        return std::array<double, 4>{(e.ik - x[0]) / iScale, (e.im - x[1]) / iScale, (e.vc - x[2]) / c.vr,
                                     (q / c.T - outputCurrent) / outputCurrent};
    };
    auto norm = [](const std::array<double, 4>& r) {
        double m = 0; for (double v : r) m = std::max(m, std::abs(v)); return m; };
    // Converged at 1e-6 of the current / clamp-voltage / output-current scales (µA, sub-mV): the residual has
    // small kinks where bisected switching events land, so a tighter bound is below the solve's own noise.
    constexpr double kTol = 1e-6;
    std::array<double, 4> r{};
    if (c.comp) {
        // Complementary: Levenberg–Marquardt on the scaled unknowns (initial state and duty). The magnetizing
        // current is bidirectional and never rests, so the period map has no contraction to lean on; the damping
        // keeps the step well posed where two residual rows coincide (rectifier off at Q1's turn-on: ik0 = im0).
        r = residual(z);
        const std::array<double, 4> scale{iScale, iScale, c.vr, 1.0};
        auto sq = [](const std::array<double, 4>& v) { double a = 0; for (double x : v) a += x * x; return a; };
        double mu = 1e-6;
        int it = 0;
        for (; it < 200 && norm(r) > kTol; ++it) {
            std::array<std::array<double, 4>, 4> J{};   // d r_i / d (z_k / scale_k)
            for (int k = 0; k < 4; ++k) {
                std::array<double, 4> zp = z;
                const double h = 1e-7;
                zp[k] += h * scale[k];
                const std::array<double, 4> rp = residual(zp);
                for (int i = 0; i < 4; ++i) J[i][k] = (rp[i] - r[i]) / h;
            }
            std::array<std::array<double, 4>, 4> A{};
            std::array<double, 4> g{};
            for (int a = 0; a < 4; ++a) {
                for (int b = 0; b < 4; ++b) for (int i = 0; i < 4; ++i) A[a][b] += J[i][a] * J[i][b];
                for (int i = 0; i < 4; ++i) g[a] -= J[i][a] * r[i];
            }
            bool accepted = false;
            while (mu < 1e12) {
                std::array<std::array<double, 4>, 4> M = A;
                for (int a = 0; a < 4; ++a) M[a][a] += mu * (A[a][a] + 1e-12);
                const std::array<double, 4> step = solve4(M, g);
                std::array<double, 4> zt = z;
                for (int k = 0; k < 4; ++k) zt[k] += step[k] * scale[k];
                if (zt[3] > 0.0 && zt[3] < dMax) {
                    const std::array<double, 4> rt = residual(zt);
                    if (sq(rt) < sq(r)) { z = zt; r = rt; mu = std::max(mu / 3.0, 1e-12); accepted = true; break; }
                }
                mu *= 4.0;
            }
            if (!accepted)
                throw std::runtime_error(std::string(who) + ": the steady-state solve stalled at residual "
                                         + std::to_string(norm(r)) + " (Vin " + std::to_string(c.vin) + " V)");
        }
    } else {
        // NC-ACF: the transformer resets every cycle, so the period map at a fixed duty is a contraction — its
        // only slow mode is the clamp voltage (Cc's charge balance, ~3 % per period). Iterate it, with Aitken
        // extrapolation of Vc every 20 periods, to the periodic state; then Illinois regula falsi on the duty for
        // the output current (monotone in D), each duty warm-started from the last. Levenberg–Marquardt stalls
        // here on the kinks the pulse and the valley ring put in the residual.
        State warm{z[0], z[1], z[2], 0.0, false};
        auto settle = [&](double D) {
            State x = warm;
            for (int round = 0; round < 400; ++round) {
                std::array<State, 3> xs{};
                double q = 0.0;
                for (int k = 0; k < 3; ++k) { for (int j = 0; j < 20; ++j) x = run_period(x, D, c, q, nullptr, 0); xs[k] = x; }
                const double d1 = xs[1].vc - xs[0].vc, d2 = xs[2].vc - xs[1].vc;
                // Aitken on the slow clamp mode — only while the sequence is visibly geometric and the jump stays
                // physical (a near-unit ratio extrapolated the clamp to −460 V once, into a dead state).
                if (d1 != 0.0) {
                    const double rho = d2 / d1, jump = d2 * rho / (1.0 - rho);
                    if (rho > 0.0 && rho < 0.95 && std::abs(jump) <= 20.0 * std::abs(d2)) {
                        const double vcX = xs[2].vc + jump;
                        if (vcX > c.vr && vcX < 3.0 * c.vr) x.vc = vcX;
                    }
                }
                const State e = run_period(x, D, c, q, nullptr, 0);
                const double res = std::max({std::abs(e.ik - x.ik) / iScale, std::abs(e.im - x.im) / iScale,
                                             std::abs(e.vc - x.vc) / c.vr});
                if (res < 0.1 * kTol) { warm = x; return q / c.T; }
            }
            throw std::runtime_error(std::string(who) + ": the period map did not settle at duty " + std::to_string(D)
                                     + " (Vin " + std::to_string(c.vin) + " V)");
        };
        double a = z[3], ia = settle(a), b = a, ib = ia;
        for (int k = 0; (ia - outputCurrent) * (ib - outputCurrent) > 0.0; ++k) {
            if (k > 60) throw std::runtime_error(std::string(who) + ": no duty brackets the output current");
            a = b; ia = ib;
            b = (ib < outputCurrent) ? std::min(1.08 * b, 0.5 * (b + dMax)) : 0.92 * b;
            ib = settle(b);
        }
        int side = 0;
        double D = b;
        for (int it = 0; it < 80; ++it) {
            D = (a * (ib - outputCurrent) - b * (ia - outputCurrent)) / ((ib - outputCurrent) - (ia - outputCurrent));
            const double iD = settle(D);
            if (std::abs(iD - outputCurrent) < 0.1 * kTol * outputCurrent) break;
            if ((iD - outputCurrent) * (ia - outputCurrent) > 0.0) { a = D; ia = iD; if (side == 1) ib = outputCurrent + 0.5 * (ib - outputCurrent); side = 1; }
            else                                                   { b = D; ib = iD; if (side == -1) ia = outputCurrent + 0.5 * (ia - outputCurrent); side = -1; }
            if (it == 79) throw std::runtime_error(std::string(who) + ": the duty search did not converge");
        }
        z = {warm.ik, warm.im, warm.vc, D};
        r = residual(z);
    }
    if (norm(r) > kTol)
        throw std::runtime_error(std::string(who) + ": no periodic steady state (residual " + std::to_string(norm(r))
                                 + ", Vin " + std::to_string(c.vin) + " V)");

    AcfFlybackSteadyState out{};
    out.dutyCycle = z[3];
    out.period = c.T;
    std::vector<Sample> samples;
    double q = 0.0;
    const int nGrid = 2049;
    run_period(State{z[0], z[1], z[2], 0.0, (z[1] - z[0]) > c.iTol}, z[3], c, q, &samples, nGrid);
    if (static_cast<int>(samples.size()) != nGrid)
        throw std::logic_error(std::string(who) + ": sampled " + std::to_string(samples.size()) + " points, expected "
                               + std::to_string(nGrid));
    for (const auto& sm : samples) {
        out.time.push_back(sm.t);
        out.primaryCurrent.push_back(sm.ik);
        out.primaryVoltage.push_back(sm.vpri);
        out.magnetizingCurrent.push_back(sm.im);
        out.clampVoltage.push_back(sm.vc);
        out.switchNodeVoltage.push_back(sm.vsw);
        out.secondaryCurrent.push_back(sm.isec);
        out.secondaryVoltage.push_back(sm.vsec);
    }
    out.outputCurrent = q / c.T;
    // The current that slews the switch node in the dead time before Q1 turns on: the primary current at the
    // instant the clamp gate drops (negative = discharging the node toward GND, i.e. toward ZVS).
    {
        double qq = 0.0;
        const State atOff = run_period(State{z[0], z[1], z[2], 0.0, (z[1] - z[0]) > c.iTol}, z[3], c, qq, nullptr, 0,
                                       c.T - c.dt);
        out.primaryCurrentAtClampTurnOff = atOff.ik;
        const State atOn = run_period(State{z[0], z[1], z[2], 0.0, (z[1] - z[0]) > c.iTol}, z[3], c, qq, nullptr, 0);
        out.switchNodeVoltageAtTurnOn = atOn.vsw;
        const State atMid = run_period(State{z[0], z[1], z[2], 0.0, (z[1] - z[0]) > c.iTol}, z[3], c, qq, nullptr, 0,
                                       c.T - 0.5 * c.dt);
        out.switchNodeVoltageMidDeadTime = atMid.vsw;
    }
    return out;
}

MAS::OperatingPoint active_clamp_flyback_operating_point(const AcfFlybackSteadyState& s, double switchingFrequency) {
    using WP = OpenMagnetics::WaveformProcessor;
    auto wave = [&](const std::vector<double>& data) {
        MAS::Waveform w;
        w.set_data(data);
        w.set_time(s.time);
        w.set_ancillary_label(MAS::WaveformLabel::CUSTOM);
        return w;
    };
    MAS::OperatingPoint op;
    op.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(wave(s.primaryCurrent), wave(s.primaryVoltage), switchingFrequency, "Primary"));
    op.get_mutable_excitations_per_winding().push_back(
        WP::complete_excitation(wave(s.secondaryCurrent), wave(s.secondaryVoltage), switchingFrequency, "Secondary"));
    return op;
}

} // namespace analytical

namespace {
constexpr double kDefaultMaxDuty = 0.5;
constexpr double kDefaultClampOvershoot = 0.15;   // clamp peak 15 % above the reflected voltage
constexpr double kDefaultDeadTime = 100e-9;
// NC-ACF: the share of the period the transformer is left idle at Vin,min beyond the designed cycle (on-time,
// demagnetization, ZVS pulse, dead times) — room for the leakage commutation and the valley ring.
constexpr double kNcIdleFraction = 0.10;
// NC-ACF leakage cap, as a fraction of the primary inductance (a parasitic, not a design element there).
constexpr double kDefaultNcLeakageRatio = 0.02;

analytical::AcfFlybackCircuit circuit_at(const ActiveClampFlybackDesign& d, double vin) {
    analytical::AcfFlybackCircuit c{};
    c.inputVoltage = vin;
    c.outputVoltage = d.flyback.outputVoltage;
    c.diodeDrop = d.flyback.diodeDrop;
    c.turnsRatio = d.flyback.turnsRatio;
    c.magnetizingInductance = d.magnetizingInductance;
    c.leakageInductance = d.leakageInductance;
    c.clampCapacitance = d.clampCapacitance;
    c.switchingFrequency = d.flyback.switchingFrequency;
    c.deadTime = d.deadTime;
    c.complementary = (d.mode == "complementary");
    c.clampPulse = d.clampPulse;
    c.switchNodeCapacitance = d.switchNodeCapacitance;
    return c;
}

double output_current(const ActiveClampFlybackDesign& d) { return d.flyback.outputPower / d.flyback.outputVoltage; }
} // namespace

ActiveClampFlybackDesign design_active_clamp_flyback(const json& tasInputs) {
    const char* who = "design_active_clamp_flyback";
    json spec = tasInputs;
    json config = cfg::object_of(spec);
    ActiveClampFlybackDesign d{};
    d.mode = cfg::get_str(config, "mode", "complementary");
    if (d.mode != "complementary" && d.mode != "nonComplementary")
        throw std::invalid_argument(std::string(who) + ": config.mode must be complementary or nonComplementary, got '"
                                    + d.mode + "'");
    d.clampSide = cfg::get_str(config, "clampSide", "high");
    if (d.clampSide != "high" && d.clampSide != "low")
        throw std::invalid_argument(std::string(who) + ": config.clampSide must be high or low, got '" + d.clampSide + "'");
    const bool comp = (d.mode == "complementary");
    const double overshoot = cfg::get(config, "clampOvershoot", kDefaultClampOvershoot);
    if (!(overshoot > 0.0 && overshoot <= 1.0))
        throw std::invalid_argument(std::string(who) + ": config.clampOvershoot must be in (0, 1]");
    d.deadTime = cfg::get(config, "deadTime", kDefaultDeadTime);
    d.switchNodeCapacitance = cfg::qr_resonant_cap(config);
    if (!(d.switchNodeCapacitance > 0.0))
        throw std::invalid_argument(std::string(who) + ": config.resonantCapacitance (switch-node capacitance) must be > 0");

    // The transformer turns ratio, rectifier and filter capacitors are the flyback's: CCM sizing for the
    // complementary drive (the magnetizing current never stops), DCM for NC-ACF (the clamp pulse needs an
    // idle transformer before Q1 turns on).
    json fcfg = config;
    fcfg["mode"] = comp ? "ccm" : "dcm";
    if (!fcfg.contains("maxDutyCycle")) fcfg["maxDutyCycle"] = kDefaultMaxDuty;
    spec["config"] = fcfg;
    d.flyback = design_flyback(spec);
    if (d.flyback.outputs.size() != 1)
        throw std::invalid_argument(std::string(who) + ": one output only (the active-clamp flyback is single-rail here)");
    const FlybackDesign& f = d.flyback;
    const double T = 1.0 / f.switchingFrequency;
    if (!(d.deadTime > 0.0 && d.deadTime < 0.05 * T))
        throw std::invalid_argument(std::string(who) + ": config.deadTime must be in (0, 5 % of the period)");
    const double io = output_current(d), vr = f.turnsRatio * (f.outputVoltage + f.diodeDrop);

    // ZVS current: a negative magnetizing current that slews the switch node from Vin + Vc down to GND within
    // the dead time, at the highest node voltage (Vin,max, clamp at its overshoot peak).
    d.zvsCurrent = d.switchNodeCapacitance * (f.inputVoltageMax + vr * (1.0 + overshoot)) / d.deadTime;

    // Magnetizing inductance and the off-time at Vin,min.
    double ipkMin, tOffMin;
    if (comp) {
        // Complementary: the bidirectional magnetizing current must dip to −2·Izvs at every input corner — the
        // current that slews the node in HALF the dead time, the margin the steady state is checked against:
        //   Iavg − ΔI/2 ≤ −2·Izvs  with  Iavg = Io/(n·(1−D)),  ΔI = Vin·D·T/Lm,  D = Vr/(Vin + Vr).
        double lm = std::numeric_limits<double>::infinity();
        for (double vin : {f.inputVoltageMin, f.inputVoltageMax}) {
            const double D = vr / (vin + vr), iAvg = io / f.turnsRatio / (1.0 - D);
            lm = std::min(lm, vin * D * T / (2.0 * (iAvg + 2.0 * d.zvsCurrent)));
        }
        d.magnetizingInductance = lm;
        const double D = vr / (f.inputVoltageMin + vr);
        ipkMin = io / f.turnsRatio / (1.0 - D) + f.inputVoltageMin * D * T / (2.0 * lm);
        tOffMin = (1.0 - D) * T;
    } else {
        // NC-ACF: the transformer must reset every cycle at every input corner, leaving room for what the ZVS
        // itself costs. At Vin,min, with a negative magnetizing current Ineg = 2·Izvs (enough to slew the node in
        // half the dead time) and the energy balance ½·Lp·(Ipk² − Ineg²)·fsw = P:
        //     t_on = (Ipk + Ineg)·Lp/Vin,   t_demag = Ipk·Lm/Vr,   t_pulse = Ineg·Lm/Vr,
        //     t_on + t_demag + t_pulse + 2·dead time ≤ (1 − kNcIdleFraction)·T.
        // Lm is the largest that fits (bisection; the left side grows with L). A fixed fraction of the critical
        // inductance (0.5·Lcrit) ran the transformer at the conduction boundary once the pulse's circulating
        // current was paid for, and the pulse could no longer pull the magnetizing current negative.
        const double vinMin = f.inputVoltageMin, ineg = 2.0 * d.zvsCurrent, P = f.outputPower;
        // What ZVS costs in NC-ACF: the pulse leaves ½·Lp·Ineg² in the magnetizing inductance each cycle, which
        // goes back to the input at Q1's turn-on. Bounded at half the energy delivered per cycle (checked once Lp
        // is known, below): past that the steady state is dominated by recirculation, the solver finds several of
        // them, and the design is a poor one anyway.
        auto cycle = [&](double lp) {
            const double lm = lp * (1.0 - cfg::get(config, "leakageRatio", kDefaultNcLeakageRatio));
            const double ipk = std::sqrt(2.0 * P / (lp * f.switchingFrequency) + ineg * ineg);
            return (ipk + ineg) * lp / vinMin + ipk * lm / vr + ineg * lm / vr + 2.0 * d.deadTime;
        };
        const double budget = (1.0 - kNcIdleFraction) * T;
        double lo = 0.0, hi = vinMin * T / (2.0 * ineg);   // at hi the on-time alone recovers 2·Ineg in T/2
        if (!(cycle(hi) > budget))
            throw std::logic_error(std::string(who) + ": NC-ACF inductance bracket does not close");
        for (int it = 0; it < 80; ++it) { const double m = 0.5 * (lo + hi); (cycle(m) <= budget ? lo : hi) = m; }
        if (!(lo > 0.0))
            throw std::invalid_argument(std::string(who) + ": no magnetizing inductance leaves the NC-ACF cycle room "
                "for its ZVS pulse at Vin,min (raise the dead time, lower the switch-node capacitance or the frequency)");
        d.magnetizingInductance = lo * (1.0 - cfg::get(config, "leakageRatio", kDefaultNcLeakageRatio));
        const double recirculated = 0.5 * lo * ineg * ineg * f.switchingFrequency;
        if (recirculated > 0.5 * P)
            throw std::invalid_argument(std::string(who) + ": NC-ACF ZVS would recirculate " + std::to_string(recirculated)
                + " W against the " + std::to_string(P) + " W delivered (the " + std::to_string(2.0 * d.zvsCurrent)
                + " A negative magnetizing current that slews the switch node in half the dead time at Vin,max); "
                "use the complementary drive, a longer dead time or less switch-node capacitance");
        ipkMin = std::sqrt(2.0 * P / (lo * f.switchingFrequency) + ineg * ineg);
        tOffMin = T - (ipkMin + ineg) * lo / vinMin;
    }
    if (comp) {
        // Complementary: Lk is designed with Cc. Their half resonant period is the off-time at Vin,min, and the
        // resonance's characteristic impedance puts the clamp peak `overshoot`·Vr above the reflected voltage at
        // the peak primary current:  π·√(Lk·Cc) = t_off,  √(Lk/Cc) = overshoot·Vr / Ipk.
        const double Z = overshoot * vr / ipkMin;
        d.leakageInductance = Z * tOffMin / M_PI;
        d.clampCapacitance = tOffMin / (M_PI * Z);
        if (!(d.leakageInductance < 0.5 * d.magnetizingInductance))
            throw std::invalid_argument(std::string(who) + ": the designed leakage inductance "
                + std::to_string(d.leakageInductance) + " H is not below half the magnetizing inductance "
                + std::to_string(d.magnetizingInductance) + " H (lower config.clampOvershoot or raise the switching frequency)");
        d.primaryInductance = d.magnetizingInductance + d.leakageInductance;
    } else {
        // NC-ACF: Qc is off through the off-time, so Lk is not a resonant design element: it is a parasitic,
        // capped at config.leakageRatio of Lp (a MAXIMUM requirement on the magnetic). (The complementary drive's
        // resonant Lk ≈ 8 % of Lp with Cc ≈ 100 nF gives the NC circuit several steady states.)
        // Cc keeps the same rule — half its Lk resonance period is the off-time at Vin,min — which against the
        // small parasitic Lk makes it large (hundreds of nF): the pulse then stays well inside a quarter Lk–Cc
        // period and the clamp holds rather than rings. Sized from the leakage energy alone (Cc·Vr·ΔV = ½·Lk·Ipk²)
        // it came out ~3 nF, its quarter period shorter than the pulse, and the clamp peaked ~50 % above Vr
        // whatever the pulse. The leakage energy must still lift the clamp by no more than half the overshoot
        // budget; checked.
        const double ratio = cfg::get(config, "leakageRatio", kDefaultNcLeakageRatio);
        if (!(ratio > 0.0 && ratio < 0.2))
            throw std::invalid_argument(std::string(who) + ": config.leakageRatio must be in (0, 0.2)");
        d.primaryInductance = d.magnetizingInductance / (1.0 - ratio);
        d.leakageInductance = d.primaryInductance - d.magnetizingInductance;
        d.clampCapacitance = (tOffMin / M_PI) * (tOffMin / M_PI) / d.leakageInductance;
        const double lift = 0.5 * d.leakageInductance * ipkMin * ipkMin / (d.clampCapacitance * vr);
        if (!(lift <= 0.5 * overshoot * vr))
            throw std::invalid_argument(std::string(who) + ": absorbing the leakage energy lifts the clamp "
                + std::to_string(lift) + " V, over half the " + std::to_string(overshoot * vr) + " V overshoot budget");

        // Pulse width: ZVS. While the rectifier conducts, the transformer clamps Lm and only Lk rings the switch
        // node — about Vin + Vr with Z = √(Lk/Csw), which for a parasitic Lk bottoms out far above GND however
        // negative the clamp leaves the primary current. What pulls the node down is a negative MAGNETIZING
        // current: with the rectifier off, all of Lp drives it. So the pulse runs until the magnetizing current
        // reaches about −Izvs (tp0 = Izvs·Lm/Vr, with Vr across Lm), and is then the shortest that, in the
        // steady state, lets the node reach GND within HALF the dead time at every corner (the longest of the
        // per-corner pulses). Half, not all: the shortest pulse that just makes it by Q1's turn-on has no margin,
        // and ngspice — real diode drops — lagged it by 30 V. Cc's charge balance sets the clamp voltage that results. (With the resonant
        // ~100 nF Cc this pulse drained the clamp to Vr·Lp/Lm and nothing reached the output; the large Cc
        // holds it.)
        const double tp0 = d.zvsCurrent * d.magnetizingInductance / vr;
        auto zvsAt = [&](double vin, double pulse) {
            d.clampPulse = pulse;
            return analytical::solve_active_clamp_flyback(circuit_at(d, vin), io).switchNodeVoltageMidDeadTime
                   <= 1e-6 * vin;
        };
        double tp = 0.0;
        for (double vin : {f.inputVoltageMin, f.inputVoltageMax}) {
            const double tMax = T - 2.0 * d.deadTime - (vin / (vin + vr)) * T;   // the off-time, at most
            double fail = 0.0, ok = std::min(tp0, 0.9 * tMax);
            if (zvsAt(vin, ok)) {
                for (int k = 0; ; ++k) {                       // shorten until it fails
                    if (k > 40) throw std::logic_error(std::string(who) + ": ZVS with a vanishing clamp pulse");
                    const double x = 0.8 * ok;
                    if (zvsAt(vin, x)) ok = x; else { fail = x; break; }
                }
            } else {
                for (int k = 0; ; ++k) {                       // lengthen until it works
                    fail = ok;
                    ok = 1.25 * ok;
                    if (ok >= tMax || k > 40)
                        throw std::invalid_argument(std::string(who) + ": at Vin " + std::to_string(vin) + " V no clamp "
                            "pulse that fits the off-time brings the switch node to GND within half the dead time (no ZVS; "
                            "raise the dead time or lower the switch-node capacitance)");
                    if (zvsAt(vin, ok)) break;
                }
            }
            for (int it = 0; it < 12; ++it) {                  // bisect to 0.03 % of the bracket
                const double x = 0.5 * (fail + ok);
                if (zvsAt(vin, x)) ok = x; else fail = x;
            }
            tp = std::max(tp, ok);
        }
        d.clampPulse = tp;
    }

    // Steady state at the operating point (the PWM duty) and at both corners (the ratings).
    const auto nom = analytical::solve_active_clamp_flyback(circuit_at(d, f.inputVoltage), io);
    d.dutyCycle = nom.dutyCycle;
    d.clampVoltageMax = 0.0;
    d.switchVoltageMax = 0.0;
    for (double vin : {f.inputVoltageMin, f.inputVoltage, f.inputVoltageMax}) {
        const auto ss = analytical::solve_active_clamp_flyback(circuit_at(d, vin), io);
        const double vcMax = *std::max_element(ss.clampVoltage.begin(), ss.clampVoltage.end());
        d.clampVoltageMax = std::max(d.clampVoltageMax, vcMax);
        d.switchVoltageMax = std::max(d.switchVoltageMax, vin + vcMax);
        if (ss.switchNodeVoltageMidDeadTime > 1e-6 * vin)
            throw std::invalid_argument(std::string(who) + ": at Vin " + std::to_string(vin) + " V the switch node is "
                "still at " + std::to_string(ss.switchNodeVoltageMidDeadTime) + " V half-way through the dead time "
                "before Q1 turns on (no ZVS margin; raise the dead time or lower the switch-node capacitance)");
        if (!comp) {
            // NC-ACF only works if the transformer has reset before the pulse: an idle (zero) rectifier current
            // just before the clamp gate rises.
            const double tPulse = T - d.deadTime - d.clampPulse;
            const std::size_t k = static_cast<std::size_t>(std::floor(tPulse / T * (ss.time.size() - 1))) - 1;
            if (ss.secondaryCurrent.at(k) > 1e-6 * io)
                throw std::invalid_argument(std::string(who) + ": at Vin " + std::to_string(vin) + " V the transformer is "
                    "still delivering when the clamp pulse starts — NC-ACF needs discontinuous conduction "
                    "(lower the power or the magnetizing inductance)");
        }
    }
    return d;
}

json build_active_clamp_flyback_tas(const ActiveClampFlybackDesign& d) {
    namespace AN = analytical;
    const FlybackDesign& f = d.flyback;
    const double fsw = f.switchingFrequency, T = 1.0 / fsw;
    const bool comp = (d.mode == "complementary"), high = (d.clampSide == "high");

    // The rectifier, output filter, transformer stage and external ports are the flyback's; the ACF replaces
    // the primary switch stage and the transformer's requirements.
    FlybackDesign fd = f;
    fd.magnetizingInductance = d.primaryInductance;
    fd.dutyCycle = d.dutyCycle;
    fd.resonantCapacitance = 0.0;
    fd.valleyDeadTime = 0.0;
    json tas = build_flyback_tas(fd);

    const double io = output_current(d);
    const auto ssNom = AN::solve_active_clamp_flyback(circuit_at(d, f.inputVoltage), io);
    const auto ssMin = AN::solve_active_clamp_flyback(circuit_at(d, f.inputVoltageMin), io);
    const MAS::OperatingPoint opNom = AN::active_clamp_flyback_operating_point(ssNom, fsw);
    auto rms = [](const std::vector<double>& v) { double a = 0; for (double x : v) a += x * x; return std::sqrt(a / v.size()); };
    auto peakAbs = [](const std::vector<double>& v) { double a = 0; for (double x : v) a = std::max(a, std::abs(x)); return a; };
    const double ipk = peakAbs(ssMin.primaryCurrent), irms = rms(ssMin.primaryCurrent);

    // Transformer: Lp = Lm + Lk, with Lk as a leakage requirement.
    bool replaced = false;
    for (auto& st : tas.at("topology").at("stages")) {
        if (st.at("name") != "transformer") continue;
        for (auto& c : st.at("circuit").at("components")) {
            if (c.at("name") != "T1") continue;
            std::vector<std::string> iso{"primary", req::isolation_side(1)};
            std::optional<double> isoV = f.isolationVoltage > 0 ? std::optional<double>(f.isolationVoltage) : std::nullopt;
            json& inputs = c.at("data")["inputs"];
            inputs = req::magnetic_inputs(d.primaryInductance, 0.1, {f.turnsRatio}, iso, isoV, 25.0,
                                          AN::excitations_processed(opNom, "T1"));
            // Complementary: Lk is a designed element the magnetic must realise (nominal). NC-ACF: a parasitic to
            // stay under (maximum). The deck resolves either bound to Lk.
            req::set_leakage_requirement(inputs, {d.leakageInductance}, comp ? "nominal" : "maximum");
            replaced = true;
        }
    }

    if (!replaced) throw std::logic_error("build_active_clamp_flyback_tas: the flyback TAS has no transformer T1");

    // Primary stage.
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp_ = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };

    json* inv = nullptr;
    for (auto& st : tas.at("topology").at("stages")) if (st.at("name") == "inverter") inv = &st;
    if (!inv) throw std::logic_error("build_active_clamp_flyback_tas: the flyback TAS has no 'inverter' stage");
    json cin;
    for (const auto& c : inv->at("circuit").at("components")) if (c.at("name") == "Cin") cin = c.at("data");
    if (cin.is_null()) throw std::logic_error("build_active_clamp_flyback_tas: the flyback switch stage lacks Cin");

    const double vds = d.switchVoltageMax;
    cfg::check_maximum_drain_source_voltage(f.config, vds, "build_active_clamp_flyback_tas");
    const double ratedVds = vds / cfg::v_derate_mosfet(f.config);
    const double pout = f.outputPower;
    json q1; q1["semiconductor"]["mosfet"] = json::object();
    q1["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, ipk,
                                                     cfg::rds_on_loss_fraction(f.config) * pout / (irms * irms), 125.0);
    // The clamp switch carries the primary current while Qc conducts: the off-interval part of the waveform.
    double clampSq = 0.0, clampPk = 0.0;
    for (std::size_t k = 0; k < ssMin.time.size(); ++k)
        if (ssMin.time[k] >= ssMin.dutyCycle * T) {
            clampSq += ssMin.primaryCurrent[k] * ssMin.primaryCurrent[k];
            clampPk = std::max(clampPk, std::abs(ssMin.primaryCurrent[k]));
        }
    const double clampRms = std::sqrt(clampSq / ssMin.time.size());
    json qc; qc["semiconductor"]["mosfet"] = json::object();
    qc["inputs"]["designRequirements"] = req::mosfet("mainSwitch", ratedVds, clampPk,
                                                     cfg::rds_on_loss_fraction(f.config) * pout / (clampRms * clampRms), 125.0);
    auto bodyDio = [&](double ratedIf) {
        json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(ratedVds, ratedIf); return j; };
    // Clamp capacitor: high-side it holds Vc, low-side Vin + Vc.
    const double ccVolts = high ? d.clampVoltageMax : f.inputVoltageMax + d.clampVoltageMax;
    json cc; cc["capacitor"] = json::object();
    cc["inputs"]["designRequirements"] = req::capacitor(
        d.clampCapacitance, ccVolts / cfg::v_derate_capacitor(f.config), clampRms,
        0.01 * std::sqrt(d.leakageInductance / d.clampCapacitance), "resonant");
    // Switch-node capacitance (Coss + winding capacitance, lumped): the part the dead-time transition slews.
    json csw; csw["capacitor"] = json::object();
    csw["inputs"]["designRequirements"] = req::capacitor(
        d.switchNodeCapacitance, ratedVds, d.switchNodeCapacitance * vds * fsw,
        0.01 * std::sqrt(d.primaryInductance / d.switchNodeCapacitance), "resonant");

    json brick;
    brick["name"] = "active-clamp-primary";
    brick["ports"] = json::array({port("dc+"), port("sw"), port("gate"), port("gate_clamp"), port("dc-")});
    brick["components"] = json::array({comp_("Q1", q1), comp_("Dq1", bodyDio(ipk)), comp_("Qc", qc),
                                       comp_("Dqc", bodyDio(clampPk)), comp_("Cc", cc), comp_("Csw", csw),
                                       comp_("Cin", cin)});
    if (high) {
        brick["connections"] = json::array({
            conn("dc_pos", {pin("Cin", "1"), pin("Cc", "1"), prt("dc+")}),
            conn("clamp",  {pin("Cc", "2"), pin("Qc", "drain"), pin("Dqc", "cathode")}),
            conn("sw",     {pin("Q1", "drain"), pin("Dq1", "cathode"), pin("Qc", "source"), pin("Dqc", "anode"),
                            pin("Csw", "1"), prt("sw")}),
            conn("gate",   {pin("Q1", "gate"), prt("gate")}),
            conn("gate_clamp", {pin("Qc", "gate"), prt("gate_clamp")}),
            conn("dc_neg", {pin("Q1", "source"), pin("Dq1", "anode"), pin("Csw", "2"), pin("Cin", "2"), prt("dc-")})});
    } else {
        brick["connections"] = json::array({
            conn("dc_pos", {pin("Cin", "1"), prt("dc+")}),
            conn("sw",     {pin("Q1", "drain"), pin("Dq1", "cathode"), pin("Cc", "1"), pin("Csw", "1"), prt("sw")}),
            conn("clamp",  {pin("Cc", "2"), pin("Qc", "source"), pin("Dqc", "anode")}),
            conn("gate",   {pin("Q1", "gate"), prt("gate")}),
            conn("gate_clamp", {pin("Qc", "gate"), prt("gate_clamp")}),
            conn("dc_neg", {pin("Q1", "source"), pin("Dq1", "anode"), pin("Qc", "drain"), pin("Dqc", "cathode"),
                            pin("Csw", "2"), pin("Cin", "2"), prt("dc-")})});
    }
    (*inv)["circuit"] = brick;

    // Series-RC numerical snubber across the output rectifier (anode → Csnd → Rsnd → cathode). The leakage
    // resonance turns D1 off with a steep current zero, which the system ngspice-42 (KLU) ran into "timestep too
    // small" at 120 V while libngspice got through: the RC gives that node a defined dV/dt. Series, not the LLC's
    // R ∥ C: 100 Ω across a diode blocking Vo + Vin/n ≈ 45 V would burn ~10 W in the deck; in series it costs
    // C·V²·f ≈ 30 mW. Both parts tagged for the real-fidelity strip.
    {
        json* rect = nullptr;
        for (auto& st : tas.at("topology").at("stages")) if (st.at("name") == "rectifier") rect = &st;
        if (!rect) throw std::logic_error("build_active_clamp_flyback_tas: the flyback TAS has no 'rectifier' stage");
        const double vBlock = f.outputVoltage + f.inputVoltageMax / f.turnsRatio;
        json csn; csn["capacitor"] = json::object();
        csn["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::rectifier_snubber_cap(f.config);
        csn["inputs"]["designRequirements"]["ratedVoltage"] = 2.0 * vBlock;
        cfg::mark_numerical_aid(csn);
        json rsn; rsn["resistor"] = json::object();
        auto& rdr = rsn["inputs"]["designRequirements"];
        rdr["deviceType"] = "resistor";
        rdr["resistance"]["nominal"] = cfg::snubber_res(f.config);
        rdr["powerRating"] = cfg::rectifier_snubber_cap(f.config) * vBlock * vBlock * fsw;
        rdr["role"] = "snubber";
        cfg::mark_numerical_aid(rsn);
        json& brick = rect->at("circuit");
        brick["components"].push_back(comp_("Csnd", csn));
        brick["components"].push_back(comp_("Rsnd", rsn));
        for (auto& cn : brick.at("connections")) {
            if (cn.at("name") == "anode")   cn["endpoints"].push_back(pin("Csnd", "1"));
            if (cn.at("name") == "cathode") cn["endpoints"].push_back(pin("Rsnd", "2"));
        }
        brick["connections"].push_back(conn("snub_mid", {pin("Csnd", "2"), pin("Rsnd", "1")}));
    }

    // Gate drives: Q1 at the solved duty; Qc complementary (dead time either side) or one pulse before Q1.
    json q2stim;
    for (auto& s : tas.at("simulation").at("stimulus"))
        if (s.value("stage", "") == "inverter" && s.value("component", "") == "Q1") q2stim = s;
    if (q2stim.is_null()) throw std::logic_error("build_active_clamp_flyback_tas: no Q1 gate stimulus");
    q2stim["component"] = "Qc";
    if (comp) {
        q2stim["waveform"]["dutyCycle"] = 1.0 - d.dutyCycle - 2.0 * d.deadTime / T;
        q2stim["waveform"]["phase"] = (d.dutyCycle + d.deadTime / T) * 360.0;
    } else {
        q2stim["waveform"]["dutyCycle"] = d.clampPulse / T;
        q2stim["waveform"]["phase"] = (1.0 - (d.deadTime + d.clampPulse) / T) * 360.0;
    }
    tas["simulation"]["stimulus"].push_back(q2stim);

    req::finalize_control_seeds(tas, Topology::ACTIVE_CLAMP_FLYBACK_CONVERTER);
    return tas;
}

} // namespace Kirchhoff
