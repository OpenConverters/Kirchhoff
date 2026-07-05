#include "Dab.hpp"
#include "DimensionJson.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include "KirchhoffConfig.hpp"
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }

// SPS (Single-Phase-Shift) modulation: only the inter-bridge outer shift D3 is non-zero (D1=D2=0).
// MKF's process_design_requirements picks D3 ≈ 25° (good controllability margin) when no series
// inductance / phase is specified, then solves L for the rated power at that D3. We reproduce that
// exact choice so Kirchhoff designs the same N / L / Lm as MKF.
constexpr double kD3Deg       = 25.0;
// Per-switch on-fraction = (halfPeriod − deadTime)/period. MINIMAL dead time (~20 ns at 100 kHz — the
// least that prevents ideal-bridge shoot-through and keeps ngspice convergent). Real designs minimise
// dead time; the body-diode conduction a LARGE dead time causes is exactly what pulls the open-loop
// output below the lossless SPS target, so with minimal dead time the DAB delivers spec with no fudge.
constexpr double kSwitchDuty  = 0.499;
} // namespace

DabDesign design_dab(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    DabDesign d{};
    const json config = cfg::object_of(tasInputs);
    const double d3deg = cfg::get(config, "dabPhaseShiftDeg", kD3Deg);
    // `dabPhaseShiftDeg` is the SPS outer phase shift D3 in DEGREES (inner shifts D1=D2=0). Power transfer
    // and the series-inductance sizing L = N·V1·V2·D3·(π−|D3|)/(2π²·Fs·P) are only physical for
    // 0 < D3 < 180° (L ≤ 0 outside that band); the controllable SPS band is ~0–90°. Guard loudly rather
    // than emitting a negative/zero Lr downstream. (A common mistake is passing a per-unit shift ×180 — e.g.
    // D3=0.3 → 54°, not 0.3.)
    if (!(d3deg > 0.0 && d3deg < 180.0))
        throw std::invalid_argument("design_dab: dabPhaseShiftDeg must be a degrees value in (0, 180) — SPS "
                                    "outer phase shift D3 (recommended 0–90°); got " + std::to_string(d3deg));
    // Inner (intra-bridge) phase shifts D1 (primary) / D2 (secondary), in DEGREES, for EPS/DPS/TPS. Default
    // 0 (SPS). `dabModulationType` = SPS|EPS|DPS|TPS is an optional hint: for DPS an unspecified D2 mirrors D1
    // (the defining DPS constraint); an explicit dabInnerPhaseShift2Deg always wins. Bounds [0, 90) — 90°
    // collapses the half-cycle to zero width (Huang 2018 / standard TPS literature).
    const double d1deg = cfg::get(config, "dabInnerPhaseShift1Deg", 0.0);
    std::string modType = config.value("dabModulationType", std::string{});
    for (char& ch : modType) ch = (char)std::toupper((unsigned char)ch);
    const double d2deg = cfg::get(config, "dabInnerPhaseShift2Deg", modType == "DPS" ? d1deg : 0.0);
    if (!(d1deg >= 0.0 && d1deg < 90.0))
        throw std::invalid_argument("design_dab: dabInnerPhaseShift1Deg (D1) must be in [0, 90) degrees; got "
                                    + std::to_string(d1deg));
    if (!(d2deg >= 0.0 && d2deg < 90.0))
        throw std::invalid_argument("design_dab: dabInnerPhaseShift2Deg (D2) must be in [0, 90) degrees; got "
                                    + std::to_string(d2deg));
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency = dr.value("efficiency", 0.9);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty()) {
        const json& op = tasInputs.at("operatingPoints").at(0);
        d.inputVoltage = op.at("inputVoltage").get<double>();
        d.outputPower = op.at("outputs").at(0).at("power").get<double>();
    } else {
        d.inputVoltage = nominal(dr.at("inputVoltage"));
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));
    }
    const json& iv = dr.at("inputVoltage");
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage, Fs = d.switchingFrequency;
    const double P = d.outputPower;
    const double D3 = d3deg * M_PI / 180.0;

    // 1. Turns ratio N = V1_nom / V2_nom (MKF rounds to 2 decimals).
    double N = Vin / Vo;
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(N * 100.0) / 100.0);
    // 2. Series inductance L for the rated power at D3 = 25° (default) — UNLESS the caller pinned it.
    // della-Pollock / MKF AdvancedDab: a pinned Lr (desiredSeriesInductance in designRequirements, or
    // config["seriesInductance"]) fixes the tank and the rest of the stage sizes around THAT value
    // instead of the modulation-derived one (ABT #95 item 5). std::optional (never an in-band sentinel)
    // records whether a pin was supplied.
    std::optional<double> pinnedLr = req::provided_series_inductance(dr);
    if (!pinnedLr && config.is_object() && config.contains("seriesInductance") &&
        config.at("seriesInductance").is_number())
        pinnedLr = config.at("seriesInductance").get<double>();
    d.seriesInductancePinned = pinnedLr.has_value();
    if (pinnedLr) {
        if (!(*pinnedLr > 0.0))
            throw std::invalid_argument("design_dab: pinned seriesInductance must be > 0, got " +
                                        std::to_string(*pinnedLr));
        d.seriesInductance = *pinnedLr;
    } else if (d1deg == 0.0 && d2deg == 0.0) {
        //    SPS: L = N·V1·V2·D3·(π−|D3|) / (2π²·Fs·P).  (MKF Dab::compute_series_inductance — the exact ideal
        //    SPS power, not FHA.) With minimal dead time (above) the open-loop output lands on spec.
        d.seriesInductance = N * Vin * Vo * D3 * (M_PI - std::abs(D3)) / (2.0 * M_PI * M_PI * Fs * P);
    } else {
        //    EPS/DPS/TPS: the SPS closed form no longer holds — size L from the general power model (same tank
        //    kernel analytical_dab emits) so the design still delivers P at the chosen (D1, D2, D3).
        d.seriesInductance = Kirchhoff::analytical::dab_series_inductance_for_power(
            Vin, Vo, N, D3, d1deg * M_PI / 180.0, d2deg * M_PI / 180.0, Fs, P);
    }

    // 3. Magnetizing inductance: max(Vin²/(1.2·Fs·P), 10·L) — 30% magnetizing-ripple target, floored
    //    at 10× the series inductance (MKF Dab::process_design_requirements step 4).
    double LmFromCurrent = Vin * Vin / (1.2 * Fs * P);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        std::max(LmFromCurrent, 10.0 * d.seriesInductance));

    // useLeakageInductance (ABT #95): FOLD Lr into the transformer's leakage instead of emitting a discrete
    // series inductor. The transformer already carries leakage L_leak = Lm·(1−K²); setting K so L_leak == Lr
    // makes the coupling itself provide the tank inductance. Requires Lr < Lm for a real K (Lm >= 10·Lr by
    // the floor above, but a small pinned Lm could violate it — throw loudly rather than emit an imaginary K).
    d.useLeakageInductance = cfg::get_bool(config, "useLeakageInductance", false);
    if (d.useLeakageInductance && !(d.seriesInductance < d.magnetizingInductance))
        throw std::invalid_argument(
            "design_dab: useLeakageInductance requires seriesInductance (" +
            std::to_string(d.seriesInductance) + ") < magnetizingInductance (" +
            std::to_string(d.magnetizingInductance) + ") for a real coupling coefficient");

    d.phaseShiftDeg = d3deg;
    d.innerPhaseShift1Deg = d1deg;
    d.innerPhaseShift2Deg = d2deg;
    d.switchDuty = cfg::get(config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Vo * Vo / P;
    d.config = config;
    d.outputCapacitance = cfg::get(config, "outputCapacitance", 100e-6);  // DAB has no output L; MKF uses 100u

    // Per-output rails (multi-output: N isolated secondaries, ABT #86). Every secondary bridge hangs off the
    // SAME primary tank (one Lr, one D3); each rail gets its own transformer secondary winding turns ratio
    // N_i = V1_nom/V2_i so, at the shared phase shift, it regulates to its OWN Vout. outputs[0] reproduces the
    // scalar single-output design (turnsRatio/outputVoltage/outputPower/outputCapacitance) byte-for-byte.
    const size_t nOut = dr.at("outputs").size();
    const bool haveOp = tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty();
    for (size_t i = 0; i < nOut; ++i) {
        DabOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"));
        if (haveOp)
            leg.power = tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        else
            leg.power = nominal(dr.at("outputs").at(i).at("power"));
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;             // preserve the pinned/rounded main ratio exactly
            leg.outputCapacitance = d.outputCapacitance;
        } else {
            double ni = std::round((Vin / leg.voltage) * 100.0) / 100.0;   // same N = V1_nom/V2_i rule as rail 0
            leg.turnsRatio = req::provided_turns_ratio(dr, i).value_or(ni);
            leg.outputCapacitance = cfg::get(config, "outputCapacitance", 100e-6);
        }
        leg.loadResistance = leg.voltage * leg.voltage / leg.power;
        d.outputs.push_back(leg);
    }
    return d;
}

json build_dab_tas(const DabDesign& d) {
    auto port = [](const char* n) { json p; p["name"] = n; return p; };
    auto pin  = [](const char* c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const char* p) { json e; e["port"] = p; return e; };
    auto conn = [](const char* name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const char* name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto sp = [](const char* st, const char* po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const char* name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };
    auto mosfet = [](json reqs = json()) { json j; j["semiconductor"]["mosfet"] = json::object();
        if (!reqs.is_null()) { j["inputs"]["designRequirements"] = reqs; } return j; };
    auto diode  = [&](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs.is_null()
            ? req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage) : reqs; return j; };

    const double N = d.turnsRatio, Lm = d.magnetizingInductance;
    const double fsw = d.switchingFrequency, Lr_H = d.seriesInductance;

    // --- SPS tank-current stress (the current shared by Lr and the transformer primary) ---
    // SPS inductor current is the piecewise-linear (trapezoidal) tank current. Primary-referred levels:
    // V1 = Vin, V2' = N·Vout; the inductor sees (V1+V2') for the phase-shift fraction φ then (V1−V2') for
    // the rest of the half-period, sign-symmetric. di/dθ = v_L/(ωL). Extrema (half-wave symmetric, so
    // i(π) = −i(0)):
    const double w = 2.0 * M_PI * fsw;                            // electrical angular frequency
    const double phi = d.phaseShiftDeg * M_PI / 180.0;           // outer phase shift D3 (rad)
    const double V1t = d.inputVoltage, V2t = N * d.outputVoltage; // primary-referred bridge square levels
    const double i0  = -(V1t * M_PI + V2t * (2.0 * phi - M_PI)) / (2.0 * w * Lr_H);  // i_L at θ=0
    const double iphi = i0 + (V1t + V2t) * phi / (w * Lr_H);                          // i_L at θ=φ
    const double IpkTank = std::max(std::abs(i0), std::abs(iphi));
    // RMS over a half period of the two linear segments [i0→iphi over φ] and [iphi→−i0 over π−φ]:
    auto segSq = [](double a, double b, double len) { return len * (a * a + a * b + b * b) / 3.0; };
    const double IrmsTank = std::sqrt((segSq(i0, iphi, phi) + segSq(iphi, -i0, M_PI - phi)) / M_PI);
    const double dITank = std::abs(iphi - i0);                   // pk-pk swing within a half-period

    // Inductor voltage levels (square, sign-symmetric); RMS = sqrt(d·(V1+V2')² + (1−d)·(V1−V2')²).
    const double dFrac = phi / M_PI;
    const double vLrPk = std::max(std::abs(V1t + V2t), std::abs(V1t - V2t)), vLrPkPk = 2.0 * vLrPk;
    const double vLrRms = std::sqrt(dFrac * (V1t + V2t) * (V1t + V2t) + (1.0 - dFrac) * (V1t - V2t) * (V1t - V2t));

    // Series (resonant + leakage) inductor Lr — single-winding magnetic carrying the full tank current.
    // Only emitted as a DISCRETE component when useLeakageInductance is off; otherwise Lr is folded into the
    // transformer coupling (see below) and no separate inductor exists.
    json lr; lr["magnetic"] = json::object();
    lr["inputs"] = req::magnetic_inputs(Lr_H, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("triangular", fsw, IpkTank, IrmsTank, 0.0, dITank, std::nullopt,
                                    vLrPk, vLrRms, 0.0, vLrPkPk)});

    // --- semiconductor stresses: primary bridge blocks Vin, each secondary bridge blocks its own Vout ---
    // QA..QD (primary full bridge) block the DC-link Vin_max and carry the tank current (IrmsTank).
    // QE..QH (secondary active bridge, per rail) block Vout_i and carry the reflected + load-shared tank
    // current. All switches are real, independently-driven (no passive rectifier — the secondary is a
    // driven bridge, the phase shift sets the power transfer); their anti-parallel diodes are BODY diodes.
    const double ratedVdsPri = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnPri = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsTank * IrmsTank);

    // Load-share weighting for the reflected secondary current (MATCHES analytical_dab exactly: each
    // secondary carries n_i·iL·share_i, share_i = (Io_i/Vo_i)/Σ(Io/Vo)). For a single output share == 1.0,
    // so IpkSec/IrmsSec below reduce to IpkTank·N / IrmsTank·N — the previous single-output ratings.
    const size_t nOut = d.outputs.size();
    // KNOWN LIMITATION (ABT #86): the multi-output DAB DESIGN (per-rail turns ratios) and the analytical
    // per-winding excitations (fed to analytical_dab below) are accurate and mirror MKF. But the emitted
    // OPEN-LOOP ngspice deck regulates only the PRIMARY rail: an SPS DAB has a single control degree of
    // freedom (the outer phase shift D3) for one output, so with a shared series tank + one D3 the auxiliary
    // rails cannot be independently regulated (a real multi-output DAB needs per-output CLOSED-LOOP phase
    // control — DPS/TPS). This mirrors MKF, which treats multi-output DAB as analytical-only and never
    // SPICE-validates it. Warn loudly (once) rather than silently ship a deck whose aux rails sit at 0 V.
    if (nOut > 1) {
        static thread_local bool warned = false;
        if (!warned) {
            std::cerr << "[Kirchhoff::DAB] multi-output configuration (" << nOut << " rails): the design + "
                         "analytical winding excitations are accurate, but the emitted OPEN-LOOP ngspice deck "
                         "regulates only the primary rail. Auxiliary rails require closed-loop per-output phase "
                         "control (SPS has one D3 for one output) and will not settle to spec in an open-loop "
                         "transient. See ABT #86." << std::endl;
            warned = true;
        }
    }
    double total_g = 0.0;
    for (const auto& leg : d.outputs) {
        const double Io_i = leg.power / leg.voltage;
        if (leg.voltage > 0.0 && Io_i > 0.0) total_g += Io_i / leg.voltage;
    }
    if (total_g <= 0.0) total_g = 1.0;

    // Transformer (T1) EMBEDDED excitations from the SINGLE FHA source — the SPICE-validated analytical DAB
    // solver. It emits (1 + nOut) windings (primary + one per rail), matching T1's turnsRatios = [N_0, …].
    // The series inductor Lr (when discrete) and the switch ratings keep the inline tank stresses above.
    namespace AN = Kirchhoff::analytical;
    std::vector<double> Vouts, Iouts, Ns;
    for (const auto& leg : d.outputs) {
        Vouts.push_back(leg.voltage);
        Iouts.push_back(leg.power / leg.voltage);
        Ns.push_back(leg.turnsRatio);
    }
    const MAS::OperatingPoint aopT1 = AN::analytical_dab(d.inputVoltage, Vouts, Iouts, Ns,
                                                         fsw, Lm, Lr_H, d.innerPhaseShift1Deg,
                                                         d.innerPhaseShift2Deg, d.phaseShiftDeg);
    const std::vector<json> windings = AN::excitations_processed(aopT1, "T1");
    // turnsRatios (secondary-side only, primary implicit) = [N_0, N_1, …]; isolationSides carries the
    // explicit primary too (length = turnsRatios.size()+1). Each rail is its own galvanic side.
    std::vector<double> turnsRatios;
    std::vector<std::string> isoSides{"primary"};
    for (size_t i = 0; i < nOut; ++i) {
        turnsRatios.push_back(d.outputs[i].turnsRatio);
        isoSides.push_back(req::isolation_side(1 + i));   // sec0->secondary, sec1->tertiary, …
    }
    json xfmr; xfmr["magnetic"] = json::object();
    // NB: DAB keeps its DESIGNED (nominal) Lm — unlike the other transformers it ties Lm to the series
    // inductor (Lm = max(.., 10*Lr)) for the magnetizing-ripple/ZVS, so maximising Lm (lmIsMinimum)
    // detunes its operating point and it stops regulating (verified). abt #56.
    xfmr["inputs"] = req::magnetic_inputs(Lm, 0.1, turnsRatios, isoSides, std::nullopt, 25.0, windings);
    // useLeakageInductance (ABT #95): fold Lr into the transformer leakage — set L_leak == Lr so the emitter
    // recovers K = sqrt(1 − L_leak/Lm) (same L_leak = Lm·(1−K²) relation every KH coupled magnetic uses; cf.
    // Llc/Cuk). With this the coupling itself provides the tank inductance and NO discrete Lr is emitted.
    if (d.useLeakageInductance)
        xfmr["inputs"]["designRequirements"]["leakageInductance"] = json::array({ json{{"nominal", Lr_H}} });

    // Per-switch R∥C across every switch's two power terminals (8 switches) — but the R and C play DIFFERENT
    // roles, so they're named (and handled) differently:
    //   Csn* = a dV/dt SNUBBER: tames the floating-midpoint dV/dt at each switching/dead-time event. C =
    //          energy-budget rule at V_block=Vin. The deck's real-fidelity path STRIPS it (a real switch's
    //          Coss does this physically) — hence the "Csn" snubber name.
    //   Rbias* = a FUNCTIONAL bias resistor: the DC path that DEFINES the floating midpoint during the dead
    //          time. R = bias-loss rule (<= biasLossFrac of rated P at Vin); a small R would bleed Vin²/R
    //          (~kW at 800 V), starving Vout. A switch's Coss does NOT provide a DC bias path, so this is
    //          NOT a snubber — named "Rbias" precisely so the snubber strip never removes it.
    const double snubCval = cfg::snubber_cap(d.config, d.outputPower, d.inputVoltage, d.switchingFrequency);
    const double snubRval = cfg::bias_res(d.config, d.inputVoltage, d.outputPower);
    auto biasR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = snubRval;
        dr["powerRating"] = d.inputVoltage * d.inputVoltage / snubRval;  // bias bleed: P = V^2/R
        dr["role"] = "bleed"; return c; };
    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = snubCval;
        c["inputs"]["designRequirements"]["ratedVoltage"] = (d.inputVoltage + d.outputVoltage) * 3;
        cfg::mark_numerical_aid(c);   // dV/dt convergence aid — tagged for the real-fidelity strip (ABT #96)
        return c; };

    // REAL series-RC EMI/ring snubber across the primary bridge output (midA↔midC, i.e. across Lr + the
    // transformer primary). A genuine board part (sourced + rendered), distinct from the numerical, stripped
    // Csn* per-switch dV/dt caps and the FUNCTIONAL Rbias midpoint-bias resistors. Sized from the same energy
    // budget as the Csn caps (cfg::snubber_cap at V_block=Vin) + cfg::snubber_res, so it stores « throughput
    // and does not detune the phase-shift power transfer. REAL refdes (Crc_pri/Rrc_pri) -> not stripped.
    const auto rcSnub = req::snubber(snubCval, cfg::snubber_res(d.config), d.inputVoltage, d.switchingFrequency);
    const json& rcCap = rcSnub.first;
    const json& rcRes = rcSnub.second;

    json cell; cell["name"] = "dab-cell";
    // string-keyed variants of the const-char* helpers, for the per-rail suffixed component/port names.
    auto pinS  = [](const std::string& c, const std::string& p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto portS = [](const std::string& n) { json p; p["name"] = n; return p; };
    auto compS = [](const std::string& n, json data) { json c; c["name"] = n; c["data"] = data; return c; };
    auto connS = [](const std::string& n, std::vector<json> eps) { json c; c["name"] = n; c["endpoints"] = eps; return c; };
    auto mosfetPri = [&]() { return mosfet(req::mosfet("mainSwitch", ratedVdsPri, IpkTank, maxRdsOnPri, 125.0)); };

    // Ports: primary vin/gnd/vout + primary gates; each EXTRA rail adds its own vout<i> + gateE..H<i>.
    std::vector<json> ports{port("vin"), port("gnd"), port("vout"),
                            port("gateA"), port("gateB"), port("gateC"), port("gateD")};
    // Components in the single-output order: primary bridge, per-rail secondary bridges, Lr(if discrete)+T1,
    // per-rail Cout, primary snubbers, per-rail secondary snubbers, real RC snubber. For nOut==1 the per-rail
    // blocks emit exactly QE..QH/DE..DH/Cout/RbiasE.. with no suffix — byte-identical to the old deck.
    std::vector<json> comps{
        comp("QA", mosfetPri()), comp("QB", mosfetPri()), comp("QC", mosfetPri()), comp("QD", mosfetPri()),
        comp("DA", diode()),  comp("DB", diode()),  comp("DC", diode()),  comp("DD", diode())};
    std::vector<json> secSwitchComps, secCapComps, secSnubComps, secConns, secGateConns, gndSecEps, secStim;
    std::vector<json> gatePorts;

    for (size_t i = 0; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const std::string s = (i == 0) ? std::string() : std::to_string(i + 1);   // suffix "", "2", "3", …
        const double Io_i  = leg.power / leg.voltage;
        const double share = (leg.voltage > 0.0 && Io_i > 0.0) ? (Io_i / leg.voltage) / total_g
                                                               : (i == 0 ? 1.0 : 0.0);
        // Per-rail bridge stresses: reflected + load-shared tank current, blocking this rail's own Vout.
        const double IpkSec_i  = IpkTank  * leg.turnsRatio * share;
        const double IrmsSec_i = IrmsTank * leg.turnsRatio * share;
        const double ratedVdsSec_i = leg.voltage / cfg::v_derate_mosfet(d.config);
        if (!(IrmsSec_i > 0.0))
            throw std::invalid_argument("build_dab_tas: secondary rail " + std::to_string(i) +
                                        " carries no current — cannot size its bridge (power must be > 0)");
        const double maxRdsOnSec_i = cfg::rds_on_loss_fraction(d.config) * leg.power / (IrmsSec_i * IrmsSec_i);
        const json mreq = req::mosfet("mainSwitch", ratedVdsSec_i, IpkSec_i, maxRdsOnSec_i, 125.0);
        auto mkSec = [&]() { return mosfet(mreq); };

        // per-rail component names
        const std::string qE = "QE" + s, qF = "QF" + s, qG = "QG" + s, qH = "QH" + s;
        const std::string dE = "DE" + s, dF = "DF" + s, dG = "DG" + s, dH = "DH" + s;
        const std::string rEh = "RbiasE_hi" + s, cEh = "CsnE_hi" + s, rEl = "RbiasE_lo" + s, cEl = "CsnE_lo" + s;
        const std::string rGh = "RbiasG_hi" + s, cGh = "CsnG_hi" + s, rGl = "RbiasG_lo" + s, cGl = "CsnG_lo" + s;
        const std::string coutN = "Cout" + s;
        const std::string secStart = "secondary" + std::to_string(i + 1) + "_start";
        const std::string secEnd   = "secondary" + std::to_string(i + 1) + "_end";
        const std::string voutPort = "vout" + s;
        const std::string gE = "gateE" + s, gF = "gateF" + s, gG = "gateG" + s, gH = "gateH" + s;

        secSwitchComps.push_back(compS(qE, mkSec())); secSwitchComps.push_back(compS(qF, mkSec()));
        secSwitchComps.push_back(compS(qG, mkSec())); secSwitchComps.push_back(compS(qH, mkSec()));
        // body diodes of the SECONDARY FETs — mirror their host (block the secondary Vds, carry IpkSec),
        // NOT the primary rating a bare diode() would default to (a 60 V FET has no 400 V body diode).
        const json secBody = req::body_diode(ratedVdsSec_i, IpkSec_i);
        secSwitchComps.push_back(compS(dE, diode(secBody))); secSwitchComps.push_back(compS(dF, diode(secBody)));
        secSwitchComps.push_back(compS(dG, diode(secBody))); secSwitchComps.push_back(compS(dH, diode(secBody)));

        json capi; capi["capacitor"] = json::object();
        capi["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.outputCapacitance;
        capi["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;
        secCapComps.push_back(compS(coutN, capi));

        secSnubComps.push_back(compS(rEh, biasR())); secSnubComps.push_back(compS(cEh, snubC()));
        secSnubComps.push_back(compS(rEl, biasR())); secSnubComps.push_back(compS(cEl, snubC()));
        secSnubComps.push_back(compS(rGh, biasR())); secSnubComps.push_back(compS(cGh, snubC()));
        secSnubComps.push_back(compS(rGl, biasR())); secSnubComps.push_back(compS(cGl, snubC()));

        // ── Secondary active bridge for this rail. QE/QG high-side (vout->sec), QF/QH low-side (sec->gnd);
        // body diodes freewheel during the leg dead time. The D3 phase shift vs the primary sets the power.
        secConns.push_back(connS("sec_a" + s, {pinS("T1", secStart), pinS(qE, "source"), pinS(qF, "drain"),
                          pinS(dE, "anode"), pinS(dF, "cathode"),
                          pinS(rEh, "2"), pinS(cEh, "2"), pinS(rEl, "1"), pinS(cEl, "1")}));
        secConns.push_back(connS("sec_b" + s, {pinS("T1", secEnd), pinS(qG, "source"), pinS(qH, "drain"),
                          pinS(dG, "anode"), pinS(dH, "cathode"),
                          pinS(rGh, "2"), pinS(cGh, "2"), pinS(rGl, "1"), pinS(cGl, "1")}));
        secConns.push_back(connS((i == 0 ? std::string("vout_net") : voutPort + "_net"),
                         {pinS(qE, "drain"), pinS(qG, "drain"), pinS(dE, "cathode"), pinS(dG, "cathode"),
                          pinS(coutN, "1"), pinS(rEh, "1"), pinS(cEh, "1"), pinS(rGh, "1"), pinS(cGh, "1"),
                          prt((i == 0 ? "vout" : voutPort.c_str()))}));

        // this rail's low-side sources/anodes + cap return + low snubber returns join the shared ground.
        for (const auto& e : { pinS(qF, "source"), pinS(qH, "source"), pinS(dF, "anode"), pinS(dH, "anode"),
                               pinS(coutN, "2"), pinS(rEl, "2"), pinS(cEl, "2"), pinS(rGl, "2"), pinS(cGl, "2") })
            gndSecEps.push_back(e);

        gatePorts.push_back(portS(gE)); gatePorts.push_back(portS(gF));
        gatePorts.push_back(portS(gG)); gatePorts.push_back(portS(gH));
        secGateConns.push_back(connS("gateE_net" + s, {pinS(qE, "gate"), prt(gE.c_str())}));
        secGateConns.push_back(connS("gateF_net" + s, {pinS(qF, "gate"), prt(gF.c_str())}));
        secGateConns.push_back(connS("gateG_net" + s, {pinS(qG, "gate"), prt(gG.c_str())}));
        secGateConns.push_back(connS("gateH_net" + s, {pinS(qH, "gate"), prt(gH.c_str())}));

        // Per-rail secondary drive: all rails share the outer shift D3 (+ inner D2). QE/QF leg vs QG/QH leg.
        const double p3 = d.phaseShiftDeg, p2 = d.innerPhaseShift2Deg;
        auto stimR = [&](const std::string& sw, double phaseDeg) {
            json st; st["stage"] = "dabCell"; st["component"] = sw; st["signal"] = "gate";
            st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
            st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg; return st; };
        secStim.push_back(stimR(qE, p3 + p2));  secStim.push_back(stimR(qF, 180.0 + p3 + p2));
        secStim.push_back(stimR(qG, 180.0 + p3)); secStim.push_back(stimR(qH, p3));
    }

    // Assemble components in the byte-identical single-output order.
    for (auto& c : secSwitchComps) comps.push_back(std::move(c));
    if (!d.useLeakageInductance) comps.push_back(comp("Lr", lr));   // discrete Lr only when not folded into leakage
    comps.push_back(comp("T1", xfmr));
    for (auto& c : secCapComps) comps.push_back(std::move(c));
    // primary per-switch RC snubbers (A/C legs)
    for (auto& c : { comp("RbiasA_hi", biasR()), comp("CsnA_hi", snubC()), comp("RbiasA_lo", biasR()), comp("CsnA_lo", snubC()),
                     comp("RbiasC_hi", biasR()), comp("CsnC_hi", snubC()), comp("RbiasC_lo", biasR()), comp("CsnC_lo", snubC()) })
        comps.push_back(c);
    for (auto& c : secSnubComps) comps.push_back(std::move(c));
    comps.push_back(comp("Crc_pri", rcCap)); comps.push_back(comp("Rrc_pri", rcRes));

    for (auto& p : gatePorts) ports.push_back(std::move(p));
    cell["ports"] = ports;
    cell["components"] = comps;

    // Primary bridge connections. The transformer primary_start ties to Lr's far end via pri_x when Lr is a
    // discrete inductor; when Lr is folded into the leakage it lands directly on midA (no series inductor).
    std::vector<json> conns;
    conns.push_back(conn("vin_net",  {pin("QA", "drain"), pin("QC", "drain"),
                          pin("DA", "cathode"), pin("DC", "cathode"),
                          pin("RbiasA_hi", "1"), pin("CsnA_hi", "1"),
                          pin("RbiasC_hi", "1"), pin("CsnC_hi", "1"), prt("vin")}));
    conns.push_back(conn("midA_net", {pin("QA", "source"), pin("QB", "drain"),
                          pin("DA", "anode"), pin("DB", "cathode"),
                          (d.useLeakageInductance ? pin("T1", "primary_start") : pin("Lr", "primary_start")),
                          pin("RbiasA_hi", "2"), pin("CsnA_hi", "2"),
                          pin("RbiasA_lo", "1"), pin("CsnA_lo", "1"), pin("Crc_pri", "1")}));
    conns.push_back(conn("midC_net", {pin("QC", "source"), pin("QD", "drain"),
                          pin("DC", "anode"), pin("DD", "cathode"), pin("T1", "primary_end"),
                          pin("RbiasC_hi", "2"), pin("CsnC_hi", "2"),
                          pin("RbiasC_lo", "1"), pin("CsnC_lo", "1"), pin("Rrc_pri", "2")}));
    conns.push_back(conn("rc_pri_mid", {pin("Crc_pri", "2"), pin("Rrc_pri", "1")}));
    if (!d.useLeakageInductance)
        conns.push_back(conn("pri_x", {pin("Lr", "primary_end"), pin("T1", "primary_start")}));
    for (auto& c : secConns) conns.push_back(std::move(c));

    // Shared ground: primary low-side sources/anodes + low snubber returns, then every rail's ground pins.
    std::vector<json> gndEps{pin("QB", "source"), pin("QD", "source"), pin("DB", "anode"), pin("DD", "anode"),
                             pin("RbiasA_lo", "2"), pin("CsnA_lo", "2"), pin("RbiasC_lo", "2"), pin("CsnC_lo", "2")};
    for (auto& e : gndSecEps) gndEps.push_back(std::move(e));
    gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));

    conns.push_back(conn("gateA_net", {pin("QA", "gate"), prt("gateA")}));
    conns.push_back(conn("gateB_net", {pin("QB", "gate"), prt("gateB")}));
    conns.push_back(conn("gateC_net", {pin("QC", "gate"), prt("gateC")}));
    conns.push_back(conn("gateD_net", {pin("QD", "gate"), prt("gateD")}));
    for (auto& c : secGateConns) conns.push_back(std::move(c));
    cell["connections"] = conns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    // Per-rail output requirements + operating-point powers (the assembler synthesizes one load per external
    // output port). Rail 0 keeps name "out"/port "vout" (single-output byte-identical); rails i>0 -> "out<i+1>".
    dreq["outputs"] = json::array();
    json opDoc; opDoc["name"] = "full_load"; opDoc["inputVoltage"] = d.inputVoltage; opDoc["ambientTemperature"] = 25.0;
    opDoc["outputs"] = json::array();
    for (size_t i = 0; i < nOut; ++i) {
        const std::string oname = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        json o; o["name"] = oname; o["voltage"]["nominal"] = d.outputs[i].voltage; o["regulation"] = "voltage";
        dreq["outputs"].push_back(o);
        json oo; oo["name"] = oname; oo["power"] = d.outputs[i].power; opDoc["outputs"].push_back(oo);
    }
    tas["inputs"]["operatingPoints"] = json::array({opDoc});

    tas["topology"]["stages"] = json::array({
        req::control_stage("pwmController"),
        req::control_stage("gateDriver", "gate-driver", "UDR"),
        pstage("dabCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("dabCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("dabCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("dabCell", "vout")})};
    for (size_t i = 1; i < nOut; ++i) {
        const std::string g = "Vout" + std::to_string(i + 1), pt = "vout" + std::to_string(i + 1);
        iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("dabCell", pt.c_str())}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Eight PWM drives. Each half-bridge leg is 50 %-duty; the bridge output is the difference of its two
    // legs. Modulation (degrees):
    //   Primary   Vab: leg A (QA top / QB bot) shifted by D1 vs leg C (QC/QD). D1=0 → ±Vin square wave.
    //   Secondary Vcd: whole bridge shifted by D3 vs the primary; leg E (QE/QF) shifted by D2 vs leg G (QG/QH).
    // This reproduces analytical_dab's dab_Vab_at(θ,V1,D1) / dab_Vcd_at(θ,V2,D2,D3) exactly: shifting leg A's
    // turn-on to D1 makes Vab = Va−Vc give a zero plateau on [0,D1) then +Vin on [D1,π) (and mirror), and the
    // same for the secondary with D2/D3. D1=D2=0 collapses to the original SPS drive.
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "dabCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    const double p1 = d.innerPhaseShift1Deg;
    // Primary drives (QA..QD) once; every rail's secondary drives were built alongside its bridge (secStim).
    std::vector<json> stimuli{stim("QA", p1), stim("QB", 180.0 + p1), stim("QC", 180.0), stim("QD", 0.0)};
    for (auto& s : secStim) stimuli.push_back(std::move(s));
    tas["simulation"]["stimulus"] = stimuli;
    req::finalize_control_seeds(tas, Topology::DUAL_ACTIVE_BRIDGE_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
