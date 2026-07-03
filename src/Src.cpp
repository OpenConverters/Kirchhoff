#include "Src.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kBridgeFactor  = 0.5;   // half-bridge: Vo_fha = 0.5·Vin
constexpr double kQualityFactor = 0.8;   // broader, lower-loss tank (was 2.0: a sharp high-Q tank carries
                                         // large circulating current -> heavy loss-driven Vout sag, abt #62)
constexpr double kLmRatio       = 10.0;  // Lm = 10·Lr (large, keeps Lm out of the resonance)
constexpr double kGainHeadroom  = 1.08;  // size n so the fr peak delivers 1.08·Vo, giving the regulator
                                         // room to hit Vo just ABOVE fr (the SRC tank only steps DOWN)
constexpr double kSwitchDuty    = 0.45;
} // namespace

SrcDesign design_src(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    SrcDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.outputVoltage = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
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
    const double vinMax = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MAXIMUM);
    const double vinMin = PEAS::resolve_dimensional_values(iv, PEAS::DimensionalValues::MINIMUM);
    d.inputVoltageMin = vinMin;
    d.inputVoltageMax = vinMax;

    const double Vin = d.inputVoltage, Vo = d.outputVoltage;
    const double Iout = d.outputPower / Vo;

    // Turns ratio: n = k_bridge·Vin/(Vo+Vd). The SRC tank gain PEAKS at unity at fr (it can only step
    // DOWN off resonance — there is no parallel Lm to boost like the LLC), so the realized output at fr is
    // 0.5·Vin/n MINUS the rectifier drop Vd. Without the +Vd compensation the converter tops out at Vo−Vd
    // and can NEVER regulate to Vo (at 12 V the 0.85 V CT-rectifier drop is ~7% — the closed-loop search
    // lands ~11.1 V and reports regulated=False). Compensating Vd puts the fr peak at Vo so the regulator
    // can hit target just off resonance. (Mirrors the LLC n; Vd is negligible at 48 V, ~7% at 12 V.)
    // Rectifier variant (CT default, matching MKF Src + the fixture). SRC has no voltage-doubler.
    d.rectifierType = parse_rectifier_type(cfg::get_str(d.config, "rectifierType", "centerTapped"),
                                           RectifierType::CenterTapped);
    // Primary bridge topology (MKF SrcBridgeType HALF_BRIDGE | FULL_BRIDGE; the FULL_BRIDGE_PHASE_SHIFT
    // variant is a P2 nuance not modelled here). Half-bridge (default, matching the MKF Src fixture) drives
    // the tank at ±Vin/2 (bridge factor 0.5); full-bridge at ±Vin (1.0), so n doubles to hold spec (ABT #91).
    d.fullBridge = cfg::full_bridge_selected(d.config);
    const double Vd = req::dideal_diode_drop(Iout);
    // Gain headroom: the SRC tank peaks at M=1 at fr and can only step DOWN. Sizing n for the fr peak to
    // deliver gainHeadroom·(Vo+Vd) lets the regulator hit Vo just ABOVE fr (efficient, monotonic) instead of
    // pinning the nominal point at the M=1 peak where any loss sags Vout below target (abt #62). Per variant:
    // CT one diode drop (Vo+Vd); FB two (Vo+2Vd); CD delivers ~0.465·Vsec (cdOutputFactor). All keep the
    // headroom divisor.
    const double kBridge = d.fullBridge ? 1.0 : kBridgeFactor;   // half-bridge ±Vin/2 (0.5) | full-bridge ±Vin (1.0)
    const double Vbridge = cfg::get(d.config, "bridgeFactor", kBridge) * Vin;
    const double Ghr = cfg::get(d.config, "gainHeadroom", kGainHeadroom);
    double n;
    switch (d.rectifierType) {
        case RectifierType::CenterTapped:   n = Vbridge / (Ghr * (Vo + Vd));       break;
        case RectifierType::FullBridge:     n = Vbridge / (Ghr * (Vo + 2.0 * Vd)); break;
        case RectifierType::CurrentDoubler:
            n = cfg::get(d.config, "cdOutputFactor", 0.465) * Vbridge / (Ghr * (Vo + Vd)); break;
        case RectifierType::VoltageDoubler:
            throw std::runtime_error("Kirchhoff SRC: voltageDoubler rectifier not supported "
                                     "(SRC variants: centerTapped, fullBridge, currentDoubler)");
    }
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(std::round(n * 100.0) / 100.0);
    // Two-element series tank (no resonant Lm): Rac = 8n²/π²·Rload, Zr = Q·Rac, operate at fr = fsw,
    // Lr = Zr/(2π·fr), Cr = 1/(2π·fr·Zr). Lm made large (10·Lr) so it does not load the resonance.
    const double Rload = Vo / Iout;
    const double Rac = (8.0 * n * n) / (M_PI * M_PI) * Rload;
    const double fr = d.switchingFrequency;   // designed/operated at series resonance
    const double Zr = cfg::get(d.config, "qualityFactor", kQualityFactor) * Rac;
    d.resonantFrequency = fr;
    d.resonantInductance = Zr / (2.0 * M_PI * fr);
    d.resonantCapacitance = 1.0 / (2.0 * M_PI * fr * Zr);
    d.magnetizingInductance = req::provided_inductance(dr).value_or(
        cfg::get(d.config, "inductanceRatio", kLmRatio) * d.resonantInductance);

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 47e-6;
    // CURRENT_DOUBLER output inductors Lo1/Lo2 (each carries Iout/2, ripple cancels at 2·fr). Unused by
    // the other variants. Lo = Vout/(4·fr·ΔI·Iout).
    d.outputInductance = d.outputVoltage /
        (4.0 * d.resonantFrequency * cfg::ripple_ratio(d.config, 0.30) * Iout);

    // ── Per-output legs (multi-output: N isolated secondaries, ABT #86) ──
    // Every rail hangs off the SAME series Lr–Cr tank + transformer primary. The per-rail turns ratio uses
    // the identical rectifier-variant formula as the main rail (with that rail's Vout_i, Vd_i), so
    // n_i·(Vout_i+…Vd_i) = Vbridge/Ghr is EQUAL across rails — i.e. every secondary reflects to the same
    // primary clamp and conducts together (the coupled-winding condition that makes multi-output work).
    // outputs[0] reproduces the scalars above byte-for-byte.
    auto nForRail = [&](double Vo_i, double Vd_i) -> double {
        switch (d.rectifierType) {
            case RectifierType::CenterTapped:   return Vbridge / (Ghr * (Vo_i + Vd_i));
            case RectifierType::FullBridge:     return Vbridge / (Ghr * (Vo_i + 2.0 * Vd_i));
            case RectifierType::CurrentDoubler:
                return cfg::get(d.config, "cdOutputFactor", 0.465) * Vbridge / (Ghr * (Vo_i + Vd_i));
            case RectifierType::VoltageDoubler:
                throw std::runtime_error("Kirchhoff SRC: voltageDoubler rectifier not supported");
        }
        throw std::runtime_error("Kirchhoff SRC: unreachable rectifier type");
    };
    const int wpo = rectifier_windings_per_output(d.rectifierType);   // 2 (CT) | 1 (FB/CD)
    const size_t nOut = dr.at("outputs").size();
    for (size_t i = 0; i < nOut; ++i) {
        SrcOutputLeg leg{};
        leg.voltage = nominal(dr.at("outputs").at(i).at("voltage"));
        if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
            leg.power = tasInputs.at("operatingPoints").at(0).at("outputs").at(i).at("power").get<double>();
        else
            leg.power = nominal(dr.at("outputs").at(i).at("power"));
        const double iout_i = leg.power / leg.voltage;
        leg.diodeDrop = req::dideal_diode_drop(iout_i);
        if (i == 0) {
            leg.turnsRatio = d.turnsRatio;               // preserve the main rail's exact scalar
            leg.outputCapacitance = d.outputCapacitance;
            leg.outputInductance = d.outputInductance;
        } else {
            const double ni = std::round(nForRail(leg.voltage, leg.diodeDrop) * 100.0) / 100.0;
            leg.turnsRatio = req::provided_turns_ratio(dr, wpo * i).value_or(ni);
            leg.outputCapacitance = 47e-6;
            leg.outputInductance = leg.voltage /
                (4.0 * d.resonantFrequency * cfg::ripple_ratio(d.config, 0.30) * iout_i);
        }
        leg.loadResistance = leg.voltage / iout_i;
        d.outputs.push_back(leg);
    }

    // Shared tank re-sizing for N reflected loads (ABT #86): each rail reflects an AC-equivalent
    // Rac_i = 8·n_i²/π²·Rload_i to the primary; the tank sees them in PARALLEL, so size the characteristic
    // impedance against the parallel combination Rac_total = 1/Σ(1/Rac_i), keeping the design Q relative to
    // the TOTAL load (Zr = Q·Rac_total). fr is unchanged (Lr·Cr fixed), so the main rail still resonates at
    // fsw. Guarded on nOut>1 so the single-output tank (Zr = Q·Rac above) stays byte-identical.
    if (nOut > 1) {
        double gac = 0.0;   // Σ 1/Rac_i  (parallel reflected conductance)
        for (const auto& leg : d.outputs)
            gac += (M_PI * M_PI) / (8.0 * leg.turnsRatio * leg.turnsRatio * leg.loadResistance);
        const double RacTotal = 1.0 / gac;
        const double ZrTotal = cfg::get(d.config, "qualityFactor", kQualityFactor) * RacTotal;
        d.resonantInductance = ZrTotal / (2.0 * M_PI * fr);
        d.resonantCapacitance = 1.0 / (2.0 * M_PI * fr * ZrTotal);
        d.magnetizingInductance = req::provided_inductance(dr).value_or(
            cfg::get(d.config, "inductanceRatio", kLmRatio) * d.resonantInductance);
    }
    return d;
}

json build_src_tas(const SrcDesign& d) {
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
    // Bare seeds (no designRequirements). Body diodes (anti-parallel to a FET) use these as-is — the HS
    // fill DEFERS a requirement-less diode as a FET body diode. REAL switches/rectifiers take a `req`.
    auto diode  = [&]() { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage); return j; };
    auto mosfetReq = [](const json& r) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };
    auto diodeReq  = [](const json& r) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };

    const double n = d.turnsRatio;

    // --- resonant-tank stresses (FHA, evaluated at the nominal point, operated AT series resonance fr) ---
    // The SRC tank is SINUSOIDAL at fr, so every magnetic excitation is a sine (label "sinusoidal",
    // vRms=vPk/√2, vPkPk=2·vPk, offset 0). The bridge presents a ±Vdrive square (Vdrive = Vin/2 half-bridge,
    // Vin full-bridge — ABT #91; fund. rms 2√2·Vdrive/π). Unlike LLC the magnetizing Lm is large (10·Lr) and
    // kept out of the resonance, so the tank current is essentially the real load current (Pin/Vtank1_rms);
    // the small magnetizing component is added too.
    const double fr   = d.resonantFrequency, Tfr = 1.0 / fr;
    const double Vdrive = d.fullBridge ? d.inputVoltage : d.inputVoltage / 2.0;   // bridge-leg amplitude
    const double Pin  = d.outputPower / d.efficiency;
    const double Iout = d.outputPower / d.outputVoltage;
    const double Vtank1Rms = 2.0 * std::sqrt(2.0) * Vdrive / M_PI;       // fund. rms of the ±Vdrive square
    const double IloadRms  = Pin / Vtank1Rms;                            // real (in-phase) tank current
    const double ImagPk    = Vdrive * (Tfr / 4.0) / d.magnetizingInductance;  // Lm triangle pk
    const double ImagRms   = ImagPk / std::sqrt(3.0);
    const double ItankRms  = std::sqrt(IloadRms * IloadRms + ImagRms * ImagRms);  // primary winding current
    const double ItankPk   = std::sqrt(2.0) * ItankRms;                 // sinusoidal peak
    const double ItankPkPk = 2.0 * ItankPk;
    const int wpo = rectifier_windings_per_output(d.rectifierType);  // 2 (CT) | 1 (FB/CD)
    // Secondary rectifier current peak: a rectified half-sine of the reflected tank current, pk=(π/2)·Iout
    // (same for every rectifier variant; the diode reverse-block level and winding count differ elsewhere).
    const double IsecPk = (M_PI / 2.0) * Iout;
    // Resonant-inductor winding voltage (sinusoidal at fr): i·Zr, Zr=2π·fr·Lr. The TRANSFORMER winding
    // voltages come from the embedded analytical-SRC excitations below, not an inline reflected-clamp calc.
    const double Zr     = 2.0 * M_PI * fr * d.resonantInductance;
    const double vLrPk  = ItankPk * Zr, vLrRms = vLrPk / std::sqrt(2.0), vLrPkPk = 2.0 * vLrPk;

    // --- semiconductor requirements (sourceable) ---
    // Primary half-bridge MOSFETs Q1/Q2 each block the full bus Vin when off and carry the resonant
    // tank current (peak ItankPk, rms ItankRms — the same primary current that drives the magnetic).
    const double ratedVdsQ = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnQ  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (ItankRms * ItankRms);
    const json reqQ = req::mosfet("mainSwitch", ratedVdsQ, ItankPk, maxRdsOnQ, 125.0);
    // Center-tapped rectifier diodes D1/D2: each non-conducting half is reverse-biased to ~2·Vout, and
    // conducts a rectified half-sine of the reflected tank current (peak IsecPk, avg current Iout).
    // NOT a body diode — an independent output rectifier.
    const double ratedVrD = 2.0 * d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxVfD    = (ratedVrD < 100.0) ? 0.6 : 1.2;
    const json reqD = req::diode(ratedVrD, IsecPk, maxVfD, 0.05 * Tfr);

    json cr; cr["capacitor"] = json::object();
    cr["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.resonantCapacitance;
    cr["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2;
    // RESONANT cap: it sets the series-tank frequency, so it must be sourced CLOSE to nominal. The default
    // fill treats capacitance as a ripple MINIMUM and oversizes up to 2x (and may pick a lossy electrolytic),
    // which detunes the SRC tank — fr drops below fsw, so the converter is dragged far below resonance with
    // large circulating current (poor efficiency). role=resonant picks the NEAREST film value (abt #54, as
    // the LLC Cr already does). NOTE: unlike LLC/CLLC the SRC magnetizing Lm is deliberately kept LARGE
    // (10·Lr, out of the resonance), so a pinned Lm does NOT detune the Lr–Cr tank — no Lr/Cr re-sizing here.
    cr["inputs"]["designRequirements"]["role"] = "resonant";

    // Resonant inductor Lr: its OWN single-winding magnetic (carries the full sinusoidal tank current).
    json lr; lr["magnetic"] = json::object();
    lr["inputs"] = req::magnetic_inputs(d.resonantInductance, 0.2, /*single winding*/ {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vLrPk, vLrRms, 0.0, vLrPkPk)});

    // Transformer: primary Lpri = Lm; CT has TWO secondary half-windings per output (turnsRatios=[n,n,…]),
    // FB/CD ONE ([n,…]). K=0.999 (MKF). windings = primary + wpo·nOut secondaries. Multi-output (ABT #86):
    // each rail's windings reuse the same isolationSide ordinal (CT halves share a centre tap); rail i gets
    // isolation_side(1+i). Single-output produces exactly [n]/[n,n] + [primary,secondary(,secondary)].
    const size_t nOut = d.outputs.size();
    std::vector<std::string> isoSides{"primary"};
    std::vector<double> turnsRatios;
    for (size_t i = 0; i < nOut; ++i) {
        const std::string side = req::isolation_side(1 + i);
        for (int w = 0; w < wpo; ++w) { isoSides.push_back(side); turnsRatios.push_back(d.outputs[i].turnsRatio); }
    }
    // Transformer (T1) EMBEDDED excitations from the SINGLE FHA source (the SPICE-validated analytical SRC
    // solver). bridgeVoltageFactor = 0.5 half-bridge | 1.0 full-bridge (ABT #91; analytical_src's own default
    // is the full-bridge 1.0); turnsRatios/V/I are PER OUTPUT (CENTER_TAPPED splits each into 2 half-windings).
    // The resonant inductor Lr and the switch/diode ratings keep the inline tank FHA.
    namespace AN = Kirchhoff::analytical;
    const AN::SrcRectifier rect = (d.rectifierType == RectifierType::CenterTapped)
                                  ? AN::SrcRectifier::CENTER_TAPPED : AN::SrcRectifier::FULL_BRIDGE;
    // CD/VD reflect the winding through an output doubler, so n was sized against the WINDING terminal, not
    // the output terminal. The solver reflects n·Vout_arg, so it must be fed the winding-terminal V/I
    // (VD: Vo/2 & 2·Iout; CD: Vo/cdF & cdF·Iout); the raw output V/I would solve the embedded tank/windings
    // at 2×/~0.47× the real reflected operating point. FB/CT feed the winding directly. Per rail.
    std::vector<double> vEmbeds, iEmbeds, nEmbeds;
    for (const auto& leg : d.outputs) {
        const double iout_i = leg.power / leg.voltage;
        double vE = leg.voltage, iE = iout_i;
        if (d.rectifierType == RectifierType::VoltageDoubler) { vE = leg.voltage / 2.0; iE = 2.0 * iout_i; }
        else if (d.rectifierType == RectifierType::CurrentDoubler) {
            const double cdF = cfg::get(d.config, "cdOutputFactor", 0.465);
            vE = leg.voltage / cdF; iE = cdF * iout_i;
        }
        vEmbeds.push_back(vE); iEmbeds.push_back(iE); nEmbeds.push_back(leg.turnsRatio);
    }
    const MAS::OperatingPoint aopT1 = AN::analytical_src(d.inputVoltage, vEmbeds, iEmbeds, nEmbeds, fr,
                                                         d.resonantInductance, d.resonantCapacitance,
                                                         d.fullBridge ? 1.0 : 0.5, rect);
    const std::vector<json> windings = AN::excitations_processed(aopT1, "T1");
    json t1; t1["magnetic"] = json::object();
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, turnsRatios, isoSides,
        std::nullopt, 25.0, windings);
    { const double kCpl = cfg::get(d.config, "transformerCoupling", 0.999);
      t1["inputs"]["designRequirements"]["leakageInductance"] =
          json::array({ json{{"nominal", (1.0 - kCpl*kCpl) * d.magnetizingInductance}} }); }

    auto busCap = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::get(d.config, "busSplitCap", 10e-6);
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2; return c; };

    json cout; cout["capacitor"] = json::object();
    cout["inputs"]["designRequirements"]["capacitance"]["nominal"] = d.outputCapacitance;
    cout["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 2;

    auto snubC = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = cfg::rectifier_snubber_cap(d.config);
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.outputVoltage * 3;
        cfg::mark_numerical_aid(c); return c; };   // rectifier dV/dt aid — tagged for the real strip (ABT #96)
    auto snubR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = cfg::snubber_res(d.config);
        // RC-snubber R dissipates the cap energy each cycle: P = C*V^2*f (V = clamped reverse swing).
        const double vClamp = d.outputVoltage * 3.0;
        dr["powerRating"] = cfg::rectifier_snubber_cap(d.config) * vClamp * vClamp * d.switchingFrequency;
        dr["role"] = "snubber"; cfg::mark_numerical_aid(c); return c; };   // RC-snubber R paired with snubC — tagged (ABT #96)
    // Cap-divider BALANCING resistors — give the bus-split midpoint msplit a DC reference at Vbus/2.
    // Without them msplit (Chi/Clo junction + transformer primary) has no DC path -> singular MNA
    // matrix -> garbage Vout at off-design frequencies and the regulator never converges (abt #54).
    auto balR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = 100000.0;
        dr["powerRating"] = 0.5;
        dr["role"] = "balancing"; return c; };
    // Half-bridge switching-node numerical dV/dt snubber (Csw -> stripped when the FET carries real Coss).
    // The IDEAL switch slews sw_node infinitely fast; at some operating points ngspice can't resolve the
    // edge and aborts ("Timestep too small ... node sw_node"). A small cap to ground mimics Coss so the
    // ideal deck converges; named Csw* so the assembler drops it once a real FET's Coss does the limiting
    // physically, leaving the DATASHEET/MKF_MODEL deck untouched (abt #54).
    auto swSnub = [&]() { json c; c["capacitor"] = json::object();
        c["inputs"]["designRequirements"]["capacitance"]["nominal"] = 100e-12;
        c["inputs"]["designRequirements"]["ratedVoltage"] = d.inputVoltage * 2;
        cfg::mark_numerical_aid(c); return c; };   // sw-node dV/dt aid — tagged for the real strip (ABT #96)
    // CURRENT_DOUBLER output inductor (single-winding magnetic; carries Iout/2 DC with 2·fr ripple).
    auto outInductor = [&]() { json m; m["magnetic"] = json::object();
        const double Idc = Iout / 2.0, Irip = cfg::ripple_ratio(d.config, 0.30) * Idc;
        m["inputs"] = req::magnetic_inputs(d.outputInductance, 0.3, {}, {"primary"}, std::nullopt, 25.0, {
            req::winding_excitation("triangular", 2.0 * fr, Idc + Irip / 2.0, Idc, Idc, Irip, std::nullopt,
                                    d.outputVoltage, d.outputVoltage / std::sqrt(2.0), 0.0, 2.0 * d.outputVoltage)});
        return m; };
    // CURRENT_DOUBLER loop-breaker R: secondary winding + Lo1 + Lo2 form an all-inductor loop (singular
    // at DC). A tiny series resistance (~mΩ) in one output-inductor leg breaks it without dissipating.
    auto loopBreakR = [&]() { json c; c["resistor"] = json::object();
        auto& dr = c["inputs"]["designRequirements"]; dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = cfg::loop_breaker_res(d.config, d.loadResistance);
        dr["powerRating"] = 0.25; dr["role"] = "balancing"; return c; };

    json cell; cell["name"] = "src-cell";
    // Cell ports: vin/gnd/vout/g1/g2 for the main rail; extra isolated rails append a vout<i> port below.
    std::vector<json> cports{port("vin"), port("gnd"), port("vout"), port("g1"), port("g2")};

    // Primary side: series resonant tank + either a split-cap half-bridge or a 4-MOSFET full bridge
    // (config.bridgeType, ABT #91), identical secondary rectifier for both. The half-bridge branch + the CT
    // rectifier reproduce the original validated card ORDER exactly (this split-cap resonant deck is
    // order-sensitive in ngspice, abt #54): Csw1 between Cout and the snubbers; gate nets last.
    std::vector<json> comps;
    std::vector<json> conns;
    std::vector<json> gndPrimary;
    if (!d.fullBridge) {
        // Half-bridge: Q1 (vbus->sw), Q2 (sw->gnd); Dq1/Dq2 body diodes; split caps Chi/Clo set the midpoint
        // msplit (= Vbus/2): sw_node -> Cr -> Lr -> Lpri(=Lm) -> msplit. Csw1 declared with the rectifier below.
        comps = {
            comp("Q1", mosfetReq(reqQ)), comp("Q2", mosfetReq(reqQ)),
            comp("Dq1", diode()), comp("Dq2", diode()),   // Dq1/Dq2 = Q1/Q2 body diodes -> bare seed (deferred)
            comp("Chi", busCap()), comp("Clo", busCap()),
            comp("Rbal_hi", balR()), comp("Rbal_lo", balR()),   // define bus-split midpoint at DC (abt #54)
            comp("Cr", cr), comp("Lr", lr), comp("T1", t1)};
        conns = {
            conn("vin_net",  {pin("Q1", "drain"), pin("Dq1", "cathode"), pin("Chi", "1"),
                              pin("Rbal_hi", "1"), prt("vin")}),
            conn("sw_node",  {pin("Q1", "source"), pin("Q2", "drain"),
                              pin("Dq1", "anode"), pin("Dq2", "cathode"), pin("Cr", "1"), pin("Csw1", "1")}),
            conn("msplit",   {pin("Chi", "2"), pin("Clo", "1"), pin("T1", "primary_end"),
                              pin("Rbal_hi", "2"), pin("Rbal_lo", "1")}),
            conn("cr_mid",   {pin("Cr", "2"), pin("Lr", "primary_start")}),
            conn("pri_top",  {pin("Lr", "primary_end"), pin("T1", "primary_start")})};
        gndPrimary = {pin("Q2", "source"), pin("Dq2", "anode"), pin("Clo", "2"),
                      pin("Rbal_lo", "2"), pin("Csw1", "2")};
    } else {
        // Full bridge (ABT #91): 4 MOSFETs Q1..Q4 + ZVS body diodes drive the tank at ±Vin between the leg
        // midpoints node_a/node_b — no split cap. Diagonal pairs (Q1,Q4) on g1, (Q2,Q3) on g2 -> ±Vin (bridge
        // factor 1.0). Tank: node_a -> Cr -> Lr -> Lpri(=Lm) -> node_b. Csw1 on node_a (declared with the
        // rectifier below to keep the shared card order), Csw2 on node_b (declared here).
        comps = {
            comp("Q1", mosfetReq(reqQ)), comp("Q2", mosfetReq(reqQ)),
            comp("Q3", mosfetReq(reqQ)), comp("Q4", mosfetReq(reqQ)),
            comp("Dq1", diode()), comp("Dq2", diode()), comp("Dq3", diode()), comp("Dq4", diode()),
            comp("Csw2", swSnub()),
            comp("Cr", cr), comp("Lr", lr), comp("T1", t1)};
        conns = {
            conn("vin_net",  {pin("Q1", "drain"), pin("Q3", "drain"),
                              pin("Dq1", "cathode"), pin("Dq3", "cathode"), prt("vin")}),
            conn("node_a",   {pin("Q1", "source"), pin("Q2", "drain"),
                              pin("Dq1", "anode"), pin("Dq2", "cathode"), pin("Cr", "1"), pin("Csw1", "1")}),
            conn("node_b",   {pin("Q3", "source"), pin("Q4", "drain"),
                              pin("Dq3", "anode"), pin("Dq4", "cathode"), pin("T1", "primary_end"),
                              pin("Csw2", "1")}),
            conn("cr_mid",   {pin("Cr", "2"), pin("Lr", "primary_start")}),
            conn("pri_top",  {pin("Lr", "primary_end"), pin("T1", "primary_start")})};
        gndPrimary = {pin("Q2", "source"), pin("Dq2", "anode"), pin("Q4", "source"), pin("Dq4", "anode"),
                      pin("Csw1", "2"), pin("Csw2", "2")};
    }

    // Shared ground-return endpoints: the primary side first, then each rail appends its own returns. Pushed
    // as ONE gnd_net after the extra-rail loop (prt("gnd") last). Single-output reproduces the original
    // [gndPrimary…, rail-0 returns, gnd] byte-for-byte (no extra rails append anything).
    std::vector<json> gndEps = gndPrimary;
    switch (d.rectifierType) {
    case RectifierType::CenterTapped: {
        comps.insert(comps.end(), {comp("D1", diodeReq(reqD)), comp("D2", diodeReq(reqD)), comp("Cout", cout),
            comp("Csw1", swSnub()),
            comp("Rsn1", snubR()), comp("Csn1", snubC()), comp("Rsn2", snubR()), comp("Csn2", snubC())});
        conns.push_back(conn("sec_top", {pin("T1", "secondary1_start"), pin("D1", "anode"),
                                         pin("Rsn1", "1"), pin("Csn1", "1")}));
        conns.push_back(conn("sec_bot", {pin("T1", "secondary2_end"), pin("D2", "anode"),
                                         pin("Rsn2", "1"), pin("Csn2", "1")}));
        conns.push_back(conn("vout_net", {pin("D1", "cathode"), pin("D2", "cathode"),
                          pin("Rsn1", "2"), pin("Csn1", "2"), pin("Rsn2", "2"), pin("Csn2", "2"),
                          pin("Cout", "1"), prt("vout")}));
        gndEps.insert(gndEps.end(), {pin("T1", "secondary1_end"), pin("T1", "secondary2_start"),
                                     pin("Cout", "2")});
        break; }
    case RectifierType::FullBridge: {
        comps.insert(comps.end(), {comp("DH1", diodeReq(reqD)), comp("DH2", diodeReq(reqD)),
            comp("DL1", diodeReq(reqD)), comp("DL2", diodeReq(reqD)), comp("Cout", cout),
            comp("Csw1", swSnub()), comp("Rsn1", snubR()), comp("Csn1", snubC())});
        conns.push_back(conn("sec_a", {pin("T1", "secondary1_start"), pin("DH1", "anode"),
                                       pin("DL1", "cathode"), pin("Rsn1", "1"), pin("Csn1", "1")}));
        conns.push_back(conn("sec_b", {pin("T1", "secondary1_end"), pin("DH2", "anode"),
                                       pin("DL2", "cathode")}));
        conns.push_back(conn("vout_net", {pin("DH1", "cathode"), pin("DH2", "cathode"),
                          pin("Rsn1", "2"), pin("Csn1", "2"), pin("Cout", "1"), prt("vout")}));
        gndEps.insert(gndEps.end(), {pin("DL1", "anode"), pin("DL2", "anode"), pin("Cout", "2")});
        break; }
    case RectifierType::CurrentDoubler: {
        comps.insert(comps.end(), {comp("D1", diodeReq(reqD)), comp("D2", diodeReq(reqD)),
            comp("Lo1", outInductor()), comp("Lo2", outInductor()), comp("Rlb", loopBreakR()),
            comp("Cout", cout), comp("Csw1", swSnub())});
        conns.push_back(conn("node_a", {pin("T1", "secondary1_start"), pin("D1", "cathode"),
                                        pin("Lo1", "primary_start")}));
        conns.push_back(conn("node_b", {pin("T1", "secondary1_end"), pin("D2", "cathode"),
                                        pin("Lo2", "primary_start")}));
        conns.push_back(conn("lo2_out", {pin("Lo2", "primary_end"), pin("Rlb", "1")}));
        conns.push_back(conn("vout_net", {pin("Lo1", "primary_end"), pin("Rlb", "2"),
                          pin("Cout", "1"), prt("vout")}));
        gndEps.insert(gndEps.end(), {pin("D1", "anode"), pin("D2", "anode"), pin("Cout", "2")});
        break; }
    case RectifierType::VoltageDoubler:
        throw std::runtime_error("Kirchhoff SRC: voltageDoubler rectifier not supported");
    }

    // ── Extra isolated rails (outputs[1..], ABT #86) ──
    // Each hangs its own diode rectifier + output cap on its dedicated transformer secondary winding(s) and
    // exposes an external vout<i> port (the assembler synthesizes that rail's load). Rail i's secondary
    // windings are secondary(base)/secondary(base+1) (1-based over ALL secondaries) with base = wpo·i+1.
    // Only the pure-diode rectifiers (CT/FB) are generalized to N rails; the currentDoubler adds per-rail
    // output inductors + loop-breakers (not yet ported), so a multi-output CD spec throws loudly.
    for (size_t i = 1; i < nOut; ++i) {
        const auto& leg = d.outputs[i];
        const double iout_i = leg.power / leg.voltage;
        const std::string sfx = std::to_string(i + 1);                 // "2", "3", …
        const size_t base = wpo * i + 1;                               // 1-based first secondary winding
        const std::string voutP = "vout" + sfx;
        // Per-rail rectifier diode rating: reverse-blocks ~2·Vout_i (CT) and carries the reflected half-sine.
        const double ratedVrDi = 2.0 * leg.voltage / cfg::v_derate_diode(d.config);
        const double maxVfDi   = (ratedVrDi < 100.0) ? 0.6 : 1.2;
        const json reqDi = req::diode(ratedVrDi, (M_PI / 2.0) * iout_i, maxVfDi, 0.05 * Tfr);
        json couti; couti["capacitor"] = json::object();
        couti["inputs"]["designRequirements"]["capacitance"]["nominal"] = leg.outputCapacitance;
        couti["inputs"]["designRequirements"]["ratedVoltage"] = leg.voltage * 2;
        const std::string wA = "secondary" + std::to_string(base) + "_start";
        const std::string wAe = "secondary" + std::to_string(base) + "_end";
        const std::string wB = "secondary" + std::to_string(base + 1) + "_start";
        const std::string wBe = "secondary" + std::to_string(base + 1) + "_end";
        switch (d.rectifierType) {
        case RectifierType::CenterTapped: {
            const std::string D1 = "D1_" + sfx, D2 = "D2_" + sfx, Co = "Cout_" + sfx,
                Rs1 = "Rsn1_" + sfx, Cs1 = "Csn1_" + sfx, Rs2 = "Rsn2_" + sfx, Cs2 = "Csn2_" + sfx;
            comps.insert(comps.end(), {comp(D1.c_str(), diodeReq(reqDi)), comp(D2.c_str(), diodeReq(reqDi)),
                comp(Co.c_str(), couti), comp(Rs1.c_str(), snubR()), comp(Cs1.c_str(), snubC()),
                comp(Rs2.c_str(), snubR()), comp(Cs2.c_str(), snubC())});
            conns.push_back(conn(("sec_top" + sfx).c_str(), {pin("T1", wA.c_str()), pin(D1.c_str(), "anode"),
                                                             pin(Rs1.c_str(), "1"), pin(Cs1.c_str(), "1")}));
            conns.push_back(conn(("sec_bot" + sfx).c_str(), {pin("T1", wBe.c_str()), pin(D2.c_str(), "anode"),
                                                             pin(Rs2.c_str(), "1"), pin(Cs2.c_str(), "1")}));
            conns.push_back(conn((voutP + "_net").c_str(), {pin(D1.c_str(), "cathode"), pin(D2.c_str(), "cathode"),
                              pin(Rs1.c_str(), "2"), pin(Cs1.c_str(), "2"), pin(Rs2.c_str(), "2"), pin(Cs2.c_str(), "2"),
                              pin(Co.c_str(), "1"), prt(voutP.c_str())}));
            gndEps.insert(gndEps.end(), {pin("T1", wAe.c_str()), pin("T1", wB.c_str()), pin(Co.c_str(), "2")});
            break; }
        case RectifierType::FullBridge: {
            const std::string DH1 = "DH1_" + sfx, DH2 = "DH2_" + sfx, DL1 = "DL1_" + sfx, DL2 = "DL2_" + sfx,
                Co = "Cout_" + sfx, Rs1 = "Rsn1_" + sfx, Cs1 = "Csn1_" + sfx;
            comps.insert(comps.end(), {comp(DH1.c_str(), diodeReq(reqDi)), comp(DH2.c_str(), diodeReq(reqDi)),
                comp(DL1.c_str(), diodeReq(reqDi)), comp(DL2.c_str(), diodeReq(reqDi)), comp(Co.c_str(), couti),
                comp(Rs1.c_str(), snubR()), comp(Cs1.c_str(), snubC())});
            conns.push_back(conn(("sec_a" + sfx).c_str(), {pin("T1", wA.c_str()), pin(DH1.c_str(), "anode"),
                                                           pin(DL1.c_str(), "cathode"), pin(Rs1.c_str(), "1"), pin(Cs1.c_str(), "1")}));
            conns.push_back(conn(("sec_b" + sfx).c_str(), {pin("T1", wAe.c_str()), pin(DH2.c_str(), "anode"),
                                                           pin(DL2.c_str(), "cathode")}));
            conns.push_back(conn((voutP + "_net").c_str(), {pin(DH1.c_str(), "cathode"), pin(DH2.c_str(), "cathode"),
                              pin(Rs1.c_str(), "2"), pin(Cs1.c_str(), "2"), pin(Co.c_str(), "1"), prt(voutP.c_str())}));
            gndEps.insert(gndEps.end(), {pin(DL1.c_str(), "anode"), pin(DL2.c_str(), "anode"), pin(Co.c_str(), "2")});
            break; }
        default:
            throw std::invalid_argument("Kirchhoff SRC multi-output: rectifierType '" +
                cfg::get_str(d.config, "rectifierType", "centerTapped") +
                "' is not supported for outputs[1..]; only centerTapped and fullBridge are (ABT #86)");
        }
        cports.push_back(port(voutP.c_str()));
    }
    gndEps.push_back(prt("gnd"));
    conns.push_back(conn("gnd_net", gndEps));
    // Gate nets: half-bridge Q1->g1, Q2->g2; full-bridge diagonal pairs (Q1,Q4)->g1, (Q2,Q3)->g2 (ABT #91).
    if (!d.fullBridge) {
        conns.push_back(conn("g1_net", {pin("Q1", "gate"), prt("g1")}));
        conns.push_back(conn("g2_net", {pin("Q2", "gate"), prt("g2")}));
    } else {
        conns.push_back(conn("g1_net", {pin("Q1", "gate"), pin("Q4", "gate"), prt("g1")}));
        conns.push_back(conn("g2_net", {pin("Q2", "gate"), pin("Q3", "gate"), prt("g2")}));
    }
    cell["ports"] = cports;
    cell["components"] = comps;
    cell["connections"] = conns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", d.inputVoltageMin}, {"nominal", d.inputVoltage}, {"maximum", d.inputVoltageMax}};
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    // Per-rail designRequirements.outputs + operatingPoints.outputs (main = "out"/Vout; extras "out2"/Vout2…).
    // Single-output emits exactly {out} — byte-identical to the original.
    dreq["outputs"] = json::array();
    json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltage; op["ambientTemperature"] = 25.0;
    op["outputs"] = json::array();
    for (size_t i = 0; i < nOut; ++i) {
        const std::string oname = (i == 0) ? "out" : "out" + std::to_string(i + 1);
        json o; o["name"] = oname; o["voltage"]["nominal"] = d.outputs[i].voltage; o["regulation"] = "voltage";
        dreq["outputs"].push_back(o);
        json oo; oo["name"] = oname; oo["power"] = d.outputs[i].power; op["outputs"].push_back(oo);
    }
    tas["inputs"]["operatingPoints"] = json::array({op});

    tas["topology"]["stages"] = json::array({
        req::control_stage("llcController"),
        pstage("srcCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput"))});
    std::vector<json> iscs{
        isc("Vin", "externalPort", "input", {sp("srcCell", "vin")}),
        isc("GND", "externalPort", "input", {sp("srcCell", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("srcCell", "vout")})};
    for (size_t i = 1; i < nOut; ++i) {
        const std::string g = "Vout" + std::to_string(i + 1), pt = "vout" + std::to_string(i + 1);
        iscs.push_back(isc(g.c_str(), "externalPort", "output", {sp("srcCell", pt.c_str())}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    auto stim = [&](const char* sw, double phaseDeg) {
        json st; st["stage"] = "srcCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.switchingFrequency;
        st["waveform"]["dutyCycle"] = d.switchDuty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = json::array({stim("Q1", 0.0), stim("Q2", 180.0)});
    req::finalize_control_seeds(tas, Topology::SERIES_RESONANT_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
