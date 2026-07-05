#include "Pfc.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>
#include <cctype>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
constexpr double kSenseResistance   = 0.1;     // input-current sense [Ω]
constexpr double kRippleFraction    = 0.30;    // hysteretic current ripple as a fraction of peak iL
constexpr double kOutputCapacitance = 220e-6;  // bus cap
constexpr double kPi                = 3.14159265358979323846;

std::string to_lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// Normalize the conduction-mode knob to the canonical {ccm, dcm, crm, transition}. crm and transition
// are the SAME boundary-conduction sizing (MKF process_design_requirements :283 folds them together);
// they are kept distinct only as design intent. Throws on an unrecognised mode (no silent CCM fallback).
std::string normalize_pfc_mode(const std::string& raw) {
    const std::string m = to_lower(raw);
    if (m == "ccm" || m == "continuous" || m == "continuousconductionmode") return "ccm";
    if (m == "dcm" || m == "discontinuous" || m == "discontinuousconductionmode") return "dcm";
    if (m == "crm" || m == "critical" || m == "criticalconductionmode") return "crm";
    if (m == "transition" || m == "transitionmode" || m == "boundary" || m == "bcm" || m == "tm")
        return "transition";
    throw std::invalid_argument("design_pfc: unknown mode '" + raw +
                                "' (expected ccm | dcm | crm | transition)");
}

// Normalize the topology-variant knob to the canonical name. Throws on an unrecognised variant.
std::string normalize_pfc_variant(const std::string& raw) {
    const std::string v = to_lower(raw);
    if (v == "boost" || v.empty()) return "boost";
    if (v == "totempole" || v == "totem-pole" || v == "totem_pole") return "totemPole";
    if (v == "interleaved" || v == "interleavedboost" || v == "interleaved-boost" ||
        v == "interleaved_boost") return "interleaved";
    if (v == "sepic") return "sepic";
    if (v == "cuk" || v == "ćuk" || v == "cúk") return "cuk";
    throw std::invalid_argument("design_pfc: unknown topologyVariant '" + raw +
                                "' (expected boost | totemPole | interleaved | sepic | cuk)");
}
} // namespace

PfcDesign design_pfc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    PfcDesign d{};
    d.config = cfg::object_of(tasInputs);
    d.inputVoltageRms    = nominal(dr.at("inputVoltage"));
    d.lineFrequency      = nominal(dr.at("lineFrequency"));
    d.outputVoltage      = nominal(dr.at("outputs").at(0).at("voltage"));
    d.switchingFrequency = nominal(dr.at("switchingFrequency"));
    d.efficiency         = dr.value("efficiency", 1.0);
    if (tasInputs.contains("operatingPoints") && !tasInputs.at("operatingPoints").empty())
        d.outputPower = tasInputs.at("operatingPoints").at(0).at("outputs").at(0).at("power").get<double>();
    else
        d.outputPower = nominal(dr.at("outputs").at(0).at("power"));

    // ── Conduction mode + topology variant (ABT #92) ─────────────────────────────────────────────────
    d.mode            = normalize_pfc_mode(cfg::get_str(d.config, "mode", "ccm"));
    d.topologyVariant = normalize_pfc_variant(cfg::get_str(d.config, "topologyVariant", "boost"));
    d.bipolar         = (d.topologyVariant == "totemPole");
    // Supported-variant gate (mirrors MKF PowerFactorCorrection::validate_topology_variant :46). An
    // unsupported variant THROWS here — never a silent boost fallback.
    if (d.topologyVariant == "interleaved") {
        d.numberOfPhases = static_cast<int>(std::llround(cfg::get(d.config, "numberOfPhases", 2.0)));
        if (d.numberOfPhases < 2 || d.numberOfPhases > 3)   // MKF :71-83 (2 or 3 phases only)
            throw std::invalid_argument("design_pfc: interleaved topologyVariant requires numberOfPhases "
                                        "in {2,3}, got " + std::to_string(d.numberOfPhases));
    } else {
        d.numberOfPhases = 1;
        // totemPole (bipolar true-sine, bridgeless) and sepic/cuk (buck-boost class) each have their own
        // deck builder below; boost is the default. The buck-boost front ends are sized in continuous
        // conduction only — the DCM/CrM formulas below are boost-specific derivations, so reject a non-CCM
        // request rather than apply a wrong-topology formula (no silent mis-sizing).
        if ((d.topologyVariant == "sepic" || d.topologyVariant == "cuk") && d.mode != "ccm")
            throw std::invalid_argument("design_pfc: '" + d.topologyVariant + "' PFC is sized in CCM only; "
                                        "mode '" + d.mode + "' is not supported for the buck-boost class");
    }
    const bool buckBoost = (d.topologyVariant == "sepic" || d.topologyVariant == "cuk");

    const double pin   = d.outputPower / std::max(d.efficiency, 1e-6);
    const double vpeak = d.inputVoltageRms * std::sqrt(2.0);
    const double rEmul = d.inputVoltageRms * d.inputVoltageRms / pin;  // emulated input resistance (total)
    const double iPeak = vpeak / rEmul;                                // total peak inductor/line current

    d.senseResistance = cfg::get(d.config, "senseResistance", kSenseResistance);
    // i_ref voltage = kref·V(busP) must equal the PER-PHASE target sense voltage iL_phase·Rsense. Each of the
    // N interleaved phases carries 1/N of the line current, so its emulated resistance is N·rEmul → the
    // per-phase reference gain is Rsense/(N·rEmul). N = 1 for boost/totem-pole (unchanged).
    d.referenceGain = d.senseResistance / (rEmul * d.numberOfPhases);

    // ── Conduction mode drives the boost-inductor sizing (MKF PowerFactorCorrection::calculate_inductance_
    // ccm/dcm/crcm, :171/205/218). Every formula evaluates at the line-voltage peak (the worst case). For
    // an INTERLEAVED PFC each of the N phases carries 1/N of the line current, so the per-phase inductor is
    // sized to the per-phase power/current (MKF per_phase_power :163); N = 1 for boost/totem-pole.
    // Duty at the line peak: boost-class D = 1 − Vpk/Vout; buck-boost-class (SEPIC/Ćuk) D = Vout/(Vout+Vpk).
    const double dutyPeak   = buckBoost ? d.outputVoltage / (d.outputVoltage + vpeak)
                                        : 1.0 - vpeak / d.outputVoltage;
    const double pinPhase   = pin   / d.numberOfPhases;        // per-phase input power
    const double iPeakPhase = iPeak / d.numberOfPhases;        // per-phase peak inductor current
    const double ripple     = cfg::get(d.config, "currentRippleFraction", kRippleFraction);
    if (d.mode == "ccm") {
        // ΔiL = ripple·iLpeak; L = Vpk·D / (ΔiL·fsw) ≡ the original Vpk·(Vout−Vpk)/(ΔiL·fsw·Vout).
        d.boostInductance = vpeak * dutyPeak / (ripple * iPeakPhase * d.switchingFrequency);
    } else if (d.mode == "dcm") {
        // L = Vpk²·D² / (2·Pin·fsw)  (MKF calculate_inductance_dcm :214). Smaller L → deep ripple, the
        // inductor fully de-energises each cycle.
        d.boostInductance = vpeak * vpeak * dutyPeak * dutyPeak
                            / (2.0 * pinPhase * d.switchingFrequency);
    } else {   // "crm" || "transition" — boundary conduction (MKF calculate_inductance_crcm :229)
        // L = Vpk·D / (2·iLpeak·fsw): the boundary where ΔiL = 2·iLpeak (the valley just touches zero).
        d.boostInductance = vpeak * dutyPeak / (2.0 * iPeakPhase * d.switchingFrequency);
    }
    // Comparator hysteresis = half the ACTUAL peak ripple the sized (L, D) produce at the line peak, so the
    // hysteretic current band stays self-consistent with the mode's inductor. For CCM this collapses to
    // exactly ripple·iLpeak (byte-identical to the original boost/CCM band).
    const double dILactual = vpeak * dutyPeak / (d.boostInductance * d.switchingFrequency);
    d.currentHysteresis = 0.5 * dILactual * d.senseResistance;   // half-band on the i·Rsense signal
    d.outputCapacitance = cfg::get(d.config, "outputCapacitance", kOutputCapacitance);
    d.loadResistance = d.outputVoltage * d.outputVoltage / d.outputPower;

    // ── SEPIC / Ćuk second inductor L2 + energy-transfer cap Cs. L2 is taken equal to the input inductor L1
    // (a standard SEPIC/Ćuk choice; the two carry comparable ripple). Cs is placed so the L2–Cs resonance
    // sits a decade below fsw — large enough to hold ~constant over a switching cycle, low enough that its
    // resonance does not beat the current loop. Populated only for the buck-boost class (0 otherwise).
    if (buckBoost) {
        d.coupledInductance = d.boostInductance;                       // L2 = L1
        const double fRes = d.switchingFrequency / 10.0;               // resonance a decade below fsw
        d.couplingCapacitance = 1.0 / (std::pow(2.0 * kPi * fRes, 2.0) * d.coupledInductance);
    } else {
        d.coupledInductance = 0.0;
        d.couplingCapacitance = 0.0;
    }

    // ── Outer voltage loop: a DESIGNED PI compensator (derived from the plant, not hand-tuned). ──
    // The inner hysteretic current loop makes the stage emulate a conductance Ge = (1−gv)/Rsense, so the
    // small-signal plant from the control gv to the bus voltage is a SINGLE pole — the bus cap loaded by
    // Rload:   P(s) = −K0/(s+ωp),  K0 = Vrms²/(Rsense·C·Vbus),  ωp = 2/(Rload·C)  (the load pole).
    // A PI controller gv = g0 + kp·e + ki·∫e dt  (e = kv·(Vout−Vtarget)) places its zero ωz = ki/kp at the
    // load pole ωp, cancelling it and leaving a clean single-integrator loop → ~90° phase margin, robust
    // across line and load. Crossover ωc is set a DECADE BELOW the line frequency so the voltage loop does
    // not fight the inherent 2·fline bus ripple (which would distort the |Vac| current reference and spoil
    // the power factor). Unity loop gain at ωc fixes kp; ki = kp·ωp closes the pole/zero cancellation:
    //     kp = ωc·Rsense·C·Vbus / (kv·Vrms²) = ωc/(kv·K0),   ki = kp·ωp.
    d.outputDividerGain = 0.01;                                   // kv: V(voutScaled)=kv·Vout (~4 V at 400 V)
    const double kv = d.outputDividerGain;
    // N interleaved phases each present conductance (1−gv)/Rsense, so the total control-to-power gain scales
    // by N (the plant pole/zero placement is otherwise identical). N = 1 for boost/totem-pole (unchanged).
    const double K0 = d.numberOfPhases * d.inputVoltageRms * d.inputVoltageRms
                      / (d.senseResistance * d.outputCapacitance * d.outputVoltage);
    const double wp = 2.0 / (d.loadResistance * d.outputCapacitance);   // load pole
    const double wc = 2.0 * kPi * d.lineFrequency / 10.0;               // crossover (ripple-safe)
    d.proportionalGain = wc / (kv * K0);
    d.integralGain     = d.proportionalGain * wp;
    return d;
}

// Variant deck builders (ABT #92) — defined after build_pfc_tas.
static json build_pfc_totempole_tas(const PfcDesign& d);
static json build_pfc_interleaved_tas(const PfcDesign& d);
static json build_pfc_buckboost_tas(const PfcDesign& d);   // sepic + cuk

json build_pfc_tas(const PfcDesign& d) {
    // Deck router (ABT #92). The mode (ccm/dcm/crm/transition) only changes the boost-inductor VALUE, so it
    // is common to every deck; the topology VARIANT changes the switching cell, so it selects the builder.
    if (d.topologyVariant == "totemPole") return build_pfc_totempole_tas(d);
    if (d.topologyVariant == "interleaved") return build_pfc_interleaved_tas(d);
    if (d.topologyVariant == "sepic" || d.topologyVariant == "cuk") return build_pfc_buckboost_tas(d);
    if (d.topologyVariant != "boost")
        throw std::invalid_argument("build_pfc_tas: topologyVariant '" + d.topologyVariant +
                                    "' deck is not implemented");
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
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](const json& reqs) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L, const json& excitation) { json j; j["magnetic"] = json::object();
        j["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"},
            std::nullopt, 25.0, {excitation}); return j; };
    auto resBrick = [&](double r, double powerW = -1.0) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = r;
        // Power rating = the resistor's ACTUAL dissipation, passed by the caller (series I^2R, divider
        // share, or reference floor). A bare call defaults to a shunt across the full bus (V^2/R). Using
        // V^2/R with the FULL bus for a high-value divider/reference leg over-rates it by orders of
        // magnitude (a 1k divider bottom is a 16 mW part, not 160 W).
        dr["powerRating"] = (powerW >= 0.0 ? powerW : d.outputVoltage * d.outputVoltage / r);
        return j; };
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    auto integrator = [&](double gain, double initial, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"] = gain; e["initial"] = initial; e["reference"] = ref;
        e["outputLow"] = lo; e["outputHigh"] = hi; return j; };

    // ───────────── POWER stage: diode bridge + boost + inductor-current sense ─────────────
    // Rsense is in SERIES with the inductor (busP -> Rsense -> nL -> L), so V(busP)-V(nL) = iL·Rsense is
    // the true input/inductor current (a return-side resistor would sense the load return, not iL). The
    // boost return is clean ground.
    // Boost-inductor excitation: the current is a rectified-sine ENVELOPE (peak iPeak at the line peak)
    // with HF switching ripple riding on it. Saturation is set by the line-peak instantaneous peak; heating
    // by the line-rms input current (plus the HF ripple); flux swing (core loss) by the HF voltage at the
    // boost rate. (No DC bias — the envelope returns to ~0 each line zero-crossing; offset = line average.)
    // Boost-inductor excitation from the SINGLE FHA source (the SPICE-validated analytical PFC line-cycle
    // solver): the rectified-sine current envelope + HF ripple, as processed peak/rms/offset. PFC has ONE
    // AC operating point (fixed line rms), so the same op feeds both the embedded excitation and the ratings.
    namespace AN = Kirchhoff::analytical;
    const MAS::OperatingPoint aopPfc = AN::analytical_pfc(d.inputVoltageRms, d.outputVoltage, d.outputPower,
                                                          d.lineFrequency, d.switchingFrequency,
                                                          d.boostInductance, d.efficiency);
    const double IpkL  = AN::winding_current(aopPfc, 0, "peak");
    const double IrmsL = AN::winding_current(aopPfc, 0, "rms");
    const double IavgL = AN::winding_current(aopPfc, 0, "offset");
    const json indExc = AN::excitations_processed(aopPfc, "L").at(0);

    // ── semiconductor requirements (worst-case corner) ──
    // The boost MOSFET SW and the boost diode D5 both block the DC bus Vout. The four bridge
    // diodes D1..D4 rectify the AC line; the off diode blocks ~Vout (the bus rail sits across the
    // off-side bridge legs). All carry the boost-inductor / line current envelope already computed.
    const double ratedVdsP = d.outputVoltage / cfg::v_derate_mosfet(d.config);   // SW blocks Vout
    const double ratedVrBoost = d.outputVoltage / cfg::v_derate_diode(d.config);// D5 blocks Vout
    const double ratedVrBridge = d.outputVoltage / cfg::v_derate_diode(d.config);// bridge diode blocks Vbus
    const double maxRdsOnP = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsL * IrmsL);
    const double maxVfBoost  = (ratedVrBoost  < 100.0) ? 0.6 : 1.2;
    const double maxVfBridge = (ratedVrBridge < 100.0) ? 0.6 : 1.2;
    // Bridge diodes carry the full rectified line current (avg of |i_line|); the boost diode carries
    // the output-side average current. Size both to the line current envelope (peak->continuous rating).
    const double IfBridge = IavgL / 0.7;   // line-current average with margin
    const double IfBoost  = IavgL / 0.7;   // boost-diode average current (= mean inductor current)
    const json reqSW = req::mosfet("mainSwitch", ratedVdsP, IpkL, maxRdsOnP, 125.0);
    const json reqD5 = req::diode(ratedVrBoost, IfBoost, maxVfBoost, 0.05 / d.switchingFrequency);
    const json reqBridge = req::diode(ratedVrBridge, IfBridge, maxVfBridge);  // line-freq rectifier (no trr spec)

    json pcell; pcell["name"] = "pfc-power";
    pcell["ports"] = json::array({port("acLine"), port("acNeutral"), port("gnd"), port("vout"),
                                  port("busP"), port("nL"), port("g")});
    pcell["components"] = json::array({
        comp("D1", diode(reqBridge)), comp("D2", diode(reqBridge)), comp("D3", diode(reqBridge)), comp("D4", diode(reqBridge)),
        comp("Rsense", resBrick(d.senseResistance, d.outputPower / (d.inputVoltageRms * d.efficiency) * d.outputPower / (d.inputVoltageRms * d.efficiency) * d.senseResistance * 2.0)),   // inductor-current sense (in series with L)
        comp("L", indBrick(d.boostInductance, indExc)), comp("SW", mosfet(reqSW)), comp("D5", diode(reqD5)),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        comp("Rref", resBrick(10e3, 0.25))});                 // floating-mains common-mode reference
    pcell["connections"] = json::array({
        // Full bridge: A=acLine, B=acNeutral, top rail busP, bottom rail = ground.
        conn("acLine",  {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
        conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
        conn("busP",    {pin("D1","cathode"), pin("D2","cathode"), pin("Rsense","1"), prt("busP")}),
        conn("nL",      {pin("Rsense","2"), pin("L","primary_start"), prt("nL")}),
        conn("gnd_net", {pin("D3","anode"), pin("D4","anode"), pin("SW","source"),
                         pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
        // Boost: nL -> L -> swNode -> D5 -> vout ; SW shunts swNode to ground.
        conn("swNode",  {pin("L","primary_end"), pin("SW","drain"), pin("D5","anode")}),
        conn("vout_net",{pin("D5","cathode"), pin("Cout","1"), prt("vout")}),
        conn("g_net",   {pin("SW","gate"), prt("g")})});

    // ──────────────── CONTROL stage (swappable): current loop + outer VOLTAGE loop ────────────────
    // INNER current loop: the comparator gates the switch on V(nL)−V(vth), where vth = V(busP)·gv. That
    // holds iL ≈ V(busP)·(1−gv)/Rsense ∝ |Vac| → unity PF (the differential sense cancels the busP common
    // mode). OUTER voltage loop: a DESIGNED PI compensator drives gv from the bus error (see design_pfc):
    //     gv = Iv + kp·voutScaled,   Iv = clamp(ivInit + ki·∫(voutScaled − vref)dt)
    // The integrator Iv is the integral path (ki, zero placed at the load pole); the summer Sgv adds the
    // proportional path kp·voutScaled. The constant kp·vref is folded into the integrator's initial so gv
    // starts at g0, and the integrator clamp is the anti-windup rail (gv held to ~0.1×–4× the design
    // conductance). A multiplier forms vth = busP·gv. Bus low → gv falls → more current; high → less.
    const double kref = d.referenceGain;
    const double g0   = 1.0 - kref;                                  // nominal current-loop gain
    const double kv   = d.outputDividerGain;
    const double kp   = d.proportionalGain;
    const double vref = kv * d.outputVoltage;                        // scaled bus setpoint
    const double ivInit = g0 - kp * vref;                            // so gv(0)=g0 at the precharge point
    const double gvLo = 1.0 - 4.0 * kref, gvHi = 1.0 - 0.1 * kref;   // gv draw-range rails (0.1×–4× design)
    const double ivLo = gvLo - kp * vref, ivHi = gvHi - kp * vref;   // integrator clamp = gv rail − offset
    const double rv2 = 1e3, rv1 = rv2 * (1.0 - kv) / kv;
    json ccell; ccell["name"] = "pfc-voltage-current-control";
    ccell["ports"] = json::array({port("busP"), port("nL"), port("vout"), port("gnd"), port("g")});
    ccell["components"] = json::array({
        comp("Rv1", resBrick(rv1, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv1)), comp("Rv2", resBrick(rv2, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv2)),
        comp("Iv", integrator(d.integralGain, ivInit, vref, ivLo, ivHi)),
        comp("Sgv", summer(1.0, kp)),     // gv = Iv + kp·voutScaled (PI: integral + proportional paths)
        comp("Mv", multiplier()),
        comp("Cmp", comparator(d.currentHysteresis))});
    ccell["connections"] = json::array({
        conn("vout",      {pin("Rv1","1"), prt("vout")}),
        conn("voutScaled",{pin("Rv1","2"), pin("Rv2","1"), pin("Iv","in"), pin("Sgv","inB")}),
        conn("gnd",       {pin("Rv2","2"), prt("gnd")}),
        conn("ivout",     {pin("Iv","out"), pin("Sgv","inA")}),
        conn("gv",        {pin("Sgv","out"), pin("Mv","inB")}),
        conn("busP",      {pin("Mv","inA"), prt("busP")}),
        conn("vth",       {pin("Mv","out"), pin("Cmp","inMinus")}),
        conn("nL",        {pin("Cmp","inPlus"), prt("nL")}),
        conn("g",         {pin("Cmp","out"), prt("g")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acSinglePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltageRms; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pfcController"),
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput")),
        pstage("pfcControl", "control", ccell, bind("nL", "sense"), bind("g", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd"), sp("pfcControl", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout"), sp("pfcControl", "vout")}),
        isc("busP", "wire", "", {sp("pfcPower", "busP"), sp("pfcControl", "busP")}),
        isc("nL", "wire", "", {sp("pfcPower", "nL"), sp("pfcControl", "nL")}),
        isc("drive", "wire", "", {sp("pfcControl", "g"), sp("pfcPower", "g")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.06); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 2e-7);
    tas["simulation"]["analyses"] = json::array({an});
    // The controller drives the switch (closed loop) — no open-loop stimulus. Precharge the bus so steady
    // state is reached in a few line cycles (the bus-cap RC is far longer than the sim window).
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::POWER_FACTOR_CORRECTION);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

// TOTEM-POLE bridgeless PFC (ABT #92) — the highest-value non-boost variant. There is NO diode bridge: the
// boost inductor L sits directly on the AC line and sees a TRUE bipolar sine (analytical_pfc bipolar=true).
// Two legs share the bus node `vout` (top of Cout) and ground:
//   • FAST (HF) leg  : Q1 (high, vout↔SW) + Q2 (low, SW↔gnd), each with an anti-parallel free-wheel diode
//     DQ1/DQ2 (real MOSFET body diodes — functional rectifiers, NOT numerical aids). L feeds the SW midpoint.
//   • SLOW (line-freq) leg: two ordinary diodes Da (acNeutral→vout) + Db (gnd→acNeutral) return the neutral
//     to whichever rail matches the line polarity (Db≈gnd on the positive half, Da≈vout on the negative).
// Only ONE fast switch is actively PWM'd per half-cycle (the other leg free-wheels through its body diode):
//   • positive half (V(acLine)>V(acNeutral)): Q2 is the boost switch; DQ1 rectifies to the bus.
//   • negative half: Q1 is the boost switch; DQ2 free-wheels from ground.
// The SAME hysteretic current law drives both: with iRefV = vLineClean·(1−gv) [a bipolar reference, the
// analog of the bridged boost's busP·(1−gv)] and iSenseV = iL·Rsense (both ground-referenced via summers
// with perfect common-mode rejection), Q2 turns ON when iRefV−iSenseV>+hyst (energise) and Q1 turns ON when
// iSenseV−iRefV>+hyst — each gated by the line polarity so exactly one is active. The OUTER voltage loop is
// BYTE-IDENTICAL to the boost's (same Iv integrator + Sgv summer producing gv); the totem-pole's sign
// inversion lives entirely in forming (1−gv) against the SIGNED line voltage, not in the compensator.
static json build_pfc_totempole_tas(const PfcDesign& d) {
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
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](const json& reqs) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L, const json& excitation) { json j; j["magnetic"] = json::object();
        j["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"},
            std::nullopt, 25.0, {excitation}); return j; };
    auto resBrick = [&](double r, double powerW = -1.0) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor"; dr["resistance"]["nominal"] = r;
        // Power from the resistor's actual dissipation (caller-supplied); bare = shunt across the bus.
        dr["powerRating"] = (powerW >= 0.0 ? powerW : d.outputVoltage * d.outputVoltage / r);
        return j; };
    auto comparator = [&](double hyst, double lo, double hi) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = hi; e["outputLow"] = lo; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&](double gain) { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = gain; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    auto integrator = [&](double gain, double initial, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"] = gain; e["initial"] = initial; e["reference"] = ref;
        e["outputLow"] = lo; e["outputHigh"] = hi; return j; };

    // Bipolar boost-inductor excitation from the SPICE-validated line-cycle solver (analytical_pfc
    // bipolar=true): a TRUE bipolar sine current envelope (zero mean, ±I_pk) + HF ripple. PFC has ONE AC
    // operating point, so the same op feeds both the embedded excitation and the semiconductor ratings.
    namespace AN = Kirchhoff::analytical;
    const MAS::OperatingPoint aopPfc = AN::analytical_pfc(d.inputVoltageRms, d.outputVoltage, d.outputPower,
                                                          d.lineFrequency, d.switchingFrequency,
                                                          d.boostInductance, d.efficiency,
                                                          /*diodeVoltageDrop*/ 0.0, /*numberOfPeriods*/ 2,
                                                          /*bipolar*/ true);
    const double IpkL  = AN::winding_current(aopPfc, 0, "peak");
    const double IrmsL = AN::winding_current(aopPfc, 0, "rms");
    const double IavgL = AN::winding_current(aopPfc, 0, "offset");   // ~0 for a true bipolar sine
    const json indExc = AN::excitations_processed(aopPfc, "L").at(0);

    // ── semiconductor requirements (worst-case corner) ──  Every device blocks the DC bus Vout; the fast
    // switches / free-wheel diodes carry the full inductor current; the slow-leg diodes carry the line-
    // current envelope (peak of |i_line| per half-cycle). RMS heating uses the inductor RMS.
    const double ratedVdsP  = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double ratedVrD   = d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxRdsOnP  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsL * IrmsL);
    const double maxVfD     = (ratedVrD < 100.0) ? 0.6 : 1.2;
    const double IfFast     = IrmsL / 0.7;                       // fast free-wheel diode: inductor current
    const double IfSlow     = (IavgL > 0.0 ? IavgL : IrmsL) / 0.7;  // slow-leg line-return current
    const json reqSW   = req::mosfet("mainSwitch", ratedVdsP, IpkL, maxRdsOnP, 125.0);
    const json reqDfw  = req::diode(ratedVrD, IfFast, maxVfD, 0.05 / d.switchingFrequency);  // fast free-wheel
    const json reqDslow = req::diode(ratedVrD, IfSlow, maxVfD);  // line-freq return (no trr spec)

    // ───────────── POWER stage: bridgeless totem-pole (fast leg + slow-diode return leg) ─────────────
    // Rsense is in series with L on the AC-LINE side: acLine -> Rsense -> nL -> L -> SW. V(acLine)-V(nL) =
    // iL·Rsense (signed) is the true bipolar inductor current; the control summer differences it to ground.
    json pcell; pcell["name"] = "pfc-power-totempole";
    pcell["ports"] = json::array({port("acLine"), port("acNeutral"), port("gnd"), port("vout"),
                                  port("nL"), port("gHi"), port("gLo")});
    pcell["components"] = json::array({
        comp("Rsense", resBrick(d.senseResistance, d.outputPower / (d.inputVoltageRms * d.efficiency) * d.outputPower / (d.inputVoltageRms * d.efficiency) * d.senseResistance * 2.0)),
        comp("L", indBrick(d.boostInductance, indExc)),
        comp("Q1", mosfet(reqSW)), comp("Q2", mosfet(reqSW)),               // fast leg (high, low)
        comp("DQ1", diode(reqDfw)), comp("DQ2", diode(reqDfw)),             // anti-parallel free-wheel
        comp("Da", diode(reqDslow)), comp("Db", diode(reqDslow)),          // slow-leg line return
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        comp("Rref", resBrick(10e3, 0.25))});                                     // floating-mains reference
    pcell["connections"] = json::array({
        conn("acLine",  {pin("Rsense","1"), prt("acLine")}),
        conn("nL",      {pin("Rsense","2"), pin("L","primary_start"), prt("nL")}),
        // Fast-leg midpoint SW = L end + Q1 source + Q2 drain + both free-wheel diodes.
        conn("swNode",  {pin("L","primary_end"), pin("Q1","source"), pin("Q2","drain"),
                         pin("DQ1","anode"), pin("DQ2","cathode")}),
        // Bus rail vout: Cout top, Q1 drain, DQ1 cathode, slow-leg Da cathode.
        conn("vout_net",{pin("Q1","drain"), pin("DQ1","cathode"), pin("Da","cathode"),
                         pin("Cout","1"), prt("vout")}),
        // Ground rail: Q2 source, DQ2 anode, slow-leg Db anode, Cout bottom.
        conn("gnd_net", {pin("Q2","source"), pin("DQ2","anode"), pin("Db","anode"),
                         pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
        // Slow-leg neutral node: Da anode + Db cathode + Rref + the AC neutral terminal.
        conn("acNeutral", {pin("Da","anode"), pin("Db","cathode"), pin("Rref","1"), prt("acNeutral")}),
        conn("gHi_net", {pin("Q1","gate"), prt("gHi")}),
        conn("gLo_net", {pin("Q2","gate"), prt("gLo")})});

    // ──────────────── CONTROL stage: bipolar current loop + polarity steering + boost voltage loop ───────
    // Voltage loop (IDENTICAL to the bridged boost): a designed PI produces gv from the bus error.
    const double kref = d.referenceGain;
    const double g0   = 1.0 - kref;
    const double kv   = d.outputDividerGain;
    const double kp   = d.proportionalGain;
    const double vref = kv * d.outputVoltage;
    const double ivInit = g0 - kp * vref;
    const double gvLo = 1.0 - 4.0 * kref, gvHi = 1.0 - 0.1 * kref;
    const double ivLo = gvLo - kp * vref, ivHi = gvHi - kp * vref;
    const double rv2 = 1e3, rv1 = rv2 * (1.0 - kv) / kv;
    const double hyst = d.currentHysteresis;

    json ccell; ccell["name"] = "pfc-totempole-control";
    ccell["ports"] = json::array({port("acLine"), port("acNeutral"), port("nL"),
                                  port("vout"), port("gnd"), port("gHi"), port("gLo")});
    ccell["components"] = json::array({
        // Outer voltage loop → gv (boost-identical).
        comp("Rv1", resBrick(rv1, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv1)), comp("Rv2", resBrick(rv2, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv2)),
        comp("Iv", integrator(d.integralGain, ivInit, vref, ivLo, ivHi)),
        comp("Sgv", summer(1.0, kp)),
        // Bipolar current reference iRefV = vLineClean·(1−gv):
        comp("Sline", summer(1.0, -1.0)),      // vLineClean = V(acLine) − V(acNeutral) (source-enforced sine)
        comp("Mgv", multiplier(1.0)),          // vLineClean·gv
        comp("Sref", summer(1.0, -1.0)),       // iRefV = vLineClean − vLineClean·gv
        // Ground-referenced bipolar current sense iSenseV = V(acLine) − V(nL) = iL·Rsense:
        comp("Ssense", summer(1.0, -1.0)),
        // Line polarity (0/1) — steers which fast switch is active this half-cycle:
        comp("Cpol", comparator(0.0, 0.0, 1.0)),   // 1 on the positive half (acLine>acNeutral)
        comp("Cpoln", comparator(0.0, 0.0, 1.0)),  // 1 on the negative half (acNeutral>acLine)
        // Hysteretic current comparators (0/1). cA: energise the LOW switch; cB: energise the HIGH switch:
        comp("CcA", comparator(hyst, 0.0, 1.0)),   // 1 when iRefV − iSenseV > +hyst
        comp("CcB", comparator(hyst, 0.0, 1.0)),   // 1 when iSenseV − iRefV > +hyst
        // Gate = polarity AND current-demand, scaled to a 5 V drive (gain 5 → 5 V only when both are 1):
        comp("MgLo", multiplier(5.0)),             // Q2 gate = 5·Cpol·CcA
        comp("MgHi", multiplier(5.0))});           // Q1 gate = 5·Cpoln·CcB
    ccell["connections"] = json::array({
        // Voltage divider + PI (boost-identical).
        conn("vout",      {pin("Rv1","1"), prt("vout")}),
        conn("voutScaled",{pin("Rv1","2"), pin("Rv2","1"), pin("Iv","in"), pin("Sgv","inB")}),
        conn("gnd",       {pin("Rv2","2"), prt("gnd")}),
        conn("ivout",     {pin("Iv","out"), pin("Sgv","inA")}),
        conn("gv",        {pin("Sgv","out"), pin("Mgv","inB")}),
        // Bipolar reference chain.
        conn("acLineC",   {pin("Sline","inA"), pin("Ssense","inA"), pin("Cpol","inPlus"),
                           pin("Cpoln","inMinus"), prt("acLine")}),
        conn("acNeutralC",{pin("Sline","inB"), pin("Cpol","inMinus"), pin("Cpoln","inPlus"), prt("acNeutral")}),
        conn("vLineClean",{pin("Sline","out"), pin("Mgv","inA"), pin("Sref","inA")}),
        conn("vLineGv",   {pin("Mgv","out"), pin("Sref","inB")}),
        conn("iRefV",     {pin("Sref","out"), pin("CcA","inPlus"), pin("CcB","inMinus")}),
        // Bipolar current sense.
        conn("nLC",       {pin("Ssense","inB"), prt("nL")}),
        conn("iSenseV",   {pin("Ssense","out"), pin("CcA","inMinus"), pin("CcB","inPlus")}),
        // Polarity + current-demand → gates.
        conn("pol",       {pin("Cpol","out"), pin("MgLo","inA")}),
        conn("poln",      {pin("Cpoln","out"), pin("MgHi","inA")}),
        conn("cA",        {pin("CcA","out"), pin("MgLo","inB")}),
        conn("cB",        {pin("CcB","out"), pin("MgHi","inB")}),
        conn("gLo",       {pin("MgLo","out"), prt("gLo")}),
        conn("gHi",       {pin("MgHi","out"), prt("gHi")})});

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acSinglePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltageRms; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pfcController"),
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput")),
        pstage("pfcControl", "control", ccell, bind("nL", "sense"), bind("gLo", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine"), sp("pfcControl", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral"), sp("pfcControl", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd"), sp("pfcControl", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout"), sp("pfcControl", "vout")}),
        isc("nL", "wire", "", {sp("pfcPower", "nL"), sp("pfcControl", "nL")}),
        isc("driveHi", "wire", "", {sp("pfcControl", "gHi"), sp("pfcPower", "gHi")}),
        isc("driveLo", "wire", "", {sp("pfcControl", "gLo"), sp("pfcPower", "gLo")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.06);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 2e-7);
    tas["simulation"]["analyses"] = json::array({an});
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::POWER_FACTOR_CORRECTION);
    return tas;
}

// INTERLEAVED boost PFC (ABT #92): N phase-shifted boost legs share ONE input bridge and ONE bus cap. Each
// leg carries 1/N of the line current (design_pfc sizes L and the per-phase reference gain accordingly), so
// N smaller inductors replace the single boost inductor. This REUSES the proven single-phase boost cell and
// its hysteretic current loop per phase, all driven by the ONE shared outer voltage loop (the phases self-
// oscillate hysteretically and share the load). MKF PowerFactorCorrection INTERLEAVED_BOOST (per_phase_power
// :163). N = 2 or 3 (validated in design_pfc, mirroring MKF :71-83).
static json build_pfc_interleaved_tas(const PfcDesign& d) {
    const int N = d.numberOfPhases;
    auto port = [](const std::string& n) { json p; p["name"] = n; return p; };
    auto pin  = [](const std::string& c, const char* p) { json e; e["component"] = c; e["pin"] = p; return e; };
    auto prt  = [](const std::string& p) { json e; e["port"] = p; return e; };
    auto conn = [](const std::string& name, std::vector<json> eps) { json c; c["name"] = name; c["endpoints"] = eps; return c; };
    auto comp = [](const std::string& name, json data) { json c; c["name"] = name; c["data"] = data; return c; };
    auto bind = [](const char* p, const char* type) { json b; b["port"] = p; b["type"] = type; return b; };
    auto pstage = [](const char* name, const char* role, json brick, json inb, json outb) {
        json s; s["name"] = name; s["role"] = role; s["circuit"] = brick;
        s["inputPort"] = inb; s["outputPort"] = outb; return s; };
    auto sp = [](const char* st, const std::string& po) { json e; e["stage"] = st; e["port"] = po; return e; };
    auto isc = [](const std::string& name, const char* kind, const char* dir, std::vector<json> eps) {
        json c; c["name"] = name; c["kind"] = kind; if (dir[0]) c["direction"] = dir; c["endpoints"] = eps; return c; };
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](const json& reqs) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L, const json& excitation) { json j; j["magnetic"] = json::object();
        j["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"},
            std::nullopt, 25.0, {excitation}); return j; };
    auto resBrick = [&](double r, double powerW = -1.0) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor"; dr["resistance"]["nominal"] = r;
        // Power from the resistor's actual dissipation (caller-supplied); bare = shunt across the bus.
        dr["powerRating"] = (powerW >= 0.0 ? powerW : d.outputVoltage * d.outputVoltage / r);
        return j; };
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    auto integrator = [&](double gain, double initial, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"] = gain; e["initial"] = initial; e["reference"] = ref;
        e["outputLow"] = lo; e["outputHigh"] = hi; return j; };
    auto nL = [](int p) { return "nL" + std::to_string(p); };
    auto gp = [](int p) { return "g"  + std::to_string(p); };

    // Per-PHASE inductor excitation: analytical_pfc at the per-phase power (Pout/N) and the per-phase L.
    namespace AN = Kirchhoff::analytical;
    const double poutPhase = d.outputPower / N;
    const MAS::OperatingPoint aopPfc = AN::analytical_pfc(d.inputVoltageRms, d.outputVoltage, poutPhase,
                                                          d.lineFrequency, d.switchingFrequency,
                                                          d.boostInductance, d.efficiency);
    const double IpkL  = AN::winding_current(aopPfc, 0, "peak");
    const double IrmsL = AN::winding_current(aopPfc, 0, "rms");
    const double IavgL = AN::winding_current(aopPfc, 0, "offset");
    const json indExc = AN::excitations_processed(aopPfc, "L").at(0);

    // Ratings: each phase switch/diode carries the PER-PHASE current; the shared bridge carries the TOTAL
    // (sum of all N phases). All block the DC bus.
    const double ratedVdsP     = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double ratedVrBoost  = d.outputVoltage / cfg::v_derate_diode(d.config);
    const double ratedVrBridge = d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxRdsOnP     = cfg::rds_on_loss_fraction(d.config) * poutPhase / (IrmsL * IrmsL);
    const double maxVfBoost    = (ratedVrBoost  < 100.0) ? 0.6 : 1.2;
    const double maxVfBridge   = (ratedVrBridge < 100.0) ? 0.6 : 1.2;
    const double IfBridge = N * IavgL / 0.7;   // bridge carries the sum of the phase currents
    const double IfBoost  = IavgL / 0.7;       // per-phase boost-diode average current
    const json reqSW = req::mosfet("mainSwitch", ratedVdsP, IpkL, maxRdsOnP, 125.0);
    const json reqD5 = req::diode(ratedVrBoost, IfBoost, maxVfBoost, 0.05 / d.switchingFrequency);
    const json reqBridge = req::diode(ratedVrBridge, IfBridge, maxVfBridge);

    // ───────────── POWER stage: one bridge + N boost legs sharing busP / vout / gnd ─────────────
    json pcell; pcell["name"] = "pfc-power-interleaved";
    std::vector<json> pports = {port("acLine"), port("acNeutral"), port("gnd"), port("vout"), port("busP")};
    for (int p = 1; p <= N; ++p) { pports.push_back(port(nL(p))); pports.push_back(port(gp(p))); }
    pcell["ports"] = pports;

    std::vector<json> pcomps = {
        comp("D1", diode(reqBridge)), comp("D2", diode(reqBridge)),
        comp("D3", diode(reqBridge)), comp("D4", diode(reqBridge)),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        comp("Rref", resBrick(10e3, 0.25))};
    for (int p = 1; p <= N; ++p) {
        pcomps.push_back(comp("Rsense" + std::to_string(p), resBrick(d.senseResistance, d.outputPower / (d.inputVoltageRms * d.efficiency) * d.outputPower / (d.inputVoltageRms * d.efficiency) * d.senseResistance * 2.0)));
        pcomps.push_back(comp("L" + std::to_string(p), indBrick(d.boostInductance, indExc)));
        pcomps.push_back(comp("SW" + std::to_string(p), mosfet(reqSW)));
        pcomps.push_back(comp("D5" + std::to_string(p), diode(reqD5)));
    }
    pcell["components"] = pcomps;

    // Shared nets: busP (all Rsense_p.1), gnd (all SW_p.source), vout (all D5_p.cathode).
    std::vector<json> busPeps  = {pin("D1","cathode"), pin("D2","cathode"), prt("busP")};
    std::vector<json> gndEps   = {pin("D3","anode"), pin("D4","anode"), pin("Cout","2"), pin("Rref","2"), prt("gnd")};
    std::vector<json> voutEps  = {pin("Cout","1"), prt("vout")};
    for (int p = 1; p <= N; ++p) {
        busPeps.push_back(pin("Rsense" + std::to_string(p), "1"));
        gndEps.push_back(pin("SW" + std::to_string(p), "source"));
        voutEps.push_back(pin("D5" + std::to_string(p), "cathode"));
    }
    std::vector<json> pconns = {
        conn("acLine",    {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
        conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
        conn("busP",    busPeps),
        conn("gnd_net", gndEps),
        conn("vout_net",voutEps)};
    for (int p = 1; p <= N; ++p) {
        const std::string ps = std::to_string(p);
        pconns.push_back(conn(nL(p),      {pin("Rsense"+ps,"2"), pin("L"+ps,"primary_start"), prt(nL(p))}));
        pconns.push_back(conn("swNode"+ps,{pin("L"+ps,"primary_end"), pin("SW"+ps,"drain"), pin("D5"+ps,"anode")}));
        pconns.push_back(conn(gp(p),      {pin("SW"+ps,"gate"), prt(gp(p))}));
    }
    pcell["connections"] = pconns;

    // ──────────────── CONTROL stage: ONE outer voltage loop + N per-phase current comparators ───────────
    const double kref = d.referenceGain;            // PER-PHASE reference gain (Rsense/(N·rEmul))
    const double g0   = 1.0 - kref;
    const double kv   = d.outputDividerGain;
    const double kp   = d.proportionalGain;
    const double vref = kv * d.outputVoltage;
    const double ivInit = g0 - kp * vref;
    const double gvLo = 1.0 - 4.0 * kref, gvHi = 1.0 - 0.1 * kref;
    const double ivLo = gvLo - kp * vref, ivHi = gvHi - kp * vref;
    const double rv2 = 1e3, rv1 = rv2 * (1.0 - kv) / kv;
    json ccell; ccell["name"] = "pfc-voltage-current-control-interleaved";
    std::vector<json> cports = {port("busP"), port("vout"), port("gnd")};
    for (int p = 1; p <= N; ++p) { cports.push_back(port(nL(p))); cports.push_back(port(gp(p))); }
    ccell["ports"] = cports;
    std::vector<json> ccomps = {
        comp("Rv1", resBrick(rv1, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv1)), comp("Rv2", resBrick(rv2, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv2)),
        comp("Iv", integrator(d.integralGain, ivInit, vref, ivLo, ivHi)),
        comp("Sgv", summer(1.0, kp)),
        comp("Mv", multiplier())};
    for (int p = 1; p <= N; ++p)
        ccomps.push_back(comp("Cmp" + std::to_string(p), comparator(d.currentHysteresis)));
    ccell["components"] = ccomps;
    // Shared reference vth = busP·gv fans out to every phase comparator's inMinus.
    std::vector<json> vthEps = {pin("Mv","out")};
    for (int p = 1; p <= N; ++p) vthEps.push_back(pin("Cmp" + std::to_string(p), "inMinus"));
    std::vector<json> cconns = {
        conn("vout",      {pin("Rv1","1"), prt("vout")}),
        conn("voutScaled",{pin("Rv1","2"), pin("Rv2","1"), pin("Iv","in"), pin("Sgv","inB")}),
        conn("gnd",       {pin("Rv2","2"), prt("gnd")}),
        conn("ivout",     {pin("Iv","out"), pin("Sgv","inA")}),
        conn("gv",        {pin("Sgv","out"), pin("Mv","inB")}),
        conn("busP",      {pin("Mv","inA"), prt("busP")}),
        conn("vth",       vthEps)};
    for (int p = 1; p <= N; ++p) {
        cconns.push_back(conn(nL(p), {pin("Cmp"+std::to_string(p),"inPlus"), prt(nL(p))}));
        cconns.push_back(conn(gp(p), {pin("Cmp"+std::to_string(p),"out"),    prt(gp(p))}));
    }
    ccell["connections"] = cconns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acSinglePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltageRms; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pfcController"),
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput")),
        pstage("pfcControl", "control", ccell, bind("nL1", "sense"), bind("g1", "drive"))});
    std::vector<json> iscs = {
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd"), sp("pfcControl", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout"), sp("pfcControl", "vout")}),
        isc("busP", "wire", "", {sp("pfcPower", "busP"), sp("pfcControl", "busP")})};
    for (int p = 1; p <= N; ++p) {
        iscs.push_back(isc(nL(p), "wire", "", {sp("pfcPower", nL(p)), sp("pfcControl", nL(p))}));
        iscs.push_back(isc("drive" + std::to_string(p), "wire", "",
                           {sp("pfcControl", gp(p)), sp("pfcPower", gp(p))}));
    }
    tas["topology"]["interStageConnections"] = iscs;

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.06);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 2e-7);
    tas["simulation"]["analyses"] = json::array({an});
    { json ic; ic["node"] = "Vout"; ic["voltage"] = d.outputVoltage;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::POWER_FACTOR_CORRECTION);
    return tas;
}

// SEPIC / Ćuk PFC (ABT #92) — a BUCK-BOOST-class front end (Vout may sit above OR below the line peak).
// The input side (bridge → Rsense → L1 → SW-to-ground) and its hysteretic current loop are IDENTICAL to the
// bridged boost: L1's current is sensed across Rsense and driven to track busP·(1−gv) ∝ |Vac|, so the same
// designed voltage/current loops apply unchanged. Only the OUTPUT cell differs:
//   • SEPIC (non-inverting): Cs from the switch node to nB; L2 nB→gnd; D5 nB→vout(+); Cout vout→gnd.
//   • Ćuk  (inverting):      Cs from the switch node to nB; L2 nB→vout(−); D5 gnd→nB; Cout vout→gnd.
// For Ćuk the bus is NEGATIVE, so the voltage-loop sense is a summer negator voutScaled = −kv·Vout (the
// resistive divider used for the positive SEPIC/boost bus would feed the PI a negative signal); the rest of
// the compensator — Iv integrator, Sgv summer, Mv=busP·gv, hysteretic comparator — is byte-identical.
static json build_pfc_buckboost_tas(const PfcDesign& d) {
    const bool inverting = (d.topologyVariant == "cuk");
    const double vbusSign = inverting ? -1.0 : 1.0;
    const double vbus = vbusSign * d.outputVoltage;   // signed bus rail (SEPIC +Vout, Ćuk −Vout)

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
    auto mosfet = [](const json& reqs) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto diode  = [](const json& reqs) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs; return j; };
    auto capBrick = [&](double c, double vr) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vr; return j; };
    auto indBrick = [&](double L, const json& excitation) { json j; j["magnetic"] = json::object();
        j["inputs"] = req::magnetic_inputs(L, 0.2, /*single winding*/ {}, {"primary"},
            std::nullopt, 25.0, {excitation}); return j; };
    auto resBrick = [&](double r, double powerW = -1.0) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor"; dr["resistance"]["nominal"] = r;
        // Power from the resistor's actual dissipation (caller-supplied); bare = shunt across the bus.
        dr["powerRating"] = (powerW >= 0.0 ? powerW : d.outputVoltage * d.outputVoltage / r);
        return j; };
    auto comparator = [&](double hyst) { json j; json& e = j["analog"]["comparator"]["behavioral"];
        e["outputHigh"] = 5.0; e["outputLow"] = 0.0; e["threshold"] = 0.0; e["hysteresis"] = hyst; return j; };
    auto multiplier = [&]() { json j; j["analog"]["multiplier"]["behavioral"]["gain"] = 1.0; return j; };
    auto summer = [&](double gA, double gB) { json j; json& e = j["analog"]["summer"]["behavioral"];
        e["gainA"] = gA; e["gainB"] = gB; return j; };
    auto integrator = [&](double gain, double initial, double ref, double lo, double hi) {
        json j; json& e = j["analog"]["integrator"]["behavioral"];
        e["gain"] = gain; e["initial"] = initial; e["reference"] = ref;
        e["outputLow"] = lo; e["outputHigh"] = hi; return j; };

    // Input-inductor L1 excitation from the (unipolar, bridged) analytical PFC solver — its current is the
    // rectified-sine line-current envelope + HF ripple, exactly as for the boost (L1 IS the PFC inductor).
    namespace AN = Kirchhoff::analytical;
    const MAS::OperatingPoint aopPfc = AN::analytical_pfc(d.inputVoltageRms, d.outputVoltage, d.outputPower,
                                                          d.lineFrequency, d.switchingFrequency,
                                                          d.boostInductance, d.efficiency);
    const double IpkL  = AN::winding_current(aopPfc, 0, "peak");
    const double IrmsL = AN::winding_current(aopPfc, 0, "rms");
    const double IavgL = AN::winding_current(aopPfc, 0, "offset");
    const json indExc = AN::excitations_processed(aopPfc, "L").at(0);

    // ── semiconductor requirements. The switch SW and diode D5 block Vout + the peak line voltage
    // (buck-boost stress = Vout + Vpk); the bridge diodes block the bus; all carry the L1/line current. ──
    const double vpeak = d.inputVoltageRms * std::sqrt(2.0);
    const double vSwStress = d.outputVoltage + vpeak;
    const double ratedVdsP     = vSwStress / cfg::v_derate_mosfet(d.config);
    const double ratedVrD5     = vSwStress / cfg::v_derate_diode(d.config);
    const double ratedVrBridge = d.outputVoltage / cfg::v_derate_diode(d.config);
    const double maxRdsOnP     = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IrmsL * IrmsL);
    const double maxVfD5       = (ratedVrD5 < 100.0) ? 0.6 : 1.2;
    const double maxVfBridge   = (ratedVrBridge < 100.0) ? 0.6 : 1.2;
    const double IfBridge = IavgL / 0.7;
    const double IfD5     = (d.outputPower / d.outputVoltage) / 0.7;   // output-side average current
    const json reqSW = req::mosfet("mainSwitch", ratedVdsP, IpkL, maxRdsOnP, 125.0);
    const json reqD5 = req::diode(ratedVrD5, IfD5, maxVfD5, 0.05 / d.switchingFrequency);
    const json reqBridge = req::diode(ratedVrBridge, IfBridge, maxVfBridge);

    // ───────────── POWER stage: diode bridge + SEPIC/Ćuk front end + L1-current sense ─────────────
    json pcell; pcell["name"] = inverting ? "pfc-power-cuk" : "pfc-power-sepic";
    pcell["ports"] = json::array({port("acLine"), port("acNeutral"), port("gnd"), port("vout"),
                                  port("busP"), port("nL"), port("g")});
    std::vector<json> pcomps = {
        comp("D1", diode(reqBridge)), comp("D2", diode(reqBridge)),
        comp("D3", diode(reqBridge)), comp("D4", diode(reqBridge)),
        comp("Rsense", resBrick(d.senseResistance, d.outputPower / (d.inputVoltageRms * d.efficiency) * d.outputPower / (d.inputVoltageRms * d.efficiency) * d.senseResistance * 2.0)),
        comp("L1", indBrick(d.boostInductance, indExc)),
        comp("SW", mosfet(reqSW)),
        comp("Cs", capBrick(d.couplingCapacitance, vSwStress * 2)),
        comp("L2", indBrick(d.coupledInductance, indExc)),
        comp("D5", diode(reqD5)),
        comp("Cout", capBrick(d.outputCapacitance, d.outputVoltage * 2)),
        comp("Rref", resBrick(10e3, 0.25))};
    pcell["components"] = pcomps;
    // The input side (bridge + Rsense + L1 + SW-to-gnd + Cs on the switch node nA) is common; only the
    // output cell (L2 / D5 orientation and the bus sign) differs between SEPIC and Ćuk.
    if (!inverting) {
        // SEPIC (non-inverting): L2 nB→gnd; D5 nB(anode)→vout(+)(cathode); Cout vout(+)→gnd.
        pcell["connections"] = json::array({
            conn("acLine",  {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
            conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
            conn("busP",    {pin("D1","cathode"), pin("D2","cathode"), pin("Rsense","1"), prt("busP")}),
            conn("nL",      {pin("Rsense","2"), pin("L1","primary_start"), prt("nL")}),
            conn("gnd_net", {pin("D3","anode"), pin("D4","anode"), pin("SW","source"),
                             pin("L2","primary_end"), pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
            conn("nA",      {pin("L1","primary_end"), pin("SW","drain"), pin("Cs","1")}),
            conn("nB",      {pin("Cs","2"), pin("L2","primary_start"), pin("D5","anode")}),
            conn("vout_net",{pin("D5","cathode"), pin("Cout","1"), prt("vout")}),
            conn("g_net",   {pin("SW","gate"), prt("g")})});
    } else {
        // Ćuk (inverting): Cs on the switch node; D5 nB(anode)→gnd(cathode) freewheels when SW is off;
        // L2 nB→vout(−); Cout vout(−)→gnd. The bus is negative (precharged to −Vout).
        pcell["connections"] = json::array({
            conn("acLine",  {pin("D1","anode"), pin("D3","cathode"), prt("acLine")}),
            conn("acNeutral", {pin("D2","anode"), pin("D4","cathode"), pin("Rref","1"), prt("acNeutral")}),
            conn("busP",    {pin("D1","cathode"), pin("D2","cathode"), pin("Rsense","1"), prt("busP")}),
            conn("nL",      {pin("Rsense","2"), pin("L1","primary_start"), prt("nL")}),
            conn("gnd_net", {pin("D3","anode"), pin("D4","anode"), pin("SW","source"),
                             pin("D5","cathode"), pin("Cout","2"), pin("Rref","2"), prt("gnd")}),
            conn("nA",      {pin("L1","primary_end"), pin("SW","drain"), pin("Cs","1")}),
            conn("nB",      {pin("Cs","2"), pin("L2","primary_start"), pin("D5","anode")}),
            conn("vout_net",{pin("L2","primary_end"), pin("Cout","1"), prt("vout")}),
            conn("g_net",   {pin("SW","gate"), prt("g")})});
    }

    // ──────────────── CONTROL stage: boost current loop + outer voltage loop (Ćuk: negated sense) ─────────
    const double kref = d.referenceGain;
    const double g0   = 1.0 - kref;
    const double kv   = d.outputDividerGain;
    const double kp   = d.proportionalGain;
    const double vref = kv * d.outputVoltage;
    const double ivInit = g0 - kp * vref;
    const double gvLo = 1.0 - 4.0 * kref, gvHi = 1.0 - 0.1 * kref;
    const double ivLo = gvLo - kp * vref, ivHi = gvHi - kp * vref;
    const double rv2 = 1e3, rv1 = rv2 * (1.0 - kv) / kv;

    json ccell; ccell["name"] = inverting ? "pfc-cuk-control" : "pfc-sepic-control";
    ccell["ports"] = json::array({port("busP"), port("nL"), port("vout"), port("gnd"), port("g")});
    std::vector<json> ccomps = {
        comp("Iv", integrator(d.integralGain, ivInit, vref, ivLo, ivHi)),
        comp("Sgv", summer(1.0, kp)),
        comp("Mv", multiplier()),
        comp("Cmp", comparator(d.currentHysteresis))};
    // Voltage sense → voutScaled (always positive). SEPIC: resistive divider. Ćuk (negative bus): a summer
    // negator voutScaled = −kv·V(vout) (inA=vout gain −kv; inB=gnd gain 0).
    if (!inverting) {
        ccomps.push_back(comp("Rv1", resBrick(rv1, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv1)));
        ccomps.push_back(comp("Rv2", resBrick(rv2, d.outputVoltage / (rv1 + rv2) * d.outputVoltage / (rv1 + rv2) * rv2)));
    } else {
        ccomps.push_back(comp("Svo", summer(-kv, 0.0)));
    }
    ccell["components"] = ccomps;
    std::vector<json> cconns = {
        conn("ivout",     {pin("Iv","out"), pin("Sgv","inA")}),
        conn("gv",        {pin("Sgv","out"), pin("Mv","inB")}),
        conn("busP",      {pin("Mv","inA"), prt("busP")}),
        conn("vth",       {pin("Mv","out"), pin("Cmp","inMinus")}),
        conn("nL",        {pin("Cmp","inPlus"), prt("nL")}),
        conn("g",         {pin("Cmp","out"), prt("g")})};
    if (!inverting) {
        cconns.push_back(conn("vout",      {pin("Rv1","1"), prt("vout")}));
        cconns.push_back(conn("voutScaled",{pin("Rv1","2"), pin("Rv2","1"), pin("Iv","in"), pin("Sgv","inB")}));
        cconns.push_back(conn("gnd",       {pin("Rv2","2"), prt("gnd")}));
    } else {
        cconns.push_back(conn("vout",      {pin("Svo","inA"), prt("vout")}));
        cconns.push_back(conn("gnd",       {pin("Svo","inB"), prt("gnd")}));
        cconns.push_back(conn("voutScaled",{pin("Svo","out"), pin("Iv","in"), pin("Sgv","inB")}));
    }
    ccell["connections"] = cconns;

    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "acSinglePhase";
    dreq["inputVoltage"]["nominal"] = d.inputVoltageRms;
    dreq["lineFrequency"]["nominal"] = d.lineFrequency;
    dreq["switchingFrequency"]["nominal"] = d.switchingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = d.outputVoltage; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = d.inputVoltageRms; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("pfcController"),
        pstage("pfcPower", "switchingCell", pcell, bind("acLine", "acInput"), bind("vout", "dcOutput")),
        pstage("pfcControl", "control", ccell, bind("nL", "sense"), bind("g", "drive"))});
    tas["topology"]["interStageConnections"] = json::array({
        isc("AcLine", "externalPort", "input", {sp("pfcPower", "acLine")}),
        isc("AcNeutral", "externalPort", "input", {sp("pfcPower", "acNeutral")}),
        isc("GND", "externalPort", "input", {sp("pfcPower", "gnd"), sp("pfcControl", "gnd")}),
        isc("Vout", "externalPort", "output", {sp("pfcPower", "vout"), sp("pfcControl", "vout")}),
        isc("busP", "wire", "", {sp("pfcPower", "busP"), sp("pfcControl", "busP")}),
        isc("nL", "wire", "", {sp("pfcPower", "nL"), sp("pfcControl", "nL")}),
        isc("drive", "wire", "", {sp("pfcControl", "g"), sp("pfcPower", "g")})});

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.06);
    an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 2e-7);
    tas["simulation"]["analyses"] = json::array({an});
    // Precharge the SIGNED bus (SEPIC +Vout, Ćuk −Vout) so steady state is reached in a few line cycles.
    { json ic; ic["node"] = "Vout"; ic["voltage"] = vbus;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::POWER_FACTOR_CORRECTION);
    return tas;
}

} // namespace Kirchhoff
