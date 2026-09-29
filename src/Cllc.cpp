#include "Cllc.hpp"
#include "DimensionJson.hpp"
#include "KirchhoffConfig.hpp"
#include "ComponentRequirements.hpp"
#include "ConverterAnalytical.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace Kirchhoff {
using nlohmann::json;

namespace {
double nominal(const json& j) { return PEAS::resolve_dimensional_values(j); }
// Numbers in the operating-frequency messages: 6 significant digits.
std::string num(double v) { std::ostringstream o; o << v; return o.str(); }
constexpr double kQualityFactor   = 0.3;   // MKF Cllc default (Infineon AN: 0.2–0.4)
constexpr double kInductanceRatio = 4.45;  // k = Lm/Lr1 (MKF defaultInductanceRatio)
constexpr double kSwitchDuty      = 0.47;  // ~50% minus dead time
constexpr double kSenseResistance = 0.01;  // in-line current-sense resistor in the secondary tank [Ω]
constexpr double kSenseHysteresis = 5e-3;  // SR comparator hysteresis on the i·Rsense signal [V]
constexpr double kGainHeadroom    = 1.08;  // size n for M=1 at fr -> 1.08·Vo, so the nominal operating
                                           // point sits just ABOVE fr (efficient) not at the M=1 peak
} // namespace

CllcDesign design_cllc(const json& tasInputs) {
    const json& dr = tasInputs.at("designRequirements");
    CllcDesign d{};
    d.config = cfg::object_of(tasInputs);
    // Rail polarity (ABT #904): reject a negative setpoint loudly. Without this it would flow
    // straight into the design math (negative load resistance and friends) and emit nonsense.
    for (const auto& o : dr.at("outputs"))
        if (nominal(o.at("voltage")) < 0)
            throw std::invalid_argument(
                "Kirchhoff CLLC: a negative output rail is not supported by this topology. Its secondary is an ACTIVE BRIDGE, whose output polarity is set by the gate PATTERN rather than by device orientation, so the mirror used for the diode-rectified topologies (flyback, forward family, push-pull, acf, llc, src) does not apply here. Send |Vout| — refusing rather than silently designing a POSITIVE rail (ABT #904).");
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

    // n = Vin_nom/(headroom·Vout) (full bridge both sides). fr = fsw. With zero headroom (n=Vin/Vo) the
    // nominal point sits exactly at the fr gain PEAK (M=1), so any real loss (FET Rds, magnetic DCR/core,
    // rectifier drop) sags Vout below target and the regulator must dive FAR below resonance to recover —
    // high circulating current, ~50% efficiency (abt #62). Sizing ~8% gain headroom puts the nominal point
    // just ABOVE fr on the efficient monotonic edge: the regulator trims frequency DOWN toward fr to cover
    // losses while staying near resonance. The realized headroom n flows into the pinned turnsRatio below.
    double n = req::conversion_efficiency(dr) * Vin / (cfg::get(d.config, "gainHeadroom", kGainHeadroom) * Vo);
    // della-Pollock Pass 2: a pinned turns ratio (the realized ratio of the chosen magnetic) overrides
    // the duty-derived value so the rest of the stage is sized around the fixed transformer.
    d.turnsRatio = req::provided_turns_ratio(dr, 0).value_or(n);
    const double fr = d.switchingFrequency;
    d.resonantFrequency = fr;
    // MAS cllcResonant.bridgeType: Kirchhoff's CLLC is a full bridge on both sides (bridge-voltage factor 1).
    if (cfg::get_str(d.config, "bridgeType", "fullBridge") != "fullBridge")
        throw std::invalid_argument("design_cllc: bridgeType '" + cfg::get_str(d.config, "bridgeType", "") +
                                    "' is not modelled; Kirchhoff's CLLC has a full-bridge primary");

    // Infineon FHA: Ro = 8n²/π²·Rload, Cr1 = 1/(2π·Q·fr·Ro), Lr1 = 1/((2π·fr)²·Cr1), Lm = k·Lr1.
    // Symmetric tank (a=b=1): Lr2 = Lr1/n², Cr2 = n²·Cr1.  (MKF Cllc::calculate_resonant_parameters)
    const double Rload = Vo * Vo / d.outputPower;
    const double Ro = (8.0 * n * n / (M_PI * M_PI)) * Rload;
    const double wr = 2.0 * M_PI * fr;
    d.primaryResonantCapacitance = 1.0 / (2.0 * M_PI * cfg::get(d.config, "qualityFactor", kQualityFactor) * fr * Ro);
    d.primaryResonantInductance = 1.0 / (wr * wr * d.primaryResonantCapacitance);
    const auto pinnedLm = req::provided_inductance(dr);
    d.magnetizingInductance = pinnedLm.value_or(
        cfg::get(d.config, "inductanceRatio", kInductanceRatio) * d.primaryResonantInductance);
    // della-Pollock resonant tank CO-DESIGN: the closed-loop realize pins Lm to the REALIZED magnetizing
    // inductance of the chosen transformer core (sized for a saturation margin, so typically larger than
    // k·Lr1). Once Lm is fixed it no longer equals k·Lr1, so the tank is DETUNED (k=Lm/Lr1 drifts, the gain
    // curve shifts, the converter is pushed off resonance and can't reach target Vout). Re-size the PRIMARY
    // tank around the pinned Lm to PRESERVE the design ratio k AND keep Lr1–Cr1 resonant at fr:
    //   Lr1 = Lm/k,  Cr1 = 1/((2π·fr)²·Lr1).  The secondary tank (Lr2/Cr2 below) follows from the new
    // Lr1/Cr1, staying symmetric (Lr2·Cr2 = Lr1·Cr1 → same fr). Lr1/Lr2 are their own freshly-designed
    // magnetics and Cr1/Cr2 near-nominal (role="resonant") sourced caps, so all track the new values; only
    // the pinned transformer is fixed. (No pin → original Q·Ro sizing stands; the mkf_equivalence ideal
    // deck never pins Lm and is unchanged.)
    if (pinnedLm) {
        const double k = cfg::get(d.config, "inductanceRatio", kInductanceRatio);
        d.primaryResonantInductance = *pinnedLm / k;
        d.primaryResonantCapacitance = 1.0 / (wr * wr * d.primaryResonantInductance);
    }
    // Tank asymmetry (MAS cllcResonant.resonantInductorRatio a = n²·Lr2/Lr1, resonantCapacitorRatio
    // b = Cr2/(n²·Cr1); symmetric a = b = 1). symmetricDesign (deprecated) must agree with the ratios.
    const double a = cfg::get(d.config, "resonantInductorRatio", 1.0);
    const double b = cfg::get(d.config, "resonantCapacitorRatio", 1.0);
    if (!(a > 0) || !(b > 0))
        throw std::invalid_argument("design_cllc: resonantInductorRatio and resonantCapacitorRatio must be > 0");
    if (d.config.contains("symmetricDesign")) {
        const bool symmetric = cfg::get_bool(d.config, "symmetricDesign", true);
        const bool ratiosSymmetric = (a == 1.0 && b == 1.0);
        if (symmetric != ratiosSymmetric)
            throw std::invalid_argument(std::string("design_cllc: symmetricDesign=") + (symmetric ? "true" : "false") +
                                        " contradicts resonantInductorRatio=" + std::to_string(a) +
                                        ", resonantCapacitorRatio=" + std::to_string(b));
    }
    d.secondaryResonantInductance = a * d.primaryResonantInductance / (n * n);
    d.secondaryResonantCapacitance = b * n * n * d.primaryResonantCapacitance;

    d.switchDuty = cfg::get(d.config, "switchDutyFraction", kSwitchDuty);
    d.loadResistance = Rload;
    d.outputCapacitance = 10e-6;    // matches MKF CLLC (Cout=10u)
    // Power-flow direction (ABT #85). "reverse" makes the Vout side the source and delivers to the Vin side;
    // the tank/turns-ratio sizing is unchanged (symmetric bidirectional design), only the deck wiring flips.
    const std::string dir = cfg::get_str(d.config, "powerFlowDirection", "forward");
    if (dir != "forward" && dir != "reverse")
        throw std::invalid_argument("design_cllc: config.powerFlowDirection must be 'forward' or 'reverse', got '" + dir + "'");
    d.reverse = (dir == "reverse");
    // MAS cllcResonant.bidirectional (default true): false means a diode secondary rectifier, which cannot
    // carry reverse power. The forward operating point's winding waveforms are the same either way.
    if (d.reverse && !cfg::get_bool(d.config, "bidirectional", true))
        throw std::invalid_argument("design_cllc: a reverse power-flow operating point needs bidirectional=true");

    // ── Operating frequency (ABT #1503) ──
    // The CLLC regulates by frequency. The tank resonates at fr, where a symmetric tank's FHA gain is 1 whatever
    // the ratio, but neither ratio is sized for unity gain there: the engine-sized one carries the 1.08 gain
    // headroom (at fr it would deliver 1.08·Vout) and a pinned one is whatever the chosen magnetic realised.
    // Driving at fr anyway clamped the transformer to ±n·Vout while the tank current came from a drive that
    // delivers a different voltage. Solve the frequency where the gain delivers the output, or refuse.
    const bool pinnedRatio = req::provided_turns_ratio(dr, 0).has_value();
    const auto op = two_sided_resonant_operating_point(
        "design_cllc",
        pinnedRatio ? "pinned turns ratio"
                    : "turns ratio sized with a gain headroom of " +
                          num(cfg::get(d.config, "gainHeadroom", kGainHeadroom)),
        d.config, d.reverse, d.turnsRatio, req::conversion_efficiency(dr), Vin, Vo, d.outputPower, fr,
        d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
        d.secondaryResonantInductance, d.secondaryResonantCapacitance);
    d.operatingFrequency = op.operatingFrequency;
    d.requiredGain = op.requiredGain;
    return d;
}

TwoSidedOperatingPoint two_sided_resonant_operating_point(
    const std::string& who, const std::string& ratioOrigin, const json& config, bool reverse,
    double turnsRatio, double efficiency, double inputVoltage, double outputVoltage, double outputPower,
    double resonantFrequency, double magnetizingInductance, double primaryResonantInductance,
    double primaryResonantCapacitance, double secondaryResonantInductance, double secondaryResonantCapacitance) {
    namespace AN = Kirchhoff::analytical;
    const double N = turnsRatio, eta = efficiency, fr = resonantFrequency;
    if (!(N > 0) || !(eta > 0) || !(inputVoltage > 0) || !(outputVoltage > 0) || !(outputPower > 0) || !(fr > 0))
        throw std::invalid_argument(who + ": turns ratio, efficiency, voltages, power and fr must all be > 0");
    // The switching band: MAS cllcResonant/clllcResonant min/maxSwitchingFrequency, or the resonantBandMin/Max
    // keys the web runtime sends for every resonant topology. Both spellings stated and disagreeing -> refuse.
    auto bandEnd = [&](const char* masKey, const char* aliasKey) -> std::optional<double> {
        const bool hasMas = config.contains(masKey), hasAlias = config.contains(aliasKey);
        if (!hasMas && !hasAlias) return std::nullopt;
        const double vMas = hasMas ? cfg::get(config, masKey, 0.0) : 0.0;
        const double vAlias = hasAlias ? cfg::get(config, aliasKey, 0.0) : 0.0;
        if (hasMas && hasAlias && std::abs(vMas - vAlias) > 1e-9 * std::max(std::abs(vMas), std::abs(vAlias)))
            throw std::invalid_argument(who + ": config." + masKey + " = " + num(vMas) + " Hz and config." + aliasKey +
                                        " = " + num(vAlias) + " Hz state two different switching bands");
        const double v = hasMas ? vMas : vAlias;
        if (!(v > 0.0))
            throw std::invalid_argument(who + ": config." + (hasMas ? masKey : aliasKey) + " must be > 0; got " + num(v));
        return v;
    };
    const auto fmin = bandEnd("minSwitchingFrequency", "resonantBandMin");
    const auto fmax = bandEnd("maxSwitchingFrequency", "resonantBandMax");
    if (fmin && fmax && !(*fmax >= *fmin))
        throw std::invalid_argument(who + ": the switching-frequency band [" + num(*fmin) + ", " + num(*fmax) +
                                    "] Hz is not a band (need min <= max)");

    // The tank as the DRIVING bridge sees it. Forward: the Vin side drives Lr1/Cr1, Lm, and Lr2/Cr2 referred up by
    // N², into Rac = (8/π²)·N²·Vout²/P. Reverse: the Vout side drives Lr2/Cr2, Lm/N², and Lr1/Cr1 referred down,
    // into (8/π²)·Vin²/(N²·P). Required gain = delivered rail referred to the driver / (η·driving bus).
    double Lm = magnetizingInductance, Lr1 = primaryResonantInductance, Cr1 = primaryResonantCapacitance;
    double Lr2p = secondaryResonantInductance * N * N, Cr2p = secondaryResonantCapacitance / (N * N);
    double Rac = (8.0 / (M_PI * M_PI)) * N * N * outputVoltage * outputVoltage / outputPower;
    double requiredGain = N * outputVoltage / (eta * inputVoltage);
    double ratioAtResonance = eta * inputVoltage / outputVoltage;   // gain 1: n = η·Vin/Vout
    if (reverse) {
        Lm = magnetizingInductance / (N * N);
        Lr1 = secondaryResonantInductance; Cr1 = secondaryResonantCapacitance;
        Lr2p = primaryResonantInductance / (N * N); Cr2p = primaryResonantCapacitance * N * N;
        Rac = (8.0 / (M_PI * M_PI)) * inputVoltage * inputVoltage / (N * N * outputPower);
        requiredGain = inputVoltage / (eta * N * outputVoltage);
        ratioAtResonance = inputVoltage / (eta * outputVoltage);
    }
    TwoSidedOperatingPoint op{fr, requiredGain};
    // At fr the requirement is already met (a unity-gain ratio behind a symmetric tank): run there, unsolved.
    const double gainAtResonance = AN::cllc_fha_tank(fr, Lm, Lr1, Cr1, Lr2p, Cr2p, Rac).gain();
    const bool frInBand = (!fmin || fr >= *fmin) && (!fmax || fr <= *fmax);
    if (frInBand && std::abs(gainAtResonance - requiredGain) <= 1e-9 * requiredGain) return op;
    try {
        op.operatingFrequency = AN::solve_cllc_operating_frequency(requiredGain, fmin, fmax, Lm, Lr1, Cr1, Lr2p, Cr2p, Rac);
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument(
            who + ": the " + ratioOrigin + " " + num(N) + " cannot deliver " +
            (reverse ? num(inputVoltage) + " V to the HV side from " + num(outputVoltage) + " V (reverse power flow)"
                     : num(outputVoltage) + " V from " + num(inputVoltage) + " V") +
            ": " + e.what() + ". A turns ratio of " + num(ratioAtResonance) +
            " gives the output at the tank resonance (" + num(fr) + " Hz)");
    }
    return op;
}

json build_cllc_tas(const CllcDesign& d) {
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
    // fill DEFERS a requirement-less diode as a FET body diode. REAL switches take a `req`.
    auto diode  = [&](json reqs = json()) { json j; j["semiconductor"]["diode"] = json::object();
        j["inputs"]["designRequirements"] = reqs.is_null()
            ? req::body_diode(d.inputVoltage, d.outputPower / d.inputVoltage) : reqs; return j; };
    auto mosfetReq = [](const json& r) { json j; j["semiconductor"]["mosfet"] = json::object();
        j["inputs"]["designRequirements"] = r; return j; };

    const double n = d.turnsRatio;

    auto capBrick = [&](double c, double vrated) { json j; j["capacitor"] = json::object();
        j["inputs"]["designRequirements"]["capacitance"]["nominal"] = c;
        j["inputs"]["designRequirements"]["ratedVoltage"] = vrated; return j; };
    auto resBrick = [&](double r) { json j; j["resistor"] = json::object();
        auto& dr = j["inputs"]["designRequirements"];
        dr["deviceType"] = "resistor";
        dr["resistance"]["nominal"] = r;
        // Conservative requirement floor: a resistor dissipates I^2*R (series) or V^2/R (shunt);
        // the physical value is the smaller. Exact for load resistors (=Pout), safe for sense/divider.
        const double Iout_ = d.outputPower / d.outputVoltage, Vb_ = d.outputVoltage;
        const double i2r_ = Iout_*Iout_*r, v2r_ = Vb_*Vb_/r;
        dr["powerRating"] = (i2r_ < v2r_ ? i2r_ : v2r_);
        return j; };

    // --- resonant-tank stresses (FHA, evaluated at the nominal point, operated AT resonance fr) ---
    // CLLC is a FULL bridge both sides, so the primary tank sees a ±Vin square (fund. rms 2√2·Vin/π).
    // Tank is SINUSOIDAL → every magnetic excitation is a sine ("sinusoidal", vRms=vPk/√2, vPkPk=2·vPk,
    // offset 0). Primary tank current = real load current (Pin/Vtank1_rms) + reactive magnetizing
    // (Lm sees ±Vin, triangle peak Vin·(T/4)/Lm). The secondary tank/winding carries the reflected real
    // load current ×n (n=Vin/Vo>1 here).
    // Evaluated at the OPERATING frequency design_cllc settled (fr when the ratio gives the output there, else the
    // FHA-solved frequency in the band — ABT #1503); the tank itself still resonates at d.resonantFrequency.
    const double fr   = d.operatingFrequency, Tfr = 1.0 / fr;
    const double Pin  = d.outputPower / d.efficiency;
    // The DRIVING bus sets the tank fundamental: forward drives the Vin side, reverse drives the Vout side
    // (ABT #85). The symmetric tank makes the sizing identical — only WHICH winding is the driver (carrying
    // the real load current + the magnetizing current) vs the receiver (carrying the reflected real load
    // current) flips. ItankX names the Vin-side winding + primary tank Lr1; IsecX the Vout-side winding + Lr2.
    const double Vdrive        = d.reverse ? d.outputVoltage : d.inputVoltage;
    const double VtankDriveRms = 2.0 * std::sqrt(2.0) * Vdrive / M_PI;      // fund. rms of the ±Vdrive square
    const double IloadRms      = Pin / VtankDriveRms;                       // real current in the driving winding
    const double LmDrive       = d.reverse ? d.magnetizingInductance / (n * n) : d.magnetizingInductance;  // Lm ref. to driver
    const double ImagPk        = Vdrive * (Tfr / 4.0) / LmDrive;            // Lm triangle pk (driver side)
    const double ImagRms       = ImagPk / std::sqrt(3.0);
    const double IdriveRms     = std::sqrt(IloadRms * IloadRms + ImagRms * ImagRms);  // driving winding total
    const double IrecvRms      = IloadRms * (d.reverse ? 1.0 / n : n);      // reflected to the receiving winding
    const double ItankRms  = d.reverse ? IrecvRms : IdriveRms;             // Vin-side winding (Lr1 + Q1..Q4)
    const double ItankPk   = std::sqrt(2.0) * ItankRms, ItankPkPk = 2.0 * ItankPk;
    const double IsecRms   = d.reverse ? IdriveRms : IrecvRms;             // Vout-side winding (Lr2 + Qa..Qd)
    const double IsecPk    = std::sqrt(2.0) * IsecRms, IsecPkPk = 2.0 * IsecPk;
    // Resonant-inductor winding voltages (sinusoidal at fr): each Lr sees i·Z (Z=2π·fr·L). The TRANSFORMER
    // winding voltages come from the embedded analytical excitations below, not an inline reflected clamp.
    const double Zr1    = 2.0 * M_PI * fr * d.primaryResonantInductance;
    const double Zr2    = 2.0 * M_PI * fr * d.secondaryResonantInductance;
    const double vLr1Pk = ItankPk * Zr1, vLr1Rms = vLr1Pk / std::sqrt(2.0), vLr1PkPk = 2.0 * vLr1Pk;
    const double vLr2Pk = IsecPk * Zr2,  vLr2Rms = vLr2Pk / std::sqrt(2.0), vLr2PkPk = 2.0 * vLr2Pk;

    // --- semiconductor requirements (sourceable). All rectification is ACTIVE (synchronous-rectifier
    // MOSFETs both sides); the only diodes (DS1..DS4, DSa..DSd) are FET body diodes -> left bare/deferred.
    // Primary full-bridge Q1..Q4: each blocks the full bus Vin when off, carries the primary tank current
    // (peak ItankPk, rms ItankRms — the same primary current that drives the magnetic).
    const double ratedVdsPri = d.inputVoltageMax / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnPri  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (ItankRms * ItankRms);
    const json reqPri = req::mosfet("mainSwitch", ratedVdsPri, ItankPk, maxRdsOnPri, 125.0);
    // Secondary SR full-bridge Qa..Qd: each blocks the output rail Vout when off, carries the secondary
    // (reflected) tank current (peak IsecPk, rms IsecRms).
    const double ratedVdsSec = d.outputVoltage / cfg::v_derate_mosfet(d.config);
    const double maxRdsOnSec  = cfg::rds_on_loss_fraction(d.config) * d.outputPower / (IsecRms * IsecRms);
    const json reqSec = req::mosfet("mainSwitch", ratedVdsSec, IsecPk, maxRdsOnSec, 125.0);

    json cr1 = capBrick(d.primaryResonantCapacitance, d.inputVoltage * 2);
    // RESONANT caps set the tank frequency, so they must be sourced CLOSE to nominal — the default fill
    // treats capacitance as a ripple MINIMUM and oversizes up to 2x (and may pick a lossy electrolytic),
    // which detunes the CLLC tank (a 1.9nF Cr1 sourced at 3.3nF dropped fr 126→97kHz, below the regulator's
    // bracket, so the converter could never boost to target). role=resonant tells the HS fill to pick the
    // NEAREST value with a proper (film) dielectric, not oversize (abt #54, as the LLC Cr already does).
    cr1["inputs"]["designRequirements"]["role"] = "resonant";
    // Primary resonant inductor Lr1: its OWN single-winding magnetic (full primary tank current).
    json lr1; lr1["magnetic"] = json::object();
    lr1["inputs"] = req::magnetic_inputs(d.primaryResonantInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, ItankPk, ItankRms, 0.0, ItankPkPk, std::nullopt,
                                    vLr1Pk, vLr1Rms, 0.0, vLr1PkPk)});
    // Secondary resonant inductor Lr2: its OWN single-winding magnetic (reflected secondary tank current).
    json lr2; lr2["magnetic"] = json::object();
    lr2["inputs"] = req::magnetic_inputs(d.secondaryResonantInductance, 0.2, {}, {"primary"},
        std::nullopt, 25.0, {
            req::winding_excitation("sinusoidal", fr, IsecPk, IsecRms, 0.0, IsecPkPk, std::nullopt,
                                    vLr2Pk, vLr2Rms, 0.0, vLr2PkPk)});
    json cr2 = capBrick(d.secondaryResonantCapacitance, d.outputVoltage * 2);
    cr2["inputs"]["designRequirements"]["role"] = "resonant";   // secondary tank cap — source near nominal (abt #54)
    json cout = capBrick(d.outputCapacitance, d.outputVoltage * 2);

    // Transformer: primary Lpri = Lm, single secondary, turnsRatios=[n], K=0.9999.
    // 2 physical windings = turnsRatios.size()+1: primary (tank current) + secondary (reflected ×n).
    // Transformer (T1) EMBEDDED excitations from the SINGLE FHA source (the SPICE-validated analytical CLLC
    // solver): full-bridge drive (bridgeVoltageFactor=1.0) -> primary + single secondary (2 windings),
    // matching. Lr1/Lr2 (primary/secondary resonant inductors) and the switch/diode ratings keep the inline
    // tank FHA (they are not transformer windings). (CLLC FHA is ~0.25 NRMSE off-unity — documented limit.)
    namespace AN = Kirchhoff::analytical;
    std::vector<std::string> isoSides{"primary", "secondary"};
    MAS::OperatingPoint aopT1;
    if (!d.reverse) {
        // Forward: the Vin-side full bridge drives (bridgeVoltageFactor=1.0) -> primary + single secondary.
        const double IoutT = d.outputPower / d.outputVoltage;
        aopT1 = AN::analytical_cllc(d.inputVoltage, {d.outputVoltage}, {IoutT}, {n}, fr,
            d.magnetizingInductance, d.primaryResonantInductance, d.primaryResonantCapacitance,
            d.secondaryResonantInductance, d.secondaryResonantCapacitance, 1.0, AN::SrcRectifier::FULL_BRIDGE);
    } else {
        // Reverse (ABT #85): the Vout (LV) side drives and power flows to the Vin (HV) side. Reflect the FHA
        // about the symmetric tank — the driving tank is the Vout-side tank (Lr2/Cr2), Lm is referred to the
        // Vout winding (Lm/n²), and the driver->receiver turns ratio is 1/n. analytical_cllc returns
        // [Primary(driver = Vout winding), Secondary(receiver = Vin winding)]; T1's physical windings are
        // [primary(Vin), secondary(Vout)], so place the receiver excitation on the primary and the driver on
        // the secondary (reorder to physical winding order before capture, so the registry stays consistent).
        const double IinT = d.outputPower / d.inputVoltage;
        MAS::OperatingPoint raw = AN::analytical_cllc(d.outputVoltage, {d.inputVoltage}, {IinT}, {1.0 / n}, fr,
            d.magnetizingInductance / (n * n), d.secondaryResonantInductance, d.secondaryResonantCapacitance,
            d.primaryResonantInductance, d.primaryResonantCapacitance, 1.0, AN::SrcRectifier::FULL_BRIDGE);
        auto& ex = raw.get_mutable_excitations_per_winding();
        if (ex.size() >= 2) {
            MAS::OperatingPointExcitation vinW = ex[1];   vinW.set_name(std::string("Primary"));       // Vin = receiver
            MAS::OperatingPointExcitation voutW = ex[0];  voutW.set_name(std::string("Secondary 0"));  // Vout = driver
            // MAS convention: the primary-side (Vin) winding is PASSIVE and the secondary (Vout) winding SOURCE.
            // The raw solve modelled the Vout winding as the driven primary (passive) and the Vin winding as
            // its load (source), so both currents flip sign when the windings return to their physical
            // sides. Voltages are already in the common dot reference and stay as they are.
            vinW = Kirchhoff::analytical::with_negated_current(vinW);
            voutW = Kirchhoff::analytical::with_negated_current(voutW);
            aopT1.get_mutable_excitations_per_winding().push_back(vinW);
            aopT1.get_mutable_excitations_per_winding().push_back(voutW);
        } else {
            aopT1 = raw;   // degenerate (shouldn't happen for a full-bridge rectifier) — keep as-is
        }
    }
    json t1; t1["magnetic"] = json::object();
    t1["inputs"] = req::magnetic_inputs(d.magnetizingInductance, 0.1, {n}, isoSides,
        std::nullopt, 25.0, AN::excitations_processed(aopT1, "T1"));
    // MAS cllcResonant.integratedResonantInductor1/2 (default true): the resonant inductor realised as the
    // transformer's own leakage. The transformer must then REALISE that leakage, primary-referred:
    // Lr1 (primary side) + n²·Lr2 (secondary side); the discrete Lr1/Lr2 are folded out of the cell below.
    {
        const bool int1 = cfg::get_bool(d.config, "integratedResonantInductor1", false);
        const bool int2 = cfg::get_bool(d.config, "integratedResonantInductor2", false);
        if (int1 || int2) {
            double leakage = 0.0;
            if (int1) leakage += d.primaryResonantInductance;
            if (int2) leakage += n * n * d.secondaryResonantInductance;
            req::set_leakage_requirement(t1["inputs"], {leakage}, "nominal");
        }
    }

    json cell; cell["name"] = "cllc-cell";
    cell["ports"] = json::array({port("vin"), port("gnd"), port("sgnd"), port("vout"),
                                 port("g1"), port("g2"), port("g3"), port("g4"),
                                 port("senseP"), port("senseM")});
    cell["components"] = json::array({
        // primary full bridge + body diodes (DS1..DS4 = Q1..Q4 body diodes -> bare seed, deferred)
        comp("Q1", mosfetReq(reqPri)), comp("Q2", mosfetReq(reqPri)),
        comp("Q3", mosfetReq(reqPri)), comp("Q4", mosfetReq(reqPri)),
        comp("DS1", diode()), comp("DS2", diode()), comp("DS3", diode()), comp("DS4", diode()),
        // primary tank + transformer + secondary tank
        comp("Cr1", cr1), comp("Lr1", lr1), comp("T1", t1), comp("Lr2", lr2), comp("Cr2", cr2),
        // in-line secondary-tank current sense: the SR controller reads the tank-current sign across it
        comp("Rsense", resBrick(cfg::get(d.config, "senseResistance", kSenseResistance))),
        // secondary active synchronous rectifier (4 switches) + body diodes (DSa..DSd = Qa..Qd body
        // diodes -> bare seed, deferred; the rectifiers are the SR MOSFETs themselves)
        comp("Qa", mosfetReq(reqSec)), comp("Qb", mosfetReq(reqSec)),
        comp("Qc", mosfetReq(reqSec)), comp("Qd", mosfetReq(reqSec)),
        // DSa..DSd = Qa..Qd (SECONDARY SR bridge) body diodes — rated to their host secondary FETs
        // (block Vout, carry IsecPk), NOT the primary rating a bare diode() would default to
        comp("DSa", diode(req::body_diode(ratedVdsSec, IsecPk))), comp("DSb", diode(req::body_diode(ratedVdsSec, IsecPk))),
        comp("DSc", diode(req::body_diode(ratedVdsSec, IsecPk))), comp("DSd", diode(req::body_diode(ratedVdsSec, IsecPk))),
        comp("Cout", cout)});
    cell["connections"] = json::array({
        // ── Primary full bridge. Diagonal pairs (Q1,Q4) on g1, (Q2,Q3) on g2 -> vab=±Vin.
        conn("vin_net",  {pin("Q1", "drain"), pin("Q3", "drain"),
                          pin("DS1", "cathode"), pin("DS3", "cathode"), prt("vin")}),
        conn("node_a",   {pin("Q1", "source"), pin("Q2", "drain"),
                          pin("DS1", "anode"), pin("DS2", "cathode"), pin("Cr1", "1")}),
        conn("node_b",   {pin("Q3", "source"), pin("Q4", "drain"),
                          pin("DS3", "anode"), pin("DS4", "cathode"), pin("T1", "primary_end")}),
        // Primary series tank: node_a -> Cr1 -> Lr1 -> Lpri(=Lm) -> node_b.
        conn("c1_mid",   {pin("Cr1", "2"), pin("Lr1", "primary_start")}),
        conn("pri_top",  {pin("Lr1", "primary_end"), pin("T1", "primary_start")}),
        // Secondary series tank: sec_p -> Lr2 -> Cr2 -> senseP -> Rsense -> node_c ; sec_n -> node_d.
        conn("sec_p",    {pin("T1", "secondary1_start"), pin("Lr2", "primary_start")}),
        conn("l2_mid",   {pin("Lr2", "primary_end"), pin("Cr2", "1")}),
        conn("senseP",   {pin("Cr2", "2"), pin("Rsense", "1"), prt("senseP")}),
        conn("node_c",   {pin("Rsense", "2"), pin("Qa", "source"), pin("Qb", "drain"),
                          pin("DSa", "anode"), pin("DSb", "cathode"), prt("senseM")}),
        conn("node_d",   {pin("T1", "secondary1_end"), pin("Qc", "source"), pin("Qd", "drain"),
                          pin("DSc", "anode"), pin("DSd", "cathode")}),
        // ── Secondary active bridge. Diagonal pairs (Qa,Qd) on g3, (Qb,Qc) on g4 — its OWN gate nets, not
        // the primary's (ABT #1525). Forward, srControl gates them on the sign of the secondary tank current
        // (synchronous rectifier); the body diodes DSa..DSd carry the dead time around each current zero.
        conn("vout_net", {pin("Qa", "drain"), pin("Qc", "drain"),
                          pin("DSa", "cathode"), pin("DSc", "cathode"), pin("Cout", "1"), prt("vout")}),
        // Primary and secondary returns are DIFFERENT nodes (ABT #778). One gnd_net here put the
        // primary bridge's low-side sources on the same net as the secondary SR bridge's, so every
        // winding of T1 sat on one galvanic island and the drawn barrier did not exist.
        conn("gnd_net",  {pin("Q2", "source"), pin("Q4", "source"),
                          pin("DS2", "anode"), pin("DS4", "anode"), prt("gnd")}),
        conn("sgnd_net", {pin("Qb", "source"), pin("Qd", "source"),
                          pin("DSb", "anode"), pin("DSd", "anode"), pin("Cout", "2"), prt("sgnd")}),
        conn("g1_net", {pin("Q1", "gate"), pin("Q4", "gate"), prt("g1")}),
        conn("g2_net", {pin("Q2", "gate"), pin("Q3", "gate"), prt("g2")}),
        conn("g3_net", {pin("Qa", "gate"), pin("Qd", "gate"), prt("g3")}),
        conn("g4_net", {pin("Qb", "gate"), pin("Qc", "gate"), prt("g4")})});
    // Integrated resonant inductors (MAS cllcResonant.integratedResonantInductor1/2) are realised as T1's
    // leakage (requirement set on T1 above): fold the discrete equivalents out of the cell.
    if (cfg::get_bool(d.config, "integratedResonantInductor1", false)) req::fold_series_inductor(cell, "Lr1");
    if (cfg::get_bool(d.config, "integratedResonantInductor2", false)) req::fold_series_inductor(cell, "Lr2");

    // ──────────────────── SR CONTROL stage (swappable) ────────────────────
    // ONE CTAS `controller` component — a current-sensed full-bridge synchronous-rectifier controller, the same
    // one CLLLC uses. The assembler lowers its agnostic behavioural law (CTAS controller.behavioral) into two
    // comparators that read the tank-current sign across Rsense (senseP/senseM) and gate the two rectifying
    // diagonals (gA/gB), so the SR follows the tank current at any operating frequency (ABT #1525).
    auto syncRect = [&](double hyst) { json j; json& b = j["controller"]["behavioral"];
        b["controlScheme"] = "synchronousRectifier"; b["topology"] = "fullBridge"; b["sensing"] = "current";
        b["hysteresis"] = hyst; b["driveHigh"] = 5.0; b["driveLow"] = 0.0; b["threshold"] = 0.0; return j; };
    json ccell; ccell["name"] = "cllc-sr-control";
    ccell["ports"] = json::array({port("senseP"), port("senseM"), port("gA"), port("gB")});
    ccell["components"] = json::array({comp("SR", syncRect(cfg::get(d.config, "senseHysteresis", kSenseHysteresis)))});
    ccell["connections"] = json::array({
        conn("senseP", {pin("SR","senseP"), prt("senseP")}),
        conn("senseM", {pin("SR","senseM"), prt("senseM")}),
        conn("gA", {pin("SR","gA"), prt("gA")}), conn("gB", {pin("SR","gB"), prt("gB")})});

    // The TAS inputs describe the SOURCE and DELIVERED rails. Forward: source = Vin, deliver = Vout. Reverse
    // (ABT #85): source = Vout (LV), deliver = Vin (HV). The assembler drives a DC source on the "input"
    // external port at inputVoltage and sizes the load on the "output" port from outputs[0], so flipping
    // these (together with the interStageConnection directions below) realises reverse power flow.
    const double srcV    = d.reverse ? d.outputVoltage : d.inputVoltage;
    const double srcVmin = d.reverse ? d.outputVoltage : d.inputVoltageMin;
    const double srcVmax = d.reverse ? d.outputVoltage : d.inputVoltageMax;
    const double deliverV = d.reverse ? d.inputVoltage : d.outputVoltage;
    json tas;
    json& dreq = tas["inputs"]["designRequirements"];
    dreq["efficiency"] = d.efficiency;
    dreq["inputType"] = "dc";
    dreq["inputVoltage"] = {{"minimum", srcVmin}, {"nominal", srcV}, {"maximum", srcVmax}};
    // The frequency the converter actually switches at (ABT #1503), so the diagnostics report the operating point
    // that was computed rather than the request (which is the tank resonance).
    dreq["switchingFrequency"]["nominal"] = d.operatingFrequency;
    { json o; o["name"] = "out"; o["voltage"]["nominal"] = deliverV; o["regulation"] = "voltage";
      dreq["outputs"] = json::array({o}); }
    { json op; op["name"] = "full_load"; op["inputVoltage"] = srcV; op["ambientTemperature"] = 25.0;
      json o; o["name"] = "out"; o["power"] = d.outputPower; op["outputs"] = json::array({o});
      tas["inputs"]["operatingPoints"] = json::array({op}); }

    tas["topology"]["stages"] = json::array({
        req::control_stage("llcController"),
        pstage("cllcCell", "switchingCell", cell, bind("vin", "dcBus"), bind("vout", "dcOutput")),
        pstage("srControl", "control", ccell, bind("senseP", "sense"), bind("gA", "drive"))});
    // Node names stay tied to the physical rails (Vin = HV bridge port, Vout = LV bridge port); only the
    // source/load DIRECTION flips for reverse power flow (ABT #85).
    json powerRails = d.reverse
        ? json::array({
            isc("Vout", "externalPort", "input",  {sp("cllcCell", "vout")}),   // LV rail sources
            isc("GND",  "externalPort", "input",  {sp("cllcCell", "gnd")}),
            isc("SGND", "externalPort", "input",  {sp("cllcCell", "sgnd")}),
            isc("Vin",  "externalPort", "output", {sp("cllcCell", "vin")})})    // HV rail delivered
        : json::array({
            isc("Vin",  "externalPort", "input",  {sp("cllcCell", "vin")}),
            isc("GND",  "externalPort", "input",  {sp("cllcCell", "gnd")}),
            isc("SGND", "externalPort", "input",  {sp("cllcCell", "sgnd")}),
            isc("Vout", "externalPort", "output", {sp("cllcCell", "vout")})});
    // Tank-current sense: power -> control. Forward, the SR controller drives the secondary diagonals: gA -> g3
    // (Qa,Qd) conducts for a positive senseP->senseM current, gB -> g4 (Qb,Qc) for a negative one (ABT #1525).
    // Reverse, the secondary bridge is the DRIVEN one (stimulus on g3/g4) and gA/gB stay unwired.
    powerRails.push_back(isc("senseP", "wire", "", {sp("cllcCell", "senseP"), sp("srControl", "senseP")}));
    powerRails.push_back(isc("senseM", "wire", "", {sp("cllcCell", "senseM"), sp("srControl", "senseM")}));
    if (!d.reverse) {
        powerRails.push_back(isc("srGateA", "wire", "", {sp("srControl", "gA"), sp("cllcCell", "g3")}));
        powerRails.push_back(isc("srGateB", "wire", "", {sp("srControl", "gB"), sp("cllcCell", "g4")}));
    }
    tas["topology"]["interStageConnections"] = std::move(powerRails);

    json an; an["type"] = "transient"; an["stopTime"] = cfg::tran_stop_time(d.config, 0.004); an["maximumTimeStep"] = cfg::tran_max_timestep(d.config, 5e-8);
    tas["simulation"]["analyses"] = json::array({an});
    // Only the SOURCE-side bridge is open-loop driven (ABT #1525). Forward: g1 (Q1,Q4) phase 0 / g2 (Q2,Q3) phase
    // 180; the secondary bridge is the synchronous rectifier, gated by srControl on the sign of its tank current.
    // It used to be gated in lock-step with the primary, which is only right AT the tank resonance, where the
    // receiving current is in phase with the drive. Since ABT #1503 the converter runs at the FHA-solved frequency,
    // often off resonance; there the lock-step FETs conducted against the current (a 3.3 kW 400 V -> 400 V design
    // solved to 139 kHz sagged ~11% below its target).
    // Reverse: the secondary bridge is driven, g3 (Qa,Qd) phase 0 / g4 (Qb,Qc) phase 180, and the primary (HV)
    // bridge rectifies through its body diodes (its gates held off: dutyCycle 0 is a DC 0 V gate). The deck has
    // no primary-side current sense for an SR there; one diode drop on the HV rail is a small fraction of it.
    // ONE stimulus per gate net (a voltage source per switch would short the net's shared gate node).
    auto stim = [&](const char* sw, double phaseDeg, double duty) {
        json st; st["stage"] = "cllcCell"; st["component"] = sw; st["signal"] = "gate";
        st["waveform"]["type"] = "pwm"; st["waveform"]["frequency"] = d.operatingFrequency;
        st["waveform"]["dutyCycle"] = duty; st["waveform"]["phase"] = phaseDeg;
        return st; };
    tas["simulation"]["stimulus"] = d.reverse
        ? json::array({stim("Qa", 0.0, d.switchDuty), stim("Qb", 180.0, d.switchDuty),
                       stim("Q1", 0.0, 0.0), stim("Q2", 0.0, 0.0)})
        : json::array({stim("Q1", 0.0, d.switchDuty), stim("Q2", 180.0, d.switchDuty)});
    // Precharge the DELIVERED bus to its target so the SR has a rail to rectify into from the first cycle and the deck
    // runs with use-initial-conditions (skipping the resonant tank's singular DC operating point). Forward
    // precharges Vout; reverse precharges Vin (the HV side is now the delivered rail — ABT #85).
    { json ic; ic["node"] = d.reverse ? "Vin" : "Vout"; ic["voltage"] = deliverV;
      tas["simulation"]["initialConditions"] = json::array({ic}); }
    req::finalize_control_seeds(tas, Topology::CLLC_RESONANT_CONVERTER);  // CTAS seed: topology+fsw for switching controllers
    return tas;
}

} // namespace Kirchhoff
