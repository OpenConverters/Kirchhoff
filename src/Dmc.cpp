#include "Dmc.hpp"
#include "ChokeDesign.hpp"            // shared reactance math + filter_choke_requirements + make_inputs
#include "DimensionJson.hpp"          // PEAS::resolve_dimensional_values
#include "ConverterAnalytical.hpp"    // analytical_differential_mode_choke — the excitation half

#include <cmath>
#include <limits>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

namespace {

// configuration string → winding count (MKF DifferentialModeChoke::get_number_of_windings).
int windings_for(MAS::Configuration cfg) {
    switch (cfg) {
        case MAS::Configuration::SINGLE_PHASE:            return 1;
        case MAS::Configuration::SINGLE_PHASE_BALANCED:   return 2;
        case MAS::Configuration::THREE_PHASE:             return 3;
        case MAS::Configuration::THREE_PHASE_WITH_NEUTRAL:return 4;
    }
    return 1;
}

// MAS::Configuration → the analytical excitation enum (same order/meaning).
analytical::DmcConfiguration analytical_config(MAS::Configuration cfg) {
    switch (cfg) {
        case MAS::Configuration::SINGLE_PHASE:            return analytical::DmcConfiguration::SINGLE_PHASE;
        case MAS::Configuration::SINGLE_PHASE_BALANCED:   return analytical::DmcConfiguration::SINGLE_PHASE_BALANCED;
        case MAS::Configuration::THREE_PHASE:             return analytical::DmcConfiguration::THREE_PHASE;
        case MAS::Configuration::THREE_PHASE_WITH_NEUTRAL:return analytical::DmcConfiguration::THREE_PHASE_WITH_NEUTRAL;
    }
    return analytical::DmcConfiguration::SINGLE_PHASE;
}

const char* config_name(MAS::Configuration cfg) {
    switch (cfg) {
        case MAS::Configuration::SINGLE_PHASE:            return "singlePhase";
        case MAS::Configuration::SINGLE_PHASE_BALANCED:   return "singlePhaseBalanced";
        case MAS::Configuration::THREE_PHASE:             return "threePhase";
        case MAS::Configuration::THREE_PHASE_WITH_NEUTRAL:return "threePhaseWithNeutral";
    }
    return "singlePhase";
}

}  // namespace

// ═══ dmc_required_inductance — ported verbatim from MKF calculate_required_inductance (:549) ═════════

double dmc_required_inductance(double targetAttenuationDb, double frequencyHz, double capacitanceF) {
    // A = 40·log10(f/fc) → fc = f / 10^(A/40); fc = 1/(2π√LC) → L = 1/(4π²fc²C).
    const double cutoffFrequency = frequencyHz / std::pow(10.0, targetAttenuationDb / 40.0);
    return 1.0 / (4.0 * M_PI * M_PI * cutoffFrequency * cutoffFrequency * capacitanceF);
}

// ═══ design_dmc — ported from MKF DifferentialModeChoke(json) ctor (:14) + process_design_requirements
// (:37). The inductance requirement comes from minimumImpedance (L at the LOWEST spec frequency) or a
// direct minimumInductance; diagnostics mirror the calculate_dmc_inputs dmcDiagnostics block. ═════════

DmcDesign design_dmc(const json& spec) {
    // MKF ctor's explicit required-field guards (clearer than a bare j.at()).
    for (const char* k : {"lineFrequency", "inputVoltage", "operatingCurrent", "ambientTemperature"})
        if (!spec.contains(k))
            throw std::invalid_argument(std::string("design_dmc: '") + k + "' is required");

    DmcDesign d{};
    d.inputVoltage = PEAS::resolve_dimensional_values(spec.at("inputVoltage"));
    d.operatingCurrent = spec.at("operatingCurrent").get<double>();
    d.lineFrequency = spec.at("lineFrequency").get<double>();
    d.ambientTemperature = spec.at("ambientTemperature").get<double>();
    if (spec.contains("switchingFrequency"))
        d.switchingFrequency = spec.at("switchingFrequency").get<double>();
    d.configuration = spec.contains("configuration")
        ? spec.at("configuration").get<MAS::Configuration>()   // MAS from_json maps the schema strings
        : MAS::Configuration::SINGLE_PHASE;
    d.numberOfWindings = windings_for(d.configuration);
    if (spec.contains("peakCurrent")) d.peakCurrent = spec.at("peakCurrent").get<double>();

    if (d.operatingCurrent <= 0)
        throw std::invalid_argument("design_dmc: operatingCurrent must be > 0");
    if (d.lineFrequency <= 0)
        throw std::invalid_argument("design_dmc: lineFrequency must be > 0");
    if (spec.contains("switchingFrequency") && d.switchingFrequency <= 0)
        throw std::invalid_argument("design_dmc: switchingFrequency must be > 0 when supplied");
    if (spec.contains("filterCapacitance")) {
        d.filterCapacitance = spec.at("filterCapacitance").get<double>();
        if (*d.filterCapacitance <= 0)
            throw std::invalid_argument("design_dmc: filterCapacitance must be > 0 when supplied");
    }

    // Impedance spec → points + L_min at the LOWEST spec frequency (MKF :78-95, :108-123). NOT an
    // else-if against minimumInductance: an EMPTY minimumImpedance array (a UI serializing its blank
    // impedance table) must never shadow a supplied minimumInductance.
    if (spec.contains("minimumImpedance")) {
        double lowestFreq = std::numeric_limits<double>::max();
        double highestFreq = 0.0;
        double zAtLowest = 0.0;
        for (const auto& item : spec.at("minimumImpedance")) {
            const double f = item.at("frequency").get<double>();
            const double z = item.at("impedance").get<double>();
            d.impedancePoints.push_back(impedance_point(f, z));
            if (f < lowestFreq) { lowestFreq = f; zAtLowest = z; }
            if (f > highestFreq) highestFreq = f;
        }
        if (!d.impedancePoints.empty()) {
            d.computedInductance = impedance_to_inductance(zAtLowest, lowestFreq);
            d.computedMinFrequency = lowestFreq;
            d.computedMaxFrequency = highestFreq;
            d.computedImpedanceAtMinFreq = zAtLowest;
        }
    }
    if (spec.contains("minimumInductance")) {
        d.minimumInductance = spec.at("minimumInductance").get<double>();
        if (*d.minimumInductance <= 0)
            throw std::invalid_argument("design_dmc: minimumInductance must be > 0");
        // Both modes given → the stricter (larger) bound wins.
        d.computedInductance = std::max(d.computedInductance, *d.minimumInductance);
    }

    if (d.impedancePoints.empty() && !d.minimumInductance)
        throw std::invalid_argument(
            "design_dmc: needs an inductance target — supply minimumImpedance[] or minimumInductance "
            "(help-mode wizards call propose_dmc_design first, then re-call with the proposed L)");

    return d;
}

// ═══ build_dmc_inputs — shared filter-choke requirements (subApplication=differentialModeNoiseFiltering,
// topology=DIFFERENTIAL_MODE_CHOKE — MKF DMC DID set the topology, unlike CMC) + the analytical DMC
// operating point. ════════════════════════════════════════════════════════════════════════════════════

MAS::Inputs build_dmc_inputs(const DmcDesign& d) {
    if (d.computedInductance <= 0)
        throw std::invalid_argument("build_dmc_inputs: resolved DM inductance must be > 0");

    MAS::DimensionWithTolerance lmSpec;
    lmSpec.set_minimum(d.computedInductance);   // MKF emitted L_min un-rounded

    MAS::DesignRequirements dr = filter_choke_requirements(
        "differentialModeNoiseFiltering", MAS::Topology::DIFFERENTIAL_MODE_CHOKE,
        d.numberOfWindings, lmSpec, d.impedancePoints);

    // MKF require_input parity: the ripple spectrum is meaningless without the real switching
    // frequency — modelling it at 50/60 Hz would silently hand the adviser a wrong excitation.
    if (d.switchingFrequency <= 0)
        throw std::invalid_argument(
            "build_dmc_inputs: switchingFrequency is required (it sets the ripple spectrum the "
            "MagneticAdviser sizes core loss on)");

    const double peak = d.peakCurrent.value_or(std::numeric_limits<double>::quiet_NaN());
    MAS::OperatingPoint op = analytical::analytical_differential_mode_choke(
        d.operatingCurrent, d.inputVoltage, d.lineFrequency, d.switchingFrequency,
        analytical_config(d.configuration), peak, d.ambientTemperature);

    return make_inputs(std::move(dr), std::move(op));
}

// ═══ propose_dmc_design — analytical LC sizing, ported from MKF propose_design (:675) MINUS the ngspice
// verification block (that lives with the other ngspice sims). ═══════════════════════════════════════

json propose_dmc_design(const json& spec) {
    DmcDesign d = ([&] {
        // propose runs BEFORE an inductance target exists (help mode), so bypass design_dmc's
        // "need a target" guard by parsing the common fields directly here.
        for (const char* k : {"lineFrequency", "inputVoltage", "operatingCurrent", "ambientTemperature"})
            if (!spec.contains(k))
                throw std::invalid_argument(std::string("propose_dmc_design: '") + k + "' is required");
        DmcDesign r{};
        r.inputVoltage = PEAS::resolve_dimensional_values(spec.at("inputVoltage"));
        r.operatingCurrent = spec.at("operatingCurrent").get<double>();
        r.lineFrequency = spec.at("lineFrequency").get<double>();
        r.ambientTemperature = spec.at("ambientTemperature").get<double>();
        if (spec.contains("switchingFrequency")) r.switchingFrequency = spec.at("switchingFrequency").get<double>();
        r.configuration = spec.contains("configuration")
            ? spec.at("configuration").get<MAS::Configuration>() : MAS::Configuration::SINGLE_PHASE;
        r.numberOfWindings = windings_for(r.configuration);
        if (spec.contains("peakCurrent")) r.peakCurrent = spec.at("peakCurrent").get<double>();
        if (spec.contains("minimumImpedance"))
            for (const auto& item : spec.at("minimumImpedance"))
                r.impedancePoints.push_back(impedance_point(
                    item.at("frequency").get<double>(), item.at("impedance").get<double>()));
        return r;
    })();
    if (d.operatingCurrent <= 0)
        throw std::invalid_argument("propose_dmc_design: operatingCurrent must be > 0");

    const double loadImpedance = d.inputVoltage / d.operatingCurrent;

    // Target frequency + attenuation: the MOST DEMANDING impedance requirement — the point whose
    // required attenuation A = 20·log10(Z/Rload) forces the lowest LC cutoff fc = f/10^(A/40).
    // Satisfying it satisfies every other point on the 40 dB/dec slope. (Deviates from MKF, which
    // took minImpedance->front() — an order-dependent pick that could propose a filter missing the
    // user's harder requirement.) Falls back to the switching frequency at a default 40 dB.
    double targetFrequency = d.switchingFrequency;
    double targetAttenuation = 40.0;
    double lowestRequiredCutoff = std::numeric_limits<double>::max();
    for (const auto& pt : d.impedancePoints) {
        const double f = pt.get_frequency();
        const double atten = 20.0 * std::log10(pt.get_impedance().get_magnitude() / loadImpedance);
        const double requiredCutoff = f / std::pow(10.0, atten / 40.0);
        if (requiredCutoff < lowestRequiredCutoff) {
            lowestRequiredCutoff = requiredCutoff;
            targetFrequency = f;
            targetAttenuation = atten;
        }
    }
    if (!(targetFrequency > 0))
        throw std::invalid_argument(
            "propose_dmc_design: needs a target frequency — supply minimumImpedance[] or switchingFrequency");

    const double cutoffFrequency = targetFrequency / std::pow(10.0, targetAttenuation / 40.0);

    // Choose a practical capacitance (MKF :700-714): default via a 470 µH target inductance, clamped
    // to [100 nF, 10 µF], overridden by an explicit filterCapacitance.
    double capacitance = spec.value("filterCapacitance", 0.0);
    if (capacitance <= 0) {
        const double targetInductance = 470e-6;
        capacitance = 1.0 / (4.0 * M_PI * M_PI * cutoffFrequency * cutoffFrequency * targetInductance);
        capacitance = std::min(std::max(capacitance, 100e-9), 10e-6);
    }

    double inductance = 1.0 / (4.0 * M_PI * M_PI * cutoffFrequency * cutoffFrequency * capacitance);
    if (spec.contains("minimumInductance")) {
        const double minL = spec.at("minimumInductance").get<double>();
        if (minL > inductance) {
            inductance = minL;
            capacitance = 1.0 / (4.0 * M_PI * M_PI * cutoffFrequency * cutoffFrequency * inductance);
        }
    }

    // Peak current for saturation sizing: explicit, else operatingCurrent × (1 + 0.40) margin (MKF
    // resolve_peak_current(0.40), :732).
    const double peakCurrent = d.peakCurrent.value_or(d.operatingCurrent * 1.40);
    const double energyStorage = 0.5 * inductance * peakCurrent * peakCurrent;

    return json{
        {"inductance", inductance},
        {"inductance_uH", inductance * 1e6},
        {"capacitance", capacitance},
        {"capacitance_nF", capacitance * 1e9},
        {"cutoffFrequency", cutoffFrequency},
        {"cutoffFrequency_kHz", cutoffFrequency / 1e3},
        {"targetAttenuation_dB", targetAttenuation},
        {"peakCurrent", peakCurrent},
        {"energyStorage_mJ", energyStorage * 1e3},
        {"configuration", config_name(d.configuration)},
        {"numberOfWindings", d.numberOfWindings},
    };
}

} // namespace Kirchhoff
