#include "Cmc.hpp"
#include "ChokeDesign.hpp"            // shared reactance math + filter_choke_requirements + make_inputs
#include "DimensionJson.hpp"          // PEAS::resolve_dimensional_values — canonical {nominal,min,max} resolver
#include "ConverterAnalytical.hpp"    // analytical_common_mode_choke — the excitation half

#include <cmath>
#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

// ═══ CMC-specific spec conversions — ported from MKF CommonModeChoke.cpp:27-104 ═════════════════════

double cmc_insertion_loss_to_impedance(double insertionLossDb, double lineImpedanceOhms) {
    return lineImpedanceOhms * (std::pow(10.0, insertionLossDb / 20.0) - 1.0);
}

double cmc_noise_params_to_impedance(double parasiticCapPf,
                                     double dvdtVPerNs,
                                     double lineImpedanceOhms,
                                     double safetyMarginDb,
                                     double testFrequencyHz,
                                     double limitDbuv) {
    (void)testFrequencyHz;  // the impedance requirement is placed AT this frequency by the caller
    // CM current injected by the switching transients: I_cm = C_parasitic × dV/dt.
    const double icmA = (parasiticCapPf * 1e-12) * (dvdtVPerNs * 1e9);
    // CM noise voltage across half the LISN impedance, in dBµV (CISPR reference: 1 µV).
    const double vnoiseV = icmA * (lineImpedanceOhms / 2.0);
    const double vnoiseDbuv = 20.0 * std::log10(std::max(vnoiseV, 1e-9) / 1e-6);
    const double attenDb = std::max(0.0, vnoiseDbuv - limitDbuv + safetyMarginDb);
    return (lineImpedanceOhms / 2.0) * std::pow(10.0, attenDb / 20.0);
}

double cmc_emissions_limit_dbuv(const std::string& standardName) {
    // Quasi-peak conducted-emissions limits at 150 kHz (dBµV).
    if (standardName == "CISPR 32 Class A") return 79.0;
    if (standardName == "CISPR 32 Class B") return 66.0;
    if (standardName == "FCC Part 15 Class A") return 79.0;
    if (standardName == "FCC Part 15 Class B") return 66.0;
    throw std::invalid_argument(
        "cmc_emissions_limit_dbuv: unknown regulatory standard '" + standardName +
        "' (supported: CISPR 32 Class A/B, FCC Part 15 Class A/B)");
}

// ═══ design_cmc — ported from the MKF CommonModeChoke(json) constructor (CommonModeChoke.cpp:110)
// plus run_checks (:228). All three spec modes collapse into d.impedancePoints; the most demanding
// point (max L = Z/(2πf)) fixes computedInductance and the dominant excitation frequency. ════════════

CmcDesign design_cmc(const json& spec) {
    CmcDesign d{};
    d.operatingVoltage = PEAS::resolve_dimensional_values(spec.at("operatingVoltage"));
    d.operatingCurrent = spec.at("operatingCurrent").get<double>();
    d.lineFrequency = spec.at("lineFrequency").get<double>();
    d.ambientTemperature = spec.at("ambientTemperature").get<double>();
    // CISPR-16 / LISN reference impedance per line — the documented spec-level default (MKF
    // Defaults.h commonModeChokeLineImpedanceDefault), not a silent fabrication.
    d.lineImpedance = spec.value("lineImpedance", 50.0);
    d.numberOfWindings = spec.value("numberOfWindings", 2);

    if (d.numberOfWindings < 2 || d.numberOfWindings > 4)
        throw std::invalid_argument("design_cmc: numberOfWindings must be 2, 3, or 4");
    if (d.operatingCurrent <= 0)
        throw std::invalid_argument("design_cmc: operatingCurrent must be > 0");
    if (d.lineFrequency <= 0)
        throw std::invalid_argument("design_cmc: lineFrequency must be > 0");

    // Mode 1 — direct impedance points.
    if (spec.contains("minimumImpedance"))
        for (const auto& item : spec.at("minimumImpedance"))
            d.impedancePoints.push_back(
                impedance_point(item.at("frequency").get<double>(), item.at("impedance").get<double>()));

    // Mode 2 — insertion-loss targets, converted through the LISN line impedance.
    if (spec.contains("targetInsertionLoss"))
        for (const auto& item : spec.at("targetInsertionLoss"))
            d.impedancePoints.push_back(impedance_point(
                item.at("frequency").get<double>(),
                cmc_insertion_loss_to_impedance(item.at("insertionLoss").get<double>(), d.lineImpedance)));

    // Noise-estimation params. Two distinct effects (MKF CommonModeChoke.cpp:158-195):
    //   (A) they set the physical CM-excitation amplitude I_cm = C·dV/dt for the operating point,
    //       in EVERY spec mode;
    //   (B) only when the user gave no explicit impedance/insertion-loss spec, they synthesise the
    //       required-impedance point from the noise level vs the regulatory limit at 150 kHz.
    const bool hasParasiticCap = spec.contains("parasiticCap_pF");
    const bool hasDvdt = spec.contains("dvdt_V_ns");
    if (hasParasiticCap != hasDvdt)
        throw std::invalid_argument(
            "design_cmc: parasiticCap_pF and dvdt_V_ns must be supplied together — "
            "I_cm = C·dV/dt needs both");
    if (hasParasiticCap) {
        d.parasiticCapPf = spec.at("parasiticCap_pF").get<double>();
        d.dvdtVPerNs = spec.at("dvdt_V_ns").get<double>();
        if (d.parasiticCapPf <= 0 || d.dvdtVPerNs <= 0)
            throw std::invalid_argument("design_cmc: parasiticCap_pF and dvdt_V_ns must be > 0");
        if (d.impedancePoints.empty()) {
            const double safetyMarginDb = spec.value("safetyMargin_dB", 6.0);
            const double limitDbuv = cmc_emissions_limit_dbuv(
                spec.value("regulatoryStandard", std::string("CISPR 32 Class B")));
            const double testFrequencyHz = 150e3;  // EN 55032 / CISPR 32 conducted band start
            d.impedancePoints.push_back(impedance_point(
                testFrequencyHz,
                cmc_noise_params_to_impedance(d.parasiticCapPf, d.dvdtVPerNs, d.lineImpedance,
                                              safetyMarginDb, testFrequencyHz, limitDbuv)));
        }
    }

    // Required inductance: L = Z/(2πf) per point, take the maximum (the most demanding point).
    for (const auto& pt : d.impedancePoints) {
        const double L = impedance_to_inductance(pt.get_impedance().get_magnitude(), pt.get_frequency());
        if (L > d.computedInductance) {
            d.computedInductance = L;
            d.dominantFrequency = pt.get_frequency();
            d.dominantImpedance = pt.get_impedance().get_magnitude();
        }
    }

    // Advanced ("I know the design I want") mode: the user pins L; the excitation runs at
    // designFrequency (default 150 kHz — the CISPR 32 / FCC Part 15 lower band edge, where CMC
    // impedance is typically characterised). Mirrors MKF AdvancedCommonModeChoke.
    if (spec.contains("desiredInductance")) {
        d.desiredInductance = spec.at("desiredInductance").get<double>();
        if (*d.desiredInductance <= 0)
            throw std::invalid_argument("design_cmc: desiredInductance must be > 0");
    }
    d.designFrequency = spec.value("designFrequency", 150e3);

    if (!d.desiredInductance && d.impedancePoints.empty())
        throw std::invalid_argument(
            "design_cmc: at least one impedance requirement must be provided (minimumImpedance, "
            "targetInsertionLoss, or parasiticCap_pF+dvdt_V_ns), or pin desiredInductance");

    return d;
}

// ═══ build_cmc_inputs — the shared filter-choke requirements (ChokeDesign.hpp) with the CMC sub-
// application + the analytical_common_mode_choke operating point. ════════════════════════════════════

MAS::Inputs build_cmc_inputs(const CmcDesign& d) {
    const bool advanced = d.desiredInductance.has_value();
    const double lm = advanced ? *d.desiredInductance : d.computedInductance;
    if (lm <= 0)
        throw std::invalid_argument("build_cmc_inputs: resolved CM inductance must be > 0");

    // CM inductance requirement: a MINIMUM bound when derived from the impedance spec (the solver may
    // exceed it), a nominal target when the user pinned it (advanced mode).
    MAS::DimensionWithTolerance lmSpec;
    if (advanced)
        lmSpec.set_nominal(round_to(lm, 10));
    else
        lmSpec.set_minimum(round_to(lm, 10));

    // topology deliberately UNSET — MKF's CommonModeChoke::process_design_requirements did not set it
    // (unlike DMC/CT), and the CM path routes off can_be_common_mode_choke (identical winding excitations)
    // rather than the topology tag. Keep the emitted requirements identical to pre-cutover MKF.
    MAS::DesignRequirements dr = filter_choke_requirements(
        "commonModeNoiseFiltering", std::nullopt,
        d.numberOfWindings, lmSpec, d.impedancePoints);

    // The excitation frequency: the shared cmc_excitation_frequency rule (Cmc.hpp) — the ngspice
    // ideal sim uses the SAME rule, so simulated and analytical operating points always agree.
    const double excitationFrequency = cmc_excitation_frequency(d);

    MAS::OperatingPoint op = analytical::analytical_common_mode_choke(
        lm, d.operatingCurrent, d.operatingVoltage, excitationFrequency, d.numberOfWindings,
        d.parasiticCapPf, d.dvdtVPerNs, d.ambientTemperature);

    return make_inputs(std::move(dr), std::move(op));
}

} // namespace Kirchhoff
