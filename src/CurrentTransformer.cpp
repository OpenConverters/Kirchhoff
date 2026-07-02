#include "CurrentTransformer.hpp"
#include "ChokeDesign.hpp"            // make_inputs + round_to (shared assembly seam)
#include "ConverterAnalytical.hpp"    // analytical_current_transformer — the excitation half

#include <stdexcept>

namespace Kirchhoff {
using nlohmann::json;

// ═══ design_current_transformer — ported from MKF CurrentTransformer::process_design_requirements(:21) +
// process(:104). The excitation half is analytical_current_transformer; the requirements are a real
// 2-winding transformer (turns ratio, Lm floor, primary/secondary isolation, CT topology). ════════════

MAS::Inputs design_current_transformer(const json& spec) {
    for (const char* k : {"waveformLabel", "maximumPrimaryCurrentPeak", "frequency", "turnsRatio",
                          "burdenResistor", "ambientTemperature"})
        if (!spec.contains(k))
            throw std::invalid_argument(std::string("design_current_transformer: '") + k + "' is required");

    const auto label = spec.at("waveformLabel").get<MAS::WaveformLabel>();  // MAS from_json maps the string
    const double primaryPeak = spec.at("maximumPrimaryCurrentPeak").get<double>();
    const double frequency = spec.at("frequency").get<double>();
    const double turnsRatio = spec.at("turnsRatio").get<double>();
    const double burdenResistor = spec.at("burdenResistor").get<double>();
    const double ambientTemperature = spec.at("ambientTemperature").get<double>();
    const double secondaryDcResistance = spec.value("secondaryDcResistance", 0.0);
    const double dutyCycle = spec.value("dutyCycle", 0.5);
    const double diodeVoltageDrop = spec.value("diodeVoltageDrop", 0.0);

    if (turnsRatio <= 0)
        throw std::invalid_argument("design_current_transformer: turnsRatio must be > 0");

    // ── DesignRequirements (MKF :21-40) — a 2-winding transformer, NOT a filter choke ──
    MAS::DesignRequirements dr;
    MAS::DimensionWithTolerance ratio;
    // Trim float noise at 10 decimals (like the Lm bound). Deliberately NOT MKF's roundFloat(2):
    // step-down sensing ratios are ≤ 0.01 (1:100), so 2 decimals turned a 1:250 CT (0.004) into a
    // 0.0 turns-ratio requirement and distorted 1:150 (0.00667 → 0.01) by 50%.
    ratio.set_nominal(round_to(turnsRatio, 10));
    dr.get_mutable_turns_ratios().push_back(ratio);
    MAS::DimensionWithTolerance lmSpec;
    lmSpec.set_minimum(round_to(1e-6, 10));       // defaults.currentTransformerMinimumMagnetizingInductance
    dr.set_magnetizing_inductance(lmSpec);
    dr.set_isolation_sides(std::vector<MAS::IsolationSide>{
        MAS::IsolationSide::PRIMARY, MAS::IsolationSide::SECONDARY});
    dr.set_topology(MAS::Topology::CURRENT_TRANSFORMER);

    // ── Operating point (the analytical excitation model; throws on an unsupported waveform label) ──
    MAS::OperatingPoint op = analytical::analytical_current_transformer(
        label, primaryPeak, frequency, turnsRatio, burdenResistor,
        secondaryDcResistance, dutyCycle, diodeVoltageDrop);
    op.get_mutable_conditions().set_ambient_temperature(ambientTemperature);

    return make_inputs(std::move(dr), std::move(op));
}

} // namespace Kirchhoff
