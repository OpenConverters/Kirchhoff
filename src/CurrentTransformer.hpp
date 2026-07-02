#pragma once

// Current transformer (CT) — the requirement-synthesis half, plus the MAS::Inputs assembly over
// analytical_current_transformer (the excitation half, in ConverterAnalytical). Ported from MKF
// converter_models/CurrentTransformer.{h,cpp} (deleted in 3e0261fd).
//
// A CT is NOT a filter choke — it is a real 2-winding transformer (turns ratio ≠ 1, primary/secondary
// isolation, one magnetizing-inductance floor), so it does not use the ChokeDesign filter skeleton; it
// only shares make_inputs. The excitation is the burden-resistor sensing model:
// secondary current = primary/n, secondary voltage = I_sec·(R_burden + R_dc) + V_diode, reflected to
// the primary. The whole DSP already lives in analytical_current_transformer.

#include "MAS.hpp"

#include <nlohmann/json.hpp>

namespace Kirchhoff {

// spec (flat process_current_transformer payload): waveformLabel (schema string — sinusoidal /
// unipolarRectangular / unipolarTriangular), maximumPrimaryCurrentPeak, frequency, turnsRatio,
// burdenResistor, ambientTemperature, and optional secondaryDcResistance (default 0), dutyCycle
// (default 0.5), diodeVoltageDrop (default 0). Returns the MAS::Inputs (designRequirements with the
// turns ratio + Lm floor + primary/secondary isolation + CURRENT_TRANSFORMER topology, and the
// primary/secondary operating point). Throws on missing required fields, non-positive peak / frequency /
// turnsRatio, and (via the analytical model) on an unsupported waveform label.
MAS::Inputs design_current_transformer(const nlohmann::json& spec);

} // namespace Kirchhoff
