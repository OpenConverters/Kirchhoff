#pragma once

// ChokeDesign — the SHARED requirement-synthesis core for Kirchhoff's magnetic-component designers
// (common-mode choke, differential-mode choke, current transformer). Header-only free functions in
// `namespace Kirchhoff`, the analog of ComponentRequirements.hpp's `req::` helpers but producing typed
// MAS objects for the string-API designers (Cmc/Dmc/CurrentTransformer) instead of the TAS json seeds.
//
// The point of this file is that CMC and DMC differ only in a handful of spec-parsing rules; everything
// downstream — the reactance math, the DesignRequirements skeleton (application tags, 1:1 turns ratios,
// all-primary isolation sides, the magnetizing-inductance bound, the impedance points) and the Inputs
// assembly — is identical and lives here ONCE. A new filter-choke variant reuses this whole core and only
// contributes its own spec→(inductance, excitation) mapping.

#include "MAS.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace Kirchhoff {

// ── Reactance primitives (generic; shared by every filter-choke designer) ───────────────────────────
// |Z_L| = 2πf·L. impedance_to_inductance THROWS on non-positive frequency (no silent 0 — a bad spec
// point must surface, per the no-fallbacks rule).
inline double impedance_to_inductance(double impedanceOhms, double frequencyHz) {
    if (frequencyHz <= 0)
        throw std::invalid_argument("impedance_to_inductance: frequency must be > 0");
    return impedanceOhms / (2.0 * M_PI * frequencyHz);
}
inline double inductance_to_impedance(double inductanceH, double frequencyHz) {
    return 2.0 * M_PI * frequencyHz * inductanceH;
}

// ── Impedance requirement point construction ────────────────────────────────────────────────────────
// Build one MAS::ImpedanceAtFrequency, validating both coordinates (a filter spec point with a
// non-positive frequency or magnitude is meaningless — throw rather than silently size to it).
inline MAS::ImpedanceAtFrequency impedance_point(double frequencyHz, double impedanceOhms) {
    if (frequencyHz <= 0 || impedanceOhms <= 0)
        throw std::invalid_argument("impedance_point: frequency and impedance must be > 0");
    MAS::ImpedanceAtFrequency p;
    p.set_frequency(frequencyHz);
    MAS::ImpedancePoint mag;
    mag.set_magnitude(impedanceOhms);
    p.set_impedance(mag);
    return p;
}

// MKF roundFloat(x, decimals): trim float noise off a derived value before it becomes a requirement.
inline double round_to(double value, int decimals) {
    const double scale = std::pow(10.0, decimals);
    return std::round(value * scale) / scale;
}

// ── The DesignRequirements skeleton shared by every EMI filter choke (CMC, DMC) ──────────────────────
// application = interferenceSuppression, the given subApplication (commonModeNoiseFiltering /
// differentialModeNoiseFiltering), (numberOfWindings-1) exactly-1:1 turns ratios, all-PRIMARY isolation
// sides (every winding on the same core, no primary/secondary split), the magnetizing-inductance spec (a
// {minimum} bound derived from impedance, or a {nominal} when the user pinned it), and the impedance
// points (emitted only when non-empty — CoreAdviser scores materials by complex permeability at each).
// Downstream advisers key off application/subApplication to restrict to toroids, favour high-µ lossy CM
// materials, and lay out paired windings.
//
// `topology` is OPTIONAL and defaults to unset, matching MKF EXACTLY: MKF's CommonModeChoke::
// process_design_requirements did NOT set the topology field, whereas DifferentialModeChoke DID (its
// MagneticFilterSaturation routes inductor- vs transformer-style B off it). Preserving that asymmetry
// keeps the emitted requirements byte-for-byte compatible with what the MagneticAdviser saw before the
// Kirchhoff cutover; a caller that wants the tag passes it explicitly.
inline MAS::DesignRequirements filter_choke_requirements(
    const std::string& subApplication,
    std::optional<MAS::Topology> topology,
    int numberOfWindings,
    const MAS::DimensionWithTolerance& magnetizingInductance,
    const std::vector<MAS::ImpedanceAtFrequency>& impedancePoints) {
    MAS::DesignRequirements dr;
    dr.set_application(std::string("interferenceSuppression"));
    dr.set_sub_application(subApplication);
    if (topology) dr.set_topology(*topology);

    for (int i = 1; i < numberOfWindings; ++i) {
        MAS::DimensionWithTolerance ratio;
        ratio.set_nominal(1.0);
        dr.get_mutable_turns_ratios().push_back(ratio);
    }
    dr.set_magnetizing_inductance(magnetizingInductance);
    if (!impedancePoints.empty())
        dr.set_minimum_impedance(impedancePoints);
    dr.set_isolation_sides(std::vector<MAS::IsolationSide>(
        static_cast<size_t>(numberOfWindings), MAS::IsolationSide::PRIMARY));
    return dr;
}

// ── Inputs assembly ──────────────────────────────────────────────────────────────────────────────────
// One designRequirements + one operating point → a MAS::Inputs, ready for MKF's MagneticAdviser. The
// single seam every component designer funnels through.
inline MAS::Inputs make_inputs(MAS::DesignRequirements designRequirements, MAS::OperatingPoint operatingPoint) {
    MAS::Inputs inputs;
    inputs.set_design_requirements(std::move(designRequirements));
    inputs.set_operating_points({std::move(operatingPoint)});
    return inputs;
}

} // namespace Kirchhoff
