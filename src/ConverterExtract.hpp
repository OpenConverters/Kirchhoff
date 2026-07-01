#pragma once

// Kirchhoff::extract — the converter-model INTEGRATION surface that replaces MKF's per-topology
// "simulate_and_extract" trio (generate_ngspice_circuit + simulate_and_extract_operating_points +
// simulate_and_extract_topology_waveforms + get_extra_components_inputs). All of it is topology-AGNOSTIC:
// it operates on the assembled TAS document (produced by build_<topology>_tas), which already carries every
// magnetic's MAS::Inputs (design requirements + per-winding excitations) and the wired circuit for ngspice.
//
//   * extract_operating_point(tas, engine, magneticName) -> MAS::OperatingPoint
//         ANALYTICAL: the per-winding excitations the analytical engine already assembled into the TAS.
//         NGSPICE:    run the deck and extract each winding's current/voltage from the sim.
//     Replaces simulate_and_extract_operating_points (the `engine` parameter chooses which run engine).
//   * topology_waveforms(tas) -> vector<MagneticExtract>
//         Every magnetic in the topology (transformer(s), output inductor, resonant Lr, ...) with its
//         MAS::Inputs. These ARE "the inputs and outputs of the TAS already" — replaces
//         simulate_and_extract_topology_waveforms AND is the source for the legacy get_extra_components_inputs.
//
// The legacy MKF entry points (get_extra_components_inputs, MAS DesignRequirements) become thin MKF-side
// shims that consume this KH output — see MKF's migration shim. No topology-specific code lives here.

#include "MAS.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace Kirchhoff {

// Which run engine produces the operating point.
enum class ExtractEngine {
    ANALYTICAL,  // read the analytically-computed excitations embedded in the TAS (fast, no simulation)
    NGSPICE      // run the ngspice deck and extract the winding waveforms (slow, full circuit fidelity)
};

// One magnetic component of the assembled topology, with the MAS::Inputs the TAS carries for it.
// `isMain` marks the magnetic the converter is primarily designed around (the transformer for isolated
// topologies, the single inductor for non-isolated) — the one MKF's adviser designs; the rest are the
// "extra components" (output inductor, resonant Lr, common-mode choke, ...).
struct MagneticExtract {
    std::string name;
    MAS::Inputs inputs;
    bool isMain = false;
};

// Every magnetic in the TAS, in stage order, each with its MAS::Inputs (design requirements + per-winding
// operating-point excitations). Replaces simulate_and_extract_topology_waveforms and is the single source
// for the legacy get_extra_components_inputs (= the non-main entries).
std::vector<MagneticExtract> topology_waveforms(const nlohmann::json& tas);

// The operating point (per-winding excitations) of one magnetic in the TAS. `magneticName` selects the
// magnetic by name; empty selects the main magnetic (the one with the most windings / flagged main).
// ANALYTICAL returns the excitations already assembled into the TAS by the analytical build; NGSPICE runs
// the deck (tas_to_ngspice + run_ngspice_in_process) and extracts each winding's current & voltage.
MAS::OperatingPoint extract_operating_point(const nlohmann::json& tas, ExtractEngine engine,
                                            const std::string& magneticName = "");

// --- Legacy-compatibility shims (consumed by MKF-side wrappers) ------------------------------------
// The user's migration plan: "two methods in MKF that get this TAS and return the design requirements for
// a MAS and the get_extra_components_inputs, to maintain the legacy." Because KH owns the TAS format and
// the KH<->MKF boundary is JSON (KH's CAS submodule is a converter, not a generated-types lib), the TAS
// walk lives HERE (single authority, no duplicate walk in MKF); the two MKF methods are thin wrappers that
// call these and deserialize into MKF's own types.

// The MAS::Inputs the converter is designed around — the main magnetic (transformer / single inductor).
// This IS "the design requirements for a MAS": a full MAS::Inputs (designRequirements + operatingPoints)
// ready to hand to MKF's MagneticAdviser. Equivalent to the isMain entry of topology_waveforms(tas).
MAS::Inputs main_magnetic_inputs(const nlohmann::json& tas);

// The legacy get_extra_components_inputs content: every NON-main component the converter needs designed
// besides the main magnetic — the extra magnetics (output inductor, resonant Lr, CM choke, …) and the
// capacitors (resonant Cr, output Co, …). Returned as a tagged JSON array so an MKF-side wrapper can
// deserialize each element into the legacy std::variant<Inputs, CAS::Inputs>:
//   [ { "componentType": "magnetic",   "inputs": <MAS::Inputs json> },
//     { "componentType": "capacitor",  "inputs": <CAS::Inputs-shaped designRequirements json> }, ... ]
// (The magnetic entries carry full per-winding operating points; the capacitor entries carry the TAS's
// designRequirements — capacitance/ratedVoltage/role — but not per-OP cap waveforms, which the TAS does
// not currently embed. Documented gap, surfaced not hidden.)
nlohmann::json extra_components_inputs(const nlohmann::json& tas);

// Per-topology design diagnostics, derived from the assembled TAS. Replaces MKF's per-model
// "<name>Diagnostics" objects (Flyback::get_last_*, Llc::get_computed_*, …) with the topology-AGNOSTIC
// subset that is recoverable from the TAS: the computed component values (magnetizing/resonant inductance,
// turns ratios, resonant/output capacitances) and the per-operating-point per-winding stresses (peak/rms
// current & voltage, duty cycle) plus an inferred CCM/DCM flag. Structure mirrors MKF's WebLibMKF
// serialization — a flat block (first operating point) + an `operatingPoints[]` array (one row per OP).
//
// NOTE (documented gap, not a silent omission): the solver-INTERNAL resonant diagnostics MKF exposed
// (Nielsen `lastMode`/`lastSubStateSequence`/`lastSteadyStateResidual`) are intentionally ABSENT — KH's
// resonant solvers are FHA, not the Nielsen/4-state TDA, so those fields would be meaningless here. ZVS
// margins and flux-excursion feasibility need per-topology physics + device Coss that the TAS does not
// carry; they are out of scope for this TAS-derived view (see the migration notes).
nlohmann::json diagnostics(const nlohmann::json& tas);

}  // namespace Kirchhoff
