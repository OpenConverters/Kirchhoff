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

}  // namespace Kirchhoff
