#pragma once

// TasAssembler — the generic "TAS module method to generate the circuit": walk ANY TAS topology
// document (topology.stages[] each instantiating a CIAS brick of PEAS parts, wired by
// interStageConnections[]), expand every component via its per-family to_cias generator, flatten the
// two-level net namespace (stage-local brick nets + inter-stage nets), and emit a runnable ngspice
// deck (cards via the CIAS converter + a testbench synthesized from inputs + simulation.stimulus).
//
// This generalizes the flyback-specific builder. It lives in Kirchhoff for now (Kirchhoff links all family +
// CIAS libs); Phase 7 moves it to TAS/src as the canonical TAS method. Realizes decision 5 (CIAS/
// the orchestrator expands non-ideal components via the family to_cias).
//
// Net flattening:
//   * inter-stage connections group (stage,port) pairs into one node; externalPort named *GND* -> "0",
//     other externalPort/wire -> the connection name (Vin/Vout/sw_node/...).
//   * a stage's brick connection that exposes a grouped port uses that node; otherwise it is a
//     stage-internal node "<stage>__<connection>".
//   * each (stage,component,pin) maps to its node; component pins == the atom terminals from to_cias.

#include <nlohmann/json.hpp>
#include <string>
#include "Fidelity.hpp"

namespace Kirchhoff {

/**
 * @brief Turn any TAS topology document into a runnable ngspice deck.
 *
 * This is the generic simulate step of the Kirchhoff pipeline. It accepts the
 * TAS document produced by any `build_<topology>_tas(...)` (or a hand-written
 * one) and returns a complete ngspice netlist, including a `.control` block that
 * runs the transient analysis and measures the output voltage — so the returned
 * text can be simulated directly with `ngspice -b <file>`.
 *
 * @param tasDoc   A TAS document: `inputs` (designRequirements + operatingPoints),
 *                 `topology` (stages[] of CIAS bricks + interStageConnections[]),
 *                 and `simulation` (analyses + stimulus).
 * @param fidelity How each component is modelled (see PEAS::Fidelity): use
 *                 `Origin::REQUIREMENTS` for an ideal-component deck; the assembler
 *                 upgrades individual components to real models where parts are bound.
 * @return The ngspice deck as a string.
 */
std::string tas_to_ngspice(const nlohmann::json& tasDoc, const PEAS::Fidelity& fidelity);

/**
 * @brief The SAME assembly rendered in the LTspice dialect (a second SPICE backend).
 *
 * Identical circuit, different simulator target — this demonstrates that the CIAS intermediate
 * representation is simulator-AGNOSTIC and the lowering is not ngspice-specific. Almost every card is
 * byte-identical; the dialect-specific pieces are the behavioural ternary (`if()` vs `?:`) and the
 * batch/measurement convention (deck-level `.meas` vs an ngspice `.control` block). Run with
 * `wine LTspice.exe -b -Run <file>`; see tests/test_ltspice_backend.cpp for the cross-simulator check.
 */
std::string tas_to_ltspice(const nlohmann::json& tasDoc, const PEAS::Fidelity& fidelity);

} // namespace Kirchhoff
