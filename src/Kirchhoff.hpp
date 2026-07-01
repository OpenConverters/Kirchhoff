#pragma once
//
// Kirchhoff — public umbrella header.
//
// Include this single header to get the whole Kirchhoff API (every converter
// designer + the generic TAS-to-ngspice assembler + the fidelity selector):
//
//     #include "Kirchhoff.hpp"
//
// The documentation comment below is the user guide. It is written for someone
// who wants to USE Kirchhoff to design and simulate a converter — not to work
// on Kirchhoff's internals.
//
// =============================================================================
/**
 * @mainpage Kirchhoff — design & simulate any power converter from a spec
 *
 * @section what What Kirchhoff does
 *
 * Kirchhoff turns a high-level converter specification (input voltage, output
 * voltage, output power, switching frequency) into:
 *
 *   1. a **design** — the component values for the chosen topology
 *      (turns ratios, inductances, capacitances, duty cycle, …), and
 *   2. a **circuit** — a vendor-neutral TAS topology document whose stages are
 *      CIAS bricks of ideal/real PEAS parts, and
 *   3. a **runnable SPICE deck** — an ngspice netlist you can simulate to read
 *      back the actual output voltage, current, efficiency and ripple.
 *
 * It is built entirely on the PSMA-org Agnostic-Structure data models
 * (PEAS / RAS / CAS / SAS / MAS / CIAS / TAS); it does not invent its own
 * schema. Every topology Kirchhoff produces is verified against OpenMagnetics
 * MKF's own design + simulation to within a few percent.
 *
 * @section pipeline The three-step pipeline
 *
 * For every topology there are two functions plus one shared assembler:
 *
 *     Kirchhoff::design_<topology>(spec)        -> <Topology>Design   // 1. design
 *     Kirchhoff::build_<topology>_tas(design)   -> TAS document (JSON)// 2. circuit
 *     Kirchhoff::tas_to_ngspice(tas, fidelity)  -> ngspice deck (text)// 3. simulate
 *
 * `tas_to_ngspice` is generic: it walks ANY TAS topology document and emits the
 * netlist, so the same step works for every topology (and for hand-written TAS
 * documents, not just the ones these builders produce).
 *
 * @section spec The input specification
 *
 * `design_<topology>` takes one JSON object — a TAS-style design input. The
 * fields it reads:
 *
 * @code{.json}
 * {
 *   "designRequirements": {
 *     "efficiency": 1.0,                              // 0..1 (use 1.0 for an ideal design)
 *     "inputVoltage": { "minimum": 45.6, "nominal": 48, "maximum": 50.4 },
 *     "switchingFrequency": { "nominal": 100000 },    // Hz
 *     "outputs": [ { "name": "out", "voltage": { "nominal": 12 } } ]   // Volts
 *   },
 *   "operatingPoints": [
 *     { "inputVoltage": 48, "outputs": [ { "power": 24 } ] }           // Watts
 *   ]
 * }
 * @endcode
 *
 * All quantities are SI base units: volts, amperes, watts, hertz, henries,
 * farads, seconds. Output load is given as power (W); output current is
 * power / voltage.
 *
 * @section units Sign & polarity conventions
 *
 * Most topologies produce a positive output. Two exceptions:
 *  - **Cuk** is inverting: its output voltage/current are negative.
 *  - **Isolated buck-boost** (inverting Fly-Buck-Boost) has an inverting primary
 *    rail V_pri = -Vin*D/(1-D); it is compared on magnitude (like Cuk).
 * Component-terminal naming used inside the generated circuit: resistor/capacitor
 * pins are "1"/"2"; MOSFET pins are "drain"/"gate"/"source"; diode pins are
 * "anode"/"cathode"; transformer/inductor winding pins are "primary_start",
 * "primary_end", "secondaryN_start", "secondaryN_end". Ground is SPICE node 0.
 *
 * @section cpp Complete C++ example
 *
 * @code{.cpp}
 * #include "Kirchhoff.hpp"
 * #include <nlohmann/json.hpp>
 * #include <fstream>
 * #include <cstdio>
 * using nlohmann::json;
 *
 * int main() {
 *     // 1. Specify: 48 V -> 12 V, 24 W, 100 kHz flyback.
 *     json spec;
 *     spec["designRequirements"]["efficiency"] = 1.0;
 *     spec["designRequirements"]["inputVoltage"]  = {{"minimum",45.6},{"nominal",48},{"maximum",50.4}};
 *     spec["designRequirements"]["switchingFrequency"]["nominal"] = 100000;
 *     spec["designRequirements"]["outputs"] = json::array({ {{"name","out"},{"voltage",{{"nominal",12}}}} });
 *     spec["operatingPoints"] = json::array({ {{"inputVoltage",48},{"outputs",json::array({{{"power",24}}})}} });
 *
 *     // 2. Design + assemble.
 *     Kirchhoff::FlybackDesign d = Kirchhoff::design_flyback(spec);
 *     json tas = Kirchhoff::build_flyback_tas(d);
 *
 *     // 3. Emit an ideal-component ngspice deck (see Fidelity below).
 *     PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
 *     std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
 *
 *     // The deck ends with a `.control … meas tran vout … .endc` block, so it
 *     // is directly runnable; write it out and simulate:
 *     std::ofstream("flyback.cir") << deck;
 *     std::system("ngspice -b flyback.cir");   // prints the measured Vout
 *
 *     // The design struct also carries the headline numbers:
 *     //   d.turnsRatio, d.magnetizingInductance, d.loadResistance, …
 * }
 * @endcode
 *
 * @section py Python example (PyKirchhoff)
 *
 * The pybind11 module exposes the pipeline to Python (JSON in / JSON / text out):
 *
 * @code{.py}
 * import PyKirchhoff, json, subprocess
 * spec = {
 *   "designRequirements": {
 *     "efficiency": 1.0,
 *     "inputVoltage": {"minimum": 45.6, "nominal": 48, "maximum": 50.4},
 *     "switchingFrequency": {"nominal": 100000},
 *     "outputs": [{"name": "out", "voltage": {"nominal": 12}}],
 *   },
 *   "operatingPoints": [{"inputVoltage": 48, "outputs": [{"power": 24}]}],
 * }
 * tas  = PyKirchhoff.design_flyback_tas(spec)               # design + assemble
 * deck = PyKirchhoff.tas_to_ngspice(tas, {"origin": "REQUIREMENTS"})
 * open("flyback.cir", "w").write(deck)
 * subprocess.run(["ngspice", "-b", "flyback.cir"])          # prints Vout
 * @endcode
 *
 * @section topologies Supported topologies
 *
 * | Topology                  | design / build functions                                  | notes                                  |
 * |---------------------------|-----------------------------------------------------------|----------------------------------------|
 * | Flyback                   | `design_flyback`        / `build_flyback_tas`              | isolated, CCM                          |
 * | Boost                     | `design_boost`          / `build_boost_tas`               | non-isolated step-up                   |
 * | Buck                      | `design_buck`           / `build_buck_tas`                | non-isolated step-down                 |
 * | Forward (single-switch)   | `design_forward`        / `build_forward_tas`             | isolated, 3-winding (demag reset)      |
 * | Two-switch forward        | `design_two_switch_forward` / `build_two_switch_forward_tas` | isolated, clamp-diode reset          |
 * | SEPIC                     | `design_sepic`          / `build_sepic_tas`               | non-isolated step up/down             |
 * | Cuk                       | `design_cuk`            / `build_cuk_tas`                 | non-isolated, **inverting**           |
 * | Zeta                      | `design_zeta`           / `build_zeta_tas`                | non-isolated step up/down             |
 * | Push-pull                 | `design_push_pull`      / `build_push_pull_tas`           | isolated, center-tapped, 2 switches    |
 * | Phase-shifted full bridge | `design_psfb`           / `build_psfb_tas`                | isolated, 4 switches, phase-shift control |
 * | Asymmetric half-bridge    | `design_ahb`            / `build_ahb_tas`                 | isolated, 2 switches, complementary duty |
 * | Active-clamp forward      | `design_acf`            / `build_acf_tas`                 | isolated forward, active-clamp reset   |
 * | Four-switch buck-boost    | `design_fsbb`           / `build_fsbb_tas`                | non-isolated H-bridge buck-boost       |
 * | Phase-shifted half-bridge | `design_pshb`           / `build_pshb_tas`                | isolated, 3-level NPC leg, phase-shift control |
 * | Dual active bridge        | `design_dab`            / `build_dab_tas`                 | isolated bidirectional, 8 switches, SPS phase-shift |
 * | Isolated buck (Flybuck)   | `design_isolated_buck`  / `build_isolated_buck_tas`      | coupled-inductor buck + isolated secondary rail |
 * | Isolated buck-boost       | `design_isolated_buck_boost` / `build_isolated_buck_boost_tas` | inverting Fly-Buck-Boost + isolated secondary |
 * | Weinberg                  | `design_weinberg`       / `build_weinberg_tas`           | current-fed push-pull, input coupled inductor, boost-capable |
 * | LLC resonant              | `design_llc`            / `build_llc_tas`                | half-bridge Lr-Cr-Lm resonant tank, CT rectifier, freq gain |
 * | SRC series resonant       | `design_src`            / `build_src_tas`                | half-bridge Lr-Cr series tank (no resonant Lm), step-down |
 * | CLLC resonant             | `design_cllc`           / `build_cllc_tas`               | bidirectional, active bridges both sides, dual resonant tanks |
 * | CLLLC resonant            | `design_clllc`          / `build_clllc_tas`              | bidirectional symmetric 5-element tank, current-aware SR in CIAS † |
 * | PFC (boost)               | `design_pfc`            / `build_pfc_tas`                | AC-input, 1-phase hysteretic boost PFC, closed-loop control in CIAS † |
 * | Vienna rectifier          | `design_vienna`         / `build_vienna_tas`            | AC-input, 3-phase 3-level boost PFC, per-phase current shaping in CIAS † |
 *
 * The first 21 topologies are gated by the MKF-equivalence suite (MKF is the
 * reference). The three marked **†** diverge from MKF by design — AC input
 * and/or closed-loop control expressed in CIAS — so they are validated
 * standalone (their own demos + tests), not against an MKF fixture.
 *
 * @section fidelity Component fidelity
 *
 * `tas_to_ngspice` takes a PEAS::Fidelity that selects how each component is
 * modelled in the netlist:
 *  - `PEAS::Fidelity::Origin::REQUIREMENTS` — ideal components sized from the
 *    design requirements (ideal switches, near-ideal diodes, lossless reactives).
 *    This is what you want for a first design pass and for the equivalence checks.
 *  - `PEAS::Fidelity::Origin::DATASHEET` — real device models, used once parts
 *    have been bound to the circuit (real ESR, on-resistance, diode parameters).
 *  - `PEAS::Fidelity::Origin::MKF_MODEL` — magnetics-specific: a real, MKF-designed
 *    magnetic with its measured winding resistance, AC-resistance ladder and
 *    leakage coupling (magnetics only).
 * The assembler infers the right fidelity per component from the part data, so a
 * single TAS document can mix an ideal passive with a real, bound magnetic.
 *
 * @section reading Reading results back
 *
 * The generated deck saves and measures the output node, so `ngspice -b deck.cir`
 * prints the steady-state output voltage. For a converged open-loop mean you must
 * let the simulation settle past several output-filter time constants
 * (RC = load_resistance · output_capacitance); the `design` struct exposes both
 * `loadResistance` and `outputCapacitance` so you can size the run length.
 */
// =============================================================================

#include "Fidelity.hpp"
#include "TasAssembler.hpp"
#include "Settings.hpp"     // Kirchhoff::Settings (project-wide config) + RunEngine
#include "Analytical.hpp"   // analytical_operating_point — the simulator-free run engine
#include "NgspiceRunner.hpp" // run_ngspice_in_process — the in-process libngspice runner
#include "ConverterExtract.hpp" // extract_operating_point / topology_waveforms — the MKF-extract-trio replacement

#include "Flyback.hpp"
#include "Boost.hpp"
#include "Buck.hpp"
#include "Forward.hpp"
#include "TwoSwitchForward.hpp"
#include "Sepic.hpp"
#include "Cuk.hpp"
#include "Zeta.hpp"
#include "PushPull.hpp"
#include "Psfb.hpp"
#include "Ahb.hpp"
#include "Acf.hpp"
#include "Fsbb.hpp"
#include "Pshb.hpp"
#include "Dab.hpp"
#include "IsolatedBuck.hpp"
#include "IsolatedBuckBoost.hpp"
#include "Weinberg.hpp"
#include "Llc.hpp"
#include "Src.hpp"
#include "Cllc.hpp"
#include "Clllc.hpp"
#include "Pfc.hpp"
#include "Vienna.hpp"
