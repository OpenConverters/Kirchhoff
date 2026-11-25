#pragma once
// ABT #2524 test helper: bind a real-inductor subcircuit with winding self-capacitance (the shape MKF's
// ngspice export has: winding resistance, magnetizing inductance, Cself across the terminals P1+ P1-) to
// named magnetic components of a TAS, then run the deck through Kirchhoff's in-process libngspice.
#include "NgspiceRunner.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace abt2524 {

inline std::string inductor_with_self_capacitance(const std::string& ref, double inductance, double rdc,
                                                  double cself) {
    std::ostringstream sk; sk.precision(12);
    sk << ".subckt " << ref << " P1+ P1-\n"
       << "Rdc1 P1+ nmid " << rdc << "\n"
       << "Lmag1 nmid P1- " << inductance << "\n"
       << "Cself_1 P1+ P1- " << cself << "\n"
       << ".ends " << ref << "\n";
    return sk.str();
}

// Store the subcircuit in outputs.spiceSubcircuit of every magnetic component named in `names` (the
// component's own PEAS outputs, where the assembler reads it). Throws if a name is not found.
inline void bind_subcircuit(nlohmann::json& tas, const std::vector<std::string>& names, const std::string& text,
                            const std::string& ref) {
    for (const auto& n : names) {
        int hits = 0;
        for (auto& st : tas.at("topology").at("stages")) {
            if (!st.contains("circuit") || !st.at("circuit").contains("components")) continue;
            for (auto& c : st["circuit"]["components"]) {
                if (c.value("name", "") != n) continue;
                if (!c.at("data").contains("magnetic")) throw std::runtime_error("component '" + n + "' is not a magnetic");
                c["data"]["outputs"]["spiceSubcircuit"] = {{"text", text}, {"reference", ref}};
                ++hits;
            }
        }
        if (hits != 1) throw std::runtime_error("expected one component named '" + n + "', found " + std::to_string(hits));
    }
}

inline Kirchhoff::NgspiceRunResult run_real(const nlohmann::json& tas, double timeoutSeconds) {
    // REQUIREMENTS base: the magnetics carrying a subcircuit resolve to MKF_MODEL per component.
    const std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    if (deck.find("Cself_1") == std::string::npos) throw std::runtime_error("the deck does not carry the subcircuit");
    return Kirchhoff::run_ngspice_in_process(deck, timeoutSeconds);
}

}  // namespace abt2524
