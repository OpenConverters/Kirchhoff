// Automated magnetic BIND step (the Heaviside/MKF hand-off): read a TAS, and for every magnetic
// component that is still a seed (has inputs but no bound model), design a real magnetic via MKF from
// its own inputs and stamp the exported ngspice subcircuit into magnetic.modelOutputs.spiceSubcircuit.
// Kirchhoff then simulates the bound TAS at MKF_MODEL fidelity. Usage: bind_magnetics <in.json> <out.json>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <nlohmann/json.hpp>
#include "processors/Inputs.h"
#include "advisers/MagneticAdviser.h"
#include "processors/CircuitSimulatorInterface.h"

using json = nlohmann::json;
using namespace OpenMagnetics;

static std::string subckt_name(const std::string& text) {
    std::smatch m;
    if (std::regex_search(text, m, std::regex(R"(\.subckt\s+(\S+))"))) return m[1].str();
    return "";
}

// Bind every magnetic component (data.magnetic + data.inputs, not already bound). Returns the count.
static int bind(json& node) {
    int n = 0;
    if (node.is_object()) {
        if (node.contains("data") && node["data"].is_object()
            && node["data"].contains("magnetic") && node["data"].contains("inputs")) {
            json& comp = node["data"];
            if (comp["magnetic"].is_object() && comp["magnetic"].contains("modelOutputs"))
                return 0;   // already bound — leave it
            try {
                OpenMagnetics::Inputs inputs(comp.at("inputs"), /*processWaveform=*/true);
                MagneticAdviser adviser;
                auto results = adviser.get_advised_magnetic(inputs, 1);
                if (results.empty()) { std::cerr << "  bind: adviser returned no magnetic\n"; return 0; }
                OpenMagnetics::Magnetic mag = results[0].first.get_magnetic();
                std::string sub = CircuitSimulatorExporter(CircuitSimulatorExporterModels::NGSPICE)
                    .export_magnetic_as_subcircuit(mag, 100e3, 25.0);
                comp["magnetic"]["modelOutputs"]["spiceSubcircuit"] = {{"text", sub}, {"reference", subckt_name(sub)}};
                std::cerr << "  bound magnetic -> " << mag.get_reference() << "\n";
                return 1;   // bound; do not recurse into it
            } catch (const std::exception& e) {
                std::cerr << "  bind FAILED: " << e.what() << "\n";
                return 0;
            }
        }
        for (auto& [k, v] : node.items()) n += bind(v);
    } else if (node.is_array()) {
        for (auto& e : node) n += bind(e);
    }
    return n;
}

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "usage: bind_magnetics <in.json> <out.json>\n"; return 2; }
    std::ifstream f(argv[1]); if (!f) { std::cerr << "cannot read " << argv[1] << "\n"; return 1; }
    json tas; f >> tas;
    int n = bind(tas);
    std::ofstream(argv[2]) << tas.dump(2) << "\n";
    std::cerr << "bound " << n << " magnetic(s) -> " << argv[2] << "\n";
    return n > 0 ? 0 : 1;
}
