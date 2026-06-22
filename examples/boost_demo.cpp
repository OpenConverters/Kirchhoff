// Kirchhoff Boost: second topology through the SAME generic assembler — proves topology-generalization.
#include "Boost.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <regex>

using nlohmann::json;

static std::string run_ngspice(const std::string& deck) {
    { std::ofstream f("/tmp/kirchhoff_boost.cir"); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen("ngspice -b /tmp/kirchhoff_boost.cir 2>&1", "r");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}

int main() {
    json tasInputs = json::parse(R"({
        "designRequirements": { "efficiency": 0.9,
            "inputVoltage": { "nominal": 12.0 }, "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "24V", "voltage": { "nominal": 24.0 } } ] },
        "operatingPoints": [ { "inputVoltage": 12.0, "outputs": [ { "power": 24.0 } ] } ]
    })");

    Kirchhoff::BoostDesign d = Kirchhoff::design_boost(tasInputs);
    std::cout << "=== Boost design: " << d.inputVoltage << "V -> " << d.outputVoltage
              << "V / " << d.outputPower << "W ===\n"
              << "  D=" << d.dutyCycle << " L=" << d.inductance * 1e6 << "uH Rload="
              << d.loadResistance << " Cout=" << d.outputCapacitance * 1e6 << "uF\n";

    json tas = Kirchhoff::build_boost_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    std::cout << "\n=== boost deck ===\n" << deck << "\n";

    std::string out = run_ngspice(deck);
    std::smatch m;
    if (!std::regex_search(out, m, std::regex(R"(vout\s*=\s*([-0-9.eE+]+))"))) {
        std::cerr << "FAIL: no vout\n"; return 1;
    }
    double vout = std::stod(m[1].str());
    std::cout << "=== RESULT: Boost Vout = " << vout << " V (target 24) ===\n";
    if (vout > 20.0 && vout < 28.0) { std::cout << "BOOST WORKS THROUGH GENERIC ASSEMBLER — PASS\n"; return 0; }
    std::cerr << "FAIL: Vout " << vout << " out of [20,28]\n";
    return 1;
}
