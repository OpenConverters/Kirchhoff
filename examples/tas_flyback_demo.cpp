// Kirchhoff generic path: design -> TAS-of-CIAS document -> generic tas_to_ngspice assembler -> run.
// Proves the topology-agnostic assembler (walks any TAS doc), not the flyback-specific builder.
#include "Flyback.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <regex>

using nlohmann::json;

static std::string run_ngspice(const std::string& deck) {
    std::string path = "/tmp/kirchhoff_tas_flyback.cir";
    { std::ofstream f(path); f << deck; }
    std::string out; char buf[4096];
    FILE* pipe = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    if (!pipe) throw std::runtime_error("ngspice launch failed");
    while (fgets(buf, sizeof(buf), pipe)) out += buf;
    pclose(pipe);
    return out;
}

int main() {
    json tasInputs = json::parse(R"({
        "designRequirements": { "efficiency": 0.88,
            "inputVoltage": { "nominal": 48.0 }, "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "12V", "voltage": { "nominal": 12.0 } } ] },
        "operatingPoints": [ { "inputVoltage": 48.0, "outputs": [ { "power": 24.0 } ] } ]
    })");

    Kirchhoff::FlybackDesign d = Kirchhoff::design_flyback(tasInputs);
    json tas = Kirchhoff::build_flyback_tas(d);
    std::cout << "=== TAS topology document (stages) ===\n";
    for (auto& s : tas["topology"]["stages"])
        std::cout << "  stage '" << s["name"].get<std::string>() << "' role="
                  << s["role"].get<std::string>() << " brick='"
                  << s["circuit"]["name"].get<std::string>() << "' ("
                  << s["circuit"]["components"].size() << " components)\n";

    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    std::cout << "\n=== generic-assembler ngspice deck ===\n" << deck << "\n";

    std::string out = run_ngspice(deck);
    std::smatch m;
    if (!std::regex_search(out, m, std::regex(R"(vout\s*=\s*([-0-9.eE+]+))"))) {
        std::cerr << "FAIL: no vout in output\n" << out.substr(out.size() > 800 ? out.size() - 800 : 0);
        return 1;
    }
    double vout = std::stod(m[1].str());
    std::cout << "=== RESULT (generic TAS assembler): Vout = " << vout << " V (target 12) ===\n";
    if (vout > 9.0 && vout < 15.0) { std::cout << "GENERIC TAS->NGSPICE PATH WORKS — PASS\n"; return 0; }
    std::cerr << "FAIL: Vout " << vout << " out of [9,15]\n";
    return 1;
}
