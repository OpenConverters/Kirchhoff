// Kirchhoff end-to-end demo: TAS inputs -> Flyback design -> CIAS atom-brick -> ngspice deck -> run.
// Verifies the whole P2+P3+P4+P5 critical path produces a flyback that regulates near Vout.
#include "Flyback.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <regex>

using nlohmann::json;

static std::string run_ngspice(const std::string& deck) {
    std::string path = "/tmp/kirchhoff_flyback.cir";
    { std::ofstream f(path); f << deck; }
    std::string cmd = "ngspice -b " + path + " 2>&1";
    std::string out;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) throw std::runtime_error("failed to launch ngspice");
    char buf[4096];
    while (fgets(buf, sizeof(buf), pipe)) out += buf;
    pclose(pipe);
    return out;
}

int main() {
    // TAS inputs: 48 V -> 12 V, 24 W flyback @ 100 kHz.
    json tasInputs = json::parse(R"({
        "designRequirements": {
            "efficiency": 0.88, "inputType": "dc",
            "inputVoltage": { "minimum": 36.0, "nominal": 48.0, "maximum": 60.0 },
            "switchingFrequency": { "nominal": 100000 }, "isolationVoltage": 1500,
            "outputs": [ { "name": "12V", "voltage": { "nominal": 12.0 }, "regulation": "voltage" } ]
        },
        "operatingPoints": [
            { "name": "full_load", "inputVoltage": 48.0, "ambientTemperature": 25.0,
              "outputs": [ { "name": "12V", "power": 24.0 } ] }
        ]
    })");

    Kirchhoff::FlybackDesign d = Kirchhoff::design_flyback(tasInputs);
    std::cout << "=== Kirchhoff Flyback design ===\n"
              << "  Vin=" << d.inputVoltage << " Vout=" << d.outputVoltage
              << " Pout=" << d.outputPower << " fsw=" << d.switchingFrequency << "\n"
              << "  turnsRatio n = " << d.turnsRatio << "\n"
              << "  dutyCycle  D = " << d.dutyCycle << "\n"
              << "  Lp           = " << d.magnetizingInductance * 1e6 << " uH\n"
              << "  Rload        = " << d.loadResistance << " ohm\n"
              << "  Cout         = " << d.outputCapacitance * 1e6 << " uF\n";

    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::emit_flyback_ngspice(d, ideal);
    std::cout << "\n=== ngspice deck ===\n" << deck << "\n";

    std::string out = run_ngspice(deck);
    std::cout << "=== ngspice output (tail) ===\n";
    { // print last ~12 lines
        size_t pos = out.size(); int n = 0;
        while (pos > 0 && n < 12) { pos = out.rfind('\n', pos ? pos - 1 : 0); ++n; if (pos==std::string::npos){pos=0;break;} }
        std::cout << out.substr(pos) << "\n";
    }

    auto meas = [&](const std::string& name, double& v) -> bool {
        std::smatch m;
        std::regex re(name + R"(\s*=\s*([-0-9.eE+]+))");
        if (!std::regex_search(out, m, re)) return false;
        v = std::stod(m[1].str());
        return true;
    };
    double vout = 0, vout_pp = 0, iin = 0;
    if (!meas("vout", vout)) { std::cerr << "FAIL: could not parse vout\n"; return 1; }
    meas("vout_pp", vout_pp);
    meas("iin", iin);

    // --- P6: results extraction into a structured outputs object ---
    const double pout = vout * vout / d.loadResistance;
    const double pin = std::abs(iin) * d.inputVoltage;
    const double eff = pin > 0 ? pout / pin : 0.0;
    json outputs;
    outputs["operatingPoint"] = "full_load";
    outputs["outputVoltage"] = vout;
    outputs["outputVoltageRipplePkPk"] = std::abs(vout_pp);
    outputs["outputPower"] = pout;
    outputs["inputCurrent"] = std::abs(iin);
    outputs["inputPower"] = pin;
    outputs["efficiency"] = eff;
    std::cout << "\n=== RESULT (extracted outputs) ===\n" << outputs.dump(2) << "\n";

    if (vout > 9.0 && vout < 15.0) { std::cout << "\nFLYBACK REGULATES NEAR TARGET — PASS\n"; return 0; }
    std::cerr << "FAIL: Vout " << vout << " out of [9,15]\n";
    return 1;
}
