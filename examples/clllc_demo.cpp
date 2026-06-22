// CLLLC closed-loop synchronous-rectifier demo: design -> 2-stage TAS (power + SR control stage) ->
// ngspice deck. Prints the deck and (if ngspice is on PATH) the settled Vout. Demonstrates a control
// stage expressed entirely in CIAS (AAS comparators) driving the secondary SR from the tank current.
#include "Clllc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using nlohmann::json;

int main(int argc, char** argv) {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputVoltage"] = {{"nominal",400.0},{"minimum",380.0},{"maximum",420.0}};
    di["designRequirements"]["switchingFrequency"]["nominal"] = 350e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=400.0; json o; o["power"]=400.0*16.5; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }

    auto d = Kirchhoff::design_clllc(di);
    json tas = Kirchhoff::build_clllc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    if (argc > 1 && std::string(argv[1]) == "--deck") { std::cout << deck; return 0; }
    std::cout << "n=" << d.turnsRatio << " Lr1=" << d.primaryResonantInductance*1e6 << "uH Cr1="
              << d.primaryResonantCapacitance*1e9 << "nF Lm=" << d.magnetizingInductance*1e6 << "uH\n";
    std::cout << deck;
    return 0;
}
