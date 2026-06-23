#include "Pfc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using nlohmann::json;
int main(int argc, char** argv) {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=300.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }
    auto d = Kirchhoff::design_pfc(di);
    json tas = Kirchhoff::build_pfc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);
    if (argc>1 && std::string(argv[1])=="--deck") { std::cout<<deck; return 0; }
    std::cout << "L=" << d.boostInductance*1e6 << "uH D=" << d.switchDuty << " Cout=" << d.outputCapacitance*1e6 << "uF Rload=" << d.loadResistance << "\n" << deck;
    return 0;
}
