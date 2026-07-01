// Mimics MKF's bridge: includes ONLY KirchhoffApi.hpp (strings, no MAS types), links libKirchhoffApi.so.
#include "KirchhoffApi.hpp"
#include <iostream>
int main() {
    std::string spec = R"({"designRequirements":{"efficiency":1.0,"inputVoltage":{"minimum":360,"nominal":400,"maximum":440},"switchingFrequency":{"nominal":100000},"outputs":[{"name":"out","voltage":{"nominal":12}}]},"operatingPoints":[{"inputVoltage":400,"outputs":[{"power":120}]}]})";
    std::string tas = Kirchhoff::api::design_tas("llc", spec);
    std::cout << "design_tas ok: " << (tas.rfind("{",0)==0) << "\n";
    std::string mi = Kirchhoff::api::main_magnetic_inputs(tas);
    std::cout << "main_magnetic_inputs ok: " << (mi.find("designRequirements")!=std::string::npos) << "\n";
    std::string sim = Kirchhoff::api::simulate_ngspice(tas, R"({"origin":"REQUIREMENTS"})");
    std::cout << "simulate_ngspice: " << sim.substr(0, 60) << "\n";
    return 0;
}
