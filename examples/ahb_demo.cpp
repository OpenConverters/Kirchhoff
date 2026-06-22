// ahb_demo — build the PSFB TAS, emit the ideal ngspice deck, run it, print Vout.
// Iteration harness for the PSFB port (not a ctest).
#include "Ahb.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>

using nlohmann::json;

static std::string run_ngspice(const std::string& deck, const std::string& tag) {
    const std::string path = "/tmp/ahb_demo_" + tag + ".cir";
    { std::ofstream f(path); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}
static std::string fmt(double v) { std::ostringstream os; os.precision(12); os << v; return os.str(); }
static bool parse_meas(const std::string& o, const std::string& n, double& v) {
    std::smatch m; std::regex re(n + R"(\s*=\s*([-0-9.eE+]+))");
    if (!std::regex_search(o, m, re)) return false; v = std::stod(m[1].str()); return true;
}

int main(int argc, char** argv) {
    const double Vin = argc > 1 ? std::stod(argv[1]) : 48.0;
    const double Vout = 12.0, Pout = 24.0, Fs = 100e3;
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputVoltage"] = {{"nominal", Vin}, {"minimum", Vin*0.95}, {"maximum", Vin*1.05}};
    di["designRequirements"]["switchingFrequency"]["nominal"] = Fs;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=Vout; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=Vin; json oo; oo["power"]=Pout; op["outputs"]=json::array({oo});
      di["operatingPoints"]=json::array({op}); }

    Kirchhoff::AhbDesign d = Kirchhoff::design_ahb(di);
    if (const char* pe = getenv("AHB_DUTY")) d.dutyCycle = std::stod(pe);
    std::cout << "design: n=" << d.turnsRatio << " Lm=" << d.magnetizingInductance*1e6
              << "uH Lo=" << d.outputInductance*1e6 << "uH Cb=" << d.dcBlockingCapacitance*1e6
              << "uF D=" << d.dutyCycle << " Rload=" << d.loadResistance << "\n";

    json tas = Kirchhoff::build_ahb_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, ideal);

    const double period = 1.0 / Fs;
    const double rc = d.loadResistance * d.outputCapacitance;
    const double settle = std::max(400.0*period, std::ceil(30.0*rc/period)*period);
    const double tstep = period / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(settle) + " 0 " + fmt(tstep));
    const double from = settle - period;
    auto cpos = deck.rfind("\n.control");
    if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\n"
            "meas tran vout AVG v(Vout) from=" + fmt(from) + " to=" + fmt(settle) + "\n"
            "meas tran iin AVG i(VVin) from=" + fmt(from) + " to=" + fmt(settle) + "\n"
            "print vout iin\n.endc\n.end\n";

    if (argc > 2 && std::string(argv[2]) == "deck") std::cout << deck << "\n";

    std::string out = run_ngspice(deck, "main");
    double vout = 0, iin = 0;
    if (!parse_meas(out, "vout", vout)) { std::cerr << out << "\n"; return 1; }
    parse_meas(out, "iin", iin);
    const double pout = vout*vout/d.loadResistance, pin = std::fabs(iin)*d.inputVoltage;
    std::cout << "SIM: Vout=" << vout << " Iout=" << vout/d.loadResistance
              << " eff=" << (pin>1e-9?pout/pin:0) << "\n";
    return 0;
}
