// Kirchhoff side of the bind pipeline test.
//   tas_io dump <boost|flyback|forward> <out.json>   -> write a seed TAS (ideal magnetics)
//   tas_io sim  <in.json>                             -> assemble at MKF_MODEL + simulate, print Vout/eff
// Pipeline: tas_io dump -> bind_magnetics (MKF) -> tas_io sim.
#include "Boost.hpp"
#include "Flyback.hpp"
#include "Forward.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
using json = nlohmann::json;

static std::string fmt(double v) { std::ostringstream o; o.precision(12); o << v; return o.str(); }

static double nominal(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.contains("nominal")) return j.at("nominal").get<double>();
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: tas_io dump <topo> <out> | sim <in>\n"; return 2; }
    std::string mode = argv[1];

    if (mode == "dump") {
        json boostDi = json::parse(R"({"designRequirements":{"efficiency":1.0,
          "inputVoltage":{"minimum":11.4,"nominal":12,"maximum":12.6},"switchingFrequency":{"nominal":100000},
          "outputs":[{"name":"out","voltage":{"nominal":24}}]},"operatingPoints":[{"inputVoltage":12,"outputs":[{"power":24}]}]})");
        json isoDi = json::parse(R"({"designRequirements":{"efficiency":1.0,
          "inputVoltage":{"minimum":45.6,"nominal":48,"maximum":50.4},"switchingFrequency":{"nominal":100000},
          "outputs":[{"name":"out","voltage":{"nominal":12}}]},"operatingPoints":[{"inputVoltage":48,"outputs":[{"power":24}]}]})");
        std::string topo = argv[2]; json tas;
        if (topo == "boost")        tas = Kirchhoff::build_boost_tas(Kirchhoff::design_boost(boostDi));
        else if (topo == "flyback") tas = Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(isoDi));
        else if (topo == "forward") tas = Kirchhoff::build_forward_tas(Kirchhoff::design_forward(isoDi));
        else { std::cerr << "unknown topo " << topo << "\n"; return 2; }
        std::ofstream(argv[3]) << tas.dump(2) << "\n";
        std::cerr << "dumped " << topo << " TAS -> " << argv[3] << "\n";
        return 0;
    }

    if (mode == "sim") {
        std::ifstream f(argv[2]); if (!f) { std::cerr << "cannot read " << argv[2] << "\n"; return 1; }
        json tas; f >> tas;
        const json& dr = tas.at("inputs").at("designRequirements");
        const double Vin = nominal(dr.at("inputVoltage"));
        const double Vout = nominal(dr.at("outputs").at(0).at("voltage"));
        const double Pout = nominal(tas.at("inputs").at("operatingPoints").at(0).at("outputs").at(0).at("power"));
        const double Rload = Vout * Vout / Pout;

        std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::MKF_MODEL));
        const double period = 1.0 / 100000.0, settle = 2400 * period, tstep = period / 200.0;
        deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                                  ".tran " + fmt(tstep) + " " + fmt(settle) + " 0 " + fmt(tstep));
        auto cpos = deck.rfind("\n.control"); if (cpos != std::string::npos) deck = deck.substr(0, cpos);
        deck += "\n.control\nrun\nmeas tran vout AVG v(Vout) from=" + fmt(settle - period) + " to=" + fmt(settle) +
                "\nmeas tran iin AVG i(VVin) from=" + fmt(settle - period) + " to=" + fmt(settle) +
                "\nprint vout iin\n.endc\n.end\n";
        { std::ofstream o("/tmp/tas_io_sim.cir"); o << deck; }
        std::string out; char buf[4096];
        FILE* p = popen("ngspice -b /tmp/tas_io_sim.cir 2>&1", "r");
        while (fgets(buf, sizeof(buf), p)) out += buf;
        pclose(p);
        auto get = [&](const char* nm) -> double { std::smatch m;
            if (std::regex_search(out, m, std::regex(std::string(nm) + R"(\s*=\s*([-0-9.eE+]+))"))) return std::stod(m[1].str());
            return NAN; };
        double vout = get("vout"), iin = get("iin");
        double pout = vout * vout / Rload, pin = std::fabs(iin) * Vin;
        const bool real = deck.find("modelOutputs") != std::string::npos || deck.find(".subckt 9") != std::string::npos;
        std::cout << "Vout=" << vout << " Iout=" << vout / Rload << " eff=" << (pin > 1e-9 ? pout / pin : 0)
                  << "  (" << (deck.find("\nRdc") != std::string::npos || deck.find(" Rdc") != std::string::npos ? "real" : "ideal/?") << ")\n";
        if (std::isnan(vout)) { std::cerr << out.substr(0, 600) << "\n"; return 1; }
        return 0;
    }
    std::cerr << "unknown mode\n"; return 2;
}
