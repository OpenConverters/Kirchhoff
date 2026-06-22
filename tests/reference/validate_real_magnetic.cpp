// Validate the MKF_MODEL magnetic integration on CURRENT topologies (boost 1-winding, flyback
// 2-winding, forward 3-winding): build the TAS, inject MKF's exported real-magnetic subcircuit into
// the (first) magnetic, emit the deck (CIAS inlines X; assembler hoists the .subckt), simulate, and
// compare the real magnetic to the ideal one. Reads /tmp/<topo>_mag.subckt from gen_real_magnetic.
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

static bool inject_subckt(json& node, const std::string& text, const std::string& ref) {
    if (node.is_object()) {
        if (node.contains("data") && node["data"].is_object() && node["data"].contains("magnetic")) {
            node["data"]["magnetic"]["modelOutputs"]["spiceSubcircuit"] = {{"text", text}, {"reference", ref}};
            return true;
        }
        for (auto& [k, v] : node.items()) if (inject_subckt(v, text, ref)) return true;
    } else if (node.is_array()) {
        for (auto& e : node) if (inject_subckt(e, text, ref)) return true;
    }
    return false;
}

static std::string fmt(double v) { std::ostringstream o; o.precision(12); o << v; return o.str(); }

static double meas(const std::string& deck, const std::string& tag, const std::string& name) {
    std::string safe; for (char c : tag) safe += (std::isalnum((unsigned char)c) ? c : '_');
    const std::string path = "/tmp/krf_realmag_" + safe + ".cir";
    { std::ofstream f(path); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen(("ngspice -b " + path + " 2>&1").c_str(), "r");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    std::smatch m; std::regex re(name + R"(\s*=\s*([-0-9.eE+]+))");
    if (std::regex_search(out, m, re)) return std::stod(m[1].str());
    std::cerr << "MEAS FAIL (" << name << ") [" << tag << "]:\n" << out.substr(0, 600) << "\n";
    return NAN;
}

static double run(json tas, double Rload, double Cout, double Vin, const std::string& tag) {
    PEAS::Fidelity fid(PEAS::Fidelity::Origin::MKF_MODEL);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, fid);
    const double period = 1.0 / 100000.0, settle = 2400 * period, tstep = period / 200.0;
    deck = std::regex_replace(deck, std::regex(R"(\.tran\s+\S+\s+\S+\s+\S+\s+\S+)"),
                              ".tran " + fmt(tstep) + " " + fmt(settle) + " 0 " + fmt(tstep));
    auto cpos = deck.rfind("\n.control"); if (cpos != std::string::npos) deck = deck.substr(0, cpos);
    deck += "\n.control\nrun\nmeas tran vout AVG v(Vout) from=" + fmt(settle - period) + " to=" + fmt(settle) +
            "\nmeas tran iin AVG i(VVin) from=" + fmt(settle - period) + " to=" + fmt(settle) +
            "\nprint vout iin\n.endc\n.end\n";
    double vout = meas(deck, tag, "vout"), iin = meas(deck, tag, "iin");
    double pout = vout * vout / Rload, pin = std::fabs(iin) * Vin;
    std::cout << "  [" << tag << "] Vout=" << vout << " Iout=" << vout / Rload
              << " eff=" << (pin > 1e-9 ? pout / pin : 0) << "\n";
    return vout;
}

// Read the exported subckt + parse its .subckt name (the X-instance must reference the same token).
static bool load_subckt(const std::string& path, std::string& text, std::string& ref) {
    std::ifstream f(path); if (!f) { std::cerr << "missing " << path << "\n"; return false; }
    std::stringstream ss; ss << f.rdbuf(); text = ss.str();
    std::smatch m;
    if (!std::regex_search(text, m, std::regex(R"(\.subckt\s+(\S+))"))) { std::cerr << "no .subckt in " << path << "\n"; return false; }
    ref = m[1].str(); return true;
}

template <class Design, class DesignFn, class BuildFn>
static void validate(const std::string& tag, const json& di, DesignFn design, BuildFn build,
                     const std::string& subcktPath) {
    Design d = design(di);
    std::cout << "--- " << tag << " ---\n";
    run(build(d), d.loadResistance, d.outputCapacitance, d.inputVoltage, tag + ".ideal");
    std::string text, ref;
    if (!load_subckt(subcktPath, text, ref)) return;
    json tas = build(d);
    if (!inject_subckt(tas, text, ref)) { std::cerr << tag << ": no magnetic found\n"; return; }
    std::cout << "  (real magnetic subckt: " << ref << ")\n";
    run(tas, d.loadResistance, d.outputCapacitance, d.inputVoltage, tag + ".real");
}

int main() {
    json boostDi = json::parse(R"({"designRequirements":{"efficiency":1.0,
      "inputVoltage":{"minimum":11.4,"nominal":12,"maximum":12.6},"switchingFrequency":{"nominal":100000},
      "outputs":[{"name":"out","voltage":{"nominal":24}}]},"operatingPoints":[{"inputVoltage":12,"outputs":[{"power":24}]}]})");
    json isoDi = json::parse(R"({"designRequirements":{"efficiency":1.0,
      "inputVoltage":{"minimum":45.6,"nominal":48,"maximum":50.4},"switchingFrequency":{"nominal":100000},
      "outputs":[{"name":"out","voltage":{"nominal":12}}]},"operatingPoints":[{"inputVoltage":48,"outputs":[{"power":24}]}]})");

    validate<Kirchhoff::BoostDesign>("boost(1w)", boostDi, Kirchhoff::design_boost,
                                     Kirchhoff::build_boost_tas, "/tmp/boost_mag.subckt");
    validate<Kirchhoff::FlybackDesign>("flyback(2w)", isoDi, Kirchhoff::design_flyback,
                                       Kirchhoff::build_flyback_tas, "/tmp/flyback_mag.subckt");
    validate<Kirchhoff::ForwardDesign>("forward(3w)", isoDi, Kirchhoff::design_forward,
                                       Kirchhoff::build_forward_tas, "/tmp/forward_mag.subckt");
    return 0;
}
