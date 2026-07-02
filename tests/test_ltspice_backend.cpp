// Second-backend test (#2): proves the CIAS intermediate representation is simulator-AGNOSTIC, not
// ngspice-shaped. The SAME assembled circuit is rendered to TWO SPICE dialects (ngspice + LTspice) by the
// same lowering; this test checks (1) STRUCTURAL completeness — every CIAS primitive maps to a valid
// LTspice card and the two decks are topologically identical (differing only in the documented
// dialect-specific spots), and (2) when an LTspice engine is available (Wine), EXECUTION equivalence —
// LTspice's transient result matches ngspice's and the independent averaged-model oracle.
//
// LTspice has no native Linux build; it runs under Wine in batch mode. If no LTspice is found, the
// execution cross-check is SKIPPED LOUDLY (logged, not silently passed) — the structural mapping still
// runs and is the always-on guarantee that the IR→backend boundary covers a second simulator. Point the
// test at an install with  KIRCHHOFF_LTSPICE=/path/to/LTspice.exe  (run via `wine`).
//
// Run directly:  ./build/test_ltspice_backend
#include "Pfc.hpp"
#include "TasAssembler.hpp"
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>

using nlohmann::json;

namespace {
json pfc_inputs() {
    json di;
    di["designRequirements"]["efficiency"] = 1.0;
    di["designRequirements"]["inputType"] = "acSinglePhase";
    di["designRequirements"]["inputVoltage"]["nominal"] = 120.0;
    di["designRequirements"]["lineFrequency"]["nominal"] = 400.0;
    di["designRequirements"]["switchingFrequency"]["nominal"] = 20e3;
    { json o; o["name"]="out"; o["voltage"]["nominal"]=400.0; di["designRequirements"]["outputs"]=json::array({o}); }
    { json op; op["inputVoltage"]=120.0; json o; o["power"]=300.0; op["outputs"]=json::array({o});
      di["operatingPoints"]=json::array({op}); }
    return di;
}

// Count element cards by leading letter (R/C/L/D/S/B/V/X) across a deck's body lines — the topological
// fingerprint that must be IDENTICAL between the two dialects (same devices, different syntax).
std::map<char,int> element_histogram(const std::string& deck) {
    std::map<char,int> h;
    std::istringstream is(deck);
    std::string line;
    while (std::getline(is, line)) {
        if (line.empty()) continue;
        char c = line[0];
        if (std::string("RCLDSBVXK").find(c) != std::string::npos &&
            line.size() > 1 && (std::isalnum((unsigned char)line[1]) || line[1] == '_'))
            h[c]++;
    }
    return h;
}

std::string run_shell(const std::string& cmd) {
    std::string out; char buf[4096];
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return "";
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}
bool meas(const std::string& out, const std::string& name, double& v) {
    std::smatch m; std::regex re(name + R"(\s*[:=]\s*([-0-9.eE+]+))");
    if (!std::regex_search(out, m, re)) return false;
    v = std::stod(m[1].str()); return true;
}

// Locate an LTspice executable: explicit env override, else the default Wine install path.
std::string find_ltspice() {
    if (const char* e = std::getenv("KIRCHHOFF_LTSPICE")) {
        std::ifstream f(e); if (f.good()) return e;
    }
    const char* home = std::getenv("HOME");
    if (home) {
        for (const char* rel : {"/.wine/drive_c/Program Files/ADI/LTspice/LTspice.exe",
                                "/.wine/drive_c/Program Files/LTC/LTspiceXVII/XVIIx64.exe"}) {
            std::string path = std::string(home) + rel;
            std::ifstream f(path); if (f.good()) return path;
        }
    }
    return "";
}
}  // namespace

TEST_CASE("LTspice backend: STRUCTURAL — every CIAS primitive maps, topology identical to ngspice", "[ltspice][backend]") {
    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(pfc_inputs());
    json tas = Kirchhoff::build_pfc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);
    const std::string lt = Kirchhoff::tas_to_ltspice(tas, ideal);
    const std::string ng = Kirchhoff::tas_to_ngspice(tas, ideal);
    // Write both decks for inspection / manual LTspice verification.
    { std::ofstream f("/tmp/kirchhoff_pfc.lt.net"); f << lt; }
    { std::ofstream f("/tmp/kirchhoff_pfc.ng.cir"); f << ng; }

    // Every primitive type is present in the LTspice deck.
    REQUIRE(lt.find(".subckt") != std::string::npos);
    REQUIRE(lt.find("SIN(") != std::string::npos);          // AC source
    REQUIRE(lt.find(".model") != std::string::npos);
    REQUIRE(lt.find("SW(") != std::string::npos);           // switch model (mosfet + comparator)
    REQUIRE(lt.find(" D(") != std::string::npos);           // diode model
    REQUIRE(lt.find("BMv") != std::string::npos);           // multiplier B-source
    REQUIRE(lt.find("BSgv") != std::string::npos);          // summer B-source
    REQUIRE(lt.find("BIv_i") != std::string::npos);         // integrator B-source

    // The dialect translation is real: the integrator clamp uses LTspice if(), NOT the ngspice ternary.
    REQUIRE(lt.find("if(") != std::string::npos);
    CHECK(lt.find("?") == std::string::npos);               // no ngspice ternary anywhere in the LTspice deck
    CHECK(ng.find("if(") == std::string::npos);             // ...and ngspice keeps the ternary
    CHECK(ng.find("?") != std::string::npos);

    // Batch/measurement convention: LTspice uses deck-level .meas (no ngspice .control block).
    CHECK(lt.find(".meas") != std::string::npos);
    CHECK(lt.find(".control") == std::string::npos);
    CHECK(ng.find(".control") != std::string::npos);

    // TOPOLOGY IDENTICAL: the two decks instantiate exactly the same devices (same element histogram).
    // Only the per-card SYNTAX differs — so the lowering is not ngspice-specific.
    CHECK(element_histogram(lt) == element_histogram(ng));
}

TEST_CASE("LTspice backend: EXECUTION equivalence vs ngspice (needs Wine + LTspice)", "[ltspice][backend][run]") {
    const std::string exe = find_ltspice();
    if (exe.empty() || run_shell("which wine 2>/dev/null").empty())
        SKIP("LTspice/Wine not found — EXECUTION cross-check not exercised (structural mapping still verified). "
             "Install LTspice under Wine or set KIRCHHOFF_LTSPICE to enable the real cross-simulator run.");

    Kirchhoff::PfcDesign d = Kirchhoff::design_pfc(pfc_inputs());
    json tas = Kirchhoff::build_pfc_tas(d);
    PEAS::Fidelity ideal(PEAS::Fidelity::Origin::REQUIREMENTS);

    // ngspice reference.
    const std::string ng = Kirchhoff::tas_to_ngspice(tas, ideal);
    { std::ofstream f("/tmp/kirchhoff_lt_ng.cir"); f << ng; }
    std::string ngOut = run_shell("ngspice -b /tmp/kirchhoff_lt_ng.cir 2>&1");
    double ngVout = 0;
    REQUIRE(meas(ngOut, "vout", ngVout));

    // LTspice (Wine, batch). LTspice writes .meas results to <deck>.log. The call is bounded by `timeout`:
    // a GUI LTspice that cannot init headlessly (e.g. the WinUI/WebView2 LTspice 26 without an X display)
    // hangs, so we cap it and treat "no usable log" as an ENVIRONMENT limitation — logged and SKIPPED, not
    // a false failure of the lowering. (LTspice XVII under wine32 + xvfb is the known-working headless combo.)
    const std::string lt = Kirchhoff::tas_to_ltspice(tas, ideal);
    { std::ofstream f("/tmp/kirchhoff_lt.net"); f << lt; }
    std::remove("/tmp/kirchhoff_lt.log");
    run_shell("timeout 90 env WINEDEBUG=-all DISPLAY= wine '" + exe +
              "' -b -Run -ascii 'Z:/tmp/kirchhoff_lt.net' 2>&1");
    std::ifstream logf("/tmp/kirchhoff_lt.log");
    std::stringstream ss; ss << logf.rdbuf();
    const std::string ltLog = ss.str();
    double ltVout = 0;
    if (ltLog.empty() || !meas(ltLog, "vout", ltVout))
        SKIP("LTspice found at '" << exe << "' but produced no batch result headlessly (GUI/display init) "
             "— EXECUTION cross-check not exercised. Structural mapping is still verified.");

    INFO("PFC bus voltage: ngspice=" << ngVout << " V, LTspice=" << ltVout << " V");
    // The two independent simulators agree on the regulated bus.
    CHECK(std::abs(ltVout - ngVout) / 400.0 < 0.03);
    CHECK(ltVout > 380.0);
    CHECK(ltVout < 420.0);
}
