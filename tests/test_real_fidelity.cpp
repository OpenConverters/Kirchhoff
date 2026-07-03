// Per-component fidelity + multi-atom real device models.
//
// Demonstrates the Heaviside librarian round-trip: Kirchhoff emits a TAS of pre-sourcing seeds (ideal);
// the librarian BINDS a real part into one component's data; Kirchhoff re-simulates and that component
// becomes its real equivalent circuit (here a real electrolytic capacitor = C + series ESR), while
// every still-seed component stays ideal. Fidelity is inferred per component from the data.

#include "Flyback.hpp"
#include "Psfb.hpp"
#include "Dab.hpp"
#include "Llc.hpp"
#include "Src.hpp"
#include "TasAssembler.hpp"
#include "KirchhoffConfig.hpp"   // cfg::is_numerical_aid / mark_numerical_aid (ABT #96)
#include "Fidelity.hpp"

#include <nlohmann/json.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>
#include <regex>
#include <string>

using nlohmann::json;

namespace {
std::string run_ngspice(const std::string& deck) {
    { std::ofstream f("/tmp/kf_real_fidelity.cir"); f << deck; }
    std::string out; char buf[4096];
    FILE* p = popen("ngspice -b /tmp/kf_real_fidelity.cir 2>&1", "r");
    if (!p) throw std::runtime_error("failed to launch ngspice");
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}

json flyback_inputs() {
    return json::parse(R"({
        "designRequirements": { "efficiency": 0.88, "inputType": "dc",
            "inputVoltage": { "minimum": 36, "nominal": 48, "maximum": 60 },
            "switchingFrequency": { "nominal": 100000 }, "isolationVoltage": 1500,
            "outputs": [ { "name": "12V", "voltage": { "nominal": 12 }, "regulation": "voltage" } ] },
        "operatingPoints": [ { "name": "full", "inputVoltage": 48, "ambientTemperature": 25,
            "outputs": [ { "name": "12V", "power": 24 } ] } ]
    })");
}

// Bind a schema-valid real MOSFET (Coss + body diode) into every switch and a real diode (Cj) into every
// diode of a TAS. Returns {nfet, ndio}. Form per deps/SAS/tests/test_sas.cpp.
std::pair<int,int> bind_real_semiconductors(json& tas, double coss = 1e-9) {
    json realMosfet = json::parse(R"({
        "semiconductor": { "mosfet": { "manufacturerInfo": { "name": "T", "datasheetInfo": {
            "part": { "partNumber": "TESTFET", "technology": "Si" },
            "electrical": { "drainSourceVoltage": 650, "onResistance": 0.02,
                "continuousDrainCurrent": 30, "gateThresholdVoltage": { "nominal": 3.0 },
                "totalGateCharge": 1.5e-7, "outputCapacitance": 1e-9,
                "bodyDiodeForwardVoltage": 0.9 } } } } } })");
    realMosfet["semiconductor"]["mosfet"]["manufacturerInfo"]["datasheetInfo"]["electrical"]["outputCapacitance"] = coss;
    json realDiode = json::parse(R"({
        "semiconductor": { "diode": { "manufacturerInfo": { "name": "T", "datasheetInfo": {
            "part": { "partNumber": "TESTDIODE", "technology": "SiC" },
            "electrical": { "reverseVoltage": 650, "forwardVoltage": 0.8, "forwardCurrent": 10,
                "junctionCapacitance": 1e-9 } } } } } })");
    int nfet = 0, ndio = 0;
    for (auto& st : tas["topology"]["stages"])
        if (st.contains("circuit") && st["circuit"].is_object())
            for (auto& c : st["circuit"]["components"])
                if (c["data"].is_object() && c["data"].contains("semiconductor")) {
                    if (c["data"]["semiconductor"].contains("mosfet")) { c["data"] = realMosfet; ++nfet; }
                    else if (c["data"]["semiconductor"].contains("diode")) { c["data"] = realDiode; ++ndio; }
                }
    return {nfet, ndio};
}
}  // namespace

TEST_CASE("A bound real capacitor becomes a C+ESR model; seeds stay ideal", "[real][fidelity]") {
    Kirchhoff::FlybackDesign d = Kirchhoff::design_flyback(flyback_inputs());
    json tas = Kirchhoff::build_flyback_tas(d);

    // The librarian binds a real electrolytic (with ESR) for Cout, retaining the requirements it met.
    json realCap = json::parse(R"({
        "capacitor": { "manufacturerInfo": { "name": "Nichicon", "datasheetInfo": {
            "part": { "partNumber": "UPW1V680MDD", "technology": "Alum. Electrolytic" },
            "electrical": { "capacitance": { "nominal": 6.8e-5 }, "ratedVoltage": 35, "esr": 0.12 },
            "mechanical": { "shape": { "assembly": "THT", "shapeType": "radial" } } } } },
        "inputs": { "designRequirements": { "capacitance": { "nominal": 6.8e-5 }, "ratedVoltage": 24 } }
    })");
    bool bound = false;
    for (auto& st : tas["topology"]["stages"])
        if (st.contains("circuit") && st["circuit"].is_object())
            for (auto& c : st["circuit"]["components"])
                if (c["name"] == "Cout") { c["data"] = realCap; bound = true; }
    REQUIRE(bound);

    PEAS::Fidelity base(PEAS::Fidelity::Origin::REQUIREMENTS);
    std::string deck = Kirchhoff::tas_to_ngspice(tas, base);
    INFO(deck);

    // Cout expanded to a multi-atom equivalent circuit: C atom + series ESR resistor atom (0.12 Ohm).
    CHECK(deck.find("CCout_C") != std::string::npos);
    CHECK(std::regex_search(deck, std::regex(R"(RCout_Resr\s+\S+\s+\S+\s+0\.12\b)")));
    // Everything else stayed a pre-sourcing seed -> ideal: the switch is still the ideal SW model,
    // and no spurious ESR appeared on the input cap.
    CHECK(deck.find(".model SW_") != std::string::npos);
    CHECK(deck.find("RCin_Resr") == std::string::npos);

    // And the mixed-fidelity deck still simulates to a sane output.
    std::string out = run_ngspice(deck);
    std::smatch m;
    REQUIRE(std::regex_search(out, m, std::regex(R"(vout\s*=\s*([-0-9.eE+]+))")));
    const double vout = std::stod(m[1].str());
    CHECK(vout > 9.0);
    CHECK(vout < 15.0);
}

// Real-fidelity deck: bind SCHEMA-VALID real MOSFETs (-> SAS multi-atom leaf: switch + Coss + body diode)
// and real diodes (-> D model + Cj) into every semiconductor of a bridge, then SIMULATE. The point is the
// thing that failed before: a real-device deck must actually converge and produce a sane output. (The
// numerical snubbers are still present — kept for convergence; this proves real parasitics coexist and the
// deck runs. Stripping them in favour of Coss-only is the separate accuracy refinement.)
TEST_CASE("A real-MOSFET bridge deck (Coss + body diode + real rectifier) converges and is sane",
          "[real][fidelity][realdeck]") {
    json psfb = json::parse(R"({
        "designRequirements": { "efficiency": 1.0,
            "inputVoltage": { "minimum": 380, "nominal": 400, "maximum": 420 },
            "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "out", "voltage": { "nominal": 12 } } ] },
        "operatingPoints": [ { "inputVoltage": 400, "outputs": [ { "power": 600 } ] } ]
    })");
    json tas = Kirchhoff::build_psfb_tas(Kirchhoff::design_psfb(psfb));

    // Schema-valid real parts (form per deps/SAS/tests/test_sas.cpp) + the parasitics: Coss + Cj.
    json realMosfet = json::parse(R"({
        "semiconductor": { "mosfet": { "manufacturerInfo": { "name": "Infineon", "datasheetInfo": {
            "part": { "partNumber": "IPW60R070", "technology": "Si" },
            "electrical": { "drainSourceVoltage": 650, "onResistance": 0.02,
                "continuousDrainCurrent": 30, "gateThresholdVoltage": { "nominal": 3.0 },
                "totalGateCharge": 1.5e-7, "outputCapacitance": 1e-9,
                "bodyDiodeForwardVoltage": 0.9 } } } } } })");
    json realDiode = json::parse(R"({
        "semiconductor": { "diode": { "manufacturerInfo": { "name": "Wolfspeed", "datasheetInfo": {
            "part": { "partNumber": "TEST", "technology": "SiC" },
            "electrical": { "reverseVoltage": 650, "forwardVoltage": 0.8, "forwardCurrent": 10,
                "junctionCapacitance": 1e-9 } } } } } })");
    int nfet = 0, ndio = 0;
    for (auto& st : tas["topology"]["stages"])
        if (st.contains("circuit") && st["circuit"].is_object())
            for (auto& c : st["circuit"]["components"])
                if (c["data"].is_object() && c["data"].contains("semiconductor")) {
                    if (c["data"]["semiconductor"].contains("mosfet")) { c["data"] = realMosfet; ++nfet; }
                    else if (c["data"]["semiconductor"].contains("diode")) { c["data"] = realDiode; ++ndio; }
                }
    REQUIRE(nfet > 0);
    REQUIRE(ndio > 0);

    const std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    // PSFB's node snubber is 2.2 nF; this 400 V part's Coss (1 nF) is SMALLER, so the MAGNITUDE GATE keeps
    // the snubber — stripping it and relying on 1 nF wouldn't converge at the coarse period/200 timestep.
    // The real device Coss is still emitted alongside it. (DAB + LLC below exercise the STRIP path, where the
    // snubber is energy-/rectifier-sized below the device parasitic.)
    CHECK(deck.find("Csn") != std::string::npos);  // node snubber KEPT (1 nF Coss < 2.2 nF snubber)
    const bool hasCoss = deck.find("CQ") != std::string::npos || deck.find("Coss") != std::string::npos;
    CHECK(hasCoss);                                // real device Coss emitted alongside
    std::string out = run_ngspice(deck);
    std::smatch m;
    const bool got = std::regex_search(out, m, std::regex(R"(vout\s*=\s*([-0-9.eE+]+))"));
    INFO("real-deck ngspice tail:\n" << out.substr(out.size() > 800 ? out.size() - 800 : 0));
    REQUIRE(got);                                  // converges (snubber kept + Coss)
    const double v = std::stod(m[1].str());
    INFO("real-deck vout = " << v);
    CHECK(v > 8.0);                                // delivers spec (real Ron/Vf cause some droop; not 0/blown up)
    CHECK(v < 14.0);
}

// DAB real deck: the strip MUST keep the functional Rbias* (DC midpoint bias — Coss can't replace it) while
// stripping the Csn* dV/dt caps. Proves the fix for the bug where Rsn-named bias resistors were wrongly
// stripped. All 8 switches real -> Csn stripped, Rbias kept, Coss present, and it still converges + delivers.
TEST_CASE("Real-MOSFET DAB keeps its bias resistor, strips the dV/dt cap, converges + delivers",
          "[real][fidelity][realdeck]") {
    // 48->12 V @ 24 W (the DAB reference fixture point). At this point the energy-sized snubber (~0.5 nF) is
    // below the 1 nF Coss so the snubber strips, the body diodes strip, and the deck converges. (A 400 V
    // high-power DAB point is operating-point-fragile under the strip — recorded by the non-gating sweep.)
    json dab = json::parse(R"({
        "designRequirements": { "efficiency": 1.0,
            "inputVoltage": { "minimum": 45, "nominal": 48, "maximum": 51 },
            "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "out", "voltage": { "nominal": 12 } } ] },
        "operatingPoints": [ { "inputVoltage": 48, "outputs": [ { "power": 24 } ] } ]
    })");
    json tas = Kirchhoff::build_dab_tas(Kirchhoff::design_dab(dab));
    auto [nfet, ndio] = bind_real_semiconductors(tas);
    REQUIRE(nfet == 8);                            // DAB = two active full bridges, no rectifier diodes

    const std::string deck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    CHECK(deck.find("Csn") == std::string::npos);  // dV/dt snubber caps stripped (Coss replaces them)
    CHECK(deck.find("Rbias") != std::string::npos);// FUNCTIONAL midpoint bias resistor KEPT (the bug fix)
    const bool hasCoss = deck.find("CQ") != std::string::npos || deck.find("Coss") != std::string::npos;
    CHECK(hasCoss);
    std::string out = run_ngspice(deck);
    std::smatch m;
    const bool got = std::regex_search(out, m, std::regex(R"(vout\s*=\s*([-0-9.eE+]+))"));
    INFO("DAB real-deck tail:\n" << out.substr(out.size() > 800 ? out.size() - 800 : 0));
    REQUIRE(got);
    const double v = std::stod(m[1].str());
    INFO("DAB real-deck vout = " << v << " (spec 12)");
    CHECK(v > 9.0);
    CHECK(v < 15.0);
}

// ABT #96: the real-fidelity snubber strip is driven by an EXPLICIT marker
// (cfg::mark_numerical_aid -> inputs.designRequirements.name == cfg::kNumericalAidName), NOT the refdes
// prefix. The old test stripped ANY component whose refdes started with Csn/Rsn/Csw, silently deleting real,
// sourceable parts that happened to match. These two cases pin the new contract:
//   (1) the predicate keys off the marker, so a REAL part with a Csn*-looking refdes but no marker is kept;
//   (2) every design_*_tas marks EXACTLY the Csn*/Rsn*/Csw* numerical aids and nothing else (no real part,
//       no functional Rbias*/Rdcr*/Rvd*/balancing resistor carries the marker).
TEST_CASE("Numerical-aid predicate keys off the explicit marker, not the refdes (ABT #96)",
          "[real][fidelity][abt96]") {
    using Kirchhoff::cfg::is_numerical_aid;
    using Kirchhoff::cfg::mark_numerical_aid;

    // A real, sourced X7R cap that a librarian might name "Csnap" (starts with "Csn" — the old trap). It has
    // NO marker -> not a numerical aid -> must be KEPT.
    json realCap = json::parse(R"({
        "capacitor": { "manufacturerInfo": { "name": "TDK", "datasheetInfo": {
            "part": { "partNumber": "C3216X7R", "technology": "MLCC Class II" },
            "electrical": { "capacitance": { "nominal": 1e-7 }, "ratedVoltage": 100 } } } },
        "inputs": { "designRequirements": { "capacitance": { "nominal": 1e-7 }, "ratedVoltage": 100 } }
    })");
    CHECK_FALSE(is_numerical_aid(realCap));
    json aid; aid["capacitor"] = json::object();
    aid["inputs"]["designRequirements"]["capacitance"]["nominal"] = 1e-9;
    CHECK_FALSE(is_numerical_aid(aid));       // before marking: not an aid
    mark_numerical_aid(aid);
    CHECK(is_numerical_aid(aid));             // after marking: an aid

    // Every builder that emits a Csn*/Rsn*/Csw* numerical cap/resistor tags it, and ONLY those. Sweep the
    // topologies that carry numerical aids and assert marker <=> refdes-prefix, per stage brick.
    auto llc  = [](const json& s){ return Kirchhoff::build_llc_tas(Kirchhoff::design_llc(s)); };
    auto src  = [](const json& s){ return Kirchhoff::build_src_tas(Kirchhoff::design_src(s)); };
    auto dab  = [](const json& s){ return Kirchhoff::build_dab_tas(Kirchhoff::design_dab(s)); };
    auto psfb = [](const json& s){ return Kirchhoff::build_psfb_tas(Kirchhoff::design_psfb(s)); };

    json isoSpec = json::parse(R"({ "designRequirements": { "efficiency": 1.0,
        "inputVoltage": { "minimum": 360, "nominal": 400, "maximum": 420 },
        "switchingFrequency": { "nominal": 100000 },
        "outputs": [ { "name": "out", "voltage": { "nominal": 48 } } ] },
        "operatingPoints": [ { "inputVoltage": 400, "outputs": [ { "power": 480 } ] } ] })");

    for (auto build : {+llc, +src, +dab, +psfb}) {
        json tas = build(isoSpec);
        int marked = 0, prefixed = 0;
        for (const auto& st : tas["topology"]["stages"]) {
            if (!st.contains("circuit") || !st["circuit"].is_object()) continue;
            for (const auto& c : st["circuit"]["components"]) {
                const std::string n = c["name"].get<std::string>();
                const bool isPrefix = n.rfind("Csn", 0) == 0 || n.rfind("Rsn", 0) == 0 || n.rfind("Csw", 0) == 0;
                const bool isMarked = is_numerical_aid(c["data"]);
                if (isMarked) ++marked;
                if (isPrefix) ++prefixed;
                INFO("component " << n << " marked=" << isMarked << " prefix=" << isPrefix);
                CHECK(isMarked == isPrefix);   // the marker set == the old prefix set, exactly
            }
        }
        INFO("topology marked=" << marked << " prefixed=" << prefixed);
        CHECK(marked > 0);                     // each of these topologies has ≥1 numerical aid
    }
}

// NB: a resonant LLC/SRC real deck is NOT gated here. Once the body-diode strip (which the full-bridge
// floating midpoints REQUIRE to avoid a duplicate-diode singular mesh) removes the half-bridge switch body
// diode, the resonant tank + real rectifier diodes go singular ("timestep too small" at the tank node).
// That conflict is provably not statically resolvable (psfb needs the strip, llc needs it kept), so real-deck
// convergence for llc/src is recorded as known-hard by the NON-GATING sweep in test_requirements.cpp
// (check_real_deck), not asserted here. The DAB case above gates the snubber-strip + Rbias-kept mechanism on
// a bridge; the bridge case gates the body-diode strip + magnitude-gated snubber keep.
