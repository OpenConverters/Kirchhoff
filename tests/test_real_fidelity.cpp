// Per-component fidelity + multi-atom real device models.
//
// Demonstrates the Heaviside librarian round-trip: Kirchhoff emits a TAS of pre-sourcing seeds (ideal);
// the librarian BINDS a real part into one component's data; Kirchhoff re-simulates and that component
// becomes its real equivalent circuit (here a real electrolytic capacitor = C + series ESR), while
// every still-seed component stays ideal. Fidelity is inferred per component from the data.

#include "Flyback.hpp"
#include "Psfb.hpp"
#include "TasAssembler.hpp"
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

// #1 fidelity-aware numerical snubbers: the ideal-switch deck carries small convergence snubbers
// (Csn*/Rsn*/Csw*/Rdcr*) to tame infinite dV/dt at ideal commutation. A real-semiconductor (DATASHEET)
// deck must NOT carry them — the sourced device's Coss/Cj/Qrr provide the real damping, and an extra
// numerical cap would skew ZVS/efficiency. tas_to_ngspice strips them for a DATASHEET base fidelity.
TEST_CASE("Numerical convergence snubbers are stripped in a real-semiconductor deck", "[real][fidelity][snubber]") {
    json psfb = json::parse(R"({
        "designRequirements": { "efficiency": 1.0,
            "inputVoltage": { "minimum": 380, "nominal": 400, "maximum": 420 },
            "switchingFrequency": { "nominal": 100000 },
            "outputs": [ { "name": "out", "voltage": { "nominal": 12 } } ] },
        "operatingPoints": [ { "inputVoltage": 400, "outputs": [ { "power": 600 } ] } ]
    })");
    Kirchhoff::PsfbDesign d = Kirchhoff::design_psfb(psfb);
    json tas = Kirchhoff::build_psfb_tas(d);

    const std::string idealDeck = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::REQUIREMENTS));
    const std::string realDeck  = Kirchhoff::tas_to_ngspice(tas, PEAS::Fidelity(PEAS::Fidelity::Origin::DATASHEET));

    // Ideal deck carries the midpoint node snubbers; the real deck drops them.
    CHECK(idealDeck.find("Csn") != std::string::npos);
    CHECK(realDeck.find("Csn")  == std::string::npos);
    // ...while the real power train (output inductor + cap + the bridge switches) stays intact.
    CHECK(realDeck.find("Lout") != std::string::npos);
    CHECK(realDeck.find("Cout") != std::string::npos);
    CHECK(realDeck.find("SQ") != std::string::npos);   // bridge switches still present
}
