// Advanced-path verification — KH honors PEAS pins already present in the spec/TAS, reproducing MKF's
// Advanced<Name> reference-design flow (AdvancedFlyback/AdvancedLlc take desiredInductance,
// desiredTurnsRatios, desiredResonant{Inductance,Capacitance} as constraints and size the rest around them).
//
// The user's question: "Advanced is supposed to be supported by current KH when we already pass a TAS with
// some PEAS already defined, right?" — this test answers it concretely: pinning magnetizingInductance /
// turnsRatios / resonant Lr,Cr in the design requirements flows through design_<topo> into the assembled
// TAS's magnetic MAS::Inputs, verbatim.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Kirchhoff.hpp"
#include "Dimension.hpp"       // PEAS::resolve_dimensional_values (typed DimensionWithTolerance overload)
#include "DimensionJson.hpp"   // PEAS::resolve_dimensional_values (nlohmann::json overload, for raw-json caps)

#include <cmath>
#include <string>

using json = nlohmann::json;

namespace {

json spec_for(double vin, double vout, double power, double fsw, double eta = 1.0) {
    json s;
    s["designRequirements"]["efficiency"] = eta;
    s["designRequirements"]["inputVoltage"] = {{"minimum", vin * 0.9}, {"nominal", vin}, {"maximum", vin * 1.1}};
    s["designRequirements"]["switchingFrequency"]["nominal"] = fsw;
    s["designRequirements"]["outputs"] = json::array({ {{"name", "out"}, {"voltage", {{"nominal", vout}}}} });
    s["operatingPoints"] = json::array({ {{"inputVoltage", vin}, {"outputs", json::array({{{"power", power}}})}} });
    return s;
}

// Resolve the magnetizing inductance the assembled TAS carries for the named magnetic.
double tas_magnetizing_inductance(const json& tas, const std::string& name) {
    for (const auto& m : Kirchhoff::topology_waveforms(tas))
        if (m.name == name)
            return PEAS::resolve_dimensional_values(m.inputs.get_design_requirements().get_magnetizing_inductance());
    throw std::runtime_error("magnetic '" + name + "' not in TAS");
}

double tas_turns_ratio(const json& tas, const std::string& name, size_t idx) {
    for (const auto& m : Kirchhoff::topology_waveforms(tas)) {
        if (m.name != name) continue;
        const auto& trs = m.inputs.get_design_requirements().get_turns_ratios();
        REQUIRE(idx < trs.size());
        return PEAS::resolve_dimensional_values(trs.at(idx));
    }
    throw std::runtime_error("magnetic '" + name + "' not in TAS");
}

}  // namespace

TEST_CASE("advanced: flyback honors pinned inductance + turns ratio", "[advanced][flyback]") {
    const double pinnedL = 220e-6, pinnedN = 3.5;
    json spec = spec_for(48, 12, 30, 100000);
    spec["designRequirements"]["magnetizingInductance"]["nominal"] = pinnedL;
    spec["designRequirements"]["turnsRatios"] = json::array({ {{"nominal", pinnedN}} });

    json tas = Kirchhoff::build_flyback_tas(Kirchhoff::design_flyback(spec));
    CHECK(tas_magnetizing_inductance(tas, "T1") == Catch::Approx(pinnedL).epsilon(1e-6));
    CHECK(tas_turns_ratio(tas, "T1", 0) == Catch::Approx(pinnedN).epsilon(1e-6));
}

TEST_CASE("advanced: LLC honors pinned magnetizing inductance", "[advanced][llc]") {
    const double pinnedLm = 600e-6;
    json spec = spec_for(400, 12, 120, 100000);
    spec["designRequirements"]["magnetizingInductance"]["nominal"] = pinnedLm;

    json tas = Kirchhoff::build_llc_tas(Kirchhoff::design_llc(spec));
    CHECK(tas_magnetizing_inductance(tas, "T1") == Catch::Approx(pinnedLm).epsilon(1e-6));
}

TEST_CASE("advanced: LLC honors pinned resonant Lr (and Cr)", "[advanced][llc]") {
    const double pinnedLr = 33e-6, pinnedCr = 22e-9;
    json spec = spec_for(400, 12, 120, 100000);
    spec["designRequirements"]["desiredResonantInductance"]["nominal"] = pinnedLr;
    spec["designRequirements"]["desiredResonantCapacitance"]["nominal"] = pinnedCr;

    json tas = Kirchhoff::build_llc_tas(Kirchhoff::design_llc(spec));
    // Lr is its own single-winding magnetic named "Lr" — its magnetizingInductance is the resonant L.
    CHECK(tas_magnetizing_inductance(tas, "Lr") == Catch::Approx(pinnedLr).epsilon(1e-6));

    // Cr surfaces as the role="resonant" capacitor's designRequirements.capacitance — walk the circuit.
    double crFound = -1;
    for (const auto& st : tas.at("topology").at("stages")) {
        if (!st.contains("circuit") || !st.at("circuit").is_object()) continue;
        for (const auto& c : st.at("circuit").at("components")) {
            if (!c.contains("data") || !c.at("data").is_object()) continue;
            const auto& data = c.at("data");
            if (!data.contains("inputs") || !data.at("inputs").is_object()) continue;
            const auto& in = data.at("inputs");
            if (!in.contains("designRequirements") || !in.at("designRequirements").contains("capacitance")) continue;
            double cap = PEAS::resolve_dimensional_values(in.at("designRequirements").at("capacitance"));
            if (std::abs(cap - pinnedCr) < pinnedCr * 1e-3) crFound = cap;
        }
    }
    CHECK(crFound == Catch::Approx(pinnedCr).epsilon(1e-3));
}
