#include "Analytical.hpp"
#include "DimensionJson.hpp"   // PEAS::resolve_dimensional_values

#include <stdexcept>

namespace Kirchhoff {
namespace {

using nlohmann::json;

// Read excitationsPerWinding[].{current,voltage}.processed.{peak,rms,offset} from one magnetic
// component's inputs and append a WindingStress per winding.
void collect_magnetic_stresses(const std::string& compName, const json& inputs,
                               std::vector<WindingStress>& out) {
    if (!inputs.contains("operatingPoints") || !inputs.at("operatingPoints").is_array()
        || inputs.at("operatingPoints").empty())
        return;
    const json& op = inputs.at("operatingPoints").at(0);
    if (!op.contains("excitationsPerWinding") || !op.at("excitationsPerWinding").is_array()) return;
    int w = 0;
    for (const auto& exc : op.at("excitationsPerWinding")) {
        WindingStress s;
        s.component = compName;
        s.winding = w++;
        if (exc.contains("current") && exc.at("current").contains("processed")) {
            const json& c = exc.at("current").at("processed");
            s.currentPeak    = c.value("peak", 0.0);
            s.currentRms     = c.value("rms", 0.0);
            s.currentAverage = c.value("offset", 0.0);   // DC bias = average
        }
        if (exc.contains("voltage") && exc.at("voltage").contains("processed")) {
            const json& v = exc.at("voltage").at("processed");
            s.voltagePeak = v.value("peak", 0.0);
            s.voltageRms  = v.value("rms", 0.0);
        }
        out.push_back(std::move(s));
    }
}

} // namespace

AnalyticalOperatingPoint analytical_operating_point(const json& tas) {
    if (!tas.contains("inputs") || !tas.at("inputs").contains("designRequirements"))
        throw std::runtime_error("analytical_operating_point: tas.inputs.designRequirements missing");
    const json& in = tas.at("inputs");
    const json& dr = in.at("designRequirements");

    AnalyticalOperatingPoint r;

    // Output voltage — a dimensionWithTolerance, resolved with the canonical resolver (not hand-read).
    if (!dr.contains("outputs") || !dr.at("outputs").is_array() || dr.at("outputs").empty())
        throw std::runtime_error("analytical_operating_point: designRequirements.outputs missing");
    r.outputVoltage = PEAS::resolve_dimensional_values(dr.at("outputs").at(0).at("voltage"));

    // Efficiency (set by every design) and switching frequency.
    if (!dr.contains("efficiency"))
        throw std::runtime_error("analytical_operating_point: designRequirements.efficiency missing");
    r.efficiency = dr.at("efficiency").get<double>();
    if (dr.contains("switchingFrequency"))
        r.switchingFrequency = PEAS::resolve_dimensional_values(dr.at("switchingFrequency"));

    // Output power from the operating point.
    if (!in.contains("operatingPoints") || !in.at("operatingPoints").is_array()
        || in.at("operatingPoints").empty())
        throw std::runtime_error("analytical_operating_point: inputs.operatingPoints missing");
    const json& op0 = in.at("operatingPoints").at(0);
    if (!op0.contains("outputs") || !op0.at("outputs").is_array() || op0.at("outputs").empty())
        throw std::runtime_error("analytical_operating_point: operatingPoints[0].outputs missing");
    r.outputPower = op0.at("outputs").at(0).at("power").get<double>();

    if (r.outputVoltage == 0.0)
        throw std::runtime_error("analytical_operating_point: output voltage resolved to 0");
    r.outputCurrent = r.outputPower / r.outputVoltage;
    if (r.efficiency <= 0.0)
        throw std::runtime_error("analytical_operating_point: non-positive efficiency");
    r.inputPower = r.outputPower / r.efficiency;

    // Per-winding stresses: walk every stage's magnetic components.
    if (tas.contains("topology") && tas.at("topology").contains("stages")) {
        for (const auto& st : tas.at("topology").at("stages")) {
            if (!st.contains("circuit") || !st.at("circuit").is_object()
                || !st.at("circuit").contains("components")) continue;
            for (const auto& c : st.at("circuit").at("components")) {
                if (!c.contains("data") || !c.at("data").is_object()) continue;
                const json& data = c.at("data");
                if (!data.contains("magnetic")) continue;          // only magnetics carry excitations
                if (!data.contains("inputs") || !data.at("inputs").is_object()) continue;
                collect_magnetic_stresses(c.value("name", std::string{}), data.at("inputs"), r.windings);
            }
        }
    }
    return r;
}

} // namespace Kirchhoff
