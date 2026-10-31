#pragma once

// Small shared JSON utilities for the facade/bindings layer.

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace Kirchhoff {

// Recursively drop null members. The quicktype-generated MAS to_json emits EVERY optional field,
// null when unset — but the schemas admit ABSENT optional properties, not null-valued ones (a null
// "name" fails `type: string`, a null inside a oneOf branch kills the whole composite). Any typed
// MAS object serialized for an out-of-process consumer must pass through this so the emitted JSON
// stays schema-valid (the no-schema-invalid-objects rule applies to everything we materialize).
inline nlohmann::json strip_nulls(const nlohmann::json& j) {
    if (j.is_object()) {
        nlohmann::json out = nlohmann::json::object();
        for (const auto& [key, value] : j.items())
            if (!value.is_null()) out[key] = strip_nulls(value);
        return out;
    }
    if (j.is_array()) {
        nlohmann::json out = nlohmann::json::array();
        for (const auto& value : j) out.push_back(strip_nulls(value));
        return out;
    }
    return j;
}

// Refuse a spec with more outputs than a single-rail topology can build. Its design reads outputs[0] only, so
// without this a second (third, ...) rail was dropped without a word and the caller got a one-output
// converter back as the answer to a multi-output request. Counts designRequirements.outputs and every
// operating point's outputs, since the design reads both.
inline void require_one_output(const nlohmann::json& tasInputs, const std::string& topology) {
    const auto refuse = [&](size_t n) {
        throw std::invalid_argument(topology + " supports one output; the spec has " + std::to_string(n));
    };
    const nlohmann::json& outs = tasInputs.at("designRequirements").at("outputs");
    if (outs.size() > 1) refuse(outs.size());
    if (tasInputs.contains("operatingPoints"))
        for (const auto& op : tasInputs.at("operatingPoints"))
            if (op.contains("outputs") && op.at("outputs").size() > 1) refuse(op.at("outputs").size());
}

} // namespace Kirchhoff
