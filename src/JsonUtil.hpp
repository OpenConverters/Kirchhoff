#pragma once

// Small shared JSON utilities for the facade/bindings layer.

#include <nlohmann/json.hpp>

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

} // namespace Kirchhoff
