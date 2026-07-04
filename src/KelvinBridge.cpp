// Kirchhoff -> Kelvin bridge: the KH::api component-sourcing verbs forward to Kelvin (the shared
// deterministic selector), so selection logic lives in exactly one place. Kept in its own TU (part
// of the `kirchhoff` core lib) so KirchhoffApi.cpp itself carries no Kelvin include — every target
// that links `kirchhoff` gets these, and the WASM/shared-API builds inherit them via the core lib.
#include "KirchhoffApi.hpp"

#include <string>

#include <nlohmann/json.hpp>

#include "KelvinApi.hpp"  // kelvin::api::Engine / select_components / bind_part

using json = nlohmann::json;

namespace Kirchhoff {
namespace api {

// Local guarded() mirror (KirchhoffApi.cpp's is TU-local); keep the boundary contract identical.
namespace {
template <class F>
std::string kelvin_guarded(F&& body) {
    try {
        return body();
    } catch (const std::exception& e) {
        return std::string("Exception: ") + e.what();
    } catch (...) {
        return std::string("Exception: unknown error");
    }
}
}  // namespace

std::string select_components(const std::string& tasJson, const std::string& dataDir,
                              const std::string& cacheDir, const std::string& optionsJson) {
    return kelvin_guarded([&] {
        kelvin::api::Engine engine(dataDir, cacheDir, /*quiet=*/true);
        json options = optionsJson.empty() ? json::object() : json::parse(optionsJson);
        return kelvin::api::select_components(engine, json::parse(tasJson), options).dump();
    });
}

std::string bind_part(const std::string& tasJson, const std::string& ref,
                      const std::string& envelopeJson) {
    return kelvin_guarded([&] {
        return kelvin::api::bind_part(json::parse(tasJson), ref, json::parse(envelopeJson)).dump();
    });
}

}  // namespace api
}  // namespace Kirchhoff
