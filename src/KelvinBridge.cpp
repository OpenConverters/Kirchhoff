// Kirchhoff -> Kelvin bridge: the KH::api component-sourcing verbs forward to Kelvin (the shared
// deterministic selector), so selection logic lives in exactly one place. Kept in its own TU (part
// of the `kirchhoff` core lib) so KirchhoffApi.cpp itself carries no Kelvin include — every target
// that links `kirchhoff` gets these, and the WASM/shared-API builds inherit them via the core lib.
#include "KirchhoffApi.hpp"

#include <string>

#include <nlohmann/json.hpp>

#include "KelvinApi.hpp"  // kelvin::api::Engine / select_components / bind_part
#include "Select.hpp"     // kelvin::NoCandidates

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

// --- browser sourcing: a persistent in-module engine fed prebuilt shard bytes ---------------
namespace {
kelvin::api::Engine& web_engine() {
    static kelvin::api::Engine engine("", "", /*quiet=*/true);  // no filesystem; shards loaded by bytes
    return engine;
}
}  // namespace

std::string kelvin_load_shard(const std::string& family, const std::string& shardBytes) {
    return kelvin_guarded([&] {
        kelvin::ShardMeta m = web_engine().load_shard_bytes(family, shardBytes);
        return json{{"family", kelvin::family_name(m.family)},
                    {"rowCount", m.row_count},
                    {"buildId", m.build_id}}
            .dump();
    });
}

std::string kelvin_select(const std::string& category, const std::string& reqJson,
                          const std::string& optionsJson) {
    try {
        json req = reqJson.empty() ? json::object() : json::parse(reqJson);
        json options = optionsJson.empty() ? json::object() : json::parse(optionsJson);
        return web_engine().select(category, req, options).dump();
    } catch (const kelvin::NoCandidates& e) {
        return json{{"error", "NoCandidates"},
                    {"category", e.category},
                    {"rejections", e.rejections},
                    {"totalRowsConsidered", e.total_rows_considered}}
            .dump();
    } catch (const std::exception& e) {
        return std::string("Exception: ") + e.what();
    }
}

}  // namespace api
}  // namespace Kirchhoff
