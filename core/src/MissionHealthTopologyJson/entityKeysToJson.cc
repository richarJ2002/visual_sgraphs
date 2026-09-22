/*!
 * @file         entityKeysToJson.cc
 *
 * @brief        Implements entityKeysToJson declared in MissionHealthTopologyJson.h.
 */

#include "MissionHealthTopologyJson.h"

#include <algorithm>
#include <vector>

#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{

nlohmann::json entityKeysToJson(std::vector<semantic::EntityKey> keys_in)
{
    std::sort(keys_in.begin(), keys_in.end());
    nlohmann::json json = nlohmann::json::array();
    for (const semantic::EntityKey &key : keys_in)
    {
        json.push_back({{"kind", semantic::entityKindName(key.kind)},
                        {"mapId", key.mapId},
                        {"entityId", key.entityId}});
    }
    return json;
}

} // namespace core
} // namespace vs_graphs
