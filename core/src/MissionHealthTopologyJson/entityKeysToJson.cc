/*!
 * @file         entityKeysToJson.cc
 *
 * @brief        Implements entityKeysToJson declared in
 *               private_functions.h.
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
    nlohmann::json entityKeysJson = nlohmann::json::array();
    for (const semantic::EntityKey &key : keys_in)
    {
        entityKeysJson.push_back({{"kind", semantic::entityKindName(key.kind)},
                                  {"mapId", key.mapId},
                                  {"entityId", key.entityId}});
    }
    return entityKeysJson;
}

} // namespace core
} // namespace vs_graphs
