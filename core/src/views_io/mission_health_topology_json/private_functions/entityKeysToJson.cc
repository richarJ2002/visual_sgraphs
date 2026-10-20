/*!
 * @file            entityKeysToJson.cc
 *
 * @brief           Implements entityKeysToJson declared in
 *                  private_functions.h.
 */

#include "views_io/mission_health_topology_json/mission_health_topology_json.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/SemanticAxiomEvaluatorStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/semantic_graph_snapshot.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

#include "scene_understanding/semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"

namespace vs_graphs
{
namespace core
{

MissionHealthTopologyJsonStatus
    entityKeysToJson(std::vector<semantic::EntityKey> keys_in,
                     nlohmann::json                  &json_out)
{
    std::sort(keys_in.begin(), keys_in.end());
    nlohmann::json entityKeysJson = nlohmann::json::array();
    for (const semantic::EntityKey &key : keys_in)
    {
        std::string entityKindName2{};
        if (semantic::entityKindName(key.kind, entityKindName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: entityKindName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        entityKeysJson.push_back({{"kind", entityKindName2},
                                  {"mapId", key.mapId},
                                  {"entityId", key.entityId}});
    }
    json_out = entityKeysJson;
    return MissionHealthTopologyJsonStatus::
        MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
