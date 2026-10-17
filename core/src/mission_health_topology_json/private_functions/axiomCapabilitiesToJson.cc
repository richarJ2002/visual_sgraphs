/*!
 * @file            axiomCapabilitiesToJson.cc
 *
 * @brief           Implements axiomCapabilitiesToJson declared in
 *                  private_functions.h.
 */

#include "mission_health_topology_json/mission_health_topology_json.h"
#include "semantic/semantic_axiom_evaluator/objects/SemanticAxiomEvaluatorStatusEnum.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

#include "semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"
#include "semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"

namespace vs_graphs
{
namespace core
{

MissionHealthTopologyJsonStatus
    axiomCapabilitiesToJson(nlohmann::json &json_out)
{
    std::vector<semantic::AxiomCapabilityEntry> table{};
    if (semantic::computeAxiomCapabilityTable(table) !=
        semantic::SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeAxiomCapabilityTable returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(table.begin(),
              table.end(),
              [](const semantic::AxiomCapabilityEntry &lhs_in,
                 const semantic::AxiomCapabilityEntry &rhs_in)
              { return lhs_in.axiomCode < rhs_in.axiomCode; });

    nlohmann::json capabilitiesJson = nlohmann::json::array();
    for (const semantic::AxiomCapabilityEntry &row : table)
    {
        std::string axiomCodeName2{};
        if (semantic::axiomCodeName(row.axiomCode, axiomCodeName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomCodeName returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::string axiomClassName2{};
        if (semantic::axiomClassName(row.classification, axiomClassName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomClassName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string capabilityLevelName2{};
        if (semantic::capabilityLevelName(row.capability,
                                          capabilityLevelName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: capabilityLevelName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string missingProofOwnerName2{};
        if (semantic::missingProofOwnerName(row.owner,
                                            missingProofOwnerName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: missingProofOwnerName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        capabilitiesJson.push_back(
            {{"axiomCode", axiomCodeName2},
             {"classification", axiomClassName2},
             {"capability", capabilityLevelName2},
             {"missingProofOwner", missingProofOwnerName2}});
    }
    json_out = capabilitiesJson;
    return MissionHealthTopologyJsonStatus::
        MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
