/*!
 * @file         axiomCapabilitiesToJson.cc
 *
 * @brief        Implements axiomCapabilitiesToJson declared in
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

nlohmann::json axiomCapabilitiesToJson()
{
    std::vector<semantic::AxiomCapabilityEntry> table =
        semantic::computeAxiomCapabilityTable();
    std::sort(table.begin(),
              table.end(),
              [](const semantic::AxiomCapabilityEntry &lhs_in,
                 const semantic::AxiomCapabilityEntry &rhs_in)
              { return lhs_in.axiomCode < rhs_in.axiomCode; });

    nlohmann::json json = nlohmann::json::array();
    for (const semantic::AxiomCapabilityEntry &row : table)
    {
        json.push_back(
            {{"axiomCode", semantic::axiomCodeName(row.axiomCode)},
             {"classification", semantic::axiomClassName(row.classification)},
             {"capability", semantic::capabilityLevelName(row.capability)},
             {"missingProofOwner",
              semantic::missingProofOwnerName(row.owner)}});
    }
    return json;
}

} // namespace core
} // namespace vs_graphs
