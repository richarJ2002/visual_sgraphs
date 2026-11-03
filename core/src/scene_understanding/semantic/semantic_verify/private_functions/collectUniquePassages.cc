/*!
 * @file            collectUniquePassages.cc
 *
 * @brief           Implements collectUniquePassages(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <set>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<PassageContext> collectUniquePassages(
    const std::vector<SemanticMergeRoomEvidence> &rooms_in)
{
    std::vector<PassageContext> passages;
    std::set<int>               seenPassageIds;
    for (const SemanticMergeRoomEvidence &room : rooms_in)
    {
        for (const PassageContext &passage : room.context.passageContexts)
        {
            if (seenPassageIds.insert(passage.id).second)
            {
                passages.push_back(passage);
            }
        }
    }
    return passages;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
