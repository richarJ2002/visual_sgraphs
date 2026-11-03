/*!
 * @file            findAnchorRoomPairs.cc
 *
 * @brief           Implements findAnchorRoomPairs(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<MergePair> findAnchorRoomPairs(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in)
{
    std::vector<MergePair> anchorPairs;
    for (std::size_t absorbedIndex = 0U;
         absorbedIndex < absorbedRooms_in.size();
         absorbedIndex++)
    {
        const std::string &absorbedTag =
            absorbedRooms_in[absorbedIndex].context.roomTag;
        if (absorbedTag.empty())
        {
            continue;
        }
        for (std::size_t survivingIndex = 0U;
             survivingIndex < survivingRooms_in.size();
             survivingIndex++)
        {
            if (survivingRooms_in[survivingIndex].context.roomTag ==
                absorbedTag)
            {
                anchorPairs.push_back({absorbedIndex, survivingIndex});
            }
        }
    }
    return anchorPairs;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
