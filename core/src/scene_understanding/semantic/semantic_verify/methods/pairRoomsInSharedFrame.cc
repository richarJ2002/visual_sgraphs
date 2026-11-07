/*!
 * @file            pairRoomsInSharedFrame.cc
 *
 * @brief           Implements SemanticVerify::pairRoomsInSharedFrame(),
 *                  declared in semantic_verify/objects/SemanticVerifyClass.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "scene_understanding/scene_graph/Room/Room.h"

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<std::pair<Room *, Room *>> SemanticVerify::pairRoomsInSharedFrame(
    const std::vector<Room *>  &importedRooms_in,
    const std::vector<Room *>  &retainedRooms_in,
    const Room                 *p_importedAnchorRoom_in,
    const Room                 *p_retainedAnchorRoom_in,
    const SemanticVerifyConfig &configuration_in)
{
    std::vector<Room *>                    importedAdmittedRooms{};
    std::vector<Room *>                    retainedAdmittedRooms{};
    std::vector<SemanticMergeRoomEvidence> importedEvidence;
    std::vector<SemanticMergeRoomEvidence> retainedEvidence;
    collectRoomEvidence(importedRooms_in,
                        configuration_in,
                        importedAdmittedRooms,
                        importedEvidence);
    collectRoomEvidence(retainedRooms_in,
                        configuration_in,
                        retainedAdmittedRooms,
                        retainedEvidence);

    /* Both lists already live in one frame, so no transform is applied. */
    const std::vector<MergePair> anchorPairs =
        findAnchorRoomPairs(retainedAdmittedRooms,
                            importedAdmittedRooms,
                            p_retainedAnchorRoom_in,
                            p_importedAnchorRoom_in);
    const std::vector<RoomPair> wallPairedRooms =
        pairRooms(importedEvidence, retainedEvidence, configuration_in);

    constexpr std::size_t NO_ROOM = std::numeric_limits<std::size_t>::max();
    std::vector<std::pair<Room *, Room *>> duplicatePairs;
    for (std::size_t importedIndex = 0U;
         importedIndex < importedAdmittedRooms.size();
         importedIndex++)
    {
        std::size_t bestRetainedIndex = NO_ROOM;

        /* The map chain link is the only identity link; it needs no
         * geometry. */
        for (const MergePair &anchorPair : anchorPairs)
        {
            if (anchorPair.absorbedIndex == importedIndex)
            {
                bestRetainedIndex = anchorPair.survivingIndex;
                break;
            }
        }

        /* Otherwise the room whose walls pair most wins. A tie is ambiguous
         * and fuses nothing. */
        std::size_t bestWallPairCount = 0U;
        bool        isTied            = false;
        if (bestRetainedIndex == NO_ROOM)
        {
            for (const RoomPair &roomPair : wallPairedRooms)
            {
                if (roomPair.absorbedIndex != importedIndex)
                {
                    continue;
                }
                if (roomPair.wallPairs.size() > bestWallPairCount)
                {
                    bestWallPairCount = roomPair.wallPairs.size();
                    bestRetainedIndex = roomPair.survivingIndex;
                    isTied            = false;
                }
                else if (roomPair.wallPairs.size() == bestWallPairCount)
                {
                    isTied = true;
                }
            }
        }

        if (bestRetainedIndex == NO_ROOM || isTied ||
            retainedAdmittedRooms[bestRetainedIndex] ==
                importedAdmittedRooms[importedIndex])
        {
            continue;
        }
        duplicatePairs.push_back({importedAdmittedRooms[importedIndex],
                                  retainedAdmittedRooms[bestRetainedIndex]});
    }
    return duplicatePairs;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
