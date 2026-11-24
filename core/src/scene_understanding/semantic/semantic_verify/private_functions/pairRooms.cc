/*!
 * @file            pairRooms.cc
 *
 * @brief           Implements pairRooms(), declared in private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <algorithm>
#include <set>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<RoomPair>
    pairRooms(const std::vector<SemanticMergeRoomEvidence>
                  &absorbedRooms_survivingWorld_in,
              const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
              const SemanticVerifyConfig                   &configuration_in)
{
    std::vector<RoomPair> roomPairs;
    for (std::size_t absorbedIndex = 0U;
         absorbedIndex < absorbedRooms_survivingWorld_in.size();
         absorbedIndex++)
    {
        const SemanticMergeRoomEvidence &absorbedRoom =
            absorbedRooms_survivingWorld_in[absorbedIndex];
        for (std::size_t survivingIndex = 0U;
             survivingIndex < survivingRooms_in.size();
             survivingIndex++)
        {
            const SemanticMergeRoomEvidence &survivingRoom =
                survivingRooms_in[survivingIndex];
            const std::size_t smallerWallCount =
                std::min(absorbedRoom.walls.size(), survivingRoom.walls.size());
            if (smallerWallCount == 0U)
            {
                continue;
            }

            MergePairing          wallPairing = pairWalls(absorbedRoom.walls,
                                                 survivingRoom.walls,
                                                 configuration_in);
            std::set<std::size_t> pairedAbsorbedWalls;
            std::set<std::size_t> pairedSurvivingWalls;
            for (const MergePair &wallPair : wallPairing.pairs)
            {
                pairedAbsorbedWalls.insert(wallPair.absorbedIndex);
                pairedSurvivingWalls.insert(wallPair.survivingIndex);
            }
            const std::size_t pairedWallCount =
                std::min(pairedAbsorbedWalls.size(),
                         pairedSurvivingWalls.size());
            if (pairedWallCount >= configuration_in.roomPairMinWalls &&
                static_cast<double>(pairedWallCount) >=
                    configuration_in.roomPairWallFraction *
                        static_cast<double>(smallerWallCount))
            {
                roomPairs.push_back({absorbedIndex,
                                     survivingIndex,
                                     std::move(wallPairing.pairs)});
            }
        }
    }
    return roomPairs;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
