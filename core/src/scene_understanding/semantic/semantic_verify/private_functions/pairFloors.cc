/*!
 * @file            pairFloors.cc
 *
 * @brief           Implements pairFloors(), declared in private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<MergePair>
    pairFloors(const std::vector<SemanticMergeRoomEvidence>
                   &absorbedRooms_survivingWorld_in,
               const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
               const SemanticVerifyConfig                   &configuration_in)
{
    std::vector<MergePair> floorPairs;
    for (std::size_t absorbedIndex = 0U;
         absorbedIndex < absorbedRooms_survivingWorld_in.size();
         absorbedIndex++)
    {
        const SemanticMergeRoomEvidence &absorbedRoom =
            absorbedRooms_survivingWorld_in[absorbedIndex];
        if (!absorbedRoom.floorPlane_world.has_value())
        {
            continue;
        }
        const Eigen::Vector4d &absorbedFloor_world =
            *absorbedRoom.floorPlane_world;

        /* Height is compared where the absorbed room is: the foot of its
         * centroid on its own floor plane (a point on that plane). */
        const double centroidHeight_m =
            absorbedFloor_world.head<3>().dot(absorbedRoom.context.centroid) +
            absorbedFloor_world(3);
        const Eigen::Vector3d footPoint_world =
            absorbedRoom.context.centroid -
            centroidHeight_m * absorbedFloor_world.head<3>();

        for (std::size_t survivingIndex = 0U;
             survivingIndex < survivingRooms_in.size();
             survivingIndex++)
        {
            if (!survivingRooms_in[survivingIndex].floorPlane_world.has_value())
            {
                continue;
            }
            const PlaneGap floorGap = measurePlaneGap(
                absorbedFloor_world,
                *survivingRooms_in[survivingIndex].floorPlane_world,
                footPoint_world,
                true);
            if (floorGap.normalAngle_deg <=
                    configuration_in.coplanarAngle_deg &&
                floorGap.offset_m <= configuration_in.floorMatchTolerance_m)
            {
                floorPairs.push_back({absorbedIndex, survivingIndex});
            }
        }
    }
    return floorPairs;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
