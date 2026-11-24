/*!
 * @file            adoptMergedMapLinks.cc
 *
 * @brief           Implements Map::adoptMergedMapLinks(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::adoptMergedMapLinks(Map *p_absorbedMap_in)
{
    if (p_absorbedMap_in == nullptr || p_absorbedMap_in == this)
    {
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    int  absorbedStartingRoomId = NO_LINKED_ROOM_ID;
    int  absorbedFinalRoomId    = NO_LINKED_ROOM_ID;
    int  absorbedBeyondRoomId   = NO_LINKED_ROOM_ID;
    Map *p_absorbedFollowingMap = nullptr;
    {
        std::unique_lock<std::mutex> lock(p_absorbedMap_in->mapMutex);
        absorbedStartingRoomId = p_absorbedMap_in->startingRoom_id;
        absorbedFinalRoomId    = p_absorbedMap_in->finalRoom_id;
        absorbedBeyondRoomId   = p_absorbedMap_in->finalRoomBeyondPassage_id;
        p_absorbedFollowingMap = p_absorbedMap_in->p_followingMap;
    }

    /* The merged map starts where the earlier map started and ends where the
     * later map ended, with the later map's anchor candidates. Maps that are
     * not neighbours in the chain keep this map's links. */
    bool isFollowingAbsorbedMap = false;
    {
        std::unique_lock<std::mutex> lock(mapMutex);
        if (p_absorbedFollowingMap == this)
        {
            startingRoom_id = absorbedStartingRoomId;
        }
        else if (p_followingMap == p_absorbedMap_in)
        {
            finalRoom_id              = absorbedFinalRoomId;
            finalRoomBeyondPassage_id = absorbedBeyondRoomId;
            isFollowingAbsorbedMap    = true;
        }
    }

    /* The map after the absorbed one now follows this map, in both
     * directions. */
    if (isFollowingAbsorbedMap && linkFollowingMap(p_absorbedFollowingMap) !=
                                      MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: linkFollowingMap returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
