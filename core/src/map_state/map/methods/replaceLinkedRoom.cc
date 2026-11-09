/*!
 * @file            replaceLinkedRoom.cc
 *
 * @brief           Implements Map::replaceLinkedRoom(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::replaceLinkedRoom(int fusedRoomId_in, int keptRoomId_in)
{
    std::unique_lock<std::mutex> lock(mapMutex);
    if (startingRoom_id == fusedRoomId_in)
    {
        startingRoom_id = keptRoomId_in;
    }
    if (finalRoom_id == fusedRoomId_in)
    {
        finalRoom_id = keptRoomId_in;
    }
    if (finalRoomBeyondPassage_id == fusedRoomId_in)
    {
        finalRoomBeyondPassage_id = keptRoomId_in;
    }

    /* Two candidates fused into one room leave a single candidate. */
    if (finalRoomBeyondPassage_id == finalRoom_id)
    {
        finalRoomBeyondPassage_id = NO_LINKED_ROOM_ID;
    }
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
