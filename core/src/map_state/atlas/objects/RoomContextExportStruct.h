/*!
 * @file            RoomContextExportStruct.h
 *
 * @brief           Declares RoomContextExport, the plain-data room context of
 *                  one map that tracking exports when the map is left.
 */

#ifndef VS_GRAPHS_CORE_ATLAS_OBJECTS_ROOM_CONTEXT_EXPORT_STRUCT_H
#define VS_GRAPHS_CORE_ATLAS_OBJECTS_ROOM_CONTEXT_EXPORT_STRUCT_H

#include "scene_understanding/scene_graph/room/objects/RoomContextSnapshotStruct.h"

#include <vector>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

/*!
 * @brief           What a map leaves behind for the next one: the geometry of
 *                  its rooms and the anchor rooms at departure, as plain data
 *                  that holds no pointer into the scene graph. The semantics
 *                  manager's cycle refreshes a cached copy at its end, so
 *                  tracking can still export while a cycle holds the graph
 *                  lock.
 */
struct RoomContextExport
{
  public:
    /*!
     * @brief           Id of the map the context was captured from.
     */
    unsigned long mapId{0UL};

    /*!
     * @brief           True once the context was captured; false for a
     *                  default-constructed value.
     */
    bool isCaptured{false};

    /*!
     * @brief           Geometry of every live detected and candidate room, in
     *                  ascending room id with detected rooms first.
     */
    std::vector<semantic::RoomContextSnapshot> roomSnapshots;

    /*!
     * @brief           Id of the room the robot was in, or
     *                  map::Map::NO_LINKED_ROOM_ID when it was in none.
     */
    int finalRoomId{-1};

    /*!
     * @brief           Id of the passage whose whole opening was in view, or
     *                  -1 when none was.
     */
    int passageInViewId{-1};

    /*!
     * @brief           Id of the room beyond that passage, or
     *                  map::Map::NO_LINKED_ROOM_ID when none was seen.
     */
    int roomBeyondPassageId{-1};

    /*!
     * @brief           Id of the newest key frame, whose pose located the
     *                  departure, or -1 when there is none; for the log.
     */
    long lastKeyFrameId{-1L};
};

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_ATLAS_OBJECTS_ROOM_CONTEXT_EXPORT_STRUCT_H */
