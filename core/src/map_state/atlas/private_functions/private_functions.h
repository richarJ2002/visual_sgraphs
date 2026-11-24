/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  Atlas translation units.
 *
 * @note            These helpers were file-scope entities inside the
 *                  anonymous namespace of Atlas.cc; external linkage here
 *                  is module-internal only. Names are kept verbatim
 *                  (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/RoomContextExportStruct.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include <atomic>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Moves an identity counter past an identity that already
 *                  exists, so the next reservation cannot repeat it. The
 *                  counter never goes down; a negative identity is ignored.
 *
 * @param[in,out]   nextIdentity_inout
 *                  Counter holding the next identity to hand out.
 *
 * @param[in]       observedIdentity_in
 *                  Identity already in use.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] atlas::AtlasStatus
    advanceIdentityAllocator(std::atomic<int> &nextIdentity_inout,
                             const int         observedIdentity_in);

/*!
 * @brief           Fills a room snapshot from a live room: id, floor, centroid,
 *                  confirmed or candidate variant, boundary status, capture
 *                  time (steady clock) and per wall and per passage geometry in
 *                  the world frame. Callers set the fields it does not touch
 *                  (wasPreviouslyVisited).
 *
 * @param[in]       p_room_in
 *                  Live room to read; must not be null.
 *
 * @param[out]      snapshot_out
 *                  Snapshot to fill; expected to be default-constructed.
 */
void captureRoomContext(semantic::Room                *p_room_in,
                        semantic::RoomContextSnapshot &snapshot_out);

/*!
 * @brief           Copies the room geometry of a map into plain data and finds
 *                  the anchor rooms at departure: the room with id
 *                  \p departureRoomId_in and, when a passage of it was in view
 *                  from the newest key frame, the room beyond that passage.
 *                  The caller holds a lock on the map's graph (shared or
 *                  exclusive) while this runs; the result holds no pointer
 *                  into the graph.
 *
 * @param[in]       semanticsManager_in
 *                  Manager that serves the map's graph.
 *
 * @param[in]       map_in
 *                  Map to read.
 *
 * @param[in]       departureRoomId_in
 *                  Id of the room the robot is in, or negative for none.
 *
 * @param[out]      roomContext_out
 *                  The context; isCaptured is true on return. Without any
 *                  live room its roomSnapshots stay empty.
 */
void captureMapRoomContext(const SemanticsManager   &semanticsManager_in,
                           map::Map                 &map_in,
                           int                       departureRoomId_in,
                           atlas::RoomContextExport &roomContext_out);

/*!
 * @brief           Tells whether a key frame's camera sees the whole opening
 *                  of a passage: both opening edges (centre plus and minus
 *                  half the width, across the passage plane) and the centre
 *                  project inside the image columns (the camera's horizontal
 *                  field of view) at a depth inside the sensor's range. When
 *                  the camera sees the opening it sees through it, so the
 *                  next map may build its first room from walls beyond it.
 *
 * @param[in]       p_passage_in
 *                  Passage to test, in the key frame's map; must not be null.
 *
 * @param[in]       p_keyFrame_in
 *                  Key frame whose pose and intrinsics (fx, cx, image column
 *                  bounds) are used; must not be null.
 *
 * @param[in]       upDirection_world_in
 *                  Unit up direction of the map's world frame.
 *
 * @param[in]       depthRange_m_in
 *                  Near and far depth limits of the sensor along the camera
 *                  z axis, metres.
 *
 * @param[out]      openingDepth_m_out
 *                  Depth of the passage centre along the camera z axis,
 *                  metres; 0 when the opening is not in view.
 *
 * @return          True when the whole opening is in view.
 */
[[nodiscard]] bool
    isPassageOpeningInView(semantic::Passage             *p_passage_in,
                           keyframe::KeyFrame            *p_keyFrame_in,
                           const Eigen::Vector3d         &upDirection_world_in,
                           const std::pair<float, float> &depthRange_m_in,
                           double                        &openingDepth_m_out);

/*!
 * @brief           Finds the room beyond the passage of the final room that
 *                  the camera saw when the map was left, the second anchor
 *                  candidate of the map chain link.
 *
 * @param[in]       rooms_in
 *                  Rooms of the map being left; borrowed, bad rooms skipped.
 *
 * @param[in]       p_finalRoom_in
 *                  Room the robot was in; borrowed, may be null.
 *
 * @param[in]       p_keyFrame_in
 *                  Last key frame of the map; borrowed, may be null.
 *
 * @param[in]       upDirection_world_in
 *                  Unit up direction of the map's world frame.
 *
 * @param[in]       depthRange_m_in
 *                  Near and far depth limits of the sensor, metres.
 *
 * @param[out]      p_passageInView_out
 *                  Passage that gave the room; nullptr when none did.
 *
 * @return          Among the final room's passages whose whole opening is in
 *                  view (isPassageOpeningInView), the nearest one's other
 *                  room: the other room listing that passage, else the
 *                  passage's far-side room when it is a room of rooms_in.
 *                  nullptr when no passage is in view or none has a room
 *                  beyond it. Borrowed.
 */
[[nodiscard]] semantic::Room *
    findRoomBeyondPassageInView(const std::vector<semantic::Room *> &rooms_in,
                                semantic::Room        *p_finalRoom_in,
                                keyframe::KeyFrame    *p_keyFrame_in,
                                const Eigen::Vector3d &upDirection_world_in,
                                const std::pair<float, float> &depthRange_m_in,
                                semantic::Passage *&p_passageInView_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H */
