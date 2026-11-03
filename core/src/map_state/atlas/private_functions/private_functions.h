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
#include "scene_understanding/semantic/room/room.h"
#include <atomic>

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
 *                  (roomTag, wasPreviouslyVisited).
 *
 * @param[in]       p_room_in
 *                  Live room to read; must not be null.
 *
 * @param[out]      snapshot_out
 *                  Snapshot to fill; expected to be default-constructed.
 */
void captureRoomContext(semantic::Room                *p_room_in,
                        semantic::RoomContextSnapshot &snapshot_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H */
