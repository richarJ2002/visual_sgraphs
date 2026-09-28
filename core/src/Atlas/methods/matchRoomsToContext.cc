/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Atlas.h"

#if defined(VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK)
extern "C" void vsGraphsAtlasLockOrderBeforeMapSnapshot() __attribute__((weak));
#endif

namespace vs_graphs
{
namespace core
{

void Atlas::matchRoomsToContext(Map *p_newMap_in)
{
    if (!p_newMap_in)
        return;

    /* Candidate generation is intentionally separate from P4 verification.
     * This legacy method used cross-map centroids/normals and transferred live
     * walls before verification; it is retained as a disabled compatibility
     * entry point until the verified merge seam exists. */
    return;

    /* Snapshot Atlas membership before taking the room-context lock. Map
     * creation takes these locks in the opposite sequence by necessity. */
#if defined(VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK)
    if (vsGraphsAtlasLockOrderBeforeMapSnapshot != nullptr)
    {
        vsGraphsAtlasLockOrderBeforeMapSnapshot();
    }
#endif
    const std::vector<Map *> allMaps = getAllMaps();

    std::unique_lock<std::mutex> lock(roomContextMutex);

    if (roomContextHistory.empty())
        return;

    /* Match BOTH detected rooms AND candidate/prospective rooms.
     * Candidate rooms need identity tags for cross-restart continuity. */
    std::vector<semantic::Room *> newRooms =
        p_newMap_in->getAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms =
        p_newMap_in->getAllCandidateMapRooms();
    newRooms.insert(newRooms.end(),
                    candidateRooms.begin(),
                    candidateRooms.end());

    if (newRooms.empty())
        return;

    std::vector<semantic::RoomContextSnapshot> allContext;
    for (const auto &entry : roomContextHistory)
        for (const auto &snap : entry.second)
            allContext.push_back(snap);

    if (allContext.empty())
        return;

    for (semantic::Room *p_room : newRooms)
    {
        if (!p_room || p_room->isBad())
            continue;

        if (p_room->hasRoomTag())
            continue;

        Eigen::Vector3d roomCentroid = p_room->getCentroid();
        double          bestDistance = std::numeric_limits<double>::max();
        const semantic::RoomContextSnapshot *p_bestMatch = nullptr;

        for (const semantic::RoomContextSnapshot &snap : allContext)
        {
            /* PREFER tag-based matching if snapshot has persistent tag.
             * This provides deterministic identity across restarts. */
            if (!snap.roomTag.empty() && p_room->hasRoomTag())
            {
                if (p_room->getRoomTag() == snap.roomTag)
                {
                    p_bestMatch  = &snap;
                    bestDistance = 0.0;
                    break;
                }
            }

            double distance = (snap.centroid - roomCentroid).norm();

            if (distance > kRoomContextMatchThreshold_m)
                continue;

            /* Verify wall-normal agreement: compare the first available
             * wall normal of the new room against each snapshot wall
             * normal. Accept when |cosθ| > kWallNormalAlignmentCosTheta. */
            std::vector<geometric::Plane *> roomWalls = p_room->getWalls();
            if (roomWalls.empty() || snap.wallNormals.empty())
            {
                /* Fallback: accept on centroid distance alone when no wall
                 * normals are available for normal validation. */
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    p_bestMatch  = &snap;
                }
                continue;
            }

            std::optional<Eigen::Vector3d> newRoomNormal =
                p_room->getWallNormalTowardRoom_World(roomWalls[0]);
            if (!newRoomNormal)
                continue;

            bool normalOk = false;
            for (const Eigen::Vector3d &snapNormal : snap.wallNormals)
            {
                double dot = newRoomNormal->dot(snapNormal);
                if (std::abs(dot) > kWallNormalAlignmentCosTheta)
                {
                    normalOk = true;
                    break;
                }
            }

            if (!normalOk)
                continue;

            if (distance < bestDistance)
            {
                bestDistance = distance;
                p_bestMatch  = &snap;
            }
        }

        if (p_bestMatch)
        {
            p_room->setRoomTag("room_" + std::to_string(p_bestMatch->roomId));

            /* Locate the snapshot pointer in the stored history so the
             * room can hold a non-owning reference for WP3. */
            for (auto &entry : roomContextHistory)
            {
                for (auto &storedSnap : entry.second)
                {
                    if (storedSnap.roomId == p_bestMatch->roomId &&
                        storedSnap.centroid.isApprox(p_bestMatch->centroid))
                    {
                        p_room->setMatchedContext(&storedSnap);
                        break;
                    }
                }
            }

            /* ----------------------------------------------------------- *
             * CONTINUITY: Transfer walls/passages from prior room instance.
             * The matched prior room (in stored map) already has accumulated
             * boundary walls. Re-associate them to this new room instance
             * so boundary validation continues from where it left off.
             * ----------------------------------------------------------- */

            /* Find the stored map that contains the prior room. */
            Map            *p_priorMap  = nullptr;
            semantic::Room *p_priorRoom = nullptr;
            for (Map *p_map : allMaps)
            {
                if (!p_map || p_map->isBad() || p_map == p_newMap_in)
                    continue;
                for (semantic::Room *r : p_map->getAllDetectedMapRooms())
                {
                    if (r && !r->isBad() && r->getId() == p_bestMatch->roomId)
                    {
                        p_priorRoom = r;
                        p_priorMap  = p_map;
                        break;
                    }
                }
                if (p_priorRoom)
                    break;
            }

            if (p_priorRoom && p_priorMap)
            {
                std::cout << "[Atlas] Prior semantic::Room#"
                          << p_priorRoom->getId() << " has "
                          << p_priorRoom->getWalls().size() << " walls"
                          << std::endl;

                /* Transfer walls from prior room to current room */
                for (geometric::Plane *p_wall : p_priorRoom->getWalls())
                {
                    if (!p_wall || p_wall->isBad())
                        continue;

                    /* Re-associate wall to new room */
                    p_priorRoom->removeWall(p_wall);
                    p_room->setWalls(p_wall);

                    std::cout
                        << "[Atlas] Transferred Wall#" << p_wall->getId()
                        << " from prior semantic::Room#" << p_priorRoom->getId()
                        << " to matched semantic::Room#" << p_room->getId()
                        << std::endl;
                }

                /* Passages will be re-associated by associatePassagesToRooms()
                 */

                std::cout << "[Atlas] semantic::Room#" << p_room->getId()
                          << " now has " << p_room->getWalls().size()
                          << " walls (continuing from prior semantic::Room#"
                          << p_bestMatch->roomId << ")" << std::endl;
            }
            else
            {
                std::cout
                    << "[Atlas] NO prior room found for match (p_priorRoom="
                    << p_priorRoom << ", p_priorMap=" << p_priorMap << ")"
                    << std::endl;
            }

            std::cout << "[Atlas] Matched room " << p_room->getId()
                      << " in new map, tagged with identity \""
                      << p_room->getRoomTag() << "\" (prior room "
                      << p_bestMatch->roomId << ", dist=" << bestDistance
                      << " m)" << std::endl;
        }
    }
}

} // namespace core
} // namespace vs_graphs
