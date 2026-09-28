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

namespace vs_graphs
{
namespace core
{

void Atlas::exportRoomContextFromCurrentMap()
{
    if (!p_activeMap)
        return;

    /* Export BOTH confirmed detected rooms AND candidate/marker-based rooms.
     * Candidate rooms (prospective/provisional) may not have full wall loops
     * yet but still carry spatial identity needed for cross-restart matching.
     */
    std::vector<semantic::Room *> rooms = p_activeMap->getAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms =
        p_activeMap->getAllCandidateMapRooms();
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());

    if (rooms.empty())
        return;

    std::vector<semantic::RoomContextSnapshot> snapshots;
    snapshots.reserve(rooms.size());

    const long unsigned int mapId = p_activeMap->getId();

    for (semantic::Room *p_room : rooms)
    {
        if (!p_room || p_room->isBad())
            continue;

        semantic::RoomContextSnapshot snap;
        snap.roomId                  = p_room->getId();
        semantic::Floor *p_snapFloor = p_room->getFloor();
        snap.floorId  = p_snapFloor != nullptr ? p_snapFloor->getId() : -1;
        snap.centroid = p_room->getCentroid();
        snap.wasConfirmedRoom =
            p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM;
        snap.wasPreviouslyVisited = p_room->hasPreviouslyVisited();
        snap.boundaryStatus = static_cast<int>(p_room->getBoundaryStatus());
        snap.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();

        for (geometric::Plane *p_wall : p_room->getWalls())
        {
            semantic::WallBounds bounds;
            if (!p_wall || p_wall->isBad())
            {
                snap.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.wallCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.wallDistances.push_back(
                    std::numeric_limits<double>::quiet_NaN());
                snap.wallBounds.push_back(bounds);
                continue;
            }

            std::optional<Eigen::Vector3d> orientedNormal =
                p_room->getWallNormalTowardRoom_World(p_wall);
            if (orientedNormal)
                snap.wallNormals.push_back(*orientedNormal);
            else
                snap.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));

            snap.wallCentroids.push_back(p_wall->getCentroid());
            snap.wallDistances.push_back(
                p_wall->getGlobalEquation().distance());
            const geometric::Plane::GeometrySnapshot geometry =
                p_wall->getGeometrySnapshot();
            bounds.minU_m = geometry.minPlaneU_m;
            bounds.maxU_m = geometry.maxPlaneU_m;
            bounds.minV_m = geometry.minPlaneV_m;
            bounds.maxV_m = geometry.maxPlaneV_m;
            bounds.isValid =
                std::isfinite(bounds.minU_m) && std::isfinite(bounds.maxU_m) &&
                std::isfinite(bounds.minV_m) && std::isfinite(bounds.maxV_m) &&
                bounds.maxU_m > bounds.minU_m && bounds.maxV_m > bounds.minV_m;
            snap.wallBounds.push_back(bounds);
        }

        for (semantic::Passage *p_passage : p_room->getPassages())
        {
            if (!p_passage)
            {
                snap.passageCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.passageContexts.push_back(semantic::PassageContext());
                continue;
            }
            snap.passageCentroids.push_back(p_passage->getCentroid());
            semantic::PassageContext context;
            context.id         = p_passage->getId();
            context.isPassable = p_passage->isPassable();
            const std::optional<int> roomIdOfPassageObservationConnection =
                p_passage->getProspectiveRoomId();
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            context.width_m         = p_passage->getWidth();
            context.height_m        = p_passage->getHeight();
            context.isApertureValid = std::isfinite(context.width_m) &&
                                      std::isfinite(context.height_m) &&
                                      context.width_m > 0.0 &&
                                      context.height_m > 0.0;
            const semantic::Passage::KnownSideProvenance knownSide =
                p_passage->getKnownSideProvenance();
            context.hasKnownSideDirection = knownSide.hasDirection();
            if (context.hasKnownSideDirection)
            {
                context.knownSideDirection_World = knownSide.direction_World;
            }
            context.hasKnownSideRoom = knownSide.p_room != nullptr;
            if (context.hasKnownSideRoom)
            {
                context.knownSideRoomId = knownSide.p_room->getId();
            }
            context.traversalKnownToFarCount =
                p_passage->getTraversalKnownToFarCount();
            context.traversalFarToKnownCount =
                p_passage->getTraversalFarToKnownCount();
            context.traversalUnknownCount =
                p_passage->getTraversalUnknownCount();
            context.associatedWallCount = p_passage->getAssociateWalls().size();
            context.hasBidirectionalTraversalEvidence =
                p_passage->hasBidirectionalTraversalEvidence();
            snap.passageContexts.push_back(context);
        }

        /* Assign persistent tag to old map rooms for merge trigger.
         * Tag format: "room_<id>" matches what matchRoomsToContext() assigns.
         */
        const std::string roomTag = "room_" + std::to_string(p_room->getId());
        snap.roomTag              = roomTag;
        if (!p_room->hasRoomTag())
        {
            p_room->setRoomTag(roomTag);
        }

        snapshots.push_back(snap);
    }

    const std::size_t exportedRoomCount = snapshots.size();
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        roomContextHistory[mapId] = std::move(snapshots);
    }

    /* Record the departure room for mission-chain tracing: the room the UAV
     * was following when this map was stranded. */
    const int departureRoomId = getCurrentSemanticRoomIdentity();
    if (departureRoomId >= 0)
    {
        for (semantic::Room *p_room : rooms)
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getId() == departureRoomId)
            {
                p_activeMap->setFinalRoom(p_room);
                break;
            }
        }
    }

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;
}

} // namespace core
} // namespace vs_graphs
