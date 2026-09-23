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

    for (semantic::Room *room : rooms)
    {
        if (!room || room->isBad())
            continue;

        semantic::RoomContextSnapshot snap;
        snap.roomId                  = room->getId();
        semantic::Floor *p_snapFloor = room->getFloor();
        snap.floorId  = p_snapFloor != nullptr ? p_snapFloor->getId() : -1;
        snap.centroid = room->getCentroid();
        snap.wasConfirmedRoom =
            room->getRoomVariant() == semantic::Room::RoomVariant::ROOM;
        snap.wasPreviouslyVisited = room->hasPreviouslyVisited();
        snap.boundaryStatus       = static_cast<int>(room->getBoundaryStatus());
        snap.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();

        for (geometric::Plane *wall : room->getWalls())
        {
            semantic::WallBounds bounds;
            if (!wall || wall->isBad())
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
                room->getWallNormalTowardRoom_World(wall);
            if (orientedNormal)
                snap.wallNormals.push_back(*orientedNormal);
            else
                snap.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));

            snap.wallCentroids.push_back(wall->getCentroid());
            snap.wallDistances.push_back(wall->getGlobalEquation().distance());
            const geometric::Plane::GeometrySnapshot geometry =
                wall->getGeometrySnapshot();
            bounds.minU_m = geometry.minPlaneU_m;
            bounds.maxU_m = geometry.maxPlaneU_m;
            bounds.minV_m = geometry.minPlaneV_m;
            bounds.maxV_m = geometry.maxPlaneV_m;
            bounds.valid =
                std::isfinite(bounds.minU_m) && std::isfinite(bounds.maxU_m) &&
                std::isfinite(bounds.minV_m) && std::isfinite(bounds.maxV_m) &&
                bounds.maxU_m > bounds.minU_m && bounds.maxV_m > bounds.minV_m;
            snap.wallBounds.push_back(bounds);
        }

        for (semantic::Passage *passage : room->getPassages())
        {
            if (!passage)
            {
                snap.passageCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.passageContexts.push_back(semantic::PassageContext());
                continue;
            }
            snap.passageCentroids.push_back(passage->getCentroid());
            semantic::PassageContext context;
            context.id       = passage->getId();
            context.passable = passage->isPassable();
            const std::optional<int> roomIdOfPassageObservationConnection =
                passage->getProspectiveRoomId();
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            context.width_m       = passage->getWidth();
            context.height_m      = passage->getHeight();
            context.apertureValid = std::isfinite(context.width_m) &&
                                    std::isfinite(context.height_m) &&
                                    context.width_m > 0.0 &&
                                    context.height_m > 0.0;
            const semantic::Passage::KnownSideProvenance knownSide =
                passage->getKnownSideProvenance();
            context.hasKnownSideDirection = knownSide.hasDirection();
            if (context.hasKnownSideDirection)
            {
                context.knownSideDirection_World = knownSide.direction_World;
            }
            context.hasKnownSideRoom = knownSide.pRoom != nullptr;
            if (context.hasKnownSideRoom)
            {
                context.knownSideRoomId = knownSide.pRoom->getId();
            }
            context.traversalKnownToFarCount =
                passage->getTraversalKnownToFarCount();
            context.traversalFarToKnownCount =
                passage->getTraversalFarToKnownCount();
            context.traversalUnknownCount = passage->getTraversalUnknownCount();
            context.associatedWallCount   = passage->getAssociateWalls().size();
            context.hasBidirectionalTraversalEvidence =
                passage->hasBidirectionalTraversalEvidence();
            snap.passageContexts.push_back(context);
        }

        /* Assign persistent tag to old map rooms for merge trigger.
         * Tag format: "room_<id>" matches what matchRoomsToContext() assigns.
         */
        const std::string roomTag = "room_" + std::to_string(room->getId());
        snap.roomTag              = roomTag;
        if (!room->hasRoomTag())
        {
            room->setRoomTag(roomTag);
        }

        snapshots.push_back(snap);
    }

    const std::size_t exportedRoomCount = snapshots.size();
    {
        std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
        roomContextHistory[mapId] = std::move(snapshots);
    }

    /* Record the departure room for mission-chain tracing: the room the UAV
     * was following when this map was stranded. */
    const int departureRoomId = getCurrentSemanticRoomIdentity();
    if (departureRoomId >= 0)
    {
        for (semantic::Room *room : rooms)
        {
            if (room != nullptr && !room->isBad() &&
                room->getId() == departureRoomId)
            {
                p_activeMap->setFinalRoom(room);
                break;
            }
        }
    }

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;
}

} // namespace core
} // namespace vs_graphs
