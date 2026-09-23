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

std::vector<semantic::RoomContextSnapshot>
    Atlas::copyRoomContextForMap(Map *p_map_in)
{
    std::vector<semantic::RoomContextSnapshot> snapshots;
    if (p_map_in == nullptr)
        return snapshots;
    std::vector<semantic::Room *> rooms = p_map_in->getAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms =
        p_map_in->getAllCandidateMapRooms();
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());
    for (semantic::Room *p_room : rooms)
    {
        if (p_room == nullptr || p_room->isBad())
            continue;
        semantic::RoomContextSnapshot snapshot;
        snapshot.roomId                  = p_room->getId();
        semantic::Floor *p_snapshotFloor = p_room->getFloor();
        snapshot.floorId =
            p_snapshotFloor != nullptr ? p_snapshotFloor->getId() : -1;
        snapshot.centroid = p_room->getCentroid();
        snapshot.wasConfirmedRoom =
            p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM;
        snapshot.boundaryStatus = static_cast<int>(p_room->getBoundaryStatus());
        snapshot.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        snapshot.roomTag = p_room->hasRoomTag() ? p_room->getRoomTag() : "";
        for (geometric::Plane *p_wall : p_room->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                snapshot.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.wallCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.wallDistances.push_back(
                    std::numeric_limits<double>::quiet_NaN());
                snapshot.wallBounds.push_back(semantic::WallBounds());
                continue;
            }
            const std::optional<Eigen::Vector3d> normal =
                p_room->getWallNormalTowardRoom_World(p_wall);
            if (normal)
                snapshot.wallNormals.push_back(*normal);
            else
                snapshot.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
            snapshot.wallCentroids.push_back(p_wall->getCentroid());
            snapshot.wallDistances.push_back(
                p_wall->getGlobalEquation().distance());
            const geometric::Plane::GeometrySnapshot geometry =
                p_wall->getGeometrySnapshot();
            snapshot.wallBounds.push_back(
                {std::isfinite(geometry.minPlaneU_m) &&
                     std::isfinite(geometry.maxPlaneU_m) &&
                     std::isfinite(geometry.minPlaneV_m) &&
                     std::isfinite(geometry.maxPlaneV_m) &&
                     geometry.maxPlaneU_m > geometry.minPlaneU_m &&
                     geometry.maxPlaneV_m > geometry.minPlaneV_m,
                 geometry.minPlaneU_m,
                 geometry.maxPlaneU_m,
                 geometry.minPlaneV_m,
                 geometry.maxPlaneV_m});
        }
        for (semantic::Passage *p_passage : p_room->getPassages())
        {
            if (p_passage == nullptr)
            {
                snapshot.passageCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.passageContexts.push_back(semantic::PassageContext());
                continue;
            }
            snapshot.passageCentroids.push_back(p_passage->getCentroid());
            semantic::PassageContext context;
            context.id       = p_passage->getId();
            context.passable = p_passage->isPassable();
            const std::optional<int> roomIdOfPassageObservationConnection =
                p_passage->getProspectiveRoomId();
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            context.width_m       = p_passage->getWidth();
            context.height_m      = p_passage->getHeight();
            context.apertureValid = std::isfinite(context.width_m) &&
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
            context.hasKnownSideRoom = knownSide.pRoom != nullptr;
            if (context.hasKnownSideRoom)
            {
                context.knownSideRoomId = knownSide.pRoom->getId();
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
            snapshot.passageContexts.push_back(context);
        }
        snapshots.push_back(std::move(snapshot));
    }
    return snapshots;
}

} // namespace core
} // namespace vs_graphs
