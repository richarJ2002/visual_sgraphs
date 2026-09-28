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
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room == nullptr || roomIsBad)
            continue;
        semantic::RoomContextSnapshot snapshot;
        int                           roomId2{};
        if (p_room->getId(roomId2) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        snapshot.roomId                  = roomId2;
        semantic::Floor *p_snapshotFloor = nullptr;
        if (p_room->getFloor(p_snapshotFloor) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getFloor cannot fail; continue as before.
        }
        int snapshotFloorId{};
        if ((p_snapshotFloor != nullptr) &&
            p_snapshotFloor->getId(snapshotFloorId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        snapshot.floorId = p_snapshotFloor != nullptr ? snapshotFloorId : -1;
        Eigen::Vector3d roomCentroid{};
        if (p_room->getCentroid(roomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        snapshot.centroid = roomCentroid;
        semantic::Room::RoomVariant roomVariant{};
        if (p_room->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }
        snapshot.wasConfirmedRoom =
            roomVariant == semantic::Room::RoomVariant::ROOM;
        semantic::Room::BoundaryStatus roomBoundaryStatus{};
        if (p_room->getBoundaryStatus(roomBoundaryStatus) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getBoundaryStatus cannot fail; continue as before.
        }
        snapshot.boundaryStatus = static_cast<int>(roomBoundaryStatus);
        snapshot.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        bool roomHasRoomTag{};
        if (p_room->hasRoomTag(roomHasRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // hasRoomTag cannot fail; continue as before.
        }
        std::string roomTag2{};
        if ((roomHasRoomTag) && p_room->getRoomTag(roomTag2) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomTag cannot fail; continue as before.
        }
        snapshot.roomTag = roomHasRoomTag ? roomTag2 : "";
        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        for (geometric::Plane *p_wall : roomWalls)
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
            std::optional<Eigen::Vector3d> normal{};
            if (p_room->getWallNormalTowardRoom_World(p_wall, normal) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWallNormalTowardRoom_World cannot fail; continue as
                // before.
            }
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
        std::vector<vs_graphs::core::semantic::Passage *> roomPassages{};
        if (p_room->getPassages(roomPassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getPassages cannot fail; continue as before.
        }
        for (semantic::Passage *p_passage : roomPassages)
        {
            if (p_passage == nullptr)
            {
                snapshot.passageCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.passageContexts.push_back(semantic::PassageContext());
                continue;
            }
            Eigen::Vector3d passageCentroid{};
            if (p_passage->getCentroid(passageCentroid) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            snapshot.passageCentroids.push_back(passageCentroid);
            semantic::PassageContext context;
            int                      passageId{};
            if (p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            context.id = passageId;
            bool passageIsPassable{};
            if (p_passage->isPassable(passageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // isPassable cannot fail; continue as before.
            }
            context.isPassable = passageIsPassable;
            std::optional<int> roomIdOfPassageObservationConnection{};
            if (p_passage->getProspectiveRoomId(
                    roomIdOfPassageObservationConnection) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getProspectiveRoomId cannot fail; continue as before.
            }
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            double passageWidth{};
            if (p_passage->getWidth(passageWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getWidth cannot fail; continue as before.
            }
            context.width_m = passageWidth;
            double passageHeight{};
            if (p_passage->getHeight(passageHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getHeight cannot fail; continue as before.
            }
            context.height_m        = passageHeight;
            context.isApertureValid = std::isfinite(context.width_m) &&
                                      std::isfinite(context.height_m) &&
                                      context.width_m > 0.0 &&
                                      context.height_m > 0.0;
            semantic::Passage::KnownSideProvenance knownSide{};
            if (p_passage->getKnownSideProvenance(knownSide) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getKnownSideProvenance cannot fail; continue as before.
            }
            bool knownSideHasDirection{};
            if (knownSide.hasDirection(knownSideHasDirection) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                // hasDirection cannot fail; continue as before.
            }
            context.hasKnownSideDirection = knownSideHasDirection;
            if (context.hasKnownSideDirection)
            {
                context.knownSideDirection_World = knownSide.direction_World;
            }
            context.hasKnownSideRoom = knownSide.p_room != nullptr;
            if (context.hasKnownSideRoom)
            {
                int id2{};
                if (knownSide.p_room->getId(id2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                context.knownSideRoomId = id2;
            }
            std::size_t passageTraversalKnownToFarCount{};
            if (p_passage->getTraversalKnownToFarCount(
                    passageTraversalKnownToFarCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getTraversalKnownToFarCount cannot fail; continue as before.
            }
            context.traversalKnownToFarCount = passageTraversalKnownToFarCount;
            std::size_t passageTraversalFarToKnownCount{};
            if (p_passage->getTraversalFarToKnownCount(
                    passageTraversalFarToKnownCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getTraversalFarToKnownCount cannot fail; continue as before.
            }
            context.traversalFarToKnownCount = passageTraversalFarToKnownCount;
            std::size_t passageTraversalUnknownCount{};
            if (p_passage->getTraversalUnknownCount(
                    passageTraversalUnknownCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getTraversalUnknownCount cannot fail; continue as before.
            }
            context.traversalUnknownCount = passageTraversalUnknownCount;
            std::vector<vs_graphs::core::geometric::Plane *>
                passageAssociateWalls{};
            if (p_passage->getAssociateWalls(passageAssociateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getAssociateWalls cannot fail; continue as before.
            }
            context.associatedWallCount = passageAssociateWalls.size();
            bool passageHasBidirectionalTraversalEvidence{};
            if (p_passage->hasBidirectionalTraversalEvidence(
                    passageHasBidirectionalTraversalEvidence) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // hasBidirectionalTraversalEvidence cannot fail; continue as
                // before.
            }
            context.hasBidirectionalTraversalEvidence =
                passageHasBidirectionalTraversalEvidence;
            snapshot.passageContexts.push_back(context);
        }
        snapshots.push_back(std::move(snapshot));
    }
    return snapshots;
}

} // namespace core
} // namespace vs_graphs
