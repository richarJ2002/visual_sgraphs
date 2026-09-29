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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::copyRoomContextForMap(
    Map                                        *p_map_in,
    std::vector<semantic::RoomContextSnapshot> &roomContexts_out)
{
    std::vector<semantic::RoomContextSnapshot> snapshots;
    if (p_map_in == nullptr)
    {
        roomContexts_out = snapshots;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    std::vector<semantic::Room *> rooms{};
    if (p_map_in->getAllDetectedMapRooms(rooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> candidateRooms{};
    if (p_map_in->getAllCandidateMapRooms(candidateRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllCandidateMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());
    for (semantic::Room *p_room : rooms)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
            continue;
        semantic::RoomContextSnapshot snapshot;
        int                           roomId2{};
        if (p_room->getId(roomId2) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.roomId                  = roomId2;
        semantic::Floor *p_snapshotFloor = nullptr;
        if (p_room->getFloor(p_snapshotFloor) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFloor returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int snapshotFloorId{};
        if ((p_snapshotFloor != nullptr) &&
            p_snapshotFloor->getId(snapshotFloorId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.floorId = p_snapshotFloor != nullptr ? snapshotFloorId : -1;
        Eigen::Vector3d roomCentroid{};
        if (p_room->getCentroid(roomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.centroid = roomCentroid;
        semantic::Room::RoomVariant roomVariant{};
        if (p_room->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.wasConfirmedRoom =
            roomVariant == semantic::Room::RoomVariant::ROOM;
        semantic::Room::BoundaryStatus roomBoundaryStatus{};
        if (p_room->getBoundaryStatus(roomBoundaryStatus) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBoundaryStatus returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
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
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string roomTag2{};
        if ((roomHasRoomTag) && p_room->getRoomTag(roomTag2) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.roomTag = roomHasRoomTag ? roomTag2 : "";
        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls)
        {
            bool wallIsBad{};
            if (!(p_wall == nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall == nullptr || wallIsBad)
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
            Eigen::Vector3d wallGetCentroid{};
            if (p_wall->getCentroid(wallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            snapshot.wallCentroids.push_back(wallGetCentroid);
            g2o::Plane3D wallGetGlobalEquation{};
            if (p_wall->getGlobalEquation(wallGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            snapshot.wallDistances.push_back(wallGetGlobalEquation.distance());
            geometric::Plane::GeometrySnapshot geometry{};
            if (p_wall->getGeometrySnapshot(geometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            snapshot.passageCentroids.push_back(passageCentroid);
            semantic::PassageContext context;
            int                      passageId{};
            if (p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            context.id = passageId;
            bool passageIsPassable{};
            if (p_passage->isPassable(passageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            context.isPassable = passageIsPassable;
            std::optional<int> roomIdOfPassageObservationConnection{};
            if (p_passage->getProspectiveRoomId(
                    roomIdOfPassageObservationConnection) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getProspectiveRoomId returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            double passageWidth{};
            if (p_passage->getWidth(passageWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            context.width_m = passageWidth;
            double passageHeight{};
            if (p_passage->getHeight(passageHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
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
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            bool knownSideHasDirection{};
            if (knownSide.hasDirection(knownSideHasDirection) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasDirection returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                context.knownSideRoomId = id2;
            }
            std::size_t passageTraversalKnownToFarCount{};
            if (p_passage->getTraversalKnownToFarCount(
                    passageTraversalKnownToFarCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getTraversalKnownToFarCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            context.traversalKnownToFarCount = passageTraversalKnownToFarCount;
            std::size_t passageTraversalFarToKnownCount{};
            if (p_passage->getTraversalFarToKnownCount(
                    passageTraversalFarToKnownCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getTraversalFarToKnownCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            context.traversalFarToKnownCount = passageTraversalFarToKnownCount;
            std::size_t passageTraversalUnknownCount{};
            if (p_passage->getTraversalUnknownCount(
                    passageTraversalUnknownCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getTraversalUnknownCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            context.traversalUnknownCount = passageTraversalUnknownCount;
            std::vector<vs_graphs::core::geometric::Plane *>
                passageAssociateWalls{};
            if (p_passage->getAssociateWalls(passageAssociateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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
    roomContexts_out = snapshots;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
