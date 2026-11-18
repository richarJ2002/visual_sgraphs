/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            enforcePassageApertureBackstop.cc
 *
 * @brief           Implements
 *                  SemanticsManager::enforcePassageApertureBackstop(), declared
 *                  in SemanticsManager.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::enforcePassageApertureBackstop(
    semantic::Room                                  *p_room_inout,
    geometric::Plane                                *p_wall_in,
    const std::vector<semantic::Passage *>          &allPassages_in,
    const Eigen::Vector3d                           &groundNormal_world_in,
    SemanticsManager::PassageSideEnforcementOutcome &outcome_out)
{
    bool room_inoutIsBad{};
    if (!(p_room_inout == nullptr) &&
        p_room_inout->isBad(room_inoutIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool wallIsBad{};
    if (!(p_room_inout == nullptr || room_inoutIsBad || p_wall_in == nullptr) &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_room_inout == nullptr || room_inoutIsBad || p_wall_in == nullptr ||
        wallIsBad)
    {
        outcome_out = PassageSideEnforcementOutcome::NO_VIOLATION;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* The wall's support points: the part of it actually observed. */
    geometric::Plane::GeometrySnapshot wallGeometry{};
    if (p_wall_in->getGeometrySnapshot(wallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    for (semantic::Passage *p_passage : allPassages_in)
    {
        bool passageIsBad{};
        if (!(p_passage == nullptr) &&
            p_passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passage == nullptr || passageIsBad)
        {
            continue;
        }

        const double minimumSideDistance_m = static_cast<double>(
            p_sysParams->roomSeg.passagePartition.minimumSideDistance_m);

        /* The passage's far-side handle (the prospective room, or the
         * confirmed room beyond it once both sides are known). The room under
         * test is either that far-side room or a room on the known side. */
        vs_graphs::core::semantic::Room *p_prospective = nullptr;
        if (p_passage->getProspectiveRoom(p_prospective) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool isFarSideRoom = p_prospective == p_room_inout;

        /* segmentCrossesPassageOpening reports "no crossing" whenever the
         * segment start lies within minimumSideDistance_m of the passage
         * plane, which the centroid of a sparsely observed room often does.
         * When the passage's known-side direction is available, substitute a
         * point pushed off the plane on the tested room's own side: along the
         * known-side direction for a known-side room, against it for the
         * far-side room. Without a known side the residual sign is noise, so
         * the start is left unchanged. For the room the camera is in, the
         * camera is the viewpoint instead (see below). */
        Eigen::Vector3d segmentStart_world_m{};
        Eigen::Vector3d cameraCenter_world_m{};
        bool            hasCameraViewpoint{};
        if (getRoomViewpoints(p_room_inout,
                              segmentStart_world_m,
                              cameraCenter_world_m,
                              hasCameraViewpoint) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomViewpoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        g2o::Plane3D passageGlobalEquation{};
        if (p_passage->getGlobalEquation(passageGlobalEquation) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d passageEquation_world = passageGlobalEquation.coeffs();
        const double passageNormalNorm = passageEquation_world.head<3>().norm();
        const bool   hasPassagePlane =
            passageEquation_world.allFinite() && passageNormalNorm > 1e-8;
        Eigen::Vector3d passageNormal_world = Eigen::Vector3d::Zero();
        if (hasPassagePlane)
        {
            passageEquation_world /= passageNormalNorm;
            passageNormal_world = passageEquation_world.head<3>();
            const double roomViewpointSide_m =
                passageNormal_world.dot(segmentStart_world_m) +
                passageEquation_world(3);

            if (std::abs(roomViewpointSide_m) < minimumSideDistance_m)
            {
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
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (knownSideHasDirection)
                {
                    Eigen::Vector3d passageCentroid{};
                    if (p_passage->getCentroid(passageCentroid) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const double roomSideOffset_m =
                        (isFarSideRoom ? -2.0 : 2.0) * minimumSideDistance_m;
                    segmentStart_world_m =
                        passageCentroid +
                        roomSideOffset_m * knownSide.knownSideDirection_world;
                }
            }
        }

        Eigen::Vector3d wallGetCentroid{};
        if (p_wall_in->getCentroid(wallGetCentroid) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const auto crossesTo =
            [this, p_passage, &groundNormal_world_in, minimumSideDistance_m](
                const Eigen::Vector3d &segmentStart_in,
                const Eigen::Vector3d &segmentEnd_in)
        {
            bool crossesPassageOpening_out{};
            if (segmentCrossesPassageOpening(
                    segmentStart_in,
                    segmentEnd_in,
                    p_passage,
                    groundNormal_world_in,
                    static_cast<double>(
                        p_sysParams->roomSeg.passagePartition.openingMargin_m),
                    minimumSideDistance_m,
                    crossesPassageOpening_out) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: segmentCrossesPassageOpening returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            return crossesPassageOpening_out;
        };
        /* The wall is reached through the opening when the line to its
         * centroid is, or when the lines to most of its observed support
         * points are: a wall seen through a doorway is often only partly in
         * view, and its centroid may lie behind the solid part of the door
         * wall. Support points on the start's side or inside the side band
         * count against the wall. */
        const auto crossesFrom = [&crossesTo, &wallGetCentroid, &wallGeometry](
                                     const Eigen::Vector3d &segmentStart_in)
        {
            if (crossesTo(segmentStart_in, wallGetCentroid))
            {
                return true;
            }
            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr
                p_wallSupportCloud = wallGeometry.supportCloud;
            if (p_wallSupportCloud == nullptr || p_wallSupportCloud->empty())
            {
                return false;
            }
            /* ponytail: fixed stride to about 200 tested points per wall
             * and passage; replace with a per-cycle cached decision if the
             * every-cycle sweep ever shows in the semantic cycle time. */
            const std::size_t supportPointStride =
                std::max<std::size_t>(1U, p_wallSupportCloud->size() / 200U);
            std::size_t testedPointCount   = 0U;
            std::size_t crossingPointCount = 0U;
            for (std::size_t pointIndex = 0U;
                 pointIndex < p_wallSupportCloud->size();
                 pointIndex += supportPointStride)
            {
                const pcl::PointXYZRGBA &supportPoint =
                    p_wallSupportCloud->points[pointIndex];
                if (!pcl::isFinite(supportPoint))
                {
                    continue;
                }
                ++testedPointCount;
                if (crossesTo(
                        segmentStart_in,
                        Eigen::Vector3d(static_cast<double>(supportPoint.x),
                                        static_cast<double>(supportPoint.y),
                                        static_cast<double>(supportPoint.z))))
                {
                    ++crossingPointCount;
                }
            }
            return 2U * crossingPointCount > testedPointCount;
        };

        /* For the room the camera is in, the camera is the viewpoint: a wall
         * it sees through the opening from the room's side lies beyond the
         * passage, wherever the room centroid's own line to the wall meets
         * the door wall. The room's side is the start's side, which must be
         * callable. A camera inside the side band stands in the doorway and
         * has not traversed it; it is moved along the passage normal onto
         * the room's side, twice the band off the plane like the start
         * substitute above. A camera decisively on the far side has crossed
         * ahead of the current-room estimate and routes nothing, as the
         * start's line no longer agrees with it. */
        Eigen::Vector3d viewpoint_world_m     = segmentStart_world_m;
        bool            isViewpointOnRoomSide = true;
        if (hasCameraViewpoint)
        {
            const double startSide_m =
                passageNormal_world.dot(segmentStart_world_m) +
                passageEquation_world(3);
            const double cameraSide_m =
                passageNormal_world.dot(cameraCenter_world_m) +
                passageEquation_world(3);
            const bool isCameraInSideBand =
                std::abs(cameraSide_m) < minimumSideDistance_m;
            isViewpointOnRoomSide =
                hasPassagePlane &&
                std::abs(startSide_m) >= minimumSideDistance_m &&
                (isCameraInSideBand || cameraSide_m * startSide_m > 0.0);
            viewpoint_world_m = cameraCenter_world_m;
            if (isViewpointOnRoomSide && isCameraInSideBand)
            {
                viewpoint_world_m +=
                    (std::copysign(2.0 * minimumSideDistance_m, startSide_m) -
                     cameraSide_m) *
                    passageNormal_world;
            }
        }
        const bool crossesPassageOpening =
            isViewpointOnRoomSide && crossesFrom(viewpoint_world_m);
        if (!crossesPassageOpening)
        {
            continue;
        }

        /* Never steal a wall already claimed by a distinct confirmed room. */
        bool                          ownedByConfirmedRoom = false;
        std::vector<semantic::Room *> atlasAllRooms{};
        if (getAllRoomsInIdOrder(atlasAllRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRoomsInIdOrder returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        for (vs_graphs::core::semantic::Room *p_other : atlasAllRooms)
        {
            bool otherIsBad{};
            if (!(p_other == nullptr) &&
                p_other->isBad(otherIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant otherRoomVariant{};
            if (!(p_other == nullptr || otherIsBad ||
                  p_other == p_room_inout) &&
                p_other->getRoomVariant(otherRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_other == nullptr || otherIsBad || p_other == p_room_inout ||
                otherRoomVariant ==
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
            {
                continue;
            }
            std::vector<geometric::Plane *> otherWalls{};
            if (p_other->getWalls(otherWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (std::find(otherWalls.begin(), otherWalls.end(), p_wall_in) !=
                otherWalls.end())
            {
                ownedByConfirmedRoom = true;
                break;
            }
        }
        if (ownedByConfirmedRoom)
        {
            /* A distinct confirmed room already owns this wall. Leave it on
             * that owner rather than re-binding it to the near room. */
            bool room_inoutWasWallRemoved{};
            if (p_room_inout->removeWall(p_wall_in, room_inoutWasWallRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                room_inoutWasWallRemoved = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: removeWall rejected its input; continuing as before.",
                    __func__);
            }
            outcome_out = PassageSideEnforcementOutcome::REMOVED_UNBOUND;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        /* The opening bounds only the rooms it joins. A room that does not
         * hold the passage sees it, if at all, through its own walls and
         * cannot say which side a wall lies on: its line to the corridor's
         * end wall threads a door on the corridor's far face. */
        std::vector<semantic::Passage *> roomPassages{};
        if (p_room_inout->getPassages(roomPassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const bool isKnownSideRoom =
            std::find(roomPassages.begin(), roomPassages.end(), p_passage) !=
            roomPassages.end();

        /* Seen from the far-side room, a wall reached through the opening
         * lies on the known side and cannot bound this room; seen from a
         * room the opening does not join, it is beyond that room's own
         * walls. A wall the room already owns is kept: the far-side room's
         * viewpoint may be the camera while the current-room estimate still
         * lags a crossing back through the opening, and evicting on that
         * alone would strip the room's own walls. A wall not yet owned is
         * refused and left to ordinary admission, which applies the full
         * admission checks to the known-side rooms. */
        if (isFarSideRoom || !isKnownSideRoom)
        {
            std::vector<geometric::Plane *> testedRoomWalls{};
            if (p_room_inout->getWalls(testedRoomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (std::find(testedRoomWalls.begin(),
                          testedRoomWalls.end(),
                          p_wall_in) != testedRoomWalls.end())
            {
                continue;
            }
            outcome_out = PassageSideEnforcementOutcome::REMOVED_UNBOUND;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        bool prospectiveIsBad{};
        if (!(p_prospective == nullptr) &&
            p_prospective->isBad(prospectiveIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_prospective == nullptr || prospectiveIsBad)
        {
            bool room_inoutWasWallRemoved2{};
            if (p_room_inout->removeWall(p_wall_in,
                                         room_inoutWasWallRemoved2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                room_inoutWasWallRemoved2 = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: removeWall rejected its input; continuing as before.",
                    __func__);
            }
            int passageId{};
            if (p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int wallGetId{};
            if (p_wall_in->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Far-side Wall#" << wallGetId
                      << " at Passage#" << passageId
                      << " has no opposite stable room; left unbound."
                      << std::endl;
            outcome_out = PassageSideEnforcementOutcome::REMOVED_UNBOUND;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        bool room_inoutWasWallRemoved3{};
        if (p_room_inout->removeWall(p_wall_in, room_inoutWasWallRemoved3) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            room_inoutWasWallRemoved3 = false;
            RCLCPP_WARN(
                rclcpp::get_logger("vs_graphs"),
                "%s: removeWall rejected its input; continuing as before.",
                __func__);
        }
        int wallGetId2{};
        if (p_wall_in->getId(wallGetId2) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::geometric::Plane *p_atlasRoomWallPlaneById = nullptr;
        if (p_atlas->getRoomWallPlaneById(wallGetId2,
                                          p_atlasRoomWallPlaneById) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomWallPlaneById returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlasRoomWallPlaneById == nullptr)
        {
            if (p_atlas->addRoomWallPlane(p_wall_in) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addRoomWallPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        if (p_prospective->setWalls(p_wall_in) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int prospectiveId{};
        if (p_prospective->getId(prospectiveId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int wallGetId3{};
        if (p_wall_in->getId(wallGetId3) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Redirected far-side Wall#" << wallGetId3
                  << " to prospective Room#" << prospectiveId << "."
                  << std::endl;
        outcome_out = PassageSideEnforcementOutcome::REROUTED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    outcome_out = PassageSideEnforcementOutcome::NO_VIOLATION;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
