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

#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

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
         * the start is left unchanged. */
        Eigen::Vector3d segmentStart_world_m{};
        bool            isCameraViewpoint{};
        if (getRoomViewpoint(p_room_inout,
                             segmentStart_world_m,
                             isCameraViewpoint) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomViewpoint returned a failure status "
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
        if (passageEquation_world.allFinite() && passageNormalNorm > 1e-8)
        {
            passageEquation_world /= passageNormalNorm;
            const Eigen::Vector3d passageNormal_world =
                passageEquation_world.head<3>();
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
        bool crossesPassageOpening{};
        if (segmentCrossesPassageOpening(
                segmentStart_world_m,
                wallGetCentroid.cast<double>(),
                p_passage,
                groundNormal_world_in,
                static_cast<double>(
                    p_sysParams->roomSeg.passagePartition.openingMargin_m),
                minimumSideDistance_m,
                crossesPassageOpening) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: segmentCrossesPassageOpening returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!crossesPassageOpening)
        {
            continue;
        }

        /* Never steal a wall already claimed by a distinct confirmed room. */
        bool                          ownedByConfirmedRoom = false;
        std::vector<semantic::Room *> atlasAllRooms{};
        if (p_atlas->getAllRooms(atlasAllRooms) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
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

        /* Seen from the far-side room, a wall reached through the opening
         * lies on the known side and cannot bound this room. A wall the room
         * already owns is kept: the far-side room's viewpoint may be the
         * camera while the current-room estimate still lags a crossing back
         * through the opening, and evicting on that alone would strip the
         * room's own walls. A wall not yet owned is refused and left to
         * ordinary admission, which applies the full admission checks to the
         * known-side rooms. */
        if (isFarSideRoom)
        {
            std::vector<geometric::Plane *> farSideRoomWalls{};
            if (p_room_inout->getWalls(farSideRoomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (std::find(farSideRoomWalls.begin(),
                          farSideRoomWalls.end(),
                          p_wall_in) != farSideRoomWalls.end())
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
