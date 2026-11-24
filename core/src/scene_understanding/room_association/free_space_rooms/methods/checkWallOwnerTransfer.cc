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
 * @file            checkWallOwnerTransfer.cc
 *
 * @brief           Implements SemanticsManager::checkWallOwnerTransfer(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::checkWallOwnerTransfer(
    semantic::Room        *p_wallOwner_in,
    geometric::Plane      *p_wall_in,
    const Eigen::Vector3d &clusterCentroid_in,
    bool                  &isTransferableProvisional_out,
    semantic::Passage    *&p_transferPassage_out)
{
    isTransferableProvisional_out = false;
    p_transferPassage_out         = nullptr;

    g2o::Plane3D candidateWallGlobalEquation{};
    if (p_wall_in->getGlobalEquation(candidateWallGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d wallEquation_world = candidateWallGlobalEquation.coeffs();
    const double    wallNormalNorm     = wallEquation_world.head<3>().norm();

    if (wallEquation_world.allFinite() && wallNormalNorm > 1e-8)
    {
        wallEquation_world /= wallNormalNorm;

        constexpr double maximumProvisionalPlaneDistance_m = 0.20;
        Eigen::Vector3d  existingWallOwnerCentroid{};
        if (p_wallOwner_in->getCentroid(existingWallOwnerCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const double ownerPlaneDistance_m = std::abs(
            wallEquation_world.head<3>().dot(existingWallOwnerCentroid) +
            wallEquation_world(3));

        semantic::Room::RoomVariant existingWallOwnerRoomVariant{};
        if (p_wallOwner_in->getRoomVariant(existingWallOwnerRoomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> existingWallOwnerWalls{};
        if ((existingWallOwnerRoomVariant ==
             semantic::Room::RoomVariant::UNDEFINED) &&
            p_wallOwner_in->getWalls(existingWallOwnerWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        isTransferableProvisional_out =
            existingWallOwnerRoomVariant ==
                semantic::Room::RoomVariant::UNDEFINED &&
            existingWallOwnerWalls.size() == 1U &&
            ownerPlaneDistance_m <= maximumProvisionalPlaneDistance_m;
    }

    /*
     * A wall initially assigned before passage partitioning may
     * belong to the room on the far side. Transfer it only when
     * the confirmed opening separates both room centres and the
     * wall's observing cameras lie on the candidate-room side.
     */
    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d meanObservationPosition_world_m = Eigen::Vector3d::Zero();
    std::size_t     validObservationCount           = 0U;

    std::map<keyframe::KeyFrame *, geometric::Plane::Observation>
        wallGetObservations{};
    if (p_wall_in->getObservations(wallGetObservations) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservations returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (const auto &[p_keyFrame, observation] : wallGetObservations)
    {
        static_cast<void>(observation);

        bool keyFrameIsBad{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame == nullptr || keyFrameIsBad)
        {
            continue;
        }

        Eigen::Vector3f keyFrameCameraCenter{};
        if (p_keyFrame->getCameraCenter(keyFrameCameraCenter) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d cameraCentre_world_m =
            keyFrameCameraCenter.cast<double>();

        if (cameraCentre_world_m.allFinite())
        {
            meanObservationPosition_world_m += cameraCentre_world_m;
            validObservationCount++;
        }
    }

    bool ownerGroundPlaneIsBad{};
    if ((!isTransferableProvisional_out && validObservationCount > 0U &&
         p_groundPlane != nullptr) &&
        p_groundPlane->isBad(ownerGroundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isTransferableProvisional_out && validObservationCount > 0U &&
        p_groundPlane != nullptr && !ownerGroundPlaneIsBad)
    {
        meanObservationPosition_world_m /=
            static_cast<double>(validObservationCount);

        g2o::Plane3D ownerGroundPlaneGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(ownerGroundPlaneGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d groundEquation_world =
            ownerGroundPlaneGlobalEquation.coeffs();
        const double groundNormalNorm = groundEquation_world.head<3>().norm();

        if (groundEquation_world.allFinite() && groundNormalNorm > 1e-8)
        {
            const Eigen::Vector3d groundNormal_world =
                groundEquation_world.head<3>() / groundNormalNorm;

            std::vector<vs_graphs::core::semantic::Passage *>
                ownerBoundaryPassages{};
            if (getAllPassagesInIdOrder(ownerBoundaryPassages) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassagesInIdOrder returned a "
                             "failure status; continuing as before.",
                             __func__);
            }
            for (semantic::Passage *p_passage : ownerBoundaryPassages)
            {
                Eigen::Vector3d passageOwnerCentroid{};
                if (p_wallOwner_in->getCentroid(passageOwnerCentroid) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getCentroid returned a failure status "
                                 "although it cannot fail; continuing as "
                                 "before.",
                                 __func__);
                }
                bool crossesOwnerPassageOpening{};
                if (segmentCrossesPassageOpening(
                        passageOwnerCentroid,
                        clusterCentroid_in,
                        p_passage,
                        groundNormal_world,
                        p_sysParams->roomSeg.passagePartition.openingMargin_m,
                        p_sysParams->roomSeg.passagePartition
                            .minimumSideDistance_m,
                        crossesOwnerPassageOpening) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: segmentCrossesPassageOpening returned "
                                 "a failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (!crossesOwnerPassageOpening)
                {
                    continue;
                }

                g2o::Plane3D ownerPassageGlobalEquation{};
                if (p_passage->getGlobalEquation(ownerPassageGlobalEquation) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    /*!
                     * getGlobalEquation cannot fail; continue as
                     * before.
                     */
                }
                Eigen::Vector4d passageEquation_world =
                    ownerPassageGlobalEquation.coeffs();
                const double passageNormalNorm =
                    passageEquation_world.head<3>().norm();

                if (!passageEquation_world.allFinite() ||
                    passageNormalNorm < 1e-8)
                {
                    continue;
                }

                passageEquation_world /= passageNormalNorm;
                const double candidateSide_m =
                    passageEquation_world.head<3>().dot(clusterCentroid_in) +
                    passageEquation_world(3);
                const double observationSide_m =
                    passageEquation_world.head<3>().dot(
                        meanObservationPosition_world_m) +
                    passageEquation_world(3);

                constexpr double minimumEvidenceSideDistance_m = 0.10;

                if (candidateSide_m * observationSide_m > 0.0 &&
                    std::abs(candidateSide_m) >=
                        minimumEvidenceSideDistance_m &&
                    std::abs(observationSide_m) >=
                        minimumEvidenceSideDistance_m)
                {
                    p_transferPassage_out = p_passage;
                    break;
                }
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
