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
 * @file            applyPlaneCheckpoint.cc
 *
 * @brief           Implements SemanticSegmentation::applyPlaneCheckpoint(),
 *                  declared in
 *                  semantic_segmentation/objects/SemanticSegmentationClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantic_constraints/plane_rules/plane_rules.h"
#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationClass.h"
#include "scene_understanding/semantic_segmentation/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

SemanticSegmentationStatus SemanticSegmentation::applyPlaneCheckpoint(
    keyframe::KeyFrame                                   *p_keyFrame_in,
    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                      Eigen::Vector4d>>> &p_clsPlanes_inout)
{
    PROFILE_FUNCTION();
    const types::SystemParams::PlaneCheckpoint &checkpointParams =
        p_sysParams->planeCheckpoint;

    /*!
     * The patches are in the camera frame, so turn the map up direction into
     * the camera frame once instead of every patch into the world frame.
     */
    Eigen::Vector3d upDirection_world{};
    bool            isFromImuGravity{};
    if (p_atlas->getUpDirection(upDirection_world, isFromImuGravity) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getUpDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f cameraPose_worldToCamera{};
    if (p_keyFrame_in->getPose(cameraPose_worldToCamera) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3d upDirection_camera =
        cameraPose_worldToCamera.rotationMatrix().cast<double>() *
        upDirection_world;

    for (std::size_t classId = 0U; classId < p_clsPlanes_inout.size();
         classId++)
    {
        geometric::Plane::PlaneVariant planeLabel{};
        if (utils::utils::Utils::getPlaneTypeFromClassId(
                static_cast<int>(classId),
                planeLabel) != utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getPlaneTypeFromClassId returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        std::vector<
            std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
            keptPatches{};
        for (std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>
                 &candidatePatch : p_clsPlanes_inout[classId])
        {
            const std::size_t patchPointCount =
                candidatePatch.first == nullptr ? 0U
                                                : candidatePatch.first->size();
            const bool hasEnoughPoints =
                patchPointCount >= checkpointParams.minimumPointCount;

            double fitRms_m = 0.0;
            if (hasEnoughPoints && computePlaneFitRms(candidatePatch.first,
                                                      candidatePatch.second,
                                                      fitRms_m) !=
                                       SemanticSegmentationStatus::
                                           SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                /* computePlaneFitRms cannot fail; continue as before. */
            }
            const bool hasGoodFit =
                hasEnoughPoints && fitRms_m <= checkpointParams.maximumFitRms_m;

            /*!
             * The tilt rules of the plane rule table, the same the map-level
             * re-check applies, judge the patch in the camera frame. No
             * height is given, so the ground step rule does not apply here.
             */
            bool isOrientedAsLabelled = false;
            if (hasGoodFit)
            {
                semantic_constraints::PlaneRuleInput patchRuleInput{};
                patchRuleInput.planeLabel     = planeLabel;
                patchRuleInput.hasOrientation = true;
                patchRuleInput.planeNormal_common =
                    candidatePatch.second.head<3>().normalized();
                patchRuleInput.upDirection_common = upDirection_camera;
                isOrientedAsLabelled =
                    semantic_constraints::arePlaneRulesSatisfied(patchRuleInput,
                                                                 *p_sysParams);
            }

            bool isLargeEnough = isOrientedAsLabelled;
            if (isOrientedAsLabelled &&
                planeLabel == geometric::Plane::PlaneVariant::WALL &&
                checkWallPatchSize(checkpointParams.wallSize,
                                   candidatePatch.first,
                                   isLargeEnough) !=
                    SemanticSegmentationStatus::
                        SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                /* checkWallPatchSize cannot fail; continue as before. */
            }

            if (isLargeEnough)
            {
                keptPatches.push_back(candidatePatch);
            }
        }
        p_clsPlanes_inout[classId] = std::move(keptPatches);
    }

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
