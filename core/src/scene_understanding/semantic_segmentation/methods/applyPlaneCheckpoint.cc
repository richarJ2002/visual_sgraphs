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
    keyframe::KeyFrame      *p_keyFrame_in,
    std::vector<PlanePatch> &planePatches_inout)
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

    std::vector<PlanePatch> keptPatches{};
    for (PlanePatch &candidatePatch : planePatches_inout)
    {
        const std::size_t patchPointCount =
            candidatePatch.p_supportCloud_camera_m == nullptr
                ? 0U
                : candidatePatch.p_supportCloud_camera_m->size();
        const bool hasEnoughPoints =
            patchPointCount >= checkpointParams.minimumPointCount;

        double fitRms_m = 0.0;
        if (hasEnoughPoints &&
            computePlaneFitRms(candidatePatch.p_supportCloud_camera_m,
                               candidatePatch.planeEquation_camera,
                               fitRms_m) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            /* computePlaneFitRms cannot fail; continue as before. */
        }
        const bool hasGoodFit =
            hasEnoughPoints && fitRms_m <= checkpointParams.maximumFitRms_m;
        if (!hasGoodFit)
        {
            continue;
        }

        /*!
         * Geometry decides the label; the incoming one is only the guess.
         * The label rules use the same tilts as the reject rules, so a
         * labelled patch also passes the tilt rules for its label. The
         * normal faces the camera (d >= 0, camera at the origin), which the
         * level rule needs to tell ground from ceiling.
         */
        Eigen::Vector3d planeNormal_camera =
            candidatePatch.planeEquation_camera.head<3>().normalized();
        if (candidatePatch.planeEquation_camera(3) < 0.0)
        {
            planeNormal_camera = -planeNormal_camera;
        }
        semantic_constraints::PlaneRuleInput patchRuleInput{};
        patchRuleInput.planeLabel         = candidatePatch.planeLabel;
        patchRuleInput.hasOrientation     = true;
        patchRuleInput.planeNormal_common = planeNormal_camera;
        patchRuleInput.upDirection_common = upDirection_camera;
        const geometric::Plane::PlaneVariant geometryLabel =
            semantic_constraints::assignPlaneLabel(patchRuleInput,
                                                   *p_sysParams);
        if (geometryLabel == geometric::Plane::PlaneVariant::UNDEFINED)
        {
            continue;
        }
        candidatePatch.planeLabel = geometryLabel;

        bool isLargeEnough = true;
        if (geometryLabel == geometric::Plane::PlaneVariant::WALL &&
            checkWallPatchSize(checkpointParams.wallSize,
                               candidatePatch.p_supportCloud_camera_m,
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
    planePatches_inout = std::move(keptPatches);

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
