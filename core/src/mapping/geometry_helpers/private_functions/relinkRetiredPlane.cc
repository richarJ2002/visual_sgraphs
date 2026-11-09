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
 * @file            relinkRetiredPlane.cc
 *
 * @brief           Implements relinkRetiredPlane(), declared in
 *                  mapping/geometry_helpers/private_functions/private_functions.h.
 */

#include "mapping/geometry_helpers/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>
#include <vector>

#include "map_state/atlas/atlas.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus relinkRetiredPlane(atlas::Atlas     *p_atlas_in,
                               geometric::Plane *p_retiredPlane_inout,
                               geometric::Plane *p_retainedPlane_inout)
{
    SemanticsManager *p_semanticsManager = nullptr;
    if (p_atlas_in->getSemanticsManager(p_semanticsManager) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getSemanticsManager returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }

    /* The plane's own observation records move to the survivor in one locked
     * step. */
    if (p_retainedPlane_inout->absorb(p_retiredPlane_inout) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: absorb returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* The rooms and passages that listed the retired plane list the survivor
     * instead; the scene graph writes both sides in one step. */
    if (p_semanticsManager->replacePlane(p_retiredPlane_inout,
                                         p_retainedPlane_inout) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: replacePlane returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* The keyframes that saw the retired plane now see the survivor. */
    std::vector<keyframe::KeyFrame *> atlasAllKeyFrames{};
    if (p_atlas_in->getAllKeyFrames(atlasAllKeyFrames) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (keyframe::KeyFrame *p_keyFrame : atlasAllKeyFrames)
    {
        bool keyFrameIsBad{true};
        if (p_keyFrame != nullptr &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool keyFrameWasReplaced{};
        if (p_keyFrame != nullptr && !keyFrameIsBad &&
            p_keyFrame->replaceMapPlane(p_retiredPlane_inout,
                                        p_retainedPlane_inout,
                                        keyFrameWasReplaced) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: replaceMapPlane rejected its input; continuing "
                        "as before.",
                        __func__);
        }
    }
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
