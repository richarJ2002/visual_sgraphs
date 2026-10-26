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
 * @file            getCurrentCameraCenter.cc
 *
 * @brief           Implements SemanticsManager::getCurrentCameraCenter(),
 *                  declared in SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getCurrentCameraCenter(
    std::optional<Eigen::Vector3d> &cameraCenter_world_m_out)
{
    cameraCenter_world_m_out.reset();

    map::Map *p_activeMap = nullptr;
    if ((p_atlas != nullptr) && p_atlas->getCurrentMap(p_activeMap) !=
                                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_activeMap == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<keyframe::KeyFrame *> keyFrames{};
    if (p_activeMap->getAllKeyFrames(keyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(keyFrames.begin(), keyFrames.end(), keyframe::KeyFrame::lId);

    /* Newest key frame first; the first usable centre wins. */
    for (std::vector<keyframe::KeyFrame *>::reverse_iterator keyFrameIterator =
             keyFrames.rbegin();
         keyFrameIterator != keyFrames.rend();
         keyFrameIterator++)
    {
        keyframe::KeyFrame *p_keyFrame = *keyFrameIterator;
        bool                keyFrameIsBad{};
        if ((p_keyFrame != nullptr) &&
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
        const Eigen::Vector3d keyFrameCameraCenter_world_m =
            keyFrameCameraCenter.cast<double>();
        /* An exactly-zero centre marks an uninitialised first-frame pose
         * (brand-new map, identity pose), never a genuine measurement: real
         * computed centres carry rotation/translation noise. */
        if (keyFrameCameraCenter_world_m.allFinite() &&
            !keyFrameCameraCenter_world_m.isZero())
        {
            cameraCenter_world_m_out = keyFrameCameraCenter_world_m;
            break;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
