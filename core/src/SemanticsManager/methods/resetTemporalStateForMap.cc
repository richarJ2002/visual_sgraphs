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

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::resetTemporalStateForMap(Map *p_activeMap_in)
{
    std::uint64_t activeMapWorldFrameEpoch{};
    if ((p_activeMap_in != nullptr) &&
        p_activeMap_in->getWorldFrameEpoch(activeMapWorldFrameEpoch) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWorldFrameEpoch returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::uint64_t worldFrameEpoch =
        p_activeMap_in != nullptr ? activeMapWorldFrameEpoch : 0U;
    const bool mapChanged   = p_temporalStateMap != p_activeMap_in;
    const bool frameChanged = !mapChanged && p_activeMap_in != nullptr &&
                              temporalStateWorldFrameEpoch != worldFrameEpoch;

    if (!mapChanged && !frameChanged)
    {
        return;
    }

    openPassageEvidence.clear();
    lastSkeletonFingerprint = 0U;
    hasSkeletonFingerprint  = false;

    if (mapChanged)
    {
        {
            std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
            currentRoomId = -1;
        }
        currentCameraCenter_World_m  = Eigen::Vector3d::Zero();
        previousCameraCenter_World_m = Eigen::Vector3d::Zero();
        hasCameraCenter              = false;
        p_cameraCenterMap            = p_activeMap_in;
        lastTraversalFrameId         = 0U;
        lastTraversalKeyFrameId      = 0U;
        hasTraversalKeyFrameCursor   = false;

        disconnectedRoomIds.clear();
        prospectiveRoomCycles.clear();
        undefendedWalls.clear();
        loggedOrphanWallIds.clear();
        loggedWallRejectionReasons.clear();
        loggedRetiredWallIds.clear();
        loggedRoomCleanupIds.clear();
    }
    else if (frameChanged && hasTraversalKeyFrameCursor)
    {
        /* Re-read the cursor keyframe in the rebased frame. Keeping its IDs
         * preserves every later, not-yet-processed trajectory segment. */
        hasCameraCenter = false;
        std::vector<KeyFrame *> activeMapAllKeyFrames{};
        if (p_activeMap_in->getAllKeyFrames(activeMapAllKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (KeyFrame *p_keyFrame : activeMapAllKeyFrames)
        {
            bool keyFrameIsBad{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame == nullptr || keyFrameIsBad ||
                p_keyFrame->frameId != lastTraversalFrameId ||
                p_keyFrame->id != lastTraversalKeyFrameId)
            {
                continue;
            }

            Eigen::Vector3f keyFrameCameraCenter{};
            if (p_keyFrame->getCameraCenter(keyFrameCameraCenter) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector3d correctedCenter_World_m =
                keyFrameCameraCenter.cast<double>();
            if (correctedCenter_World_m.allFinite())
            {
                currentCameraCenter_World_m  = correctedCenter_World_m;
                previousCameraCenter_World_m = correctedCenter_World_m;
                hasCameraCenter              = true;
                p_cameraCenterMap            = p_activeMap_in;
            }
            break;
        }
    }

    p_temporalStateMap           = p_activeMap_in;
    temporalStateWorldFrameEpoch = worldFrameEpoch;
}

} // namespace core
} // namespace vs_graphs
