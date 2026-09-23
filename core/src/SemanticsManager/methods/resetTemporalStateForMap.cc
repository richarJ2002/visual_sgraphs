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

namespace vs_graphs
{
namespace core
{

void SemanticsManager::resetTemporalStateForMap(Map *p_activeMap_in)
{
    const std::uint64_t worldFrameEpoch =
        p_activeMap_in != nullptr ? p_activeMap_in->getWorldFrameEpoch() : 0U;
    const bool mapChanged   = pTemporalStateMap_ != p_activeMap_in;
    const bool frameChanged = !mapChanged && p_activeMap_in != nullptr &&
                              temporalStateWorldFrameEpoch_ != worldFrameEpoch;

    if (!mapChanged && !frameChanged)
    {
        return;
    }

    openPassageEvidence_.clear();
    lastSkeletonFingerprint_ = 0U;
    hasSkeletonFingerprint_  = false;

    if (mapChanged)
    {
        {
            std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
            currentRoomId_ = -1;
        }
        currentCameraCenter_World_m  = Eigen::Vector3d::Zero();
        previousCameraCenter_World_m = Eigen::Vector3d::Zero();
        hasCameraCenter_             = false;
        pCameraCenterMap_            = p_activeMap_in;
        lastTraversalFrameId_        = 0U;
        lastTraversalKeyFrameId_     = 0U;
        hasTraversalKeyFrameCursor_  = false;

        disconnectedRoomIds_.clear();
        prospectiveRoomCycles_.clear();
        undefendedWalls_.clear();
        loggedOrphanWallIds_.clear();
        loggedWallRejectionReasons_.clear();
        loggedRetiredWallIds_.clear();
        loggedRoomCleanupIds_.clear();
    }
    else if (frameChanged && hasTraversalKeyFrameCursor_)
    {
        /* Re-read the cursor keyframe in the rebased frame. Keeping its IDs
         * preserves every later, not-yet-processed trajectory segment. */
        hasCameraCenter_ = false;
        for (KeyFrame *p_keyFrame : p_activeMap_in->getAllKeyFrames())
        {
            if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
                p_keyFrame->frameId != lastTraversalFrameId_ ||
                p_keyFrame->mnId != lastTraversalKeyFrameId_)
            {
                continue;
            }

            const Eigen::Vector3d correctedCenter_World_m =
                p_keyFrame->getCameraCenter().cast<double>();
            if (correctedCenter_World_m.allFinite())
            {
                currentCameraCenter_World_m  = correctedCenter_World_m;
                previousCameraCenter_World_m = correctedCenter_World_m;
                hasCameraCenter_             = true;
                pCameraCenterMap_            = p_activeMap_in;
            }
            break;
        }
    }

    pTemporalStateMap_            = p_activeMap_in;
    temporalStateWorldFrameEpoch_ = worldFrameEpoch;
}

} // namespace core
} // namespace vs_graphs
