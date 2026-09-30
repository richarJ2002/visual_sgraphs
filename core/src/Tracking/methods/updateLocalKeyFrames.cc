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

#include "System.h"
#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::updateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    std::map<KeyFrame *, int> keyframeCounter;
    bool                      atlasIsImuInitialized{};
    if (p_atlas->isImuInitialized(atlasIsImuInitialized) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!atlasIsImuInitialized || (currentFrame.id < lastRelocFrameId + 2))
    {
        for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
             keyPointIndex++)
        {
            MapPoint *p_mapPoint = currentFrame.mapPoints[keyPointIndex];
            if (p_mapPoint)
            {
                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!mapPointIsBad)
                {
                    std::map<KeyFrame *, std::tuple<int, int>> observations{};
                    if (p_mapPoint->getObservations(observations) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservations returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    for (std::map<KeyFrame *,
                                  std::tuple<int, int>>::const_iterator
                             keyFrameCounterIt = observations.begin(),
                             itend             = observations.end();
                         keyFrameCounterIt != itend;
                         keyFrameCounterIt++)
                        keyframeCounter[keyFrameCounterIt->first]++;
                }
                else
                {
                    currentFrame.mapPoints[keyPointIndex] = nullptr;
                }
            }
        }
    }
    else
    {
        for (int keyPointIndex = 0; keyPointIndex < lastFrame.keyPointCount;
             keyPointIndex++)
        {
            // Using lastframe since current frame has not matches yet
            if (lastFrame.mapPoints[keyPointIndex])
            {
                MapPoint *p_mapPoint = lastFrame.mapPoints[keyPointIndex];
                if (!p_mapPoint)
                    continue;
                bool mapPointIsBad2{};
                if (p_mapPoint->isBad(mapPointIsBad2) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!mapPointIsBad2)
                {
                    std::map<KeyFrame *, std::tuple<int, int>> observations{};
                    if (p_mapPoint->getObservations(observations) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservations returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    for (std::map<KeyFrame *,
                                  std::tuple<int, int>>::const_iterator
                             keyFrameCounterIt = observations.begin(),
                             itend             = observations.end();
                         keyFrameCounterIt != itend;
                         keyFrameCounterIt++)
                        keyframeCounter[keyFrameCounterIt->first]++;
                }
                else
                {
                    // MODIFICATION
                    lastFrame.mapPoints[keyPointIndex] = nullptr;
                }
            }
        }
    }

    int       maximum           = 0;
    KeyFrame *p_keyFrameMaximum = static_cast<KeyFrame *>(nullptr);

    localKeyFrames.clear();
    localKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map.
    // Also check which keyframe shares most points
    for (std::map<KeyFrame *, int>::const_iterator
             keyFrameCounterIt = keyframeCounter.begin(),
             itEnd             = keyframeCounter.end();
         keyFrameCounterIt != itEnd;
         keyFrameCounterIt++)
    {
        KeyFrame *p_keyFrame = keyFrameCounterIt->first;

        bool keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (keyFrameIsBad)
            continue;

        if (keyFrameCounterIt->second > maximum)
        {
            maximum           = keyFrameCounterIt->second;
            p_keyFrameMaximum = p_keyFrame;
        }

        localKeyFrames.push_back(p_keyFrame);
        p_keyFrame->trackReferenceFrameId = currentFrame.id;
    }

    // Include also some not-already-included keyframes that are neighbors to
    // already-included keyframes
    for (std::vector<KeyFrame *>::const_iterator
             itKeyFrame    = localKeyFrames.begin(),
             itEndKeyFrame = localKeyFrames.end();
         itKeyFrame != itEndKeyFrame;
         itKeyFrame++)
    {
        // Limit the number of keyframes - use configurable max (200 for
        // corridors)
        if (localKeyFrames.size() > static_cast<size_t>(maxKFsInLocalMap))
        {
            break;
        }

        KeyFrame *p_keyFrame = *itKeyFrame;

        std::vector<KeyFrame *> neighbors{};
        if (p_keyFrame->getBestCovisibilityKeyFrames(10, neighbors) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getBestCovisibilityKeyFrames returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        for (std::vector<KeyFrame *>::const_iterator
                 itNeighborKeyFrame    = neighbors.begin(),
                 itEndNeighborKeyFrame = neighbors.end();
             itNeighborKeyFrame != itEndNeighborKeyFrame;
             itNeighborKeyFrame++)
        {
            KeyFrame *p_neighborKeyFrame = *itNeighborKeyFrame;
            bool      neighborKeyFrameIsBad{};
            if (p_neighborKeyFrame->isBad(neighborKeyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!neighborKeyFrameIsBad)
            {
                if (p_neighborKeyFrame->trackReferenceFrameId !=
                    currentFrame.id)
                {
                    localKeyFrames.push_back(p_neighborKeyFrame);
                    p_neighborKeyFrame->trackReferenceFrameId = currentFrame.id;
                    break;
                }
            }
        }

        std::set<KeyFrame *> childs{};
        if (p_keyFrame->getChilds(childs) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getChilds returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (std::set<KeyFrame *>::const_iterator sit  = childs.begin(),
                                                  send = childs.end();
             sit != send;
             sit++)
        {
            KeyFrame *p_childKeyFrame = *sit;
            bool      childKeyFrameIsBad{};
            if (p_childKeyFrame->isBad(childKeyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!childKeyFrameIsBad)
            {
                if (p_childKeyFrame->trackReferenceFrameId != currentFrame.id)
                {
                    localKeyFrames.push_back(p_childKeyFrame);
                    p_childKeyFrame->trackReferenceFrameId = currentFrame.id;
                    break;
                }
            }
        }

        KeyFrame *p_parent = nullptr;
        if (p_keyFrame->getParent(p_parent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_parent)
        {
            if (p_parent->trackReferenceFrameId != currentFrame.id)
            {
                localKeyFrames.push_back(p_parent);
                p_parent->trackReferenceFrameId = currentFrame.id;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        localKeyFrames.size() < 80)
    {
        KeyFrame *p_tempKeyFrame = currentFrame.p_lastKeyFrame;

        const int Nd = 20;
        for (int keyPointIndex = 0; keyPointIndex < Nd; keyPointIndex++)
        {
            if (!p_tempKeyFrame)
                break;
            if (p_tempKeyFrame->trackReferenceFrameId != currentFrame.id)
            {
                localKeyFrames.push_back(p_tempKeyFrame);
                p_tempKeyFrame->trackReferenceFrameId = currentFrame.id;
                p_tempKeyFrame = p_tempKeyFrame->p_prevKF;
            }
        }
    }

    if (p_keyFrameMaximum)
    {
        p_referenceKF                    = p_keyFrameMaximum;
        currentFrame.p_referenceKeyFrame = p_referenceKF;
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
