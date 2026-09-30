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

/*!
 * @file            updateNormalAndDepth.cc
 *
 * @brief           Implements MapPoint::updateNormalAndDepth(), declared in
 *                  MapPoint.h.
 */

#include "MapPoint.h"

#include "ORBmatcher.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapPointStatus MapPoint::updateNormalAndDepth()
{
    std::map<KeyFrame *, std::tuple<int, int>> observedKeyFrames;
    KeyFrame                                  *p_localReferenceKeyFrame;
    Eigen::Vector3f                            Pos;
    {
        std::unique_lock<std::mutex> lock1(featuresMutex);
        std::unique_lock<std::mutex> lock2(positionMutex);
        if (isFlaggedBad)
            return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
        observedKeyFrames        = observations;
        p_localReferenceKeyFrame = p_referenceKeyFrame;
        Pos                      = worldPos;
    }

    if (observedKeyFrames.empty())
        return MapPointStatus::MAP_POINT_STATUS_SUCCESS;

    Eigen::Vector3f normal;
    normal.setZero();
    int n = 0;
    for (std::map<KeyFrame *, std::tuple<int, int>>::iterator
             mit  = observedKeyFrames.begin(),
             mend = observedKeyFrames.end();
         mit != mend;
         mit++)
    {
        KeyFrame *p_keyFrame = mit->first;

        std::tuple<int, int> indexes = mit->second;
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

        if (leftIndex != -1)
        {
            Eigen::Vector3f Owi{};
            if (p_keyFrame->getCameraCenter(Owi) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
        if (rightIndex != -1)
        {
            Eigen::Vector3f Owi{};
            if (p_keyFrame->getRightCameraCenter(Owi) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getRightCameraCenter returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f localReferenceKeyFrameCameraCenter{};
    if (p_localReferenceKeyFrame->getCameraCenter(
            localReferenceKeyFrameCameraCenter) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCameraCenter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3f PC       = Pos - localReferenceKeyFrameCameraCenter;
    const float     distance = PC.norm();

    std::tuple<int, int> indexes = observedKeyFrames[p_localReferenceKeyFrame];
    int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
    int level;
    if (p_localReferenceKeyFrame->leftKeyPointCount == -1)
    {
        level =
            p_localReferenceKeyFrame->keyPointsUndistorted[leftIndex].octave;
    }
    else if (leftIndex != -1)
    {
        level = p_localReferenceKeyFrame->keyPoints[leftIndex].octave;
    }
    else
    {
        level =
            p_localReferenceKeyFrame
                ->keyPointsRight[rightIndex -
                                 p_localReferenceKeyFrame->leftKeyPointCount]
                .octave;
    }

    // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =
        p_localReferenceKeyFrame->scaleFactors[level];
    const int levelCount = p_localReferenceKeyFrame->scaleLevelCount;

    {
        std::unique_lock<std::mutex> lock3(positionMutex);
        maxDistance = distance * levelScaleFactor;
        minDistance = maxDistance /
                      p_localReferenceKeyFrame->scaleFactors[levelCount - 1];
        normalVector = normal / n;
    }

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
