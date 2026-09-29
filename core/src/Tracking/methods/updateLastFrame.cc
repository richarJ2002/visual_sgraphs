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

void Tracking::updateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame    *p_reference = lastFrame.p_referenceKeyFrame;
    Sophus::SE3f Tlr =
        relativeFramePoses.empty() ? Sophus::SE3f() : relativeFramePoses.back();
    Sophus::SE3f referencePose{};
    if (p_reference->getPose(referencePose) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (lastFrame.setPose(Tlr * referencePose) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    if (lastKeyFrameId == lastFrame.id || sensor == System::MONOCULAR ||
        sensor == System::IMU_MONOCULAR || !isTrackingOnlyMode)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D
    // sensor
    vector<pair<float, int>> depthIndices;
    const int                featureCount = lastFrame.leftKeyPointCount == -1
                                                ? lastFrame.keyPointCount
                                                : lastFrame.leftKeyPointCount;
    depthIndices.reserve(featureCount);
    for (int featureIndex = 0; featureIndex < featureCount; featureIndex++)
    {
        float z = lastFrame.depths[featureIndex];
        if (z > 0)
        {
            depthIndices.push_back(make_pair(z, featureIndex));
        }
    }

    if (depthIndices.empty())
        return;

    sort(depthIndices.begin(), depthIndices.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int pointCount = 0;
    for (size_t depthIndexIndex = 0; depthIndexIndex < depthIndices.size();
         depthIndexIndex++)
    {
        int featureIndex = depthIndices[depthIndexIndex].second;

        bool shouldCreateNewPoint = false;

        MapPoint *p_mapPoint = lastFrame.mapPoints[featureIndex];

        if (!p_mapPoint)
        {
            shouldCreateNewPoint = true;
        }
        else
        {
            int mapPointObservationCount{};
            if (p_mapPoint->getObservationCount(mapPointObservationCount) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getObservationCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (mapPointObservationCount < 1)
            {
                shouldCreateNewPoint = true;
            }
        }

        if (shouldCreateNewPoint)
        {
            Eigen::Vector3f x3D;

            if (lastFrame.leftKeyPointCount == -1)
            {
                bool lastFrameIsUnprojected{};
                if (lastFrame.unprojectStereo(featureIndex,
                                              x3D,
                                              lastFrameIsUnprojected) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: unprojectStereo returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                Eigen::Vector3f lastFrameStereoFishEye{};
                if (lastFrame.unprojectStereoFishEye(featureIndex,
                                                     lastFrameStereoFishEye) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: unprojectStereoFishEye returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                x3D = lastFrameStereoFishEye;
            }

            MapPoint *p_newMapPoint           = new MapPoint(x3D,
                                                   p_atlas->getCurrentMap(),
                                                   &lastFrame,
                                                   featureIndex);
            lastFrame.mapPoints[featureIndex] = p_newMapPoint;

            temporalMapPoints.push_back(p_newMapPoint);
            pointCount++;
        }
        else
        {
            pointCount++;
        }

        if (depthIndices[depthIndexIndex].first > depthThreshold &&
            pointCount > 100)
            break;
    }
}

} // namespace core
} // namespace vs_graphs
