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

#include "LocalMapping.h"
#include "System.h"
#include "Tracking.h"

namespace vs_graphs
{
namespace core
{

void Tracking::createNewKeyFrame()
{
    if (p_localMapper->isInitializing() && !p_atlas->isImuInitialized())
    {
        return;
    }

    if (!p_localMapper->setNotStop(true))
    {
        return;
    }

    KeyFrame *p_keyFrame = new KeyFrame(currentFrame,
                                        p_atlas->getCurrentMap(),
                                        p_keyFrameDatabase);

    if (p_atlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
    {
        p_keyFrame->isImu = true;
    }

    p_keyFrame->setNewBias(currentFrame.imuBias);
    p_referenceKF                    = p_keyFrame;
    currentFrame.p_referenceKeyFrame = p_keyFrame;

    if (p_lastKeyFrame)
    {
        p_keyFrame->p_prevKF     = p_lastKeyFrame;
        p_lastKeyFrame->p_nextKF = p_keyFrame;
    }
    else
    {
        Verbose::printMess("No last KF in KF creation!!",
                           Verbose::VERBOSITY_NORMAL);
    }

    // Reset preintegration from last KF (Create new object)
    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(p_keyFrame->getImuBias(),
                                   p_keyFrame->imuCalibration);
    }

    if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
    {
        currentFrame.updatePoseMatrices();
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // Both sensor branches intentionally use the same cap of 100.
        int maximumPoint = 100;
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            maximumPoint = 100;
        }

        vector<pair<float, int>> depthIndices;
        int                      N = (currentFrame.leftKeyPointCount != -1)
                                         ? currentFrame.leftKeyPointCount
                                         : currentFrame.keyPointCount;
        depthIndices.reserve(currentFrame.keyPointCount);
        for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
        {
            float z = currentFrame.depths[keyPointIndex];
            if (z > 0)
            {
                depthIndices.push_back(make_pair(z, keyPointIndex));
            }
        }

        if (!depthIndices.empty())
        {
            sort(depthIndices.begin(), depthIndices.end());

            int pointCount = 0;
            for (size_t depthIndexIndex = 0;
                 depthIndexIndex < depthIndices.size();
                 depthIndexIndex++)
            {
                bool shouldCreateNewPoint = false;
                int  keyPointIndex = depthIndices[depthIndexIndex].second;

                MapPoint *p_mapPoint = currentFrame.mapPoints[keyPointIndex];
                if (!p_mapPoint)
                {
                    shouldCreateNewPoint = true;
                }
                else if (p_mapPoint->getObservationCount() < 1)
                {
                    shouldCreateNewPoint = true;
                    currentFrame.mapPoints[keyPointIndex] =
                        static_cast<MapPoint *>(nullptr);
                }

                if (shouldCreateNewPoint)
                {
                    Eigen::Vector3f x3D;

                    if (currentFrame.leftKeyPointCount == -1)
                    {
                        currentFrame.unprojectStereo(keyPointIndex, x3D);
                    }
                    else
                    {
                        x3D =
                            currentFrame.unprojectStereoFishEye(keyPointIndex);
                    }

                    MapPoint *p_newMapPoint =
                        new MapPoint(x3D, p_keyFrame, p_atlas->getCurrentMap());
                    p_newMapPoint->addObservation(p_keyFrame, keyPointIndex);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    if (currentFrame.leftKeyPointCount != -1 &&
                        currentFrame.leftToRightMatches[keyPointIndex] >= 0)
                    {
                        currentFrame.mapPoints
                            [currentFrame.leftKeyPointCount +
                             currentFrame.leftToRightMatches[keyPointIndex]] =
                            p_newMapPoint;
                        p_newMapPoint->addObservation(
                            p_keyFrame,
                            currentFrame.leftKeyPointCount +
                                currentFrame.leftToRightMatches[keyPointIndex]);
                        p_keyFrame->addMapPoint(
                            p_newMapPoint,
                            currentFrame.leftKeyPointCount +
                                currentFrame.leftToRightMatches[keyPointIndex]);
                    }

                    p_keyFrame->addMapPoint(p_newMapPoint, keyPointIndex);
                    p_newMapPoint->computeDistinctiveDescriptors();
                    p_newMapPoint->updateNormalAndDepth();
                    p_atlas->addMapPoint(p_newMapPoint);

                    currentFrame.mapPoints[keyPointIndex] = p_newMapPoint;
                    pointCount++;
                }
                else
                {
                    pointCount++;
                }

                if (depthIndices[depthIndexIndex].first > depthThreshold &&
                    pointCount > maximumPoint)
                {
                    break;
                }
            }
        }
    }

    // Check if the marker ids fromt he current frame exist in all the previous
    // keyframes first get the mapped marker from the keyframes
    for (const auto p_currentMapMarker : p_atlas->getAllMarkers())
    {
        // Check if the marker is already in the Global map
        for (auto p_currentFrameMaker : currentFrame.mapMarkers)
        {
            int currentFrameMakerId{};
            if (p_currentFrameMaker->getId(currentFrameMakerId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int currentMapMarkerId{};
            if (p_currentMapMarker->getId(currentMapMarkerId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (currentFrameMakerId == currentMapMarkerId)
            {
                if (p_currentFrameMaker->setMarkerInGMap(true) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    // setMarkerInGMap cannot fail; continue as before.
                }
            }
        }
    }

    p_localMapper->insertKeyFrame(p_keyFrame);

    p_localMapper->setNotStop(false);

    lastKeyFrameId = currentFrame.id;
    p_lastKeyFrame = p_keyFrame;
}

} // namespace core
} // namespace vs_graphs
