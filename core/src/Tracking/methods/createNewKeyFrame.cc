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

#include "Tracking.h"

namespace vs_graphs
{
namespace core
{

void Tracking::createNewKeyFrame()
{
    if (p_localMapper->isInitializing() && !p_atlas->isImuInitialized())
        return;

    if (!p_localMapper->setNotStop(true))
        return;

    KeyFrame *pKF = new KeyFrame(currentFrame,
                                 p_atlas->getCurrentMap(),
                                 p_keyFrameDatabase);

    if (p_atlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->isImu = true;

    pKF->setNewBias(currentFrame.imuBias);
    p_referenceKF                    = pKF;
    currentFrame.p_referenceKeyFrame = pKF;

    if (p_lastKeyFrame)
    {
        pKF->p_prevKF            = p_lastKeyFrame;
        p_lastKeyFrame->p_nextKF = pKF;
    }
    else
        Verbose::printMess("No last KF in KF creation!!",
                           Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(pKF->getImuBias(), pKF->imuCalibration);
    }

    if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
    {
        currentFrame.updatePoseMatrices();
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // Both sensor branches intentionally use the same cap of 100.
        int maxPoint = 100;
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            maxPoint = 100;

        vector<pair<float, int>> vDepthIdx;
        int                      N =
            (currentFrame.Nleft != -1) ? currentFrame.Nleft : currentFrame.N;
        vDepthIdx.reserve(currentFrame.N);
        for (int i = 0; i < N; i++)
        {
            float z = currentFrame.depths[i];
            if (z > 0)
                vDepthIdx.push_back(make_pair(z, i));
        }

        if (!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++)
            {
                bool bCreateNew = false;
                int  i          = vDepthIdx[j].second;

                MapPoint *pMP = currentFrame.mapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->getObservationCount() < 1)
                {
                    bCreateNew = true;
                    currentFrame.mapPoints[i] =
                        static_cast<MapPoint *>(nullptr);
                }

                if (bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    if (currentFrame.Nleft == -1)
                        currentFrame.unprojectStereo(i, x3D);
                    else
                        x3D = currentFrame.unprojectStereoFishEye(i);

                    MapPoint *pNewMP =
                        new MapPoint(x3D, pKF, p_atlas->getCurrentMap());
                    pNewMP->addObservation(pKF, i);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    if (currentFrame.Nleft != -1 &&
                        currentFrame.leftToRightMatches[i] >= 0)
                    {
                        currentFrame
                            .mapPoints[currentFrame.Nleft +
                                       currentFrame.leftToRightMatches[i]] =
                            pNewMP;
                        pNewMP->addObservation(
                            pKF,
                            currentFrame.Nleft +
                                currentFrame.leftToRightMatches[i]);
                        pKF->addMapPoint(
                            pNewMP,
                            currentFrame.Nleft +
                                currentFrame.leftToRightMatches[i]);
                    }

                    pKF->addMapPoint(pNewMP, i);
                    pNewMP->computeDistinctiveDescriptors();
                    pNewMP->updateNormalAndDepth();
                    p_atlas->addMapPoint(pNewMP);

                    currentFrame.mapPoints[i] = pNewMP;
                    nPoints++;
                }
                else
                    nPoints++;

                if (vDepthIdx[j].first > depthThreshold && nPoints > maxPoint)
                {
                    break;
                }
            }
        }
    }

    // Check if the marker ids fromt he current frame exist in all the previous
    // keyframes first get the mapped marker from the keyframes
    for (const auto currentMapMarker : p_atlas->getAllMarkers())
    {
        // Check if the marker is already in the Global map
        for (auto currentFrameMaker : currentFrame.mapMarkers)
            if (currentFrameMaker->getId() == currentMapMarker->getId())
                currentFrameMaker->setMarkerInGMap(true);
    }

    p_localMapper->insertKeyFrame(pKF);

    p_localMapper->setNotStop(false);

    lastKeyFrameId = currentFrame.mnId;
    p_lastKeyFrame = pKF;
}

} // namespace core
} // namespace vs_graphs
