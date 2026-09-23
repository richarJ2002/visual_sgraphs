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

#include "ResetCause.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

// Map initialization for Stereo and RGB-D (with/without IMU) setups
void Tracking::stereoInitialization()
{
    // Require more points for robust initialization in corridors
    if (currentFrame.N > initializationMinPoints)
    {
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            if (!currentFrame.p_imuPreintegrated ||
                !lastFrame.p_imuPreintegrated)
            {
                std::cout << "[Tracking] IMU measurements are not available "
                             "for the current frame!"
                          << std::endl;
                return;
            }

            // Check acceleration difference for fast initialization
            if (!fastInit)
            {
                const double accelDiff =
                    (currentFrame.p_imuPreintegratedFrame->avgA -
                     lastFrame.p_imuPreintegratedFrame->avgA)
                        .norm();

                if (accelDiff < imuThresh)
                {
                    std::cout << "[Tracking] Low IMU acceleration changes: "
                              << std::fixed << std::setprecision(2) << accelDiff
                              << " (threshold: " << imuThresh
                              << ")! Skipping ..." << std::endl;
                    return;
                }
            }

            if (p_imuPreintegratedFromLastKF)
                delete p_imuPreintegratedFromLastKF;

            // Reset IMU preintegration from last keyframe
            p_imuPreintegratedFromLastKF =
                new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
            currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 =
                currentFrame.imuCalibration.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 =
                currentFrame.imuCalibration.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            currentFrame.setImuPoseVelocity(Rwb0, twb0, Vwb0);
        }
        else
            currentFrame.setPose(Sophus::SE3f());

        // Create KeyFrame
        vs_graphs::core::KeyFrame *pKFini =
            new vs_graphs::core::KeyFrame(currentFrame,
                                          p_atlas->getCurrentMap(),
                                          p_keyFrameDatabase);

        // Insert KeyFrame in the map
        p_atlas->addKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        int nPointsCreated = 0;
        if (!p_camera2)
        {
            for (int i = 0; i < currentFrame.N; i++)
            {
                float z = currentFrame.depths[i];
                if (z > 0)
                {
                    Eigen::Vector3f x3D;
                    currentFrame.unprojectStereo(i, x3D);
                    MapPoint *pNewMP =
                        new MapPoint(x3D, pKFini, p_atlas->getCurrentMap());
                    pNewMP->addObservation(pKFini, i);
                    pKFini->addMapPoint(pNewMP, i);
                    pNewMP->computeDistinctiveDescriptors();
                    pNewMP->updateNormalAndDepth();
                    p_atlas->addMapPoint(pNewMP);

                    currentFrame.mapPoints[i] = pNewMP;
                    nPointsCreated++;
                }
            }
        }
        else
        {
            for (int i = 0; i < currentFrame.Nleft; i++)
            {
                int rightIndex = currentFrame.leftToRightMatches[i];
                if (rightIndex != -1)
                {
                    Eigen::Vector3f x3D = currentFrame.stereoPoints3D[i];

                    MapPoint *pNewMP =
                        new MapPoint(x3D, pKFini, p_atlas->getCurrentMap());

                    pNewMP->addObservation(pKFini, i);
                    pNewMP->addObservation(pKFini,
                                           rightIndex + currentFrame.Nleft);

                    pKFini->addMapPoint(pNewMP, i);
                    pKFini->addMapPoint(pNewMP,
                                        rightIndex + currentFrame.Nleft);

                    pNewMP->computeDistinctiveDescriptors();
                    pNewMP->updateNormalAndDepth();
                    p_atlas->addMapPoint(pNewMP);

                    currentFrame.mapPoints[i] = pNewMP;
                    currentFrame.mapPoints[rightIndex + currentFrame.Nleft] =
                        pNewMP;
                    nPointsCreated++;
                }
            }
        }

        std::cout << "\n[Tracking] New map created with #" +
                         to_string(p_atlas->getMapPointCount()) + " points!"
                  << std::endl;

        // Require minimum points for successful initialization
        if (nPointsCreated < initializationMinPoints)
        {
            std::cout << "[Tracking] Insufficient points for initialization ("
                      << nPointsCreated << " < " << initializationMinPoints
                      << "), resetting..." << std::endl;
            p_system->requestResetActiveMapWithCause(
                ResetCause::INITIALIZATION_INSUFFICIENT_POINTS);
            return;
        }

        p_localMapper->insertKeyFrame(pKFini);

        lastFrame      = Frame(currentFrame);
        lastKeyFrameId = currentFrame.mnId;
        p_lastKeyFrame = pKFini;

        localKeyFrames.push_back(pKFini);
        localMapPoints                   = p_atlas->getAllMapPoints();
        p_referenceKF                    = pKFini;
        currentFrame.p_referenceKeyFrame = pKFini;

        p_atlas->setReferenceMapPoints(localMapPoints);

        p_atlas->getCurrentMap()->keyFrameOrigins.push_back(pKFini);

        p_mapDrawer->setCurrentCameraPose(currentFrame.getPose());

        state = OK;
    }
}

} // namespace core
} // namespace vs_graphs
