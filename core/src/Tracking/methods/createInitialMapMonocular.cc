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

#include "LocalMapping.h"
#include "Optimizer.h"
#include "ResetCause.h"
#include "System.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

void Tracking::createInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame *p_keyFrameInitial = new KeyFrame(initialFrame,
                                               p_atlas->getCurrentMap(),
                                               p_keyFrameDatabase);
    KeyFrame *p_keyFrameCurrent = new KeyFrame(currentFrame,
                                               p_atlas->getCurrentMap(),
                                               p_keyFrameDatabase);

    if (sensor == System::IMU_MONOCULAR)
        p_keyFrameInitial->p_imuPreintegrated = (IMU::Preintegrated *)(nullptr);

    p_keyFrameInitial->computeBagOfWords();
    p_keyFrameCurrent->computeBagOfWords();

    // Insert KFs in the map
    p_atlas->addKeyFrame(p_keyFrameInitial);
    p_atlas->addKeyFrame(p_keyFrameCurrent);

    for (size_t initialMatchIndex = 0; initialMatchIndex < iniMatches.size();
         initialMatchIndex++)
    {
        if (iniMatches[initialMatchIndex] < 0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPosition;
        worldPosition << iniP3D[initialMatchIndex].x,
            iniP3D[initialMatchIndex].y, iniP3D[initialMatchIndex].z;
        Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), worldPosition);
        Sophus::SE3f Twmp    = poseTc0w.inverse() * Tc0mp;
        worldPosition        = Twmp.translation();
        MapPoint *p_mapPoint = new MapPoint(worldPosition,
                                            p_keyFrameCurrent,
                                            p_atlas->getCurrentMap());

        p_keyFrameInitial->addMapPoint(p_mapPoint, initialMatchIndex);
        p_keyFrameCurrent->addMapPoint(p_mapPoint,
                                       iniMatches[initialMatchIndex]);

        p_mapPoint->addObservation(p_keyFrameInitial, initialMatchIndex);
        p_mapPoint->addObservation(p_keyFrameCurrent,
                                   iniMatches[initialMatchIndex]);

        p_mapPoint->computeDistinctiveDescriptors();
        p_mapPoint->updateNormalAndDepth();

        // Fill Current Frame structure
        currentFrame.mapPoints[iniMatches[initialMatchIndex]]    = p_mapPoint;
        currentFrame.outlierFlags[iniMatches[initialMatchIndex]] = false;

        // Add to Map
        p_atlas->addMapPoint(p_mapPoint);
    }

    // Update Connections
    p_keyFrameInitial->updateConnections();
    p_keyFrameCurrent->updateConnections();

    std::set<MapPoint *> mapPoints;
    mapPoints = p_keyFrameInitial->getMapPoints();

    // Bundle Adjustment
    std::cout << "\n[Tracking]" << std::endl;
    std::cout << "- New map created with #"
              << to_string(p_atlas->getMapPointCount()) << " points!"
              << std::endl;
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    Optimizer::globalBundleAdjustment(p_atlas->getCurrentMap(),
                                      20,
                                      nullptr,
                                      0,
                                      true,
                                      p_params->markers.impact);

    float medianDepth = p_keyFrameInitial->computeSceneMedianDepth(2);
    float invMedianDepth;
    if (sensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;
    else
        invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || p_keyFrameCurrent->getTrackedMapPointCount(1) <
                               50) // TODO Check, originally 100
                                   // tracks
    {
        Verbose::printMess("Wrong initialization, reseting...",
                           Verbose::VERBOSITY_QUIET);
        p_system->requestResetActiveMapWithCause(
            ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP);
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = p_keyFrameCurrent->getPose();
    Tc2w.translation() *= invMedianDepth;
    p_keyFrameCurrent->setPose(Tc2w);

    // Scale points
    vector<MapPoint *> allMapPoints = p_keyFrameInitial->getMapPointMatches();
    for (size_t mapPointIndex = 0; mapPointIndex < allMapPoints.size();
         mapPointIndex++)
    {
        if (allMapPoints[mapPointIndex])
        {
            MapPoint *p_mapPoint = allMapPoints[mapPointIndex];
            p_mapPoint->setWorldPos(p_mapPoint->getWorldPos() * invMedianDepth);
            p_mapPoint->updateNormalAndDepth();
        }
    }

    if (sensor == System::IMU_MONOCULAR)
    {
        p_keyFrameCurrent->p_prevKF           = p_keyFrameInitial;
        p_keyFrameInitial->p_nextKF           = p_keyFrameCurrent;
        p_keyFrameCurrent->p_imuPreintegrated = p_imuPreintegratedFromLastKF;

        p_imuPreintegratedFromLastKF = new IMU::Preintegrated(
            p_keyFrameCurrent->p_imuPreintegrated->getUpdatedBias(),
            p_keyFrameCurrent->imuCalibration);
    }

    p_localMapper->insertKeyFrame(p_keyFrameInitial);
    p_localMapper->insertKeyFrame(p_keyFrameCurrent);
    p_localMapper->firstTimestamp = p_keyFrameCurrent->timeStamp;

    currentFrame.setPose(p_keyFrameCurrent->getPose());
    lastKeyFrameId = currentFrame.id;
    p_lastKeyFrame = p_keyFrameCurrent;
    // mnLastRelocFrameId = mInitialFrame.id;

    localKeyFrames.push_back(p_keyFrameCurrent);
    localKeyFrames.push_back(p_keyFrameInitial);
    localMapPoints                   = p_atlas->getAllMapPoints();
    p_referenceKF                    = p_keyFrameCurrent;
    currentFrame.p_referenceKeyFrame = p_keyFrameCurrent;

    // Compute here initial velocity
    vector<KeyFrame *> keyFrames = p_atlas->getAllKeyFrames();

    Sophus::SE3f deltaT =
        keyFrames.back()->getPose() * keyFrames.front()->getPoseInverse();
    isVelocityAvailable = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (currentFrame.timeStamp - lastFrame.timeStamp) /
                 (currentFrame.timeStamp - initialFrame.timeStamp);
    phi *= aux;

    lastFrame = Frame(currentFrame);

    p_atlas->setReferenceMapPoints(localMapPoints);

    p_mapDrawer->setCurrentCameraPose(p_keyFrameCurrent->getPose());

    p_atlas->getCurrentMap()->keyFrameOrigins.push_back(p_keyFrameInitial);

    state = OK;

    initId = p_keyFrameCurrent->id;
}

} // namespace core
} // namespace vs_graphs
