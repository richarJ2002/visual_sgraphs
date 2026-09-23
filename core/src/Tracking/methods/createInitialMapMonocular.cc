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

#include "Optimizer.h"
#include "ResetCause.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

void Tracking::createInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(initialFrame,
                                    p_atlas->getCurrentMap(),
                                    p_keyFrameDatabase);
    KeyFrame *pKFcur = new KeyFrame(currentFrame,
                                    p_atlas->getCurrentMap(),
                                    p_keyFrameDatabase);

    if (sensor == System::IMU_MONOCULAR)
        pKFini->p_imuPreintegrated = (IMU::Preintegrated *)(nullptr);

    pKFini->computeBagOfWords();
    pKFcur->computeBagOfWords();

    // Insert KFs in the map
    p_atlas->addKeyFrame(pKFini);
    p_atlas->addKeyFrame(pKFcur);

    for (size_t i = 0; i < iniMatches.size(); i++)
    {
        if (iniMatches[i] < 0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << iniP3D[i].x, iniP3D[i].y, iniP3D[i].z;
        Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), worldPos);
        Sophus::SE3f Twmp = poseTc0w.inverse() * Tc0mp;
        worldPos          = Twmp.translation();
        MapPoint *pMP =
            new MapPoint(worldPos, pKFcur, p_atlas->getCurrentMap());

        pKFini->addMapPoint(pMP, i);
        pKFcur->addMapPoint(pMP, iniMatches[i]);

        pMP->addObservation(pKFini, i);
        pMP->addObservation(pKFcur, iniMatches[i]);

        pMP->computeDistinctiveDescriptors();
        pMP->updateNormalAndDepth();

        // Fill Current Frame structure
        currentFrame.mapPoints[iniMatches[i]]    = pMP;
        currentFrame.outlierFlags[iniMatches[i]] = false;

        // Add to Map
        p_atlas->addMapPoint(pMP);
    }

    // Update Connections
    pKFini->updateConnections();
    pKFcur->updateConnections();

    std::set<MapPoint *> sMPs;
    sMPs = pKFini->getMapPoints();

    // Bundle Adjustment
    std::cout << "\n[Tracking]" << std::endl;
    std::cout << "- New map created with #"
              << to_string(p_atlas->getMapPointCount()) << " points!"
              << std::endl;
    Optimizer::globalBundleAdjustment(
        p_atlas->getCurrentMap(),
        20,
        nullptr,
        0,
        true,
        types::SystemParams::getParams()->markers.impact);

    float medianDepth = pKFini->computeSceneMedianDepth(2);
    float invMedianDepth;
    if (sensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;
    else
        invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 ||
        pKFcur->getTrackedMapPointCount(1) < 50) // TODO Check, originally 100
                                                 // tracks
    {
        Verbose::printMess("Wrong initialization, reseting...",
                           Verbose::VERBOSITY_QUIET);
        p_system->requestResetActiveMapWithCause(
            ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP);
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->getPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->setPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->getMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->setWorldPos(pMP->getWorldPos() * invMedianDepth);
            pMP->updateNormalAndDepth();
        }
    }

    if (sensor == System::IMU_MONOCULAR)
    {
        pKFcur->p_prevKF           = pKFini;
        pKFini->p_nextKF           = pKFcur;
        pKFcur->p_imuPreintegrated = p_imuPreintegratedFromLastKF;

        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(pKFcur->p_imuPreintegrated->getUpdatedBias(),
                                   pKFcur->imuCalibration);
    }

    p_localMapper->insertKeyFrame(pKFini);
    p_localMapper->insertKeyFrame(pKFcur);
    p_localMapper->firstTimestamp = pKFcur->timeStamp;

    currentFrame.setPose(pKFcur->getPose());
    lastKeyFrameId = currentFrame.mnId;
    p_lastKeyFrame = pKFcur;
    // mnLastRelocFrameId = mInitialFrame.mnId;

    localKeyFrames.push_back(pKFcur);
    localKeyFrames.push_back(pKFini);
    localMapPoints                   = p_atlas->getAllMapPoints();
    p_referenceKF                    = pKFcur;
    currentFrame.p_referenceKeyFrame = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame *> vKFs = p_atlas->getAllKeyFrames();

    Sophus::SE3f deltaT =
        vKFs.back()->getPose() * vKFs.front()->getPoseInverse();
    velocityAvailable   = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (currentFrame.timeStamp - lastFrame.timeStamp) /
                 (currentFrame.timeStamp - initialFrame.timeStamp);
    phi *= aux;

    lastFrame = Frame(currentFrame);

    p_atlas->setReferenceMapPoints(localMapPoints);

    p_mapDrawer->setCurrentCameraPose(pKFcur->getPose());

    p_atlas->getCurrentMap()->keyFrameOrigins.push_back(pKFini);

    state = OK;

    initId = pKFcur->mnId;
}

} // namespace core
} // namespace vs_graphs
