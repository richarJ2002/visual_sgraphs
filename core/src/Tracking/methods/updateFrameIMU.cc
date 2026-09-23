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

void Tracking::updateFrameIMU(const float      s,
                              const IMU::Bias &b,
                              KeyFrame        *pCurrentKeyFrame)
{
    Map *pMap = pCurrentKeyFrame->getMap();
    list<vs_graphs::core::KeyFrame *>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator                        lbL  = mlbLost.begin();
    for (auto lit = relativeFramePoses.begin(), lend = relativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        while (pKF->isBad() && pKF->getParent())
        {
            pKF = pKF->getParent();
        }

        if (pKF->getMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }

    lastBias = b;

    p_lastKeyFrame = pCurrentKeyFrame;

    lastFrame.setNewBias(lastBias);
    currentFrame.setNewBias(lastBias);

    while (!currentFrame.isImuPreintegrated())
    {
        usleep(500);
    }

    if (lastFrame.mnId == lastFrame.p_lastKeyFrame->frameId)
    {
        lastFrame.setImuPoseVelocity(lastFrame.p_lastKeyFrame->getImuRotation(),
                                     lastFrame.p_lastKeyFrame->getImuPosition(),
                                     lastFrame.p_lastKeyFrame->getVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = lastFrame.p_lastKeyFrame->getImuPosition();
        const Eigen::Matrix3f Rwb1 = lastFrame.p_lastKeyFrame->getImuRotation();
        const Eigen::Vector3f Vwb1 = lastFrame.p_lastKeyFrame->getVelocity();
        float                 t12  = lastFrame.p_imuPreintegrated->dT;

        lastFrame.setImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 * lastFrame.p_imuPreintegrated->getUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * lastFrame.p_imuPreintegrated->getUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 * lastFrame.p_imuPreintegrated->getUpdatedDeltaVelocity());
    }

    if (currentFrame.p_imuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 =
            currentFrame.p_lastKeyFrame->getImuPosition();
        const Eigen::Matrix3f Rwb1 =
            currentFrame.p_lastKeyFrame->getImuRotation();
        const Eigen::Vector3f Vwb1 = currentFrame.p_lastKeyFrame->getVelocity();
        float                 t12  = currentFrame.p_imuPreintegrated->dT;

        currentFrame.setImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 *
                currentFrame.p_imuPreintegrated->getUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 *
                    currentFrame.p_imuPreintegrated->getUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 *
                    currentFrame.p_imuPreintegrated->getUpdatedDeltaVelocity());
    }

    firstImuFrameId = currentFrame.mnId;
}

} // namespace core
} // namespace vs_graphs
