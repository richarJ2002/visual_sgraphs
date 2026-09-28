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

#include <iostream>

namespace vs_graphs
{
namespace core
{

bool Tracking::predictStateIMU()
{
    if (!currentFrame.p_previousFrame)
    {
        Verbose::printMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if (isMapUpdated && p_lastKeyFrame)
    {
        const Eigen::Vector3f twb1 = p_lastKeyFrame->getImuPosition();
        const Eigen::Matrix3f Rwb1 = p_lastKeyFrame->getImuRotation();
        const Eigen::Vector3f Vwb1 = p_lastKeyFrame->getVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float           t12 = p_imuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::normalizeRotation(
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaRotation(
                       p_lastKeyFrame->getImuBias()));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaPosition(
                       p_lastKeyFrame->getImuBias());
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaVelocity(
                       p_lastKeyFrame->getImuBias());
        currentFrame.setImuPoseVelocity(Rwb2, twb2, Vwb2);

        currentFrame.imuBias       = p_lastKeyFrame->getImuBias();
        currentFrame.predictedBias = currentFrame.imuBias;
        return true;
    }
    else if (!isMapUpdated)
    {
        const Eigen::Vector3f twb1 = lastFrame.getImuPosition();
        const Eigen::Matrix3f Rwb1 = lastFrame.getImuRotation();
        const Eigen::Vector3f Vwb1 = lastFrame.getVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float           t12 = currentFrame.p_imuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::normalizeRotation(
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaRotation(
                       lastFrame.imuBias));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaPosition(
                       lastFrame.imuBias);
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaVelocity(
                       lastFrame.imuBias);

        currentFrame.setImuPoseVelocity(Rwb2, twb2, Vwb2);

        currentFrame.imuBias       = lastFrame.imuBias;
        currentFrame.predictedBias = currentFrame.imuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

} // namespace core
} // namespace vs_graphs
