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

namespace vs_graphs
{
namespace core
{

void Tracking::createMapInAtlas()
{
    lastInitFrameId = currentFrame.id;
    p_atlas->createNewMap();
    if (sensor == System::IMU_STEREO || sensor == System::IMU_MONOCULAR ||
        sensor == System::IMU_RGBD)
    {
        p_atlas->setInertialSensor();
    }
    isInitSet = false;

    initialFrameId = currentFrame.id + 1;
    state          = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    isVelocityAvailable = false;
    // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is
    // the current id, because it is the new starting point for new map
    Verbose::printMess("First frame id in map: " +
                           to_string(lastInitFrameId + 1),
                       Verbose::VERBOSITY_NORMAL);
    isVisualOdometry = false; // Init value for know if there are enough
                              // MapPoints in the last KF
    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
    {
        isReadyToInitialize = false;
    }

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        p_imuPreintegratedFromLastKF)
    {
        delete p_imuPreintegratedFromLastKF;
        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
    }

    if (p_lastKeyFrame)
        p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);

    if (p_referenceKF)
        p_referenceKF = static_cast<KeyFrame *>(nullptr);

    lastFrame    = Frame();
    currentFrame = Frame();
    iniMatches.clear();

    hasCreatedMap = true;
}

} // namespace core
} // namespace vs_graphs
