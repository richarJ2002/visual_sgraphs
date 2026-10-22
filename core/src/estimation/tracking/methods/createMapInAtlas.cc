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
 * @file            createMapInAtlas.cc
 *
 * @brief           Implements Tracking::createMapInAtlas(), declared in
 *                  Tracking.h.
 */

#include "execution/system/system.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/frame/frame.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/preintegrated.h"
#include "map_state/keyframe/keyframe.h"
#include "execution/system/objects/VerboseStatusEnum.h"
#include "estimation/tracking/objects/TrackingClass.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::createMapInAtlas()
{
    lastInitFrameId = currentFrame.id;
    if (p_atlas->createNewMap() != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: createNewMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (sensor == system::System::IMU_STEREO ||
        sensor == system::System::IMU_MONOCULAR ||
        sensor == system::System::IMU_RGBD)
    {
        if (p_atlas->setInertialSensor() != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialSensor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    initialFrameId = currentFrame.id + 1;
    state          = NO_IMAGES_YET;

    /* Restart the variable with information about the last KF */
    isVelocityAvailable = false;
    if (system::Verbose::printMess(
            "First frame id in map: " + std::to_string(lastInitFrameId + 1),
            system::Verbose::VERBOSITY_NORMAL) != system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    /* Init value for know if there are enough MapPoints in the last KF */
    isVisualOdometry = false;
    if (sensor == system::System::MONOCULAR ||
        sensor == system::System::IMU_MONOCULAR)
    {
        isReadyToInitialize = false;
    }

    if ((sensor == system::System::IMU_MONOCULAR ||
         sensor == system::System::IMU_STEREO ||
         sensor == system::System::IMU_RGBD) &&
        p_imuPreintegratedFromLastKF)
    {
        delete p_imuPreintegratedFromLastKF;
        p_imuPreintegratedFromLastKF =
            new imu_types::preintegrated::Preintegrated(imu_types::bias::Bias(), *p_imuCalibration);
    }

    if (p_lastKeyFrame)
        p_lastKeyFrame = static_cast<keyframe::KeyFrame *>(nullptr);

    if (p_referenceKF)
        p_referenceKF = static_cast<keyframe::KeyFrame *>(nullptr);

    lastFrame    = frame::Frame();
    currentFrame = frame::Frame();
    iniMatches.clear();

    hasCreatedMap = true;

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
