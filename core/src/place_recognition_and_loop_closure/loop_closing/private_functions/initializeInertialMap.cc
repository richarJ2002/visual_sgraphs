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
 * @file            initializeInertialMap.cc
 *
 * @brief           Implements initializeInertialMap(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "estimation/tracking/objects/TrackingStatusEnum.h"
#include "estimation/tracking/tracking.h"
#include "execution/system/system.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "optimization/optimizer/objects/OptimizerStatusEnum.h"
#include "optimization/optimizer/optimizer.h"
#include "sensor_models/imu_types/bias/bias.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void initializeInertialMap(tracking::Tracking *p_tracker_inout,
                           map::Map           *p_currentMap_inout)
{
    bool currentMapInertialBA2{};
    if (((p_tracker_inout->sensor == system::System::IMU_MONOCULAR ||
          p_tracker_inout->sensor == system::System::IMU_STEREO ||
          p_tracker_inout->sensor == system::System::IMU_RGBD)) &&
        p_currentMap_inout->getInertialBA2(currentMapInertialBA2) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInertialBA2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if ((p_tracker_inout->sensor == system::System::IMU_MONOCULAR ||
         p_tracker_inout->sensor == system::System::IMU_STEREO ||
         p_tracker_inout->sensor == system::System::IMU_RGBD) &&
        !currentMapInertialBA2)
    {
        /* map::Map is not completly initialized */
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        if (optimizer::Optimizer::inertialOptimization(p_currentMap_inout,
                                                       bg,
                                                       ba) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: inertialOptimization returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        imu_types::bias::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        std::unique_lock<std::mutex> currentMapUpdateLock(
            p_currentMap_inout->mapUpdateMutex);
        keyframe::KeyFrame *p_trackerLastKeyFrame2 = nullptr;
        if (p_tracker_inout->getLastKeyFrame(p_trackerLastKeyFrame2) !=
            tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker_inout->updateFrameIMU(1.0f, b, p_trackerLastKeyFrame2) !=
            tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateFrameIMU returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Set map initialized */
        if (p_currentMap_inout->setInertialBA2() !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialBA2 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap_inout->setInertialBA1() !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialBA1 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap_inout->setImuInitialized() !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
