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
 * @file            applyInertialMapAlignment.cc
 *
 * @brief           Implements applyInertialMapAlignment(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "estimation/tracking/objects/TrackingStatusEnum.h"
#include "estimation/tracking/tracking.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/local_mapping/local_mapping.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "sensor_models/imu_types/bias/bias.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void applyInertialMapAlignment(
    local_mapping::LocalMapping *p_localMapper_inout,
    tracking::Tracking          *p_tracker_inout,
    keyframe::KeyFrame          *p_currentKeyFrame_in,
    map::Map                    *p_currentMap_inout,
    SemanticsManager            *p_semanticsManager_in,
    const g2o::Sim3             &mapTransform_currentWorldToMergeWorld_in)
{
    float        s_on = mapTransform_currentWorldToMergeWorld_in.scale();
    Sophus::SE3f T_on(
        mapTransform_currentWorldToMergeWorld_in.rotation().cast<float>(),
        mapTransform_currentWorldToMergeWorld_in.translation().cast<float>());

    std::unique_lock<std::mutex> currentMapUpdateLock(
        p_currentMap_inout->mapUpdateMutex);

    if (p_localMapper_inout->emptyQueue() !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: emptyQueue returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    bool shouldScaleVelocity = false;
    if (s_on != 1)
        shouldScaleVelocity = true;
    if (p_currentMap_inout->applyScaledRotation(p_semanticsManager_in,
                                                T_on,
                                                s_on,
                                                shouldScaleVelocity) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: applyScaledRotation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    imu_types::bias::Bias currentKFImuBias{};
    if (p_currentKeyFrame_in->getImuBias(currentKFImuBias) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getImuBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    keyframe::KeyFrame *p_trackerLastKeyFrame = nullptr;
    if (p_tracker_inout->getLastKeyFrame(p_trackerLastKeyFrame) !=
        tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLastKeyFrame returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_tracker_inout->updateFrameIMU(s_on,
                                        currentKFImuBias,
                                        p_trackerLastKeyFrame) !=
        tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateFrameIMU returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
