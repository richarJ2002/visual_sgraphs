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
 * @file            detectAndReffineSim3FromLastKF.cc
 *
 * @brief           Implements LoopClosing::detectAndReffineSim3FromLastKF(),
 *                  declared in loop_closing/objects/LoopClosingClass.h.
 */

#include "loop_closing/objects/LoopClosingClass.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "map/map.h"
#include "map/objects/MapStatusEnum.h"
#include "map_point/map_point.h"
#include "optimizer/objects/OptimizerStatusEnum.h"

#include "system/system.h"
#include "optimizer/optimizer.h"
#include "tracking/tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace loop_closing
{

LoopClosingStatus LoopClosing::detectAndReffineSim3FromLastKF(
    keyframe::KeyFrame                 *p_currentKeyFrame_in,
    keyframe::KeyFrame                 *p_matchedKeyFrame_in,
    g2o::Sim3                          &gScw_inout,
    int                                &countProjectionMatchCount_out,
    std::vector<map_point::MapPoint *> &mapPoints_inout,
    std::vector<map_point::MapPoint *> &matchedMapPoints_inout,
    bool                               &isDetected_out)
{
    int matches2{};
    if (findMatchesByProjection(p_currentKeyFrame_in,
                                p_matchedKeyFrame_in,
                                gScw_inout,
                                mapPoints_inout,
                                matchedMapPoints_inout,
                                matches2) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: findMatchesByProjection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    countProjectionMatchCount_out = matches2;

    int projectionMatchCount      = 30;
    int projectionOptMatchCount   = 50;
    int projectionMatchesRepCount = 100;

    if (countProjectionMatchCount_out >= projectionMatchCount)
    {
        Sophus::SE3f matchedKeyFramePoseInverse{};
        if (p_matchedKeyFrame_in->getPoseInverse(matchedKeyFramePoseInverse) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d mTwm = matchedKeyFramePoseInverse.cast<double>();
        g2o::Sim3    gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
        g2o::Sim3    gScm = gScw_inout * gSwm;

        bool isFixedScale =
            isScaleFixed; /* TODO CHECK; Solo para el monocular inertial */
        map::Map *p_currentKeyFrameMap = nullptr;
        if ((p_tracker->sensor == system::System::IMU_MONOCULAR) &&
            p_currentKeyFrame_in->getMap(p_currentKeyFrameMap) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool inertialBA2{};
        if ((p_tracker->sensor == system::System::IMU_MONOCULAR) &&
            p_currentKeyFrameMap->getInertialBA2(inertialBA2) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getInertialBA2 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker->sensor == system::System::IMU_MONOCULAR && !inertialBA2)
            isFixedScale = false;
        int optMatchCount{};
        if (optimizer::Optimizer::optimizeSim3(p_currentKF,
                                    p_matchedKeyFrame_in,
                                    matchedMapPoints_inout,
                                    gScm,
                                    10,
                                    isFixedScale,
                                    optMatchCount,
                                    true) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: optimizeSim3 returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (optMatchCount > projectionOptMatchCount)
        {
            g2o::Sim3 gScw_estimation(gScw_inout.rotation(),
                                      gScw_inout.translation(),
                                      1.0);

            std::vector<map_point::MapPoint *> matchedMapPoints;
            std::vector<map_point::MapPoint *> currentKFMapPointMatches{};
            if (p_currentKF->getMapPointMatches(currentKFMapPointMatches) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPointMatches returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            matchedMapPoints.resize(
                currentKFMapPointMatches.size(),
                static_cast<map_point::MapPoint *>(nullptr));

            int matches3{};
            if (findMatchesByProjection(p_currentKeyFrame_in,
                                        p_matchedKeyFrame_in,
                                        gScw_estimation,
                                        mapPoints_inout,
                                        matchedMapPoints_inout,
                                        matches3) !=
                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: findMatchesByProjection returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            countProjectionMatchCount_out = matches3;
            if (countProjectionMatchCount_out >= projectionMatchesRepCount)
            {
                gScw_inout     = gScw_estimation;
                isDetected_out = true;
                return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
            }
        }
    }
    isDetected_out = false;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */
