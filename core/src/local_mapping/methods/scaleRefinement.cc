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
 * @file            scaleRefinement.cc
 *
 * @brief           Implements LocalMapping::scaleRefinement(), declared in
 *                  LocalMapping.h.
 */

#include "local_mapping/objects/LocalMappingClass.h"
#include "atlas/objects/AtlasStatusEnum.h"
#include "imu_types/bias/bias.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "map/map.h"
#include "map/objects/MapStatusEnum.h"
#include "optimizer/objects/OptimizerStatusEnum.h"
#include "tracking/objects/TrackingStatusEnum.h"

#include "optimizer/optimizer.h"
#include "tracking/tracking.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace local_mapping
{

LocalMappingStatus LocalMapping::scaleRefinement()
{
    /*!
     * Minimum number of keyframes to compute a solution
     * Minimum time (seconds) between first and last keyframe to compute a
     * solution. Make the difference between monocular and stereo
     */
    if (isResetRequested)
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;

    /* Retrieve all keyframes in temporal order */
    std::list<keyframe::KeyFrame *> temporalKeyFrames;
    keyframe::KeyFrame             *p_walkKeyFrame = p_currentKeyFrame;
    while (p_walkKeyFrame->p_prevKF)
    {
        temporalKeyFrames.push_front(p_walkKeyFrame);
        p_walkKeyFrame = p_walkKeyFrame->p_prevKF;
    }
    temporalKeyFrames.push_front(p_walkKeyFrame);
    std::vector<keyframe::KeyFrame *> orderedKeyFrames(
        temporalKeyFrames.begin(),
        temporalKeyFrames.end());

    for (;;)
    {
        bool hasNewKeyFrames{};
        if (checkNewKeyFrames(hasNewKeyFrames) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkNewKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!hasNewKeyFrames)
        {
            break;
        }
        if (processNewKeyFrame() !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: processNewKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        orderedKeyFrames.push_back(p_currentKeyFrame);
        temporalKeyFrames.push_back(p_currentKeyFrame);
    }

    mRwg  = Eigen::Matrix3d::Identity();
    scale = 1.0;

    map::Map *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (optimizer::Optimizer::inertialOptimization(p_atlasCurrentMap, mRwg, scale) !=
        optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: inertialOptimization returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (scale < 1e-1) /* 1e-1 */
    {
        std::cout << "scale too small" << std::endl;
        isInitializationInProgress = false;
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
    }

    Sophus::SO3d                 so3wg(mRwg);
    /* Before this line we are not changing the map */
    std::unique_lock<std::mutex> semanticUpdateLock{};
    if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: acquireSemanticUpdateLock returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    map::Map *p_activeMap = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_activeMap == nullptr)
    {
        isInitializationInProgress = false;
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
    }

    std::unique_lock<std::mutex> mapUpdateLock(p_activeMap->mapUpdateMutex);
    if ((fabs(scale - 1.f) > 0.002) || !isMonocular)
    {
        Sophus::SE3f alignmentPose_worldToGravity(
            mRwg.cast<float>().transpose(),
            Eigen::Vector3f::Zero());
        if (p_activeMap->applyScaledRotation(alignmentPose_worldToGravity,
                                             scale,
                                             true) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyScaledRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        imu_types::bias::Bias currentKeyFrameImuBias{};
        if (p_currentKeyFrame->getImuBias(currentKeyFrameImuBias) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker->updateFrameIMU(scale,
                                      currentKeyFrameImuBias,
                                      p_currentKeyFrame) !=
            tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateFrameIMU returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    for (std::list<keyframe::KeyFrame *>::iterator
             newKeyFrameIt  = newKeyFrames.begin(),
             newKeyFrameEnd = newKeyFrames.end();
         newKeyFrameIt != newKeyFrameEnd;
         newKeyFrameIt++)
    {
        if ((*newKeyFrameIt)->setBadFlag() !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBadFlag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        delete *newKeyFrameIt;
    }
    newKeyFrames.clear();

    /* To perform pose-inertial opt w.r.t. last keyframe */
    map::Map *p_currentKeyFrameMap = nullptr;
    if (p_currentKeyFrame->getMap(p_currentKeyFrameMap) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_currentKeyFrameMap->increaseChangeIndex() !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} /* namespace local_mapping */
} /* namespace core */
} /* namespace vs_graphs */
