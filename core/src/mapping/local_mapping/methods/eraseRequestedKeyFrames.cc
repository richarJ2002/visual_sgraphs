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
 * @file            eraseRequestedKeyFrames.cc
 *
 * @brief           Implements LocalMapping::eraseRequestedKeyFrames(),
 *                  declared in LocalMapping.h.
 */

#include "mapping/local_mapping/objects/LocalMappingClass.h"

#include <algorithm>
#include <list>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <vector>

#include "estimation/tracking/objects/TrackingStatusEnum.h"
#include "estimation/tracking/tracking.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace local_mapping
{

LocalMappingStatus LocalMapping::eraseRequestedKeyFrames()
{
    SemanticsManager          *p_semanticsManager = nullptr;
    std::vector<unsigned long> requestedKeyFrameIds{};
    if (p_atlas->getSemanticsManager(p_semanticsManager) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
        (p_semanticsManager != nullptr &&
         p_semanticsManager->takeKeyFrameErasureRequests(
             requestedKeyFrameIds) !=
             SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSemanticsManager/takeKeyFrameErasureRequests "
                     "returned a failure status although they cannot fail; "
                     "continuing as before.",
                     __func__);
    }
    if (requestedKeyFrameIds.empty())
    {
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
    }

    /* Key frames other threads may still be using are kept. */
    keyframe::KeyFrame *p_trackerReferenceKeyFrame = nullptr;
    keyframe::KeyFrame *p_trackerLastKeyFrame      = nullptr;
    if (p_tracker->getReferenceKeyFrame(p_trackerReferenceKeyFrame) !=
            tracking::TrackingStatus::TRACKING_STATUS_SUCCESS ||
        p_tracker->getLastKeyFrame(p_trackerLastKeyFrame) !=
            tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getReferenceKeyFrame/getLastKeyFrame returned a "
                     "failure status although they cannot fail; continuing "
                     "as before.",
                     __func__);
    }
    std::list<keyframe::KeyFrame *> queuedKeyFrames{};
    {
        std::unique_lock<std::mutex> lock(newKeyFramesMutex);
        queuedKeyFrames = newKeyFrames;
    }

    std::size_t erasedKeyFrameCount = 0U;
    for (const unsigned long requestedKeyFrameId : requestedKeyFrameIds)
    {
        keyframe::KeyFrame *p_keyFrame = nullptr;
        bool                keyFrameIsBad{true};
        map::Map           *p_keyFrameMap = nullptr;
        unsigned long       initKeyFrameId{};
        if (p_atlas->getKeyFrameById(requestedKeyFrameId, p_keyFrame) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
            (p_keyFrame != nullptr &&
             (p_keyFrame->isBad(keyFrameIsBad) !=
                  keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS ||
              p_keyFrame->getMap(p_keyFrameMap) !=
                  keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS ||
              p_keyFrameMap == nullptr ||
              p_keyFrameMap->getInitKeyFrameId(initKeyFrameId) !=
                  map::MapStatus::MAP_STATUS_SUCCESS)))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKeyFrameById/isBad/getMap/getInitKeyFrameId "
                         "returned a failure status; the key frame is kept.",
                         __func__);
            continue;
        }
        if (p_keyFrame == nullptr || keyFrameIsBad ||
            p_keyFrame->id == initKeyFrameId ||
            p_keyFrame == p_currentKeyFrame ||
            p_keyFrame == p_trackerReferenceKeyFrame ||
            p_keyFrame == p_trackerLastKeyFrame ||
            std::find(queuedKeyFrames.begin(),
                      queuedKeyFrames.end(),
                      p_keyFrame) != queuedKeyFrames.end())
        {
            continue;
        }

        /* Inertial: join the neighbours' preintegration as keyFrameCulling()
         * does; a key frame at either end of the chain, among the newest two,
         * or with a gap of 3 s or more between its neighbours is kept. */
        if (isInertial)
        {
            if (p_keyFrame->p_prevKF == nullptr ||
                p_keyFrame->p_nextKF == nullptr ||
                p_keyFrame->id + 2U > p_currentKeyFrame->id ||
                p_keyFrame->p_nextKF->timeStamp -
                        p_keyFrame->p_prevKF->timeStamp >=
                    3.0)
            {
                continue;
            }
            if (p_keyFrame->p_nextKF->p_imuPreintegrated->mergePrevious(
                    p_keyFrame->p_imuPreintegrated) !=
                imu_types::preintegrated::PreintegratedStatus::
                    PREINTEGRATED_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: mergePrevious returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame->p_nextKF->p_prevKF = p_keyFrame->p_prevKF;
            p_keyFrame->p_prevKF->p_nextKF = p_keyFrame->p_nextKF;
            p_keyFrame->p_nextKF           = nullptr;
            p_keyFrame->p_prevKF           = nullptr;
        }

        /* A key frame with loop edges is deferred by setBadFlag() itself. */
        if (p_keyFrame->setBadFlag() !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBadFlag returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        erasedKeyFrameCount++;
    }

    RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                "[LocalMapping] Erased %zu of %zu key frames of deleted "
                "rooms; the others are in use or protected.",
                erasedKeyFrameCount,
                requestedKeyFrameIds.size());
    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} /* namespace local_mapping */
} /* namespace core */
} /* namespace vs_graphs */
