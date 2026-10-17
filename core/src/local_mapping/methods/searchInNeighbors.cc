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
 * @file            searchInNeighbors.cc
 *
 * @brief           Implements LocalMapping::searchInNeighbors(), declared in
 *                  LocalMapping.h.
 */

#include "local_mapping/objects/LocalMappingClass.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "map_point/map_point.h"
#include "map_point/objects/MapPointStatusEnum.h"
#include "orb_matcher/objects/ORBmatcherStatusEnum.h"

#include "orb_matcher/orb_matcher.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace local_mapping
{

LocalMappingStatus LocalMapping::searchInNeighbors()
{
    /* Retrieve neighbor keyframes */
    int neighborKeyFrameCount = 10;
    if (isMonocular)
        neighborKeyFrameCount = 30;
    std::vector<keyframe::KeyFrame *> neighborKeyFrames{};
    if (p_currentKeyFrame->getBestCovisibilityKeyFrames(neighborKeyFrameCount,
                                                        neighborKeyFrames) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBestCovisibilityKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<keyframe::KeyFrame *> targetKeyFrames;
    for (std::vector<keyframe::KeyFrame *>::const_iterator
             targetKeyFrameIt  = neighborKeyFrames.begin(),
             targetKeyFrameEnd = neighborKeyFrames.end();
         targetKeyFrameIt != targetKeyFrameEnd;
         targetKeyFrameIt++)
    {
        keyframe::KeyFrame *p_targetKeyFrame = *targetKeyFrameIt;
        bool                targetKeyFrameIsBad{};
        if (p_targetKeyFrame->isBad(targetKeyFrameIsBad) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (targetKeyFrameIsBad ||
            p_targetKeyFrame->fuseTargetKeyFrameId == p_currentKeyFrame->id)
            continue;
        targetKeyFrames.push_back(p_targetKeyFrame);
        p_targetKeyFrame->fuseTargetKeyFrameId = p_currentKeyFrame->id;
    }

    /*!
     * Add some covisible of covisible
     * Extend to some second neighbors if abort is not requested
     */
    for (int elementIndex = 0, targetKeyFrameCount = targetKeyFrames.size();
         elementIndex < targetKeyFrameCount;
         elementIndex++)
    {
        std::vector<keyframe::KeyFrame *> secondNeighborKeyFrames{};
        if (targetKeyFrames[elementIndex]->getBestCovisibilityKeyFrames(
                20,
                secondNeighborKeyFrames) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getBestCovisibilityKeyFrames returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        for (std::vector<keyframe::KeyFrame *>::const_iterator
                 secondNeighborKeyFrameIt  = secondNeighborKeyFrames.begin(),
                 secondNeighborKeyFrameEnd = secondNeighborKeyFrames.end();
             secondNeighborKeyFrameIt != secondNeighborKeyFrameEnd;
             secondNeighborKeyFrameIt++)
        {
            keyframe::KeyFrame *p_secondNeighborKeyFrame =
                *secondNeighborKeyFrameIt;
            bool secondNeighborKeyFrameIsBad{};
            if (p_secondNeighborKeyFrame->isBad(secondNeighborKeyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (secondNeighborKeyFrameIsBad ||
                p_secondNeighborKeyFrame->fuseTargetKeyFrameId ==
                    p_currentKeyFrame->id ||
                p_secondNeighborKeyFrame->id == p_currentKeyFrame->id)
                continue;
            targetKeyFrames.push_back(p_secondNeighborKeyFrame);
            p_secondNeighborKeyFrame->fuseTargetKeyFrameId =
                p_currentKeyFrame->id;
        }
        if (shouldAbortBa)
            break;
    }

    /* Extend to temporal neighbors */
    if (isInertial)
    {
        keyframe::KeyFrame *p_targetKeyFrame = p_currentKeyFrame->p_prevKF;
        while (targetKeyFrames.size() < 20 && p_targetKeyFrame)
        {
            bool targetKeyFrameIsBad2{};
            if (p_targetKeyFrame->isBad(targetKeyFrameIsBad2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (targetKeyFrameIsBad2 ||
                p_targetKeyFrame->fuseTargetKeyFrameId == p_currentKeyFrame->id)
            {
                p_targetKeyFrame = p_targetKeyFrame->p_prevKF;
                continue;
            }
            targetKeyFrames.push_back(p_targetKeyFrame);
            p_targetKeyFrame->fuseTargetKeyFrameId = p_currentKeyFrame->id;
            p_targetKeyFrame                       = p_targetKeyFrame->p_prevKF;
        }
    }

    /* Search matches by projection from current KF in target KFs */
    orb_matcher::ORBmatcher                         matcher;
    std::vector<map_point::MapPoint *> currentMapPointMatches{};
    if (p_currentKeyFrame->getMapPointMatches(currentMapPointMatches) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (std::vector<keyframe::KeyFrame *>::iterator
             targetKeyFrameIt  = targetKeyFrames.begin(),
             targetKeyFrameEnd = targetKeyFrames.end();
         targetKeyFrameIt != targetKeyFrameEnd;
         targetKeyFrameIt++)
    {
        keyframe::KeyFrame *p_targetKeyFrame = *targetKeyFrameIt;

        int matcherFusedCount{};
        if (matcher.fuse(p_targetKeyFrame,
                         currentMapPointMatches,
                         matcherFusedCount) !=
            orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fuse returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_targetKeyFrame->leftKeyPointCount != -1)
        {
            int matcherFusedCount2{};
            if (matcher.fuse(p_targetKeyFrame,
                             currentMapPointMatches,
                             matcherFusedCount2,
                             true) !=
                orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: fuse returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    if (shouldAbortBa)
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;

    /* Search matches by projection from target KFs in current KF */
    std::vector<map_point::MapPoint *> fuseCandidateMapPoints;
    fuseCandidateMapPoints.reserve(targetKeyFrames.size() *
                                   currentMapPointMatches.size());

    for (std::vector<keyframe::KeyFrame *>::iterator
             targetKeyFrameIt2  = targetKeyFrames.begin(),
             targetKeyFrameEnd2 = targetKeyFrames.end();
         targetKeyFrameIt2 != targetKeyFrameEnd2;
         targetKeyFrameIt2++)
    {
        keyframe::KeyFrame *p_targetKeyFrame = *targetKeyFrameIt2;

        std::vector<map_point::MapPoint *> targetMapPoints{};
        if (p_targetKeyFrame->getMapPointMatches(targetMapPoints) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        for (std::vector<map_point::MapPoint *>::iterator
                 targetMapPointIt  = targetMapPoints.begin(),
                 targetMapPointEnd = targetMapPoints.end();
             targetMapPointIt != targetMapPointEnd;
             targetMapPointIt++)
        {
            map_point::MapPoint *p_mapPoint = *targetMapPointIt;
            if (!p_mapPoint)
                continue;
            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad ||
                p_mapPoint->fuseCandidateKeyFrameId == p_currentKeyFrame->id)
                continue;
            p_mapPoint->fuseCandidateKeyFrameId = p_currentKeyFrame->id;
            fuseCandidateMapPoints.push_back(p_mapPoint);
        }
    }

    int matcherFusedCount3{};
    if (matcher.fuse(p_currentKeyFrame,
                     fuseCandidateMapPoints,
                     matcherFusedCount3) !=
        orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: fuse returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_currentKeyFrame->leftKeyPointCount != -1)
    {
        int matcherFusedCount4{};
        if (matcher.fuse(p_currentKeyFrame,
                         fuseCandidateMapPoints,
                         matcherFusedCount4,
                         true) != orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fuse returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Update points */
    std::vector<map_point::MapPoint *> currentKeyFrameMapPointMatches{};
    if (p_currentKeyFrame->getMapPointMatches(currentKeyFrameMapPointMatches) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    currentMapPointMatches = currentKeyFrameMapPointMatches;
    for (size_t elementIndex = 0, mapPointCount = currentMapPointMatches.size();
         elementIndex < mapPointCount;
         elementIndex++)
    {
        map_point::MapPoint *p_mapPoint = currentMapPointMatches[elementIndex];
        if (p_mapPoint)
        {
            bool mapPointIsBad2{};
            if (p_mapPoint->isBad(mapPointIsBad2) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!mapPointIsBad2)
            {
                if (p_mapPoint->computeDistinctiveDescriptors() !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: computeDistinctiveDescriptors returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_mapPoint->updateNormalAndDepth() !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: updateNormalAndDepth returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    /* Update connections in covisibility graph */
    if (p_currentKeyFrame->updateConnections() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} /* namespace local_mapping */
} /* namespace core */
} /* namespace vs_graphs */
