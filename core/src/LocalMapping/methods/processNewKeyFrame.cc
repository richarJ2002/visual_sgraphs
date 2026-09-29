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

#include "LocalMapping.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LocalMappingStatus LocalMapping::processNewKeyFrame()
{
    {
        unique_lock<mutex> newKeyFramesLock(newKeyFramesMutex);
        p_currentKeyFrame = newKeyFrames.front();
        newKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    if (p_currentKeyFrame->computeBagOfWords() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeBagOfWords returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Associate MapPoints to the new keyframe and update normal and descriptor
    std::vector<MapPoint *> matchedMapPoints{};
    if (p_currentKeyFrame->getMapPointMatches(matchedMapPoints) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    for (size_t mapPointIndex = 0; mapPointIndex < matchedMapPoints.size();
         mapPointIndex++)
    {
        MapPoint *p_mapPoint = matchedMapPoints[mapPointIndex];
        if (p_mapPoint)
        {
            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!mapPointIsBad)
            {
                bool mapPointIsInKeyFrame{};
                if (p_mapPoint->isInKeyFrame(p_currentKeyFrame,
                                             mapPointIsInKeyFrame) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isInKeyFrame returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (!mapPointIsInKeyFrame)
                {
                    if (p_mapPoint->addObservation(p_currentKeyFrame,
                                                   mapPointIndex) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_mapPoint->updateNormalAndDepth() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: updateNormalAndDepth returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    if (p_mapPoint->computeDistinctiveDescriptors() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDistinctiveDescriptors "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                }
                else // this can only happen for new stereo points inserted by
                     // the Tracking
                {
                    recentAddedMapPoints.push_back(p_mapPoint);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    if (p_currentKeyFrame->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Insert Keyframe in Map
    if (p_atlas->addKeyFrame(p_currentKeyFrame) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
