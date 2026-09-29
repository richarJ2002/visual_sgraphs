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

#include "MapPoint.h"

#include "Map.h"
#include "ORBmatcher.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapPointStatus MapPoint::replace(MapPoint *p_mapPoint_inout)
{
    if (p_mapPoint_inout->id == this->id)
        return MapPointStatus::MAP_POINT_STATUS_SUCCESS;

    int                              nvisible, nfound;
    map<KeyFrame *, tuple<int, int>> observation;
    {
        unique_lock<mutex> lock1(featuresMutex);
        unique_lock<mutex> lock2(positionMutex);
        observation = observations;
        observations.clear();
        isFlaggedBad = true;
        nvisible     = visibleCount;
        nfound       = foundCount;
        p_replaced   = p_mapPoint_inout;
    }

    for (map<KeyFrame *, tuple<int, int>>::iterator mit  = observation.begin(),
                                                    mend = observation.end();
         mit != mend;
         mit++)
    {
        // Replace measurement in keyframe
        KeyFrame *p_keyFrame = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        bool mapPointIsInKeyFrame{};
        if (p_mapPoint_inout->isInKeyFrame(p_keyFrame, mapPointIsInKeyFrame) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInKeyFrame returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!mapPointIsInKeyFrame)
        {
            if (leftIndex != -1)
            {
                if (p_keyFrame->replaceMapPointMatch(leftIndex,
                                                     p_mapPoint_inout) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: replaceMapPointMatch returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_mapPoint_inout->addObservation(p_keyFrame, leftIndex) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addObservation returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
            if (rightIndex != -1)
            {
                if (p_keyFrame->replaceMapPointMatch(rightIndex,
                                                     p_mapPoint_inout) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: replaceMapPointMatch returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_mapPoint_inout->addObservation(p_keyFrame, rightIndex) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addObservation returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
        else
        {
            if (leftIndex != -1)
            {
                if (p_keyFrame->eraseMapPointMatch(leftIndex) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseMapPointMatch returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            if (rightIndex != -1)
            {
                if (p_keyFrame->eraseMapPointMatch(rightIndex) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseMapPointMatch returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }
    if (p_mapPoint_inout->increaseFound(nfound) !=
        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseFound returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_mapPoint_inout->increaseVisible(nvisible) !=
        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseVisible returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_mapPoint_inout->computeDistinctiveDescriptors() !=
        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeDistinctiveDescriptors returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    if (p_map->eraseMapPoint(this) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: eraseMapPoint returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
