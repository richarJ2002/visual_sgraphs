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
 * @file            mapPointCulling.cc
 *
 * @brief           Implements LocalMapping::mapPointCulling(), declared in
 *                  LocalMapping.h.
 */

#include "LocalMapping.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LocalMappingStatus LocalMapping::mapPointCulling()
{
    // Check Recent Added MapPoints
    std::list<MapPoint *>::iterator recentMapPointIt =
        recentAddedMapPoints.begin();
    const unsigned long int currentKeyFrameId = p_currentKeyFrame->id;

    int rawObservationThreshold;
    if (isMonocular)
        rawObservationThreshold = 2;
    else
        rawObservationThreshold = 3;
    const int observationThreshold = rawObservationThreshold;

    int remainingCandidateCount = recentAddedMapPoints.size();

    while (recentMapPointIt != recentAddedMapPoints.end())
    {
        MapPoint *p_mapPoint = *recentMapPointIt;

        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad)
        {
            recentMapPointIt = recentAddedMapPoints.erase(recentMapPointIt);
        }
        else
        {
            float mapPointFoundRatio{};
            if (p_mapPoint->getFoundRatio(mapPointFoundRatio) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getFoundRatio returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointFoundRatio < 0.25f)
            {
                if (p_mapPoint->setBadFlag() !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setBadFlag returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                recentMapPointIt = recentAddedMapPoints.erase(recentMapPointIt);
            }
            else
            {
                int mapPointObservationCount{};
                if (((static_cast<int>(currentKeyFrameId) -
                      static_cast<int>(p_mapPoint->firstKeyFrameId)) >= 2) &&
                    p_mapPoint->getObservationCount(mapPointObservationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getObservationCount returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if ((static_cast<int>(currentKeyFrameId) -
                     static_cast<int>(p_mapPoint->firstKeyFrameId)) >= 2 &&
                    mapPointObservationCount <= observationThreshold)
                {
                    if (p_mapPoint->setBadFlag() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setBadFlag returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    recentMapPointIt =
                        recentAddedMapPoints.erase(recentMapPointIt);
                }
                else if ((static_cast<int>(currentKeyFrameId) -
                          static_cast<int>(p_mapPoint->firstKeyFrameId)) >= 3)
                {
                    recentMapPointIt =
                        recentAddedMapPoints.erase(recentMapPointIt);
                }
                else
                {
                    recentMapPointIt++;
                    remainingCandidateCount--;
                }
            }
        }
    }

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
