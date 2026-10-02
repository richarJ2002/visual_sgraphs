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
 * @file            preSave.cc
 *
 * @brief           Implements Map::preSave(), declared in Map.h.
 */

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "SerializationUtils.h"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapStatus Map::preSave(
    std::set<camera_models::geometriccamera::GeometricCamera *> &cams_inout)
{
    int mapPointWithoutObservationCount = 0;

    std::set<MapPoint *> temporaryMspMapPoints1;
    temporaryMspMapPoints1.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *p_mapPoint : temporaryMspMapPoints1)
    {
        bool mapPointIsBad{};
        if (!(!p_mapPoint) && p_mapPoint->isBad(mapPointIsBad) !=
                                  MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_mapPoint || mapPointIsBad)
            continue;

        std::map<KeyFrame *, std::tuple<int, int>> mapPointObservations{};
        if (p_mapPoint->getObservations(mapPointObservations) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointObservations.size() == 0)
        {
            mapPointWithoutObservationCount++;
        }
        std::map<KeyFrame *, std::tuple<int, int>> observations{};
        if (p_mapPoint->getObservations(observations) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::map<KeyFrame *, std::tuple<int, int>>::iterator
                 observationIt = observations.begin(),
                 end           = observations.end();
             observationIt != end;
             ++observationIt)
        {
            Map *p_map = nullptr;
            if (observationIt->first->getMap(p_map) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            bool isBad2{};
            if (!(p_map != this) &&
                observationIt->first->isBad(isBad2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map != this || isBad2)
            {
                if (p_mapPoint->eraseObservation(observationIt->first) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseObservation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    // Saves the id of KF origins
    backupKeyFrameOriginIds.clear();
    backupKeyFrameOriginIds.reserve(keyFrameOrigins.size());
    for (int elIndex = 0, elCount = keyFrameOrigins.size(); elIndex < elCount;
         ++elIndex)
    {
        backupKeyFrameOriginIds.push_back(keyFrameOrigins[elIndex]->id);
    }

    // Backup of MapPoints
    backupMapPoints.clear();

    std::set<MapPoint *> temporaryMspMapPoints2;
    temporaryMspMapPoints2.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *p_mapPoint : temporaryMspMapPoints2)
    {
        bool mapPointIsBad2{};
        if (!(!p_mapPoint) && p_mapPoint->isBad(mapPointIsBad2) !=
                                  MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_mapPoint || mapPointIsBad2)
            continue;

        backupMapPoints.push_back(p_mapPoint);
        if (p_mapPoint->preSave(keyFrames, mapPoints) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: preSave returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Backup of KeyFrames
    backupKeyFrames.clear();
    for (KeyFrame *p_keyFrame : keyFrames)
    {
        bool keyFrameIsBad{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad)
            continue;

        backupKeyFrames.push_back(p_keyFrame);
        if (p_keyFrame->preSave(keyFrames, mapPoints, cams_inout) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: preSave returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    backupInitialKeyFrameId = NO_SAVED_ID<unsigned long int>;
    if (p_initialKeyFrame)
    {
        backupInitialKeyFrameId = p_initialKeyFrame->id;
    }

    backupLowerKeyFrameId = NO_SAVED_ID<unsigned long int>;
    if (p_lowerIdKeyFrame)
    {
        backupLowerKeyFrameId = p_lowerIdKeyFrame->id;
    }

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
