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
 * @file            correctMapPointsToCurrentWorld.cc
 *
 * @brief           Implements correctMapPointsToCurrentWorld(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void correctMapPointsToCurrentWorld(
    const std::vector<map_point::MapPoint *> &mapPoints_in,
    map::Map                                 *p_mergeMap_in,
    const g2o::Sim3 &mapTransform_mergeWorldToCurrentWorld_in)
{
    for (map_point::MapPoint *p_mapPoint : mapPoints_in)
    {
        bool mapPointIsBad{};
        if (!(p_mapPoint == nullptr) &&
            p_mapPoint->isBad(mapPointIsBad) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_mapPointMap = nullptr;
        if (!(p_mapPoint == nullptr || mapPointIsBad) &&
            p_mapPoint->getMap(p_mapPointMap) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint == nullptr || mapPointIsBad ||
            p_mapPointMap != p_mergeMap_in)
        {
            continue;
        }

        Eigen::Vector3f position_mergeWorld_m{};
        if (p_mapPoint->getWorldPos(position_mergeWorld_m) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        Eigen::Vector3f normal_mergeWorld{};
        if (p_mapPoint->getNormal(normal_mergeWorld) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->setWorldPos(
                mapTransform_mergeWorldToCurrentWorld_in
                    .map(position_mergeWorld_m.cast<double>())
                    .cast<float>()) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->setNormalVector(
                mapTransform_mergeWorldToCurrentWorld_in.rotation()
                    .cast<float>() *
                normal_mergeWorld) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setNormalVector returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->updateNormalAndDepth() !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
