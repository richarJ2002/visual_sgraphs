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
 * @file            correctMergeMapPoints.cc
 *
 * @brief           Implements correctMergeMapPoints(), declared in
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

void correctMergeMapPoints(
    std::set<map_point::MapPoint *> &mapPointMerges_inout,
    map::Map                        *p_mergeMap_in,
    const g2o::Sim3                 &mapTransform_mergeWorldToCurrentWorld_in)
{
    for (std::set<map_point::MapPoint *>::const_iterator itMapPoint =
             mapPointMerges_inout.begin();
         itMapPoint != mapPointMerges_inout.end();)
    {
        /* Copy the map points */
        map_point::MapPoint *p_currentMapPoint = *itMapPoint;

        /* If the mapped points are invalud, erase and skip */
        bool currentMapPointIsBad{};
        if (!(!p_currentMapPoint) &&
            p_currentMapPoint->isBad(currentMapPointIsBad) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_currentMapPointMap = nullptr;
        if (!(!p_currentMapPoint || currentMapPointIsBad) &&
            p_currentMapPoint->getMap(p_currentMapPointMap) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_currentMapPoint || currentMapPointIsBad ||
            p_currentMapPointMap != p_mergeMap_in)
        {
            itMapPoint = mapPointMerges_inout.erase(itMapPoint);
            continue;
        }

        /* Extract position of point */
        Eigen::Vector3f currentMapPointWorldPos{};
        if (p_currentMapPoint->getWorldPos(currentMapPointWorldPos) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d P3DwMerge =
            currentMapPointWorldPos.cast<double>();

        /* Transform the point into the current map world frame */
        p_currentMapPoint->posMerge =
            mapTransform_mergeWorldToCurrentWorld_in.map(P3DwMerge)
                .cast<float>();

        /* Transform the points surface normal into current map world frame */
        Eigen::Vector3f currentMapPointNormal{};
        if (p_currentMapPoint->getNormal(currentMapPointNormal) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_currentMapPoint->normalVectorMerge =
            mapTransform_mergeWorldToCurrentWorld_in.rotation().cast<float>() *
            currentMapPointNormal;

        /* Step to next mapped point */
        itMapPoint++;
    }
}

} /* namespace core */
} /* namespace vs_graphs */
