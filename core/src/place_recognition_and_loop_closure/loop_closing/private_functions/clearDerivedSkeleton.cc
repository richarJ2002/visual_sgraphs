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
 * @file            clearDerivedSkeleton.cc
 *
 * @brief           Implements clearDerivedSkeleton(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void clearDerivedSkeleton(map::Map *p_currentMap_inout)
{
    /*
     * Voxblox topology is derived from a TSDF/ESDF volume and is not an
     * independently mergeable landmark set. Appending snapshots from two
     * map frames creates disconnected duplicate edges and false wall
     * crossings. The external Voxblox node receives the map-revision event,
     * clears its volume, and supplies a fresh snapshot after reintegration.
     */
    if (p_currentMap_inout->setSkeletonClusterPoints({}) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap_inout->setSkeletonEdges({}) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonEdges returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
