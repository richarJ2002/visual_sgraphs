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
 * @file            collectMergeMapContents.cc
 *
 * @brief           Implements collectMergeMapContents(), declared in
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

void collectMergeMapContents(
    map::Map                       *p_mergeMap_in,
    loop_closing::MergeMapContents &mergeMapContents_out)
{
    mergeMapContents_out = loop_closing::MergeMapContents{};

    /* Copy all planes currently owned by the merge map */
    if (p_mergeMap_in->getAllPlanes(mergeMapContents_out.planes) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all keyframes currently owned by the merge map */
    if (p_mergeMap_in->getAllKeyFrames(mergeMapContents_out.keyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all map points currently owned by the merge map */
    if (p_mergeMap_in->getAllMapPoints(mergeMapContents_out.mapPoints) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all passages currently owned by the merge map */
    if (p_mergeMap_in->getAllPassages(mergeMapContents_out.passages) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all detected rooms currently owned by the merge map */
    if (p_mergeMap_in->getAllDetectedMapRooms(
            mergeMapContents_out.detectedRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all marker-based rooms currently owned by the merge map */
    if (p_mergeMap_in->getAllMarkerBasedMapRooms(
            mergeMapContents_out.markerBasedRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkerBasedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all floors currently owned by the merge map */
    if (p_mergeMap_in->getAllFloors(mergeMapContents_out.floors) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
