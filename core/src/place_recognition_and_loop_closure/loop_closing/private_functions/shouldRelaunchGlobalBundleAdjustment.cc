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
 * @file            shouldRelaunchGlobalBundleAdjustment.cc
 *
 * @brief           Implements shouldRelaunchGlobalBundleAdjustment(), declared
 * in LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool shouldRelaunchGlobalBundleAdjustment(
    bool          wasGlobalBundleAdjustmentStopped_in,
    map::Map     *p_currentMap_in,
    atlas::Atlas *p_atlas_inout)
{
    bool currentMapIsImuInitialized4{};
    if ((wasGlobalBundleAdjustmentStopped_in) &&
        p_currentMap_in->isImuInitialized(currentMapIsImuInitialized4) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long currentMapKeyFrameCount{};
    if ((wasGlobalBundleAdjustmentStopped_in) &&
        !(!currentMapIsImuInitialized4) &&
        p_currentMap_in->getKeyFrameCount(currentMapKeyFrameCount) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    int atlasMaps{};
    if ((wasGlobalBundleAdjustmentStopped_in) &&
        !(!currentMapIsImuInitialized4) && (currentMapKeyFrameCount < 200) &&
        p_atlas_inout->countMaps(atlasMaps) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return wasGlobalBundleAdjustmentStopped_in &&
           (!currentMapIsImuInitialized4 ||
            (currentMapKeyFrameCount < 200 && atlasMaps == 1));
}

} /* namespace core */
} /* namespace vs_graphs */
