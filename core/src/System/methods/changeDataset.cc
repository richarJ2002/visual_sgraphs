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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            changeDataset.cc
 *
 * @brief           Implements System::changeDataset(), declared in System.h.
 */

#include "System.h"
#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::changeDataset()
{
    unsigned long keyFrameCount{};
    Map          *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_atlasCurrentMap->getKeyFrameCount(keyFrameCount) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (keyFrameCount < 12)
    {
        if (reportResetAttribution(ResetCause::DATASET_CHANGE_SMALL_MAP,
                                   ResetAction::RESET_ACTIVE_MAP_EXECUTION) !=
            ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reportResetAttribution returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker->resetActiveMap() !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resetActiveMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        resetCount.fetch_add(1U, std::memory_order_relaxed);
    }
    else
    {
        if (reportResetAttribution(ResetCause::DATASET_CHANGE_NEW_MAP,
                                   ResetAction::CREATE_MAP_EXECUTION) !=
            ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reportResetAttribution returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker->createMapInAtlas() !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createMapInAtlas returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (p_tracker->newDataset() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: newDataset returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
