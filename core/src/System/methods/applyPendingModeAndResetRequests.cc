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
 * @file            applyPendingModeAndResetRequests.cc
 *
 * @brief           Implements System::applyPendingModeAndResetRequests(),
 *                  declared in System.h.
 */

#include "LocalMapping.h"
#include "System.h"
#include "Tracking.h"

#include <mutex>
#include <rclcpp/logging.hpp>
#include <unistd.h>

namespace vs_graphs
{
namespace core
{

SystemStatus System::applyPendingModeAndResetRequests()
{
    // Check mode change
    {
        std::unique_lock<std::mutex> lock(modeMutex);
        if (isLocalizationModeActivationRequested)
        {
            if (p_localMapper->requestStop() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: requestStop returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            // Wait until Local Mapping has effectively stopped
            for (;;)
            {
                bool localMapperIsStopped{};
                if (p_localMapper->isStopped(localMapperIsStopped) !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isStopped returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (localMapperIsStopped)
                {
                    break;
                }
                usleep(1000);
            }

            if (p_tracker->informOnlyTracking(true) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: informOnlyTracking returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            isLocalizationModeActivationRequested = false;
        }
        if (isLocalizationModeDeactivationRequested)
        {
            if (p_tracker->informOnlyTracking(false) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: informOnlyTracking returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_localMapper->release() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: release returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            isLocalizationModeDeactivationRequested = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(resetMutex);
        if (isResetRequested)
        {
            ResetCause resetCause{};
            if (consumeResetCause(this, resetCause) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: consumeResetCause returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_tracker->reset() != TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: reset returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            isResetRequested          = false;
            isResetActiveMapRequested = false;
        }
        else if (isResetActiveMapRequested)
        {
            ResetCause resetCause{};
            if (consumeResetCause(this, resetCause) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: consumeResetCause returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (reportResetAttribution(
                    resetCause,
                    ResetAction::RESET_ACTIVE_MAP_EXECUTION) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
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
            isResetActiveMapRequested = false;
        }
    }

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
