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
 *                  declared in system/objects/SystemClass.h.
 */

#include "mapping/local_mapping/local_mapping.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "execution/reset_cause/objects/ResetActionEnum.h"
#include "execution/reset_cause/objects/ResetCauseEnum.h"
#include "execution/reset_cause/objects/ResetCauseStatusEnum.h"
#include "estimation/tracking/objects/TrackingStatusEnum.h"
#include "execution/system/objects/SystemClass.h"
#include "estimation/tracking/tracking.h"

#include <mutex>
#include <rclcpp/logging.hpp>
#include <unistd.h>

namespace vs_graphs
{
namespace core
{
namespace system
{

SystemStatus System::applyPendingModeAndResetRequests()
{
    /* Check mode change */
    {
        std::unique_lock<std::mutex> lock(modeMutex);
        if (isLocalizationModeActivationRequested)
        {
            if (p_localMapper->requestStop() !=
                local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: requestStop returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Wait until Local Mapping has effectively stopped */
            for (;;)
            {
                bool localMapperIsStopped{};
                if (p_localMapper->isStopped(localMapperIsStopped) !=
                    local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
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
                tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
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
                tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: informOnlyTracking returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_localMapper->release() !=
                local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: release returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            isLocalizationModeDeactivationRequested = false;
        }
    }

    /* Check reset */
    {
        std::unique_lock<std::mutex> lock(resetMutex);
        if (isResetRequested)
        {
            reset_cause::ResetCause resetCause{};
            if (reset_cause::consumeResetCause(this, resetCause) !=
                reset_cause::ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: consumeResetCause returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_tracker->reset() != tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
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
            reset_cause::ResetCause resetCause{};
            if (reset_cause::consumeResetCause(this, resetCause) !=
                reset_cause::ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: consumeResetCause returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (reset_cause::reportResetAttribution(
                    resetCause,
                    reset_cause::ResetAction::RESET_ACTIVE_MAP_EXECUTION) !=
                reset_cause::ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reportResetAttribution returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_tracker->resetActiveMap() !=
                tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
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

} /* namespace system */
} /* namespace core */
} /* namespace vs_graphs */
