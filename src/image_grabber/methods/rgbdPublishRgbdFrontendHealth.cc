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
 * @file            rgbdPublishRgbdFrontendHealth.cc
 *
 * @brief           Publishes RGB-D frontend health to the SLAM system.
 */

#include "common/common.h"
#include "image_grabber/objects/RgbdImageGrabberClass.h"
#include "observability/observability.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs::image_grabber
{

void RgbdImageGrabber::publishRgbdFrontendHealth() const
{
    if (vs_graphs::common::p_slamSystem == nullptr)
    {
        return;
    }

    const vs_graphs::observability::RgbdObservabilitySnapshot snapshot =
        rgbdObservability.snapshot();
    const std::uint64_t terminalCount =
        snapshot.processedPackets + snapshot.imageConversionRejects +
        snapshot.cloudConversionRejects + snapshot.trackFailures +
        snapshot.publishTopicsFailures + snapshot.shutdownPendingDrops;
    if (vs_graphs::common::p_slamSystem->updateRgbdFrontendHealth(
            snapshot.pendingStores,
            terminalCount,
            snapshot.pendingOverwrites,
            snapshot.workersInFlight > 0U,
            snapshot.lastProcessedSensorTimestampNanoseconds) !=
        vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateRgbdFrontendHealth returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace vs_graphs::image_grabber */
