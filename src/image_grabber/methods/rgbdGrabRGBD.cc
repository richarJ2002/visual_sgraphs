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
 * @file            rgbdGrabRGBD.cc
 *
 * @brief           Admits coherent RGB-D and point-cloud packets.
 */

#include "common/common.h"
#include "image_grabber/objects/RgbdImageGrabberClass.h"
#include "observability/observability.h"

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <rclcpp/logging.hpp>

namespace vs_graphs::image_grabber
{

void RgbdImageGrabber::grabRgbd(
    const sensor_msgs::msg::Image::ConstSharedPtr       &p_rgbImageMessage_in,
    const sensor_msgs::msg::Image::ConstSharedPtr       &p_depthImageMessage_in,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &p_pointCloudMessage_in)
{
    const std::chrono::steady_clock::time_point callbackArrival =
        std::chrono::steady_clock::now();
    rgbdObservability.recordCallbackAdmission(callbackArrival);

    const rclcpp::Time rgbTimestamp(p_rgbImageMessage_in->header.stamp);
    const rclcpp::Time depthTimestamp(p_depthImageMessage_in->header.stamp);
    const rclcpp::Time pointCloudTimestamp(
        p_pointCloudMessage_in->header.stamp);
    const double rgbTimestamp_seconds = rgbTimestamp.seconds();

    /* Keep inclusive skew limits exact regardless of the sensor epoch. */
    constexpr std::int64_t MAXIMUM_RGB_DEPTH_SKEW_NS = 10'000'000;
    const std::int64_t     rgbDepthSkew_ns =
        std::abs((rgbTimestamp - depthTimestamp).nanoseconds());
    if (rgbDepthSkew_ns > MAXIMUM_RGB_DEPTH_SKEW_NS)
    {
        rgbdObservability.recordRgbDepthSkewReject();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of RGB-depth skew.",
            rclcpp::Duration::from_nanoseconds(rgbDepthSkew_ns).seconds());
        return;
    }

    constexpr std::int64_t MAXIMUM_CLOUD_IMAGE_SKEW_NS = 33'000'000;
    const std::int64_t     cloudImageSkew_ns           = std::max(
        std::abs((rgbTimestamp - pointCloudTimestamp).nanoseconds()),
        std::abs((depthTimestamp - pointCloudTimestamp).nanoseconds()));
    if (cloudImageSkew_ns > MAXIMUM_CLOUD_IMAGE_SKEW_NS)
    {
        rgbdObservability.recordCloudImageSkewReject();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of cloud-image "
            "skew.",
            rclcpp::Duration::from_nanoseconds(cloudImageSkew_ns).seconds());
        return;
    }

    bool didRejectNonMonotonic = false;
    bool didRejectShutdown     = false;
    bool didOverwritePending   = false;
    {
        std::lock_guard<std::mutex> packetLock(rgbdPacketMutex);

        if (stopRequested)
        {
            didRejectShutdown = true;
        }
        else if (hasReceivedRgbdPacket &&
                 rgbTimestamp_seconds <= lastReceivedRgbdTimestamp_seconds)
        {
            didRejectNonMonotonic = true;
        }
        else
        {
            didOverwritePending = hasPendingRgbdPacket;
            latestRgbdPacket    = {
                p_rgbImageMessage_in,
                p_depthImageMessage_in,
                p_pointCloudMessage_in,
                rclcpp::Time(p_rgbImageMessage_in->header.stamp).nanoseconds(),
                callbackArrival};
            hasPendingRgbdPacket              = true;
            hasReceivedRgbdPacket             = true;
            lastReceivedRgbdTimestamp_seconds = rgbTimestamp_seconds;
        }
    }

    if (didRejectShutdown)
    {
        rgbdObservability.recordShutdownAdmissionReject();
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding an RGB-D packet after shutdown "
                             "was requested.");
        return;
    }

    if (didRejectNonMonotonic)
    {
        rgbdObservability.recordNonMonotonicReject();
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic RGB-D packet.");
        return;
    }

    rgbdObservability.recordPendingStore(1U, didOverwritePending);
    publishRgbdFrontendHealth();
    rgbdPacketCondition.notify_one();
}

} /* namespace vs_graphs::image_grabber */
