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

#include <condition_variable>
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

    const double rgbTimestamp_seconds =
        rclcpp::Time(p_rgbImageMessage_in->header.stamp).seconds();
    const double depthTimestamp_seconds =
        rclcpp::Time(p_depthImageMessage_in->header.stamp).seconds();
    const double pointCloudTimestamp_seconds =
        rclcpp::Time(p_pointCloudMessage_in->header.stamp).seconds();

    constexpr double maximumRgbDepthSkew_seconds = 0.010;
    const double     rgbDepthSkew_seconds =
        std::abs(rgbTimestamp_seconds - depthTimestamp_seconds);
    if (rgbDepthSkew_seconds > maximumRgbDepthSkew_seconds)
    {
        rgbdObservability.recordRgbDepthSkewReject();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of RGB-depth skew.",
            rgbDepthSkew_seconds);
        return;
    }

    constexpr double maximumCloudImageSkew_seconds = 0.033;
    const double     cloudImageSkew_seconds        = std::max(
        std::abs(rgbTimestamp_seconds - pointCloudTimestamp_seconds),
        std::abs(depthTimestamp_seconds - pointCloudTimestamp_seconds));
    if (cloudImageSkew_seconds > maximumCloudImageSkew_seconds)
    {
        rgbdObservability.recordCloudImageSkewReject();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of cloud-image "
            "skew.",
            cloudImageSkew_seconds);
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
