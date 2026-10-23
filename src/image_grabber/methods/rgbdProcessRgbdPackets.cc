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
 * @file            rgbdProcessRgbdPackets.cc
 *
 * @brief           Processes admitted RGB-D packets on the tracking worker.
 */

#include "common/common.h"
#include "image_grabber/objects/RgbdImageGrabberClass.h"
#include "observability/observability.h"

#include <condition_variable>
#include <rclcpp/logging.hpp>

namespace vs_graphs::image_grabber
{

void RgbdImageGrabber::processRgbdPackets()
{
    while (true)
    {
        RgbdPacket rgbdPacket;

        {
            std::unique_lock<std::mutex> packetLock(rgbdPacketMutex);
            rgbdPacketCondition.wait(
                packetLock,
                [this]() { return stopRequested || hasPendingRgbdPacket; });

            if (stopRequested)
            {
                const bool hadPendingPacket = hasPendingRgbdPacket;
                hasPendingRgbdPacket        = false;
                packetLock.unlock();
                if (hadPendingPacket)
                {
                    rgbdObservability.recordShutdownPendingDrop();
                    publishRgbdFrontendHealth();
                }
                break;
            }

            rgbdPacket           = std::move(latestRgbdPacket);
            hasPendingRgbdPacket = false;
        }

        rgbdObservability.recordWorkerStart();
        publishRgbdFrontendHealth();

        const double rgbTimestamp_seconds =
            rclcpp::Time(rgbdPacket.p_rgbImageMessage->header.stamp).seconds();

        cv_bridge::CvImageConstPtr p_depthImage;
        cv_bridge::CvImageConstPtr p_rgbImage;

        try
        {
            p_depthImage = cv_bridge::toCvShare(rgbdPacket.p_depthImageMessage);
            p_rgbImage   = cv_bridge::toCvShare(rgbdPacket.p_rgbImageMessage);
        }
        catch (const cv_bridge::Exception &exception)
        {
            rgbdObservability.recordImageConversionReject();
            publishRgbdFrontendHealth();
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding an RGB-D packet after cv_bridge failed: %s",
                exception.what());
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr        p_pointCloud;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudCameraMessage;
        std::string                                   cloudFailureReason;
        if (!vs_graphs::common::preparePointCloudForTracking(
                rgbdPacket.p_pointCloudMessage,
                directGazeboFluCloud,
                p_pointCloud,
                p_pointCloudCameraMessage,
                cloudFailureReason))
        {
            rgbdObservability.recordCloudConversionReject();
            publishRgbdFrontendHealth();
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "Discarding point cloud: %s.",
                                 cloudFailureReason.c_str());
            continue;
        }

        std::pair<double, std::vector<vs_graphs::core::semantic::Marker *>>
            nearestMarker =
                vs_graphs::common::findNearestMarker(rgbTimestamp_seconds);
        const double markerTimeDifference_seconds = nearestMarker.first;
        std::vector<vs_graphs::core::semantic::Marker *> matchedMarkers =
            std::move(nearestMarker.second);

        try
        {
            if (markerTimeDifference_seconds < 0.05)
            {
                Sophus::SE3f slamSystemCameraPose{};
                if (vs_graphs::common::p_slamSystem->trackRGBD(
                        p_rgbImage->image,
                        p_depthImage->image,
                        p_pointCloud,
                        rgbTimestamp_seconds,
                        slamSystemCameraPose,
                        {},
                        "",
                        matchedMarkers) !=
                    vs_graphs::core::system::SystemStatus::
                        SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackRGBD returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                Sophus::SE3f slamSystemCameraPose2{};
                if (vs_graphs::common::p_slamSystem->trackRGBD(
                        p_rgbImage->image,
                        p_depthImage->image,
                        p_pointCloud,
                        rgbTimestamp_seconds,
                        slamSystemCameraPose2) !=
                    vs_graphs::core::system::SystemStatus::
                        SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackRGBD returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
        catch (...)
        {
            rgbdObservability.recordTrackFailure();
            publishRgbdFrontendHealth();
            throw;
        }
        if (markerTimeDifference_seconds < 0.05)
        {
            vs_graphs::common::markersBuffer.clear();
        }
        double estimatorInterval_seconds = 0.0;
        if (hasProcessedRgbdPacket)
        {
            const double frameInterval_seconds =
                rgbTimestamp_seconds - lastProcessedRgbdTimestamp_seconds;
            estimatorInterval_seconds = frameInterval_seconds;
            if (frameInterval_seconds > 0.5)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "RGB-D processing cannot keep pace: estimator frames are "
                    "%.3f seconds apart.",
                    frameInterval_seconds);
            }
        }
        hasProcessedRgbdPacket             = true;
        lastProcessedRgbdTimestamp_seconds = rgbTimestamp_seconds;
        vs_graphs::common::recordEstimatorFrame(estimatorInterval_seconds);

        const rclcpp::Time messageTimestamp =
            rgbdPacket.p_rgbImageMessage->header.stamp;
        try
        {
            vs_graphs::common::publishTopics(messageTimestamp,
                                             Eigen::Vector3f::Zero(),
                                             p_pointCloudCameraMessage);
        }
        catch (...)
        {
            rgbdObservability.recordPublishTopicsFailure();
            publishRgbdFrontendHealth();
            throw;
        }
        rgbdObservability.recordProcessed(
            rgbdPacket.sensorTimestampNanoseconds);
        publishRgbdFrontendHealth();
    }
}

} /* namespace vs_graphs::image_grabber */
