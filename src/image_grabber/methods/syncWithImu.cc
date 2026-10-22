/*!
 * @file            syncWithImu.cc
 *
 * @brief           Processes admitted RGB-D packets with their IMU intervals.
 *
 * @date            09/10/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

#include "image_grabber/objects/ImageGrabberClass.h"

namespace vs_graphs::image_grabber
{

void ImageGrabber::syncWithImu()
{
    /* Confirm that Imu grabber is initialized */
    if (!p_imuGrabber)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[Error] IMU Grabber not initialized!");
        return;
    }

    while (!mustStop)
    {
        sensor_msgs::msg::Image::ConstSharedPtr       p_rgbImageMessage;
        sensor_msgs::msg::Image::ConstSharedPtr       p_depthImageMessage;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
        std::vector<vs_graphs::core::imu_types::point::Point> imuMeasurements;
        Eigen::Vector3f angularVelocity_body_radPerSec =
            Eigen::Vector3f::Zero();
        double imageTimestamp_seconds     = 0.0;
        double latestImuTimestamp_seconds = 0.0;
        bool   waitingForImu              = false;
        double maximumImuGap_seconds      = 0.0;
        bool   imuContinuityOverflow      = false;

        /* Transfer one synchronized sensor packet while holding both buffer
         * mutexes. Keeping every queue access inside this critical section
         * prevents the ROS executor from replacing a frame while this thread
         * is reading it. */
        {
            std::scoped_lock bufferLock(mBufMutex, p_imuGrabber->mBufMutex);
            if (!synchronizedRgbdPacketBuffer.empty() &&
                !p_imuGrabber->imuBuf.empty())
            {
                const SynchronizedRgbdPacket &rgbdPacket =
                    synchronizedRgbdPacketBuffer.front();
                imageTimestamp_seconds =
                    rclcpp::Time(rgbdPacket.p_rgbImageMessage->header.stamp)
                        .seconds();
                latestImuTimestamp_seconds =
                    rclcpp::Time(p_imuGrabber->imuBuf.back()->header.stamp)
                        .seconds();
                waitingForImu =
                    imageTimestamp_seconds > latestImuTimestamp_seconds;
                if (!waitingForImu)
                {
                    p_rgbImageMessage   = rgbdPacket.p_rgbImageMessage;
                    p_depthImageMessage = rgbdPacket.p_depthImageMessage;
                    p_pointCloudMessage = rgbdPacket.p_pointCloudMessage;
                    synchronizedRgbdPacketBuffer.pop();
                    while (
                        !p_imuGrabber->imuBuf.empty() &&
                        rclcpp::Time(p_imuGrabber->imuBuf.front()->header.stamp)
                                .seconds() <= imageTimestamp_seconds)
                    {
                        const std::shared_ptr<const sensor_msgs::msg::Imu>
                            &p_imuMessage = p_imuGrabber->imuBuf.front();
                        const double imuTimestamp_seconds =
                            rclcpp::Time(p_imuMessage->header.stamp).seconds();
                        if (hasConsumedImuSample)
                        {
                            pendingMaximumImuGap_seconds =
                                std::max(pendingMaximumImuGap_seconds,
                                         imuTimestamp_seconds -
                                             lastConsumedImuTimestamp_seconds);
                        }
                        const cv::Point3f imuAcceleration_body_mPerSec2(
                            static_cast<float>(
                                p_imuMessage->linear_acceleration.x),
                            static_cast<float>(
                                p_imuMessage->linear_acceleration.y),
                            static_cast<float>(
                                p_imuMessage->linear_acceleration.z));
                        const cv::Point3f angularVelocity_body_radPerSecCv(
                            static_cast<float>(
                                p_imuMessage->angular_velocity.x),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.y),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.z));
                        constexpr std::size_t maximumPendingImuSamples = 2500U;
                        if (pendingImuMeasurements.size() >=
                            maximumPendingImuSamples)
                        {
                            pendingImuMeasurements.clear();
                            pendingMaximumImuGap_seconds = 0.0;
                            hasConsumedImuSample         = false;
                            imuContinuityOverflow        = true;
                        }
                        pendingImuMeasurements.emplace_back(
                            imuAcceleration_body_mPerSec2,
                            angularVelocity_body_radPerSecCv,
                            imuTimestamp_seconds);
                        angularVelocity_body_radPerSec << static_cast<float>(
                            p_imuMessage->angular_velocity.x),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.y),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.z);
                        p_imuGrabber->imuBuf.pop();
                        lastConsumedImuTimestamp_seconds = imuTimestamp_seconds;
                        hasConsumedImuSample             = true;
                    }
                    if (hasConsumedImuSample)
                    {
                        maximumImuGap_seconds =
                            std::max(pendingMaximumImuGap_seconds,
                                     imageTimestamp_seconds -
                                         lastConsumedImuTimestamp_seconds);
                    }
                    imuMeasurements = pendingImuMeasurements;
                }
            }
        }
        if (!p_rgbImageMessage)
        {
            if (waitingForImu)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for IMU data: RGB time %.3f is ahead of the "
                    "latest IMU time %.3f.",
                    imageTimestamp_seconds,
                    latestImuTimestamp_seconds);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            continue;
        }
        constexpr double maximumImuGapAllowed_seconds = 0.010;
        if (imuContinuityOverflow ||
            maximumImuGap_seconds > maximumImuGapAllowed_seconds)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding an inertial RGB-D frame: IMU continuity failed "
                "(gap %.4f seconds, %zu buffered samples).",
                maximumImuGap_seconds,
                imuMeasurements.size());
            pendingImuMeasurements.clear();
            pendingMaximumImuGap_seconds = 0.0;
            hasConsumedImuSample         = false;
            if (vs_graphs::common::p_slamSystem->requestResetActiveMapWithCause(
                    vs_graphs::core::reset_cause::ResetCause::
                        IMU_DELIVERY_GAP) !=
                vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: requestResetActiveMapWithCause returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            continue;
        }
        if (imuMeasurements.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "Retaining an RGB-D frame interval until IMU "
                                 "samples are available.");
            continue;
        }
        const char *processingStage = "image conversion";
        try
        {
            const rclcpp::Time messageTimestamp =
                p_rgbImageMessage->header.stamp;
            const cv::Mat rgbImage   = getImage(p_rgbImageMessage);
            const cv::Mat depthImage = getImage(p_depthImageMessage);
            if (rgbImage.empty() || depthImage.empty())
            {
                RCLCPP_ERROR_THROTTLE(get_logger(),
                                      *get_clock(),
                                      5000,
                                      "Discarding an RGB-D frame because image "
                                      "conversion returned an empty matrix.");
                continue;
            }
            processingStage = "point-cloud conversion";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud;
            sensor_msgs::msg::PointCloud2::ConstSharedPtr
                        p_pointCloudCameraMessage;
            std::string cloudFailureReason;
            if (!vs_graphs::common::preparePointCloudForTracking(
                    p_pointCloudMessage,
                    directGazeboFluCloud,
                    p_pointCloud,
                    p_pointCloudCameraMessage,
                    cloudFailureReason))
            {
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     5000,
                                     "Discarding point cloud: %s.",
                                     cloudFailureReason.c_str());
                continue;
            }
            processingStage = "marker association";
            std::pair<double, std::vector<vs_graphs::core::semantic::Marker *>>
                nearestMarker = vs_graphs::common::findNearestMarker(
                    imageTimestamp_seconds);
            const double markerTimeDifference_seconds = nearestMarker.first;
            std::vector<vs_graphs::core::semantic::Marker *> matchedMarkers =
                std::move(nearestMarker.second);
            processingStage = "inertial RGB-D tracking";
            if (markerTimeDifference_seconds < 0.05)
            {
                Sophus::SE3f slamSystemCameraPose{};
                if (vs_graphs::common::p_slamSystem->trackRGBD(
                        rgbImage,
                        depthImage,
                        p_pointCloud,
                        imageTimestamp_seconds,
                        slamSystemCameraPose,
                        imuMeasurements,
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
                vs_graphs::common::markersBuffer.clear();
            }
            else
            {
                Sophus::SE3f slamSystemCameraPose2{};
                if (vs_graphs::common::p_slamSystem->trackRGBD(
                        rgbImage,
                        depthImage,
                        p_pointCloud,
                        imageTimestamp_seconds,
                        slamSystemCameraPose2,
                        imuMeasurements) !=
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
            double estimatorInterval_seconds = 0.0;
            if (hasProcessedRgbdPacket)
            {
                const double processedFrameInterval_seconds =
                    imageTimestamp_seconds - lastProcessedRgbdTimestamp_seconds;
                estimatorInterval_seconds = processedFrameInterval_seconds;
                if (processedFrameInterval_seconds > 0.5)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Estimator input discontinuity: consecutive RGB-D "
                        "timestamps differ by %.3f seconds.",
                        processedFrameInterval_seconds);
                }
            }
            lastProcessedRgbdTimestamp_seconds = imageTimestamp_seconds;
            hasProcessedRgbdPacket             = true;
            vs_graphs::common::recordEstimatorFrame(estimatorInterval_seconds);

            /* Only a completed TrackRGBD call commits this IMU interval. A
             * rejected image or cloud therefore leaves every sample available
             * for the next accepted visual frame. */
            pendingImuMeasurements.clear();
            pendingMaximumImuGap_seconds = 0.0;

            processingStage = "ROS publication";
            vs_graphs::common::publishTopics(messageTimestamp,
                                             angularVelocity_body_radPerSec,
                                             p_pointCloudCameraMessage);
        }
        catch (const std::exception &exception)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding a synchronized sensor frame after "
                "an exception during %s: %s",
                processingStage,
                exception.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
} /* namespace vs_graphs::image_grabber */
