/*!
 * @file            grabRgbd.cc
 *
 * @brief           Admits ordered RGB-D packets for the IMU worker.
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

void ImageGrabber::grabRgbd(
    const sensor_msgs::msg::Image::ConstSharedPtr &p_rgbImageMessage_in,
    const sensor_msgs::msg::Image::ConstSharedPtr &p_depthImageMessage_in)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (!p_latestPointCloudMessage)
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Waiting for the first point cloud.");
        return;
    }
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage =
        p_latestPointCloudMessage;
    if (discardInputUntilBufferDrained)
    {
        if (!synchronizedRgbdPacketBuffer.empty())
        {
            return;
        }

        {
            std::lock_guard<std::mutex> imuBufferLock(p_imuGrabber->mBufMutex);
            std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> emptyImuBuffer;
            p_imuGrabber->imuBuf.swap(emptyImuBuffer);
        }
        if (vs_graphs::common::p_slamSystem->requestResetActiveMapWithCause(
                vs_graphs::core::reset_cause::ResetCause::
                    SENSOR_PROCESSING_OVERLOAD) !=
            vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: requestResetActiveMapWithCause returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        discardInputUntilBufferDrained = false;
        hasAdmittedRgbdPacket          = false;

        RCLCPP_WARN(get_logger(),
                    "Restarting the active map after a sustained sensor "
                    "processing overload.");
        return;
    }
    const double rgbTimestamp_seconds =
        rclcpp::Time(p_rgbImageMessage_in->header.stamp).seconds();
    const double depthTimestamp_seconds =
        rclcpp::Time(p_depthImageMessage_in->header.stamp).seconds();
    const double pointCloudTimestamp_seconds =
        rclcpp::Time(p_pointCloudMessage->header.stamp).seconds();
    if (hasReceivedRgbdPacket)
    {
        const double receivedFrameInterval_seconds =
            rgbTimestamp_seconds - lastReceivedRgbdTimestamp_seconds;
        if (receivedFrameInterval_seconds > 0.5)
        {
            RCLCPP_WARN(get_logger(),
                        "RGB-D synchronizer input discontinuity: consecutive "
                        "image pairs differ by %.3f seconds.",
                        receivedFrameInterval_seconds);
        }
    }
    lastReceivedRgbdTimestamp_seconds                  = rgbTimestamp_seconds;
    hasReceivedRgbdPacket                              = true;
    constexpr double maximumImageTimestampSkew_seconds = 0.010;
    if (std::abs(rgbTimestamp_seconds - depthTimestamp_seconds) >
        maximumImageTimestampSkew_seconds)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D image pair whose timestamps differ by "
            "%.3f seconds.",
            std::abs(rgbTimestamp_seconds - depthTimestamp_seconds));
        return;
    }
    constexpr double maximumPointCloudAge_seconds = 0.033;
    const double     cloudImageSkew_seconds       = std::max(
        std::abs(rgbTimestamp_seconds - pointCloudTimestamp_seconds),
        std::abs(depthTimestamp_seconds - pointCloudTimestamp_seconds));
    if (cloudImageSkew_seconds > maximumPointCloudAge_seconds)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding a point cloud %.3f seconds from the RGB-D image time.",
            cloudImageSkew_seconds);
        return;
    }
    if (hasAdmittedRgbdPacket &&
        rgbTimestamp_seconds <= lastAdmittedRgbdTimestamp_seconds)
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic RGB-D packet.");
        return;
    }

    /*!
     * Gazebo publishes a 30 Hz sensor on discrete simulation steps, so a
     * nominal 33.333 ms period can alternate between 33 and 34 ms. Admit that
     * bounded quantization without allowing a genuinely faster stream through
     * the configured tracking-rate limit.
     */
    const double trackingIntervalTolerance_seconds =
        std::min(1e-3, 0.05 * minimumTrackingInterval_seconds);
    if (hasAdmittedRgbdPacket && rgbTimestamp_seconds -
                                         lastAdmittedRgbdTimestamp_seconds +
                                         trackingIntervalTolerance_seconds <
                                     minimumTrackingInterval_seconds)
    {
        return;
    }

    /* Buffer several consecutive frames so expensive first-map construction
     * cannot turn normal camera traffic into an artificial timestamp jump.
     * New frames are rejected at the latency boundary so already-admitted
     * estimator input remains chronological and gap-free.
     */
    if (synchronizedRgbdPacketBuffer.size() >= maximumBufferedRgbdPackets)
    {
        discardInputUntilBufferDrained = true;
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "RGB-D processing exceeded the %.1f-second latency budget; "
            "discarding new packets while the estimator catches up.",
            static_cast<double>(maximumBufferedRgbdPackets) *
                minimumTrackingInterval_seconds);
        return;
    }
    synchronizedRgbdPacketBuffer.push(
        {p_rgbImageMessage_in, p_depthImageMessage_in, p_pointCloudMessage});
    lastAdmittedRgbdTimestamp_seconds = rgbTimestamp_seconds;
    hasAdmittedRgbdPacket             = true;
}
} /* namespace vs_graphs::image_grabber */
