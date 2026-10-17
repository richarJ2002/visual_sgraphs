/*!
 * @file            getImuSample.cc
 *
 * @brief           Buffers timestamp-ordered IMU samples with a fixed bound.
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

#include "imu_grabber/objects/ImuGrabberClass.h"

namespace vs_graphs::imu_grabber
{

void ImuGrabber::getImuSample(
    const sensor_msgs::msg::Imu::ConstSharedPtr &p_imuMessage_in)
{
    std::lock_guard<std::mutex> lock(mBufMutex);

    if (!imuBuf.empty() && rclcpp::Time(p_imuMessage_in->header.stamp) <=
                               rclcpp::Time(imuBuf.back()->header.stamp))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic IMU sample.");
        return;
    }

    /* Constant which indicates the maximum number of imu samples in buffer*/
    constexpr std::size_t maximumBufferedImuSamples = 2500;

    /* If IMU buffer is overflowing, remove oldest sample and output warning */
    if (imuBuf.size() >= maximumBufferedImuSamples)
    {
        imuBuf.pop();
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "IMU buffer overflow; discarding oldest sample.");
    }

    /* Inser IMU sample */
    imuBuf.push(p_imuMessage_in);
}
} /* namespace vs_graphs::imu_grabber */
