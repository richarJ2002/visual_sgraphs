/*!
 * @file            grabPointCloud.cc
 *
 * @brief           Retains the newest cloud for inertial RGB-D pairing.
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
void ImageGrabber::grabPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &p_pointCloudMessage_in)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (p_latestPointCloudMessage &&
        rclcpp::Time(p_pointCloudMessage_in->header.stamp) <=
            rclcpp::Time(p_latestPointCloudMessage->header.stamp))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic point cloud.");
        return;
    }

    p_latestPointCloudMessage = p_pointCloudMessage_in;
}

} /* namespace vs_graphs::image_grabber */
