/*!
 * @File:         publishTrackedPoints.cpp
 *
 * @Brief:        Converts the currently tracked ORB-SLAM3 map points into a ROS
 *                point-cloud message and publishes the result.
 *
 * @Date:         20/07/2026
 *
 */

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void publishTrackedPoints(
    const std::vector<ORB_SLAM3::MapPoint *> &trackedMapPoints_in,
    const rclcpp::Time                       &msgTime_s_in)
{
    /* Confirm that the tracked-map-point publisher is available */
    if (pubTrackedMappoints == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish tracked map points: publisher is not initialised.");

        return;
    }

    /* Convert the tracked map points into a ROS point-cloud message */
    sensor_msgs::msg::PointCloud2 trackedMapPointCloudMessage =
        mapPointToPointcloud(trackedMapPoints_in, msgTime_s_in);

    /*!
     * Explicitly set the output metadata in case the conversion function does
     * not set it, or to ensure it remains consistent with this publisher.
     */
    trackedMapPointCloudMessage.header.stamp    = msgTime_s_in;
    trackedMapPointCloudMessage.header.frame_id = frameWorld;

    /*!
     * Publish empty clouds as well. This clears previously displayed tracked
     * points when ORB-SLAM3 is no longer tracking any map points.
     */
    pubTrackedMappoints->publish(trackedMapPointCloudMessage);
}
