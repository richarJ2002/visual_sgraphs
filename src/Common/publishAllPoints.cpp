/*!
 * @File:         publishAllPoints.cpp
 *
 * @Brief:        Calls mapPointToPointscloud() function and then publishes the
 *                point cloud through the pubAllMappoints publisher.
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

void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> allMapPoints_in,
                      rclcpp::Time                       msgTime_s_in)
{
    /* Map point cloud */
    sensor_msgs::msg::PointCloud2 cloud =
        mapPointToPointcloud(allMapPoints_in, msgTime_s_in);

    /* Publish point cloud */
    pubAllMappoints->publish(cloud);
}
