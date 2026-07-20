/*!
 * @File:         mapPointToPointcloud.cpp
 *
 * @Brief:        Converts a vector of MapPoints to a PointCloud2 message
 *
 * @Date:         20/07/2026
 *
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

/* Function Includes */
#include "Common.hpp"

/* Object Includes */
#include "MapPoint.h"

/* Data Includes */
#include <sensor_msgs/msg/point_cloud2.hpp>

/* ROS Includes */
#include <rclcpp/time.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

/* Generic Libraries */
/* None */

sensor_msgs::msg::PointCloud2
    mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints_in,
                         rclcpp::Time                       msgTime_in)
{
    const int                     numChannels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    std::string                   channelId[] = {"x", "y", "z"};

    /* Set the attributes of the point cloud */
    cloud.header.stamp    = msgTime_in;
    cloud.header.frame_id = frameWorld;
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.is_bigendian    = false;
    cloud.width           = mapPoints_in.size();
    cloud.point_step      = numChannels * sizeof(float);
    cloud.row_step        = cloud.point_step * cloud.width;
    cloud.fields.resize(numChannels);

    // Set the fields of the point cloud
    for (int idx = 0; idx < numChannels; idx++)
    {
        cloud.fields[idx].count    = 1;
        cloud.fields[idx].name     = channelId[idx];
        cloud.fields[idx].offset   = idx * sizeof(float);
        cloud.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    // Set the data of the point cloud
    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloudDataPtr = &(cloud.data[0]);

    // Populate the point cloud with the map points
    for (unsigned int idx = 0; idx < cloud.width; idx++)
    {
        if (mapPoints_in[idx] && !mapPoints_in[idx]->isBad())
        {
            Eigen::Vector3d P3Dw =
                mapPoints_in[idx]->GetWorldPos().cast<double>();
            tf2::Vector3 pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());
            float        dataArray[numChannels] = {
                static_cast<float>(pointTranslation.x()),
                static_cast<float>(pointTranslation.y()),
                static_cast<float>(pointTranslation.z())};
            memcpy(cloudDataPtr + (idx * cloud.point_step),
                   dataArray,
                   numChannels * sizeof(float));
        }
    }

    return cloud;
}
