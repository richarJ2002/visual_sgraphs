/*!
 * @File:         getRoomDisplayPoints.cpp
 *
 * @Brief:        Calculates the displayed structural-graph position of a room.
 *
 * @Date:         20/07/2026
 *
 */

#include <cmath>
#include <string>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
#include "Semantic/Room.h"

/* Data include */
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

/* ROS Includes */
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

/* Generic Libraries */
/* None */

bool getRoomDisplayPoints(ORB_SLAM3::Room                  *room_in,
                          const rclcpp::Time               &msgTime_in,
                          geometry_msgs::msg::PointStamped &roomPointSE_out,
                          geometry_msgs::msg::PointStamped &roomPointWorld_out)
{
    /* Confirm the required objects are valid */
    if (room_in == nullptr || tfBuffer_ == nullptr)
    {
        return false;
    }

    /* The room centroid is stored in the world frame */
    const Eigen::Vector3d roomCentroid = room_in->getCentroid();

    if (!roomCentroid.allFinite())
    {
        return false;
    }

    /* Create the physical room point in the world frame */
    roomPointWorld_out.header.stamp    = msgTime_in;
    roomPointWorld_out.header.frame_id = frameWorld;
    roomPointWorld_out.point.x         = roomCentroid.x();
    roomPointWorld_out.point.y         = roomCentroid.y();
    roomPointWorld_out.point.z         = roomCentroid.z();

    try
    {
        /* Transform the world-frame centroid into frameSE */
        const geometry_msgs::msg::TransformStamped worldToSE =
            tfBuffer_->lookupTransform(frameSE,
                                       frameWorld,
                                       msgTime_in,
                                       rclcpp::Duration::from_seconds(0.1));

        tf2::doTransform(roomPointWorld_out, roomPointSE_out, worldToSE);
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Room display transform from '%s' to '%s' failed: %s",
                    frameWorld.c_str(),
                    frameSE.c_str(),
                    exception.what());

        return false;
    }

    roomPointSE_out.header.stamp    = msgTime_in;
    roomPointSE_out.header.frame_id = frameSE;

    return true;
}
