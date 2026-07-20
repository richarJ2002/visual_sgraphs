/*!
 * @File:         publishBodyOdometry.cpp
 *
 * @Brief:        Publishes the estimated body pose and velocity as a ROS
 *                odometry message.
 *
 * @Date:         20/07/2026
 *
 */

/* Function Includes */
#include "Common.hpp"

/* Object Includes */
#include "System.h"

/* Data Includes */
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <sophus/se3.hpp>

/* ROS Includes */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

/* Generic Libraries */
/* None */

void publishBodyOdometry(const Sophus::SE3f    &robotPose_BodToWorld_in,
                         const Eigen::Vector3f &linearVelocity_World_mps_in,
                         const Eigen::Vector3f &angularVelocity_Bod_radps_in,
                         const rclcpp::Time    &msgTims_s_in)
{
    /* Confirm that the odometry publisher has been initialised */
    if (pubOdometry == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish body odometry: publisher is not initialised.");

        return;
    }

    /* Extract the body position and orientation once */
    const Eigen::Vector3f position_World_m =
        robotPose_BodToWorld_in.translation();
    const Eigen::Quaternionf orientation_WorldToBod =
        robotPose_BodToWorld_in.unit_quaternion();

    /* Initialise the odometry message */
    nav_msgs::msg::Odometry odometryMessage;

    odometryMessage.header.stamp    = msgTims_s_in;
    odometryMessage.header.frame_id = frameWorld;
    odometryMessage.child_frame_id  = frameImu;

    /* Set the body position in the world frame */
    odometryMessage.pose.pose.position.x = position_World_m.x();
    odometryMessage.pose.pose.position.y = position_World_m.y();
    odometryMessage.pose.pose.position.z = position_World_m.z();

    /* Set the body orientation relative to the world frame */
    odometryMessage.pose.pose.orientation.x = orientation_WorldToBod.x();
    odometryMessage.pose.pose.orientation.y = orientation_WorldToBod.y();
    odometryMessage.pose.pose.orientation.z = orientation_WorldToBod.z();
    odometryMessage.pose.pose.orientation.w = orientation_WorldToBod.w();

    /* Set the body linear velocity */
    odometryMessage.twist.twist.linear.x = linearVelocity_World_mps_in.x();
    odometryMessage.twist.twist.linear.y = linearVelocity_World_mps_in.y();
    odometryMessage.twist.twist.linear.z = linearVelocity_World_mps_in.z();

    /* Set the body angular velocity */
    odometryMessage.twist.twist.angular.x = angularVelocity_Bod_radps_in.x();
    odometryMessage.twist.twist.angular.y = angularVelocity_Bod_radps_in.y();
    odometryMessage.twist.twist.angular.z = angularVelocity_Bod_radps_in.z();

    /* Publish the completed odometry message */
    pubOdometry->publish(odometryMessage);
}
