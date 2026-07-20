/*!
 * @File:         publishStaticTFTransform.cpp
 *
 * @Brief:        Publishes a static transformation between two ROS coordinate
 *                frames.
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

void publishStaticTFTransform(const std::string  &parentFrameId_in,
                              const std::string  &childFrameId_in,
                              const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the static-transform broadcaster has been initialised */
    if (staticTfBroadcaster == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish static transform: broadcaster is not initialised.");

        return;
    }

    /* Confirm that valid frame identifiers were supplied */
    if (parentFrameId_in.empty() || childFrameId_in.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish static transform: frame identifier is empty.");

        return;
    }

    /* Prevent a frame from being defined as its own child */
    if (parentFrameId_in == childFrameId_in)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish static transform: parent and child frames are "
            "identical.");

        return;
    }

    /* Calculate the child-frame orientation relative to the parent frame */
    tf2::Quaternion orientation_parent_child;

    orientation_parent_child.setRPY(roll, pitch, yaw);
    orientation_parent_child.normalize();

    /* Initialise the static transformation message */
    geometry_msgs::msg::TransformStamped staticTransformMessage;

    staticTransformMessage.header.stamp    = msgTime_s_in;
    staticTransformMessage.header.frame_id = parentFrameId_in;
    staticTransformMessage.child_frame_id  = childFrameId_in;

    /* The child and parent frames have no relative translation */
    staticTransformMessage.transform.translation.x = 0.0;
    staticTransformMessage.transform.translation.y = 0.0;
    staticTransformMessage.transform.translation.z = 0.0;

    /* Set the child-frame orientation relative to the parent frame */
    staticTransformMessage.transform.rotation.x = orientation_parent_child.x();
    staticTransformMessage.transform.rotation.y = orientation_parent_child.y();
    staticTransformMessage.transform.rotation.z = orientation_parent_child.z();
    staticTransformMessage.transform.rotation.w = orientation_parent_child.w();

    /* Publish the completed static transformation */
    staticTfBroadcaster->sendTransform(staticTransformMessage);
}
