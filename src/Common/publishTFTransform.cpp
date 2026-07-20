/*!
 * @File:         publishTFTransform.cpp
 *
 * @Brief:
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

void publishTFTransform(const Sophus::SE3f &transform_ParentToChild_in,
                        const std::string  &parentFrameId_in,
                        const std::string  &childFrameId_in,
                        const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the dynamic-transform broadcaster is available */
    if (tfBroadcaster == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish TF transform: broadcaster is not initialised.");

        return;
    }

    /* Confirm that valid frame identifiers were supplied */
    if (parentFrameId_in.empty() || childFrameId_in.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish TF transform: frame identifier is empty.");

        return;
    }

    /* Prevent a frame from being published as its own child */
    if (parentFrameId_in == childFrameId_in)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish TF transform: parent and child frames are "
                     "identical.");

        return;
    }

    /* Confirm that the supplied transformation contains valid values */
    if (!transform_ParentToChild_in.translation().allFinite() ||
        !transform_ParentToChild_in.rotationMatrix().allFinite())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish TF transform: transformation contains "
                     "non-finite values.");

        return;
    }

    /* Extract the translation and orientation once */
    const Eigen::Vector3f translation_parent_child_m =
        transform_ParentToChild_in.translation();

    Eigen::Quaternionf orientation_parent_child =
        transform_ParentToChild_in.unit_quaternion();

    orientation_parent_child.normalize();

    /* Initialise the dynamic transformation message */
    geometry_msgs::msg::TransformStamped transformMessage;

    transformMessage.header.stamp    = msgTime_s_in;
    transformMessage.header.frame_id = parentFrameId_in;
    transformMessage.child_frame_id  = childFrameId_in;

    /* Set the child-frame translation relative to the parent frame */
    transformMessage.transform.translation.x = translation_parent_child_m.x();
    transformMessage.transform.translation.y = translation_parent_child_m.y();
    transformMessage.transform.translation.z = translation_parent_child_m.z();

    /* Set the child-frame orientation relative to the parent frame */
    transformMessage.transform.rotation.x = orientation_parent_child.x();
    transformMessage.transform.rotation.y = orientation_parent_child.y();
    transformMessage.transform.rotation.z = orientation_parent_child.z();
    transformMessage.transform.rotation.w = orientation_parent_child.w();

    /* Broadcast the completed dynamic transformation */
    tfBroadcaster->sendTransform(transformMessage);
}
