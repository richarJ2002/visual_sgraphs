/*!
 * @File:         publishKeyFrameImages.cpp
 *
 * @Brief:        Publishes images from keyframes that have not yet been sent
 * for semantic segmentation.
 *
 * @Date:         20/07/2026
 *
 */

#include <vector>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void publishKeyFrameImages(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                       &msgTime_s_in)
{
    /* Confirm that the keyframe-image publisher has been initialised */
    if (pubKFImage == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish keyframe images: publisher is not initialised.");

        return;
    }

    /* Inspect every available keyframe */
    for (ORB_SLAM3::KeyFrame *keyFrame : keyFrames_in)
    {
        /* Skip invalid keyframe pointers */
        if (keyFrame == nullptr)
        {
            continue;
        }

        /* Skip keyframes that have already been published */
        if (keyFrame->isPublished)
        {
            continue;
        }

        /* Skip keyframes that do not contain a valid image */
        if (keyFrame->mImage.empty())
        {
            continue;
        }

        /* Initialise the common ROS message header */
        std_msgs::msg::Header messageHeader;

        messageHeader.stamp = msgTime_s_in;

        messageHeader.frame_id = frameWorld;

        /* Convert the OpenCV keyframe image into a ROS image message */
        const sensor_msgs::msg::Image::SharedPtr keyFrameImageMessage =
            cv_bridge::CvImage(messageHeader, "bgr8", keyFrame->mImage)
                .toImageMsg();

        /* Confirm that the image conversion succeeded */
        if (keyFrameImageMessage == nullptr)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Failed to convert KeyFrame#%lu image into a ROS message.",
                static_cast<unsigned long>(keyFrame->mnId));

            continue;
        }

        /* Create the persistent keyframe identifier message */
        std_msgs::msg::UInt64 keyFrameIdMessage;

        keyFrameIdMessage.data = keyFrame->mnId;

        /* Package the keyframe identifier and image for segmentation */
        segmenter_ros::msg::VSGraphDataMsg segmentationInputMessage;

        segmentationInputMessage.header          = messageHeader;
        segmentationInputMessage.key_frame_id    = keyFrameIdMessage;
        segmentationInputMessage.key_frame_image = *keyFrameImageMessage;

        /* Publish the keyframe image for semantic segmentation */
        pubKFImage->publish(segmentationInputMessage);

        /* Prevent the same keyframe from being published again */
        keyFrame->isPublished = true;
    }
}
