/*!
 * @File:         publishTrackingImage.cpp
 *
 * @Brief:        Converts the current ORB-SLAM3 tracking image into a ROS image
 *                message and publishes it using the tracking-image publisher.
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

void publishTrackingImage(const cv::Mat      &trackingImage_bgr8_in,
                          const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the image publisher has been initialised */
    if (pubTrackingImage == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish tracking image: publisher is not initialised.");

        return;
    }

    /* Return when ORB-SLAM3 has not supplied a valid image */
    if (trackingImage_bgr8_in.empty())
    {
        return;
    }

    /*!
     * The current publication pipeline expects a three-channel, unsigned
     * 8-bit BGR image.
     */
    if (trackingImage_bgr8_in.type() != CV_8UC3)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish tracking image: expected CV_8UC3 but received "
            "OpenCV type %d.",
            trackingImage_bgr8_in.type());

        return;
    }

    /* Initialise the ROS image header */
    std_msgs::msg::Header imageHeader;

    imageHeader.stamp = msgTime_s_in;

    /*!
     * The message contains a camera image, so its frame is the camera rather
     * than the world frame.
     */
    imageHeader.frame_id = frameCamera;

    /* Convert the OpenCV image into a ROS image message */
    const sensor_msgs::msg::Image::SharedPtr trackingImageMessage =
        cv_bridge::CvImage(imageHeader, "bgr8", trackingImage_bgr8_in)
            .toImageMsg();

    if (trackingImageMessage == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Failed to convert the tracking image into a ROS image message.");

        return;
    }

    /* Publish through the image_transport publisher */
    pubTrackingImage->publish(trackingImageMessage);
}