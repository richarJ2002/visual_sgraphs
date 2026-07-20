/*!
 * @File:         publishFramePointCloud.cpp
 *
 * @Brief:        Transforms an input point cloud from the camera frame into the
 *                world frame and publishes the transformed cloud.
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

void publishFramePointCloud(const Sophus::SE3f &cameraPose_CameraToWorld_in,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                                               &pointCloudCameraMessage_in,
                            const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the point-cloud publisher has been initialised */
    if (pubWorldFramePointCloud == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish world-frame point cloud: publisher is not "
                    "initialised.");

        return;
    }

    /* Confirm that a valid input point-cloud message was supplied */
    if (pointCloudCameraMessage_in == nullptr)
    {
        return;
    }

    /* Confirm that the camera transformation is valid */
    if (!cameraPose_CameraToWorld_in.translation().allFinite() ||
        !cameraPose_CameraToWorld_in.rotationMatrix().allFinite())
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish world-frame point cloud: camera pose contains "
            "non-finite values.");

        return;
    }

    /* Convert the ROS message into a camera-frame PCL point cloud */
    pcl::PointCloud<pcl::PointXYZRGBA> pointCloud_camera;

    pcl::fromROSMsg(*pointCloudCameraMessage_in, pointCloud_camera);

    /* Return when the input point cloud contains no points */
    if (pointCloud_camera.empty())
    {
        return;
    }

    /* Convert the camera-to-world transformation into a PCL-compatible matrix
     */
    const Eigen::Matrix4f T_world_camera_matrix =
        cameraPose_CameraToWorld_in.matrix();

    /* Transform the complete point cloud into the world frame */
    pcl::PointCloud<pcl::PointXYZRGBA> pointCloud_world;

    pcl::transformPointCloud(pointCloud_camera,
                             pointCloud_world,
                             T_world_camera_matrix);

    /* Convert the transformed cloud back into a ROS message */
    sensor_msgs::msg::PointCloud2 pointCloud_worldMessage;

    pcl::toROSMsg(pointCloud_world, pointCloud_worldMessage);

    /* Set the output message metadata */
    pointCloud_worldMessage.header.stamp = msgTime_s_in;

    pointCloud_worldMessage.header.frame_id = frameWorld;

    /* Publish the transformed point cloud */
    pubWorldFramePointCloud->publish(pointCloud_worldMessage);
}
