/*!
 * @File:         publishCameraPose.cpp
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

void publishCameraPose(const Sophus::SE3f &cameraPose_World_in,
                       const rclcpp::Time &msgTime_s_in)
{
    /* Extract the camera position and orientation once */
    const Eigen::Vector3f cameraPosition_world_m =
        cameraPose_World_in.translation();
    const Eigen::Quaternionf cameraOrientation_world =
        cameraPose_World_in.unit_quaternion();

    /* Initialise the camera-pose message */
    geometry_msgs::msg::PoseStamped cameraPoseMessage;

    cameraPoseMessage.header.frame_id = frameWorld;
    cameraPoseMessage.header.stamp    = msgTime_s_in;

    /* Set the camera position in the world frame */
    cameraPoseMessage.pose.position.x = cameraPosition_world_m.x();
    cameraPoseMessage.pose.position.y = cameraPosition_world_m.y();
    cameraPoseMessage.pose.position.z = cameraPosition_world_m.z();

    /* Set the camera orientation relative to the world frame */
    cameraPoseMessage.pose.orientation.x = cameraOrientation_world.x();
    cameraPoseMessage.pose.orientation.y = cameraOrientation_world.y();
    cameraPoseMessage.pose.orientation.z = cameraOrientation_world.z();
    cameraPoseMessage.pose.orientation.w = cameraOrientation_world.w();

    /* Publish the camera pose when the publisher is available */
    if (pubCameraPose != nullptr)
    {
        pubCameraPose->publish(cameraPoseMessage);
    }
    else
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish camera pose: publisher is not initialised.");
    }

    /* Initialise the camera visualisation marker */
    visualization_msgs::msg::Marker cameraMarker;

    cameraMarker.header.frame_id = frameWorld;
    cameraMarker.header.stamp    = msgTime_s_in;
    cameraMarker.ns              = "camera_pose";
    cameraMarker.id              = 1;
    cameraMarker.action          = visualization_msgs::msg::Marker::ADD;
    cameraMarker.type          = visualization_msgs::msg::Marker::MESH_RESOURCE;
    cameraMarker.mesh_resource = "package://vs_graphs/config/Assets/camera.dae";
    cameraMarker.mesh_use_embedded_materials = true;

    /*!
     * Reuse the pose message so the pose marker and published camera pose
     * always contain identical position and orientation values.
     */
    cameraMarker.pose = cameraPoseMessage.pose;

    cameraMarker.scale.x = 0.5;
    cameraMarker.scale.y = 0.5;
    cameraMarker.scale.z = 0.5;

    cameraMarker.color.a = 0.7;

    cameraMarker.lifetime = rclcpp::Duration::from_seconds(0);

    /* Add the camera marker to its marker array */
    visualization_msgs::msg::MarkerArray cameraMarkerArray;

    cameraMarkerArray.markers.reserve(1);
    cameraMarkerArray.markers.push_back(std::move(cameraMarker));

    /* Publish the camera visualisation */
    if (pubCameraPoseVis != nullptr)
    {
        pubCameraPoseVis->publish(cameraMarkerArray);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish camera visualisation: publisher is not "
                    "initialised.");
    }
}
