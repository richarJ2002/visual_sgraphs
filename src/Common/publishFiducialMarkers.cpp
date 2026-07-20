/*!
 * @File:         publishFiducialMarkers.cpp
 *
 * @Brief:        Publishes the estimated camera pose as a ROS pose message and
 *                as a camera mesh marker for visualisation.
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

void publishFiducialMarkers(
    const std::vector<ORB_SLAM3::Marker *> &fiducialMarkers_in,
    const rclcpp::Time                     &msgTime_s_in)
{
    /* Confirm that the marker publisher has been initialised */
    if (pubFiducialMarker == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish fiducial markers: publisher is not initialised.");

        return;
    }

    /* Return when there are no fiducial markers to publish */
    if (fiducialMarkers_in.empty())
    {
        return;
    }

    /* Initialise the output marker array */
    visualization_msgs::msg::MarkerArray fiducialMarkerArray;

    /*!
     * Reserve capacity without creating empty marker messages.
     *
     * resize() must not be used here because the markers are added using
     * push_back().
     */
    fiducialMarkerArray.markers.reserve(fiducialMarkers_in.size());

    /* Create one visualisation marker for every valid mapped marker */
    for (ORB_SLAM3::Marker *fiducialMarker : fiducialMarkers_in)
    {
        /* Skip invalid marker pointers */
        if (fiducialMarker == nullptr)
        {
            continue;
        }

        /* Extract the globally expressed fiducial-marker pose */
        const Sophus::SE3f T_world_fiducial_SE3f =
            fiducialMarker->getGlobalPose();

        /* Skip invalid poses */
        if (!T_world_fiducial_SE3f.translation().allFinite() ||
            !T_world_fiducial_SE3f.rotationMatrix().allFinite())
        {
            continue;
        }

        /* Extract the pose components once */
        const Eigen::Vector3f fiducialPosition_world_m =
            T_world_fiducial_SE3f.translation();

        const Eigen::Quaternionf fiducialOrientation_world =
            T_world_fiducial_SE3f.unit_quaternion();

        /* Initialise the visualisation marker */
        visualization_msgs::msg::Marker fiducialMarkerMessage;

        fiducialMarkerMessage.header.frame_id = frameWorld;
        fiducialMarkerMessage.header.stamp    = msgTime_s_in;
        fiducialMarkerMessage.ns              = "fiducial_markers";

        /*!
         * Use the persistent semantic marker ID rather than the current array
         * position. This keeps the RViz marker identity stable when marker
         * ordering changes.
         */
        fiducialMarkerMessage.id     = fiducialMarker->getId();
        fiducialMarkerMessage.action = visualization_msgs::msg::Marker::ADD;

        fiducialMarkerMessage.type =
            visualization_msgs::msg::Marker::MESH_RESOURCE;

        fiducialMarkerMessage.mesh_resource =
            "package://vs_graphs/config/Assets/aruco_marker.dae";

        fiducialMarkerMessage.mesh_use_embedded_materials = true;

        /* Set the marker position in the world frame */
        fiducialMarkerMessage.pose.position.x = fiducialPosition_world_m.x();
        fiducialMarkerMessage.pose.position.y = fiducialPosition_world_m.y();
        fiducialMarkerMessage.pose.position.z = fiducialPosition_world_m.z();

        /* Set the marker orientation relative to the world frame */
        fiducialMarkerMessage.pose.orientation.x =
            fiducialOrientation_world.x();

        fiducialMarkerMessage.pose.orientation.y =
            fiducialOrientation_world.y();

        fiducialMarkerMessage.pose.orientation.z =
            fiducialOrientation_world.z();

        fiducialMarkerMessage.pose.orientation.w =
            fiducialOrientation_world.w();

        /* Set the displayed mesh size */
        fiducialMarkerMessage.scale.x = 0.2;
        fiducialMarkerMessage.scale.y = 0.2;
        fiducialMarkerMessage.scale.z = 0.2;

        /*!
         * Keep the marker fully visible. The embedded mesh materials determine
         * its displayed colour.
         */
        fiducialMarkerMessage.color.a = 1.0;

        /* Keep the marker visible until it is replaced or deleted */
        fiducialMarkerMessage.lifetime = rclcpp::Duration::from_seconds(0);

        /* Add the completed marker to the output array */
        fiducialMarkerArray.markers.push_back(std::move(fiducialMarkerMessage));
    }

    /* Publish only when at least one valid marker was generated */
    if (!fiducialMarkerArray.markers.empty())
    {
        pubFiducialMarker->publish(fiducialMarkerArray);
    }
}
