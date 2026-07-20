/*!
 * @File:         publishKeyFrameMarkers.cpp
 *
 * @Brief:        Publishes the positions of all valid keyframes as an RViz
 *                sphere-list marker and publishes their ordered poses as a ROS
 *                path.
 *
 * @Date:         20/07/2026
 *
 */

#include <cstdint>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void publishKeyFrameMarkers(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                       &messageTimestamp_in)
{
    /* Return when neither output publisher has been initialised */
    if (pubKeyFrameMarker == nullptr && pubKeyFrameList == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish keyframes: publishers are not initialised.");

        return;
    }

    /* Return when there are no keyframes to publish */
    if (keyFrames_in.empty())
    {
        return;
    }

    /*!
     * Create a local collection because the input collection is const and
     * should not be reordered by the publishing function.
     */
    std::vector<ORB_SLAM3::KeyFrame *> orderedKeyFrames;

    orderedKeyFrames.reserve(keyFrames_in.size());

    /* Remove invalid keyframes before sorting */
    for (ORB_SLAM3::KeyFrame *keyFrame : keyFrames_in)
    {
        if (keyFrame == nullptr || keyFrame->isBad())
        {
            continue;
        }

        orderedKeyFrames.push_back(keyFrame);
    }

    if (orderedKeyFrames.empty())
    {
        return;
    }

    /* Order keyframes using their persistent identifiers */
    std::sort(orderedKeyFrames.begin(),
              orderedKeyFrames.end(),
              ORB_SLAM3::KeyFrame::lId);

    /* Initialise the keyframe-position marker */
    visualization_msgs::msg::Marker keyFramePositionMarker;

    keyFramePositionMarker.header.frame_id = frameWorld;
    keyFramePositionMarker.header.stamp    = messageTimestamp_in;

    keyFramePositionMarker.ns   = "keyframe_positions";
    keyFramePositionMarker.id   = 0;
    keyFramePositionMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    keyFramePositionMarker.action = visualization_msgs::msg::Marker::ADD;

    keyFramePositionMarker.pose.orientation.x = 0.0;
    keyFramePositionMarker.pose.orientation.y = 0.0;
    keyFramePositionMarker.pose.orientation.z = 0.0;
    keyFramePositionMarker.pose.orientation.w = 1.0;

    keyFramePositionMarker.scale.x = 0.05;
    keyFramePositionMarker.scale.y = 0.05;
    keyFramePositionMarker.scale.z = 0.05;

    keyFramePositionMarker.color.r = 0.0;
    keyFramePositionMarker.color.g = 1.0;
    keyFramePositionMarker.color.b = 0.0;
    keyFramePositionMarker.color.a = 1.0;

    keyFramePositionMarker.lifetime = rclcpp::Duration::from_seconds(0);
    keyFramePositionMarker.points.reserve(orderedKeyFrames.size());

    /* Initialise the ordered keyframe path */
    nav_msgs::msg::Path keyFramePathMessage;

    keyFramePathMessage.header.frame_id = frameWorld;
    keyFramePathMessage.header.stamp    = messageTimestamp_in;
    keyFramePathMessage.poses.reserve(orderedKeyFrames.size());

    /* Add every valid keyframe pose to the marker and path */
    for (ORB_SLAM3::KeyFrame *keyFrame : orderedKeyFrames)
    {
        /* Obtain the globally expressed keyframe pose */
        const Sophus::SE3f T_world_keyFrame_SE3f =
            pSLAM->GetKeyFramePose(keyFrame);

        /* Reject invalid poses */
        if (!T_world_keyFrame_SE3f.translation().allFinite() ||
            !T_world_keyFrame_SE3f.rotationMatrix().allFinite())
        {
            continue;
        }

        /* Extract the pose components once */
        const Eigen::Vector3f keyFramePosition_world_m =
            T_world_keyFrame_SE3f.translation();

        const Eigen::Quaternionf keyFrameOrientation_world =
            T_world_keyFrame_SE3f.unit_quaternion();

        /* Add the keyframe position to the RViz sphere-list marker */
        geometry_msgs::msg::Point keyFramePositionPoint;

        keyFramePositionPoint.x = keyFramePosition_world_m.x();
        keyFramePositionPoint.y = keyFramePosition_world_m.y();
        keyFramePositionPoint.z = keyFramePosition_world_m.z();

        keyFramePositionMarker.points.push_back(keyFramePositionPoint);

        /* Add the complete keyframe pose to the ROS path */
        geometry_msgs::msg::PoseStamped keyFramePoseMessage;

        keyFramePoseMessage.header.frame_id = frameWorld;

        keyFramePoseMessage.header.stamp =
            rclcpp::Time(static_cast<std::int64_t>(keyFrame->mTimeStamp * 1e9));

        keyFramePoseMessage.pose.position.x = keyFramePosition_world_m.x();
        keyFramePoseMessage.pose.position.y = keyFramePosition_world_m.y();
        keyFramePoseMessage.pose.position.z = keyFramePosition_world_m.z();

        keyFramePoseMessage.pose.orientation.x = keyFrameOrientation_world.x();
        keyFramePoseMessage.pose.orientation.y = keyFrameOrientation_world.y();
        keyFramePoseMessage.pose.orientation.z = keyFrameOrientation_world.z();
        keyFramePoseMessage.pose.orientation.w = keyFrameOrientation_world.w();

        keyFramePathMessage.poses.push_back(keyFramePoseMessage);
    }

    /* Publish the keyframe position marker */
    if (pubKeyFrameMarker != nullptr && !keyFramePositionMarker.points.empty())
    {
        visualization_msgs::msg::MarkerArray keyFrameMarkerArray;

        keyFrameMarkerArray.markers.reserve(1);

        keyFrameMarkerArray.markers.push_back(
            std::move(keyFramePositionMarker));

        pubKeyFrameMarker->publish(keyFrameMarkerArray);
    }

    /* Publish the ordered keyframe path */
    if (pubKeyFrameList != nullptr && !keyFramePathMessage.poses.empty())
    {
        pubKeyFrameList->publish(keyFramePathMessage);
    }
}
