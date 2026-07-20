/*!
 * @File:         getSkeletonMarkerWorldTransform.cpp
 *
 * @Brief:        Resolves the transformation from a Voxblox skeleton marker's
 *                local coordinate frame into the configured world frame.
 *
 * @Date:         20/07/2026
 *
 */

#include <cmath>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

bool getSkeletonMarkerWorldTransform(
    const visualization_msgs::msg::Marker &skeletonMarker_in,
    tf2::Transform                        &T_world_skeletonMarker_out)
{
    /* Reset the output so a failed call cannot return stale data */
    T_world_skeletonMarker_out.setIdentity();

    /*!
     * Use the frame supplied by Voxblox. frameMap is only used as a fallback
     * when the marker does not contain a frame identifier.
     */
    const std::string sourceFrameId = skeletonMarker_in.header.frame_id.empty()
                                          ? frameMap
                                          : skeletonMarker_in.header.frame_id;

    if (sourceFrameId.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot transform Voxblox marker: source frame is empty.");

        return false;
    }

    if (frameWorld.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot transform Voxblox marker: world frame is empty.");

        return false;
    }

    /*!
     * Transformation from the source coordinate frame into the configured
     * world frame.
     */
    tf2::Transform T_world_sourceFrame;

    T_world_sourceFrame.setIdentity();

    if (sourceFrameId != frameWorld)
    {
        if (tfBuffer_ == nullptr)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Cannot transform Voxblox marker: TF buffer is unavailable.");

            return false;
        }

        try
        {
            const geometry_msgs::msg::TransformStamped
                worldFromSourceTransformMessage =
                    tfBuffer_->lookupTransform(frameWorld,
                                               sourceFrameId,
                                               tf2::TimePointZero,
                                               tf2::durationFromSec(0.1));

            const geometry_msgs::msg::Transform &transformMessage =
                worldFromSourceTransformMessage.transform;

            if (!std::isfinite(transformMessage.translation.x) ||
                !std::isfinite(transformMessage.translation.y) ||
                !std::isfinite(transformMessage.translation.z) ||
                !std::isfinite(transformMessage.rotation.x) ||
                !std::isfinite(transformMessage.rotation.y) ||
                !std::isfinite(transformMessage.rotation.z) ||
                !std::isfinite(transformMessage.rotation.w))
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("visual_sgraphs"),
                    "Cannot transform Voxblox marker: TF from '%s' to '%s' "
                    "contains non-finite values.",
                    sourceFrameId.c_str(),
                    frameWorld.c_str());

                return false;
            }

            tf2::Quaternion sourceOrientation_world(
                transformMessage.rotation.x,
                transformMessage.rotation.y,
                transformMessage.rotation.z,
                transformMessage.rotation.w);

            constexpr tf2Scalar minimumQuaternionNormSquared =
                static_cast<tf2Scalar>(1e-12);

            if (sourceOrientation_world.length2() <
                minimumQuaternionNormSquared)
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("visual_sgraphs"),
                    "Cannot transform Voxblox marker: TF quaternion from '%s' "
                    "to '%s' is invalid.",
                    sourceFrameId.c_str(),
                    frameWorld.c_str());

                return false;
            }

            sourceOrientation_world.normalize();

            T_world_sourceFrame.setOrigin(
                tf2::Vector3(transformMessage.translation.x,
                             transformMessage.translation.y,
                             transformMessage.translation.z));

            T_world_sourceFrame.setRotation(sourceOrientation_world);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Could not resolve Voxblox transform from '%s' to '%s': %s",
                sourceFrameId.c_str(),
                frameWorld.c_str(),
                exception.what());

            return false;
        }
    }

    /* Extract and validate the marker-local pose */
    const geometry_msgs::msg::Pose &markerPose = skeletonMarker_in.pose;

    if (!std::isfinite(markerPose.position.x) ||
        !std::isfinite(markerPose.position.y) ||
        !std::isfinite(markerPose.position.z) ||
        !std::isfinite(markerPose.orientation.x) ||
        !std::isfinite(markerPose.orientation.y) ||
        !std::isfinite(markerPose.orientation.z) ||
        !std::isfinite(markerPose.orientation.w))
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot transform Voxblox marker: marker pose contains "
                    "non-finite values.");

        return false;
    }

    tf2::Quaternion markerOrientation_source(markerPose.orientation.x,
                                             markerPose.orientation.y,
                                             markerPose.orientation.z,
                                             markerPose.orientation.w);

    constexpr tf2Scalar minimumQuaternionNormSquared =
        static_cast<tf2Scalar>(1e-12);

    /*!
     * Some Marker messages leave the pose quaternion as all zeros when the
     * intended pose is identity.
     */
    if (markerOrientation_source.length2() < minimumQuaternionNormSquared)
    {
        markerOrientation_source.setValue(0.0, 0.0, 0.0, 1.0);
    }
    else
    {
        markerOrientation_source.normalize();
    }

    const tf2::Transform T_sourceFrame_skeletonMarker(
        markerOrientation_source,
        tf2::Vector3(markerPose.position.x,
                     markerPose.position.y,
                     markerPose.position.z));

    /*!
     * Transformation order:
     *
     * marker-local point -> source frame -> world frame
     */
    T_world_skeletonMarker_out =
        T_world_sourceFrame * T_sourceFrame_skeletonMarker;

    return true;
}