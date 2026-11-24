/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            lookupWorldFromFrameTransform.cc
 *
 * @brief           Implements lookupWorldFromFrameTransform(), declared in
 *                  common.h.
 */

#include "common/common.h"

namespace vs_graphs::common
{

bool lookupWorldFromFrameTransform(
    const std::string &sourceFrameId_in,
    tf2::Transform    &frameTransform_sourceToWorld_out)
{
    /* Reset the output so a failed call cannot return stale data */
    frameTransform_sourceToWorld_out.setIdentity();

    if (sourceFrameId_in.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot resolve the Voxblox frame: source frame is empty.");

        return false;
    }

    if (frameWorld.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot resolve the Voxblox frame: world frame is empty.");

        return false;
    }

    if (sourceFrameId_in != frameWorld)
    {
        if (tfBuffer_ == nullptr)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Cannot resolve the Voxblox frame: TF buffer is unavailable.");

            return false;
        }

        try
        {
            const geometry_msgs::msg::TransformStamped
                worldFromSourceTransformMessage =
                    tfBuffer_->lookupTransform(frameWorld,
                                               sourceFrameId_in,
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
                    "Cannot resolve the Voxblox frame: TF from '%s' to '%s' "
                    "contains non-finite values.",
                    sourceFrameId_in.c_str(),
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
                    "Cannot resolve the Voxblox frame: TF quaternion from '%s' "
                    "to '%s' is invalid.",
                    sourceFrameId_in.c_str(),
                    frameWorld.c_str());

                return false;
            }

            sourceOrientation_world.normalize();

            frameTransform_sourceToWorld_out.setOrigin(
                tf2::Vector3(transformMessage.translation.x,
                             transformMessage.translation.y,
                             transformMessage.translation.z));

            frameTransform_sourceToWorld_out.setRotation(
                sourceOrientation_world);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Could not resolve Voxblox transform from '%s' to '%s': %s",
                sourceFrameId_in.c_str(),
                frameWorld.c_str(),
                exception.what());

            return false;
        }
    }

    return true;
}

} /* namespace vs_graphs::common */
