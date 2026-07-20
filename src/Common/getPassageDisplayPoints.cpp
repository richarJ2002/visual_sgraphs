/*!
 * @File:         getPassageDisplayPoints.cpp
 *
 * @Brief:        Calculates the displayed structural-graph position of a
 *                passage.
 *
 * @Date:         20/07/2026
 *
 */

#include <cmath>
#include <string>

/* Function Includes */
#include "Common.hpp"

/* Object Includes */
#include "Semantic/Passage.h"
#include "System.h"

/* Data Includes */
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

/* ROS Includes */
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

/* Generic Libraries */
/* None */

bool getPassageDisplayPoints(
    ORB_SLAM3::Passage               *passage_in,
    const rclcpp::Time               &msgTime_in,
    const double                      verticalOffset_in,
    geometry_msgs::msg::PointStamped &passagePointSE_out,
    geometry_msgs::msg::PointStamped &passagePointWorld_out)
{
    /* Confirm that the passage and TF buffer are valid */
    if (passage_in == nullptr || tfBuffer_ == nullptr)
    {
        return false;
    }

    /* Extract the physical passage centroid */
    const Eigen::Vector3f passageCentroid = passage_in->getCentroid();

    if (!passageCentroid.allFinite())
    {
        return false;
    }

    /* Create the physical passage point in the world frame */
    geometry_msgs::msg::PointStamped passagePointWorldPhysical;

    /* Update the header of the msg */
    passagePointWorldPhysical.header.stamp    = msgTime_in;
    passagePointWorldPhysical.header.frame_id = frameWorld;

    /* Update the coordinates of the passage in the world frame */
    passagePointWorldPhysical.point.x = passageCentroid.x();
    passagePointWorldPhysical.point.y = passageCentroid.y();
    passagePointWorldPhysical.point.z = passageCentroid.z();

    try
    {
        /* Transform the physical doorway position into frameSE */
        const geometry_msgs::msg::TransformStamped worldToSE =
            tfBuffer_->lookupTransform(frameSE,
                                       frameWorld,
                                       msgTime_in,
                                       rclcpp::Duration::from_seconds(0.1));

        tf2::doTransform(passagePointWorldPhysical,
                         passagePointSE_out,
                         worldToSE);

        /*!
         * Move the structural passage node slightly below the room level.
         *
         * The project uses Z as the graph offset axis for IMU RGB-D and Y for
         * the other sensor configurations.
         */
        if (sensorType == ORB_SLAM3::System::IMU_RGBD)
        {
            passagePointSE_out.point.z += verticalOffset_in;
        }
        else
        {
            passagePointSE_out.point.y += verticalOffset_in;
        }

        /* Update the header in the structural element frame */
        passagePointSE_out.header.stamp    = msgTime_in;
        passagePointSE_out.header.frame_id = frameSE;

        /* Transform the displayed position back to world for graph edges */
        const geometry_msgs::msg::TransformStamped seToWorld =
            tfBuffer_->lookupTransform(frameWorld,
                                       frameSE,
                                       msgTime_in,
                                       rclcpp::Duration::from_seconds(0.1));

        tf2::doTransform(passagePointSE_out, passagePointWorld_out, seToWorld);
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Passage display transform failed: %s",
                    exception.what());

        return false;
    }

    return true;
}
