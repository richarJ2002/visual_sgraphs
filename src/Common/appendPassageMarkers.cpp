/*!
 * @File:         appendPassageMarkers.cpp
 *
 * @Brief:        Appends passage and passage-label markers to a structural
 *                marker array.
 *
 * @Date:         20/07/2026
 *
 */

#include <std_msgs/msg/color_rgba.hpp>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void appendPassageMarkers(
    const std::vector<ORB_SLAM3::Passage *> &mappedPassages_in,
    const rclcpp::Time                      &msgTime_s_in,
    visualization_msgs::msg::MarkerArray    &structuralElementMarkerArray_out)
{
    constexpr double textOffset_m = -0.5;

    const auto appendPassageDeleteMarkers =
        [&msgTime_s_in, &structuralElementMarkerArray_out](const int passageId)
    {
        visualization_msgs::msg::Marker deletePassageMarker;

        deletePassageMarker.header.frame_id = frameSE;
        deletePassageMarker.header.stamp    = msgTime_s_in;

        deletePassageMarker.ns = "passage";
        deletePassageMarker.id = passageId;

        deletePassageMarker.action = visualization_msgs::msg::Marker::DELETE;
        structuralElementMarkerArray_out.markers.push_back(deletePassageMarker);

        visualization_msgs::msg::Marker deletePassageLabelMarker;

        deletePassageLabelMarker.header.frame_id = frameSE;
        deletePassageLabelMarker.header.stamp    = msgTime_s_in;

        deletePassageLabelMarker.ns = "passageLabel";
        deletePassageLabelMarker.id = passageId;

        deletePassageLabelMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deletePassageLabelMarker);
    };

    for (ORB_SLAM3::Passage *mappedPassage : mappedPassages_in)
    {
        if (mappedPassage == nullptr)
        {
            continue;
        }

        const int passageMarkerId = static_cast<int>(mappedPassage->getId());

        geometry_msgs::msg::PointStamped passageDisplayPoint_SE;
        geometry_msgs::msg::PointStamped passageDisplayPoint_world;

        if (!getPassageDisplayPoints(mappedPassage,
                                     msgTime_s_in,
                                     0.0,
                                     passageDisplayPoint_SE,
                                     passageDisplayPoint_world))
        {
            appendPassageDeleteMarkers(passageMarkerId);

            continue;
        }

        const bool isPassageOpen = mappedPassage->isPassable();

        float passageColourRed   = 0.1F;
        float passageColourGreen = 0.0F;
        float passageColourBlue  = 0.0F;

        if (isPassageOpen)
        {
            passageColourRed   = 0.0F;
            passageColourGreen = 1.0F;
            passageColourBlue  = 0.7F;
        }

        /* ------------------------------------------------------------------ *
         * PASSAGE NODE
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker passageMarker;

        passageMarker.header.frame_id = frameSE;
        passageMarker.header.stamp    = msgTime_s_in;

        passageMarker.ns = "passage";
        passageMarker.id = passageMarkerId;

        passageMarker.type   = visualization_msgs::msg::Marker::CUBE;
        passageMarker.action = visualization_msgs::msg::Marker::ADD;

        passageMarker.pose.position.x = passageDisplayPoint_SE.point.x;
        passageMarker.pose.position.y = passageDisplayPoint_SE.point.y;
        passageMarker.pose.position.z = passageDisplayPoint_SE.point.z;

        passageMarker.pose.orientation.x = 0.0;
        passageMarker.pose.orientation.y = 0.0;
        passageMarker.pose.orientation.z = 0.0;
        passageMarker.pose.orientation.w = 1.0;

        passageMarker.scale.x = 0.30;
        passageMarker.scale.y = 0.30;
        passageMarker.scale.z = 0.30;

        passageMarker.color.r = passageColourRed;
        passageMarker.color.g = passageColourGreen;
        passageMarker.color.b = passageColourBlue;

        passageMarker.color.a = 1.0F;

        passageMarker.lifetime = rclcpp::Duration::from_seconds(0);

        structuralElementMarkerArray_out.markers.push_back(passageMarker);

        /* ------------------------------------------------------------------ *
         * PASSAGE LABEL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker passageLabelMarker;

        passageLabelMarker.header.frame_id = frameSE;
        passageLabelMarker.header.stamp    = msgTime_s_in;

        passageLabelMarker.ns = "passageLabel";
        passageLabelMarker.id = passageMarkerId;

        passageLabelMarker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        passageLabelMarker.action = visualization_msgs::msg::Marker::ADD;

        passageLabelMarker.text = "Passage#" + std::to_string(passageMarkerId) +
                                  (isPassageOpen ? " [open]" : " [blocked]");

        passageLabelMarker.pose.position.x = passageDisplayPoint_SE.point.x;
        passageLabelMarker.pose.position.y =
            passageDisplayPoint_SE.point.y + textOffset_m;
        passageLabelMarker.pose.position.z = passageDisplayPoint_SE.point.z;

        passageLabelMarker.pose.orientation.x = 0.0;
        passageLabelMarker.pose.orientation.y = 0.0;
        passageLabelMarker.pose.orientation.z = 0.0;
        passageLabelMarker.pose.orientation.w = 1.0;

        passageLabelMarker.scale.z = 0.20;

        passageLabelMarker.color.r = passageColourRed;
        passageLabelMarker.color.g = passageColourGreen;
        passageLabelMarker.color.b = passageColourBlue;
        passageLabelMarker.color.a = 0.9F;

        passageLabelMarker.lifetime = rclcpp::Duration::from_seconds(0);

        structuralElementMarkerArray_out.markers.push_back(passageLabelMarker);
    }
}
