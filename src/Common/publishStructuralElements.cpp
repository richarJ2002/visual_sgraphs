/*!
 * @File:         publishStructuralElements.cpp
 *
 * @Brief:
 *
 * @Date:         20/07/2026
 *
 */

#include <vector>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

/* Data Includes */
#include <visualization_msgs/msg/marker_array.hpp>

/* ROS Includes */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

/* Generic Libraries */
/* None */

void publishStructuralElements(
    const std::vector<ORB_SLAM3::Room *>    &mappedRooms_in,
    const std::vector<ORB_SLAM3::Floor *>   &mappedFloors_in,
    const std::vector<ORB_SLAM3::Passage *> &mappedPassages_in,
    const rclcpp::Time                      &msgTime_s_in)
{
    if (pubStructuralElements == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish structural elements: publisher is not "
                    "initialised.");

        return;
    }

    if (mappedRooms_in.empty() && mappedFloors_in.empty() &&
        mappedPassages_in.empty())
    {
        return;
    }

    visualization_msgs::msg::MarkerArray structuralElementMarkerArray;

    structuralElementMarkerArray.markers.reserve(mappedRooms_in.size() * 6 +
                                                 mappedFloors_in.size() * 3 +
                                                 mappedPassages_in.size() * 2);

    /* Append the room markers */
    appendRoomMarkers(mappedRooms_in,
                      msgTime_s_in,
                      structuralElementMarkerArray);

    /* Append the floor markers */
    appendFloorMarkers(mappedFloors_in,
                       msgTime_s_in,
                       structuralElementMarkerArray);

    /* Append the passage markers */
    appendPassageMarkers(mappedPassages_in,
                         msgTime_s_in,
                         structuralElementMarkerArray);

    if (!structuralElementMarkerArray.markers.empty())
    {
        pubStructuralElements->publish(structuralElementMarkerArray);
    }
}
