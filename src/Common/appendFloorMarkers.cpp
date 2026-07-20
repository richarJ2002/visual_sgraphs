/*!
 * @File:         appendFloorMarkers.cpp
 *
 * @Brief:        Appends floor, floor-label, and floor-to-room association
 *                markers to a structural marker array.
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

void appendFloorMarkers(
    const std::vector<ORB_SLAM3::Floor *> &mappedFloors_in,
    const rclcpp::Time                    &msgTime_s_in,
    visualization_msgs::msg::MarkerArray  &structuralElementMarkerArray_out)
{
    constexpr double floorDisplayOffset_m = -1.0;

    constexpr double textOffset_m = -0.5;

    constexpr float floorColourRed = 0.3F;

    constexpr float floorColourGreen = 0.6F;

    constexpr float floorColourBlue = 0.7F;

    const auto appendFloorDeleteMarkers =
        [&msgTime_s_in, &structuralElementMarkerArray_out](const int floorId)
    {
        visualization_msgs::msg::Marker deleteFloorMarker;

        deleteFloorMarker.header.frame_id = frameWorld;
        deleteFloorMarker.header.stamp    = msgTime_s_in;

        deleteFloorMarker.ns = "floors";
        deleteFloorMarker.id = floorId;

        deleteFloorMarker.action = visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(deleteFloorMarker);

        visualization_msgs::msg::Marker deleteFloorLabelMarker;

        deleteFloorLabelMarker.header.frame_id = frameWorld;
        deleteFloorLabelMarker.header.stamp    = msgTime_s_in;

        deleteFloorLabelMarker.ns = "floorLabels";
        deleteFloorLabelMarker.id = floorId;

        deleteFloorLabelMarker.action = visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteFloorLabelMarker);

        visualization_msgs::msg::Marker deleteFloorRoomLineMarker;

        deleteFloorRoomLineMarker.header.frame_id = frameWorld;
        deleteFloorRoomLineMarker.header.stamp    = msgTime_s_in;

        deleteFloorRoomLineMarker.ns = "floorRoomEdges";
        deleteFloorRoomLineMarker.id = floorId;

        deleteFloorRoomLineMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteFloorRoomLineMarker);
    };

    for (ORB_SLAM3::Floor *mappedFloor : mappedFloors_in)
    {
        if (mappedFloor == nullptr)
        {
            continue;
        }

        const int floorMarkerId = static_cast<int>(mappedFloor->getId());

        const std::vector<ORB_SLAM3::Room *> associatedRooms =
            mappedFloor->getRooms();

        /* Do not display floors without any associated rooms */
        if (associatedRooms.empty())
        {
            appendFloorDeleteMarkers(floorMarkerId);

            continue;
        }

        const Eigen::Vector3d floorCentroid_world_m =
            mappedFloor->getCentroid();

        if (!floorCentroid_world_m.allFinite())
        {
            appendFloorDeleteMarkers(floorMarkerId);

            continue;
        }

        Eigen::Vector3d floorDisplayPosition_world_m = floorCentroid_world_m;

        /*!
         * Offset the floor along the vertical display axis so it remains
         * visible beneath the room nodes.
         */
        if (sensorType == ORB_SLAM3::System::IMU_RGBD)
        {
            floorDisplayPosition_world_m.z() += floorDisplayOffset_m;
        }
        else
        {
            floorDisplayPosition_world_m.y() += floorDisplayOffset_m;
        }

        /* ------------------------------------------------------------------ *
         * FLOOR NODE
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker floorMarker;

        floorMarker.header.frame_id = frameWorld;
        floorMarker.header.stamp    = msgTime_s_in;

        floorMarker.ns = "floors";
        floorMarker.id = floorMarkerId;

        floorMarker.type   = visualization_msgs::msg::Marker::CUBE;
        floorMarker.action = visualization_msgs::msg::Marker::ADD;

        floorMarker.pose.position.x = floorDisplayPosition_world_m.x();
        floorMarker.pose.position.y = floorDisplayPosition_world_m.y();
        floorMarker.pose.position.z = floorDisplayPosition_world_m.z();

        floorMarker.pose.orientation.x = 0.0;
        floorMarker.pose.orientation.y = 0.0;
        floorMarker.pose.orientation.z = 0.0;
        floorMarker.pose.orientation.w = 1.0;

        floorMarker.scale.x = 0.4;
        floorMarker.scale.y = 0.4;
        floorMarker.scale.z = 0.4;

        floorMarker.color.r = floorColourRed;
        floorMarker.color.g = floorColourGreen;
        floorMarker.color.b = floorColourBlue;
        floorMarker.color.a = 1.0F;

        floorMarker.lifetime = rclcpp::Duration::from_seconds(0);

        structuralElementMarkerArray_out.markers.push_back(floorMarker);

        /* ------------------------------------------------------------------ *
         * FLOOR LABEL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker floorLabelMarker;

        floorLabelMarker.header.frame_id = frameWorld;
        floorLabelMarker.header.stamp    = msgTime_s_in;

        floorLabelMarker.ns = "floorLabels";
        floorLabelMarker.id = floorMarkerId;

        floorLabelMarker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        floorLabelMarker.action = visualization_msgs::msg::Marker::ADD;

        floorLabelMarker.text = mappedFloor->getName();

        floorLabelMarker.pose.position.x = floorDisplayPosition_world_m.x();
        floorLabelMarker.pose.position.y = floorDisplayPosition_world_m.y();
        floorLabelMarker.pose.position.z = floorDisplayPosition_world_m.z();

        if (sensorType == ORB_SLAM3::System::IMU_RGBD)
        {
            floorLabelMarker.pose.position.z += textOffset_m;
        }
        else
        {
            floorLabelMarker.pose.position.y += textOffset_m;
        }

        floorLabelMarker.pose.orientation.x = 0.0;
        floorLabelMarker.pose.orientation.y = 0.0;
        floorLabelMarker.pose.orientation.z = 0.0;
        floorLabelMarker.pose.orientation.w = 1.0;

        floorLabelMarker.scale.z = 0.2;

        floorLabelMarker.color.r = 0.0F;
        floorLabelMarker.color.g = 0.0F;
        floorLabelMarker.color.b = 0.0F;
        floorLabelMarker.color.a = 1.0F;

        floorLabelMarker.lifetime = rclcpp::Duration::from_seconds(0);

        structuralElementMarkerArray_out.markers.push_back(floorLabelMarker);

        /* ------------------------------------------------------------------ *
         * FLOOR-TO-ROOM ASSOCIATIONS
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker floorRoomAssociationMarker;

        floorRoomAssociationMarker.header.frame_id = frameWorld;
        floorRoomAssociationMarker.header.stamp    = msgTime_s_in;

        floorRoomAssociationMarker.ns = "floorRoomEdges";
        floorRoomAssociationMarker.id = floorMarkerId;

        floorRoomAssociationMarker.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        floorRoomAssociationMarker.action =
            visualization_msgs::msg::Marker::ADD;

        floorRoomAssociationMarker.pose.orientation.x = 0.0;
        floorRoomAssociationMarker.pose.orientation.y = 0.0;
        floorRoomAssociationMarker.pose.orientation.z = 0.0;
        floorRoomAssociationMarker.pose.orientation.w = 1.0;

        floorRoomAssociationMarker.scale.x = 0.05;

        floorRoomAssociationMarker.color.r = floorColourRed;
        floorRoomAssociationMarker.color.g = floorColourGreen;
        floorRoomAssociationMarker.color.b = floorColourBlue;
        floorRoomAssociationMarker.color.a = 0.9F;

        floorRoomAssociationMarker.lifetime = rclcpp::Duration::from_seconds(0);

        floorRoomAssociationMarker.points.reserve(associatedRooms.size() * 2);

        for (ORB_SLAM3::Room *associatedRoom : associatedRooms)
        {
            if (associatedRoom == nullptr || associatedRoom->isBad())
            {
                continue;
            }

            const ORB_SLAM3::Room::roomVariant roomType =
                associatedRoom->getRoomVariant();

            const bool isConfirmedRoom =
                roomType == ORB_SLAM3::Room::roomVariant::ROOM ||
                roomType == ORB_SLAM3::Room::roomVariant::CORRIDOR;

            if (!isConfirmedRoom)
            {
                continue;
            }

            geometry_msgs::msg::PointStamped roomDisplayPoint_SE;
            geometry_msgs::msg::PointStamped roomDisplayPoint_world;

            if (!getRoomDisplayPoints(associatedRoom,
                                      msgTime_s_in,
                                      roomDisplayPoint_SE,
                                      roomDisplayPoint_world))
            {
                continue;
            }

            geometry_msgs::msg::Point floorEnd_world;

            floorEnd_world.x = floorDisplayPosition_world_m.x();
            floorEnd_world.y = floorDisplayPosition_world_m.y();
            floorEnd_world.z = floorDisplayPosition_world_m.z();

            geometry_msgs::msg::Point roomEnd_world;

            roomEnd_world.x = roomDisplayPoint_world.point.x;
            roomEnd_world.y = roomDisplayPoint_world.point.y;
            roomEnd_world.z = roomDisplayPoint_world.point.z;

            floorRoomAssociationMarker.points.push_back(floorEnd_world);
            floorRoomAssociationMarker.points.push_back(roomEnd_world);
        }

        /*!
         * Publish even when empty to replace and clear any older association
         * lines belonging to this floor.
         */
        structuralElementMarkerArray_out.markers.push_back(
            floorRoomAssociationMarker);
    }
}
