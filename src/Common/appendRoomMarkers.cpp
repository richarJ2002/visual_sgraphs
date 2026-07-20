/*!
 * @File:         appendRoomMarkers.cpp
 *
 * @Brief:        Appends room, corridor, label, wall-association, and
 *                passage-association markers to a structural marker array.
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

void appendRoomMarkers(
    const std::vector<ORB_SLAM3::Room *> &mappedRooms_in,
    const rclcpp::Time                   &msgTime_s_in,
    visualization_msgs::msg::MarkerArray &structuralElementMarkerArray_out)
{
    constexpr double textOffset_m = -0.5;

    /*!
     * Cache the building-component-to-world transform once rather than
     * performing one lookup for every wall.
     */
    geometry_msgs::msg::TransformStamped transformWorldFromBC;

    bool hasTransformWorldFromBC = false;

    if (frameBC == frameWorld)
    {
        hasTransformWorldFromBC = true;
    }
    else if (tfBuffer_ != nullptr)
    {
        try
        {
            transformWorldFromBC =
                tfBuffer_->lookupTransform(frameWorld,
                                           frameBC,
                                           msgTime_s_in,
                                           rclcpp::Duration::from_seconds(0.1));

            hasTransformWorldFromBC = true;
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Unable to transform wall centroids from '%s' to '%s': %s",
                frameBC.c_str(),
                frameWorld.c_str(),
                exception.what());
        }
    }

    /* Adds DELETE messages for every marker associated with a room */
    const auto appendRoomDeleteMarkers =
        [&msgTime_s_in, &structuralElementMarkerArray_out](const int roomId)
    {
        visualization_msgs::msg::Marker deleteRoomMarker;

        deleteRoomMarker.header.frame_id = frameSE;
        deleteRoomMarker.header.stamp    = msgTime_s_in;

        deleteRoomMarker.ns = "room";
        deleteRoomMarker.id = roomId;

        deleteRoomMarker.action = visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(deleteRoomMarker);

        visualization_msgs::msg::Marker deleteRoomLabelMarker;

        deleteRoomLabelMarker.header.frame_id = frameSE;
        deleteRoomLabelMarker.header.stamp    = msgTime_s_in;

        deleteRoomLabelMarker.ns = "roomLabel";
        deleteRoomLabelMarker.id = roomId;

        deleteRoomLabelMarker.action = visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteRoomLabelMarker);

        visualization_msgs::msg::Marker deleteRoomWallLineMarker;

        deleteRoomWallLineMarker.header.frame_id = frameWorld;
        deleteRoomWallLineMarker.header.stamp    = msgTime_s_in;

        deleteRoomWallLineMarker.ns = "roomWallLine";
        deleteRoomWallLineMarker.id = roomId;

        deleteRoomWallLineMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteRoomWallLineMarker);

        visualization_msgs::msg::Marker deleteRoomPassageLineMarker;

        deleteRoomPassageLineMarker.header.frame_id = frameWorld;
        deleteRoomPassageLineMarker.header.stamp    = msgTime_s_in;

        deleteRoomPassageLineMarker.ns = "roomPassageLine";
        deleteRoomPassageLineMarker.id = roomId;

        deleteRoomPassageLineMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteRoomPassageLineMarker);
    };

    for (ORB_SLAM3::Room *mappedRoom : mappedRooms_in)
    {
        /* Null pointers do not contain an ID that can be deleted */
        if (mappedRoom == nullptr)
        {
            continue;
        }

        const int roomMarkerId = static_cast<int>(mappedRoom->getId());

        const ORB_SLAM3::Room::roomVariant roomType =
            mappedRoom->getRoomVariant();

        const bool isConfirmedRoom =
            roomType == ORB_SLAM3::Room::roomVariant::ROOM ||
            roomType == ORB_SLAM3::Room::roomVariant::CORRIDOR;

        /* Remove bad and provisional structural elements from RViz */
        if (mappedRoom->isBad() || !isConfirmedRoom)
        {
            appendRoomDeleteMarkers(roomMarkerId);

            continue;
        }

        /* Select the displayed room colour */
        float roomColourRed   = 0.5F;
        float roomColourGreen = 0.5F;
        float roomColourBlue  = 0.5F;

        if (roomType == ORB_SLAM3::Room::roomVariant::CORRIDOR)
        {
            roomColourRed   = 0.6F;
            roomColourGreen = 0.0F;
            roomColourBlue  = 0.3F;
        }
        else if (roomType == ORB_SLAM3::Room::roomVariant::ROOM)
        {
            roomColourRed   = 0.5F;
            roomColourGreen = 0.1F;
            roomColourBlue  = 1.0F;
        }

        geometry_msgs::msg::PointStamped roomDisplayPoint_SE;
        geometry_msgs::msg::PointStamped roomDisplayPoint_world;

        if (!getRoomDisplayPoints(mappedRoom,
                                  msgTime_s_in,
                                  roomDisplayPoint_SE,
                                  roomDisplayPoint_world))
        {
            appendRoomDeleteMarkers(roomMarkerId);

            continue;
        }

        /* ------------------------------------------------------------------ *
         * ROOM NODE
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker roomMarker;

        roomMarker.header.frame_id = frameSE;
        roomMarker.header.stamp    = msgTime_s_in;

        roomMarker.ns = "room";
        roomMarker.id = roomMarkerId;

        roomMarker.type   = visualization_msgs::msg::Marker::CUBE;
        roomMarker.action = visualization_msgs::msg::Marker::ADD;

        roomMarker.pose.position.x = roomDisplayPoint_SE.point.x;
        roomMarker.pose.position.y = roomDisplayPoint_SE.point.y;
        roomMarker.pose.position.z = roomDisplayPoint_SE.point.z;

        roomMarker.pose.orientation.x = 0.0;
        roomMarker.pose.orientation.y = 0.0;
        roomMarker.pose.orientation.z = 0.0;
        roomMarker.pose.orientation.w = 1.0;

        roomMarker.scale.x = 0.3;
        roomMarker.scale.y = 0.3;
        roomMarker.scale.z = 0.3;

        roomMarker.color.r = roomColourRed;
        roomMarker.color.g = roomColourGreen;
        roomMarker.color.b = roomColourBlue;
        roomMarker.color.a = 1.0F;

        roomMarker.lifetime = rclcpp::Duration::from_seconds(0);
        structuralElementMarkerArray_out.markers.push_back(roomMarker);

        /* ------------------------------------------------------------------ *
         * ROOM LABEL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker roomLabelMarker;

        roomLabelMarker.header.frame_id = frameSE;
        roomLabelMarker.header.stamp    = msgTime_s_in;

        roomLabelMarker.ns = "roomLabel";
        roomLabelMarker.id = roomMarkerId;

        roomLabelMarker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        roomLabelMarker.action = visualization_msgs::msg::Marker::ADD;

        roomLabelMarker.text = mappedRoom->getName();

        /* Keep the X, Y, and Z coordinates in their original order */
        roomLabelMarker.pose.position.x = roomDisplayPoint_SE.point.x;

        roomLabelMarker.pose.position.y =
            roomDisplayPoint_SE.point.y + textOffset_m;

        roomLabelMarker.pose.position.z = roomDisplayPoint_SE.point.z;

        roomLabelMarker.pose.orientation.x = 0.0;
        roomLabelMarker.pose.orientation.y = 0.0;
        roomLabelMarker.pose.orientation.z = 0.0;
        roomLabelMarker.pose.orientation.w = 1.0;

        roomLabelMarker.scale.z = 0.2;

        roomLabelMarker.color.r = 0.0F;
        roomLabelMarker.color.g = 0.0F;
        roomLabelMarker.color.b = 0.0F;
        roomLabelMarker.color.a = 1.0F;

        roomLabelMarker.lifetime = rclcpp::Duration::from_seconds(0);

        structuralElementMarkerArray_out.markers.push_back(roomLabelMarker);

        /* ------------------------------------------------------------------ *
         * ROOM-TO-WALL ASSOCIATIONS
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker roomWallAssociationMarker;

        roomWallAssociationMarker.header.frame_id = frameWorld;
        roomWallAssociationMarker.header.stamp    = msgTime_s_in;

        roomWallAssociationMarker.ns = "roomWallLine";
        roomWallAssociationMarker.id = roomMarkerId;

        roomWallAssociationMarker.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        roomWallAssociationMarker.action = visualization_msgs::msg::Marker::ADD;

        roomWallAssociationMarker.pose.orientation.x = 0.0;
        roomWallAssociationMarker.pose.orientation.y = 0.0;
        roomWallAssociationMarker.pose.orientation.z = 0.0;
        roomWallAssociationMarker.pose.orientation.w = 1.0;

        roomWallAssociationMarker.scale.x = 0.05;

        roomWallAssociationMarker.color.r = roomColourRed;
        roomWallAssociationMarker.color.g = roomColourGreen;
        roomWallAssociationMarker.color.b = roomColourBlue;
        roomWallAssociationMarker.color.a = 0.9F;

        roomWallAssociationMarker.lifetime = rclcpp::Duration::from_seconds(0);

        const std::vector<ORB_SLAM3::Plane *> associatedWalls =
            mappedRoom->getWalls();

        roomWallAssociationMarker.points.reserve(associatedWalls.size() * 2);

        for (ORB_SLAM3::Plane *associatedWall : associatedWalls)
        {
            if (associatedWall == nullptr || associatedWall->isBad())
            {
                continue;
            }

            const Eigen::Vector3f wallCentroid_BC_m =
                associatedWall->getCentroid();

            if (!wallCentroid_BC_m.allFinite())
            {
                continue;
            }

            geometry_msgs::msg::Point wallEnd_world;

            if (frameBC == frameWorld)
            {
                wallEnd_world.x = wallCentroid_BC_m.x();
                wallEnd_world.y = wallCentroid_BC_m.y();
                wallEnd_world.z = wallCentroid_BC_m.z();
            }
            else
            {
                if (!hasTransformWorldFromBC)
                {
                    continue;
                }

                geometry_msgs::msg::PointStamped wallPoint_BC;
                geometry_msgs::msg::PointStamped wallPoint_world;

                wallPoint_BC.header.frame_id = frameBC;
                wallPoint_BC.header.stamp    = msgTime_s_in;

                wallPoint_BC.point.x = wallCentroid_BC_m.x();
                wallPoint_BC.point.y = wallCentroid_BC_m.y();
                wallPoint_BC.point.z = wallCentroid_BC_m.z();

                tf2::doTransform(wallPoint_BC,
                                 wallPoint_world,
                                 transformWorldFromBC);

                wallEnd_world = wallPoint_world.point;
            }

            geometry_msgs::msg::Point roomEnd_world;

            roomEnd_world.x = roomDisplayPoint_world.point.x;
            roomEnd_world.y = roomDisplayPoint_world.point.y;
            roomEnd_world.z = roomDisplayPoint_world.point.z;

            roomWallAssociationMarker.points.push_back(roomEnd_world);
            roomWallAssociationMarker.points.push_back(wallEnd_world);
        }

        /*!
         * Append the marker even when it contains no points. This replaces and
         * clears any previously published lines for the same room ID.
         */
        structuralElementMarkerArray_out.markers.push_back(
            roomWallAssociationMarker);

        /* ------------------------------------------------------------------ *
         * ROOM-TO-PASSAGE ASSOCIATIONS
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker roomPassageAssociationMarker;

        roomPassageAssociationMarker.header.frame_id = frameWorld;
        roomPassageAssociationMarker.header.stamp    = msgTime_s_in;

        roomPassageAssociationMarker.ns = "roomPassageLine";
        roomPassageAssociationMarker.id = roomMarkerId;

        roomPassageAssociationMarker.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        roomPassageAssociationMarker.action =
            visualization_msgs::msg::Marker::ADD;

        roomPassageAssociationMarker.pose.orientation.x = 0.0;
        roomPassageAssociationMarker.pose.orientation.y = 0.0;
        roomPassageAssociationMarker.pose.orientation.z = 0.0;
        roomPassageAssociationMarker.pose.orientation.w = 1.0;

        roomPassageAssociationMarker.scale.x = 0.05;

        roomPassageAssociationMarker.lifetime =
            rclcpp::Duration::from_seconds(0);

        const std::vector<ORB_SLAM3::Passage *> associatedPassages =
            mappedRoom->getPassages();

        roomPassageAssociationMarker.points.reserve(associatedPassages.size() *
                                                    2);

        roomPassageAssociationMarker.colors.reserve(associatedPassages.size() *
                                                    2);

        for (ORB_SLAM3::Passage *associatedPassage : associatedPassages)
        {
            if (associatedPassage == nullptr)
            {
                continue;
            }

            geometry_msgs::msg::PointStamped passageDisplayPoint_SE;
            geometry_msgs::msg::PointStamped passageDisplayPoint_world;

            if (!getPassageDisplayPoints(associatedPassage,
                                         msgTime_s_in,
                                         0.0,
                                         passageDisplayPoint_SE,
                                         passageDisplayPoint_world))
            {
                continue;
            }

            geometry_msgs::msg::Point roomEnd_world;

            roomEnd_world.x = roomDisplayPoint_world.point.x;
            roomEnd_world.y = roomDisplayPoint_world.point.y;
            roomEnd_world.z = roomDisplayPoint_world.point.z;

            geometry_msgs::msg::Point passageEnd_world;

            passageEnd_world.x = passageDisplayPoint_world.point.x;
            passageEnd_world.y = passageDisplayPoint_world.point.y;
            passageEnd_world.z = passageDisplayPoint_world.point.z;

            std_msgs::msg::ColorRGBA passageLineColour;

            passageLineColour.a = 0.9F;

            if (associatedPassage->isPassable())
            {
                passageLineColour.r = 0.0F;
                passageLineColour.g = 1.0F;
                passageLineColour.b = 0.7F;
            }
            else
            {
                passageLineColour.r = 0.1F;
                passageLineColour.g = 0.0F;
                passageLineColour.b = 0.0F;
            }

            roomPassageAssociationMarker.points.push_back(roomEnd_world);
            roomPassageAssociationMarker.points.push_back(passageEnd_world);

            /* LINE_LIST requires one colour entry for every point */
            roomPassageAssociationMarker.colors.push_back(passageLineColour);
            roomPassageAssociationMarker.colors.push_back(passageLineColour);
        }

        /*!
         * Append an empty marker as well so old lines are cleared when the
         * room no longer has associated passages.
         */
        structuralElementMarkerArray_out.markers.push_back(
            roomPassageAssociationMarker);
    }
}
