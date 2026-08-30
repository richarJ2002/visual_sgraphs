/*!
 * @File:         common.cc
 *
 * @Brief:        This file is a modified version of a file from ORB-SLAM3.
 *
 *                Modifications Copyright (C) 2023-2025 SnT, University of
 *                Luxembourg Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis
 *                Sanchez-Lopez, and Holger Voos
 *
 *                Original Copyright (C) 2014-2021 University of Zaragoza:
 *                Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez
 *                Rodríguez, José M.M. Montiel, and Juan D. Tardós.
 *
 *                This file is part of vS-Graphs, which is free software: you
 *                can redistribute it and/or modify it under the terms of the
 *                GNU General Public License as published by the Free Software
 *                Foundation, either version 3 of the License, or (at your
 *                option) any later version.
 *
 *                vS-Graphs is distributed in the hope that it will be useful,
 *                but WITHOUT ANY WARRANTY; without even the implied warranty of
 *                MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *                GNU General Public License for more details.
 *
 *                You should have received a copy of the GNU General Public
 *                License along with this program. If not, see
 *                <https://www.gnu.org/licenses/>.
 *
 * @Date:         20/07/2026
 *
 */

#include "common.hpp"

#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>

/* -------------------------------------------------------------------------- *
 * ORB-SLAM3 STATE
 * -------------------------------------------------------------------------- */

ORB_SLAM3::System *p_slamSystem = nullptr;

ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::NOT_SET;

/* Track which prospective-room marker IDs were published in the previous
 * frame. Prospective rooms are drawn in their own marker namespace
 * (prospectiveRoom / prospectiveRoomLabel / passageToProspective) and, once a
 * prospective is promoted to a confirmed room or cleaned up, its marker must
 * be explicitly deleted - otherwise stale cubes accumulate in RViz. */
std::unordered_set<int> s_publishedProspectiveMarkerIds;

/* -------------------------------------------------------------------------- *
 * COMMON CONFIGURATION
 * -------------------------------------------------------------------------- */

bool colorPointcloud = true;

double roll = 0.0;

double pitch = 0.0;

double yaw = 0.0;

bool pubStaticTransform = false;

bool pubPointClouds = false;

/* -------------------------------------------------------------------------- *
 * COORDINATE-FRAME IDENTIFIERS
 * -------------------------------------------------------------------------- */

std::string frameWorld;

std::string frameCamera;

std::string frameImu;

std::string frameMap;

std::string frameBC;

std::string frameSE;

/* -------------------------------------------------------------------------- *
 * TF INTERFACES
 * -------------------------------------------------------------------------- */

std::shared_ptr<tf2_ros::Buffer> tfBuffer_ = nullptr;

std::shared_ptr<tf2_ros::TransformListener> tfListener_ = nullptr;

std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster = nullptr;

std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster =
    nullptr;

/* -------------------------------------------------------------------------- *
 * SHARED DATA BUFFERS
 * -------------------------------------------------------------------------- */

std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;

std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;

std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

/* -------------------------------------------------------------------------- *
 * PUBLICATION STATE
 * -------------------------------------------------------------------------- */

rclcpp::Time lastPlanePublishTime(0, 0, RCL_ROS_TIME);

/* -------------------------------------------------------------------------- *
 * BASIC SLAM PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubKeyFrameList = nullptr;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry = nullptr;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllMappoints =
    nullptr;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCameraPose =
    nullptr;

rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr pubKFImage =
    nullptr;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubTrackedMappoints = nullptr;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    p_voxbloxInputPointCloudPublisher = nullptr;

rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr p_mapRevisionPublisher =
    nullptr;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubKeyFrameMarker = nullptr;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubFreespaceCluster = nullptr;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubCameraPoseVis = nullptr;

std::shared_ptr<image_transport::Publisher> pubTrackingImage = nullptr;

/* -------------------------------------------------------------------------- *
 * ENTITY PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDoor =
    nullptr;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubFiducialMarker = nullptr;

/* -------------------------------------------------------------------------- *
 * BUILDING-COMPONENT PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubPlaneLabel = nullptr;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubBuildingComponents = nullptr;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubSegmentedPointcloud = nullptr;

/* -------------------------------------------------------------------------- *
 * MAPPED-WALL PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr
    pubAllWalls_new = nullptr;

rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr
    pubAllWalls_legacy = nullptr;

/* -------------------------------------------------------------------------- *
 * STRUCTURAL-ELEMENT PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubStructuralElements = nullptr;

/*!
 * @brief Removes indefinitely-lived RViz markers from a previous map revision.
 *
 * @param[in] p_markerPublisher_in
 *            Marker publisher whose displayed state must be cleared.
 * @param[in] msgTime_s_in
 *            Timestamp assigned to the deletion marker.
 */
static void clearPublishedMarkers(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                       &p_markerPublisher_in,
    const rclcpp::Time &msgTime_s_in)
{
    if (p_markerPublisher_in == nullptr)
    {
        return;
    }

    visualization_msgs::msg::Marker deletionMarker;
    deletionMarker.header.frame_id = frameWorld;
    deletionMarker.header.stamp    = msgTime_s_in;
    deletionMarker.action          = visualization_msgs::msg::Marker::DELETEALL;

    visualization_msgs::msg::MarkerArray deletionMarkerArray;
    deletionMarkerArray.markers.push_back(std::move(deletionMarker));
    p_markerPublisher_in->publish(deletionMarkerArray);
}

/*!
 * @brief Replaces an indefinitely-displayed point cloud with an empty cloud.
 *
 * @param[in] p_pointCloudPublisher_in
 *            Point-cloud publisher whose displayed state must be cleared.
 * @param[in] frameId_in
 *            Coordinate frame assigned to the empty cloud.
 * @param[in] msgTime_s_in
 *            Timestamp assigned to the empty cloud.
 */
static void clearPublishedPointCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
                       &p_pointCloudPublisher_in,
    const std::string  &frameId_in,
    const rclcpp::Time &msgTime_s_in)
{
    if (p_pointCloudPublisher_in == nullptr)
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 emptyPointCloudMessage;
    emptyPointCloudMessage.header.frame_id = frameId_in;
    emptyPointCloudMessage.header.stamp    = msgTime_s_in;
    emptyPointCloudMessage.height          = 1U;
    emptyPointCloudMessage.width           = 0U;
    emptyPointCloudMessage.is_dense        = true;

    p_pointCloudPublisher_in->publish(emptyPointCloudMessage);
}

/*!
 * @brief       Clears all map-scoped visualization state after a map revision.
 *
 *              RViz markers use infinite lifetimes. Merely omitting an entity
 *              after the active map changes does not remove its old marker, so
 *              stale rooms and planes otherwise appear to belong to the new
 *              map.
 *
 * @param[in] msgTime_s_in
 *            Timestamp assigned to reset messages.
 */
static void clearMapScopedVisualization(const rclcpp::Time &msgTime_s_in)
{
    clearPublishedMarkers(pubKeyFrameMarker, msgTime_s_in);
    clearPublishedMarkers(pubCameraPoseVis, msgTime_s_in);
    clearPublishedMarkers(pubDoor, msgTime_s_in);
    clearPublishedMarkers(pubFiducialMarker, msgTime_s_in);
    clearPublishedMarkers(pubPlaneLabel, msgTime_s_in);
    clearPublishedMarkers(pubStructuralElements, msgTime_s_in);

    clearPublishedPointCloud(pubAllMappoints, frameWorld, msgTime_s_in);
    clearPublishedPointCloud(pubTrackedMappoints, frameWorld, msgTime_s_in);
    clearPublishedPointCloud(pubFreespaceCluster, frameWorld, msgTime_s_in);
    clearPublishedPointCloud(pubBuildingComponents, frameBC, msgTime_s_in);
    clearPublishedPointCloud(pubSegmentedPointcloud, frameCamera, msgTime_s_in);
}

/* -------------------------------------------------------------------------- *
 * SERVICE SERVERS
 * -------------------------------------------------------------------------- */

rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMap = nullptr;

rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMapPoints = nullptr;

rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveTrajectory = nullptr;

rclcpp::Service<vs_graphs::srv::GetMissionHealth>::SharedPtr
    srvGetMissionHealth = nullptr;

namespace
{
const char *sensorModeName()
{
    return sensorType == ORB_SLAM3::System::IMU_RGBD ? "rgbd_inertial" : "rgbd";
}
} // namespace

/* -------------------------------------------------------------------------- *
 * PUBLIC METHODS
 * -------------------------------------------------------------------------- */

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
                roomType == ORB_SLAM3::Room::roomVariant::ROOM;

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

void appendPassageMarkers(
    const std::vector<ORB_SLAM3::Passage *> &mappedPassages_in,
    const std::vector<ORB_SLAM3::Room *>    &mappedRooms_in,
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

    const auto appendProspectiveDeleteMarkers =
        [&msgTime_s_in,
         &structuralElementMarkerArray_out](const int prospectiveMarkerId)
    {
        visualization_msgs::msg::Marker deleteProspectiveRoomMarker;

        deleteProspectiveRoomMarker.header.frame_id = frameSE;
        deleteProspectiveRoomMarker.header.stamp    = msgTime_s_in;

        deleteProspectiveRoomMarker.ns = "prospectiveRoom";
        deleteProspectiveRoomMarker.id = prospectiveMarkerId;

        deleteProspectiveRoomMarker.action =
            visualization_msgs::msg::Marker::DELETE;
        structuralElementMarkerArray_out.markers.push_back(
            deleteProspectiveRoomMarker);

        visualization_msgs::msg::Marker deleteProspectiveRoomLabelMarker;

        deleteProspectiveRoomLabelMarker.header.frame_id = frameSE;
        deleteProspectiveRoomLabelMarker.header.stamp    = msgTime_s_in;

        deleteProspectiveRoomLabelMarker.ns = "prospectiveRoomLabel";
        deleteProspectiveRoomLabelMarker.id = prospectiveMarkerId;

        deleteProspectiveRoomLabelMarker.action =
            visualization_msgs::msg::Marker::DELETE;
        structuralElementMarkerArray_out.markers.push_back(
            deleteProspectiveRoomLabelMarker);

        visualization_msgs::msg::Marker deletePassageToProspectiveMarker;

        deletePassageToProspectiveMarker.header.frame_id = frameWorld;
        deletePassageToProspectiveMarker.header.stamp    = msgTime_s_in;

        deletePassageToProspectiveMarker.ns = "passageToProspective";
        deletePassageToProspectiveMarker.id = prospectiveMarkerId;

        deletePassageToProspectiveMarker.action =
            visualization_msgs::msg::Marker::DELETE;
        structuralElementMarkerArray_out.markers.push_back(
            deletePassageToProspectiveMarker);
    };

    /* Delete any prospective-room marker published previously whose room is no
     * longer shown this frame (promoted to a confirmed room or cleaned up).
     * The ids that survive this frame are re-recorded below. */
    std::unordered_set<int> currentProspectiveMarkerIds;

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

        /* Fixed bright blue color for passages (connecting rooms) */
        const float passageColourRed   = 0.0F;
        const float passageColourGreen = 0.3F;
        const float passageColourBlue  = 1.0F;

        const bool isPassageOpen = mappedPassage->isPassable();

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

        std::string passageLabel = "Passage#" +
                                   std::to_string(passageMarkerId) +
                                   (isPassageOpen ? " [open]" : " [blocked]");

        passageLabelMarker.text = passageLabel;

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

        /* ------------------------------------------------------------------ *
         * PROSPECTIVE ROOM VISUALIZATION
         * ------------------------------------------------------------------ *
         * Display a marker for the prospective room on the far side of passages
         * that have only one associated room. This provides situational
         * awareness that the semantic graph hypothesizes traversable space
         * beyond the opening.
         */
        if (mappedPassage->hasProspectiveRoom())
        {
            ORB_SLAM3::Room *p_prospectiveRoom =
                mappedPassage->getProspectiveRoom();
            if (p_prospectiveRoom != nullptr && !p_prospectiveRoom->isBad())
            {
                /* A prospective room is only the placeholder for the far side
                 * of the passage. Once the real far-side room has been
                 * detected (and is therefore drawn this frame from the room
                 * list), the placeholder must not be drawn as well - otherwise
                 * its label stacks on top of the confirmed room's label at the
                 * same position. */
                bool supersededByConfirmedRoom = false;
                for (ORB_SLAM3::Room *p_confirmedRoom : mappedRooms_in)
                {
                    if (p_confirmedRoom == nullptr ||
                        p_confirmedRoom->isBad() ||
                        p_confirmedRoom == p_prospectiveRoom ||
                        p_confirmedRoom->getRoomVariant() ==
                            ORB_SLAM3::Room::roomVariant::UNDEFINED)
                    {
                        continue;
                    }
                    if ((p_confirmedRoom->getCentroid().cast<double>() -
                         p_prospectiveRoom->getCentroid())
                            .norm() < 1.5)
                    {
                        supersededByConfirmedRoom = true;
                        break;
                    }
                }

                geometry_msgs::msg::PointStamped roomDisplayPoint_SE;
                geometry_msgs::msg::PointStamped roomDisplayPoint_world;

                if (!supersededByConfirmedRoom &&
                    getRoomDisplayPoints(p_prospectiveRoom,
                                         msgTime_s_in,
                                         roomDisplayPoint_SE,
                                         roomDisplayPoint_world))
                {
                    const int prospectiveRoomMarkerId =
                        static_cast<int>(p_prospectiveRoom->getId()) +
                        10000; // offset to avoid collision

                    currentProspectiveMarkerIds.insert(prospectiveRoomMarkerId);

                    /* Purple for all rooms, prospective (unconfirmed) included
                     */
                    const float prospectiveColourRed   = 0.6F;
                    const float prospectiveColourGreen = 0.0F;
                    const float prospectiveColourBlue  = 1.0F;

                    /* Prospective room node (semi-transparent cube) */
                    visualization_msgs::msg::Marker prospectiveRoomMarker;
                    prospectiveRoomMarker.header.frame_id = frameSE;
                    prospectiveRoomMarker.header.stamp    = msgTime_s_in;
                    prospectiveRoomMarker.ns              = "prospectiveRoom";
                    prospectiveRoomMarker.id = prospectiveRoomMarkerId;
                    prospectiveRoomMarker.type =
                        visualization_msgs::msg::Marker::CUBE;
                    prospectiveRoomMarker.action =
                        visualization_msgs::msg::Marker::ADD;
                    prospectiveRoomMarker.pose.position.x =
                        roomDisplayPoint_SE.point.x;
                    prospectiveRoomMarker.pose.position.y =
                        roomDisplayPoint_SE.point.y;
                    prospectiveRoomMarker.pose.position.z =
                        roomDisplayPoint_SE.point.z;
                    prospectiveRoomMarker.pose.orientation.x = 0.0;
                    prospectiveRoomMarker.pose.orientation.y = 0.0;
                    prospectiveRoomMarker.pose.orientation.z = 0.0;
                    prospectiveRoomMarker.pose.orientation.w = 1.0;
                    prospectiveRoomMarker.scale.x            = 0.30;
                    prospectiveRoomMarker.scale.y            = 0.30;
                    prospectiveRoomMarker.scale.z            = 0.30;
                    prospectiveRoomMarker.color.r = prospectiveColourRed;
                    prospectiveRoomMarker.color.g = prospectiveColourGreen;
                    prospectiveRoomMarker.color.b = prospectiveColourBlue;
                    prospectiveRoomMarker.color.a = 0.6F; // semi-transparent
                    prospectiveRoomMarker.lifetime =
                        rclcpp::Duration::from_seconds(0);
                    structuralElementMarkerArray_out.markers.push_back(
                        prospectiveRoomMarker);

                    /* Prospective room label */
                    visualization_msgs::msg::Marker prospectiveRoomLabelMarker;
                    prospectiveRoomLabelMarker.header.frame_id = frameSE;
                    prospectiveRoomLabelMarker.header.stamp    = msgTime_s_in;
                    prospectiveRoomLabelMarker.ns = "prospectiveRoomLabel";
                    prospectiveRoomLabelMarker.id = prospectiveRoomMarkerId;
                    prospectiveRoomLabelMarker.type =
                        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    prospectiveRoomLabelMarker.action =
                        visualization_msgs::msg::Marker::ADD;
                    prospectiveRoomLabelMarker.text =
                        p_prospectiveRoom->getName() + " [prospective]";
                    prospectiveRoomLabelMarker.pose.position.x =
                        roomDisplayPoint_SE.point.x;
                    prospectiveRoomLabelMarker.pose.position.y =
                        roomDisplayPoint_SE.point.y + textOffset_m;
                    prospectiveRoomLabelMarker.pose.position.z =
                        roomDisplayPoint_SE.point.z;
                    prospectiveRoomLabelMarker.pose.orientation.x = 0.0;
                    prospectiveRoomLabelMarker.pose.orientation.y = 0.0;
                    prospectiveRoomLabelMarker.pose.orientation.z = 0.0;
                    prospectiveRoomLabelMarker.pose.orientation.w = 1.0;
                    prospectiveRoomLabelMarker.scale.z            = 0.18;
                    prospectiveRoomLabelMarker.color.r = prospectiveColourRed;
                    prospectiveRoomLabelMarker.color.g = prospectiveColourGreen;
                    prospectiveRoomLabelMarker.color.b = prospectiveColourBlue;
                    prospectiveRoomLabelMarker.color.a = 0.9F;
                    prospectiveRoomLabelMarker.lifetime =
                        rclcpp::Duration::from_seconds(0);
                    structuralElementMarkerArray_out.markers.push_back(
                        prospectiveRoomLabelMarker);

                    /* Line from passage to prospective room */
                    visualization_msgs::msg::Marker passageToProspectiveMarker;
                    passageToProspectiveMarker.header.frame_id = frameWorld;
                    passageToProspectiveMarker.header.stamp    = msgTime_s_in;
                    passageToProspectiveMarker.ns = "passageToProspective";
                    passageToProspectiveMarker.id = prospectiveRoomMarkerId;
                    passageToProspectiveMarker.type =
                        visualization_msgs::msg::Marker::LINE_STRIP;
                    passageToProspectiveMarker.action =
                        visualization_msgs::msg::Marker::ADD;
                    passageToProspectiveMarker.pose.orientation.x = 0.0;
                    passageToProspectiveMarker.pose.orientation.y = 0.0;
                    passageToProspectiveMarker.pose.orientation.z = 0.0;
                    passageToProspectiveMarker.pose.orientation.w = 1.0;
                    passageToProspectiveMarker.scale.x = 0.03; // thin line
                    passageToProspectiveMarker.color.r = prospectiveColourRed;
                    passageToProspectiveMarker.color.g = prospectiveColourGreen;
                    passageToProspectiveMarker.color.b = prospectiveColourBlue;
                    passageToProspectiveMarker.color.a = 0.7F;
                    passageToProspectiveMarker.lifetime =
                        rclcpp::Duration::from_seconds(0);

                    geometry_msgs::msg::Point passagePt;
                    passagePt.x = passageDisplayPoint_world.point.x;
                    passagePt.y = passageDisplayPoint_world.point.y;
                    passagePt.z = passageDisplayPoint_world.point.z;

                    geometry_msgs::msg::Point roomPt;
                    roomPt.x = roomDisplayPoint_world.point.x;
                    roomPt.y = roomDisplayPoint_world.point.y;
                    roomPt.z = roomDisplayPoint_world.point.z;

                    passageToProspectiveMarker.points.push_back(passagePt);
                    passageToProspectiveMarker.points.push_back(roomPt);
                    structuralElementMarkerArray_out.markers.push_back(
                        passageToProspectiveMarker);
                }
            }
        }
    }

    /* Revoke markers whose prospective room is no longer published. */
    for (const int staleProspectiveMarkerId : s_publishedProspectiveMarkerIds)
    {
        if (currentProspectiveMarkerIds.count(staleProspectiveMarkerId) == 0U)
        {
            appendProspectiveDeleteMarkers(staleProspectiveMarkerId);
        }
    }

    s_publishedProspectiveMarkerIds = std::move(currentProspectiveMarkerIds);
}

/*!
 * @brief Computes the ordered horizontal corner polygon of a room boundary.
 *
 * Each wall is treated as a vertical line in the horizontal plane: the wall
 * plane is projected onto the plane orthogonal to the room ground normal to
 * obtain a two-dimensional supporting line. The walls are then ordered by the
 * angle of their centroid around the room centroid and consecutive supporting
 * lines are intersected. Recovering corners in this way reproduces the closure
 * the core boundary validator uses and ignores partial finite extents.
 *
 * @param[in] room_in Room whose boundary corners are required.
 *
 * @return Ordered world-frame corner points forming the closed boundary loop.
 *         The loop is empty when the room, its ground plane, or the wall set is
 *         degenerate, or when any consecutive wall pair is parallel.
 */
std::vector<Eigen::Vector3d> computeRoomCorners(const ORB_SLAM3::Room *room_in)
{
    std::vector<Eigen::Vector3d> corners_World_m;

    if (room_in == nullptr)
    {
        return corners_World_m;
    }

    const std::vector<ORB_SLAM3::Plane *> walls = room_in->getWalls();

    if (walls.size() < 3)
    {
        return corners_World_m;
    }

    /* Resolve the horizontal plane from the room's ground plane, falling back
     * to the world vertical axis when no ground surface is available. */
    Eigen::Vector3d groundNormal_World_m = Eigen::Vector3d::UnitZ();

    ORB_SLAM3::Plane *p_groundPlane = room_in->getGroundPlane();

    bool   hasFloorPlane = false;
    double floorHeight_m = 0.0;

    if (p_groundPlane != nullptr && !p_groundPlane->isBad())
    {
        const Eigen::Vector4d groundEquation_World =
            p_groundPlane->getGlobalEquation().coeffs();
        const double groundNormalNorm = groundEquation_World.head<3>().norm();

        if (groundEquation_World.allFinite() && groundNormalNorm > 1e-8)
        {
            groundNormal_World_m =
                groundEquation_World.head<3>() / groundNormalNorm;

            /* The ground plane is n . x + d = 0, so a point on the floor has
             * coordinate -d / norm along the ground normal. */
            hasFloorPlane = true;
            floorHeight_m = -groundEquation_World[3] / groundNormalNorm;
        }
    }

    const Eigen::Vector3d axisU_World_m =
        groundNormal_World_m.unitOrthogonal().normalized();
    const Eigen::Vector3d axisV_World_m =
        groundNormal_World_m.cross(axisU_World_m).normalized();

    const Eigen::Vector3d roomCentroid_World_m = room_in->getCentroid();

    if (!roomCentroid_World_m.allFinite())
    {
        return corners_World_m;
    }

    /* Record the two-dimensional wall with its horizontal normal and the
     * projected centroid used for angular ordering. */
    struct HorizontalWall
    {
        Eigen::Vector2d normal2D   = Eigen::Vector2d::Zero();
        double          offset2d   = 0.0;
        Eigen::Vector2d centroid2D = Eigen::Vector2d::Zero();
    };

    std::vector<HorizontalWall> horizontalWalls;
    horizontalWalls.reserve(walls.size());

    for (ORB_SLAM3::Plane *p_wall : walls)
    {
        if (p_wall == nullptr || p_wall->isBad())
        {
            continue;
        }

        const Eigen::Vector4d wallEquation_World =
            p_wall->getGlobalEquation().coeffs();
        const double wallNormalNorm = wallEquation_World.head<3>().norm();

        if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
        {
            continue;
        }

        const Eigen::Vector3d wallNormal_World_m =
            wallEquation_World.head<3>() / wallNormalNorm;

        /* Keep only the horizontal component of the wall normal. */
        Eigen::Vector3d horizontalNormal_World_m =
            wallNormal_World_m -
            wallNormal_World_m.dot(groundNormal_World_m) * groundNormal_World_m;

        if (horizontalNormal_World_m.norm() < 1e-8)
        {
            continue;
        }

        horizontalNormal_World_m.normalize();

        const Eigen::Vector3d wallCentroid2D_World_m = p_wall->getCentroid();

        if (!wallCentroid2D_World_m.allFinite())
        {
            continue;
        }

        /* A point on the vertical wall projected onto the horizontal plane. */
        Eigen::Vector3d wallPoint_World_m =
            wallCentroid2D_World_m -
            wallCentroid2D_World_m.dot(groundNormal_World_m) *
                groundNormal_World_m;

        const Eigen::Vector2d normal2D(
            horizontalNormal_World_m.dot(axisU_World_m),
            horizontalNormal_World_m.dot(axisV_World_m));

        if (normal2D.norm() < 1e-8)
        {
            continue;
        }

        HorizontalWall horizontalWall;
        horizontalWall.normal2D = normal2D;
        horizontalWall.offset2d =
            normal2D.dot(Eigen::Vector2d(wallPoint_World_m.dot(axisU_World_m),
                                         wallPoint_World_m.dot(axisV_World_m)));
        horizontalWall.centroid2D = {wallCentroid2D_World_m.dot(axisU_World_m),
                                     wallCentroid2D_World_m.dot(axisV_World_m)};

        horizontalWalls.push_back(horizontalWall);
    }

    if (horizontalWalls.size() < 3)
    {
        return corners_World_m;
    }

    const Eigen::Vector2d roomCentroid2D(
        roomCentroid_World_m.dot(axisU_World_m),
        roomCentroid_World_m.dot(axisV_World_m));

    /* Order walls around the room centroid so adjacent walls are consecutive.
     */
    std::sort(horizontalWalls.begin(),
              horizontalWalls.end(),
              [&roomCentroid2D](const HorizontalWall &firstWall,
                                const HorizontalWall &secondWall)
              {
                  const Eigen::Vector2d firstMidpoint =
                      firstWall.centroid2D - roomCentroid2D;
                  const Eigen::Vector2d secondMidpoint =
                      secondWall.centroid2D - roomCentroid2D;

                  return std::atan2(firstMidpoint.y(), firstMidpoint.x()) <
                         std::atan2(secondMidpoint.y(), secondMidpoint.x());
              });

    /* Intersect consecutive supporting lines to obtain the shared corners. */
    std::vector<Eigen::Vector2d> corners2D;
    corners2D.reserve(horizontalWalls.size());

    for (std::size_t wallIndex = 0U; wallIndex < horizontalWalls.size();
         ++wallIndex)
    {
        const HorizontalWall &currentWall = horizontalWalls[wallIndex];
        const HorizontalWall &nextWall =
            horizontalWalls[(wallIndex + 1U) % horizontalWalls.size()];

        /* The wall line is normal2D . x = offset2d. Intersect the two
         * supporting lines by solving the 2x2 linear system N c = offsets. */
        Eigen::Matrix2d lineMatrix;
        lineMatrix(0, 0) = currentWall.normal2D.x();
        lineMatrix(0, 1) = currentWall.normal2D.y();
        lineMatrix(1, 0) = nextWall.normal2D.x();
        lineMatrix(1, 1) = nextWall.normal2D.y();

        const double determinant = lineMatrix(0, 0) * lineMatrix(1, 1) -
                                   lineMatrix(0, 1) * lineMatrix(1, 0);

        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-8)
        {
            /* Parallel consecutive walls have no well-defined corner. */
            corners_World_m.clear();
            return corners_World_m;
        }

        const Eigen::Vector2d offsets(currentWall.offset2d, nextWall.offset2d);

        const Eigen::Vector2d corner2D = lineMatrix.inverse() * offsets;

        if (!corner2D.allFinite())
        {
            corners_World_m.clear();
            return corners_World_m;
        }

        corners2D.push_back(corner2D);
    }

    /* Mark the room-to-floor boundary at the floor height when the floor plane
     * is available, falling back to the room centroid height otherwise. */
    const double boundaryHeight_m =
        hasFloorPlane ? floorHeight_m
                      : roomCentroid_World_m.dot(groundNormal_World_m);

    corners_World_m.reserve(corners2D.size());

    for (const Eigen::Vector2d &corner2D : corners2D)
    {
        corners_World_m.emplace_back(
            corner2D.x() * axisU_World_m[0] + corner2D.y() * axisV_World_m[0] +
                boundaryHeight_m * groundNormal_World_m[0],
            corner2D.x() * axisU_World_m[1] + corner2D.y() * axisV_World_m[1] +
                boundaryHeight_m * groundNormal_World_m[1],
            corner2D.x() * axisU_World_m[2] + corner2D.y() * axisV_World_m[2] +
                boundaryHeight_m * groundNormal_World_m[2]);
    }

    return corners_World_m;
}

void appendRoomMarkers(
    const std::vector<ORB_SLAM3::Room *>  &mappedRooms_in,
    const std::vector<ORB_SLAM3::Floor *> &mappedFloors_in,
    const rclcpp::Time                    &msgTime_s_in,
    visualization_msgs::msg::MarkerArray  &structuralElementMarkerArray_out)
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

        visualization_msgs::msg::Marker deleteRoomCompleteMarker;

        deleteRoomCompleteMarker.header.frame_id = frameWorld;
        deleteRoomCompleteMarker.header.stamp    = msgTime_s_in;

        deleteRoomCompleteMarker.ns = "room_complete";
        deleteRoomCompleteMarker.id = roomId;

        deleteRoomCompleteMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteRoomCompleteMarker);

        visualization_msgs::msg::Marker deleteRoomCompleteToFloorMarker;

        deleteRoomCompleteToFloorMarker.header.frame_id = frameWorld;
        deleteRoomCompleteToFloorMarker.header.stamp    = msgTime_s_in;

        deleteRoomCompleteToFloorMarker.ns = "room_complete_to_floor";
        deleteRoomCompleteToFloorMarker.id = roomId;

        deleteRoomCompleteToFloorMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteRoomCompleteToFloorMarker);
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
            roomType == ORB_SLAM3::Room::roomVariant::ROOM;

        /* Remove bad and provisional structural elements from RViz */
        if (mappedRoom->isBad() || !isConfirmedRoom)
        {
            appendRoomDeleteMarkers(roomMarkerId);

            continue;
        }

        /* Select the displayed room colour */
        float roomColourRed   = 0.6F;
        float roomColourGreen = 0.0F;
        float roomColourBlue  = 1.0F;

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

        std::string boundaryStatusLabel;

        switch (mappedRoom->getBoundaryStatus())
        {
        case ORB_SLAM3::Room::BoundaryStatus::UNOBSERVED:
            boundaryStatusLabel = " [unobserved]";
            break;
        case ORB_SLAM3::Room::BoundaryStatus::INCOMPLETE:
            boundaryStatusLabel = " [incomplete]";
            break;
        case ORB_SLAM3::Room::BoundaryStatus::COMPLETE:
            boundaryStatusLabel = " [complete]";
            break;
        case ORB_SLAM3::Room::BoundaryStatus::CONFLICTING:
            boundaryStatusLabel = " [conflicting]";
            break;
        }

        /*
         * Boundary maturity is diagnostic state, not a passage gate. Keeping
         * it in the room label makes incomplete but traversable observations
         * explicit without changing the semantic graph topology.
         */
        roomLabelMarker.text = mappedRoom->getName() + boundaryStatusLabel;

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
         * ROOM-COMPLETE VISUALIZATION: LINE FROM ROOM TO FLOOR
         * ------------------------------------------------------------------ */

        /* Helper to find the floor that owns this room. */
        auto findRoomFloor = [&mappedFloors_in](const ORB_SLAM3::Room *room_in)
            -> const ORB_SLAM3::Floor *
        {
            for (const ORB_SLAM3::Floor *floor : mappedFloors_in)
            {
                if (floor == nullptr)
                {
                    continue;
                }
                const std::vector<ORB_SLAM3::Room *> &floorRooms =
                    floor->getRooms();
                if (std::find(floorRooms.begin(), floorRooms.end(), room_in) !=
                    floorRooms.end())
                {
                    return floor;
                }
            }
            return nullptr;
        };

        const ORB_SLAM3::Floor *associatedFloor = nullptr;
        if (mappedRoom->isBoundaryComplete())
        {
            associatedFloor = findRoomFloor(mappedRoom);
        }

        if (associatedFloor != nullptr)
        {
            visualization_msgs::msg::Marker roomCompleteMarker;

            roomCompleteMarker.header.frame_id = frameWorld;
            roomCompleteMarker.header.stamp    = msgTime_s_in;

            roomCompleteMarker.ns = "room_complete_to_floor";
            roomCompleteMarker.id = roomMarkerId;

            roomCompleteMarker.type =
                visualization_msgs::msg::Marker::LINE_LIST;

            roomCompleteMarker.action = visualization_msgs::msg::Marker::ADD;

            roomCompleteMarker.pose.orientation.x = 0.0;
            roomCompleteMarker.pose.orientation.y = 0.0;
            roomCompleteMarker.pose.orientation.z = 0.0;
            roomCompleteMarker.pose.orientation.w = 1.0;

            roomCompleteMarker.scale.x = 0.05;

            roomCompleteMarker.color.r = 0.0F;
            roomCompleteMarker.color.g = 1.0F;
            roomCompleteMarker.color.b = 0.0F;
            roomCompleteMarker.color.a = 0.8F;

            roomCompleteMarker.lifetime = rclcpp::Duration::from_seconds(1.0);

            const Eigen::Vector3d roomCentroid_World_m =
                mappedRoom->getCentroid();
            const Eigen::Vector3d floorCentroid_World_m =
                associatedFloor->getCentroid();

            if (roomCentroid_World_m.allFinite() &&
                floorCentroid_World_m.allFinite())
            {
                geometry_msgs::msg::Point roomPoint;
                roomPoint.x = roomCentroid_World_m.x();
                roomPoint.y = roomCentroid_World_m.y();
                roomPoint.z = roomCentroid_World_m.z();

                geometry_msgs::msg::Point floorPoint;
                floorPoint.x = floorCentroid_World_m.x();
                floorPoint.y = floorCentroid_World_m.y();
                floorPoint.z = floorCentroid_World_m.z();

                roomCompleteMarker.points.push_back(roomPoint);
                roomCompleteMarker.points.push_back(floorPoint);

                structuralElementMarkerArray_out.markers.push_back(
                    roomCompleteMarker);
            }
        }
        else
        {
            /* Remove a previously published green line when the room is not
             * validated as COMPLETE or has no associated floor. */
            visualization_msgs::msg::Marker staleRoomCompleteMarker;

            staleRoomCompleteMarker.header.frame_id = frameWorld;
            staleRoomCompleteMarker.header.stamp    = msgTime_s_in;

            staleRoomCompleteMarker.ns = "room_complete_to_floor";
            staleRoomCompleteMarker.id = roomMarkerId;

            staleRoomCompleteMarker.action =
                visualization_msgs::msg::Marker::DELETE;

            structuralElementMarkerArray_out.markers.push_back(
                staleRoomCompleteMarker);
        }

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

            const Eigen::Vector3d wallCentroid_BC_m =
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

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframeVector_in)
{
    /* Iterate through keyframes and clear the cls point clouds */
    for (auto &keyframe : keyframeVector_in)
    {
        keyframe->clearClsClouds();
    }
}

cv::Mat convertSE3fToCvMat(const Sophus::SE3f &transformation_SE3f_in)
{
    /* Extract the homogeneous Eigen transformation matrix */
    const Eigen::Matrix4f transformationMatrix_Eigen =
        transformation_SE3f_in.matrix();

    /* Convert the Eigen matrix into an OpenCV matrix */
    cv::Mat transformationMatrix_OpenCV;

    cv::eigen2cv(transformationMatrix_Eigen, transformationMatrix_OpenCV);

    return transformationMatrix_OpenCV;
}

std::pair<double, std::vector<ORB_SLAM3::Marker *>>
    findNearestMarker(double frameTimestamp_in)
{
    /* Init variable of the minimum time difference */
    double minTimeDifference = 100;

    /* Init a variable which will be used to find best match to marker */
    std::vector<ORB_SLAM3::Marker *> matchedMarkers;

    /* Loop through the markersBuffer */
    for (const auto &markers : markersBuffer)
    {
        /* Find the time difference */
        double timeDifference = markers[0]->getTime() - frameTimestamp_in;

        /* If better match found, update */
        if (timeDifference < minTimeDifference)
        {
            matchedMarkers    = markers;
            minTimeDifference = timeDifference;
        }
    }

    /* Return the minimum time difference and best matched marker */
    return std::make_pair(minTimeDifference, matchedMarkers);
}

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
    const Eigen::Vector3d passageCentroid = passage_in->getCentroid();

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

bool getRoomDisplayPoints(ORB_SLAM3::Room                  *room_in,
                          const rclcpp::Time               &msgTime_in,
                          geometry_msgs::msg::PointStamped &roomPointSE_out,
                          geometry_msgs::msg::PointStamped &roomPointWorld_out)
{
    /* Confirm the required objects are valid */
    if (room_in == nullptr || tfBuffer_ == nullptr)
    {
        return false;
    }

    /* The room centroid is stored in the world frame */
    const Eigen::Vector3d roomCentroid = room_in->getCentroid();

    if (!roomCentroid.allFinite())
    {
        return false;
    }

    /* Create the physical room point in the world frame */
    roomPointWorld_out.header.stamp    = msgTime_in;
    roomPointWorld_out.header.frame_id = frameWorld;
    roomPointWorld_out.point.x         = roomCentroid.x();
    roomPointWorld_out.point.y         = roomCentroid.y();
    roomPointWorld_out.point.z         = roomCentroid.z();

    try
    {
        /* Transform the world-frame centroid into frameSE */
        const geometry_msgs::msg::TransformStamped worldToSE =
            tfBuffer_->lookupTransform(frameSE,
                                       frameWorld,
                                       msgTime_in,
                                       rclcpp::Duration::from_seconds(0.1));

        tf2::doTransform(roomPointWorld_out, roomPointSE_out, worldToSE);
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Room display transform from '%s' to '%s' failed: %s",
                    frameWorld.c_str(),
                    frameSE.c_str(),
                    exception.what());

        return false;
    }

    roomPointSE_out.header.stamp    = msgTime_in;
    roomPointSE_out.header.frame_id = frameSE;

    return true;
}

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

sensor_msgs::msg::PointCloud2
    mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints_in,
                         rclcpp::Time                       msgTime_in)
{
    const int                     numChannels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    std::string                   channelId[] = {"x", "y", "z"};

    /* Set the attributes of the point cloud */
    cloud.header.stamp    = msgTime_in;
    cloud.header.frame_id = frameWorld;
    cloud.height          = 1;
    cloud.is_dense        = false;
    cloud.is_bigendian    = false;
    cloud.width           = mapPoints_in.size();
    cloud.point_step      = numChannels * sizeof(float);
    cloud.row_step        = cloud.point_step * cloud.width;
    cloud.fields.resize(numChannels);

    // Set the fields of the point cloud
    for (int idx = 0; idx < numChannels; idx++)
    {
        cloud.fields[idx].count    = 1;
        cloud.fields[idx].name     = channelId[idx];
        cloud.fields[idx].offset   = idx * sizeof(float);
        cloud.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    // Set the data of the point cloud
    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloudDataPtr = &(cloud.data[0]);

    // Populate the point cloud with the map points
    for (unsigned int idx = 0; idx < cloud.width; idx++)
    {
        if (mapPoints_in[idx] && !mapPoints_in[idx]->isBad())
        {
            Eigen::Vector3d P3Dw =
                mapPoints_in[idx]->GetWorldPos().cast<double>();
            tf2::Vector3 pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());
            float        dataArray[numChannels] = {
                static_cast<float>(pointTranslation.x()),
                static_cast<float>(pointTranslation.y()),
                static_cast<float>(pointTranslation.z())};
            memcpy(cloudDataPtr + (idx * cloud.point_step),
                   dataArray,
                   numChannels * sizeof(float));
        }
        else
        {
            /* Mark skipped map points as NaN instead of leaving a phantom
               point at the origin; is_dense = false declares invalid values */
            float dataArray[numChannels] = {
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()};
            memcpy(cloudDataPtr + (idx * cloud.point_step),
                   dataArray,
                   numChannels * sizeof(float));
        }
    }

    return cloud;
}

void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *> wallsList_in,
                           rclcpp::Time                    msgTime_s_in)
{
    /* Variables */
    vs_graphs::msg::VSGraphsAllWallsData wallDataMsg;

    /* Fill the data message with wall information */
    wallDataMsg.header.stamp    = msgTime_s_in;
    wallDataMsg.header.frame_id = frameWorld;

    /* Fill in the walls data for each wall in vector */
    for (const auto &wall : wallsList_in)
    {
        if (!wall ||
            wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
            continue;

        /* Init variable of the lenfth of the wall */
        float length = 0.0f;

        /* Get the point clouds for the wall */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud =
            wall->getMapClouds();

        /* Calculate the length of the wall */
        if (wallCloud && wallCloud->points.size() > 1)
        {
            /* Create position vector of the start point */
            Eigen::Vector3f startPoint(wallCloud->points.front().x,
                                       wallCloud->points.front().y,
                                       wallCloud->points.front().z);

            /* Create position vector of the end point */
            Eigen::Vector3f endPoint(wallCloud->points.back().x,
                                     wallCloud->points.back().y,
                                     wallCloud->points.back().z);

            /*!
             * Calculate the length of the wall by finding the distance between
             * the first and last points
             */
            length = (endPoint - startPoint).norm();
        }

        /* Fill the wall data */
        vs_graphs::msg::VSGraphsWallData wallData;
        wallData.length     = length;
        wallData.id         = wall->getId();
        wallData.centroid.x = wall->getCentroid().x();
        wallData.centroid.y = wall->getCentroid().y();
        wallData.centroid.z = wall->getCentroid().z();
        wallData.normal.x   = wall->getGlobalEquation().normal().x();
        wallData.normal.y   = wall->getGlobalEquation().normal().y();
        wallData.normal.z   = wall->getGlobalEquation().normal().z();

        /* Add the wall to the message */
        wallDataMsg.walls.push_back(wallData);
    }

    /* Publish all mapped walls */
    pubAllWalls_new->publish(wallDataMsg);
}

void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> allMapPoints_in,
                      rclcpp::Time                       msgTime_s_in)
{
    /* Map point cloud */
    sensor_msgs::msg::PointCloud2 cloud =
        mapPointToPointcloud(allMapPoints_in, msgTime_s_in);

    /* Publish point cloud */
    pubAllMappoints->publish(cloud);
}

void publishBodyOdometry(const Sophus::SE3f    &robotPose_BodToWorld_in,
                         const Eigen::Vector3f &linearVelocity_World_mps_in,
                         const Eigen::Vector3f &angularVelocity_Bod_radps_in,
                         const rclcpp::Time    &msgTims_s_in)
{
    /* Confirm that the odometry publisher has been initialised */
    if (pubOdometry == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish body odometry: publisher is not initialised.");

        return;
    }

    /* Extract the body position and orientation once */
    const Eigen::Vector3f position_World_m =
        robotPose_BodToWorld_in.translation();
    const Eigen::Quaternionf orientation_WorldToBod =
        robotPose_BodToWorld_in.unit_quaternion();

    /* Initialise the odometry message */
    nav_msgs::msg::Odometry odometryMessage;

    odometryMessage.header.stamp    = msgTims_s_in;
    odometryMessage.header.frame_id = frameWorld;
    odometryMessage.child_frame_id  = frameImu;

    /* Set the body position in the world frame */
    odometryMessage.pose.pose.position.x = position_World_m.x();
    odometryMessage.pose.pose.position.y = position_World_m.y();
    odometryMessage.pose.pose.position.z = position_World_m.z();

    /* Set the body orientation relative to the world frame */
    odometryMessage.pose.pose.orientation.x = orientation_WorldToBod.x();
    odometryMessage.pose.pose.orientation.y = orientation_WorldToBod.y();
    odometryMessage.pose.pose.orientation.z = orientation_WorldToBod.z();
    odometryMessage.pose.pose.orientation.w = orientation_WorldToBod.w();

    /* Set the body linear velocity */
    odometryMessage.twist.twist.linear.x = linearVelocity_World_mps_in.x();
    odometryMessage.twist.twist.linear.y = linearVelocity_World_mps_in.y();
    odometryMessage.twist.twist.linear.z = linearVelocity_World_mps_in.z();

    /* Set the body angular velocity */
    odometryMessage.twist.twist.angular.x = angularVelocity_Bod_radps_in.x();
    odometryMessage.twist.twist.angular.y = angularVelocity_Bod_radps_in.y();
    odometryMessage.twist.twist.angular.z = angularVelocity_Bod_radps_in.z();

    /* Publish the completed odometry message */
    pubOdometry->publish(odometryMessage);
}

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

void publishFramePointCloud(const Sophus::SE3f &cameraPose_CameraToWorld_in,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                                               &pointCloudCameraMessage_in,
                            const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the point-cloud publisher has been initialised */
    if (p_voxbloxInputPointCloudPublisher == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish Voxblox input cloud: publisher is not "
                    "initialised.");

        return;
    }

    /* Confirm that a valid input point-cloud message was supplied */
    if (pointCloudCameraMessage_in == nullptr)
    {
        return;
    }

    /* Confirm that the camera transformation is valid */
    if (!cameraPose_CameraToWorld_in.translation().allFinite() ||
        !cameraPose_CameraToWorld_in.rotationMatrix().allFinite())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish Voxblox input cloud: camera pose contains "
                    "non-finite values.");

        return;
    }

    /* Return when the input point cloud contains no points */
    if (pointCloudCameraMessage_in->width == 0 ||
        pointCloudCameraMessage_in->height == 0 ||
        pointCloudCameraMessage_in->data.empty())
    {
        return;
    }

    /*!
     * Keep the points in the physical sensor frame. Voxblox raycasts from the
     * origin of the message frame, so pre-transforming the points and labelling
     * them as world-frame data would incorrectly cast every ray from the world
     * origin.
     */
    sensor_msgs::msg::PointCloud2 pointCloud_cameraMessage =
        *pointCloudCameraMessage_in;

    /* Set the output message metadata */
    pointCloud_cameraMessage.header.stamp = msgTime_s_in;

    pointCloud_cameraMessage.header.frame_id = frameCamera;

    /* Publish after the matching camera transform has been broadcast. */
    p_voxbloxInputPointCloudPublisher->publish(pointCloud_cameraMessage);
}

void publishFreeSpaceClusters(
    const std::vector<std::vector<Eigen::Vector3d>> &freeSpaceClusters_World_in,
    const rclcpp::Time                              &msgTime_s_in)
{
    /* Confirm that the publisher has been initialised */
    if (pubFreespaceCluster == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish free-space clusters: publisher is not "
                    "initialised.");

        return;
    }

    /* Return when there are no free-space clusters to publish */
    if (freeSpaceClusters_World_in.empty())
    {
        return;
    }

    /*!
     * Fixed RGB colours used to distinguish neighbouring free-space clusters.
     * Colours are reused when the number of clusters exceeds the palette size.
     */
    static constexpr std::array<std::array<std::uint8_t, 3>, 7>
        clusterColourPalette = {{{{255, 0, 0}},
                                 {{0, 255, 0}},
                                 {{0, 0, 255}},
                                 {{255, 255, 0}},
                                 {{0, 255, 255}},
                                 {{255, 0, 255}},
                                 {{128, 0, 0}}}};

    /* Calculate the total number of points so memory can be reserved once */
    std::size_t totalPointCount = 0;

    for (const std::vector<Eigen::Vector3d> &cluster :
         freeSpaceClusters_World_in)
    {
        totalPointCount += cluster.size();
    }

    if (totalPointCount == 0)
    {
        return;
    }

    /* Initialise the combined coloured free-space point cloud */
    pcl::PointCloud<pcl::PointXYZRGB> freeSpacePointCloud_world;

    freeSpacePointCloud_world.points.reserve(totalPointCount);

    /* Convert every free-space cluster into coloured PCL points */
    for (std::size_t clusterIndex = 0;
         clusterIndex < freeSpaceClusters_World_in.size();
         clusterIndex++)
    {
        const std::vector<Eigen::Vector3d> &clusterPoints_world =
            freeSpaceClusters_World_in[clusterIndex];

        const std::array<std::uint8_t, 3> &clusterColour =
            clusterColourPalette[clusterIndex % clusterColourPalette.size()];

        for (const Eigen::Vector3d &point_world_m : clusterPoints_world)
        {
            /* Ignore invalid points */
            if (!point_world_m.allFinite())
            {
                continue;
            }

            pcl::PointXYZRGB colouredPoint_world;

            colouredPoint_world.x = static_cast<float>(point_world_m.x());
            colouredPoint_world.y = static_cast<float>(point_world_m.y());
            colouredPoint_world.z = static_cast<float>(point_world_m.z());

            colouredPoint_world.r = clusterColour[0];
            colouredPoint_world.g = clusterColour[1];
            colouredPoint_world.b = clusterColour[2];

            freeSpacePointCloud_world.points.push_back(colouredPoint_world);
        }
    }

    /* Return when all supplied points were invalid */
    if (freeSpacePointCloud_world.empty())
    {
        return;
    }

    /* Complete the PCL metadata */
    freeSpacePointCloud_world.width =
        static_cast<std::uint32_t>(freeSpacePointCloud_world.points.size());

    freeSpacePointCloud_world.height = 1;

    freeSpacePointCloud_world.is_dense = true;

    /* Convert the PCL point cloud into a ROS message */
    sensor_msgs::msg::PointCloud2 freeSpacePointCloudMessage_world;

    pcl::toROSMsg(freeSpacePointCloud_world, freeSpacePointCloudMessage_world);

    /* Set the output message metadata */
    freeSpacePointCloudMessage_world.header.stamp = msgTime_s_in;

    freeSpacePointCloudMessage_world.header.frame_id = frameWorld;

    /* Publish the combined free-space cluster point cloud */
    pubFreespaceCluster->publish(freeSpacePointCloudMessage_world);
}

void publishKeyFrameImages(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                       &msgTime_s_in)
{
    /* Confirm that the keyframe-image publisher has been initialised */
    if (pubKFImage == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish keyframe images: publisher is not initialised.");

        return;
    }

    /* Inspect every available keyframe */
    for (ORB_SLAM3::KeyFrame *keyFrame : keyFrames_in)
    {
        /* Skip invalid keyframe pointers */
        if (keyFrame == nullptr)
        {
            continue;
        }

        /* Skip keyframes that have already been published */
        if (keyFrame->isPublished)
        {
            continue;
        }

        /* Skip keyframes that do not contain a valid image */
        if (keyFrame->mImage.empty())
        {
            continue;
        }

        /* Initialise the common ROS message header */
        std_msgs::msg::Header messageHeader;

        messageHeader.stamp = msgTime_s_in;

        messageHeader.frame_id = frameWorld;

        /* Convert the OpenCV keyframe image into a ROS image message */
        const sensor_msgs::msg::Image::SharedPtr keyFrameImageMessage =
            cv_bridge::CvImage(messageHeader, "bgr8", keyFrame->mImage)
                .toImageMsg();

        /* Confirm that the image conversion succeeded */
        if (keyFrameImageMessage == nullptr)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Failed to convert KeyFrame#%lu image into a ROS message.",
                static_cast<unsigned long>(keyFrame->mnId));

            continue;
        }

        /* Create the persistent keyframe identifier message */
        std_msgs::msg::UInt64 keyFrameIdMessage;

        keyFrameIdMessage.data = keyFrame->mnId;

        /* Package the keyframe identifier and image for segmentation */
        segmenter_ros::msg::VSGraphDataMsg segmentationInputMessage;

        segmentationInputMessage.header          = messageHeader;
        segmentationInputMessage.key_frame_id    = keyFrameIdMessage;
        segmentationInputMessage.key_frame_image = *keyFrameImageMessage;

        /* Publish the keyframe image for semantic segmentation */
        pubKFImage->publish(segmentationInputMessage);

        /* Prevent the same keyframe from being published again */
        keyFrame->isPublished = true;
    }
}

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
            p_slamSystem->GetKeyFramePose(keyFrame);

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

void publishPlanes(const std::vector<ORB_SLAM3::Plane *> &mappedPlanes_in,
                   const rclcpp::Time                    &msgTime_s_in)
{
    /* Return when neither required publisher has been initialised */
    if (pubBuildingComponents == nullptr && pubPlaneLabel == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish planes: plane publishers are not initialised.");

        return;
    }

    /* Return when there are no mapped planes to publish */
    if (mappedPlanes_in.empty())
    {
        return;
    }

    /* Limit expensive plane publication to once every three seconds */
    constexpr double planePublicationPeriod_s = 3.0;

    if (lastPlanePublishTime.nanoseconds() != 0 &&
        (msgTime_s_in - lastPlanePublishTime).seconds() <
            planePublicationPeriod_s)
    {
        return;
    }

    lastPlanePublishTime = msgTime_s_in;

    /* Initialise the combined building-component point cloud */
    pcl::PointCloud<pcl::PointXYZRGB> buildingComponentPointCloud_BC;

    /* Initialise the plane visualisation marker array */
    visualization_msgs::msg::MarkerArray planeVisualizationArray;

    planeVisualizationArray.markers.reserve(mappedPlanes_in.size() * 2);

    /*
     * Determine the vertical direction from the best-supported ground plane.
     * This remains valid when the map frame is not aligned with a hard-coded
     * Cartesian up axis.
     */
    Eigen::Vector3d groundNormal_BC = Eigen::Vector3d::Zero();

    std::size_t groundSupportPointCount = 0U;

    for (ORB_SLAM3::Plane *p_mappedPlane : mappedPlanes_in)
    {
        if (p_mappedPlane == nullptr || p_mappedPlane->isBad() ||
            p_mappedPlane->getPlaneType() !=
                ORB_SLAM3::Plane::planeVariant::GROUND)
        {
            continue;
        }

        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_groundCloud_BC =
            p_mappedPlane->getMapClouds();
        const Eigen::Vector3d candidateGroundNormal_BC =
            p_mappedPlane->getGlobalEquation().normal();

        if (p_groundCloud_BC == nullptr ||
            p_groundCloud_BC->size() <= groundSupportPointCount ||
            !candidateGroundNormal_BC.allFinite() ||
            candidateGroundNormal_BC.norm() < 1e-9)
        {
            continue;
        }

        groundNormal_BC         = candidateGroundNormal_BC.normalized();
        groundSupportPointCount = p_groundCloud_BC->size();
    }

    /* Visualisation constants */
    constexpr double planeLabelNormalOffset_m   = 0.12;
    constexpr double planeLabelVerticalOffset_m = 0.15;
    constexpr double planeNormalLength_m        = 0.45;
    constexpr double normalVectorTolerance      = 1e-9;

    /* Process every mapped plane */
    for (ORB_SLAM3::Plane *mappedPlane : mappedPlanes_in)
    {
        /* Skip invalid planes */
        if (mappedPlane == nullptr || mappedPlane->isBad())
        {
            continue;
        }

        /* Extract the semantic plane type once */
        const ORB_SLAM3::Plane::planeVariant planeType =
            mappedPlane->getPlaneType();

        /* Skip planes that have not received a semantic type */
        if (planeType == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the plane point cloud */
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloud_BC =
            mappedPlane->getMapClouds();

        /* Skip planes without any mapped points */
        if (planePointCloud_BC == nullptr || planePointCloud_BC->empty())
        {
            continue;
        }

        /* Extract and validate the plane centroid */
        const Eigen::Vector3d planeCentroid_BC_m = mappedPlane->getCentroid();

        if (!planeCentroid_BC_m.allFinite())
        {
            continue;
        }

        /* Extract and validate the plane normal */
        Eigen::Vector3d planeNormal_BC =
            mappedPlane->getGlobalEquation().normal();

        if (!planeNormal_BC.allFinite() ||
            planeNormal_BC.norm() < normalVectorTolerance)
        {
            continue;
        }

        /*!
         * Normalise the plane normal so the displayed arrow always has the
         * requested physical length.
         */
        planeNormal_BC.normalize();

        /* Count finite points and measure wall width and height. */
        std::size_t finiteSupportPointCount = 0U;

        double wallWidth_m  = 0.0;
        double wallHeight_m = 0.0;

        bool validWallDimensions = false;

        if (planeType == ORB_SLAM3::Plane::planeVariant::WALL &&
            groundNormal_BC.norm() >= normalVectorTolerance)
        {
            Eigen::Vector3d wallWidthAxis_BC =
                groundNormal_BC.cross(planeNormal_BC);

            if (wallWidthAxis_BC.norm() >= normalVectorTolerance)
            {
                wallWidthAxis_BC.normalize();

                double minimumWidthCoordinate_m =
                    std::numeric_limits<double>::max();
                double maximumWidthCoordinate_m =
                    std::numeric_limits<double>::lowest();
                double minimumHeightCoordinate_m =
                    std::numeric_limits<double>::max();
                double maximumHeightCoordinate_m =
                    std::numeric_limits<double>::lowest();

                for (const pcl::PointXYZRGBA &point_BC_m :
                     planePointCloud_BC->points)
                {
                    if (!std::isfinite(point_BC_m.x) ||
                        !std::isfinite(point_BC_m.y) ||
                        !std::isfinite(point_BC_m.z))
                    {
                        continue;
                    }

                    const Eigen::Vector3d position_BC_m(point_BC_m.x,
                                                        point_BC_m.y,
                                                        point_BC_m.z);
                    const double          widthCoordinate_m =
                        wallWidthAxis_BC.dot(position_BC_m);
                    const double heightCoordinate_m =
                        groundNormal_BC.dot(position_BC_m);

                    minimumWidthCoordinate_m =
                        std::min(minimumWidthCoordinate_m, widthCoordinate_m);
                    maximumWidthCoordinate_m =
                        std::max(maximumWidthCoordinate_m, widthCoordinate_m);
                    minimumHeightCoordinate_m =
                        std::min(minimumHeightCoordinate_m, heightCoordinate_m);
                    maximumHeightCoordinate_m =
                        std::max(maximumHeightCoordinate_m, heightCoordinate_m);
                    finiteSupportPointCount++;
                }

                wallWidth_m =
                    maximumWidthCoordinate_m - minimumWidthCoordinate_m;
                wallHeight_m =
                    maximumHeightCoordinate_m - minimumHeightCoordinate_m;
                validWallDimensions =
                    finiteSupportPointCount > 0U &&
                    std::isfinite(wallWidth_m) && wallWidth_m > 0.0 &&
                    std::isfinite(wallHeight_m) && wallHeight_m > 0.0;
            }
        }

        if (finiteSupportPointCount == 0U)
        {
            finiteSupportPointCount = static_cast<std::size_t>(
                std::count_if(planePointCloud_BC->begin(),
                              planePointCloud_BC->end(),
                              [](const pcl::PointXYZRGBA &point_BC_m)
                              {
                                  return std::isfinite(point_BC_m.x) &&
                                         std::isfinite(point_BC_m.y) &&
                                         std::isfinite(point_BC_m.z);
                              }));
        }

        /* Extract the configured plane colour */
        const std::vector<std::uint8_t> configuredColour =
            mappedPlane->getColor();

        std::array<std::uint8_t, 3> planeColour_rgb = {255, 255, 255};

        if (configuredColour.size() >= 3)
        {
            planeColour_rgb = {configuredColour[0],
                               configuredColour[1],
                               configuredColour[2]};
        }

        /* Append the current plane to the aggregated point cloud */
        for (const pcl::PointXYZRGBA &sourcePoint_BC :
             planePointCloud_BC->points)
        {
            /* Skip invalid point coordinates */
            if (!std::isfinite(sourcePoint_BC.x) ||
                !std::isfinite(sourcePoint_BC.y) ||
                !std::isfinite(sourcePoint_BC.z))
            {
                continue;
            }

            pcl::PointXYZRGB colouredPoint_BC;

            colouredPoint_BC.x = sourcePoint_BC.x;
            colouredPoint_BC.y = sourcePoint_BC.y;
            colouredPoint_BC.z = sourcePoint_BC.z;

            colouredPoint_BC.r = sourcePoint_BC.r;
            colouredPoint_BC.g = sourcePoint_BC.g;
            colouredPoint_BC.b = sourcePoint_BC.b;

            /* Display detected door planes using a fixed magenta colour */
            if (planeType == ORB_SLAM3::Plane::planeVariant::DOOR)
            {
                colouredPoint_BC.r = 204;
                colouredPoint_BC.g = 0;
                colouredPoint_BC.b = 102;
            }

            buildingComponentPointCloud_BC.points.push_back(colouredPoint_BC);
        }

        /*
         * Use the persistent plane ID. The label and normal markers may share
         * the same ID because they use different namespaces.
         */
        const int planeMarkerId = static_cast<int>(mappedPlane->getId());

        /* ------------------------------------------------------------------ *
         * PLANE LABEL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker planeLabelMarker;

        planeLabelMarker.header.frame_id = frameBC;
        planeLabelMarker.header.stamp    = msgTime_s_in;

        planeLabelMarker.ns = "plane_label";
        planeLabelMarker.id = planeMarkerId;

        planeLabelMarker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        planeLabelMarker.action = visualization_msgs::msg::Marker::ADD;

        std::ostringstream planeLabelText;
        planeLabelText << "Plane#" << mappedPlane->getId();

        if (validWallDimensions)
        {
            planeLabelText << '\n'
                           << std::fixed << std::setprecision(2) << "W "
                           << wallWidth_m << " x H " << wallHeight_m << " m";
        }

        planeLabelText << '\n' << "N " << finiteSupportPointCount;

        planeLabelMarker.text = planeLabelText.str();

        Eigen::Vector3d planeLabelPosition_BC_m =
            planeCentroid_BC_m + planeLabelNormalOffset_m * planeNormal_BC;

        if (groundNormal_BC.norm() >= normalVectorTolerance)
        {
            planeLabelPosition_BC_m +=
                planeLabelVerticalOffset_m * groundNormal_BC;
        }

        planeLabelMarker.pose.position.x = planeLabelPosition_BC_m.x();
        planeLabelMarker.pose.position.y = planeLabelPosition_BC_m.y();
        planeLabelMarker.pose.position.z = planeLabelPosition_BC_m.z();

        planeLabelMarker.pose.orientation.x = 0.0;
        planeLabelMarker.pose.orientation.y = 0.0;
        planeLabelMarker.pose.orientation.z = 0.0;
        planeLabelMarker.pose.orientation.w = 1.0;

        planeLabelMarker.scale.z = 0.14;

        planeLabelMarker.color.a = 1.0;

        planeLabelMarker.color.r =
            static_cast<float>(planeColour_rgb[0]) / 255.0F;

        planeLabelMarker.color.g =
            static_cast<float>(planeColour_rgb[1]) / 255.0F;

        planeLabelMarker.color.b =
            static_cast<float>(planeColour_rgb[2]) / 255.0F;

        planeLabelMarker.lifetime = rclcpp::Duration::from_seconds(0);

        planeVisualizationArray.markers.push_back(std::move(planeLabelMarker));

        /* ------------------------------------------------------------------ *
         * PLANE NORMAL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker planeNormalMarker;

        planeNormalMarker.header.frame_id = frameBC;
        planeNormalMarker.header.stamp    = msgTime_s_in;

        planeNormalMarker.ns = "plane_normal";
        planeNormalMarker.id = planeMarkerId;

        planeNormalMarker.type   = visualization_msgs::msg::Marker::ARROW;
        planeNormalMarker.action = visualization_msgs::msg::Marker::ADD;

        /* Arrow shaft diameter */
        planeNormalMarker.scale.x = 0.025;

        /* Arrowhead diameter */
        planeNormalMarker.scale.y = 0.07;

        /* Arrowhead length */
        planeNormalMarker.scale.z = 0.09;

        planeNormalMarker.color.a = 1.0;

        planeNormalMarker.color.r =
            static_cast<float>(planeColour_rgb[0]) / 255.0F;
        planeNormalMarker.color.g =
            static_cast<float>(planeColour_rgb[1]) / 255.0F;
        planeNormalMarker.color.b =
            static_cast<float>(planeColour_rgb[2]) / 255.0F;

        /* Set the beginning of the plane-normal arrow */
        geometry_msgs::msg::Point normalStartPoint_BC;

        normalStartPoint_BC.x = planeCentroid_BC_m.x();
        normalStartPoint_BC.y = planeCentroid_BC_m.y();
        normalStartPoint_BC.z = planeCentroid_BC_m.z();

        /* Set the end of the plane-normal arrow */
        geometry_msgs::msg::Point normalEndPoint_BC;

        normalEndPoint_BC.x =
            normalStartPoint_BC.x + planeNormal_BC.x() * planeNormalLength_m;

        normalEndPoint_BC.y =
            normalStartPoint_BC.y + planeNormal_BC.y() * planeNormalLength_m;

        normalEndPoint_BC.z =
            normalStartPoint_BC.z + planeNormal_BC.z() * planeNormalLength_m;

        planeNormalMarker.points.reserve(2);

        planeNormalMarker.points.push_back(normalStartPoint_BC);

        planeNormalMarker.points.push_back(normalEndPoint_BC);

        planeNormalMarker.lifetime = rclcpp::Duration::from_seconds(0);

        planeVisualizationArray.markers.push_back(std::move(planeNormalMarker));
    }

    /* Publish the aggregated building-component point cloud */
    if (pubBuildingComponents != nullptr &&
        !buildingComponentPointCloud_BC.empty())
    {
        buildingComponentPointCloud_BC.width = static_cast<std::uint32_t>(
            buildingComponentPointCloud_BC.points.size());

        buildingComponentPointCloud_BC.height = 1;

        buildingComponentPointCloud_BC.is_dense = true;

        sensor_msgs::msg::PointCloud2 buildingComponentPointCloudMessage_BC;

        pcl::toROSMsg(buildingComponentPointCloud_BC,
                      buildingComponentPointCloudMessage_BC);

        buildingComponentPointCloudMessage_BC.header.stamp = msgTime_s_in;

        buildingComponentPointCloudMessage_BC.header.frame_id = frameBC;

        pubBuildingComponents->publish(buildingComponentPointCloudMessage_BC);
    }

    /* Publish the plane labels and normal arrows */
    if (pubPlaneLabel != nullptr && !planeVisualizationArray.markers.empty())
    {
        pubPlaneLabel->publish(planeVisualizationArray);
    }
}

void publishSegmentedCloud(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in)
{
    /* Confirm that the segmented-cloud publisher has been initialised */
    if (pubSegmentedPointcloud == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish segmented point cloud: publisher is not "
                    "initialised.");

        return;
    }

    /* Return when there are no keyframes to inspect */
    if (keyFrames_in.empty())
    {
        return;
    }

    /* Latest keyframe containing valid class-specific point clouds */
    ORB_SLAM3::KeyFrame *selectedKeyFrame = nullptr;

    std::size_t selectedKeyFrameIndex = 0;

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
        classPointClouds_camera;

    /*!
     * Search backwards without converting an unsigned container size into a
     * potentially invalid negative array index.
     */
    for (std::size_t reverseIndex = keyFrames_in.size(); reverseIndex > 0;
         reverseIndex--)
    {
        const std::size_t keyFrameIndex = reverseIndex - 1;

        ORB_SLAM3::KeyFrame *keyFrame = keyFrames_in[keyFrameIndex];

        if (keyFrame == nullptr)
        {
            continue;
        }

        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            candidateClassPointClouds = keyFrame->getClsCloudPtrs();

        /* Determine whether at least one class cloud contains points */
        const bool hasSegmentedPoints = std::any_of(
            candidateClassPointClouds.begin(),
            candidateClassPointClouds.end(),
            [](const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &classPointCloud) {
                return classPointCloud != nullptr && !classPointCloud->empty();
            });

        if (!hasSegmentedPoints)
        {
            continue;
        }

        selectedKeyFrame = keyFrame;

        selectedKeyFrameIndex = keyFrameIndex;

        classPointClouds_camera = std::move(candidateClassPointClouds);

        break;
    }

    /* Return when no processed keyframe contains segmented points */
    if (selectedKeyFrame == nullptr)
    {
        return;
    }

    /*!
     * Clear stale class clouds belonging to keyframes older than the selected
     * keyframe.
     */
    for (std::size_t keyFrameIndex = 0; keyFrameIndex < selectedKeyFrameIndex;
         keyFrameIndex++)
    {
        ORB_SLAM3::KeyFrame *olderKeyFrame = keyFrames_in[keyFrameIndex];

        if (olderKeyFrame != nullptr)
        {
            olderKeyFrame->clearClsClouds();
        }
    }

    /* Calculate the required cloud capacity before aggregation */
    std::size_t totalSegmentedPointCount = 0;

    for (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &classPointCloud :
         classPointClouds_camera)
    {
        if (classPointCloud != nullptr)
        {
            totalSegmentedPointCount += classPointCloud->size();
        }
    }

    if (totalSegmentedPointCount == 0)
    {
        selectedKeyFrame->clearClsClouds();
        return;
    }

    /* Initialise the combined segmented point cloud */
    pcl::PointCloud<pcl::PointXYZRGBA> segmentedPointCloud_camera;

    segmentedPointCloud_camera.points.reserve(totalSegmentedPointCount);

    bool hasSourceHeader = false;

    /* Aggregate the class-specific point clouds */
    for (std::size_t classIndex = 0;
         classIndex < classPointClouds_camera.size();
         classIndex++)
    {
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &classPointCloud_camera =
            classPointClouds_camera[classIndex];

        /* Skip missing or empty class clouds */
        if (classPointCloud_camera == nullptr ||
            classPointCloud_camera->empty())
        {
            continue;
        }

        /* Preserve the metadata from the first valid source cloud */
        if (!hasSourceHeader)
        {
            segmentedPointCloud_camera.header = classPointCloud_camera->header;

            hasSourceHeader = true;
        }

        for (const pcl::PointXYZRGBA &sourcePoint_camera :
             classPointCloud_camera->points)
        {
            /* Skip points containing invalid coordinates */
            if (!std::isfinite(sourcePoint_camera.x) ||
                !std::isfinite(sourcePoint_camera.y) ||
                !std::isfinite(sourcePoint_camera.z))
            {
                continue;
            }

            pcl::PointXYZRGBA segmentedPoint_camera = sourcePoint_camera;

            switch (classIndex)
            {
            case 0:
            {
                /* Display ground points in green */
                segmentedPoint_camera.r = 0;
                segmentedPoint_camera.g = 255;
                segmentedPoint_camera.b = 0;
                break;
            }

            case 1:
            {
                /* Display wall points in red */
                segmentedPoint_camera.r = 255;
                segmentedPoint_camera.g = 0;
                segmentedPoint_camera.b = 0;
                break;
            }

            default:
            {
                /*!
                 * Preserve the original colour for all other semantic
                 * classes.
                 */
                break;
            }
            }

            segmentedPointCloud_camera.points.push_back(segmentedPoint_camera);
        }
    }

    /*!
     * The class clouds have now been consumed and should not be republished on
     * the next invocation.
     */
    selectedKeyFrame->clearClsClouds();

    /* Return when every available point was invalid */
    if (segmentedPointCloud_camera.empty())
    {
        return;
    }

    /* Complete the PCL point-cloud metadata */
    segmentedPointCloud_camera.width =
        static_cast<std::uint32_t>(segmentedPointCloud_camera.points.size());

    segmentedPointCloud_camera.height = 1;

    segmentedPointCloud_camera.is_dense = true;

    /* Convert the combined PCL cloud into a ROS point-cloud message */
    sensor_msgs::msg::PointCloud2 segmentedPointCloudMessage_camera;

    pcl::toROSMsg(segmentedPointCloud_camera,
                  segmentedPointCloudMessage_camera);

    /*!
     * The class-specific clouds are currently represented in the camera
     * frame.
     */
    segmentedPointCloudMessage_camera.header.frame_id = frameCamera;

    /* Publish the combined segmented point cloud */
    pubSegmentedPointcloud->publish(segmentedPointCloudMessage_camera);
}

void publishStaticTFTransform(const std::string  &parentFrameId_in,
                              const std::string  &childFrameId_in,
                              const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the static-transform broadcaster has been initialised */
    if (staticTfBroadcaster == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish static transform: broadcaster is not initialised.");

        return;
    }

    /* Confirm that valid frame identifiers were supplied */
    if (parentFrameId_in.empty() || childFrameId_in.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish static transform: frame identifier is empty.");

        return;
    }

    /* Prevent a frame from being defined as its own child */
    if (parentFrameId_in == childFrameId_in)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish static transform: parent and child frames are "
            "identical.");

        return;
    }

    /* Calculate the child-frame orientation relative to the parent frame */
    tf2::Quaternion orientation_parent_child;

    orientation_parent_child.setRPY(roll, pitch, yaw);
    orientation_parent_child.normalize();

    /* Initialise the static transformation message */
    geometry_msgs::msg::TransformStamped staticTransformMessage;

    staticTransformMessage.header.stamp    = msgTime_s_in;
    staticTransformMessage.header.frame_id = parentFrameId_in;
    staticTransformMessage.child_frame_id  = childFrameId_in;

    /* The child and parent frames have no relative translation */
    staticTransformMessage.transform.translation.x = 0.0;
    staticTransformMessage.transform.translation.y = 0.0;
    staticTransformMessage.transform.translation.z = 0.0;

    /* Set the child-frame orientation relative to the parent frame */
    staticTransformMessage.transform.rotation.x = orientation_parent_child.x();
    staticTransformMessage.transform.rotation.y = orientation_parent_child.y();
    staticTransformMessage.transform.rotation.z = orientation_parent_child.z();
    staticTransformMessage.transform.rotation.w = orientation_parent_child.w();

    /* Publish the completed static transformation */
    staticTfBroadcaster->sendTransform(staticTransformMessage);
}

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
                      mappedFloors_in,
                      msgTime_s_in,
                      structuralElementMarkerArray);

    /* Append the floor markers */
    appendFloorMarkers(mappedFloors_in,
                       msgTime_s_in,
                       structuralElementMarkerArray);

    /* Append the passage markers */
    appendPassageMarkers(mappedPassages_in,
                         mappedRooms_in,
                         msgTime_s_in,
                         structuralElementMarkerArray);

    if (!structuralElementMarkerArray.markers.empty())
    {
        pubStructuralElements->publish(structuralElementMarkerArray);
    }
}

void publishTFTransform(const Sophus::SE3f &transform_ParentToChild_in,
                        const std::string  &parentFrameId_in,
                        const std::string  &childFrameId_in,
                        const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the dynamic-transform broadcaster is available */
    if (tfBroadcaster == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish TF transform: broadcaster is not initialised.");

        return;
    }

    /* Confirm that valid frame identifiers were supplied */
    if (parentFrameId_in.empty() || childFrameId_in.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish TF transform: frame identifier is empty.");

        return;
    }

    /* Prevent a frame from being published as its own child */
    if (parentFrameId_in == childFrameId_in)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish TF transform: parent and child frames are "
                     "identical.");

        return;
    }

    /* Confirm that the supplied transformation contains valid values */
    if (!transform_ParentToChild_in.translation().allFinite() ||
        !transform_ParentToChild_in.rotationMatrix().allFinite())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish TF transform: transformation contains "
                     "non-finite values.");

        return;
    }

    /* Extract the translation and orientation once */
    const Eigen::Vector3f translation_parent_child_m =
        transform_ParentToChild_in.translation();

    Eigen::Quaternionf orientation_parent_child =
        transform_ParentToChild_in.unit_quaternion();

    orientation_parent_child.normalize();

    /* Initialise the dynamic transformation message */
    geometry_msgs::msg::TransformStamped transformMessage;

    transformMessage.header.stamp    = msgTime_s_in;
    transformMessage.header.frame_id = parentFrameId_in;
    transformMessage.child_frame_id  = childFrameId_in;

    /* Set the child-frame translation relative to the parent frame */
    transformMessage.transform.translation.x = translation_parent_child_m.x();
    transformMessage.transform.translation.y = translation_parent_child_m.y();
    transformMessage.transform.translation.z = translation_parent_child_m.z();

    /* Set the child-frame orientation relative to the parent frame */
    transformMessage.transform.rotation.x = orientation_parent_child.x();
    transformMessage.transform.rotation.y = orientation_parent_child.y();
    transformMessage.transform.rotation.z = orientation_parent_child.z();
    transformMessage.transform.rotation.w = orientation_parent_child.w();

    /* Broadcast the completed dynamic transformation */
    tfBroadcaster->sendTransform(transformMessage);
}

bool preparePointCloudForTracking(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointCloudMessage_in,
    bool                                           directGazeboFluCloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr        &pointCloud_camera_out,
    sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointCloudCameraMessage_out,
    std::string                                   &failureReason_out)
{
    if (pointCloudMessage_in == nullptr)
    {
        failureReason_out = "missing_cloud";
        return false;
    }

    auto pointCloud_source =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    try
    {
        pcl::fromROSMsg(*pointCloudMessage_in, *pointCloud_source);
    }
    catch (const std::exception &exception)
    {
        failureReason_out = std::string("cloud_conversion:") + exception.what();
        return false;
    }

    if (directGazeboFluCloud_in)
    {
        pointCloud_camera_out =
            std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(
                *pointCloud_source);
        for (pcl::PointXYZRGB &point : pointCloud_camera_out->points)
        {
            const float sourceX = point.x;
            const float sourceY = point.y;
            const float sourceZ = point.z;
            point.x             = -sourceY;
            point.y             = -sourceZ;
            point.z             = sourceX;
        }

        auto convertedMessage =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pointCloud_camera_out, *convertedMessage);
        convertedMessage->header          = pointCloudMessage_in->header;
        convertedMessage->header.frame_id = frameCamera;
        pointCloudCameraMessage_out       = convertedMessage;
        return true;
    }

    if (pointCloudMessage_in->header.frame_id == frameCamera)
    {
        pointCloud_camera_out       = pointCloud_source;
        pointCloudCameraMessage_out = pointCloudMessage_in;
        return true;
    }

    if (pointCloudMessage_in->header.frame_id.empty() || frameCamera.empty() ||
        tfBuffer_ == nullptr)
    {
        failureReason_out = "cloud_frame_unresolved";
        return false;
    }

    try
    {
        const geometry_msgs::msg::TransformStamped transformMessage =
            tfBuffer_->lookupTransform(
                frameCamera,
                pointCloudMessage_in->header.frame_id,
                rclcpp::Time(pointCloudMessage_in->header.stamp),
                rclcpp::Duration::from_seconds(0.05));
        tf2::Transform transformTargetFromSource;
        tf2::fromMsg(transformMessage.transform, transformTargetFromSource);

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                transformMatrix(row, column) = static_cast<float>(
                    transformTargetFromSource.getBasis()[row][column]);
            }
            transformMatrix(row, 3) =
                static_cast<float>(transformTargetFromSource.getOrigin()[row]);
        }

        pointCloud_camera_out =
            std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::transformPointCloud(*pointCloud_source,
                                 *pointCloud_camera_out,
                                 transformMatrix);
        auto transformedMessage =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pointCloud_camera_out, *transformedMessage);
        transformedMessage->header          = pointCloudMessage_in->header;
        transformedMessage->header.frame_id = frameCamera;
        pointCloudCameraMessage_out         = transformedMessage;
        return true;
    }
    catch (const tf2::TransformException &exception)
    {
        failureReason_out = std::string("cloud_tf:") + exception.what();
        return false;
    }
}

void publishTopics(const rclcpp::Time    &msgTime_s_in,
                   const Eigen::Vector3f &angularVelocity_body_radps_in,
                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                       &pointCloud_cameraMessage_in)
{
    /* Confirm that the SLAM system has been initialised */
    if (p_slamSystem == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish topics: SLAM system is not initialised.");

        return;
    }

    /* Obtain the current camera pose relative to the world frame */
    const Sophus::SE3f T_world_camera_SE3f = p_slamSystem->GetCamTwc();

    /* Prevent invalid camera transformations from entering ROS messages */
    if (!T_world_camera_SE3f.translation().allFinite() ||
        !T_world_camera_SE3f.rotationMatrix().allFinite())
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish topics: camera pose contains non-finite values.");

        return;
    }

    /* ---------------------------------------------------------------------- *
     * CAMERA AND FRAME TOPICS
     * ---------------------------------------------------------------------- */

    publishCameraPose(T_world_camera_SE3f, msgTime_s_in);

    publishTFTransform(T_world_camera_SE3f,
                       frameWorld,
                       frameCamera,
                       msgTime_s_in);

    /* ---------------------------------------------------------------------- *
     * DERIVED-MAP LIFECYCLE
     * ---------------------------------------------------------------------- */

    if (p_mapRevisionPublisher != nullptr)
    {
        ORB_SLAM3::Map *p_activeMap = p_slamSystem->GetCurrentMap();

        if (p_activeMap != nullptr)
        {
            const std::uint64_t mapId =
                static_cast<std::uint64_t>(p_activeMap->GetId());

            const int mapChangeIndex = p_activeMap->GetLastBigChangeIdx();

            const std::uint64_t nonNegativeMapChangeIndex =
                mapChangeIndex > 0 ? static_cast<std::uint64_t>(mapChangeIndex)
                                   : 0U;

            /*!
             * The upper and lower halves identify the map and its correction
             * generation respectively. This is a lifecycle token, not an
             * arithmetic sequence.
             */
            const std::uint64_t mapRevision =
                (mapId << 32U) | (nonNegativeMapChangeIndex & 0xffffffffULL);

            static std::uint64_t lastPublishedMapRevision =
                std::numeric_limits<std::uint64_t>::max();

            if (mapRevision != lastPublishedMapRevision)
            {
                /* Remove visualization state belonging to the old map. */
                clearMapScopedVisualization(msgTime_s_in);

                std_msgs::msg::UInt64 mapRevisionMessage;
                mapRevisionMessage.data = mapRevision;
                p_mapRevisionPublisher->publish(mapRevisionMessage);
                lastPublishedMapRevision = mapRevision;
            }
        }
    }

    publishFramePointCloud(T_world_camera_SE3f,
                           pointCloud_cameraMessage_in,
                           msgTime_s_in);

    /*!
     * A static transform only needs to be published once. The static TF
     * broadcaster stores the transform for future subscribers.
     */
    static bool hasPublishedWorldMapStaticTransform = false;

    if (pubStaticTransform && !hasPublishedWorldMapStaticTransform &&
        staticTfBroadcaster != nullptr)
    {
        publishStaticTFTransform(frameWorld, frameMap, msgTime_s_in);

        hasPublishedWorldMapStaticTransform = true;
    }

    /* ---------------------------------------------------------------------- *
     * OBTAIN A SNAPSHOT OF THE CURRENT MAP COLLECTIONS
     * ---------------------------------------------------------------------- */

    const std::vector<ORB_SLAM3::KeyFrame *> mappedKeyFrames =
        p_slamSystem->GetAllKeyFrames();

    const std::vector<ORB_SLAM3::Marker *> mappedFiducialMarkers =
        p_slamSystem->GetAllMarkers();

    const std::vector<ORB_SLAM3::Room *> mappedRooms =
        p_slamSystem->GetAllRooms();

    const std::vector<ORB_SLAM3::Floor *> mappedFloors =
        p_slamSystem->GetAllFloors();

    const std::vector<ORB_SLAM3::Passage *> mappedPassages =
        p_slamSystem->GetAllPassages();

    const std::vector<ORB_SLAM3::Plane *> mappedPlanes =
        p_slamSystem->GetAllPlanes();

    /* ---------------------------------------------------------------------- *
     * KEYFRAMES, TRACKING, AND STRUCTURAL ELEMENTS
     * ---------------------------------------------------------------------- */

    publishKeyFrameImages(mappedKeyFrames, msgTime_s_in);
    publishKeyFrameMarkers(mappedKeyFrames, msgTime_s_in);
    publishFiducialMarkers(mappedFiducialMarkers, msgTime_s_in);
    publishTrackingImage(p_slamSystem->GetCurrentFrame(), msgTime_s_in);
    publishStructuralElements(mappedRooms,
                              mappedFloors,
                              mappedPassages,
                              msgTime_s_in);

    /*!
     * Publish mapped walls independently of the point-cloud visualisation
     * setting because they are consumed by GNN-based room detection.
     */
    publishAllMappedWalls(mappedPlanes, msgTime_s_in);

    /* ------------------------------------------------------------------ *
     * POINT-CLOUD TOPICS
     * ------------------------------------------------------------------ */

    if (pubPointClouds)
    {
        publishSegmentedCloud(mappedKeyFrames);
        publishPlanes(mappedPlanes, msgTime_s_in);
        publishAllPoints(p_slamSystem->GetAllMapPoints(), msgTime_s_in);
        publishTrackedPoints(p_slamSystem->GetTrackedMapPoints(), msgTime_s_in);
        publishFreeSpaceClusters(p_slamSystem->getSkeletonCluster(),
                                 msgTime_s_in);
    }
    else
    {
        /*!
         * Semantic class clouds are temporary and should be cleared when their
         * publication has been disabled.
         */
        clearKFClsClouds(mappedKeyFrames);
    }

    /* ---------------------------------------------------------------------- *
     * INERTIAL TOPICS
     * ---------------------------------------------------------------------- */

    const bool usesInertialSensor =
        sensorType == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD;

    if (!usesInertialSensor)
    {
        return;
    }

    /* T_world_body_SE3f describes the body pose relative to the world frame */
    const Sophus::SE3f T_world_body_SE3f = p_slamSystem->GetImuTwb();

    /*!
     * ORB-SLAM3 supplies the body linear velocity expressed in the world
     * frame.
     */
    const Eigen::Vector3f linearVelocity_world_mps = p_slamSystem->GetImuVwb();

    /* Validate all inertial quantities before publication */
    if (!T_world_body_SE3f.translation().allFinite() ||
        !T_world_body_SE3f.rotationMatrix().allFinite() ||
        !linearVelocity_world_mps.allFinite() ||
        !angularVelocity_body_radps_in.allFinite())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish inertial topics: inertial state contains "
                    "non-finite values.");

        return;
    }

    publishTFTransform(T_world_body_SE3f, frameWorld, frameImu, msgTime_s_in);

    /*!
     * The angular velocity is already expressed in the body frame and should
     * not be transformed into the world frame.
     *
     * publishBodyOdometry() should rotate the world-frame linear velocity into
     * the body frame before filling the Odometry twist field.
     */
    publishBodyOdometry(T_world_body_SE3f,
                        linearVelocity_world_mps,
                        angularVelocity_body_radps_in,
                        msgTime_s_in);
}

void publishTrackedPoints(
    const std::vector<ORB_SLAM3::MapPoint *> &trackedMapPoints_in,
    const rclcpp::Time                       &msgTime_s_in)
{
    /* Confirm that the tracked-map-point publisher is available */
    if (pubTrackedMappoints == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish tracked map points: publisher is not initialised.");

        return;
    }

    /* Convert the tracked map points into a ROS point-cloud message */
    sensor_msgs::msg::PointCloud2 trackedMapPointCloudMessage =
        mapPointToPointcloud(trackedMapPoints_in, msgTime_s_in);

    /*!
     * Explicitly set the output metadata in case the conversion function does
     * not set it, or to ensure it remains consistent with this publisher.
     */
    trackedMapPointCloudMessage.header.stamp    = msgTime_s_in;
    trackedMapPointCloudMessage.header.frame_id = frameWorld;

    /*!
     * Publish empty clouds as well. This clears previously displayed tracked
     * points when ORB-SLAM3 is no longer tracking any map points.
     */
    pubTrackedMappoints->publish(trackedMapPointCloudMessage);
}

void publishTrackingImage(const cv::Mat      &trackingImage_bgr8_in,
                          const rclcpp::Time &msgTime_s_in)
{
    /* Confirm that the image publisher has been initialised */
    if (pubTrackingImage == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish tracking image: publisher is not initialised.");

        return;
    }

    /* Return when ORB-SLAM3 has not supplied a valid image */
    if (trackingImage_bgr8_in.empty())
    {
        return;
    }

    /*!
     * The current publication pipeline expects a three-channel, unsigned
     * 8-bit BGR image.
     */
    if (trackingImage_bgr8_in.type() != CV_8UC3)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish tracking image: expected CV_8UC3 but received "
            "OpenCV type %d.",
            trackingImage_bgr8_in.type());

        return;
    }

    /* Initialise the ROS image header */
    std_msgs::msg::Header imageHeader;

    imageHeader.stamp = msgTime_s_in;

    /*!
     * The message contains a camera image, so its frame is the camera rather
     * than the world frame.
     */
    imageHeader.frame_id = frameCamera;

    /* Convert the OpenCV image into a ROS image message */
    const sensor_msgs::msg::Image::SharedPtr trackingImageMessage =
        cv_bridge::CvImage(imageHeader, "bgr8", trackingImage_bgr8_in)
            .toImageMsg();

    if (trackingImageMessage == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Failed to convert the tracking image into a ROS image message.");

        return;
    }

    /* Publish through the image_transport publisher */
    pubTrackingImage->publish(trackingImageMessage);
}

void saveMapPointsAsPCDService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out)
{
    /* Confirm that a valid service response was supplied */
    if (response_out == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: service response is null.");

        return;
    }

    /* Set the default response in case validation or saving fails */
    response_out->success = false;

    /* Confirm that a valid service request was supplied */
    if (request_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: service request is null.");

        return;
    }

    /* Confirm that the SLAM system has been initialised */
    if (p_slamSystem == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: SLAM system is not initialised.");

        return;
    }

    /* Confirm that an output name was provided */
    if (request_in->name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: output file name is empty.");

        return;
    }

    /* Request that ORB-SLAM3 save the current map points */
    try
    {
        response_out->success =
            p_slamSystem->SaveMapPointsAsPCD(request_in->name);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Exception while saving map points as '%s.pcd': %s",
                     request_in->name.c_str(),
                     exception.what());

        return;
    }

    /* Report the result of the save operation */
    if (response_out->success)
    {
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                    "Map points were saved as '%s.pcd'.",
                    request_in->name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Map points could not be saved as '%s.pcd'.",
                     request_in->name.c_str());
    }
}

void saveMapService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out)
{
    /* Confirm that a valid service response was supplied */
    if (response_out == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: service response is null.");

        return;
    }

    /* Set the default response in case validation or saving fails */
    response_out->success = false;

    /* Confirm that a valid service request was supplied */
    if (request_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: service request is null.");

        return;
    }

    /* Confirm that the SLAM system has been initialised */
    if (p_slamSystem == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: SLAM system is not initialised.");

        return;
    }

    /* Confirm that an output name was provided */
    if (request_in->name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: output file name is empty.");

        return;
    }

    /* Request that ORB-SLAM3 save the current map */
    try
    {
        response_out->success = p_slamSystem->SaveMap(request_in->name);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Exception while saving map as '%s.osa': %s",
                     request_in->name.c_str(),
                     exception.what());

        return;
    }

    /* Report the result of the save operation */
    if (response_out->success)
    {
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                    "Map was saved as '%s.osa'.",
                    request_in->name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Map could not be saved as '%s.osa'.",
                     request_in->name.c_str());
    }
}

void saveTrajectoryService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out)
{
    /* Confirm that a valid service response was supplied */
    if (response_out == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save trajectories: service response is null.");

        return;
    }

    /* Set failure as the default service result */
    response_out->success = false;

    /* Confirm that a valid service request was supplied */
    if (request_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save trajectories: service request is null.");

        return;
    }

    /* Confirm that the SLAM system has been initialised */
    if (p_slamSystem == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot save trajectories: SLAM system is not initialised.");

        return;
    }

    /* Confirm that a valid output base name was supplied */
    if (request_in->name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save trajectories: output base name is empty.");

        return;
    }

    /* Construct the output trajectory file names */
    const std::string cameraTrajectoryFileName =
        request_in->name + "_cam_traj.txt";

    const std::string keyFrameTrajectoryFileName =
        request_in->name + "_kf_traj.txt";

    try
    {
        /* Save the complete estimated camera trajectory */
        p_slamSystem->SaveTrajectoryEuRoC(cameraTrajectoryFileName);

        /* Save the estimated keyframe trajectory */
        p_slamSystem->SaveKeyFrameTrajectoryEuRoC(keyFrameTrajectoryFileName);

        response_out->success = true;
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Exception while saving estimated trajectories: %s",
                     exception.what());

        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Unknown exception while saving estimated trajectories.");

        return;
    }

    /* Report the completed save operation */
    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                "Camera trajectory was saved as '%s'.",
                cameraTrajectoryFileName.c_str());

    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                "Keyframe trajectory was saved as '%s'.",
                keyFrameTrajectoryFileName.c_str());
}

tf2::Transform SE3fToTFTransform(const Sophus::SE3f &transformation_SE3f_in)
{
    /* Extract the translation component */
    const Eigen::Vector3f translation_Eigen =
        transformation_SE3f_in.translation();

    /* Extract and normalise the orientation component */
    Eigen::Quaternionf orientation_Eigen =
        transformation_SE3f_in.unit_quaternion();

    orientation_Eigen.normalize();

    /* Convert the translation into its tf2 representation */
    const tf2::Vector3 translation_tf2(
        static_cast<tf2Scalar>(translation_Eigen.x()),
        static_cast<tf2Scalar>(translation_Eigen.y()),
        static_cast<tf2Scalar>(translation_Eigen.z()));

    /* Convert the orientation into its tf2 representation */
    tf2::Quaternion orientation_tf2(
        static_cast<tf2Scalar>(orientation_Eigen.x()),
        static_cast<tf2Scalar>(orientation_Eigen.y()),
        static_cast<tf2Scalar>(orientation_Eigen.z()),
        static_cast<tf2Scalar>(orientation_Eigen.w()));

    orientation_tf2.normalize();

    /* Construct and return the complete tf2 transformation */
    return tf2::Transform(orientation_tf2, translation_tf2);
}

void setupPublishers(
    const std::shared_ptr<rclcpp::Node>                    &node_in,
    const std::shared_ptr<image_transport::ImageTransport> &imageTransport_in,
    const std::string                                      &topicNamespace_in)
{
    /* Confirm that a valid ROS node was supplied */
    if (node_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot initialise publishers: ROS node is null.");

        return;
    }

    /* Confirm that a valid image-transport interface was supplied */
    if (imageTransport_in == nullptr)
    {
        RCLCPP_ERROR(node_in->get_logger(),
                     "Cannot initialise publishers: image transport is null.");

        return;
    }

    /*!
     * Remove trailing separators from the topic namespace so generated names
     * do not contain repeated '/' characters.
     */
    std::string normalisedTopicNamespace = topicNamespace_in;

    while (!normalisedTopicNamespace.empty() &&
           normalisedTopicNamespace.back() == '/')
    {
        normalisedTopicNamespace.pop_back();
    }

    /*!
     * Construct a complete topic name using the configured namespace.
     *
     * An empty namespace produces a relative topic name such as
     * "camera_pose".
     */
    const auto makeTopicName =
        [&normalisedTopicNamespace](const std::string &topicSuffix)
    {
        if (normalisedTopicNamespace.empty())
        {
            return topicSuffix;
        }

        return normalisedTopicNamespace + "/" + topicSuffix;
    };

    /* Define the publisher queue configurations */
    const rclcpp::QoS standardPublisherQoS(rclcpp::KeepLast(1));

    const rclcpp::QoS pathPublisherQoS(rclcpp::KeepLast(2));

    const rclcpp::QoS keyFrameImagePublisherQoS =
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local();

    /* ---------------------------------------------------------------------- *
     * BASIC SLAM PUBLISHERS
     * ---------------------------------------------------------------------- */

    pubKeyFrameList = node_in->create_publisher<nav_msgs::msg::Path>(
        makeTopicName("keyframe_list"),
        pathPublisherQoS);

    pubAllMappoints = node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
        makeTopicName("all_points"),
        standardPublisherQoS);

    pubCameraPose = node_in->create_publisher<geometry_msgs::msg::PoseStamped>(
        makeTopicName("camera_pose"),
        standardPublisherQoS);

    pubKFImage = node_in->create_publisher<segmenter_ros::msg::VSGraphDataMsg>(
        makeTopicName("keyframe_image"),
        keyFrameImagePublisherQoS);

    pubTrackedMappoints =
        node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
            makeTopicName("tracked_points"),
            standardPublisherQoS);

    p_voxbloxInputPointCloudPublisher =
        node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
            makeTopicName("points_map"),
            standardPublisherQoS);

    p_mapRevisionPublisher = node_in->create_publisher<std_msgs::msg::UInt64>(
        makeTopicName("map_revision"),
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    pubKeyFrameMarker =
        node_in->create_publisher<visualization_msgs::msg::MarkerArray>(
            makeTopicName("kf_markers"),
            standardPublisherQoS);

    pubFreespaceCluster =
        node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
            makeTopicName("freespace_clusters"),
            standardPublisherQoS);

    pubCameraPoseVis =
        node_in->create_publisher<visualization_msgs::msg::MarkerArray>(
            makeTopicName("camera_pose_vis"),
            standardPublisherQoS);

    pubTrackingImage = std::make_shared<image_transport::Publisher>(
        imageTransport_in->advertise(makeTopicName("tracking_image"), 1));

    /* ---------------------------------------------------------------------- *
     * ENTITY PUBLISHERS
     * ---------------------------------------------------------------------- */

    pubDoor = node_in->create_publisher<visualization_msgs::msg::MarkerArray>(
        makeTopicName("doors"),
        standardPublisherQoS);

    pubFiducialMarker =
        node_in->create_publisher<visualization_msgs::msg::MarkerArray>(
            makeTopicName("fiducial_markers"),
            standardPublisherQoS);

    /* ---------------------------------------------------------------------- *
     * BUILDING-COMPONENT PUBLISHERS
     * ---------------------------------------------------------------------- */

    pubPlaneLabel =
        node_in->create_publisher<visualization_msgs::msg::MarkerArray>(
            makeTopicName("plane_labels"),
            standardPublisherQoS);

    pubBuildingComponents =
        node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
            makeTopicName("building_components"),
            standardPublisherQoS);

    pubSegmentedPointcloud =
        node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
            makeTopicName("segmented_point_clouds"),
            standardPublisherQoS);

    /* ---------------------------------------------------------------------- *
     * MAPPED-WALL PUBLISHER
     * ---------------------------------------------------------------------- */

    pubAllWalls_new =
        node_in->create_publisher<vs_graphs::msg::VSGraphsAllWallsData>(
            makeTopicName("all_mapped_walls"),
            standardPublisherQoS);

    /* ---------------------------------------------------------------------- *
     * STRUCTURAL-ELEMENT PUBLISHER
     * ---------------------------------------------------------------------- */

    pubStructuralElements =
        node_in->create_publisher<visualization_msgs::msg::MarkerArray>(
            makeTopicName("structural_elements"),
            standardPublisherQoS);

    /* ---------------------------------------------------------------------- *
     * INERTIAL PUBLISHERS
     * ---------------------------------------------------------------------- */

    const bool usesInertialSensor =
        sensorType == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD;

    if (usesInertialSensor)
    {
        pubOdometry = node_in->create_publisher<nav_msgs::msg::Odometry>(
            makeTopicName("body_odom"),
            standardPublisherQoS);
    }
    else
    {
        /*!
         * Ensure an old inertial publisher is not retained if this function is
         * called again using a non-inertial sensor configuration.
         */
        pubOdometry.reset();
    }

    /* ---------------------------------------------------------------------- *
     * TF INTERFACES
     * ---------------------------------------------------------------------- */

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_in->get_clock());

    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    RCLCPP_INFO(
        node_in->get_logger(),
        "Visual S-Graphs publishers were initialised under namespace '%s'.",
        normalisedTopicNamespace.empty() ? "<relative>"
                                         : normalisedTopicNamespace.c_str());
}

static void getMissionHealthService(
    const std::shared_ptr<vs_graphs::srv::GetMissionHealth::Request> request_in,
    std::shared_ptr<vs_graphs::srv::GetMissionHealth::Response> response_out)
{
    if (p_slamSystem == nullptr)
    {
        response_out->available = false;
        return;
    }

    const ORB_SLAM3::System::MissionHealthSnapshot snapshot =
        p_slamSystem->GetMissionHealthSnapshot(true);
    response_out->available            = true;
    response_out->mode                 = sensorModeName();
    response_out->frame_timestamp      = snapshot.frameTimestamp;
    response_out->tracking_state       = snapshot.trackingState;
    response_out->tracking_inliers     = snapshot.trackingInliers;
    response_out->inertial             = snapshot.inertial;
    response_out->inertial_initialized = snapshot.inertialInitialized;
    response_out->pose_valid           = snapshot.poseValid;
    response_out->map_id               = snapshot.mapId;
    response_out->map_count            = snapshot.mapCount;
    response_out->keyframe_count       = snapshot.keyFrameCount;
    response_out->reset_count          = snapshot.resetCount;

    if (snapshot.poseValid)
    {
        const Eigen::Vector3f translation =
            snapshot.cameraPose_World.translation();
        const Eigen::Quaternionf orientation =
            snapshot.cameraPose_World.unit_quaternion();
        response_out->pose.position.x    = translation.x();
        response_out->pose.position.y    = translation.y();
        response_out->pose.position.z    = translation.z();
        response_out->pose.orientation.x = orientation.x();
        response_out->pose.orientation.y = orientation.y();
        response_out->pose.orientation.z = orientation.z();
        response_out->pose.orientation.w = orientation.w();
    }

    response_out->loop_event_sequence = snapshot.loopSequence;
    response_out->accepted_loop_count = snapshot.acceptedLoopCount;
    response_out->rejected_loop_count = snapshot.rejectedLoopCount;
    response_out->has_loop_event      = snapshot.hasLoopEvent;
    response_out->last_loop_accepted  = snapshot.lastLoopAccepted;
    response_out->last_loop_map_id    = snapshot.lastLoopMapId;
    response_out->last_loop_current_keyframe_id =
        snapshot.lastLoopCurrentKeyFrameId;
    response_out->last_loop_matched_keyframe_id =
        snapshot.lastLoopMatchedKeyFrameId;
    response_out->last_loop_current_timestamp =
        snapshot.lastLoopCurrentTimestamp;
    response_out->last_loop_matched_timestamp =
        snapshot.lastLoopMatchedTimestamp;
    response_out->last_loop_reason = snapshot.lastLoopReason;

    response_out->confirmed_room_count  = snapshot.confirmedRoomCount;
    response_out->unresolved_room_count = snapshot.unresolvedRoomCount;
    response_out->current_room_id       = snapshot.currentRoomId;
    response_out->last_known_room_id    = snapshot.lastKnownRoomId;
    response_out->floor_count =
        static_cast<std::uint32_t>(snapshot.floors.size());
    response_out->floor_room_link_count = snapshot.floorRoomLinkCount;
    response_out->passage_count =
        static_cast<std::uint32_t>(snapshot.passages.size());

    json topology = {{"schema", 1},
                     {"map_id", snapshot.mapId},
                     {"active_maps", snapshot.mapCount},
                     {"reset_count", snapshot.resetCount},
                     {"confirmed_rooms", snapshot.confirmedRoomCount},
                     {"unresolved_rooms", snapshot.unresolvedRoomCount},
                     {"floor_room_links", snapshot.floorRoomLinkCount},
                     {"rooms", json::array()},
                     {"floors", json::array()},
                     {"passages", json::array()}};
    for (const ORB_SLAM3::System::RoomHealth &room : snapshot.rooms)
    {
        topology["rooms"].push_back(
            {{"id", room.id}, {"passage_ids", room.passageIds}});
    }
    for (const ORB_SLAM3::System::FloorHealth &floor : snapshot.floors)
    {
        topology["floors"].push_back(
            {{"id", floor.id}, {"room_ids", floor.roomIds}});
    }
    for (const ORB_SLAM3::System::PassageHealth &passage : snapshot.passages)
    {
        const bool traversed = passage.knownToFarCount > 0U ||
                               passage.farToKnownCount > 0U ||
                               passage.unknownCount > 0U;
        const bool bidirectional =
            passage.knownToFarCount > 0U && passage.farToKnownCount > 0U;
        if (passage.passable)
        {
            ++response_out->passable_passage_count;
        }
        if (traversed)
        {
            ++response_out->traversed_passage_count;
        }
        if (bidirectional)
        {
            ++response_out->bidirectional_passage_count;
        }
        response_out->traversal_known_to_far_count += passage.knownToFarCount;
        response_out->traversal_far_to_known_count += passage.farToKnownCount;
        response_out->traversal_unknown_count += passage.unknownCount;

        topology["passages"].push_back(
            {{"id", passage.id},
             {"passable", passage.passable},
             {"known_side_room_id", passage.knownSideRoomId},
             {"far_side_room_id", passage.farSideRoomId},
             {"known_to_far", passage.knownToFarCount},
             {"far_to_known", passage.farToKnownCount},
             {"unknown", passage.unknownCount},
             {"bidirectional", bidirectional}});
    }

    if (request_in->include_topology)
    {
        response_out->topology_json = topology.dump();
    }
}

void setupServices(const std::shared_ptr<rclcpp::Node> &node_in,
                   const std::string                   &serviceNamespace_in)
{
    /* Confirm that a valid ROS node was supplied */
    if (node_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot initialise services: ROS node is null.");

        return;
    }

    /*!
     * Remove trailing separators so generated service names do not contain
     * repeated '/' characters.
     */
    std::string normalisedServiceNamespace = serviceNamespace_in;

    while (!normalisedServiceNamespace.empty() &&
           normalisedServiceNamespace.back() == '/')
    {
        normalisedServiceNamespace.pop_back();
    }

    /*!
     * Construct a complete service name using the configured namespace.
     *
     * An empty namespace produces relative service names such as "save_map".
     */
    const auto makeServiceName =
        [&normalisedServiceNamespace](const std::string &serviceSuffix)
    {
        if (normalisedServiceNamespace.empty())
        {
            return serviceSuffix;
        }

        return normalisedServiceNamespace + "/" + serviceSuffix;
    };

    /*!
     * Release any existing service instances before replacing them. This is
     * relevant if setupServices() is called more than once.
     */
    srvSaveMap.reset();
    srvSaveMapPoints.reset();
    srvSaveTrajectory.reset();
    srvGetMissionHealth.reset();

    /* Create the complete-map save service */
    srvSaveMap = node_in->create_service<vs_graphs::srv::SaveMap>(
        makeServiceName("save_map"),
        &saveMapService);

    /* Create the map-point PCD save service */
    srvSaveMapPoints = node_in->create_service<vs_graphs::srv::SaveMap>(
        makeServiceName("save_map_points"),
        &saveMapPointsAsPCDService);

    /* Create the trajectory save service */
    srvSaveTrajectory = node_in->create_service<vs_graphs::srv::SaveMap>(
        makeServiceName("save_traj"),
        &saveTrajectoryService);

    srvGetMissionHealth =
        node_in->create_service<vs_graphs::srv::GetMissionHealth>(
            makeServiceName("get_mission_health"),
            &getMissionHealthService);

    /* Confirm that all service objects were created */
    if (srvSaveMap == nullptr || srvSaveMapPoints == nullptr ||
        srvSaveTrajectory == nullptr || srvGetMissionHealth == nullptr)
    {
        RCLCPP_ERROR(
            node_in->get_logger(),
            "One or more Visual S-Graphs services could not be initialised.");

        return;
    }

    RCLCPP_INFO(
        node_in->get_logger(),
        "Visual S-Graphs services were initialised under namespace '%s'.",
        normalisedServiceNamespace.empty()
            ? "<relative>"
            : normalisedServiceNamespace.c_str());
}

void setVoxbloxSkeletonCluster(
    const visualization_msgs::msg::MarkerArray &skeletonMarkerArray_in)
{
    /* Confirm that the SLAM system has been initialised */
    if (p_slamSystem == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot store Voxblox skeleton: SLAM system is not initialised.");

        return;
    }

    /* Obtain the configured room-segmentation parameters */
    const auto *systemParameters = ORB_SLAM3::SystemParams::GetParams();

    if (systemParameters == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot store Voxblox skeleton: system parameters are "
                    "unavailable.");

        return;
    }

    /*!
     * Prevent a negative configured value from being converted into a very
     * large unsigned integer.
     */
    const int configuredMinimumClusterVertices =
        systemParameters->room_seg.min_cluster_vertices;

    const std::size_t minimumClusterVertexCount =
        configuredMinimumClusterVertices > 0
            ? static_cast<std::size_t>(configuredMinimumClusterVertices)
            : 1U;

    /*!
     * Build new buffers locally. The global buffers are replaced only after
     * the complete marker array has been processed.
     */
    std::vector<std::vector<Eigen::Vector3d>> transformedSkeletonClusters_world;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        transformedSkeletonEdges_world;

    transformedSkeletonClusters_world.reserve(
        skeletonMarkerArray_in.markers.size());

    /* Process every marker contained in the sparse graph message */
    for (const visualization_msgs::msg::Marker &skeletonMarker :
         skeletonMarkerArray_in.markers)
    {
        /*!
         * Voxblox publishes each connected free-space component using a marker
         * namespace beginning with "connected_vertices_".
         *
         * Generic "vertices" and "closed_spaces" markers are deliberately
         * ignored because they duplicate the connected-component data.
         */
        const bool isConnectedVertexMarker =
            skeletonMarker.type == visualization_msgs::msg::Marker::CUBE_LIST &&
            skeletonMarker.ns.rfind("connected_vertices_", 0) == 0;

        /*!
         * The raw "edges" marker is used instead of connected_edges_* because
         * connected edge markers may have already been clearance filtered at
         * narrow passages.
         */
        const bool isRawEdgeMarker =
            skeletonMarker.type == visualization_msgs::msg::Marker::LINE_LIST &&
            skeletonMarker.ns == "edges";

        /* Ignore marker types that are not required by this pipeline */
        if (!isConnectedVertexMarker && !isRawEdgeMarker)
        {
            continue;
        }

        /* Ignore markers without any point data */
        if (skeletonMarker.points.empty())
        {
            continue;
        }

        /*!
         * Resolve the marker-local-to-world transformation once and reuse it
         * for every point belonging to this marker.
         */
        tf2::Transform T_world_skeletonMarker;

        if (!getSkeletonMarkerWorldTransform(skeletonMarker,
                                             T_world_skeletonMarker))
        {
            continue;
        }

        /* ------------------------------------------------------------------ *
         * CONNECTED FREE-SPACE COMPONENT
         * ------------------------------------------------------------------ */

        if (isConnectedVertexMarker)
        {
            /* Ignore undersized connected components */
            if (skeletonMarker.points.size() < minimumClusterVertexCount)
            {
                continue;
            }

            std::vector<Eigen::Vector3d> connectedComponentPoints_world;

            connectedComponentPoints_world.reserve(
                skeletonMarker.points.size());

            /* Transform every connected vertex into the world frame */
            for (const geometry_msgs::msg::Point &skeletonPoint_marker :
                 skeletonMarker.points)
            {
                Eigen::Vector3d skeletonPoint_world;

                if (!transformSkeletonPoint(T_world_skeletonMarker,
                                            skeletonPoint_marker,
                                            skeletonPoint_world))
                {
                    continue;
                }

                connectedComponentPoints_world.push_back(skeletonPoint_world);
            }

            /*!
             * Store the component only when enough valid transformed vertices
             * remain.
             */
            if (connectedComponentPoints_world.size() >=
                minimumClusterVertexCount)
            {
                transformedSkeletonClusters_world.push_back(
                    std::move(connectedComponentPoints_world));
            }

            continue;
        }

        /* ------------------------------------------------------------------ *
         * RAW SKELETON EDGES
         * ------------------------------------------------------------------ */

        /*!
         * Each consecutive pair of LINE_LIST points represents one independent
         * edge.
         */
        transformedSkeletonEdges_world.reserve(
            transformedSkeletonEdges_world.size() +
            skeletonMarker.points.size() / 2);

        for (std::size_t pointIndex = 0;
             pointIndex + 1 < skeletonMarker.points.size();
             pointIndex += 2)
        {
            Eigen::Vector3d edgeStart_world;
            Eigen::Vector3d edgeEnd_world;

            const bool isEdgeStartValid =
                transformSkeletonPoint(T_world_skeletonMarker,
                                       skeletonMarker.points[pointIndex],
                                       edgeStart_world);

            const bool isEdgeEndValid =
                transformSkeletonPoint(T_world_skeletonMarker,
                                       skeletonMarker.points[pointIndex + 1],
                                       edgeEnd_world);

            if (!isEdgeStartValid || !isEdgeEndValid)
            {
                continue;
            }

            const double edgeLength_m =
                (edgeEnd_world - edgeStart_world).norm();

            /* Reject invalid and effectively zero-length edges */
            if (!edgeStart_world.allFinite() || !edgeEnd_world.allFinite() ||
                edgeLength_m < 1e-6)
            {
                continue;
            }

            transformedSkeletonEdges_world.emplace_back(edgeStart_world,
                                                        edgeEnd_world);
        }
    }

    /*!
     * Replace the shared buffers only after processing has completed. This
     * avoids exposing partially rebuilt data through these global collections.
     */
    skeletonClusterPoints = std::move(transformedSkeletonClusters_world);

    skeletonEdges = std::move(transformedSkeletonEdges_world);

    /* Store the connected skeleton vertices in the active map */
    p_slamSystem->setSkeletonCluster(skeletonClusterPoints);

    /* Store the complete raw skeleton edges in the active map */
    p_slamSystem->setSkeletonEdges(skeletonEdges);
}

bool transformSkeletonPoint(
    const tf2::Transform            &T_world_skeletonMarker_in,
    const geometry_msgs::msg::Point &skeletonPoint_marker_in,
    Eigen::Vector3d                 &skeletonPoint_world_out)
{
    /* Set an invalid default output in case transformation fails */
    skeletonPoint_world_out =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    /* Reject invalid marker-local coordinates */
    if (!std::isfinite(skeletonPoint_marker_in.x) ||
        !std::isfinite(skeletonPoint_marker_in.y) ||
        !std::isfinite(skeletonPoint_marker_in.z))
    {
        return false;
    }

    const tf2::Vector3 skeletonPoint_marker(skeletonPoint_marker_in.x,
                                            skeletonPoint_marker_in.y,
                                            skeletonPoint_marker_in.z);

    /* Transform the marker-local point into the world frame */
    const tf2::Vector3 skeletonPoint_world =
        T_world_skeletonMarker_in * skeletonPoint_marker;

    skeletonPoint_world_out =
        Eigen::Vector3d(static_cast<double>(skeletonPoint_world.x()),
                        static_cast<double>(skeletonPoint_world.y()),
                        static_cast<double>(skeletonPoint_world.z()));

    return skeletonPoint_world_out.allFinite();
}

bool transformSkeletonPoint(
    const visualization_msgs::msg::Marker &skeletonMarker_in,
    const geometry_msgs::msg::Point       &skeletonPoint_marker_in,
    Eigen::Vector3d                       &skeletonPoint_world_out)
{
    /* Resolve the marker-local-to-world transformation */
    tf2::Transform T_world_skeletonMarker;

    if (!getSkeletonMarkerWorldTransform(skeletonMarker_in,
                                         T_world_skeletonMarker))
    {
        skeletonPoint_world_out =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

        return false;
    }

    /* Transform the supplied point using the resolved transformation */
    return transformSkeletonPoint(T_world_skeletonMarker,
                                  skeletonPoint_marker_in,
                                  skeletonPoint_world_out);
}
