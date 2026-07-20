/**
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VS_GRAPHS_COMMON_H
#define VS_GRAPHS_COMMON_H

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/pcl_node.hpp>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <segmenter_ros/msg/segmenter_data_msg.hpp>
#include <segmenter_ros/msg/vs_graph_data_msg.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/time_synchronizer.hpp>

// This file is created automatically, see here
// http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <vs_graphs/srv/save_map.hpp>

// Transformation process
#include <pcl_ros/transforms.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ORB-SLAM3-specific libraries
#include "ImuTypes.h"
#include "System.h"
#include "Types/SystemParams.h"

// ArUco-ROS library
// #include <aruco_msgs/MarkerArray.h>

// Semantics
#include "Semantic/Marker.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

// Situational Graphs Messages
#include <situational_graphs_msgs/msg/planes_data.hpp>
#include <situational_graphs_msgs/msg/rooms_data.hpp>

// vS-Graphs Custom Messages
#include <vs_graphs/msg/vs_graphs_all_detectdet_rooms.hpp>
#include <vs_graphs/msg/vs_graphs_all_walls_data.hpp>

using json = nlohmann::json;

extern ORB_SLAM3::System         *pSLAM;
extern ORB_SLAM3::System::eSensor sensorType;

extern bool   colorPointcloud;
extern double roll;
extern double pitch;
extern double yaw;
extern bool   pubStaticTransform;
extern bool   pubPointClouds;

extern std::string frameCamera;
extern std::string frameImu;
extern std::string frameWorld;
extern std::string frameMap;
extern std::string frameBC;
extern std::string frameSE;

// TF broadcasters
extern std::shared_ptr<tf2_ros::TransformBroadcaster>       tfBroadcaster;
extern std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;

// List of visited Fiducial Markers in different timestamps
extern std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;

/*!
 * @brief       Connected Voxblox verticies grouped by connected component
 */
extern std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

/*!
 * @brief       Connected voxblox graph edges transformed into the world frame
 *
 * @note        Each pair contains the start and end point of one skeleton
 *              graph edge
 */
extern std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

// List of GNN-based room candidates
extern std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;

// List of wall publishers for GNN-based room detection variants
extern rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr
    pubAllWalls_new;
extern rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr
    pubAllWalls_legacy;

extern rclcpp::Time                                lastPlanePublishTime;
extern std::shared_ptr<image_transport::Publisher> pubTrackingImage;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
extern rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr
    pubKFImage;
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pubCameraPose;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubAllMappoints;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubTrackedMappoints;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubSegmentedPointcloud;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubWorldFramePointCloud;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubCameraPoseVis;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubKeyFrameMarker;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubStructuralElements;

class MapPointStruct
{
    int             clusterId;
    Eigen::Vector3f coordinates;
    MapPointStruct(Eigen::Vector3f coords) :
        clusterId(-1),
        coordinates(coords)
    {}
};

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void setupServices(std::shared_ptr<rclcpp::Node>, const std::string &);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 * @param       TODO
 */
void publishFramePointCloud(
    Sophus::SE3f,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL,
    rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 * @param       TODO
 */
void publishTopics(
    rclcpp::Time,
    Eigen::Vector3f = Eigen::Vector3f::Zero(),
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL = nullptr);

/*!
 * @brief
 * @param
 * @param
 * @param
 */
void setupPublishers(
    std::shared_ptr<rclcpp::Node>                    node,
    std::shared_ptr<image_transport::ImageTransport> image_transport,
    const std::string                               &node_name);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishTrackingImage(cv::Mat, rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishCameraPose(Sophus::SE3f, rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 */
void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *>);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishPlanes(std::vector<ORB_SLAM3::Plane *>, rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 * @param       TODO
 * @param       TODO
 */
void publishTFTransform(Sophus::SE3f, string, string, rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);

/*!
 * @brief       TODO
 *
 * @param[in]   markers_in
 *              TODO
 */
void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *> markers_in);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);

/*!
 * @brief       TODO
 * @param       TODO
 * @param       TODO
 */
void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);

/*!
 * @brief       TODO
 *
 * @param       TODO
 * @param       TODO
 * @param       TODO
 * @param       TODO
 */
void publishBodyOdometry(Sophus::SE3f,
                         Eigen::Vector3f,
                         Eigen::Vector3f,
                         rclcpp::Time);

/*!
 * @brief       Calculates the displayed structural-graph position of a room.
 *
 * @param[in]   room_in
 *              Room whose displayed position is required.
 *
 * @param[in]   msgTime_in
 *              Timestamp used for the TF lookup.
 *
 * @param[out]  roomPointSE_out
 *              Room position represented in frameSE.
 *
 * @param[out]  roomPointWorld_out
 *              Room position represented in frameWorld for connection lines.
 *
 * @return      True when the room centroid is valid and the transformation
 *              succeeds.
 */
bool getRoomDisplayPoints(ORB_SLAM3::Room                  *room_in,
                          const rclcpp::Time               &msgTime_in,
                          geometry_msgs::msg::PointStamped &roomPointSE_out,
                          geometry_msgs::msg::PointStamped &roomPointWorld_out);

/*!
 * @brief       Calculates the displayed structural-graph position of a
 *              passage.
 *
 * @param[in]   passage_in
 *              Passage whose displayed position is required.
 *
 * @param[in]   msgTime_in
 *              Timestamp used for the frame transformations.
 *
 * @param[in]   verticalOffset_in
 *              Distance below the normal room-node level.
 *
 * @param[out]  passagePointSE_out
 *              Displayed passage position in the structural-element frame.
 *
 * @param[out]  passagePointWorld_out
 *              Displayed passage position transformed back into the world
 *              frame for connection lines.
 *
 * @return      True when both transformations succeed.
 */
bool getPassageDisplayPoints(
    ORB_SLAM3::Passage               *passage_in,
    const rclcpp::Time               &msgTime_in,
    const double                      verticalOffset_in,
    geometry_msgs::msg::PointStamped &passagePointSE_out,
    geometry_msgs::msg::PointStamped &passagePointWorld_out);

/*!
 * @brief       Method which publishes the structural elements of the S-Graph.
 *
 * @param[in]   rooms_in
 *              Rooms within the S-Graph
 *
 * @param[in]   floors_in
 *              Floors within the S-Graph
 *
 * @param[in]   passage_in
 *              Passages within the S-Graph
 *
 * @param[in]   msgTime_in
 *              Time of the message to be published
 */
extern void publishStructuralElements(
    const std::vector<ORB_SLAM3::Room *>    rooms_in,
    const std::vector<ORB_SLAM3::Floor *>   floors_in,
    const std::vector<ORB_SLAM3::Passage *> passages_in,
    const rclcpp::Time                      msgTime_in);

/*!
 * @brief       Publishes all mapped walls to detect possible rooms (mainly used
 *              in GNN-based room detector).
 *
 * @param       walls
 *              The vector of mapped walls to be published.
 *
 * @param       time
 *              The timestamp for the message.
 */
void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *>, rclcpp::Time);

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *>);

void saveMapService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request>  request,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> response);

void saveTrajectoryService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request>  request,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> response);

void saveMapPointsAsPCDService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request>  request,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> response);

/**
 * @brief       Converts a SE3f to a cv::Mat
 *
 * @param       data
 *              The SE3f data to be converted
 */
cv::Mat SE3fToCvMat(Sophus::SE3f data);

/**
 * @brief       Converts a SE3f to a tf::Transform
 *
 * @param       data
 *              The SE3f data to be converted
 */
tf2::Transform SE3fToTFTransform(Sophus::SE3f data);

/**
 * @brief       Converts a vector of MapPoints to a PointCloud2 message
 *
 * @param       mapPoints
 *              The vector of MapPoints to be converted
 *
 * @param       msgTime
 *              The timestamp for the PointCloud2 message
 */
sensor_msgs::msg::PointCloud2
    mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints,
                         rclcpp::Time                       msgTime);

/*!
 * @brief       Publishes a static transformation (TF) between two coordinate
 *              frames and define a fixed spatial relationship among them.
 *
 * @param       parentFrameId
 *              The parent frame ID for the static transformation
 *
 * @param       childFrameId
 *              The child frame ID for the static transformation
 *
 * @param       msgTime
 *              The timestamp for the transformation message
 */
void publishStaticTFTransform(string       parentFrameId,
                              string       childFrameId,
                              rclcpp::Time msgTime);

/**
 * @brief       Publishes the free space clusters obtained from
 *              `voxblox_skeleton` as a PointCloud2 message
 *
 * @param       skeletonClusterPoints
 *              The list of free space cluster points
 *
 * @param       msgTime
 *              The timestamp for the PointCloud2 message
 */
void publishFreeSpaceClusters(std::vector<std::vector<Eigen::Vector3d>>,
                              rclcpp::Time);

/**
 * @brief       Adds the markers to the buffer to be processed
 *
 * @param       markerArray
 *              The array of markers received from `aruco_ros`
 */
// void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray);

/**
 * @brief       Avoids adding duplicate markers to the buffer by checking the
 *              timestamp.
 *
 * @param       frameTimestamp
 *              The timestamp of the frame that captured the marker
 */
std::pair<double, std::vector<ORB_SLAM3::Marker *>>
    findNearestMarker(double frameTimestamp);

/*!
 * @brief       Transforms a Voxblox skeleton point into the world frame.
 *
 * @param[in]   marker_in
 *              Marker containing the point and its source frame.
 *
 * @param[in]   point_in
 *              Skeleton point to transform.
 *
 * @param[out]  transformedPoint_out
 *              Skeleton point represented in the world frame.
 *
 * @return      True if the point was successfully transformed.
 */
bool transformSkeletonPoint(const visualization_msgs::msg::Marker &marker_in,
                            const geometry_msgs::msg::Point       &point_in,
                            Eigen::Vector3d &transformedPoint_out);

/**
 * @brief       Gets skeleton voxels from `voxblox_skeleton` to be processed
 *
 * @param       skeletonArray
 *              The array of skeleton voxels received
 */
void setVoxbloxSkeletonCluster(
    const visualization_msgs::msg::MarkerArray &skeletonArray);

/*!
 * @brief       Gets the set of room candidates detected by the GNN-based room
 *              detection module Mainly designed for the legacy version of the
 *              GNN-based room detector
 *
 * @param       msgGNNRooms
 *              The message containing the detected room candidates
 */
void setGNNBasedRoomCandidates(
    const situational_graphs_msgs::msg::RoomsData &msgGNNRooms);

/*!
 * @brief       Gets the set of room candidates detected by the GNN-based room
 *              detection module Mainly designed for the new version of the
 *              GNN-based room detector.
 *
 * @param       msgGNNRooms
 *              The message containing the detected room candidates
 */
void setGNNBasedRoomCandidates(
    const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms);

#endif // VS_GRAPHS_COMMON_H