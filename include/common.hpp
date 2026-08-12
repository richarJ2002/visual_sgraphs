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

/* -------------------------------------------------------------------------- *
 * STANDARD LIBRARY
 * -------------------------------------------------------------------------- */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

/* -------------------------------------------------------------------------- *
 * EIGEN
 * -------------------------------------------------------------------------- */

#include <Eigen/Dense>

/* -------------------------------------------------------------------------- *
 * JSON
 * -------------------------------------------------------------------------- */

#include <nlohmann/json.hpp>

/* -------------------------------------------------------------------------- *
 * OPENCV
 * -------------------------------------------------------------------------- */

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 CORE
 * -------------------------------------------------------------------------- */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 MESSAGE FILTERS
 * -------------------------------------------------------------------------- */

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/time_synchronizer.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 GEOMETRY MESSAGES
 * -------------------------------------------------------------------------- */

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 NAVIGATION MESSAGES
 * -------------------------------------------------------------------------- */

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 SENSOR MESSAGES
 * -------------------------------------------------------------------------- */

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 STANDARD MESSAGES
 * -------------------------------------------------------------------------- */

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>

/* -------------------------------------------------------------------------- *
 * ROS 2 VISUALISATION MESSAGES
 * -------------------------------------------------------------------------- */

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/* -------------------------------------------------------------------------- *
 * IMAGE TRANSPORT AND CV BRIDGE
 * -------------------------------------------------------------------------- */

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

/* -------------------------------------------------------------------------- *
 * TF2
 * -------------------------------------------------------------------------- */

#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/* -------------------------------------------------------------------------- *
 * POINT CLOUD LIBRARY
 * -------------------------------------------------------------------------- */

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/pcl_node.hpp>
#include <pcl_ros/transforms.hpp>

/* -------------------------------------------------------------------------- *
 * RVIZ
 * -------------------------------------------------------------------------- */

#include <rviz_visual_tools/rviz_visual_tools.hpp>

/* -------------------------------------------------------------------------- *
 * SEGMENTATION MESSAGES
 * -------------------------------------------------------------------------- */

#include <segmenter_ros/msg/segmenter_data_msg.hpp>
#include <segmenter_ros/msg/vs_graph_data_msg.hpp>

/* -------------------------------------------------------------------------- *
 * SITUATIONAL GRAPHS MESSAGES
 * -------------------------------------------------------------------------- */

#include <situational_graphs_msgs/msg/planes_data.hpp>
#include <situational_graphs_msgs/msg/rooms_data.hpp>

/* -------------------------------------------------------------------------- *
 * VISUAL S-GRAPHS MESSAGES AND SERVICES
 * -------------------------------------------------------------------------- */

#include <vs_graphs/msg/vs_graphs_all_detectdet_rooms.hpp>
#include <vs_graphs/msg/vs_graphs_all_walls_data.hpp>
#include <vs_graphs/srv/get_mission_health.hpp>
#include <vs_graphs/srv/save_map.hpp>

/* -------------------------------------------------------------------------- *
 * ORB-SLAM3
 * -------------------------------------------------------------------------- */

#include "ImuTypes.h"
#include "System.h"
#include "Types/SystemParams.h"

/* -------------------------------------------------------------------------- *
 * SEMANTIC ELEMENTS
 * -------------------------------------------------------------------------- */

#include "Semantic/Marker.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
using json = nlohmann::json;

/* -------------------------------------------------------------------------- *
 * ORB-SLAM3 STATE
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Pointer to the active ORB-SLAM3 system.
 *
 * @note        The pointer is assigned during node initialisation and is used
 *              by the common publication, transformation, and service
 *              functions to access the current SLAM state.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern ORB_SLAM3::System *p_slamSystem;

/*!
 * @brief       Sensor configuration used by the active ORB-SLAM3 system.
 *
 * @note        The sensor type determines whether inertial publishers and
 *              sensor-specific coordinate conventions are required.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern ORB_SLAM3::System::eSensor sensorType;

/* -------------------------------------------------------------------------- *
 * COMMON CONFIGURATION
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Controls whether map-point clouds preserve their stored colour
 *              information.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern bool colorPointcloud;

/*!
 * @brief       Roll component of the configured static transformation, in
 *              radians.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern double roll;

/*!
 * @brief       Pitch component of the configured static transformation, in
 *              radians.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern double pitch;

/*!
 * @brief       Yaw component of the configured static transformation, in
 *              radians.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern double yaw;

/*!
 * @brief       Controls whether the configured static world-to-map
 *              transformation is published.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern bool pubStaticTransform;

/*!
 * @brief       Controls whether optional point-cloud visualisation topics are
 *              published.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern bool pubPointClouds;

/* -------------------------------------------------------------------------- *
 * COORDINATE-FRAME IDENTIFIERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Identifier of the global world coordinate frame.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::string frameWorld;

/*!
 * @brief       Identifier of the active camera coordinate frame.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::string frameCamera;

/*!
 * @brief       Identifier of the IMU or vehicle-body coordinate frame.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::string frameImu;

/*!
 * @brief       Identifier of the configured map coordinate frame.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::string frameMap;

/*!
 * @brief       Identifier of the building-component coordinate frame.
 *
 * @note        Plane and wall geometry may be represented in this frame before
 *              being transformed into frameWorld.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::string frameBC;

/*!
 * @brief       Identifier of the structural-element visualisation frame.
 *
 * @note        Room and passage display markers may be represented in this
 *              frame.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::string frameSE;

/* -------------------------------------------------------------------------- *
 * TF INTERFACES
 * -------------------------------------------------------------------------- */

/*!
 * @brief       TF buffer used to query transformations between ROS coordinate
 *              frames.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

/*!
 * @brief       TF listener responsible for populating tfBuffer_.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::shared_ptr<tf2_ros::TransformListener> tfListener_;

/*!
 * @brief       Broadcaster used to publish dynamic ROS transformations.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

/*!
 * @brief       Broadcaster used to publish static ROS transformations.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;

/* -------------------------------------------------------------------------- *
 * SHARED DATA BUFFERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Fiducial markers grouped according to their associated frame or
 *              observation timestamp.
 *
 * @note        The buffer is used to associate asynchronously received marker
 *              observations with SLAM frames.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;

/*!
 * @brief       Candidate rooms generated by the GNN-based room-detection
 *              pipeline.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;

/*!
 * @brief       Connected Voxblox skeleton vertices grouped by connected
 *              free-space component.
 *
 * @note        Stored points are expected to be expressed in frameWorld.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

/*!
 * @brief       Raw Voxblox skeleton graph edges expressed in frameWorld.
 *
 * @note        Each pair contains the start and end point of one skeleton graph
 *              edge.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

/* -------------------------------------------------------------------------- *
 * PUBLICATION STATE
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Timestamp of the most recent plane visualisation publication.
 *
 * @note        The timestamp is used to rate-limit expensive plane point-cloud
 *              and visualisation-marker generation.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Time lastPlanePublishTime;

/* -------------------------------------------------------------------------- *
 * BASIC SLAM PUBLISHERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Publisher for the ordered keyframe pose path.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubKeyFrameList;

/*!
 * @brief       Publisher for the estimated IMU or body odometry.
 *
 * @note        This publisher is created only for inertial sensor
 *              configurations.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;

/*!
 * @brief       Publisher for the complete ORB-SLAM3 map-point cloud.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubAllMappoints;

/*!
 * @brief       Publisher for the current camera pose.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pubCameraPose;

/*!
 * @brief       Publisher for keyframe images sent to the semantic-segmentation
 *              pipeline.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr
    pubKFImage;

/*!
 * @brief       Publisher for map points currently tracked by ORB-SLAM3.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubTrackedMappoints;

/*!
 * @brief       Publisher for the current camera-frame point cloud consumed by
 *              Voxblox.
 *
 *              The points remain in frameCamera. Voxblox obtains the
 *              camera-to-world transform at the message timestamp and uses the
 *              camera origin for free-space ray integration.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    p_voxbloxInputPointCloudPublisher;

/*!
 * @brief       Publisher for the active SLAM-map revision.
 *
 *              A revision changes whenever the active map changes identity or
 *              receives a loop-closure/global-BA correction. Derived mapping
 *              consumers use it to invalidate geometry integrated in the old
 *              coordinate system.
 */
extern rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
    p_mapRevisionPublisher;

/*!
 * @brief       Publisher for keyframe position visualisation markers.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubKeyFrameMarker;

/*!
 * @brief       Publisher for coloured Voxblox free-space cluster point clouds.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubFreespaceCluster;

/*!
 * @brief       Publisher for the camera-pose mesh visualisation marker.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubCameraPoseVis;

/*!
 * @brief       Image-transport publisher for the current ORB-SLAM3 tracking
 *              image.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern std::shared_ptr<image_transport::Publisher> pubTrackingImage;

/* -------------------------------------------------------------------------- *
 * ENTITY PUBLISHERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Publisher for door visualisation markers.
 *
 * @note        This publisher may be removed when door entities have been fully
 *              replaced by passage entities.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubDoor;

/*!
 * @brief       Publisher for globally mapped fiducial marker visualisations.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubFiducialMarker;

/* -------------------------------------------------------------------------- *
 * BUILDING-COMPONENT PUBLISHERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Publisher for plane labels and plane-normal markers.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubPlaneLabel;

/*!
 * @brief       Publisher for the aggregated building-component point cloud.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubBuildingComponents;

/*!
 * @brief       Publisher for the latest class-coloured segmented point cloud.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubSegmentedPointcloud;

/* -------------------------------------------------------------------------- *
 * MAPPED-WALL PUBLISHERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Publisher for the mapped-wall representation used by the
 *              GNN-based room-detection pipeline.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr
    pubAllWalls_new;

/*!
 * @brief       Legacy publisher for mapped planes using the previous
 *              situational-graphs message interface.
 *
 * @note        Remove this declaration and its corresponding definition when
 *              no remaining source files reference the legacy publisher.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr
    pubAllWalls_legacy;

/* -------------------------------------------------------------------------- *
 * STRUCTURAL-ELEMENT PUBLISHERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Publisher for room, floor, passage, label, and semantic
 *              association visualisation markers.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubStructuralElements;

/* -------------------------------------------------------------------------- *
 * SERVICE SERVERS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Service server used to save the complete ORB-SLAM3 map.
 *
 * @note        The shared pointer must remain alive for the service to remain
 *              available.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMap;

/*!
 * @brief       Service server used to save the current map points as a PCD
 *              file.
 *
 * @note        The shared pointer must remain alive for the service to remain
 *              available.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMapPoints;

/*!
 * @brief       Service server used to save the estimated camera and keyframe
 *              trajectories.
 *
 * @note        The shared pointer must remain alive for the service to remain
 *              available.
 *
 * @note        Global variable declared in `commonStat.cpp`
 */
extern rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveTrajectory;

extern rclcpp::Service<vs_graphs::srv::GetMissionHealth>::SharedPtr
    srvGetMissionHealth;

class MapPointStruct
{
    int             clusterId;
    Eigen::Vector3f coordinates;
    MapPointStruct(Eigen::Vector3f coords) :
        clusterId(-1),
        coordinates(coords)
    {}
};

/* -------------------------------------------------------------------------- *
 * GLOBAL VARIABLES
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Service handle for serializing the complete semantic map.
 */
extern rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMap;

/*!
 * @brief       Service handle for exporting mapped points as a PCD file.
 */
extern rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMapPoints;

/*!
 * @brief       Service handle for exporting the estimated camera trajectory.
 */
extern rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveTrajectory;

/* -------------------------------------------------------------------------- *
 * FUNCTION DEFINITIONS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       Appends floor, floor-label, and floor-to-room association
 *              markers to a structural marker array.
 *
 * @param[in]   mappedFloors_in
 *              Collection of mapped floor elements to process.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the generated markers.
 *
 * @param[out]  structuralElementMarkerArray_out
 *              Marker array to which the generated floor markers are appended.
 */
extern void appendFloorMarkers(
    const std::vector<ORB_SLAM3::Floor *> &mappedFloors_in,
    const rclcpp::Time                    &msgTime_s_in,
    visualization_msgs::msg::MarkerArray  &structuralElementMarkerArray_out);

/*!
 * @brief       Appends passage and passage-label markers to a structural
 *              marker array with situational awareness coloring.
 *
 * @param[in]   mappedRooms_in
 *              Collection of mapped room elements (to count room-passage
 * links).
 *
 * @param[in]   mappedPassages_in
 *              Collection of mapped passage elements to process.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the generated markers.
 *
 * @param[out]  structuralElementMarkerArray_out
 *              Marker array to which the generated passage markers are
 *              appended.
 */
extern void appendPassageMarkers(
    const std::vector<ORB_SLAM3::Passage *> &mappedPassages_in,
    const rclcpp::Time                      &msgTime_s_in,
    visualization_msgs::msg::MarkerArray    &structuralElementMarkerArray_out);

/*!
 * @brief       Appends room, corridor, label, wall-association, and
 *              passage-association markers to a structural marker array.
 *
 * @param[in]   mappedRooms_in
 *              Collection of mapped room elements to process.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the generated markers.
 *
 * @param[out]  structuralElementMarkerArray_out
 *              Marker array to which the generated room markers are appended.
 */
extern void appendRoomMarkers(
    const std::vector<ORB_SLAM3::Room *>  &mappedRooms_in,
    const std::vector<ORB_SLAM3::Floor *> &mappedFloors_in,
    const rclcpp::Time                    &msgTime_s_in,
    visualization_msgs::msg::MarkerArray  &structuralElementMarkerArray_out);

/*!
 * @brief Computes the ordered horizontal corner polygon of a room boundary.
 *
 * Each wall is treated as a vertical line projected onto the plane orthogonal
 * to the room ground normal. The walls are ordered by their centroid angle
 * around the room centroid and consecutive supporting lines are intersected to
 * recover the shared corners. Used to draw the green closed-loop marker when a
 * room boundary is validated as COMPLETE.
 *
 * @param[in] room_in Room whose boundary corners are required.
 *
 * @return Ordered world-frame corner points forming the closed boundary loop.
 *         The loop is empty for a degenerate room or when any consecutive wall
 *         pair is parallel.
 */
extern std::vector<Eigen::Vector3d>
    computeRoomCorners(const ORB_SLAM3::Room *room_in);

/*!
 * @brief       Function which clears the cluster points from the keyframe
 *              vectors.
 *
 * @param[in]   keyframeVector_in
 *              Vector of keyframes where the cluster clouds will be cleared
 *              from.
 */
extern void
    clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframeVector_in);

/*!
 * @brief       Converts a Sophus SE(3) transformation into a 4-by-4 OpenCV
 *              homogeneous transformation matrix.
 *
 *              The returned matrix uses single-precision floating-point values
 *              and contains the rotation and translation components in the
 *              following form:
 *
 *                  [ R  t ]
 *                  [ 0  1 ]
 *
 * @param[in]   transformation_SE3f_in
 *              Sophus SE(3) transformation to convert.
 *
 * @return      A 4-by-4 OpenCV matrix of type CV_32FC1 containing the
 *              homogeneous transformation matrix.
 */
extern cv::Mat convertSE3fToCvMat(const Sophus::SE3f &transformation_SE3f_in);

/*!
 * @brief       Avoids adding duplicate markers to the buffer by checking the
 *              timestamp.
 *
 * @param[in]   frameTimestamp_in
 *              The timestamp of the frame that captured the marker
 */
extern std::pair<double, std::vector<ORB_SLAM3::Marker *>>
    findNearestMarker(double frameTimestamp_in);

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
extern bool getPassageDisplayPoints(
    ORB_SLAM3::Passage               *passage_in,
    const rclcpp::Time               &msgTime_in,
    const double                      verticalOffset_in,
    geometry_msgs::msg::PointStamped &passagePointSE_out,
    geometry_msgs::msg::PointStamped &passagePointWorld_out);

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
extern bool
    getRoomDisplayPoints(ORB_SLAM3::Room                  *room_in,
                         const rclcpp::Time               &msgTime_in,
                         geometry_msgs::msg::PointStamped &roomPointSE_out,
                         geometry_msgs::msg::PointStamped &roomPointWorld_out);

/*!
 * @brief       Resolves the transformation from a Voxblox skeleton marker's
 *              local coordinate frame into the configured world frame.
 *
 *              The source frame is obtained from the marker header. frameMap
 *              is used only when the marker does not contain a frame
 *              identifier. The marker pose is included in the resulting
 *              transformation.
 *
 * @param[in]   skeletonMarker_in
 *              Voxblox marker containing the source frame and marker pose.
 *
 * @param[out]  T_world_skeletonMarker_out
 *              Transformation from the marker-local coordinate frame into the
 *              world frame.
 *
 * @return      True when the transformation was resolved successfully;
 *              otherwise false.
 */
extern bool getSkeletonMarkerWorldTransform(
    const visualization_msgs::msg::Marker &skeletonMarker_in,
    tf2::Transform                        &T_world_skeletonMarker_out);

/*!
 * @brief       Converts a vector of MapPoints to a PointCloud2 message
 *
 * @param[in]   mapPoints_in
 *              The vector of MapPoints to be converted
 *
 * @param[in]   msgTime_in
 *              The timestamp for the PointCloud2 message
 */
extern sensor_msgs::msg::PointCloud2
    mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints_in,
                         rclcpp::Time                       msgTime_in);

/*!
 * @brief       Publishes all mapped walls to detect possible rooms.
 *
 * @param[in]   wallsList_in
 *              The vector of mapped walls to be published.
 *
 * @param       msgTime_s_in
 *              The timestamp for the message.
 */
extern void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *> wallsList_in,
                                  rclcpp::Time                    msgTime_s_in);

/*!
 * @brief       Calls mapPointToPointscloud() function and then publishes the
 *              point cloud through the pubAllMappoints publisher.
 *
 * @param[in]   allMapPoints_in
 *              Vector of mapped points
 *
 * @param[in]   msgTime_s_in
 *              Ros time msg
 */
extern void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> allMapPoints_in,
                             rclcpp::Time                       msgTime_in);

/*!
 * @brief       Publishes the estimated body pose and velocity as a ROS
 *              odometry message.
 *
 *              The pose describes the body frame relative to the world frame.
 *              The linear and angular velocities are expected to be expressed
 *              in the world frame.
 *
 *              The published message uses frameWorld as the parent frame and
 *              frameImu as the child frame.
 *
 * @param[in]   robotPose_BodToWorld_in
 *              Transformation describing the pose of the body frame relative
 *              to the world frame.
 *
 * @param[in]   linearVelocity_World_mps_in
 *              Linear velocity of the body expressed in the world frame, in
 *              metres per second.
 *
 * @param[in]   angularVelocity_Bod_radps_in
 *              Angular velocity of the body expressed in the body frame, in
 *              radians per second.
 *
 * @param[in]   msgTims_s_in
 *              ROS timestamp associated with the odometry estimate.
 */
extern void
    publishBodyOdometry(const Sophus::SE3f    &robotPose_BodToWorld_in,
                        const Eigen::Vector3f &linearVelocity_World_mps_in,
                        const Eigen::Vector3f &angularVelocity_Bod_radps_in,
                        const rclcpp::Time    &msgTims_s_in);

/*!
 * @brief       Publishes the estimated camera pose as a ROS pose message and
 *              as a camera mesh marker for visualisation.
 *
 *              The supplied transformation describes the camera frame relative
 *              to the world frame. Both the pose message and visualisation
 *              marker are therefore published in frameWorld.
 *
 * @param[in]   cameraPose_World_in
 *              Transformation describing the pose of the camera frame relative
 *              to the world frame.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp associated with the camera-pose estimate.
 */
extern void publishCameraPose(const Sophus::SE3f &cameraPose_World_in,
                              const rclcpp::Time &msgTime_s_in);

/*!
 * @brief       Publishes the globally mapped fiducial markers as ROS
 *              visualisation mesh markers.
 *
 *              Each valid fiducial marker is represented using its persistent
 *              marker identifier and global pose. The resulting marker array is
 *              published in frameWorld.
 *
 * @param[in]   fiducialMarkers_in
 *              Collection of mapped fiducial markers to publish. Null marker
 *              pointers are ignored.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp associated with the marker visualisation update.
 */
extern void publishFiducialMarkers(
    const std::vector<ORB_SLAM3::Marker *> &fiducialMarkers_in,
    const rclcpp::Time                     &msgTime_s_in);

/*!
 * @brief       Transforms an input point cloud from the camera frame into the
 *              world frame and publishes the transformed cloud.
 *
 *              The supplied transformation describes the camera pose relative
 *              to the world frame. The input point-cloud coordinates are
 *              assumed to be expressed in the camera frame.
 *
 * @param[in]   cameraPose_CameraToWorld_in
 *              Transformation from the camera frame to the world frame.
 *
 * @param[in]   pointCloudCameraMessage_in
 *              Input ROS point-cloud message whose points are expressed in the
 *              camera frame.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the published world-frame point
 *              cloud.
 */
extern void
    publishFramePointCloud(const Sophus::SE3f &cameraPose_CameraToWorld_in,
                           const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                                              &pointCloudCameraMessage_in,
                           const rclcpp::Time &msgTime_s_in);

/*!
 * @brief       Converts the supplied Voxblox free-space clusters into a
 *              coloured ROS point-cloud message and publishes the result.
 *
 *              Each connected free-space cluster is assigned a fixed colour so
 *              that separate clusters can be distinguished in RViz. The input
 *              cluster points are expected to already be expressed in the
 *              world frame.
 *
 * @param[in]   freeSpaceClusters_World_in
 *              Collection of connected free-space clusters. Each cluster
 *              contains three-dimensional points expressed in the world frame,
 *              in metres.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the published point-cloud message.
 */
extern void publishFreeSpaceClusters(
    const std::vector<std::vector<Eigen::Vector3d>> &freeSpaceClusters_World_in,
    const rclcpp::Time                              &msgTime_s_in);

/*!
 * @brief       Publishes images from keyframes that have not yet been sent for
 *              semantic segmentation.
 *
 *              Each unpublished keyframe image is converted from its OpenCV
 *              representation into a ROS image message and packaged together
 *              with the persistent keyframe identifier. After publication, the
 *              keyframe is marked as published to prevent duplicate processing.
 *
 * @param[in]   keyFrames_in
 *              Collection of keyframes to inspect. Null keyframe pointers and
 *              keyframes that have already been published are ignored. The
 *              pointed-to keyframe objects may be updated by setting their
 *              isPublished flag.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to each published keyframe image
 *              message.
 */
extern void publishKeyFrameImages(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                       &msgTime_s_in);

/*!
 * @brief       Publishes the positions of all valid keyframes as an RViz
 *              sphere-list marker and publishes their ordered poses as a ROS
 *              path.
 *
 *              Keyframes are ordered by their persistent identifier before
 *              publication. Each keyframe pose is expected to describe the
 *              keyframe camera frame relative to the world frame.
 *
 * @param[in]   keyFrames_in
 *              Collection of keyframes to publish. Null and invalid keyframes
 *              are ignored.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the marker array and path messages.
 */
extern void publishKeyFrameMarkers(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                       &msgTime_s_in);

/*!
 * @brief       Publishes valid mapped planes as an aggregated
 *              building-component point cloud and as RViz label and normal
 *              markers.
 *
 *              Plane point clouds, centroids, and normals are assumed to be
 *              expressed in frameBC. Publication is rate-limited using
 *              lastPlanePublishTime to reduce repeated point-cloud conversion
 *              and marker construction.
 *
 * @param[in]   mappedPlanes_in
 *              Collection of mapped planes to publish. Null, invalid,
 *              undefined, and empty planes are ignored.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the published point cloud and
 *              visualisation markers.
 */
extern void
    publishPlanes(const std::vector<ORB_SLAM3::Plane *> &mappedPlanes_in,
                  const rclcpp::Time                    &msgTime_s_in);

/*!
 * @brief       Publishes the most recent available semantically segmented
 *              keyframe point cloud.
 *
 *              The function searches the supplied keyframes from newest to
 *              oldest and selects the first keyframe containing non-empty
 *              class-specific point clouds. These point clouds are combined
 *              into a single cloud for visualisation.
 *
 *              Ground points, represented by class index zero, are coloured
 *              green. Wall points, represented by class index one, are
 *              coloured red. Points belonging to other classes retain their
 *              existing colours.
 *
 * @param[in]   keyFrames_in
 *              Ordered collection of keyframes to inspect. Null keyframe
 *              pointers are ignored. Class-specific point clouds belonging to
 *              the selected keyframe and older keyframes are cleared after
 *              processing.
 */
extern void publishSegmentedCloud(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in);

/*!
 * @brief       Publishes a static transformation between two ROS coordinate
 *              frames.
 *
 *              The transformation contains zero translation and uses the
 *              globally configured roll, pitch, and yaw angles to define the
 *              orientation of the child frame relative to the parent frame.
 *
 * @param[in]   parentFrameId_in
 *              Identifier of the parent coordinate frame.
 *
 * @param[in]   childFrameId_in
 *              Identifier of the child coordinate frame.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the static transformation.
 */
extern void publishStaticTFTransform(const std::string  &parentFrameId_in,
                                     const std::string  &childFrameId_in,
                                     const rclcpp::Time &msgTime_s_in);

/*!
 * @brief       Publishes the current semantic structural graph as ROS
 *              visualisation markers.
 *
 *              Confirmed rooms, corridors, floors, passages, labels, and their
 *              association edges are converted into RViz markers. Invalid,
 *              provisional, and bad semantic elements are excluded or removed
 *              from the published graph.
 *
 *              Room and passage display positions may be represented in
 *              frameSE, while physical association edges and floor elements
 *              are represented in frameWorld.
 *
 * @param[in]   mappedRooms_in
 *              Collection of mapped room and corridor elements to publish.
 *              Null, bad, and provisional room elements are ignored or
 *              removed from the published visualisation.
 *
 * @param[in]   mappedFloors_in
 *              Collection of mapped floor elements to publish. Floors without
 *              valid associated rooms are ignored.
 *
 * @param[in]   mappedPassages_in
 *              Collection of mapped passage elements to publish. Null passage
 *              pointers are ignored.
 *
 * @param[in]   messageTimestamp_in
 *              ROS timestamp assigned to all generated structural-element
 *              markers.
 */
extern void publishStructuralElements(
    const std::vector<ORB_SLAM3::Room *>    &mappedRooms_in,
    const std::vector<ORB_SLAM3::Floor *>   &mappedFloors_in,
    const std::vector<ORB_SLAM3::Passage *> &mappedPassages_in,
    const rclcpp::Time                      &msgTime_s_in);

/*!
 * @brief       Publishes a dynamic transformation between two ROS coordinate
 *              frames.
 *
 *              The supplied transformation describes the pose of the child
 *              frame relative to the parent frame. Its translation and
 *              orientation are copied into a TransformStamped message and
 *              broadcast through tfBroadcaster.
 *
 * @param[in]   transform_ParentToChild_in
 *              Transformation describing the child frame relative to the
 *              parent frame.
 *
 * @param[in]   parentFrameId_in
 *              Identifier of the parent coordinate frame.
 *
 * @param[in]   childFrameId_in
 *              Identifier of the child coordinate frame.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the published transformation.
 */
extern void publishTFTransform(const Sophus::SE3f &transform_ParentToChild_in,
                               const std::string  &parentFrameId_in,
                               const std::string  &childFrameId_in,
                               const rclcpp::Time &msgTime_s_in);

/*!
 * @brief       Publishes the current Visual S-Graphs and ORB-SLAM3 state to
 *              the configured ROS topics.
 *
 *              The function publishes the camera pose, TF transformations,
 *              current frame point cloud, keyframes, fiducial markers,
 *              tracking image, structural elements, mapped planes, map
 *              points, free-space clusters, and inertial odometry when
 *              available.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp associated with the current SLAM update.
 *
 * @param[in]   angularVelocity_body_radps_in
 *              Angular velocity measured by the IMU and expressed in the body
 *              frame, in radians per second. This value is used only for
 *              inertial sensor configurations.
 *
 * @param[in]   pointCloud_cameraMessage_in
 *              Current point-cloud message whose points are assumed to be
 *              expressed in the camera frame.
 */
extern void publishTopics(const rclcpp::Time    &msgTime_s_in,
                          const Eigen::Vector3f &angularVelocity_body_radps_in,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                              &pointCloud_cameraMessage_in);

/**
 * Converts a direct cloud into the configured optical camera frame.
 * The Gazebo FLU conversion is applied only when explicitly enabled.
 */
extern bool preparePointCloudForTracking(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointCloudMessage_in,
    bool                                           directGazeboFluCloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr        &pointCloud_camera_out,
    sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointCloudCameraMessage_out,
    std::string                                   &failureReason_out);

/*!
 * @brief       Converts the currently tracked ORB-SLAM3 map points into a ROS
 *              point-cloud message and publishes the result.
 *
 *              The generated point cloud is expressed in frameWorld. An empty
 *              point cloud may still be published so that previously displayed
 *              tracked points are cleared from RViz when tracking is lost.
 *
 * @param[in]   trackedMapPoints_in
 *              Collection of map points currently tracked by ORB-SLAM3. Null
 *              and bad map points should be ignored by mapPointToPointcloud().
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the published point-cloud message.
 */
extern void publishTrackedPoints(
    const std::vector<ORB_SLAM3::MapPoint *> &trackedMapPoints_in,
    const rclcpp::Time                       &msgTime_s_in);

/*!
 * @brief       Converts the current ORB-SLAM3 tracking image into a ROS image
 *              message and publishes it using the tracking-image publisher.
 *
 *              The input image is expected to use the OpenCV BGR8 format. The
 *              published image is associated with frameCamera because its
 *              pixels originate from the current camera image rather than the
 *              world coordinate frame.
 *
 * @param[in]   trackingImage_bgr8_in
 *              Current annotated tracking image in OpenCV BGR8 format.
 *
 * @param[in]   msgTime_s_in
 *              ROS timestamp assigned to the published image message.
 */
extern void publishTrackingImage(const cv::Mat      &trackingImage_bgr8_in,
                                 const rclcpp::Time &msgTime_s_in);

/*!
 * @brief       Handles a ROS service request to save the current ORB-SLAM3 map
 *              points as a PCD file.
 *
 *              The requested output name is passed to ORB-SLAM3. The service
 *              response indicates whether the map-point cloud was saved
 *              successfully.
 *
 * @param[in]   request_in
 *              Service request containing the requested output file name.
 *
 * @param[out]  response_out
 *              Service response whose success field indicates whether the PCD
 *              file was saved successfully.
 */
extern void saveMapPointsAsPCDService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out);

/*!
 * @brief       Handles a ROS service request to save the current ORB-SLAM3 map
 *              as an ORB-SLAM atlas file.
 *
 *              The requested output name is passed to ORB-SLAM3. The service
 *              response indicates whether the map was saved successfully.
 *
 * @param[in]   request_in
 *              Service request containing the requested output file name.
 *
 * @param[out]  response_out
 *              Service response whose success field indicates whether the map
 *              was saved successfully.
 */
extern void saveMapService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out);

/*!
 * @brief       Handles a ROS service request to save the estimated camera and
 *              keyframe trajectories in EuRoC trajectory format.
 *
 *              Two trajectory files are generated using the requested base
 *              name:
 *
 *                  <name>_cam_traj.txt
 *                  <name>_kf_traj.txt
 *
 *              The service reports success only when both trajectory-saving
 *              operations complete without throwing an exception.
 *
 * @param[in]   request_in
 *              Service request containing the base name used for the output
 *              trajectory files.
 *
 * @param[out]  response_out
 *              Service response whose success field indicates whether both
 *              trajectory files were saved successfully.
 */
extern void saveTrajectoryService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out);

/*!
 * @brief       Converts a Sophus SE(3) transformation into a tf2
 *              transformation.
 *
 *              The translation and rotation components are copied into their
 *              corresponding tf2 vector and quaternion representations.
 *
 * @param[in]   transformation_SE3f_in
 *              Sophus SE(3) transformation to convert.
 *
 * @return      The equivalent tf2 transformation.
 */
extern tf2::Transform
    SE3fToTFTransform(const Sophus::SE3f &transformation_SE3f_in);

/*!
 * @brief       Initialises the ROS publishers and TF listener used by the
 *              Visual S-Graphs interface.
 *
 *              Topic names are constructed using the supplied topic namespace.
 *              Publishers required only by inertial sensor configurations are
 *              created conditionally.
 *
 * @param[in]   node_in
 *              ROS node used to create publishers and access the ROS clock.
 *
 * @param[in]   imageTransport_in
 *              Image transport interface used to advertise the tracking-image
 *              topic.
 *
 * @param[in]   topicNamespace_in
 *              Namespace or prefix applied to all advertised topic names.
 */
extern void setupPublishers(
    const std::shared_ptr<rclcpp::Node>                    &node_in,
    const std::shared_ptr<image_transport::ImageTransport> &imageTransport_in,
    const std::string                                      &topicNamespace_in);

/*!
 * @brief       Initialises the ROS services provided by the Visual S-Graphs
 *              interface.
 *
 *              The function creates services for saving the complete
 *              ORB-SLAM3 map, saving mapped points as a PCD file, and saving
 *              the estimated camera and keyframe trajectories.
 *
 *              The created service objects are retained in global shared
 *              pointers so that they remain active after this function
 *              returns.
 *
 * @param[in]   node_in
 *              ROS node used to create the service servers.
 *
 * @param[in]   serviceNamespace_in
 *              Namespace or prefix applied to all advertised service names.
 */
extern void setupServices(const std::shared_ptr<rclcpp::Node> &node_in,
                          const std::string &serviceNamespace_in);

/*!
 * @brief       Extracts connected free-space components and raw skeleton edges
 *              from a Voxblox visualisation marker array.
 *
 *              Connected vertex markers are transformed into the configured
 *              world frame and stored as free-space clusters. The raw "edges"
 *              marker is processed separately so narrow graph sections through
 *              passages are retained.
 *
 *              The transformation associated with each marker is resolved only
 *              once and reused for all points belonging to that marker.
 *
 * @param[in]   skeletonMarkerArray_in
 *              Voxblox sparse-graph marker array containing connected-component
 *              vertices and raw graph edges.
 */
extern void setVoxbloxSkeletonCluster(
    const visualization_msgs::msg::MarkerArray &skeletonMarkerArray_in);

/*!
 * @brief       Transforms a Voxblox skeleton point into the world frame using
 *              a previously resolved marker transformation.
 *
 *              This overload should be used when processing multiple points
 *              from the same marker because it avoids repeated TF lookups.
 *
 * @param[in]   T_world_skeletonMarker_in
 *              Transformation from the marker-local coordinate frame into the
 *              world frame.
 *
 * @param[in]   skeletonPoint_marker_in
 *              Skeleton point expressed in the marker-local coordinate frame.
 *
 * @param[out]  skeletonPoint_world_out
 *              Skeleton point expressed in the world frame.
 *
 * @return      True when the input and output points contain finite values;
 *              otherwise false.
 */
extern bool transformSkeletonPoint(
    const tf2::Transform            &T_world_skeletonMarker_in,
    const geometry_msgs::msg::Point &skeletonPoint_marker_in,
    Eigen::Vector3d                 &skeletonPoint_world_out);

/*!
 * @brief       Transforms a single Voxblox skeleton point into the world frame.
 *
 *              This compatibility overload resolves the marker transformation
 *              internally. The cached-transform overload should be preferred
 *              when processing multiple points from the same marker.
 *
 * @param[in]   skeletonMarker_in
 *              Voxblox marker containing the source frame and marker pose.
 *
 * @param[in]   skeletonPoint_marker_in
 *              Skeleton point expressed in the marker-local coordinate frame.
 *
 * @param[out]  skeletonPoint_world_out
 *              Skeleton point expressed in the world frame.
 *
 * @return      True when the transformation succeeds and the output contains
 *              finite values; otherwise false.
 */
extern bool transformSkeletonPoint(
    const visualization_msgs::msg::Marker &skeletonMarker_in,
    const geometry_msgs::msg::Point       &skeletonPoint_marker_in,
    Eigen::Vector3d                       &skeletonPoint_world_out);

#endif // VS_GRAPHS_COMMON_H
