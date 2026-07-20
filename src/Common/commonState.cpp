/*!
 * @File:         CommonState.cpp
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

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

/* -------------------------------------------------------------------------- *
 * ORB-SLAM3 STATE
 * -------------------------------------------------------------------------- */

ORB_SLAM3::System *pSLAM = nullptr;

ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::NOT_SET;

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
    pubWorldFramePointCloud = nullptr;

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

/* -------------------------------------------------------------------------- *
 * SERVICE SERVERS
 * -------------------------------------------------------------------------- */

rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMap = nullptr;

rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveMapPoints = nullptr;

rclcpp::Service<vs_graphs::srv::SaveMap>::SharedPtr srvSaveTrajectory = nullptr;