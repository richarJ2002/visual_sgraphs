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

#include "common.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

// Variables for ORB-SLAM3
ORB_SLAM3::System         *pSLAM;
ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
bool                                        colorPointcloud = true;
double                                      roll = 0, pitch = 0, yaw = 0;
bool                                        pubStaticTransform, pubPointClouds;
std::shared_ptr<tf2_ros::Buffer>            tfBuffer_;
std::vector<ORB_SLAM3::Room *>              gnnRoomCandidates;
std::shared_ptr<image_transport::Publisher> pubTrackingImage;
std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

std::vector<std::vector<ORB_SLAM3::Marker *>>  markersBuffer;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

std::vector<std::vector<Eigen::Vector3d>>                skeletonClusterPoints;
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;
std::string frameWorld, frameCamera, frameImu, frameMap, frameBC, frameSE;

rclcpp::Time lastPlanePublishTime(0, 0, RCL_ROS_TIME);
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     pubKeyFrameList;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDoor;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pubAllMappoints;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCameraPose;
rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr pubKFImage;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubBuildingComponents;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrackedMappoints;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreespaceCluster;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubPlaneLabel;
rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr
    pubAllWalls_new;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubSegmentedPointcloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pubWorldFramePointCloud;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubCameraPoseVis;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubKeyFrameMarker;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubFiducialMarker;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubStructuralElements;
rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr
    pubAllWalls_legacy;

void saveMapService(std::shared_ptr<vs_graphs::srv::SaveMap::Request>  req,
                    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    res->success = pSLAM->SaveMap(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                    "Map was saved as %s.osa",
                    req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Map could not be saved.");
}

void saveMapPointsAsPCDService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request>  req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    res->success = pSLAM->SaveMapPointsAsPCD(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                    "Map points were saved as %s.pcd",
                    req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Map points could not be saved.");
}

void saveTrajectoryService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request>  req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    const std::string cameraTrajectoryFile   = req->name + "_cam_traj.txt";
    const std::string keyframeTrajectoryFile = req->name + "_kf_traj.txt";

    try
    {
        pSLAM->SaveTrajectoryEuRoC(cameraTrajectoryFile);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(keyframeTrajectoryFile);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        res->success = false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
        res->success = false;
    }

    if (!res->success)
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Estimated trajectory could not be saved.");
}

void setupServices(std::shared_ptr<rclcpp::Node> node,
                   const std::string            &node_name)
{
    node->create_service<vs_graphs::srv::SaveMap>(node_name + "/save_map",
                                                  &saveMapService);
    node->create_service<vs_graphs::srv::SaveMap>(node_name +
                                                      "/save_map_points",
                                                  &saveMapPointsAsPCDService);
    node->create_service<vs_graphs::srv::SaveMap>(node_name + "/save_traj",
                                                  &saveTrajectoryService);
}

void setupPublishers(
    std::shared_ptr<rclcpp::Node>                    node,
    std::shared_ptr<image_transport::ImageTransport> image_transport,
    const std::string                               &node_name)
{
    // Basic
    pubKeyFrameList = node->create_publisher<nav_msgs::msg::Path>(
        node_name + "/keyframe_list",
        2);
    pubAllMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name + "/all_points",
        1);
    pubCameraPose = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        node_name + "/camera_pose",
        1);
    pubKFImage = node->create_publisher<segmenter_ros::msg::VSGraphDataMsg>(
        node_name + "/keyframe_image",
        50);
    pubTrackedMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name + "/tracked_points",
        1);
    pubWorldFramePointCloud =
        node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name +
                                                                  "/points_map",
                                                              1);
    pubKeyFrameMarker =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            node_name + "/kf_markers",
            1);
    pubFreespaceCluster = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name + "/freespace_clusters",
        1);
    pubCameraPoseVis =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            node_name + "/camera_pose_vis",
            1);
    pubTrackingImage = std::make_shared<image_transport::Publisher>(
        image_transport->advertise(node_name + "/tracking_image", 1));

    // Entities
    pubDoor = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        node_name + "/doors",
        1);
    pubFiducialMarker =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            node_name + "/fiducial_markers",
            1);

    // Building Components
    pubPlaneLabel =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            node_name + "/plane_labels",
            1);
    pubBuildingComponents =
        node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node_name + "/building_components",
            1);
    pubSegmentedPointcloud =
        node->create_publisher<sensor_msgs::msg::PointCloud2>(
            node_name + "/segmented_point_clouds",
            1);

    // All Walls
    pubAllWalls_new =
        node->create_publisher<vs_graphs::msg::VSGraphsAllWallsData>(
            node_name + "/all_mapped_walls",
            1);

    // Structural Elements
    pubStructuralElements =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            node_name + "/structural_elements",
            1);

    // Get body odometry if IMU data is also available
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
        pubOdometry = node->create_publisher<nav_msgs::msg::Odometry>(
            node_name + "/body_odom",
            1);

    tfBuffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void publishTopics(rclcpp::Time                                         msgTime,
                   Eigen::Vector3f                                      Wbb,
                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    // Avoid publishing NaN
    if (Twc.translation().array().isNaN()[0] ||
        Twc.rotationMatrix().array().isNaN()(0, 0))
        return;

    // Common topics
    publishCameraPose(Twc, msgTime);
    publishTFTransform(Twc, frameWorld, frameCamera, msgTime);
    publishFramePointCloud(Twc, msgPCL, msgTime);

    // Set a static transform between the world and map frame
    if (pubStaticTransform)
        publishStaticTFTransform(frameWorld, frameMap, msgTime);

    // Get KeyFrames
    std::vector<ORB_SLAM3::KeyFrame *> keyframes = pSLAM->GetAllKeyFrames();

    // Setup publishers

    publishKeyFrameImages(keyframes, msgTime);
    publishKeyFrameMarkers(keyframes, msgTime);
    publishFiducialMarkers(pSLAM->GetAllMarkers());
    publishTrackingImage(pSLAM->GetCurrentFrame(), msgTime);
    publishStructuralElements(pSLAM->GetAllRooms(),
                              pSLAM->GetAllFloors(),
                              pSLAM->GetAllPassages(),
                              msgTime);

    // Publish all mapped walls for GNN-based room detection
    publishAllMappedWalls(pSLAM->GetAllPlanes(), msgTime);

    // Publish pointclouds
    if (pubPointClouds)
    {
        publishSegmentedCloud(keyframes);
        publishPlanes(pSLAM->GetAllPlanes(), msgTime);
        publishAllPoints(pSLAM->GetAllMapPoints(), msgTime);
        publishTrackedPoints(pSLAM->GetTrackedMapPoints(), msgTime);
        publishFreeSpaceClusters(pSLAM->getSkeletonCluster(), msgTime);
    }
    else
    {
        clearKFClsClouds(keyframes);
    }

    // IMU-specific topics
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
    {
        // Body pose and translational velocity can be obtained from ORB-SLAM3
        Sophus::SE3f    Twb = pSLAM->GetImuTwb();
        Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

        // IMU provides body angular velocity in body frame (Wbb) which is
        // transformed to world frame (Wwb)
        Sophus::Matrix3f Rwb = Twb.rotationMatrix();
        Eigen::Vector3f  Wwb = Rwb * Wbb;

        publishTFTransform(Twb, frameWorld, frameImu, msgTime);
        publishBodyOdometry(Twb, Vwb, Wwb, msgTime);
    }
}

void publishBodyOdometry(Sophus::SE3f    Twb_SE3f,
                         Eigen::Vector3f Vwb_E3f,
                         Eigen::Vector3f ang_vel_body,
                         rclcpp::Time    msgTime)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.child_frame_id  = frameImu;
    odom_msg.header.frame_id = frameWorld;
    odom_msg.header.stamp    = msgTime;

    odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

    odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
    odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
    odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
    odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

    odom_msg.twist.twist.linear.x = Vwb_E3f.x();
    odom_msg.twist.twist.linear.y = Vwb_E3f.y();
    odom_msg.twist.twist.linear.z = Vwb_E3f.z();

    odom_msg.twist.twist.angular.x = ang_vel_body.x();
    odom_msg.twist.twist.angular.y = ang_vel_body.y();
    odom_msg.twist.twist.angular.z = ang_vel_body.z();

    pubOdometry->publish(odom_msg);
}

void publishCameraPose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msgTime)
{
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.frame_id = frameCamera;
    poseMsg.header.stamp    = msgTime;

    poseMsg.pose.position.x = Tcw_SE3f.translation().x();
    poseMsg.pose.position.y = Tcw_SE3f.translation().y();
    poseMsg.pose.position.z = Tcw_SE3f.translation().z();

    poseMsg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    poseMsg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    poseMsg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    poseMsg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pubCameraPose->publish(poseMsg);

    // Add a marker for visualization
    visualization_msgs::msg::Marker      cameraVisual;
    visualization_msgs::msg::MarkerArray cameraVisualList;

    cameraVisual.id                          = 1;
    cameraVisual.color.a                     = 0.7;
    cameraVisual.scale.x                     = 0.5;
    cameraVisual.scale.y                     = 0.5;
    cameraVisual.scale.z                     = 0.5;
    cameraVisual.ns                          = "camera_pose";
    cameraVisual.header.stamp                = msgTime;
    cameraVisual.action                      = cameraVisual.ADD;
    cameraVisual.header.frame_id             = frameWorld;
    cameraVisual.mesh_use_embedded_materials = true;
    cameraVisual.lifetime      = rclcpp::Duration::from_seconds(0);
    cameraVisual.type          = visualization_msgs::msg::Marker::MESH_RESOURCE;
    cameraVisual.mesh_resource = "package://vs_graphs/config/Assets/camera.dae";

    cameraVisual.pose.position.x    = Tcw_SE3f.translation().x();
    cameraVisual.pose.position.y    = Tcw_SE3f.translation().y();
    cameraVisual.pose.position.z    = Tcw_SE3f.translation().z();
    cameraVisual.pose.orientation.x = Tcw_SE3f.unit_quaternion().x();
    cameraVisual.pose.orientation.y = Tcw_SE3f.unit_quaternion().y();
    cameraVisual.pose.orientation.z = Tcw_SE3f.unit_quaternion().z();
    cameraVisual.pose.orientation.w = Tcw_SE3f.unit_quaternion().w();

    cameraVisualList.markers.push_back(cameraVisual);

    pubCameraPoseVis->publish(cameraVisualList);
}

void publishTFTransform(Sophus::SE3f T_SE3f,
                        std::string  parentFrameId,
                        std::string  childFrameId,
                        rclcpp::Time msgTime)
{
    // Variables
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp    = msgTime;
    transformStamped.child_frame_id  = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the values of transform messages
    transformStamped.transform.translation.x = T_SE3f.translation().x();
    transformStamped.transform.translation.y = T_SE3f.translation().y();
    transformStamped.transform.translation.z = T_SE3f.translation().z();

    // Fill rotation
    transformStamped.transform.rotation.x = T_SE3f.unit_quaternion().x();
    transformStamped.transform.rotation.y = T_SE3f.unit_quaternion().y();
    transformStamped.transform.rotation.z = T_SE3f.unit_quaternion().z();
    transformStamped.transform.rotation.w = T_SE3f.unit_quaternion().w();

    if (tfBroadcaster)
        tfBroadcaster->sendTransform(transformStamped);
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "TF broadcaster is not initialized.");
}

void publishStaticTFTransform(std::string  parentFrameId,
                              std::string  childFrameId,
                              rclcpp::Time msgTime)
{
    // Variables
    tf2::Quaternion                      quat;
    geometry_msgs::msg::TransformStamped transformStamped;

    // Set the values of transform messages
    transformStamped.header.stamp    = msgTime;
    transformStamped.child_frame_id  = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the translation to zero (static transform)
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    // Set the rotation using roll, pitch, yaw
    quat.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    if (staticTfBroadcaster)
        staticTfBroadcaster->sendTransform(transformStamped);
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Static TF broadcaster is not initialized.");
}

void publishFreeSpaceClusters(
    std::vector<std::vector<Eigen::Vector3d>> clusterPoints,
    rclcpp::Time                              msgTime)
{
    // Check if the cluster points are empty
    if (clusterPoints.empty())
        return;

    // Variables
    int                                    colorIndex = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr freeSpaceCloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<std::vector<uint8_t>> fixedColors = {{255, 0, 0},
                                                     {0, 255, 0},
                                                     {0, 0, 255},
                                                     {255, 255, 0},
                                                     {0, 255, 255},
                                                     {255, 0, 255},
                                                     {128, 0, 0}};

    // Loop through all the cluster points and add them to the point cloud
    for (const auto &cluster : clusterPoints)
    {
        // Variables
        std::vector<uint8_t> color = fixedColors[colorIndex];
        // Loop through all the points in the current cluster
        for (const auto &point : cluster)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x();
            newPoint.y = point.y();
            newPoint.z = point.z();
            newPoint.r = color[0];
            newPoint.g = color[1];
            newPoint.b = color[2];
            freeSpaceCloud->push_back(newPoint);
        }
        // Increment the color index
        colorIndex += 1;
        // if (colorIndex == fixedColors.size())
        if (colorIndex == static_cast<int>(fixedColors.size()))
            colorIndex = 0;
    }

    // Check if the point cloud is empty
    if (freeSpaceCloud->empty())
        return;

    // Convert the point cloud to a PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*freeSpaceCloud, cloudMsg);

    // Set message header
    cloudMsg.header.stamp    = msgTime;
    cloudMsg.header.frame_id = frameWorld;

    // Publish the point cloud
    pubFreespaceCluster->publish(cloudMsg);
}

void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec,
                           rclcpp::Time                       msgTime)
{
    // Check all keyframes and publish the ones that have not been published for
    // Semantic Segmentation yet
    for (auto &keyframe : keyframe_vec)
    {
        if (keyframe->isPublished)
            continue;

        // Create an object of VSGraphDataMsg
        segmenter_ros::msg::VSGraphDataMsg vsGraphPublisher =
            segmenter_ros::msg::VSGraphDataMsg();
        std_msgs::msg::Header header;
        header.stamp    = msgTime;
        header.frame_id = frameWorld;
        std_msgs::msg::UInt64 kfId;
        kfId.data = keyframe->mnId;
        const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
            cv_bridge::CvImage(header, "bgr8", keyframe->mImage).toImageMsg();

        vsGraphPublisher.header          = header;
        vsGraphPublisher.key_frame_id    = kfId;
        vsGraphPublisher.key_frame_image = *rendered_image_msg;

        pubKFImage->publish(vsGraphPublisher);
        keyframe->isPublished = true;
    }
}

void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *> walls,
                           rclcpp::Time                    msgTime)
{
    // Check the proper version of the GNN-based room detection
    bool isLegacy =
        ORB_SLAM3::SystemParams::GetParams()->room_seg.gnn_version == 1;

    if (isLegacy)
    {
        return;
    }

    // Variables
    vs_graphs::msg::VSGraphsAllWallsData wallDataMsg;

    // Fill the data message with wall information
    wallDataMsg.header.stamp    = msgTime;
    wallDataMsg.header.frame_id = frameWorld;

    // Fill in the walls data
    for (const auto &wall : walls)
    {
        if (!wall ||
            wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
            continue;

        // Calculate the length of the wall
        float                                   length = 0.0f;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud =
            wall->getMapClouds();
        if (wallCloud && wallCloud->points.size() > 1)
        {
            // Calculate the length of the wall by finding the distance between
            // the first and last points
            Eigen::Vector3f startPoint(wallCloud->points.front().x,
                                       wallCloud->points.front().y,
                                       wallCloud->points.front().z);
            Eigen::Vector3f endPoint(wallCloud->points.back().x,
                                     wallCloud->points.back().y,
                                     wallCloud->points.back().z);
            length = (endPoint - startPoint).norm();
        }

        // Fill the wall data
        vs_graphs::msg::VSGraphsWallData wallData;

        wallData.length     = length;
        wallData.id         = wall->getId();
        wallData.centroid.x = wall->getCentroid().x();
        wallData.centroid.y = wall->getCentroid().y();
        wallData.centroid.z = wall->getCentroid().z();
        wallData.normal.x   = wall->getGlobalEquation().normal().x();
        wallData.normal.y   = wall->getGlobalEquation().normal().y();
        wallData.normal.z   = wall->getGlobalEquation().normal().z();

        // Add the wall to the message
        wallDataMsg.walls.push_back(wallData);
    }

    // Publish all mapped walls
    pubAllWalls_new->publish(wallDataMsg);
}

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec)
{
    for (auto &keyframe : keyframe_vec)
        keyframe->clearClsClouds();
}

void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec)
{
    // get the latest processed keyframe
    ORB_SLAM3::KeyFrame *thisKF = nullptr;
    for (int i = keyframe_vec.size() - 1; i >= 0; i--)
    {
        if (keyframe_vec[i]->getClsCloudPtrs().size() > 0)
        {
            thisKF = keyframe_vec[i];
            // clear all clsClouds from the keyframes prior to the index i
            for (int j = 0; j < i; j++)
                keyframe_vec[j]->clearClsClouds();
            break;
        }
    }
    if (thisKF == nullptr)
        return;

    // get the class specific pointclouds from this keyframe
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs =
        thisKF->getClsCloudPtrs();

    // create a new pointcloud with aggregated points from all classes but with
    // class-specific colors
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aggregatedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (long unsigned int i = 0; i < clsCloudPtrs.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clsCloud = clsCloudPtrs[i];
        for (long unsigned int j = 0; j < clsCloud->points.size(); j++)
        {
            pcl::PointXYZRGBA point = clsCloud->points[j];
            switch (i)
            {
            case 0: // Ground is green
                point.r = 0;
                point.g = 255;
                point.b = 0;
                break;
            case 1: // Wall is red
                point.r = 255;
                point.g = 0;
                point.b = 0;
                break;
            }
            aggregatedCloud->push_back(point);
        }
    }
    aggregatedCloud->header = clsCloudPtrs[0]->header;
    thisKF->clearClsClouds();

    // create a new pointcloud2 message from the transformed and aggregated
    // pointcloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregatedCloud, cloud_msg);

    // publish the pointcloud to be seen at the plane frame
    cloud_msg.header.frame_id = frameCamera;
    pubSegmentedPointcloud->publish(cloud_msg);
}

void publishTrackingImage(cv::Mat image, rclcpp::Time msgTime)
{
    std_msgs::msg::Header header;
    header.stamp    = msgTime;
    header.frame_id = frameWorld;
    const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pubTrackingImage->publish(rendered_image_msg);
}

void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *> trackedMapPoints,
                          rclcpp::Time                       msgTime)
{
    sensor_msgs::msg::PointCloud2 cloud =
        mapPointToPointcloud(trackedMapPoints, msgTime);
    pubTrackedMappoints->publish(cloud);
}

void publishFramePointCloud(
    Sophus::SE3f                                         Twc,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL,
    rclcpp::Time                                         msgTime)
{
    if (!msgPCL)
        return;

    // Transform the point cloud to the world frame
    sensor_msgs::msg::PointCloud2           cloudMsg;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*msgPCL, *cloud);

    // Transform the point cloud to the world frame
    Eigen::Matrix4f Twc_eigen = Twc.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud, *transformedCloud, Twc_eigen);

    pcl::toROSMsg(*transformedCloud, cloudMsg);

    cloudMsg.header.stamp    = msgTime;
    cloudMsg.header.frame_id = frameWorld;

    pubWorldFramePointCloud->publish(cloudMsg);
}

void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> allMapPoints,
                      rclcpp::Time                       msgTime)
{
    sensor_msgs::msg::PointCloud2 cloud =
        mapPointToPointcloud(allMapPoints, msgTime);
    pubAllMappoints->publish(cloud);
}

void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec,
                            rclcpp::Time                       msgTime)
{
    sort(keyframe_vec.begin(), keyframe_vec.end(), ORB_SLAM3::KeyFrame::lId);
    if (keyframe_vec.size() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = frameWorld;
    kf_markers.ns              = "kf_markers";
    kf_markers.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action          = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime           = rclcpp::Duration::from_seconds(0);
    kf_markers.id                 = 0;
    kf_markers.scale.x            = 0.05;
    kf_markers.scale.y            = 0.05;
    kf_markers.scale.z            = 0.05;
    kf_markers.color.g            = 1.0;
    kf_markers.color.a            = 1.0;

    visualization_msgs::msg::Marker kf_lines;
    kf_lines.id              = 1;
    kf_lines.color.a         = 0.15;
    kf_lines.color.r         = 0.0;
    kf_lines.color.g         = 0.0;
    kf_lines.color.b         = 0.0;
    kf_lines.scale.x         = 0.003;
    kf_lines.scale.y         = 0.003;
    kf_lines.scale.z         = 0.003;
    kf_lines.action          = kf_lines.ADD;
    kf_lines.ns              = "kf_lines";
    kf_lines.lifetime        = rclcpp::Duration::from_seconds(0);
    kf_lines.header.stamp    = rclcpp::Clock().now();
    kf_lines.header.frame_id = frameWorld;
    kf_lines.type            = visualization_msgs::msg::Marker::LINE_LIST;

    nav_msgs::msg::Path kf_list;
    kf_list.header.frame_id = frameWorld;
    kf_list.header.stamp    = msgTime;

    for (auto &keyframe : keyframe_vec)
    {
        geometry_msgs::msg::Point kf_marker;

        Sophus::SE3f kf_pose = pSLAM->GetKeyFramePose(keyframe);
        kf_marker.x          = kf_pose.translation().x();
        kf_marker.y          = kf_pose.translation().y();
        kf_marker.z          = kf_pose.translation().z();
        kf_markers.points.push_back(kf_marker);

        // Populate the keyframe list
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id    = frameWorld;
        pose.pose.position.x    = kf_pose.translation().x();
        pose.pose.position.y    = kf_pose.translation().y();
        pose.pose.position.z    = kf_pose.translation().z();
        pose.pose.orientation.w = kf_pose.unit_quaternion().w();
        pose.pose.orientation.x = kf_pose.unit_quaternion().x();
        pose.pose.orientation.y = kf_pose.unit_quaternion().y();
        pose.pose.orientation.z = kf_pose.unit_quaternion().z();
        pose.header.stamp       = rclcpp::Time(keyframe->mTimeStamp * 1e9);
        kf_list.poses.push_back(pose);
    }

    markerArray.markers.push_back(kf_markers);
    pubKeyFrameMarker->publish(markerArray);
    pubKeyFrameList->publish(kf_list);
}

void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *> markers_in)
{
    int numMarkers = markers_in.size();
    if (numMarkers == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.resize(numMarkers);

    for (int idx = 0; idx < numMarkers; idx++)
    {
        visualization_msgs::msg::Marker fiducial_marker;
        Sophus::SE3f markerPose = markers_in[idx]->getGlobalPose();

        fiducial_marker.color.a  = 0;
        fiducial_marker.scale.x  = 0.2;
        fiducial_marker.scale.y  = 0.2;
        fiducial_marker.scale.z  = 0.2;
        fiducial_marker.ns       = "fiducial_markers";
        fiducial_marker.lifetime = rclcpp::Duration::from_seconds(0);
        fiducial_marker.action   = fiducial_marker.ADD;
        fiducial_marker.id       = markerArray.markers.size();
        fiducial_marker.header.stamp =
            rclcpp::Clock().now(); // rclcpp::Time().now();
        fiducial_marker.mesh_use_embedded_materials = true;
        fiducial_marker.header.frame_id             = frameBC;
        fiducial_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        fiducial_marker.mesh_resource =
            "package://vs_graphs/config/Assets/aruco_marker.dae";

        fiducial_marker.pose.position.x    = markerPose.translation().x();
        fiducial_marker.pose.position.y    = markerPose.translation().y();
        fiducial_marker.pose.position.z    = markerPose.translation().z();
        fiducial_marker.pose.orientation.x = markerPose.unit_quaternion().x();
        fiducial_marker.pose.orientation.y = markerPose.unit_quaternion().y();
        fiducial_marker.pose.orientation.z = markerPose.unit_quaternion().z();
        fiducial_marker.pose.orientation.w = markerPose.unit_quaternion().w();

        markerArray.markers.push_back(fiducial_marker);
    }

    pubFiducialMarker->publish(markerArray);
}

void publishPlanes(const std::vector<ORB_SLAM3::Plane *> planes,
                   const rclcpp::Time                    msgTime)
{
    const int numPlanes = static_cast<int>(planes.size());

    if (numPlanes == 0)
    {
        return;
    }

    /* Check if sufficient time has passed since the last plane publication */
    if ((msgTime - lastPlanePublishTime).seconds() < 3.0)
    {
        return;
    }

    lastPlanePublishTime = msgTime;

    visualization_msgs::msg::MarkerArray planeLabelArray;

    planeLabelArray.markers.reserve(numPlanes * 2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregatedCloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    int markerId = 0;

    for (const auto &plane : planes)
    {
        if (plane == nullptr)
        {
            continue;
        }

        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
        {
            continue;
        }

        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeClouds =
            plane->getMapClouds();

        if (planeClouds == nullptr || planeClouds->empty())
        {
            continue;
        }

        std::vector<uint8_t> color = plane->getColor();

        if (color.size() < 3)
        {
            color = {255, 255, 255};
        }

        const Eigen::Vector3f centroid = plane->getCentroid();
        const Eigen::Vector3d normal   = plane->getGlobalEquation().normal();

        /* Aggregate plane point cloud */
        for (const auto &point : planeClouds->points)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x;
            newPoint.y = point.y;
            newPoint.z = point.z;
            newPoint.r = point.r;
            newPoint.g = point.g;
            newPoint.b = point.b;

            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::DOOR)
            {
                newPoint.r = 204;
                newPoint.g = 0;
                newPoint.b = 102;
            }

            aggregatedCloud->push_back(newPoint);
        }

        /* Plane label marker */
        visualization_msgs::msg::Marker planeLabel;

        planeLabel.header.frame_id = frameBC;
        planeLabel.header.stamp    = msgTime;

        planeLabel.ns     = "plane_label";
        planeLabel.id     = markerId++;
        planeLabel.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        planeLabel.action = visualization_msgs::msg::Marker::ADD;

        planeLabel.text = "Plane#" + std::to_string(plane->getId());

        planeLabel.pose.position.x = centroid.x();
        planeLabel.pose.position.y = centroid.y() - 1.5;
        planeLabel.pose.position.z = centroid.z();

        planeLabel.pose.orientation.x = 0.0;
        planeLabel.pose.orientation.y = 0.0;
        planeLabel.pose.orientation.z = 0.0;
        planeLabel.pose.orientation.w = 1.0;

        planeLabel.scale.z = 0.2;

        planeLabel.color.a = 1.0;
        planeLabel.color.r = static_cast<float>(color[0]) / 255.0f;
        planeLabel.color.g = static_cast<float>(color[1]) / 255.0f;
        planeLabel.color.b = static_cast<float>(color[2]) / 255.0f;

        planeLabel.lifetime = rclcpp::Duration::from_seconds(0);

        planeLabelArray.markers.push_back(planeLabel);

        /* Plane normal marker */
        visualization_msgs::msg::Marker planeNormal;

        planeNormal.header.frame_id = frameBC;
        planeNormal.header.stamp    = msgTime;

        planeNormal.ns     = "plane_normal";
        planeNormal.id     = markerId++;
        planeNormal.type   = visualization_msgs::msg::Marker::ARROW;
        planeNormal.action = visualization_msgs::msg::Marker::ADD;

        planeNormal.scale.x = 0.01; // Shaft diameter
        planeNormal.scale.y = 0.05; // Arrowhead diameter
        planeNormal.scale.z = 0.05; // Arrowhead length

        planeNormal.color.a = 1.0;
        planeNormal.color.r = static_cast<float>(color[0]) / 255.0f;
        planeNormal.color.g = static_cast<float>(color[1]) / 255.0f;
        planeNormal.color.b = static_cast<float>(color[2]) / 255.0f;

        geometry_msgs::msg::Point normalStartPoint;
        normalStartPoint.x = centroid.x();
        normalStartPoint.y = centroid.y();
        normalStartPoint.z = centroid.z();

        geometry_msgs::msg::Point normalEndPoint;
        normalEndPoint.x = normalStartPoint.x + normal.x() * 0.2;
        normalEndPoint.y = normalStartPoint.y + normal.y() * 0.2;
        normalEndPoint.z = normalStartPoint.z + normal.z() * 0.2;

        planeNormal.points.push_back(normalStartPoint);
        planeNormal.points.push_back(normalEndPoint);

        planeNormal.lifetime = rclcpp::Duration::from_seconds(0);

        planeLabelArray.markers.push_back(planeNormal);
    }

    if (aggregatedCloud->empty())
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*aggregatedCloud, cloudMsg);

    cloudMsg.header.stamp    = msgTime;
    cloudMsg.header.frame_id = frameBC;

    pubBuildingComponents->publish(cloudMsg);
    pubPlaneLabel->publish(planeLabelArray);
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

    passagePointWorldPhysical.header.stamp = msgTime_in;

    passagePointWorldPhysical.header.frame_id = frameWorld;

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

        /*
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

        passagePointSE_out.header.stamp = msgTime_in;

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

void publishStructuralElements(
    const std::vector<ORB_SLAM3::Room *>    rooms_in,
    const std::vector<ORB_SLAM3::Floor *>   floors_in,
    const std::vector<ORB_SLAM3::Passage *> passages_in,
    const rclcpp::Time                      msgTime_in)
{
    /* Extract the number of rooms and number of floors */
    const int numRooms    = static_cast<int>(rooms_in.size());
    const int numFloors   = static_cast<int>(floors_in.size());
    const int numPassages = static_cast<int>(passages_in.size());

    /* If there are no rooms, floors, or passages then publish nothing */
    if (numRooms <= 0 && numFloors <= 0 && numPassages <= 0)
    {
        return;
    }

    /* Variables */
    const double textOffset = -0.5;

    /* Visulization markers */
    visualization_msgs::msg::MarkerArray roomArray;
    visualization_msgs::msg::MarkerArray floorArray;
    visualization_msgs::msg::MarkerArray passageArray;
    roomArray.markers.reserve(numRooms);
    floorArray.markers.reserve(numFloors);
    passageArray.markers.reserve(static_cast<std::size_t>(numPassages) * 3);

    /* ---------------------------------------------------------------------- *
     * PUBLISH ROOMS
     * ---------------------------------------------------------------------- */

    for (int idx = 0; idx < numRooms; idx++)
    {
        /* Extract the current room */
        ORB_SLAM3::Room *roomCandidate = rooms_in[idx];

        /* Determine whether the structural element is a confirmed room */
        const bool isConfirmedRoom =
            roomCandidate != nullptr &&
            (roomCandidate->getRoomVariant() ==
                 ORB_SLAM3::Room::roomVariant::ROOM ||
             roomCandidate->getRoomVariant() ==
                 ORB_SLAM3::Room::roomVariant::CORRIDOR);

        /*!
         * Delete invalid and provisional structural elements from the published
         * structural map.
         *
         * @note        Provisional SEs remain inside the semantic map but are
         *              not exposed as final structural elements.
         */
        if (roomCandidate == nullptr || roomCandidate->isBad() ||
            !isConfirmedRoom)
        {
            /* Variables */
            visualization_msgs::msg::Marker delRoom;
            visualization_msgs::msg::Marker delRoomLabel;
            visualization_msgs::msg::Marker delRoomWallLine;
            visualization_msgs::msg::Marker delRoomPassageLine;

            /* Delete previous marker for this room */
            delRoom.id              = idx;
            delRoom.ns              = "room";
            delRoom.header.stamp    = msgTime_in;
            delRoom.header.frame_id = frameSE;
            delRoom.action          = visualization_msgs::msg::Marker::DELETE;

            /* Delete previous marker for this room label */
            delRoomLabel.id              = idx;
            delRoomLabel.ns              = "roomLabel";
            delRoomLabel.header.stamp    = msgTime_in;
            delRoomLabel.header.frame_id = frameSE;
            delRoomLabel.action = visualization_msgs::msg::Marker::DELETE;

            /* Delete previous room-wall lines */
            delRoomWallLine.id              = idx;
            delRoomWallLine.ns              = "roomWallLine";
            delRoomWallLine.header.stamp    = msgTime_in;
            delRoomWallLine.header.frame_id = frameWorld;
            delRoomWallLine.action = visualization_msgs::msg::Marker::DELETE;

            /* Delete previous room-passage lines */
            delRoomPassageLine.id              = idx;
            delRoomPassageLine.ns              = "roomPassageLine";
            delRoomPassageLine.header.stamp    = msgTime_in;
            delRoomPassageLine.header.frame_id = frameWorld;
            delRoomPassageLine.action = visualization_msgs::msg::Marker::DELETE;

            /* Push the delete markers and skip them */
            roomArray.markers.push_back(delRoom);
            roomArray.markers.push_back(delRoomLabel);
            roomArray.markers.push_back(delRoomWallLine);
            roomArray.markers.push_back(delRoomPassageLine);

            /* Move onto next room */
            continue;
        }

        /* Variables for specific room */
        const std::string roomName = roomCandidate->getName();

        /* Set defulat colour */
        std::vector<double> roomColour = {0.5, 0.5, 0.5};

        /*!
         * Create color based on room type
         *      undefined:  gray
         *      corridor:   dark pink
         *      room:       purple
         */
        if (roomCandidate->getRoomVariant() ==
            ORB_SLAM3::Room::roomVariant::CORRIDOR)
        {
            roomColour = {0.6, 0.0, 0.3};
        }
        else if (roomCandidate->getRoomVariant() ==
                 ORB_SLAM3::Room::roomVariant::ROOM)
        {
            roomColour = {0.5, 0.1, 1.0};
        }

        /* Init variables used for finding the room in display frame */
        geometry_msgs::msg::PointStamped roomPointSE;
        geometry_msgs::msg::PointStamped roomPointWorld;

        /* Calculate the displayed room position */
        if (!getRoomDisplayPoints(roomCandidate,
                                  msgTime_in,
                                  roomPointSE,
                                  roomPointWorld))
        {
            continue;
        }

        visualization_msgs::msg::Marker room;
        visualization_msgs::msg::Marker roomLabel;
        visualization_msgs::msg::Marker roomWallLine;
        visualization_msgs::msg::Marker roomDoorwayLine;
        visualization_msgs::msg::Marker roomMarkerLine;

        /* Room values */
        room.id = idx;
        room.ns = "room";

        room.action = room.ADD;
        room.type   = visualization_msgs::msg::Marker::CUBE;

        room.header.stamp    = msgTime_in;
        room.header.frame_id = frameSE;

        room.scale.x = 0.3;
        room.scale.y = 0.3;
        room.scale.z = 0.3;

        room.color.a = 1.0;
        room.color.r = roomColour[0];
        room.color.g = roomColour[1];
        room.color.b = roomColour[2];

        room.pose.position.x = roomPointSE.point.x;
        room.pose.position.y = roomPointSE.point.y;
        room.pose.position.z = roomPointSE.point.z;

        room.pose.orientation.x = 0.0;
        room.pose.orientation.y = 0.0;
        room.pose.orientation.z = 0.0;
        room.pose.orientation.w = 1.0;

        room.mesh_use_embedded_materials = true;
        room.lifetime                    = rclcpp::Duration::from_seconds(0);
        roomArray.markers.push_back(room);

        /* Room label (name) */
        roomLabel.id   = idx;
        roomLabel.text = roomName;
        roomLabel.ns   = "roomLabel";

        roomLabel.action = roomLabel.ADD;
        roomLabel.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        roomLabel.header.stamp    = msgTime_in;
        roomLabel.header.frame_id = frameSE;

        roomLabel.scale.z = 0.2;

        roomLabel.pose.position.x = roomPointSE.point.x;
        roomLabel.pose.position.z = roomPointSE.point.y;
        roomLabel.pose.position.y = roomPointSE.point.z + textOffset;

        roomLabel.color.a = 1;
        roomLabel.color.r = 0;
        roomLabel.color.g = 0;
        roomLabel.color.b = 0;

        roomLabel.lifetime = rclcpp::Duration::from_seconds(0);
        roomArray.markers.push_back(roomLabel);

        /* Room to Wall connection line */
        roomWallLine.id = idx;
        roomWallLine.ns = "roomWallLine";

        roomWallLine.action = roomWallLine.ADD;

        roomWallLine.header.stamp    = msgTime_in;
        roomWallLine.header.frame_id = frameWorld;

        roomWallLine.scale.x = 0.05;
        roomWallLine.scale.y = 0.05;
        roomWallLine.scale.z = 0.05;

        roomWallLine.color.a = 0.9;
        roomWallLine.color.r = roomColour[0];
        roomWallLine.color.g = roomColour[1];
        roomWallLine.color.b = roomColour[2];

        roomWallLine.lifetime = rclcpp::Duration::from_seconds(0);
        roomWallLine.type     = visualization_msgs::msg::Marker::LINE_LIST;

        /* Room to Passage connection line */
        roomDoorwayLine.id = idx;
        roomDoorwayLine.ns = "roomPassageLine";

        roomDoorwayLine.header.stamp    = msgTime_in;
        roomDoorwayLine.header.frame_id = frameWorld;

        roomDoorwayLine.action = visualization_msgs::msg::Marker::ADD;
        roomDoorwayLine.type   = visualization_msgs::msg::Marker::LINE_LIST;

        roomDoorwayLine.scale.x = 0.05;
        roomDoorwayLine.scale.y = 0.05;
        roomDoorwayLine.scale.z = 0.05;

        roomDoorwayLine.pose.orientation.x = 0.0;
        roomDoorwayLine.pose.orientation.y = 0.0;
        roomDoorwayLine.pose.orientation.z = 0.0;
        roomDoorwayLine.pose.orientation.w = 1.0;

        roomDoorwayLine.lifetime = rclcpp::Duration::from_seconds(0);

        /* Room to passage connection line */
        for (ORB_SLAM3::Passage *passage : roomCandidate->getPassages())
        {
            /* Skip if passage is bad */
            if (passage == nullptr)
            {
                continue;
            }

            /* Calculate the displayed passage-node position */
            geometry_msgs::msg::PointStamped passagePointSE;
            geometry_msgs::msg::PointStamped passagePointWorld;

            if (!getPassageDisplayPoints(passage,
                                         msgTime_in,
                                         0.0,
                                         passagePointSE,
                                         passagePointWorld))
            {
                continue;
            }

            /* Check if the pasasge is passable */
            const bool isOpen = passage->isPassable();

            /* Init variable for tracking is a passage is open or closed */
            std::vector<double> passageColour;

            /* Set the colour of the marker based on if the passage is open */
            if (isOpen)
            {
                /* Set deault colour to green */
                passageColour = {0.0, 1.0, 0.7};
            }
            else
            {
                /* Set deault colour to red */
                passageColour = {0.1, 0.0, 0.0};
            }

            roomDoorwayLine.color.a = 0.9;
            roomDoorwayLine.color.r = passageColour[0];
            roomDoorwayLine.color.g = passageColour[1];
            roomDoorwayLine.color.b = passageColour[2];

            /* Init variable for room end */
            geometry_msgs::msg::Point roomEnd;

            /* Use the world-frame position corresponding to displayed room */
            roomEnd.x = roomPointWorld.point.x;
            roomEnd.y = roomPointWorld.point.y;
            roomEnd.z = roomPointWorld.point.z;

            geometry_msgs::msg::Point passageEnd;
            passageEnd.x = passagePointWorld.point.x;
            passageEnd.y = passagePointWorld.point.y;
            passageEnd.z = passagePointWorld.point.z;

            roomDoorwayLine.points.push_back(roomEnd);
            roomDoorwayLine.points.push_back(passageEnd);
        }

        /* Room to Wall connection line */
        for (const auto wall : roomCandidate->getWalls())
        {
            /* Skip if the wall is bad */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Variables */
            geometry_msgs::msg::Point        pointRoom;
            geometry_msgs::msg::Point        pointWall;
            geometry_msgs::msg::PointStamped wallPoint;
            geometry_msgs::msg::PointStamped wallPointTr;

            pointRoom.x = roomPointWorld.point.x;
            pointRoom.y = roomPointWorld.point.y;
            pointRoom.z = roomPointWorld.point.z;
            roomWallLine.points.push_back(pointRoom);

            wallPoint.header.stamp    = msgTime_in;
            wallPoint.header.frame_id = frameBC;
            wallPoint.point.x         = wall->getCentroid().x();
            wallPoint.point.y         = wall->getCentroid().y();
            wallPoint.point.z         = wall->getCentroid().z();

            try
            {
                /* Extract the transform from room centre to world frame */
                auto tfStamped = tfBuffer_->lookupTransform(
                    frameWorld,
                    frameBC,
                    msgTime_in,
                    rclcpp::Duration::from_seconds(0.1));

                /* Transform the room center point to the world frame */
                tf2::doTransform(wallPoint, wallPointTr, tfStamped);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                            "Wall centroid transform failed: %s",
                            ex.what());
                wallPointTr = wallPoint;
            }

            pointWall.x = wallPointTr.point.x;
            pointWall.y = wallPointTr.point.y;
            pointWall.z = wallPointTr.point.z;
            roomWallLine.points.push_back(pointWall);
        }

        /* Add items to the roomArray */
        roomArray.markers.push_back(roomWallLine);

        /* Add doorway items to the roomArray */
        if (!roomDoorwayLine.points.empty())
        {
            roomArray.markers.push_back(roomDoorwayLine);
        }
    }

    pubStructuralElements->publish(roomArray);

    /* ---------------------------------------------------------------------- *
     * PUBLISH FLOORS
     * ---------------------------------------------------------------------- */

    /* Set deault colour */
    const std::vector<double> floorColour = {0.3, 0.6, 0.7};

    /*!
     * Small world-frame display offset so the floor and room cubes do not
     * completely overlap.
     *
     * @note        Set this to 0.0 to display the floor at its exact stored
     *              centroid.
     */
    constexpr double floorDisplayOffset = -1.0;

    /* Loop through all floors */
    for (int floorId = 0; floorId < numFloors; floorId++)
    {
        /* Extract the current floor */
        ORB_SLAM3::Floor *floor = floors_in[floorId];

        /* Skip invalid or empty floors */
        if (floor == nullptr || floor->getRooms().empty())
        {
            continue;
        }

        const std::string floorName = floor->getName();

        const Eigen::Vector3d floorCentroid = floor->getCentroid();

        if (!floorCentroid.allFinite())
        {
            continue;
        }

        /*!
         * The floor centroid is already stored in the world frame.
         *
         * Apply only a small display offset so the floor marker remains visible
         * when a single floor and room have the same centroid.
         */
        Eigen::Vector3d floorDisplayPosition = floorCentroid;

        if (sensorType == ORB_SLAM3::System::IMU_RGBD)
        {
            floorDisplayPosition.z() += floorDisplayOffset;
        }
        else
        {
            floorDisplayPosition.y() += floorDisplayOffset;
        }

        /* ------------------------------------------------------------------ *
         * FLOOR MARKER
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker floorMarker;
        floorMarker.id = floorId;
        floorMarker.ns = "floors";

        floorMarker.header.stamp    = msgTime_in;
        floorMarker.header.frame_id = frameWorld;

        floorMarker.action = visualization_msgs::msg::Marker::ADD;
        floorMarker.type   = visualization_msgs::msg::Marker::CUBE;

        floorMarker.pose.position.x = floorDisplayPosition.x();
        floorMarker.pose.position.y = floorDisplayPosition.y();
        floorMarker.pose.position.z = floorDisplayPosition.z();

        floorMarker.pose.orientation.x = 0.0;
        floorMarker.pose.orientation.y = 0.0;
        floorMarker.pose.orientation.z = 0.0;
        floorMarker.pose.orientation.w = 1.0;

        floorMarker.scale.x = 0.4;
        floorMarker.scale.y = 0.4;
        floorMarker.scale.z = 0.4;

        floorMarker.color.a = 1.0;
        floorMarker.color.r = floorColour[0];
        floorMarker.color.g = floorColour[1];
        floorMarker.color.b = floorColour[2];

        floorMarker.lifetime = rclcpp::Duration::from_seconds(0);

        /* ------------------------------------------------------------------ *
         * FLOOR LABEL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker floorLabel;

        floorLabel.id = floorId;
        floorLabel.ns = "floorLabels";

        floorLabel.header.stamp    = msgTime_in;
        floorLabel.header.frame_id = frameWorld;

        floorLabel.action = visualization_msgs::msg::Marker::ADD;
        floorLabel.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        floorLabel.text   = floorName;

        floorLabel.pose.position.x = floorDisplayPosition.x();
        floorLabel.pose.position.y = floorDisplayPosition.y();
        floorLabel.pose.position.z = floorDisplayPosition.z();

        /*!
         * Move the text away from the floor cube using the same display-axis
         * convention as the other structural elements.
         */
        if (sensorType == ORB_SLAM3::System::IMU_RGBD)
        {
            floorLabel.pose.position.z += textOffset;
        }
        else
        {
            floorLabel.pose.position.y += textOffset;
        }

        floorLabel.pose.orientation.x = 0.0;
        floorLabel.pose.orientation.y = 0.0;
        floorLabel.pose.orientation.z = 0.0;
        floorLabel.pose.orientation.w = 1.0;

        floorLabel.scale.z = 0.2;

        floorLabel.color.a = 1.0;
        floorLabel.color.r = 0.0;
        floorLabel.color.g = 0.0;
        floorLabel.color.b = 0.0;

        floorLabel.lifetime = rclcpp::Duration::from_seconds(0);

        /* ------------------------------------------------------------------ *
         * FLOOR-TO-ROOM CONNECTION LINES
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker floorRoomLine;

        floorRoomLine.id = floorId;
        floorRoomLine.ns = "floorRoomEdges";

        floorRoomLine.header.stamp    = msgTime_in;
        floorRoomLine.header.frame_id = frameWorld;

        floorRoomLine.action = visualization_msgs::msg::Marker::ADD;
        floorRoomLine.type   = visualization_msgs::msg::Marker::LINE_LIST;

        floorRoomLine.pose.orientation.x = 0.0;
        floorRoomLine.pose.orientation.y = 0.0;
        floorRoomLine.pose.orientation.z = 0.0;
        floorRoomLine.pose.orientation.w = 1.0;

        floorRoomLine.scale.x = 0.05;
        floorRoomLine.scale.y = 0.05;
        floorRoomLine.scale.z = 0.05;

        floorRoomLine.color.a = 0.9;
        floorRoomLine.color.r = floorColour[0];
        floorRoomLine.color.g = floorColour[1];
        floorRoomLine.color.b = floorColour[2];

        floorRoomLine.lifetime = rclcpp::Duration::from_seconds(0);

        /* Connect the floor to every confirmed room */
        for (ORB_SLAM3::Room *room : floor->getRooms())
        {
            /* Skip invalid and provisional structural elements */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            const bool isConfirmedRoom =
                room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::ROOM ||
                room->getRoomVariant() ==
                    ORB_SLAM3::Room::roomVariant::CORRIDOR;

            if (!isConfirmedRoom)
            {
                continue;
            }

            /*!
             * Get the world-frame position corresponding to the displayed room
             * marker.
             */
            geometry_msgs::msg::PointStamped roomPointSE;
            geometry_msgs::msg::PointStamped roomPointWorld;

            if (!getRoomDisplayPoints(room,
                                      msgTime_in,
                                      roomPointSE,
                                      roomPointWorld))
            {
                continue;
            }

            geometry_msgs::msg::Point floorEnd;
            floorEnd.x = floorDisplayPosition.x();
            floorEnd.y = floorDisplayPosition.y();
            floorEnd.z = floorDisplayPosition.z();

            geometry_msgs::msg::Point roomEnd;
            roomEnd.x = roomPointWorld.point.x;
            roomEnd.y = roomPointWorld.point.y;
            roomEnd.z = roomPointWorld.point.z;

            floorRoomLine.points.push_back(floorEnd);
            floorRoomLine.points.push_back(roomEnd);
        }

        /* Add the floor marker and label */
        floorArray.markers.push_back(floorMarker);
        floorArray.markers.push_back(floorLabel);

        /* Add connection lines only when at least one pair exists */
        if (!floorRoomLine.points.empty())
        {
            floorArray.markers.push_back(floorRoomLine);
        }
    }

    pubStructuralElements->publish(floorArray);

    /* ---------------------------------------------------------------------- *
     * PUBLISH PASSAGES
     * ---------------------------------------------------------------------- */

    /* Loop through all the passages */
    for (ORB_SLAM3::Passage *passage : passages_in)
    {
        /* If the passage is invalid, skip */
        if (passage == nullptr)
        {
            continue;
        }

        /* Extract the passage id */
        const int passageId = passage->getId();

        /* Calculate the displayed structural-graph position */
        geometry_msgs::msg::PointStamped passagePointSE;
        geometry_msgs::msg::PointStamped passagePointWorld;

        if (!getPassageDisplayPoints(passage,
                                     msgTime_in,
                                     0.0,
                                     passagePointSE,
                                     passagePointWorld))
        {
            continue;
        }

        /* Check if the pasasge is passable */
        const bool isOpen = passage->isPassable();

        /* Init variable for tracking is a passage is open or closed */
        std::vector<double> passageColour;

        /* Set the colour of the marker based on if the passage is open */
        if (isOpen)
        {
            /* Set deault colour to green */
            passageColour = {0.0, 1.0, 0.7};
        }
        else
        {
            /* Set deault colour to red */
            passageColour = {0.1, 0.0, 0.0};
        }

        /* Create a marker msg variable for the passage */
        visualization_msgs::msg::Marker passageMarker;

        /* Fill the msg */
        passageMarker.header.frame_id = frameSE;
        passageMarker.header.stamp    = msgTime_in;

        passageMarker.ns     = "passage";
        passageMarker.id     = passageId;
        passageMarker.type   = visualization_msgs::msg::Marker::CUBE;
        passageMarker.action = visualization_msgs::msg::Marker::ADD;

        passageMarker.pose.position.x = passagePointSE.point.x;
        passageMarker.pose.position.y = passagePointSE.point.y;
        passageMarker.pose.position.z = passagePointSE.point.z;

        passageMarker.pose.orientation.x = 0.0;
        passageMarker.pose.orientation.y = 0.0;
        passageMarker.pose.orientation.z = 0.0;
        passageMarker.pose.orientation.w = 1.0;

        passageMarker.scale.x = 0.30;
        passageMarker.scale.y = 0.30;
        passageMarker.scale.z = 0.30;

        passageMarker.color.a = 1.0;
        passageMarker.color.r = passageColour[0];
        passageMarker.color.g = passageColour[1];
        passageMarker.color.b = passageColour[2];

        /* Set transparecy to full */
        passageMarker.color.a = 1.0;

        /* Set the lifetime of the market */
        passageMarker.lifetime = rclcpp::Duration::from_seconds(0);

        /* Add marker to array */
        passageArray.markers.push_back(passageMarker);

        /* Init label to indicate the passage is passable */
        visualization_msgs::msg::Marker passageLabel;

        /* Fill label */
        passageLabel.header.frame_id = frameSE;
        passageLabel.header.stamp    = msgTime_in;

        passageLabel.ns     = "passageLabel";
        passageLabel.id     = passageId;
        passageLabel.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        passageLabel.action = visualization_msgs::msg::Marker::ADD;

        passageLabel.text = "Passage#" + std::to_string(passageId) +
                            (isOpen ? " [open]" : " [blocked]");

        passageLabel.pose.position.x = passagePointSE.point.x;
        passageLabel.pose.position.y = passagePointSE.point.y + textOffset;
        passageLabel.pose.position.z = passagePointSE.point.z;

        passageLabel.pose.orientation.x = 0.0;
        passageLabel.pose.orientation.y = 0.0;
        passageLabel.pose.orientation.z = 0.0;
        passageLabel.pose.orientation.w = 1.0;

        passageLabel.scale.z = 0.20;

        passageLabel.color.a = 0.9;
        passageLabel.color.r = passageColour[0];
        passageLabel.color.g = passageColour[1];
        passageLabel.color.b = passageColour[2];

        passageLabel.lifetime = rclcpp::Duration::from_seconds(0);

        passageArray.markers.push_back(passageLabel);
    }

    pubStructuralElements->publish(passageArray);
}

sensor_msgs::msg::PointCloud2
    mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints,
                         rclcpp::Time                       msgTime)
{
    const int                     numChannels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    std::string                   channelId[] = {"x", "y", "z"};

    // Set the attributes of the point cloud
    cloud.header.stamp    = msgTime;
    cloud.header.frame_id = frameWorld;
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.is_bigendian    = false;
    cloud.width           = mapPoints.size();
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
        if (mapPoints[idx] && !mapPoints[idx]->isBad())
        {
            Eigen::Vector3d P3Dw = mapPoints[idx]->GetWorldPos().cast<double>();
            tf2::Vector3    pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());
            float           dataArray[numChannels] = {
                static_cast<float>(pointTranslation.x()),
                static_cast<float>(pointTranslation.y()),
                static_cast<float>(pointTranslation.z())};
            memcpy(cloudDataPtr + (idx * cloud.point_step),
                   dataArray,
                   numChannels * sizeof(float));
        }
    }

    return cloud;
}

cv::Mat SE3fToCvMat(Sophus::SE3f data)
{
    cv::Mat cvMat;

    // Convert the Eigen matrix to OpenCV matrix
    Eigen::Matrix4f T_Eig3f = data.matrix();
    cv::eigen2cv(T_Eig3f, cvMat);

    return cvMat;
}

// tf::Transform SE3fToTFTransform(Sophus::SE3f data)
tf2::Transform SE3fToTFTransform(Sophus::SE3f data)
{
    Eigen::Matrix3f rotMatrix   = data.rotationMatrix();
    Eigen::Vector3f transVector = data.translation();

    tf2::Matrix3x3 rotationTF(rotMatrix(0, 0),
                              rotMatrix(0, 1),
                              rotMatrix(0, 2),
                              rotMatrix(1, 0),
                              rotMatrix(1, 1),
                              rotMatrix(1, 2),
                              rotMatrix(2, 0),
                              rotMatrix(2, 1),
                              rotMatrix(2, 2));

    tf2::Vector3 translationTF(transVector(0), transVector(1), transVector(2));

    return tf2::Transform(rotationTF, translationTF);
}

std::pair<double, std::vector<ORB_SLAM3::Marker *>>
    findNearestMarker(double frameTimestamp)
{
    double                           minTimeDifference = 100;
    std::vector<ORB_SLAM3::Marker *> matchedMarkers;

    // Loop through the markersBuffer
    for (const auto &markers : markersBuffer)
    {
        double timeDifference = markers[0]->getTime() - frameTimestamp;
        if (timeDifference < minTimeDifference)
        {
            matchedMarkers    = markers;
            minTimeDifference = timeDifference;
        }
    }

    return std::make_pair(minTimeDifference, matchedMarkers);
}

bool transformSkeletonPoint(const visualization_msgs::msg::Marker &marker_in,
                            const geometry_msgs::msg::Point       &point_in,
                            Eigen::Vector3d &transformedPoint_out)
{
    /*!
     * Use the frame supplied by the Voxblox marker.
     *
     * @note        Voxblox currently publishes the sparse graph in
     *              "map_elevated". The configured frameMap must only be used
     *              when the marker contains no frame identifier.
     */
    const std::string sourceFrame = marker_in.header.frame_id.empty()
                                        ? frameMap
                                        : marker_in.header.frame_id;

    /* Reject the point if no valid source frame is available */
    if (sourceFrame.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Voxblox marker does not contain a valid source frame.");

        return false;
    }

    /* Confirm that the TF buffer is available */
    if (tfBuffer_ == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "TF buffer is not available for skeleton transformation.");

        return false;
    }

    /* No transform is required if the point is already in the world frame */
    if (sourceFrame == frameWorld)
    {
        transformedPoint_out = Eigen::Vector3d(static_cast<double>(point_in.x),
                                               static_cast<double>(point_in.y),
                                               static_cast<double>(point_in.z));

        return transformedPoint_out.allFinite();
    }

    /* Create the source and destination ROS point messages */
    geometry_msgs::msg::PointStamped pointIn;
    geometry_msgs::msg::PointStamped pointOut;

    pointIn.header.frame_id = sourceFrame;
    pointIn.header.stamp    = rclcpp::Time(0);
    pointIn.point           = point_in;

    try
    {
        /* Find the latest transform from the marker frame to the world frame */
        const geometry_msgs::msg::TransformStamped transformStamped =
            tfBuffer_->lookupTransform(frameWorld,
                                       sourceFrame,
                                       tf2::TimePointZero,
                                       tf2::durationFromSec(0.1));

        /* Transform the skeleton point into the world frame */
        tf2::doTransform(pointIn, pointOut, transformStamped);
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Could not transform Voxblox point from '%s' to '%s': %s",
                    sourceFrame.c_str(),
                    frameWorld.c_str(),
                    exception.what());

        /*!
         * Do not treat an untransformed point as a world-frame point.
         *
         * @note        Doing so would create incorrect room centroids and
         *              passage-wall intersections.
         */
        return false;
    }

    /* Convert the transformed ROS point into an Eigen vector */
    transformedPoint_out =
        Eigen::Vector3d(static_cast<double>(pointOut.point.x),
                        static_cast<double>(pointOut.point.y),
                        static_cast<double>(pointOut.point.z));

    return transformedPoint_out.allFinite();
}

void setVoxbloxSkeletonCluster(
    const visualization_msgs::msg::MarkerArray &skeletonArray)
{
    /* Reset the connected vertex and edge buffers */
    skeletonClusterPoints.clear();
    skeletonEdges.clear();

    /* Extract the minimum valid connected-component size */
    const std::size_t minimumClusterVertices = static_cast<std::size_t>(
        ORB_SLAM3::SystemParams::GetParams()->room_seg.min_cluster_vertices);

    /* Process every marker contained in the sparse graph message */
    for (const visualization_msgs::msg::Marker &skeleton :
         skeletonArray.markers)
    {
        /*!
         * Determine whether the marker contains vertices belonging to one
         * connected Voxblox component.
         *
         * @note        The generic "vertices" and "closed_spaces" markers are
         *              deliberately ignored because they duplicate the
         *              connected-component data.
         */
        const bool isConnectedVertexMarker =
            skeleton.type == visualization_msgs::msg::Marker::CUBE_LIST &&
            skeleton.ns.rfind("connected_vertices_", 0) == 0;

        if (isConnectedVertexMarker)
        {
            /* Ignore empty and undersized connected components */
            if (skeleton.points.size() < minimumClusterVertices)
            {
                continue;
            }

            /* Initialise the current connected-component point collection */
            std::vector<Eigen::Vector3d> clusterPoints;

            clusterPoints.reserve(skeleton.points.size());

            /* Transform every connected vertex into the world frame */
            for (const geometry_msgs::msg::Point &point : skeleton.points)
            {
                Eigen::Vector3d transformedPoint;

                /* Skip points which cannot be transformed */
                if (!transformSkeletonPoint(skeleton, point, transformedPoint))
                {
                    continue;
                }

                clusterPoints.push_back(transformedPoint);
            }

            /* Store the component when enough valid points remain */
            if (clusterPoints.size() >= minimumClusterVertices)
            {
                skeletonClusterPoints.push_back(std::move(clusterPoints));
            }

            continue;
        }

        /*!
         * Extract the complete raw sparse graph.
         *
         * @note        The connected_edges_* markers are clearance-filtered for
         *              room segmentation. The raw "edges" marker preserves
         *              narrow graph sections through passages.
         */
        const bool isRawEdgeMarker =
            skeleton.type == visualization_msgs::msg::Marker::LINE_LIST &&
            skeleton.ns == "edges";

        if (!isRawEdgeMarker)
        {
            continue;
        }

        /* Process every consecutive pair as one connected skeleton edge */
        for (std::size_t pointIndex = 0;
             pointIndex + 1 < skeleton.points.size();
             pointIndex += 2)
        {
            Eigen::Vector3d edgeStart;
            Eigen::Vector3d edgeEnd;

            /* Transform the beginning of the edge */
            const bool validStart =
                transformSkeletonPoint(skeleton,
                                       skeleton.points[pointIndex],
                                       edgeStart);

            /* Transform the end of the edge */
            const bool validEnd =
                transformSkeletonPoint(skeleton,
                                       skeleton.points[pointIndex + 1],
                                       edgeEnd);

            /* Skip the edge if either endpoint could not be transformed */
            if (!validStart || !validEnd)
            {
                continue;
            }

            /* Skip invalid and zero-length edges */
            if (!edgeStart.allFinite() || !edgeEnd.allFinite() ||
                (edgeEnd - edgeStart).norm() < 1e-6)
            {
                continue;
            }

            /* Store the connected skeleton edge */
            skeletonEdges.emplace_back(edgeStart, edgeEnd);
        }
    }

    /* Store the connected skeleton vertices in the active map */
    pSLAM->setSkeletonCluster(skeletonClusterPoints);

    /* Store the connected skeleton edges in the active map */
    pSLAM->setSkeletonEdges(skeletonEdges);

    std::cout << "[Voxblox] Stored " << skeletonClusterPoints.size()
              << " connected components and " << skeletonEdges.size()
              << " connected edges." << std::endl;
}

void setGNNBasedRoomCandidates(
    const situational_graphs_msgs::msg::RoomsData &msgGNNRooms)
{
    // Reset the buffer
    gnnRoomCandidates.clear();

    // Loop through the received GNN rooms
    // for (const auto &room : msgGNNRooms.rooms)
    // {
    //     // Create a new room object
    //     ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

    //     // Set the room properties
    //     newRoom->setId(room.id);
    //     newRoom->setHasKnownLabel(false);

    //     // [TODO] use room.wallIds to fill newRoom->setWalls
    //     // [TODO] use room.centroid to fill newRoom->setRoomCenter

    //     // Add the room to the GNN candidates buffer
    //     gnnRoomCandidates.push_back(newRoom);

    //     // [TODO] Add, update, and remove the room in the GNN-based room
    //     candidates
    // }

    // [TODO] Define a 'setGNNRoomCandidates' in System.h
    pSLAM->setGNNRoomCandidates(gnnRoomCandidates);
}

void setGNNBasedRoomCandidates(
    const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms)
{
    // Reset the buffer
    gnnRoomCandidates.clear();

    // Loop through the received GNN rooms
    for (const auto &room : msgGNNRooms.rooms)
    {
        // Create a new room object
        ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

        // Set the room properties
        newRoom->setId(room.id);
        newRoom->setHasKnownLabel(false);

        // [TODO] use room.wallIds to fill newRoom->setWalls
        // [TODO] use room.centroid to fill newRoom->setRoomCenter

        // Add the room to the GNN candidates buffer
        gnnRoomCandidates.push_back(newRoom);

        // [TODO] Add, update, and remove the room in the GNN-based room
        // candidates
    }

    // [TODO] Define a 'setGNNRoomCandidates' in System.h
    pSLAM->setGNNRoomCandidates(gnnRoomCandidates);
}