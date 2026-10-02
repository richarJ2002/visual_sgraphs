/*!
 * File:          common.cc
 *
 * Brief:         This file is a modified version of a file from ORB-SLAM3.
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
 * Date:          20/07/2026
 *
 */

/*!
 * @file            common.cc
 *
 * @brief           Implements the helpers shared by the RGB-D ROS 2 nodes,
 *                  declared in common.hpp: publishing and clearing
 *                  visualisation topics, estimator health bookkeeping and
 *                  sensor set-up.
 */

#include "common.hpp"

#include "MissionHealthTopologyJson.h"

#include "SparseClusterVerdict.h"

#include "../include/PublishTopicsTiming.h"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

/* -------------------------------------------------------------------------- *
 * ORB-SLAM3 STATE
 * -------------------------------------------------------------------------- */

vs_graphs::core::System *p_slamSystem = nullptr;

vs_graphs::core::System::SensorType sensorType =
    vs_graphs::core::System::NOT_SET;

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
 * SGRAPH JSON ARCHIVE CONFIGURATION
 * -------------------------------------------------------------------------- */

std::string sgraphArchiveTestRunDir;

bool sgraphArchiveEnabled = true;

double sgraphArchiveIntervalSec = 5.0;

int sgraphArchiveMaxFiles = 0;

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

std::vector<std::vector<vs_graphs::core::semantic::Marker *>> markersBuffer;

std::vector<vs_graphs::core::semantic::Room *> gnnRoomCandidates;

std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

namespace
{
struct VoxbloxHealthState
{
    std::mutex                            mutex;
    std::chrono::steady_clock::time_point firstInput;
    std::chrono::steady_clock::time_point lastInput;
    std::chrono::steady_clock::time_point lastSkeleton;
    std::chrono::steady_clock::time_point lastSparseGraph;
    std::chrono::steady_clock::time_point lastReset;
    std::chrono::steady_clock::time_point lastLog;
    std::uint64_t                         inputCount{0U};
    std::uint64_t                         resetRevision{0U};
    std::int64_t                          firstInputStamp_ns{0};
    std::int64_t                          lastInputStamp_ns{0};
    std::int64_t                          lastSkeletonStamp_ns{0};
    std::int64_t                          lastSparseGraphStamp_ns{0};
    std::uint32_t                         lastInputWidth{0U};
    std::size_t                           acceptedClusterCount{0U};
    std::size_t                           acceptedVertexCount{0U};
    std::size_t                           acceptedEdgeCount{0U};
    std::string                           lastSparseSummarySignature;
    std::chrono::steady_clock::time_point lastSparseSummaryLog;
    bool                                  hasInput{false};
    bool                                  hasSkeleton{false};
    bool                                  hasSparseGraph{false};
    bool                                  hasReset{false};
    bool                                  wasStalled{false};
};

VoxbloxHealthState voxbloxHealth;

/*!
 * Latest completed cluster-ingest rejection counts. Written by
 * setVoxbloxSkeletonCluster() and read back by the callback summary on the
 * same subscription thread; the sequence distinguishes a fresh all-zero scan
 * from stale data.
 */
vs_graphs::sparse::SparseIngestCounts lastSparseIngest;

double ageSeconds(const std::chrono::steady_clock::time_point &now_in,
                  const std::chrono::steady_clock::time_point &timePoint_in,
                  const bool                                   hasTimePoint_in)
{
    return hasTimePoint_in
               ? std::chrono::duration<double>(now_in - timePoint_in).count()
               : -1.0;
}

double simulatedOutputAgeSeconds(const std::int64_t inputStamp_ns_in,
                                 const std::int64_t firstInputStamp_ns_in,
                                 const std::int64_t outputStamp_ns_in)
{
    const std::int64_t referenceStamp_ns =
        outputStamp_ns_in > 0 ? outputStamp_ns_in : firstInputStamp_ns_in;
    if (inputStamp_ns_in <= 0 || referenceStamp_ns <= 0 ||
        inputStamp_ns_in < referenceStamp_ns)
    {
        return -1.0;
    }
    return static_cast<double>(inputStamp_ns_in - referenceStamp_ns) * 1e-9;
}

void observeVoxbloxInput(const std::uint32_t width_in,
                         const rclcpp::Time &stamp_in)
{
    const std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(voxbloxHealth.mutex);
    if (!voxbloxHealth.hasInput)
    {
        voxbloxHealth.firstInput         = now;
        voxbloxHealth.hasInput           = true;
        voxbloxHealth.firstInputStamp_ns = stamp_in.nanoseconds();
    }
    voxbloxHealth.lastInput      = now;
    voxbloxHealth.lastInputWidth = width_in;
    voxbloxHealth.inputCount++;
    voxbloxHealth.lastInputStamp_ns = stamp_in.nanoseconds();

    const double logAge_s =
        ageSeconds(now,
                   voxbloxHealth.lastLog,
                   voxbloxHealth.lastLog.time_since_epoch().count() != 0);
    const double skeletonAge_s =
        ageSeconds(now,
                   voxbloxHealth.hasSkeleton ? voxbloxHealth.lastSkeleton
                                             : voxbloxHealth.firstInput,
                   true);
    const double sparseAge_s =
        ageSeconds(now,
                   voxbloxHealth.hasSparseGraph ? voxbloxHealth.lastSparseGraph
                                                : voxbloxHealth.firstInput,
                   true);
    const double inputAge_s = ageSeconds(now, voxbloxHealth.lastInput, true);
    const double skeletonSimAge_s =
        simulatedOutputAgeSeconds(voxbloxHealth.lastInputStamp_ns,
                                  voxbloxHealth.firstInputStamp_ns,
                                  voxbloxHealth.lastSkeletonStamp_ns);
    const double sparseSimAge_s =
        simulatedOutputAgeSeconds(voxbloxHealth.lastInputStamp_ns,
                                  voxbloxHealth.firstInputStamp_ns,
                                  voxbloxHealth.lastSparseGraphStamp_ns);
    const bool hasSimTimeEvidence =
        skeletonSimAge_s >= 0.0 && sparseSimAge_s >= 0.0;
    const bool outputOverdue =
        hasSimTimeEvidence ? skeletonSimAge_s >= 10.0 && sparseSimAge_s >= 10.0
                           : skeletonAge_s >= 10.0 && sparseAge_s >= 10.0;
    const bool stalled = inputAge_s < 2.0 && outputOverdue;
    if (logAge_s >= 3.0 || logAge_s < 0.0 ||
        stalled != voxbloxHealth.wasStalled)
    {
        const double elapsedInput_s = std::max(
            std::chrono::duration<double>(now - voxbloxHealth.firstInput)
                .count(),
            1e-6);
        std::cout
            << "SG_PIPELINE {\"event\":\"voxblox_health\","
               "\"state\":\""
            << (stalled ? "VOXBLOX_STALLED" : "VOXBLOX_ACTIVE")
            << "\",\"input_width\":" << voxbloxHealth.lastInputWidth
            << ",\"input_rate_hz\":"
            << static_cast<double>(voxbloxHealth.inputCount) / elapsedInput_s
            << ",\"last_skeleton_age_s\":" << skeletonAge_s
            << ",\"last_sparse_age_s\":" << sparseAge_s
            << ",\"skeleton_sim_age_s\":" << skeletonSimAge_s
            << ",\"sparse_sim_age_s\":" << sparseSimAge_s
            << ",\"last_reset_age_s\":"
            << ageSeconds(now, voxbloxHealth.lastReset, voxbloxHealth.hasReset)
            << ",\"reset_revision\":" << voxbloxHealth.resetRevision
            << ",\"accepted_clusters\":" << voxbloxHealth.acceptedClusterCount
            << ",\"accepted_vertices\":" << voxbloxHealth.acceptedVertexCount
            << ",\"accepted_edges\":" << voxbloxHealth.acceptedEdgeCount << "}"
            << std::endl;
        voxbloxHealth.lastLog    = now;
        voxbloxHealth.wasStalled = stalled;
    }
}
} // namespace

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
 * MAPPED-ROOM AND MAPPED-PASSAGE PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<vs_graphs::msg::VSGraphsAllDetectdetRooms>::SharedPtr
    pubAllRooms = nullptr;

rclcpp::Publisher<vs_graphs::msg::VSGraphsAllPassagesData>::SharedPtr
    pubAllPassages = nullptr;

rclcpp::Publisher<vs_graphs::msg::VSGraphsAllFloorsData>::SharedPtr
    pubAllFloors = nullptr;

/* -------------------------------------------------------------------------- *
 * STRUCTURAL-ELEMENT PUBLISHERS
 * -------------------------------------------------------------------------- */

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pubStructuralElements = nullptr;

/*!
 * @brief           Removes indefinitely-lived RViz markers from a previous map
 *                  revision.
 *
 * @param[in]       p_markerPublisher_in
 *                  Marker publisher whose displayed state must be cleared.
 *
 * @param[in]       msgTime_s_in
 *                  Timestamp assigned to the deletion marker.
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
 * @brief           Replaces an indefinitely-displayed point cloud with an empty
 *                  cloud.
 *
 * @param[in]       p_pointCloudPublisher_in
 *                  Point-cloud publisher whose displayed state must be cleared.
 *
 * @param[in]       frameId_in
 *                  Coordinate frame assigned to the empty cloud.
 *
 * @param[in]       msgTime_s_in
 *                  Timestamp assigned to the empty cloud.
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
 * @param[in]   msgTime_s_in
 *              Timestamp assigned to reset messages.
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

rclcpp::Service<vs_graphs::srv::EstimatorHealth>::SharedPtr srvEstimatorHealth =
    nullptr;

std::atomic<double> estimatorFramesPerSecond{0.0};
std::atomic<double> estimatorFrameWallSeconds{0.0};

void recordEstimatorFrame(const double frameInterval_seconds)
{
    estimatorFramesPerSecond.store(
        frameInterval_seconds > 1e-6 ? 1.0 / frameInterval_seconds : 0.0);
    estimatorFrameWallSeconds.store(
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

void addSegmentationToSystem(
    const segmenter_ros::msg::SegmenterDataMsg &msgSegImage_in,
    const rclcpp::Logger                       &logger_in)
{
    cv_bridge::CvImageConstPtr cv_imgSeg;
    const uint64_t             keyFrameId = msgSegImage_in.key_frame_id.data;
    try
    {
        cv_imgSeg =
            cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(
                                    msgSegImage_in.segmented_image_uncertainty),
                                sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(logger_in, "[Error] `cv_bridge` exception: %s", e.what());
        return;
    }

    // Convert to PCL PointCloud2 from `sensor_msgs` PointCloud2
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage_in.segmented_image_probability,
                           *pclPc2SegPrb);

    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(
        keyFrameId,
        cv_imgSeg->image,
        pclPc2SegPrb);
    if (p_slamSystem->addSegmentedImage(&tuple) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addSegmentedImage returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
}

namespace
{
const char *sensorModeName()
{
    return sensorType == vs_graphs::core::System::IMU_RGBD ? "rgbd_inertial"
                                                           : "rgbd";
}
} // namespace

/* -------------------------------------------------------------------------- *
 * PUBLIC METHODS
 * -------------------------------------------------------------------------- */

void appendFloorMarkers(
    const std::vector<vs_graphs::core::semantic::Floor *> &mappedFloors_in,
    const rclcpp::Time                                    &msgTime_s_in,
    visualization_msgs::msg::MarkerArray &structuralElementMarkerArray_out)
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

    for (vs_graphs::core::semantic::Floor *mappedFloor : mappedFloors_in)
    {
        if (mappedFloor == nullptr)
        {
            continue;
        }

        int mappedFloorId{};
        if (mappedFloor->getId(mappedFloorId) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const int floorMarkerId = static_cast<int>(mappedFloorId);

        std::vector<vs_graphs::core::semantic::Room *> associatedRooms{};
        if (mappedFloor->getRooms(associatedRooms) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRooms returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Do not display floors without any associated rooms */
        if (associatedRooms.empty())
        {
            appendFloorDeleteMarkers(floorMarkerId);

            continue;
        }

        Eigen::Vector3d floorCentroid_world_m{};
        if (mappedFloor->getCentroid(floorCentroid_world_m) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

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
        if (sensorType == vs_graphs::core::System::IMU_RGBD)
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

        std::string mappedFloorName{};
        if (mappedFloor->getName(mappedFloorName) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        floorLabelMarker.text = mappedFloorName;

        floorLabelMarker.pose.position.x = floorDisplayPosition_world_m.x();
        floorLabelMarker.pose.position.y = floorDisplayPosition_world_m.y();
        floorLabelMarker.pose.position.z = floorDisplayPosition_world_m.z();

        if (sensorType == vs_graphs::core::System::IMU_RGBD)
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

        for (vs_graphs::core::semantic::Room *associatedRoom : associatedRooms)
        {
            bool associatedRoomIsBad{};
            if (!(associatedRoom == nullptr) &&
                associatedRoom->isBad(associatedRoomIsBad) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (associatedRoom == nullptr || associatedRoomIsBad)
            {
                continue;
            }

            vs_graphs::core::semantic::Room::RoomVariant roomType{};
            if (associatedRoom->getRoomVariant(roomType) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            const bool isConfirmedRoom =
                roomType == vs_graphs::core::semantic::Room::RoomVariant::ROOM;

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
    const std::vector<vs_graphs::core::semantic::Passage *> &mappedPassages_in,
    const rclcpp::Time                                      &msgTime_s_in,
    visualization_msgs::msg::MarkerArray &structuralElementMarkerArray_out)
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
    for (vs_graphs::core::semantic::Passage *mappedPassage : mappedPassages_in)
    {
        if (mappedPassage == nullptr)
        {
            continue;
        }

        int mappedPassageId{};
        if (mappedPassage->getId(mappedPassageId) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const int passageMarkerId = static_cast<int>(mappedPassageId);

        /* Remove invalidated passages from RViz. Passage::isBad() is a
         * per-session addition (0-associated-room invalidation); until now
         * every passage the Atlas ever created stayed drawn forever, since
         * this loop had no equivalent to appendRoomMarkers()'s own
         * isBad()-checks-delete pattern above. */
        bool mappedPassageIsBad{};
        if (mappedPassage->isBad(mappedPassageIsBad) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mappedPassageIsBad)
        {
            appendPassageDeleteMarkers(passageMarkerId);
            continue;
        }

        /* Recovery proxies carry identity and topology but no positioned
         * geometry until re-observed: drawing them would plant doorways at
         * unlocated coordinates. Fresh passages always carry aperture
         * dimensions from detection, so only proxies trip this gate. */
        double proxyWidth_m{};
        if (mappedPassage->getWidth(proxyWidth_m) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWidth returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        double proxyHeight_m{};
        if (mappedPassage->getHeight(proxyHeight_m) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHeight returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool mappedPassageIsRecoveryProxy{};
        if (mappedPassage->isRecoveryProxy(mappedPassageIsRecoveryProxy) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isRecoveryProxy returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (mappedPassageIsRecoveryProxy &&
            (!std::isfinite(proxyWidth_m) || !std::isfinite(proxyHeight_m) ||
             proxyWidth_m <= 0.0 || proxyHeight_m <= 0.0))
        {
            continue;
        }

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

        bool isPassageOpen{};
        if (mappedPassage->isPassable(isPassageOpen) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
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

        std::string passageLabel = "Passage#" +
                                   std::to_string(passageMarkerId) +
                                   (isPassageOpen ? " [open]" : " [blocked]");

        passageLabelMarker.text = passageLabel;

        passageLabelMarker.pose.position.x = passageDisplayPoint_SE.point.x;
        /* Opposite side from room labels (which use + textOffset_m): the
         * doorway label and any nearby room label no longer stack. */
        passageLabelMarker.pose.position.y =
            passageDisplayPoint_SE.point.y - textOffset_m;
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

/*
 * Computes the ordered horizontal corner polygon of a room boundary.
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
std::vector<Eigen::Vector3d>
    computeRoomCorners(const vs_graphs::core::semantic::Room *room_in)
{
    std::vector<Eigen::Vector3d> corners_World_m;

    if (room_in == nullptr)
    {
        return corners_World_m;
    }

    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    if (room_in->getWalls(walls) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (walls.size() < 3)
    {
        return corners_World_m;
    }

    /* Resolve the horizontal plane from the room's ground plane, falling back
     * to the world vertical axis when no ground surface is available. */
    Eigen::Vector3d groundNormal_World_m = Eigen::Vector3d::UnitZ();

    vs_graphs::core::geometric::Plane *p_groundPlane = nullptr;
    if (room_in->getGroundPlane(p_groundPlane) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGroundPlane returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    bool   hasFloorPlane = false;
    double floorHeight_m = 0.0;

    bool groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEquation_World =
            groundPlaneGetGlobalEquation.coeffs();
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

    Eigen::Vector3d roomCentroid_World_m{};
    if (room_in->getCentroid(roomCentroid_World_m) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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

    for (vs_graphs::core::geometric::Plane *p_wall : walls)
    {
        bool wallIsBad{};
        if (!(p_wall == nullptr) &&
            p_wall->isBad(wallIsBad) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || wallIsBad)
        {
            continue;
        }

        g2o::Plane3D wallGetGlobalEquation{};
        if (p_wall->getGlobalEquation(wallGetGlobalEquation) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d wallEquation_World =
            wallGetGlobalEquation.coeffs();
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

        Eigen::Vector3d wallCentroid2D_World_m{};
        if (p_wall->getCentroid(wallCentroid2D_World_m) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

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
    const std::vector<vs_graphs::core::semantic::Room *>    &mappedRooms_in,
    const std::vector<vs_graphs::core::semantic::Floor *>   &mappedFloors_in,
    const std::vector<vs_graphs::core::semantic::Passage *> &mappedPassages_in,
    const rclcpp::Time                                      &msgTime_s_in,
    visualization_msgs::msg::MarkerArray &structuralElementMarkerArray_out)
{
    /* Room-to-floor green lines were removed (operator: passages are the
     * only room links); the floor list is kept for signature stability. */
    (void)mappedFloors_in;

    constexpr double textOffset_m = -0.5;

    /* A live passage prospective renders as the same room object with
     * state-driven appearance below, never as a parallel marker set. Other
     * provisional rooms stay hidden, as before. */
    const auto isLiveProspectiveRoom =
        [&mappedRooms_in,
         &mappedPassages_in](vs_graphs::core::semantic::Room *p_room_in)
    {
        bool room_inIsBad{};
        if (!(p_room_in == nullptr) &&
            p_room_in->isBad(room_inIsBad) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::semantic::Room::RoomVariant room_inRoomVariant{};
        if (!(p_room_in == nullptr || room_inIsBad) &&
            p_room_in->getRoomVariant(room_inRoomVariant) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room_in == nullptr || room_inIsBad ||
            room_inRoomVariant !=
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            return false;
        }
        bool linkedByPassage = false;
        for (vs_graphs::core::semantic::Passage *p_passage : mappedPassages_in)
        {
            bool passageIsBad{};
            if ((p_passage != nullptr) &&
                p_passage->isBad(passageIsBad) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            vs_graphs::core::semantic::Room *p_passageProspectiveRoom = nullptr;
            if ((p_passage != nullptr && !passageIsBad) &&
                p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage != nullptr && !passageIsBad &&
                p_passageProspectiveRoom == p_room_in)
            {
                linkedByPassage = true;
                break;
            }
        }
        if (!linkedByPassage)
        {
            return false;
        }
        /* A hypothesis absorbed next to a confirmed room would stack its
         * label on the confirmed label at the same position. */
        for (vs_graphs::core::semantic::Room *p_otherRoom : mappedRooms_in)
        {
            bool otherRoomIsBad{};
            if (!(p_otherRoom == nullptr || p_otherRoom == p_room_in) &&
                p_otherRoom->isBad(otherRoomIsBad) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            vs_graphs::core::semantic::Room::RoomVariant otherRoomRoomVariant{};
            if (!(p_otherRoom == nullptr || p_otherRoom == p_room_in ||
                  otherRoomIsBad) &&
                p_otherRoom->getRoomVariant(otherRoomRoomVariant) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherRoom == nullptr || p_otherRoom == p_room_in ||
                otherRoomIsBad ||
                otherRoomRoomVariant !=
                    vs_graphs::core::semantic::Room::RoomVariant::ROOM)
            {
                continue;
            }
            Eigen::Vector3d otherRoomCentroid{};
            if (p_otherRoom->getCentroid(otherRoomCentroid) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d room_inCentroid{};
            if (p_room_in->getCentroid(room_inCentroid) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if ((otherRoomCentroid.cast<double>() - room_inCentroid).norm() <
                1.5)
            {
                return false;
            }
        }
        return true;
    };

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

        visualization_msgs::msg::Marker deleteRoomBoundaryLoopMarker;

        deleteRoomBoundaryLoopMarker.header.frame_id = frameWorld;
        deleteRoomBoundaryLoopMarker.header.stamp    = msgTime_s_in;

        deleteRoomBoundaryLoopMarker.ns = "roomBoundaryLoop";
        deleteRoomBoundaryLoopMarker.id = roomId;

        deleteRoomBoundaryLoopMarker.action =
            visualization_msgs::msg::Marker::DELETE;

        structuralElementMarkerArray_out.markers.push_back(
            deleteRoomBoundaryLoopMarker);
    };

    for (vs_graphs::core::semantic::Room *mappedRoom : mappedRooms_in)
    {
        /* Null pointers do not contain an ID that can be deleted */
        if (mappedRoom == nullptr)
        {
            continue;
        }

        int mappedRoomId{};
        if (mappedRoom->getId(mappedRoomId) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const int roomMarkerId = static_cast<int>(mappedRoomId);

        vs_graphs::core::semantic::Room::RoomVariant roomType{};
        if (mappedRoom->getRoomVariant(roomType) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        const bool isConfirmedRoom =
            roomType == vs_graphs::core::semantic::Room::RoomVariant::ROOM;

        /* Remove bad structural elements from RViz. Provisional rooms stay
         * hidden unless a live passage hypothesizes them. */
        bool mappedRoomIsBad{};
        if (mappedRoom->isBad(mappedRoomIsBad) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mappedRoomIsBad)
        {
            appendRoomDeleteMarkers(roomMarkerId);

            continue;
        }
        const bool showProspectiveRoom =
            !isConfirmedRoom && isLiveProspectiveRoom(mappedRoom);
        if (!isConfirmedRoom && !showProspectiveRoom)
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
        /* State-driven appearance: hypotheses render semi-transparent in the
         * same namespace, so promotion flips appearance in place instead of
         * swapping marker identities. */
        roomMarker.color.a = isConfirmedRoom ? 1.0F : 0.6F;

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

        vs_graphs::core::semantic::Room::BoundaryStatus
            mappedRoomBoundaryStatus{};
        if (mappedRoom->getBoundaryStatus(mappedRoomBoundaryStatus) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBoundaryStatus returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        switch (mappedRoomBoundaryStatus)
        {
        case vs_graphs::core::semantic::Room::BoundaryStatus::UNOBSERVED:
            boundaryStatusLabel = " [unobserved]";
            break;
        case vs_graphs::core::semantic::Room::BoundaryStatus::INCOMPLETE:
            boundaryStatusLabel = " [incomplete]";
            break;
        case vs_graphs::core::semantic::Room::BoundaryStatus::COMPLETE:
            boundaryStatusLabel = " [complete]";
            break;
        case vs_graphs::core::semantic::Room::BoundaryStatus::CONFLICTING:
            boundaryStatusLabel = " [conflicting]";
            break;
        }

        /*
         * Boundary maturity is diagnostic state, not a passage gate. Keeping
         * it in the room label makes incomplete but traversable observations
         * explicit without changing the semantic graph topology.
         */
        std::string mappedRoomName{};
        if ((isConfirmedRoom) &&
            mappedRoom->getName(mappedRoomName) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string mappedRoomName2{};
        if (!(isConfirmedRoom) &&
            mappedRoom->getName(mappedRoomName2) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        roomLabelMarker.text = isConfirmedRoom
                                   ? mappedRoomName + boundaryStatusLabel
                                   : mappedRoomName2 + " [prospective]";

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
         * ROOM-BOUNDARY LOOP OUTLINE: closed corner polygon, only while the
         * validator has classed the wall loop COMPLETE (axiom (f)'s
         * corner-alignment result, exposed via
         * Room::getBoundaryCorners_World_m()).
         * ------------------------------------------------------------------ */

        std::vector<Eigen::Vector3d> boundaryCorners_World_m{};
        if (mappedRoom->getBoundaryCorners_World_m(boundaryCorners_World_m) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getBoundaryCorners_World_m returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        bool mappedRoomIsBoundaryComplete{};
        if (mappedRoom->isBoundaryComplete(mappedRoomIsBoundaryComplete) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBoundaryComplete returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (mappedRoomIsBoundaryComplete &&
            boundaryCorners_World_m.size() >= 3U)
        {
            visualization_msgs::msg::Marker roomBoundaryLoopMarker;

            roomBoundaryLoopMarker.header.frame_id = frameWorld;
            roomBoundaryLoopMarker.header.stamp    = msgTime_s_in;

            roomBoundaryLoopMarker.ns = "roomBoundaryLoop";
            roomBoundaryLoopMarker.id = roomMarkerId;

            roomBoundaryLoopMarker.type =
                visualization_msgs::msg::Marker::LINE_STRIP;
            roomBoundaryLoopMarker.action =
                visualization_msgs::msg::Marker::ADD;

            roomBoundaryLoopMarker.pose.orientation.w = 1.0;

            roomBoundaryLoopMarker.scale.x = 0.06;

            roomBoundaryLoopMarker.color.r = roomColourRed;
            roomBoundaryLoopMarker.color.g = roomColourGreen;
            roomBoundaryLoopMarker.color.b = roomColourBlue;
            roomBoundaryLoopMarker.color.a = 1.0F;

            roomBoundaryLoopMarker.lifetime =
                rclcpp::Duration::from_seconds(1.0);

            roomBoundaryLoopMarker.points.reserve(
                boundaryCorners_World_m.size() + 1U);
            for (const Eigen::Vector3d &corner_World_m :
                 boundaryCorners_World_m)
            {
                geometry_msgs::msg::Point cornerPoint;
                cornerPoint.x = corner_World_m.x();
                cornerPoint.y = corner_World_m.y();
                cornerPoint.z = corner_World_m.z();
                roomBoundaryLoopMarker.points.push_back(cornerPoint);
            }
            /* Close the loop back to the first corner. */
            if (!roomBoundaryLoopMarker.points.empty())
            {
                roomBoundaryLoopMarker.points.push_back(
                    roomBoundaryLoopMarker.points.front());
            }

            structuralElementMarkerArray_out.markers.push_back(
                roomBoundaryLoopMarker);
        }
        else
        {
            /* Remove a previously published outline once the room stops
             * being validated as COMPLETE. */
            visualization_msgs::msg::Marker staleRoomBoundaryLoopMarker;

            staleRoomBoundaryLoopMarker.header.frame_id = frameWorld;
            staleRoomBoundaryLoopMarker.header.stamp    = msgTime_s_in;

            staleRoomBoundaryLoopMarker.ns = "roomBoundaryLoop";
            staleRoomBoundaryLoopMarker.id = roomMarkerId;

            staleRoomBoundaryLoopMarker.action =
                visualization_msgs::msg::Marker::DELETE;

            structuralElementMarkerArray_out.markers.push_back(
                staleRoomBoundaryLoopMarker);
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

        /* Wider than floorRoomEdges (0.05) and the skeleton path so a short
         * room-to-wall segment stays visually distinguishable even when it
         * overlaps those longer lines on screen -- see the "Plane#5 does not
         * seem to be linked with a wall" investigation this addresses. */
        roomWallAssociationMarker.scale.x = 0.08;

        roomWallAssociationMarker.color.r = roomColourRed;
        roomWallAssociationMarker.color.g = roomColourGreen;
        roomWallAssociationMarker.color.b = roomColourBlue;
        roomWallAssociationMarker.color.a = 0.9F;

        roomWallAssociationMarker.lifetime = rclcpp::Duration::from_seconds(0);

        std::vector<vs_graphs::core::geometric::Plane *> associatedWalls{};
        if (mappedRoom->getWalls(associatedWalls) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        roomWallAssociationMarker.points.reserve(associatedWalls.size() * 2);

        for (vs_graphs::core::geometric::Plane *associatedWall :
             associatedWalls)
        {
            bool associatedWallIsBad{};
            if (!(associatedWall == nullptr) &&
                associatedWall->isBad(associatedWallIsBad) !=
                    vs_graphs::core::geometric::PlaneStatus::
                        PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (associatedWall == nullptr || associatedWallIsBad)
            {
                continue;
            }

            Eigen::Vector3d wallCentroid_BC_m{};
            if (associatedWall->getCentroid(wallCentroid_BC_m) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

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
         * ROOM-TO-PASSAGE ASSOCIATIONS: the only lines that connect rooms.
         * Each segment runs from a room centroid to one of its passages,
         * so room<->room connectivity is shown exclusively via passages.
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

        std::vector<vs_graphs::core::semantic::Passage *> associatedPassages{};
        if (mappedRoom->getPassages(associatedPassages) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        roomPassageAssociationMarker.points.reserve(associatedPassages.size() *
                                                    2);

        roomPassageAssociationMarker.colors.reserve(associatedPassages.size() *
                                                    2);

        for (vs_graphs::core::semantic::Passage *associatedPassage :
             associatedPassages)
        {
            bool associatedPassageIsBad{};
            if (!(associatedPassage == nullptr) &&
                associatedPassage->isBad(associatedPassageIsBad) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (associatedPassage == nullptr || associatedPassageIsBad)
            {
                continue;
            }

            /* Same recovery-proxy gate as the passage node block: an
             * unpositioned proxy reports the origin as its centroid, which
             * would pin a room-to-origin line (and, at reset time, a line
             * onto the camera itself). */
            double proxyWidth_m{};
            if (associatedPassage->getWidth(proxyWidth_m) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double proxyHeight_m{};
            if (associatedPassage->getHeight(proxyHeight_m) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            bool associatedPassageIsRecoveryProxy{};
            if (associatedPassage->isRecoveryProxy(
                    associatedPassageIsRecoveryProxy) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isRecoveryProxy returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (associatedPassageIsRecoveryProxy &&
                (!std::isfinite(proxyWidth_m) ||
                 !std::isfinite(proxyHeight_m) || proxyWidth_m <= 0.0 ||
                 proxyHeight_m <= 0.0))
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

            bool associatedPassageIsPassable{};
            if (associatedPassage->isPassable(associatedPassageIsPassable) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (associatedPassageIsPassable)
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

void clearKFClsClouds(
    std::vector<vs_graphs::core::KeyFrame *> keyframeVector_in)
{
    /* Iterate through keyframes and clear the cls point clouds */
    for (vs_graphs::core::KeyFrame *&keyframe : keyframeVector_in)
    {
        if (keyframe->clearClsClouds() !=
            vs_graphs::core::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearClsClouds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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

std::pair<double, std::vector<vs_graphs::core::semantic::Marker *>>
    findNearestMarker(double frameTimestamp_in)
{
    /* Init variable of the minimum time difference */
    double minTimeDifference = 100;

    /* Init a variable which will be used to find best match to marker */
    std::vector<vs_graphs::core::semantic::Marker *> matchedMarkers;

    /* Loop through the markersBuffer */
    for (const std::vector<vs_graphs::core::semantic::Marker *> &markers :
         markersBuffer)
    {
        /* Find the time difference */
        double time2{};
        if (markers[0]->getTime(time2) !=
            vs_graphs::core::semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTime returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        double timeDifference = time2 - frameTimestamp_in;

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
    vs_graphs::core::semantic::Passage *passage_in,
    const rclcpp::Time                 &msgTime_in,
    const double                        verticalOffset_in,
    geometry_msgs::msg::PointStamped   &passagePointSE_out,
    geometry_msgs::msg::PointStamped   &passagePointWorld_out)
{
    /* Confirm that the passage and TF buffer are valid */
    if (passage_in == nullptr || tfBuffer_ == nullptr)
    {
        return false;
    }

    /* Extract the physical passage centroid */
    Eigen::Vector3d passageCentroid{};
    if (passage_in->getCentroid(passageCentroid) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
        if (sensorType == vs_graphs::core::System::IMU_RGBD)
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

bool getRoomDisplayPoints(vs_graphs::core::semantic::Room  *room_in,
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
    Eigen::Vector3d roomCentroid{};
    if (room_in->getCentroid(roomCentroid) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
    mapPointToPointcloud(std::vector<vs_graphs::core::MapPoint *> mapPoints_in,
                         rclcpp::Time                             msgTime_in)
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
    cloud.width           = static_cast<uint32_t>(mapPoints_in.size());
    cloud.point_step      = numChannels * sizeof(float);
    cloud.row_step        = cloud.point_step * cloud.width;
    cloud.fields.resize(numChannels);

    // Set the fields of the point cloud
    for (int idx = 0; idx < numChannels; idx++)
    {
        cloud.fields[idx].count    = 1;
        cloud.fields[idx].name     = channelId[idx];
        cloud.fields[idx].offset   = static_cast<uint32_t>(idx * sizeof(float));
        cloud.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    // Set the data of the point cloud
    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloudDataPtr = &(cloud.data[0]);

    // Populate the point cloud with the map points
    for (unsigned int idx = 0; idx < cloud.width; idx++)
    {
        bool isBad2{};
        if ((mapPoints_in[idx]) &&
            mapPoints_in[idx]->isBad(isBad2) !=
                vs_graphs::core::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPoints_in[idx] && !isBad2)
        {
            Eigen::Vector3f worldPos{};
            if (mapPoints_in[idx]->getWorldPos(worldPos) !=
                vs_graphs::core::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d P3Dw = worldPos.cast<double>();
            tf2::Vector3    pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());
            float           dataArray[numChannels] = {
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

void publishAllMappedWalls(
    std::vector<vs_graphs::core::geometric::Plane *> wallsList_in,
    rclcpp::Time                                     msgTime_s_in)
{
    /* Variables */
    vs_graphs::msg::VSGraphsAllWallsData wallDataMsg;

    /* Fill the data message with wall information */
    wallDataMsg.header.stamp    = msgTime_s_in;
    wallDataMsg.header.frame_id = frameWorld;

    /* Fill in the walls data for each wall in vector */
    for (vs_graphs::core::geometric::Plane *const &wall : wallsList_in)
    {
        vs_graphs::core::geometric::Plane::PlaneVariant wallPlaneType{};
        if (!(!wall) &&
            wall->getPlaneType(wallPlaneType) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!wall || wallPlaneType !=
                         vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
            continue;

        /* Init variable of the lenfth of the wall */
        float length = 0.0f;

        /* Get the point clouds for the wall */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud{};
        if (wall->getMapClouds(wallCloud) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapClouds returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

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
        wallData.length = length;
        int wallGetId{};
        if (wall->getId(wallGetId) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        wallData.id = wallGetId;
        Eigen::Vector3d wallGetCentroid{};
        if (wall->getCentroid(wallGetCentroid) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        wallData.centroid.x = wallGetCentroid.x();
        Eigen::Vector3d wallGetCentroid2{};
        if (wall->getCentroid(wallGetCentroid2) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        wallData.centroid.y = wallGetCentroid2.y();
        Eigen::Vector3d wallGetCentroid3{};
        if (wall->getCentroid(wallGetCentroid3) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        wallData.centroid.z = wallGetCentroid3.z();
        g2o::Plane3D wallGetGlobalEquation{};
        if (wall->getGlobalEquation(wallGetGlobalEquation) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        wallData.normal.x = wallGetGlobalEquation.normal().x();
        g2o::Plane3D wallGetGlobalEquation2{};
        if (wall->getGlobalEquation(wallGetGlobalEquation2) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        wallData.normal.y = wallGetGlobalEquation2.normal().y();
        g2o::Plane3D wallGetGlobalEquation3{};
        if (wall->getGlobalEquation(wallGetGlobalEquation3) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        wallData.normal.z = wallGetGlobalEquation3.normal().z();

        /* Add the wall to the message */
        wallDataMsg.walls.push_back(wallData);
    }

    /* Publish all mapped walls */
    pubAllWalls_new->publish(wallDataMsg);
}

void publishAllMappedRooms(
    std::vector<vs_graphs::core::semantic::Room *> roomsList_in,
    rclcpp::Time                                   msgTime_s_in)
{
    /* Variables */
    vs_graphs::msg::VSGraphsAllDetectdetRooms roomDataMsg;

    /* Fill the data message header */
    roomDataMsg.header.stamp    = msgTime_s_in;
    roomDataMsg.header.frame_id = frameWorld;

    /* Fill in the room data for each room in vector */
    for (vs_graphs::core::semantic::Room *const &room : roomsList_in)
    {
        if (!room)
            continue;

        vs_graphs::msg::VSGraphsRoomData roomData;
        int                              roomId{};
        if (room->getId(roomId) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        roomData.id = roomId;
        Eigen::Vector3d roomCentroid{};
        if (room->getCentroid(roomCentroid) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomData.centroid.x = roomCentroid.x();
        Eigen::Vector3d roomCentroid2{};
        if (room->getCentroid(roomCentroid2) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomData.centroid.y = roomCentroid2.y();
        Eigen::Vector3d roomCentroid3{};
        if (room->getCentroid(roomCentroid3) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomData.centroid.z = roomCentroid3.z();

        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (room->getWalls(roomWalls) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (vs_graphs::core::geometric::Plane *const &wall : roomWalls)
        {
            if (wall)
            {
                int wallGetId{};
                if (wall->getId(wallGetId) !=
                    vs_graphs::core::geometric::PlaneStatus::
                        PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                roomData.wall_ids.push_back(wallGetId);
            }
        }

        /* Add the room to the message */
        roomDataMsg.rooms.push_back(roomData);
    }

    /* Publish all mapped rooms */
    pubAllRooms->publish(roomDataMsg);
}

void publishAllMappedPassages(
    std::vector<vs_graphs::core::semantic::Passage *> passagesList_in,
    std::vector<vs_graphs::core::semantic::Room *>    roomsList_in,
    rclcpp::Time                                      msgTime_s_in)
{
    /* Variables */
    vs_graphs::msg::VSGraphsAllPassagesData passageDataMsg;

    /* Fill the data message header */
    passageDataMsg.header.stamp    = msgTime_s_in;
    passageDataMsg.header.frame_id = frameWorld;

    /* Fill in the passage data for each passage in vector */
    for (vs_graphs::core::semantic::Passage *const &passage : passagesList_in)
    {
        bool passageIsBad{};
        if (!(!passage) && passage->isBad(passageIsBad) !=
                               vs_graphs::core::semantic::PassageStatus::
                                   PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!passage || passageIsBad)
            continue;

        vs_graphs::msg::VSGraphsPassageData passageData;
        int                                 passageId{};
        if (passage->getId(passageId) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        passageData.id = passageId;
        bool passageIsPassable{};
        if (passage->isPassable(passageIsPassable) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        passageData.passable = passageIsPassable;
        Eigen::Vector3d passageCentroid{};
        if (passage->getCentroid(passageCentroid) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        passageData.centroid.x = passageCentroid.x();
        Eigen::Vector3d passageCentroid2{};
        if (passage->getCentroid(passageCentroid2) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        passageData.centroid.y = passageCentroid2.y();
        Eigen::Vector3d passageCentroid3{};
        if (passage->getCentroid(passageCentroid3) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        passageData.centroid.z = passageCentroid3.z();
        double passageWidth{};
        if (passage->getWidth(passageWidth) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWidth returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        passageData.width = static_cast<float>(passageWidth);
        double passageHeight{};
        if (passage->getHeight(passageHeight) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHeight returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        passageData.height = static_cast<float>(passageHeight);

        /*!
         * The "known-side" room is whichever mapped room owns one of this
         * passage's associated walls (Passage stores its walls, not a
         * direct room back-pointer); -1 if none of the currently mapped
         * rooms claim any of them.
         */
        passageData.known_room_id = -1;
        std::vector<vs_graphs::core::geometric::Plane *> associatedWalls{};
        if (passage->getAssociateWalls(associatedWalls) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateWalls returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (vs_graphs::core::semantic::Room *const &room : roomsList_in)
        {
            if (!room)
                continue;

            bool ownsAssociatedWall = false;
            std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
            if (room->getWalls(roomWalls) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::geometric::Plane *const &roomWall : roomWalls)
            {
                if (!roomWall)
                    continue;

                for (vs_graphs::core::geometric::Plane *const &associatedWall :
                     associatedWalls)
                {
                    int associatedWallGetId{};
                    if ((associatedWall) &&
                        associatedWall->getId(associatedWallGetId) !=
                            vs_graphs::core::geometric::PlaneStatus::
                                PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int roomWallGetId{};
                    if ((associatedWall) &&
                        roomWall->getId(roomWallGetId) !=
                            vs_graphs::core::geometric::PlaneStatus::
                                PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (associatedWall && associatedWallGetId == roomWallGetId)
                    {
                        ownsAssociatedWall = true;
                        break;
                    }
                }

                if (ownsAssociatedWall)
                    break;
            }

            if (ownsAssociatedWall)
            {
                int roomId{};
                if (room->getId(roomId) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passageData.known_room_id = roomId;
                break;
            }
        }

        std::optional<int> prospectiveRoomId{};
        if (passage->getProspectiveRoomId(prospectiveRoomId) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        passageData.prospective_room_id =
            prospectiveRoomId.has_value() ? prospectiveRoomId.value() : -1;

        /*!
         * Populate associated wall plane IDs from the passage's wall
         * associations. This enables the SGraph JSON capture tool to record
         * which walls each passage belongs to.
         */
        passageData.associated_wall_ids.clear();
        for (vs_graphs::core::geometric::Plane *const &associatedWall :
             associatedWalls)
        {
            if (associatedWall)
            {
                int associatedWallGetId2{};
                if (associatedWall->getId(associatedWallGetId2) !=
                    vs_graphs::core::geometric::PlaneStatus::
                        PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passageData.associated_wall_ids.push_back(
                    static_cast<int>(associatedWallGetId2));
            }
        }

        /* Add the passage to the message */
        passageDataMsg.passages.push_back(passageData);
    }

    /* Publish all mapped passages */
    pubAllPassages->publish(passageDataMsg);
}

void publishAllMappedFloors(
    std::vector<vs_graphs::core::semantic::Floor *> floorsList_in,
    rclcpp::Time                                    msgTime_s_in)
{
    /* Variables */
    vs_graphs::msg::VSGraphsAllFloorsData floorDataMsg;

    /* Fill the data message header */
    floorDataMsg.header.stamp    = msgTime_s_in;
    floorDataMsg.header.frame_id = frameWorld;

    /* Fill in the floor data for each floor in vector */
    for (vs_graphs::core::semantic::Floor *const &floor : floorsList_in)
    {
        if (!floor)
            continue;

        vs_graphs::msg::VSGraphsFloorData floorData;
        int                               floorId{};
        if (floor->getId(floorId) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        floorData.id = floorId;

        std::optional<vs_graphs::core::semantic::Floor::PlaneIdentity>
            planeIdentity{};
        if (floor->getPlaneIdentity(planeIdentity) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneIdentity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        floorData.has_plane_identity = planeIdentity.has_value();
        if (planeIdentity.has_value())
        {
            floorData.normal.x = planeIdentity->equation_World.x();
            floorData.normal.y = planeIdentity->equation_World.y();
            floorData.normal.z = planeIdentity->equation_World.z();
            floorData.offset_d =
                static_cast<float>(planeIdentity->equation_World.w());
            floorData.finite_support_count =
                static_cast<int32_t>(planeIdentity->finiteSupportCount);
            floorData.observation_count =
                static_cast<int32_t>(planeIdentity->observationCount);
        }

        /* Add the floor to the message */
        floorDataMsg.floors.push_back(floorData);
    }

    /* Publish all mapped floors */
    pubAllFloors->publish(floorDataMsg);
}

namespace
{
/*!
 * @brief       Sanitises a double for nlohmann::json.
 *
 *              nlohmann::json rejects non-finite values, so they are stored
 *              as strings following the MissionHealthTopologyJson convention.
 */
Json sanitiseArchiveDouble(const double value_in)
{
    if (std::isnan(value_in))
    {
        return Json("NaN");
    }
    if (std::isinf(value_in))
    {
        return Json(value_in > 0.0 ? "Infinity" : "-Infinity");
    }
    return Json(value_in);
}

Json archiveVector3(const Eigen::Vector3d &vector_in)
{
    Json array = Json::array();
    array.push_back(sanitiseArchiveDouble(vector_in.x()));
    array.push_back(sanitiseArchiveDouble(vector_in.y()));
    array.push_back(sanitiseArchiveDouble(vector_in.z()));
    return array;
}

/*!
 * @brief       Deletes the oldest SGraph archives beyond the retain limit.
 *
 * @param[in]   archiveDir_in
 *              Directory holding `sgraph_<sec>_<nsec>.json` files.
 *
 * @param[in]   maxFiles_in
 *              Maximum number of files to retain (positive by contract).
 */
void pruneSgraphArchives(const std::filesystem::path &archiveDir_in,
                         const int                    maxFiles_in)
{
    std::error_code          error;
    std::vector<std::string> fileNames;
    for (const std::filesystem::directory_entry &entry :
         std::filesystem::directory_iterator(archiveDir_in, error))
    {
        if (error)
        {
            break;
        }
        if (!entry.is_regular_file(error) || error)
        {
            continue;
        }
        const std::string fileName  = entry.path().filename().string();
        const bool        isArchive = fileName.rfind("sgraph_", 0U) == 0U &&
                               entry.path().extension() == ".json";
        if (isArchive)
        {
            fileNames.push_back(fileName);
        }
    }
    if (error)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot prune SGraph archives in '%s'.",
                    archiveDir_in.string().c_str());
        return;
    }

    /* File names embed zero-padded sim stamps, so lexical order is oldest
     * first. */
    std::sort(fileNames.begin(), fileNames.end());
    while (static_cast<int>(fileNames.size()) > maxFiles_in)
    {
        std::error_code removeError;
        std::filesystem::remove(archiveDir_in / fileNames.front(), removeError);
        if (removeError)
        {
            RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                        "Cannot remove old SGraph archive '%s'.",
                        fileNames.front().c_str());
            break;
        }
        fileNames.erase(fileNames.begin());
    }
}
} // namespace

void maybeArchiveSGraph(
    const std::vector<vs_graphs::core::semantic::Floor *>   &mappedFloors_in,
    const std::vector<vs_graphs::core::semantic::Room *>    &mappedRooms_in,
    const std::vector<vs_graphs::core::semantic::Passage *> &mappedPassages_in,
    const rclcpp::Time                                      &msgTime_s_in)
{
    static double        lastArchiveTime_s = -1.0e100;
    static std::uint64_t captureCycle      = 0U;
    static bool          warnedMissingDir  = false;

    if (!sgraphArchiveEnabled)
    {
        return;
    }

    if (sgraphArchiveTestRunDir.empty())
    {
        if (!warnedMissingDir)
        {
            RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                        "SGraph JSON archiving is disabled: the "
                        "'test_run_dir' parameter is empty.");
            warnedMissingDir = true;
        }
        return;
    }

    if (!std::isfinite(msgTime_s_in.seconds()))
    {
        return;
    }

    const double now_s          = msgTime_s_in.seconds();
    const bool   isFirstCapture = (captureCycle == 0U);
    if (!isFirstCapture && sgraphArchiveIntervalSec > 0.0 &&
        (now_s - lastArchiveTime_s) < sgraphArchiveIntervalSec)
    {
        return;
    }

    /* The file name and metadata use the ROS sim clock, never wall time. */
    const std::int64_t totalNanoseconds = msgTime_s_in.nanoseconds();
    if (totalNanoseconds < 0)
    {
        return;
    }
    const std::int64_t stampSeconds     = totalNanoseconds / 1000000000LL;
    const std::int64_t stampNanoseconds = totalNanoseconds % 1000000000LL;

    /* Enumerate every active Atlas map coherently; each becomes one mapN
     * entry keyed by 0-based ordinal. Falls back to the caller-provided
     * current-map snapshot when the Atlas is unavailable. */
    struct SgraphMapInput
    {
        long                                              mapId{-1};
        bool                                              isActive{true};
        std::uint64_t                                     worldFrameEpoch{0U};
        std::vector<vs_graphs::core::semantic::Floor *>   floorsRaw;
        std::vector<vs_graphs::core::semantic::Room *>    roomsRaw;
        std::vector<vs_graphs::core::semantic::Passage *> passagesRaw;
    };
    std::vector<SgraphMapInput> mapInputs;
    vs_graphs::core::Atlas     *p_slamSystemAtlas = nullptr;
    if (((p_slamSystem != nullptr)) &&
        p_slamSystem->getAtlas(p_slamSystemAtlas) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAtlas returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    vs_graphs::core::Atlas *p_atlas =
        (p_slamSystem != nullptr) ? p_slamSystemAtlas : nullptr;
    if (p_atlas != nullptr)
    {
        std::optional<long unsigned int>       currentMapId;
        vs_graphs::core::AtlasCurrentMapStatus mapStatus;
        std::vector<vs_graphs::core::Map *>    atlasMaps{};
        if (p_atlas->getCoherentMapView(currentMapId, mapStatus, atlasMaps) !=
            vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCoherentMapView returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::sort(
            atlasMaps.begin(),
            atlasMaps.end(),
            [](vs_graphs::core::Map *first_in, vs_graphs::core::Map *second_in)
            {
                unsigned long firstId{};
                if (first_in->getId(firstId) !=
                    vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                unsigned long secondId{};
                if (second_in->getId(secondId) !=
                    vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return firstId < secondId;
            });
        for (vs_graphs::core::Map *p_map : atlasMaps)
        {
            bool mapIsBad{};
            if (!(p_map == nullptr) &&
                p_map->isBad(mapIsBad) !=
                    vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map == nullptr || mapIsBad)
            {
                continue;
            }
            SgraphMapInput mapInput;
            unsigned long  mapId2{};
            if (p_map->getId(mapId2) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            mapInput.mapId = static_cast<long>(mapId2);
            unsigned long mapId3{};
            if ((currentMapId.has_value()) &&
                p_map->getId(mapId3) !=
                    vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            mapInput.isActive =
                currentMapId.has_value() && (*currentMapId == mapId3);
            std::uint64_t mapWorldFrameEpoch{};
            if (p_map->getWorldFrameEpoch(mapWorldFrameEpoch) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldFrameEpoch returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapInput.worldFrameEpoch = mapWorldFrameEpoch;
            std::vector<vs_graphs::core::semantic::Floor *> mapAllFloors{};
            if (p_map->getAllFloors(mapAllFloors) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllFloors returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapInput.floorsRaw = mapAllFloors;
            std::vector<vs_graphs::core::semantic::Room *> mapAllRooms{};
            if (p_map->getAllRooms(mapAllRooms) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapInput.roomsRaw = mapAllRooms;
            std::vector<vs_graphs::core::semantic::Passage *> mapAllPassages{};
            if (p_map->getAllPassages(mapAllPassages) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapInput.passagesRaw = mapAllPassages;
            mapInputs.push_back(std::move(mapInput));
        }
    }
    if (mapInputs.empty())
    {
        SgraphMapInput fallbackInput;
        fallbackInput.floorsRaw   = mappedFloors_in;
        fallbackInput.roomsRaw    = mappedRooms_in;
        fallbackInput.passagesRaw = mappedPassages_in;
        mapInputs.push_back(std::move(fallbackInput));
    }

    Json        archive;
    std::size_t totalFloors   = 0U;
    std::size_t totalRooms    = 0U;
    std::size_t totalPassages = 0U;

    for (std::size_t mapOrdinal = 0U; mapOrdinal < mapInputs.size();
         ++mapOrdinal)
    {
        const SgraphMapInput &mapInput = mapInputs[mapOrdinal];

        /* Keep only live elements, sorted by raw atlas id for determinism. */
        std::vector<vs_graphs::core::semantic::Floor *> floors;
        for (vs_graphs::core::semantic::Floor *p_floor : mapInput.floorsRaw)
        {
            if (p_floor != nullptr)
            {
                floors.push_back(p_floor);
            }
        }
        std::sort(floors.begin(),
                  floors.end(),
                  [](vs_graphs::core::semantic::Floor *first_in,
                     vs_graphs::core::semantic::Floor *second_in)
                  {
                      int first_inId{};
                      if (first_in->getId(first_inId) !=
                          vs_graphs::core::semantic::FloorStatus::
                              FLOOR_STATUS_SUCCESS)
                      {
                          RCLCPP_ERROR(
                              rclcpp::get_logger("vs_graphs"),
                              "%s: getId returned a failure status although it "
                              "cannot fail; continuing as before.",
                              __func__);
                      }
                      int second_inId{};
                      if (second_in->getId(second_inId) !=
                          vs_graphs::core::semantic::FloorStatus::
                              FLOOR_STATUS_SUCCESS)
                      {
                          RCLCPP_ERROR(
                              rclcpp::get_logger("vs_graphs"),
                              "%s: getId returned a failure status although it "
                              "cannot fail; continuing as before.",
                              __func__);
                      }
                      return first_inId < second_inId;
                  });

        std::vector<vs_graphs::core::semantic::Room *> rooms;
        for (vs_graphs::core::semantic::Room *p_room : mapInput.roomsRaw)
        {
            bool roomIsBad{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad)
            {
                rooms.push_back(p_room);
            }
        }
        std::sort(
            rooms.begin(),
            rooms.end(),
            [](vs_graphs::core::semantic::Room *first_in,
               vs_graphs::core::semantic::Room *second_in)
            {
                int first_inId{};
                if (first_in->getId(first_inId) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int second_inId{};
                if (second_in->getId(second_inId) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return first_inId < second_inId;
            });

        std::vector<vs_graphs::core::semantic::Passage *> passages;
        for (vs_graphs::core::semantic::Passage *p_passage :
             mapInput.passagesRaw)
        {
            bool passageIsBad{};
            if ((p_passage != nullptr) &&
                p_passage->isBad(passageIsBad) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage != nullptr && !passageIsBad)
            {
                passages.push_back(p_passage);
            }
        }
        std::sort(passages.begin(),
                  passages.end(),
                  [](vs_graphs::core::semantic::Passage *first_in,
                     vs_graphs::core::semantic::Passage *second_in)
                  {
                      int first_inId{};
                      if (first_in->getId(first_inId) !=
                          vs_graphs::core::semantic::PassageStatus::
                              PASSAGE_STATUS_SUCCESS)
                      {
                          RCLCPP_ERROR(
                              rclcpp::get_logger("vs_graphs"),
                              "%s: getId returned a failure status although it "
                              "cannot fail; continuing as before.",
                              __func__);
                      }
                      int second_inId{};
                      if (second_in->getId(second_inId) !=
                          vs_graphs::core::semantic::PassageStatus::
                              PASSAGE_STATUS_SUCCESS)
                      {
                          RCLCPP_ERROR(
                              rclcpp::get_logger("vs_graphs"),
                              "%s: getId returned a failure status although it "
                              "cannot fail; continuing as before.",
                              __func__);
                      }
                      return first_inId < second_inId;
                  });

        /* Group rooms by floor. Index zero is synthetic when no floor exists so
         * rooms and passages are still archived early in a run. */
        const bool        hasFloors  = !floors.empty();
        const std::size_t floorCount = hasFloors ? floors.size() : 1U;
        std::vector<std::vector<vs_graphs::core::semantic::Room *>> floorRooms(
            floorCount);
        std::unordered_map<const vs_graphs::core::semantic::Room *, std::size_t>
            roomFloorIndex;

        for (std::size_t floorIndex = 0U; floorIndex < floors.size();
             ++floorIndex)
        {
            std::vector<vs_graphs::core::semantic::Room *> rooms2{};
            if (floors[floorIndex]->getRooms(rooms2) !=
                vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRooms returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::semantic::Room *p_room : rooms2)
            {
                bool roomIsBad2{};
                if (!(p_room == nullptr) &&
                    p_room->isBad(roomIsBad2) !=
                        vs_graphs::core::semantic::RoomStatus::
                            ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room == nullptr || roomIsBad2)
                {
                    continue;
                }
                if (roomFloorIndex.find(p_room) == roomFloorIndex.end())
                {
                    roomFloorIndex[p_room] = floorIndex;
                }
            }
        }

        for (vs_graphs::core::semantic::Room *p_room : rooms)
        {
            std::size_t                                     floorIndex = 0U;
            const std::unordered_map<const vs_graphs::core::semantic::Room *,
                                     std::size_t>::iterator claimed =
                roomFloorIndex.find(p_room);
            if (claimed != roomFloorIndex.end())
            {
                floorIndex = claimed->second;
            }
            else
            {
                vs_graphs::core::semantic::Floor *p_floor = nullptr;
                if (p_room->getFloor(p_floor) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getFloor returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (std::size_t candidate = 0U; candidate < floors.size();
                     ++candidate)
                {
                    if (floors[candidate] == p_floor)
                    {
                        floorIndex = candidate;
                        break;
                    }
                }
            }
            floorRooms[floorIndex].push_back(p_room);
            roomFloorIndex[p_room] = floorIndex;
        }

        /* Zero-based room keys are scoped to their floor. */
        std::unordered_map<const vs_graphs::core::semantic::Room *, std::string>
            roomKeys;
        for (std::vector<vs_graphs::core::semantic::Room *> &floorRoomList :
             floorRooms)
        {
            for (std::size_t roomOrdinal = 0U;
                 roomOrdinal < floorRoomList.size();
                 ++roomOrdinal)
            {
                roomKeys[floorRoomList[roomOrdinal]] =
                    "room" + std::to_string(roomOrdinal);
            }
        }

        /* Assign each passage to the floor holding most of its rooms; passages
         * without any room evidence fall back to the first floor. */
        std::vector<std::vector<vs_graphs::core::semantic::Passage *>>
            floorPassages(floorCount);
        std::unordered_map<const vs_graphs::core::semantic::Passage *,
                           std::vector<int>>
            passageWallIds;
        for (vs_graphs::core::semantic::Passage *p_passage : passages)
        {
            std::vector<int> wallIds;
            std::vector<vs_graphs::core::geometric::Plane *>
                passageAssociateWalls{};
            if (p_passage->getAssociateWalls(passageAssociateWalls) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::geometric::Plane *p_wall :
                 passageAssociateWalls)
            {
                if (p_wall != nullptr)
                {
                    int wallGetId{};
                    if (p_wall->getId(wallGetId) !=
                        vs_graphs::core::geometric::PlaneStatus::
                            PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    wallIds.push_back(wallGetId);
                }
            }
            passageWallIds[p_passage] = wallIds;

            std::optional<int> prospectiveRoomId{};
            if (p_passage->getProspectiveRoomId(prospectiveRoomId) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getProspectiveRoomId returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            vs_graphs::core::semantic::Room *p_prospectiveRoom = nullptr;
            if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            std::vector<std::size_t> votes(floorCount, 0U);
            for (vs_graphs::core::semantic::Room *p_room : rooms)
            {
                bool ownsAssociatedWall = false;
                std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
                if (p_room->getWalls(roomWalls) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (vs_graphs::core::geometric::Plane *p_wall : roomWalls)
                {
                    if (p_wall == nullptr)
                    {
                        continue;
                    }
                    for (const int wallId : wallIds)
                    {
                        int wallGetId2{};
                        if (p_wall->getId(wallGetId2) !=
                            vs_graphs::core::geometric::PlaneStatus::
                                PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        if (wallGetId2 == wallId)
                        {
                            ownsAssociatedWall = true;
                            break;
                        }
                    }
                    if (ownsAssociatedWall)
                    {
                        break;
                    }
                }
                int roomId{};
                if ((!ownsAssociatedWall) && (prospectiveRoomId.has_value()) &&
                    p_room->getId(roomId) !=
                        vs_graphs::core::semantic::RoomStatus::
                            ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!ownsAssociatedWall &&
                    ((prospectiveRoomId.has_value() &&
                      roomId == prospectiveRoomId.value()) ||
                     (p_prospectiveRoom != nullptr &&
                      p_room == p_prospectiveRoom)))
                {
                    ownsAssociatedWall = true;
                }
                if (ownsAssociatedWall)
                {
                    votes[roomFloorIndex[p_room]]++;
                }
            }

            std::size_t bestFloor = 0U;
            for (std::size_t candidate = 1U; candidate < floorCount;
                 ++candidate)
            {
                if (votes[candidate] > votes[bestFloor])
                {
                    bestFloor = candidate;
                }
            }
            floorPassages[bestFloor].push_back(p_passage);
        }

        /* Index passage keys per floor so walls can list their passages. */
        std::vector<std::unordered_map<int, std::vector<std::string>>>
            floorWallPassages(floorCount);
        std::vector<
            std::unordered_map<const vs_graphs::core::semantic::Passage *,
                               std::string>>
            floorPassageKeys(floorCount);
        for (std::size_t floorIndex = 0U; floorIndex < floorCount; ++floorIndex)
        {
            for (std::size_t passageOrdinal = 0U;
                 passageOrdinal < floorPassages[floorIndex].size();
                 ++passageOrdinal)
            {
                vs_graphs::core::semantic::Passage *p_passage =
                    floorPassages[floorIndex][passageOrdinal];
                const std::string passageKey =
                    "passage" + std::to_string(passageOrdinal);
                floorPassageKeys[floorIndex][p_passage] = passageKey;
                for (const int wallId : passageWallIds[p_passage])
                {
                    floorWallPassages[floorIndex][wallId].push_back(passageKey);
                }
            }
        }

        Json mapJson;
        mapJson["map_id"]            = mapInput.mapId;
        mapJson["is_active"]         = mapInput.isActive;
        mapJson["world_frame_epoch"] = mapInput.worldFrameEpoch;

        for (std::size_t floorIndex = 0U; floorIndex < floorCount; ++floorIndex)
        {
            Json                              floorJson;
            vs_graphs::core::semantic::Floor *p_floor =
                hasFloors ? floors[floorIndex] : nullptr;
            if (p_floor != nullptr)
            {
                int floorId{};
                if (p_floor->getId(floorId) !=
                    vs_graphs::core::semantic::FloorStatus::
                        FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                floorJson["floor_id"] = floorId;
                std::string floorName{};
                if (p_floor->getName(floorName) !=
                    vs_graphs::core::semantic::FloorStatus::
                        FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getName returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                floorJson["floor_name"] = floorName;
                Eigen::Vector3d floorCentroid{};
                if (p_floor->getCentroid(floorCentroid) !=
                    vs_graphs::core::semantic::FloorStatus::
                        FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                floorJson["centroid"] = archiveVector3(floorCentroid);
                std::optional<vs_graphs::core::semantic::Floor::PlaneIdentity>
                    identity{};
                if (p_floor->getPlaneIdentity(identity) !=
                    vs_graphs::core::semantic::FloorStatus::
                        FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPlaneIdentity returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                floorJson["has_plane_identity"] = identity.has_value();
                if (identity.has_value())
                {
                    const Eigen::Vector3d normal_World_m =
                        identity->equation_World.head<3>();
                    floorJson["normal"] = archiveVector3(normal_World_m);
                    floorJson["offset_d"] =
                        sanitiseArchiveDouble(identity->equation_World[3]);
                    floorJson["finite_support_count"] =
                        identity->finiteSupportCount;
                    floorJson["observation_count"] = identity->observationCount;
                }
            }
            else
            {
                floorJson["floor_id"]   = -1;
                floorJson["floor_name"] = "synthetic";
            }

            for (std::size_t roomOrdinal = 0U;
                 roomOrdinal < floorRooms[floorIndex].size();
                 ++roomOrdinal)
            {
                vs_graphs::core::semantic::Room *p_room =
                    floorRooms[floorIndex][roomOrdinal];
                const std::string roomKey =
                    "room" + std::to_string(roomOrdinal);

                Json roomJson;
                int  roomId2{};
                if (p_room->getId(roomId2) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                roomJson["room_id"] = roomId2;
                std::string roomName{};
                if (p_room->getName(roomName) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getName returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                roomJson["room_name"] = roomName;
                vs_graphs::core::semantic::Room::RoomVariant roomVariant{};
                if (p_room->getRoomVariant(roomVariant) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                roomJson["room_variant"] = static_cast<int>(roomVariant);
                bool roomHasPreviouslyVisited{};
                if (p_room->hasPreviouslyVisited(roomHasPreviouslyVisited) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasPreviouslyVisited returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                roomJson["room_visited"] = roomHasPreviouslyVisited;
                Eigen::Vector3d roomCentroid{};
                if (p_room->getCentroid(roomCentroid) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                roomJson["centroid"] = archiveVector3(roomCentroid);

                std::vector<vs_graphs::core::geometric::Plane *> walls;
                std::vector<vs_graphs::core::geometric::Plane *> roomWalls2{};
                if (p_room->getWalls(roomWalls2) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (vs_graphs::core::geometric::Plane *p_wall : roomWalls2)
                {
                    bool wallIsBad{};
                    if (!(p_wall == nullptr) &&
                        p_wall->isBad(wallIsBad) !=
                            vs_graphs::core::geometric::PlaneStatus::
                                PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_wall == nullptr || wallIsBad)
                    {
                        continue;
                    }
                    vs_graphs::core::geometric::Plane::PlaneVariant
                        wallPlaneType{};
                    if (p_wall->getPlaneType(wallPlaneType) !=
                        vs_graphs::core::geometric::PlaneStatus::
                            PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPlaneType returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (wallPlaneType !=
                        vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
                    {
                        continue;
                    }
                    walls.push_back(p_wall);
                }
                std::sort(walls.begin(),
                          walls.end(),
                          [](vs_graphs::core::geometric::Plane *first_in,
                             vs_graphs::core::geometric::Plane *second_in)
                          {
                              int firstGetId{};
                              if (first_in->getId(firstGetId) !=
                                  vs_graphs::core::geometric::PlaneStatus::
                                      PLANE_STATUS_SUCCESS)
                              {
                                  RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                               "%s: getId returned a failure "
                                               "status although it cannot "
                                               "fail; continuing as before.",
                                               __func__);
                              }
                              int secondGetId{};
                              if (second_in->getId(secondGetId) !=
                                  vs_graphs::core::geometric::PlaneStatus::
                                      PLANE_STATUS_SUCCESS)
                              {
                                  RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                               "%s: getId returned a failure "
                                               "status although it cannot "
                                               "fail; continuing as before.",
                                               __func__);
                              }
                              return firstGetId < secondGetId;
                          });

                for (std::size_t wallOrdinal = 0U; wallOrdinal < walls.size();
                     ++wallOrdinal)
                {
                    vs_graphs::core::geometric::Plane *p_wall =
                        walls[wallOrdinal];
                    const std::string wallKey =
                        "wall" + std::to_string(wallOrdinal);
                    vs_graphs::core::geometric::PlaneGeometryMetadataSnapshot
                        geometry{};
                    if (p_wall->getGeometryMetadataSnapshot(geometry) !=
                        vs_graphs::core::geometric::PlaneStatus::
                            PLANE_STATUS_SUCCESS)
                    {
                        // getGeometryMetadataSnapshot cannot fail; continue as
                        // before.
                    }
                    g2o::Plane3D equation_World{};
                    if (p_wall->getGlobalEquation(equation_World) !=
                        vs_graphs::core::geometric::PlaneStatus::
                            PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGlobalEquation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector3d normal_World_m(
                        equation_World.normal().x(),
                        equation_World.normal().y(),
                        equation_World.normal().z());

                    Json wallJson;
                    int  wallGetId3{};
                    if (p_wall->getId(wallGetId3) !=
                        vs_graphs::core::geometric::PlaneStatus::
                            PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    wallJson["wall_id"] = wallGetId3;
                    wallJson["wall_centroid"] =
                        archiveVector3(geometry.centroid_World_m);
                    Json wallLimits;
                    wallLimits["min_u_m"] =
                        sanitiseArchiveDouble(geometry.minPlaneU_m);
                    wallLimits["max_u_m"] =
                        sanitiseArchiveDouble(geometry.maxPlaneU_m);
                    wallLimits["min_v_m"] =
                        sanitiseArchiveDouble(geometry.minPlaneV_m);
                    wallLimits["max_v_m"] =
                        sanitiseArchiveDouble(geometry.maxPlaneV_m);
                    wallLimits["finite_support_count"] =
                        geometry.finiteSupportCount;
                    wallLimits["observation_count"] = geometry.observationCount;
                    wallJson["wall_limits"]         = std::move(wallLimits);
                    wallJson["wall_normal"] = archiveVector3(normal_World_m);
                    wallJson["wall_offset_d"] =
                        sanitiseArchiveDouble(equation_World.coeffs()[3]);
                    wallJson["wall_extent_u_m"] = sanitiseArchiveDouble(
                        geometry.maxPlaneU_m - geometry.minPlaneU_m);
                    wallJson["wall_extent_v_m"] = sanitiseArchiveDouble(
                        geometry.maxPlaneV_m - geometry.minPlaneV_m);
                    wallJson["parent_room"] = roomKey;

                    Json passageList = Json::array();
                    int  wallGetId4{};
                    if (p_wall->getId(wallGetId4) !=
                        vs_graphs::core::geometric::PlaneStatus::
                            PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    const std::unordered_map<int,
                                             std::vector<std::string>>::iterator
                        wallPassages =
                            floorWallPassages[floorIndex].find(wallGetId4);
                    if (wallPassages != floorWallPassages[floorIndex].end())
                    {
                        for (const std::string &passageKey :
                             wallPassages->second)
                        {
                            passageList.push_back(passageKey);
                        }
                    }
                    wallJson["passage_list"] = std::move(passageList);
                    roomJson[wallKey]        = std::move(wallJson);
                }
                floorJson[roomKey] = std::move(roomJson);
            }

            Json passagesJson = Json::object();
            for (std::size_t passageOrdinal = 0U;
                 passageOrdinal < floorPassages[floorIndex].size();
                 ++passageOrdinal)
            {
                vs_graphs::core::semantic::Passage *p_passage =
                    floorPassages[floorIndex][passageOrdinal];
                const std::string passageKey =
                    "passage" + std::to_string(passageOrdinal);
                const std::vector<int> &wallIds = passageWallIds[p_passage];
                std::optional<int>      prospectiveRoomId{};
                if (p_passage->getProspectiveRoomId(prospectiveRoomId) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoomId returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                vs_graphs::core::semantic::Room *p_prospectiveRoom = nullptr;
                if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                Json passageJson;
                int  passageId{};
                if (p_passage->getId(passageId) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passageJson["passage_id"] = passageId;
                Eigen::Vector3d passageCentroid{};
                if (p_passage->getCentroid(passageCentroid) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passageJson["centroid"] = archiveVector3(passageCentroid);
                double passageWidth{};
                if (p_passage->getWidth(passageWidth) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWidth returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passageJson["width_m"] = sanitiseArchiveDouble(passageWidth);
                double passageHeight{};
                if (p_passage->getHeight(passageHeight) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getHeight returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passageJson["height_m"] = sanitiseArchiveDouble(passageHeight);
                bool passageIsPassable{};
                if (p_passage->isPassable(passageIsPassable) !=
                    vs_graphs::core::semantic::PassageStatus::
                        PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isPassable returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passageJson["passable"] = passageIsPassable;

                Json connects = Json::array();
                for (std::size_t roomOrdinal = 0U;
                     roomOrdinal < floorRooms[floorIndex].size();
                     ++roomOrdinal)
                {
                    vs_graphs::core::semantic::Room *p_room =
                        floorRooms[floorIndex][roomOrdinal];
                    bool connected = false;
                    std::vector<vs_graphs::core::geometric::Plane *>
                        roomWalls3{};
                    if (p_room->getWalls(roomWalls3) !=
                        vs_graphs::core::semantic::RoomStatus::
                            ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWalls returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    for (vs_graphs::core::geometric::Plane *p_wall : roomWalls3)
                    {
                        if (p_wall == nullptr)
                        {
                            continue;
                        }
                        for (const int wallId : wallIds)
                        {
                            int wallGetId5{};
                            if (p_wall->getId(wallGetId5) !=
                                vs_graphs::core::geometric::PlaneStatus::
                                    PLANE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getId returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            if (wallGetId5 == wallId)
                            {
                                connected = true;
                                break;
                            }
                        }
                        if (connected)
                        {
                            break;
                        }
                    }
                    int roomId3{};
                    if ((!connected) && (prospectiveRoomId.has_value()) &&
                        p_room->getId(roomId3) !=
                            vs_graphs::core::semantic::RoomStatus::
                                ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!connected && ((prospectiveRoomId.has_value() &&
                                        roomId3 == prospectiveRoomId.value()) ||
                                       (p_prospectiveRoom != nullptr &&
                                        p_room == p_prospectiveRoom)))
                    {
                        connected = true;
                    }
                    if (connected)
                    {
                        connects.push_back("room" +
                                           std::to_string(roomOrdinal));
                    }
                }
                passageJson["connects"] = std::move(connects);

                Json associatedWalls = Json::array();
                for (const int wallId : wallIds)
                {
                    associatedWalls.push_back(wallId);
                }
                passageJson["associated_wall_ids"] = std::move(associatedWalls);
                passagesJson[passageKey]           = std::move(passageJson);
            }
            floorJson["passages"] = std::move(passagesJson);

            mapJson["floor" + std::to_string(floorIndex)] =
                std::move(floorJson);
        }

        archive["map" + std::to_string(mapOrdinal)] = std::move(mapJson);
        totalFloors += floors.size();
        totalRooms += rooms.size();
        totalPassages += passages.size();
    }

    Json metadata;
    metadata["schema_version"]        = 2;
    metadata["sim_timestamp_sec"]     = stampSeconds;
    metadata["sim_timestamp_nanosec"] = stampNanoseconds;
    metadata["ros_node"]              = "vs_graphs";
    metadata["capture_cycle"]         = captureCycle;
    metadata["sensor_mode"]           = sensorModeName();
    metadata["map_count"]             = mapInputs.size();
    metadata["floor_count_total"]     = totalFloors;
    metadata["room_count_total"]      = totalRooms;
    metadata["passage_count_total"]   = totalPassages;
    archive["metadata"]               = std::move(metadata);

    namespace filesystem = std::filesystem;
    const filesystem::path archiveDir =
        filesystem::path(sgraphArchiveTestRunDir) / "output" / "sgraph";
    std::error_code makeError;
    filesystem::create_directories(archiveDir, makeError);
    if (makeError)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot create SGraph archive directory '%s'.",
                    archiveDir.string().c_str());
        return;
    }

    /* Fixed-width zero-padded stamp: sgraph_<sec:06>_<nsec:09>.json, so
     * lexical file order matches chronological order. */
    std::ostringstream fileName;
    fileName << "sgraph_" << std::setfill('0') << std::setw(6) << stampSeconds
             << "_" << std::setfill('0') << std::setw(9) << stampNanoseconds
             << ".json";
    const filesystem::path archiveFile = archiveDir / fileName.str();
    std::ofstream          output(archiveFile);
    if (!output.is_open())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot write SGraph archive '%s'.",
                    archiveFile.string().c_str());
        return;
    }
    output << archive.dump(2);
    output.close();

    /* Mirror the latest snapshot into archive/ (the live-UAV-RAM
     * equivalent); the full time series stays in output/sgraph/. */
    const filesystem::path liveArchiveDir =
        filesystem::path(sgraphArchiveTestRunDir) / "archive";
    filesystem::create_directories(liveArchiveDir, makeError);
    if (!makeError)
    {
        const filesystem::path liveFile = liveArchiveDir / "sgraph_latest.json";
        const filesystem::path stagingFile =
            liveArchiveDir / "sgraph_latest.json.tmp";
        std::ofstream liveOutput(stagingFile);
        if (liveOutput.is_open())
        {
            liveOutput << archive.dump(2);
            liveOutput.close();
            filesystem::rename(stagingFile, liveFile, makeError);
            if (makeError)
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                            "Cannot publish latest SGraph to '%s'.",
                            liveFile.string().c_str());
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                "Archived SGraph #%llu to '%s' (%zu maps, %zu floors, "
                "%zu rooms, %zu passages).",
                static_cast<unsigned long long>(captureCycle),
                archiveFile.string().c_str(),
                mapInputs.size(),
                totalFloors,
                totalRooms,
                totalPassages);

    if (sgraphArchiveMaxFiles > 0)
    {
        pruneSgraphArchives(archiveDir, sgraphArchiveMaxFiles);
    }

    lastArchiveTime_s = now_s;
    ++captureCycle;
}

void publishAllPoints(std::vector<vs_graphs::core::MapPoint *> allMapPoints_in,
                      rclcpp::Time                             msgTime_s_in)
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

    cameraMarker.color.a = 0.7f;

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
    const std::vector<vs_graphs::core::semantic::Marker *> &fiducialMarkers_in,
    const rclcpp::Time                                     &msgTime_s_in)
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
    for (vs_graphs::core::semantic::Marker *fiducialMarker : fiducialMarkers_in)
    {
        /* Skip invalid marker pointers */
        if (fiducialMarker == nullptr)
        {
            continue;
        }

        /* Extract the globally expressed fiducial-marker pose */
        Sophus::SE3f T_world_fiducial_SE3f{};
        if (fiducialMarker->getGlobalPose(T_world_fiducial_SE3f) !=
            vs_graphs::core::semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

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
        int fiducialMarkerId{};
        if (fiducialMarker->getId(fiducialMarkerId) !=
            vs_graphs::core::semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        fiducialMarkerMessage.id     = fiducialMarkerId;
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
    observeVoxbloxInput(pointCloud_cameraMessage.width, msgTime_s_in);
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
    const std::vector<vs_graphs::core::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                             &msgTime_s_in)
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
    for (vs_graphs::core::KeyFrame *keyFrame : keyFrames_in)
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
        if (keyFrame->colorImg.empty())
        {
            continue;
        }

        /* Initialise the common ROS message header */
        std_msgs::msg::Header messageHeader;

        messageHeader.stamp = msgTime_s_in;

        messageHeader.frame_id = frameWorld;

        /* Convert the OpenCV keyframe image into a ROS image message */
        const sensor_msgs::msg::Image::SharedPtr keyFrameImageMessage =
            cv_bridge::CvImage(messageHeader, "bgr8", keyFrame->colorImg)
                .toImageMsg();

        /* Confirm that the image conversion succeeded */
        if (keyFrameImageMessage == nullptr)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("visual_sgraphs"),
                "Failed to convert KeyFrame#%lu image into a ROS message.",
                static_cast<unsigned long>(keyFrame->id));

            continue;
        }

        /* Create the persistent keyframe identifier message */
        std_msgs::msg::UInt64 keyFrameIdMessage;

        keyFrameIdMessage.data = keyFrame->id;

        /* Package the keyframe identifier and image for segmentation */
        segmenter_ros::msg::VSGraphDataMsg segmentationInputMessage;

        segmentationInputMessage.header          = messageHeader;
        segmentationInputMessage.key_frame_id    = keyFrameIdMessage;
        segmentationInputMessage.key_frame_image = *keyFrameImageMessage;

        /* Publish the keyframe image for semantic segmentation */
        pubKFImage->publish(segmentationInputMessage);

        /* Prevent the same keyframe from being published again */
        keyFrame->isPublished = true;

        /* Mark the keyframe as in flight for the lockstep backlog signal */
        if (p_slamSystem != nullptr)
        {
            if (p_slamSystem->incrementSegmentationPublishedCount() !=
                vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: incrementSegmentationPublishedCount returned "
                             "a failure status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
        }
    }
}

void publishKeyFrameMarkers(
    const std::vector<vs_graphs::core::KeyFrame *> &keyFrames_in,
    const rclcpp::Time                             &msgTime_s_in)
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
    std::vector<vs_graphs::core::KeyFrame *> orderedKeyFrames;

    orderedKeyFrames.reserve(keyFrames_in.size());

    /* Remove invalid keyframes before sorting */
    for (vs_graphs::core::KeyFrame *keyFrame : keyFrames_in)
    {
        bool keyFrameIsBad{};
        if (!(keyFrame == nullptr) &&
            keyFrame->isBad(keyFrameIsBad) !=
                vs_graphs::core::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (keyFrame == nullptr || keyFrameIsBad)
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
              vs_graphs::core::KeyFrame::lId);

    /* Initialise the keyframe-position marker */
    visualization_msgs::msg::Marker keyFramePositionMarker;

    keyFramePositionMarker.header.frame_id = frameWorld;
    keyFramePositionMarker.header.stamp    = msgTime_s_in;

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
    keyFramePathMessage.header.stamp    = msgTime_s_in;
    keyFramePathMessage.poses.reserve(orderedKeyFrames.size());

    /* Add every valid keyframe pose to the marker and path */
    for (vs_graphs::core::KeyFrame *keyFrame : orderedKeyFrames)
    {
        /* Obtain the globally expressed keyframe pose */
        Sophus::SE3f T_world_keyFrame_SE3f{};
        if (p_slamSystem->getKeyFramePose(keyFrame, T_world_keyFrame_SE3f) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKeyFramePose returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

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
            rclcpp::Time(static_cast<std::int64_t>(keyFrame->timeStamp * 1e9));

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

void publishPlanes(
    const std::vector<vs_graphs::core::geometric::Plane *> &mappedPlanes_in,
    const std::vector<vs_graphs::core::semantic::Room *>   &mappedRooms_in,
    const rclcpp::Time                                     &msgTime_s_in,
    vs_graphs::observability::PublishTopicsTimingSink      *p_timingSink_in);

void publishPlanes(
    const std::vector<vs_graphs::core::geometric::Plane *> &mappedPlanes_in,
    const std::vector<vs_graphs::core::semantic::Room *>   &mappedRooms_in,
    const rclcpp::Time                                     &msgTime_s_in)
{
    publishPlanes(mappedPlanes_in, mappedRooms_in, msgTime_s_in, nullptr);
}

void publishPlanes(
    const std::vector<vs_graphs::core::geometric::Plane *> &mappedPlanes_in,
    const std::vector<vs_graphs::core::semantic::Room *>   &mappedRooms_in,
    const rclcpp::Time                                     &msgTime_s_in,
    vs_graphs::observability::PublishTopicsTimingSink      *p_timingSink_in)
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

    const std::chrono::steady_clock::time_point planePublishStart =
        std::chrono::steady_clock::now();

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

    for (vs_graphs::core::geometric::Plane *p_mappedPlane : mappedPlanes_in)
    {
        bool mappedPlaneIsBad{};
        if (!(p_mappedPlane == nullptr) &&
            p_mappedPlane->isBad(mappedPlaneIsBad) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::geometric::Plane::PlaneVariant mappedPlanePlaneType{};
        if (!(p_mappedPlane == nullptr || mappedPlaneIsBad) &&
            p_mappedPlane->getPlaneType(mappedPlanePlaneType) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mappedPlane == nullptr || mappedPlaneIsBad ||
            mappedPlanePlaneType !=
                vs_graphs::core::geometric::Plane::PlaneVariant::GROUND)
        {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mappedPlaneMapClouds{};
        if (p_mappedPlane->getMapClouds(mappedPlaneMapClouds) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapClouds returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_groundCloud_BC =
            mappedPlaneMapClouds;
        g2o::Plane3D mappedPlaneGetGlobalEquation{};
        if (p_mappedPlane->getGlobalEquation(mappedPlaneGetGlobalEquation) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d candidateGroundNormal_BC =
            mappedPlaneGetGlobalEquation.normal();

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

    /* Reverse plane -> owning room(s) map, built once per publish cycle, so
     * a plane's RViz label can state its ownership directly instead of
     * requiring a reader to trace a possibly-cluttered roomWallLine back to
     * its wall. */
    std::unordered_map<vs_graphs::core::geometric::Plane *, std::vector<int>>
        owningRoomIdsByPlane;
    for (vs_graphs::core::semantic::Room *p_mappedRoom : mappedRooms_in)
    {
        bool mappedRoomIsBad{};
        if (!(p_mappedRoom == nullptr) &&
            p_mappedRoom->isBad(mappedRoomIsBad) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mappedRoom == nullptr || mappedRoomIsBad)
        {
            continue;
        }
        std::vector<vs_graphs::core::geometric::Plane *> mappedRoomWalls{};
        if (p_mappedRoom->getWalls(mappedRoomWalls) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (vs_graphs::core::geometric::Plane *p_ownedWall : mappedRoomWalls)
        {
            if (p_ownedWall != nullptr)
            {
                int mappedRoomId{};
                if (p_mappedRoom->getId(mappedRoomId) !=
                    vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                owningRoomIdsByPlane[p_ownedWall].push_back(mappedRoomId);
            }
        }
    }

    /* Process every mapped plane */
    for (vs_graphs::core::geometric::Plane *mappedPlane : mappedPlanes_in)
    {
        /* Skip invalid planes */
        bool mappedPlaneIsBad2{};
        if (!(mappedPlane == nullptr) &&
            mappedPlane->isBad(mappedPlaneIsBad2) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mappedPlane == nullptr || mappedPlaneIsBad2)
        {
            continue;
        }

        /* Extract the semantic plane type once */
        vs_graphs::core::geometric::Plane::PlaneVariant planeType{};
        if (mappedPlane->getPlaneType(planeType) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Skip planes that have not received a semantic type */
        if (planeType ==
            vs_graphs::core::geometric::Plane::PlaneVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the plane point cloud */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloud_BC{};
        if (mappedPlane->getMapClouds(planePointCloud_BC) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapClouds returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Skip planes without any mapped points */
        if (planePointCloud_BC == nullptr || planePointCloud_BC->empty())
        {
            continue;
        }

        /* Extract and validate the plane centroid */
        Eigen::Vector3d planeCentroid_BC_m{};
        if (mappedPlane->getCentroid(planeCentroid_BC_m) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (!planeCentroid_BC_m.allFinite())
        {
            continue;
        }

        /* Extract and validate the plane normal */
        g2o::Plane3D mappedPlaneGetGlobalEquation2{};
        if (mappedPlane->getGlobalEquation(mappedPlaneGetGlobalEquation2) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3d planeNormal_BC = mappedPlaneGetGlobalEquation2.normal();

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

        if (planeType ==
                vs_graphs::core::geometric::Plane::PlaneVariant::WALL &&
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
        std::vector<std::uint8_t> configuredColour{};
        if (mappedPlane->getColor(configuredColour) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getColor returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

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

            /* The construction view shows camera colours: ownership state
             * stays on the plane labels, never repainted onto points. */

            buildingComponentPointCloud_BC.points.push_back(colouredPoint_BC);
        }

        /*
         * Use the persistent plane ID. The label and normal markers may share
         * the same ID because they use different namespaces.
         */
        int mappedPlaneGetId{};
        if (mappedPlane->getId(mappedPlaneGetId) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const int planeMarkerId = static_cast<int>(mappedPlaneGetId);

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
        int                mappedPlaneGetId2{};
        if (mappedPlane->getId(mappedPlaneGetId2) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        planeLabelText << "Plane#" << mappedPlaneGetId2;

        if (validWallDimensions)
        {
            planeLabelText << '\n'
                           << std::fixed << std::setprecision(2) << "W "
                           << wallWidth_m << " x H " << wallHeight_m << " m";
        }

        planeLabelText << '\n' << "N " << finiteSupportPointCount;

        const char *semanticClass = "OTHER";
        if (planeType == vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
        {
            semanticClass = "WALL";
        }
        else if (planeType ==
                 vs_graphs::core::geometric::Plane::PlaneVariant::GROUND)
        {
            semanticClass = "GROUND";
        }
        else if (planeType ==
                 vs_graphs::core::geometric::Plane::PlaneVariant::DOOR)
        {
            semanticClass = "DOOR";
        }

        {
            const std::unordered_map<vs_graphs::core::geometric::Plane *,
                                     std::vector<int>>::iterator ownerIt =
                owningRoomIdsByPlane.find(mappedPlane);
            planeLabelText << '\n' << "class=" << semanticClass;
            if (planeType ==
                vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
            {
                planeLabelText << " lifecycle="
                               << (ownerIt == owningRoomIdsByPlane.end() ||
                                           ownerIt->second.empty()
                                       ? "PENDING"
                                       : "COMMITTED");
            }
            planeLabelText << '\n' << "owner=";
            if (ownerIt == owningRoomIdsByPlane.end() ||
                ownerIt->second.empty())
            {
                planeLabelText << (planeType == vs_graphs::core::geometric::
                                                    Plane::PlaneVariant::WALL
                                       ? "PENDING"
                                       : "NONE");
            }
            else
            {
                for (std::size_t ownerIndex = 0U;
                     ownerIndex < ownerIt->second.size();
                     ++ownerIndex)
                {
                    if (ownerIndex > 0U)
                    {
                        planeLabelText << ',';
                    }
                    planeLabelText << "Room#" << ownerIt->second[ownerIndex];
                }
            }
        }

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

        /* ------------------------------------------------------------------ *
         * WALL TWIN-FACE LINK (axiom (e): a physical wall's two opposite
         * observations, linked by SemanticsManager::reconcileWallFacePairs())
         * ------------------------------------------------------------------ */

        vs_graphs::core::geometric::Plane *p_twinFace = nullptr;
        if (mappedPlane->getTwinFace(p_twinFace) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        bool twinFaceIsBad{};
        if ((p_twinFace != nullptr) &&
            p_twinFace->isBad(twinFaceIsBad) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int mappedPlaneGetId3{};
        if ((p_twinFace != nullptr && !twinFaceIsBad) &&
            mappedPlane->getId(mappedPlaneGetId3) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int twinFaceGetId{};
        if ((p_twinFace != nullptr && !twinFaceIsBad) &&
            p_twinFace->getId(twinFaceGetId) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_twinFace != nullptr && !twinFaceIsBad &&
            mappedPlaneGetId3 < twinFaceGetId)
        {
            Eigen::Vector3d twinCentroid_BC_m{};
            if (p_twinFace->getCentroid(twinCentroid_BC_m) !=
                vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (twinCentroid_BC_m.allFinite())
            {
                visualization_msgs::msg::Marker wallTwinLineMarker;

                wallTwinLineMarker.header.frame_id = frameBC;
                wallTwinLineMarker.header.stamp    = msgTime_s_in;

                wallTwinLineMarker.ns = "wallTwinLine";
                wallTwinLineMarker.id = planeMarkerId;

                wallTwinLineMarker.type =
                    visualization_msgs::msg::Marker::LINE_LIST;
                wallTwinLineMarker.action =
                    visualization_msgs::msg::Marker::ADD;

                wallTwinLineMarker.pose.orientation.w = 1.0;

                /* Thin, distinct from the room-ownership/floor lines. */
                wallTwinLineMarker.scale.x = 0.02;

                wallTwinLineMarker.color.r = 1.0F;
                wallTwinLineMarker.color.g = 0.85F;
                wallTwinLineMarker.color.b = 0.0F;
                wallTwinLineMarker.color.a = 0.8F;

                geometry_msgs::msg::Point firstFacePoint_BC;
                firstFacePoint_BC.x = planeCentroid_BC_m.x();
                firstFacePoint_BC.y = planeCentroid_BC_m.y();
                firstFacePoint_BC.z = planeCentroid_BC_m.z();

                geometry_msgs::msg::Point secondFacePoint_BC;
                secondFacePoint_BC.x = twinCentroid_BC_m.x();
                secondFacePoint_BC.y = twinCentroid_BC_m.y();
                secondFacePoint_BC.z = twinCentroid_BC_m.z();

                wallTwinLineMarker.points.reserve(2);
                wallTwinLineMarker.points.push_back(firstFacePoint_BC);
                wallTwinLineMarker.points.push_back(secondFacePoint_BC);

                wallTwinLineMarker.lifetime = rclcpp::Duration::from_seconds(0);

                planeVisualizationArray.markers.push_back(
                    std::move(wallTwinLineMarker));
            }
        }
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

    if (p_timingSink_in != nullptr && p_timingSink_in->callback != nullptr)
    {
        try
        {
            p_timingSink_in->callback(
                p_timingSink_in->p_context,
                vs_graphs::observability::PublishTopic::PLANES,
                true,
                planePublishStart,
                std::chrono::steady_clock::now());
        }
        catch (...)
        {
            /* Timing is diagnostic only and must never affect publication. */
        }
    }
}

void publishSegmentedCloud(
    const std::vector<vs_graphs::core::KeyFrame *> &keyFrames_in)
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
    vs_graphs::core::KeyFrame *selectedKeyFrame = nullptr;

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

        vs_graphs::core::KeyFrame *keyFrame = keyFrames_in[keyFrameIndex];

        if (keyFrame == nullptr)
        {
            continue;
        }

        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            candidateClassPointClouds{};
        if (keyFrame->getClsCloudPtrs(candidateClassPointClouds) !=
            vs_graphs::core::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getClsCloudPtrs returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

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
        vs_graphs::core::KeyFrame *olderKeyFrame = keyFrames_in[keyFrameIndex];

        if (olderKeyFrame != nullptr)
        {
            if (olderKeyFrame->clearClsClouds() !=
                vs_graphs::core::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearClsClouds returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
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
        if (selectedKeyFrame->clearClsClouds() !=
            vs_graphs::core::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearClsClouds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
    if (selectedKeyFrame->clearClsClouds() !=
        vs_graphs::core::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearClsClouds returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
    const std::vector<vs_graphs::core::semantic::Room *>    &mappedRooms_in,
    const std::vector<vs_graphs::core::semantic::Floor *>   &mappedFloors_in,
    const std::vector<vs_graphs::core::semantic::Passage *> &mappedPassages_in,
    const rclcpp::Time                                      &msgTime_s_in)
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
                      mappedPassages_in,
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

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pointCloud_source =
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

        std::shared_ptr<sensor_msgs::msg::PointCloud2> convertedMessage =
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
        std::shared_ptr<sensor_msgs::msg::PointCloud2> transformedMessage =
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
    publishTopics(msgTime_s_in,
                  angularVelocity_body_radps_in,
                  pointCloud_cameraMessage_in,
                  nullptr);
}

void publishTopics(
    const rclcpp::Time    &msgTime_s_in,
    const Eigen::Vector3f &angularVelocity_body_radps_in,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr
        &pointCloud_cameraMessage_in,
    vs_graphs::observability::PublishTopicsTimingSink *p_timingSink_in,
    const bool                                         publishAllPoints_in)
{
    const auto notifyTiming =
        [p_timingSink_in](const vs_graphs::observability::PublishTopic topic_in,
                          const bool isActualExecution_in,
                          const std::chrono::steady_clock::time_point start_in,
                          const std::chrono::steady_clock::time_point end_in)
    {
        if (p_timingSink_in == nullptr || p_timingSink_in->callback == nullptr)
        {
            return;
        }
        try
        {
            p_timingSink_in->callback(p_timingSink_in->p_context,
                                      topic_in,
                                      isActualExecution_in,
                                      start_in,
                                      end_in);
        }
        catch (...)
        {
            /* Timing is best-effort and must not alter publisher behaviour. */
        }
    };

    const auto publishTimed =
        [p_timingSink_in,
         &notifyTiming](const vs_graphs::observability::PublishTopic topic_in,
                        const auto &publishFunction_in)
    {
        if (p_timingSink_in == nullptr || p_timingSink_in->callback == nullptr)
        {
            publishFunction_in();
            return;
        }

        const std::chrono::steady_clock::time_point startTime =
            std::chrono::steady_clock::now();
        try
        {
            publishFunction_in();
        }
        catch (...)
        {
            notifyTiming(topic_in,
                         true,
                         startTime,
                         std::chrono::steady_clock::now());
            throw;
        }
        notifyTiming(topic_in,
                     true,
                     startTime,
                     std::chrono::steady_clock::now());
    };
    /* Confirm that the SLAM system has been initialised */
    if (p_slamSystem == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish topics: SLAM system is not initialised.");

        return;
    }

    /* Obtain the current camera pose relative to the world frame */
    Sophus::SE3f T_world_camera_SE3f{};
    if (p_slamSystem->getCamTwc(T_world_camera_SE3f) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCamTwc returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
        vs_graphs::core::Map *p_activeMap = nullptr;
        if (p_slamSystem->getCurrentMap(p_activeMap) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_activeMap != nullptr)
        {
            unsigned long activeMapId{};
            if (p_activeMap->getId(activeMapId) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            const std::uint64_t mapId = static_cast<std::uint64_t>(activeMapId);

            int mapChangeIndex{};
            if (p_activeMap->getLastBigChangeIndex(mapChangeIndex) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getLastBigChangeIndex returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

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

    std::vector<vs_graphs::core::KeyFrame *> mappedKeyFrames{};
    if (p_slamSystem->getAllKeyFrames(mappedKeyFrames) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<vs_graphs::core::semantic::Marker *> mappedFiducialMarkers{};
    if (p_slamSystem->getAllMarkers(mappedFiducialMarkers) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<vs_graphs::core::semantic::Room *> mappedRooms{};
    if (p_slamSystem->getAllRooms(mappedRooms) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<vs_graphs::core::semantic::Floor *> mappedFloors{};
    if (p_slamSystem->getAllFloors(mappedFloors) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<vs_graphs::core::semantic::Passage *> mappedPassages{};
    if (p_slamSystem->getAllPassages(mappedPassages) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<vs_graphs::core::geometric::Plane *> mappedPlanes{};
    if (p_slamSystem->getAllPlanes(mappedPlanes) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* ---------------------------------------------------------------------- *
     * KEYFRAMES, TRACKING, AND STRUCTURAL ELEMENTS
     * ---------------------------------------------------------------------- */

    publishKeyFrameImages(mappedKeyFrames, msgTime_s_in);
    publishKeyFrameMarkers(mappedKeyFrames, msgTime_s_in);
    publishFiducialMarkers(mappedFiducialMarkers, msgTime_s_in);
    cv::Mat slamSystemCurrentFrame{};
    if (p_slamSystem->getCurrentFrame(slamSystemCurrentFrame) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentFrame returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    publishTrackingImage(slamSystemCurrentFrame, msgTime_s_in);
    publishStructuralElements(mappedRooms,
                              mappedFloors,
                              mappedPassages,
                              msgTime_s_in);

    /*!
     * Publish mapped walls independently of the point-cloud visualisation
     * setting because they are consumed by GNN-based room detection.
     */
    publishTimed(vs_graphs::observability::PublishTopic::ALL_MAPPED_WALLS,
                 [&]() { publishAllMappedWalls(mappedPlanes, msgTime_s_in); });

    /*!
     * Publish mapped rooms and passages alongside walls so an offline
     * evaluation run can reconstruct the full generated scene graph
     * (rooms, walls, passages) without loading a saved Atlas file.
     */
    publishAllMappedRooms(mappedRooms, msgTime_s_in);
    publishAllMappedPassages(mappedPassages, mappedRooms, msgTime_s_in);
    publishAllMappedFloors(mappedFloors, msgTime_s_in);

    /*!
     * Archive the same snapshot to JSON. The archiver rate-limits itself on
     * the sim-clock timestamp and no-ops unless test_run_dir is set.
     */
    maybeArchiveSGraph(mappedFloors, mappedRooms, mappedPassages, msgTime_s_in);

    /* ------------------------------------------------------------------ *
     * POINT-CLOUD TOPICS
     * ------------------------------------------------------------------ */

    if (pubPointClouds)
    {
        publishTimed(vs_graphs::observability::PublishTopic::SEGMENTED_CLOUD,
                     [&]() { publishSegmentedCloud(mappedKeyFrames); });

        const std::chrono::steady_clock::time_point planeCallStart =
            p_timingSink_in != nullptr && p_timingSink_in->callback != nullptr
                ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
        notifyTiming(vs_graphs::observability::PublishTopic::PLANES,
                     false,
                     planeCallStart,
                     planeCallStart);
        publishPlanes(mappedPlanes, mappedRooms, msgTime_s_in, p_timingSink_in);

        if (publishAllPoints_in)
        {
            publishTimed(
                vs_graphs::observability::PublishTopic::ALL_POINTS,
                [&]()
                {
                    std::vector<vs_graphs::core::MapPoint *>
                        slamSystemAllMapPoints{};
                    if (p_slamSystem->getAllMapPoints(slamSystemAllMapPoints) !=
                        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getAllMapPoints returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    publishAllPoints(slamSystemAllMapPoints, msgTime_s_in);
                });
        }
        else
        {
            notifyTiming(vs_graphs::observability::PublishTopic::ALL_POINTS,
                         false,
                         std::chrono::steady_clock::time_point{},
                         std::chrono::steady_clock::time_point{});
        }
        publishTimed(
            vs_graphs::observability::PublishTopic::TRACKED_POINTS,
            [&]()
            {
                std::vector<vs_graphs::core::MapPoint *>
                    slamSystemTrackedMapPoints{};
                if (p_slamSystem->getTrackedMapPoints(
                        slamSystemTrackedMapPoints) !=
                    vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getTrackedMapPoints returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                publishTrackedPoints(slamSystemTrackedMapPoints, msgTime_s_in);
            });
        publishTimed(
            vs_graphs::observability::PublishTopic::FREE_SPACE_CLUSTERS,
            [&]()
            {
                std::vector<std::vector<Eigen::Vector3d>>
                    slamSystemSkeletonCluster{};
                if (p_slamSystem->getSkeletonCluster(
                        slamSystemSkeletonCluster) !=
                    vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getSkeletonCluster returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                publishFreeSpaceClusters(slamSystemSkeletonCluster,
                                         msgTime_s_in);
            });
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
        sensorType == vs_graphs::core::System::IMU_MONOCULAR ||
        sensorType == vs_graphs::core::System::IMU_STEREO ||
        sensorType == vs_graphs::core::System::IMU_RGBD;

    if (!usesInertialSensor)
    {
        return;
    }

    /* T_world_body_SE3f describes the body pose relative to the world frame */
    Sophus::SE3f T_world_body_SE3f{};
    if (p_slamSystem->getImuTwb(T_world_body_SE3f) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getImuTwb returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * ORB-SLAM3 supplies the body linear velocity expressed in the world
     * frame.
     */
    Eigen::Vector3f linearVelocity_world_mps{};
    if (p_slamSystem->getImuVwb(linearVelocity_world_mps) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getImuVwb returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
    const std::vector<vs_graphs::core::MapPoint *> &trackedMapPoints_in,
    const rclcpp::Time                             &msgTime_s_in)
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

    /* Throttled loccams dump (~1 Hz wall clock): what the UAV saw, for
     * post-run tracking-loss diagnosis. Skips when behind; never blocks. */
    try
    {
        static std::chrono::steady_clock::time_point lastDumpTime =
            std::chrono::steady_clock::now() - std::chrono::hours(1);
        const std::chrono::steady_clock::time_point nowTime =
            std::chrono::steady_clock::now();
        if (!sgraphArchiveTestRunDir.empty() &&
            (nowTime - lastDumpTime) >= std::chrono::seconds(1))
        {
            lastDumpTime         = nowTime;
            namespace filesystem = std::filesystem;
            const filesystem::path loccamsDir =
                filesystem::path(sgraphArchiveTestRunDir) / "output" /
                "loccams";
            std::error_code makeError;
            filesystem::create_directories(loccamsDir, makeError);
            if (!makeError)
            {
                const std::int64_t totalNanoseconds =
                    msgTime_s_in.nanoseconds();
                if (totalNanoseconds >= 0)
                {
                    std::ostringstream fileName;
                    fileName << "loccams_" << std::setfill('0') << std::setw(6)
                             << (totalNanoseconds / 1000000000LL) << "_"
                             << std::setfill('0') << std::setw(9)
                             << (totalNanoseconds % 1000000000LL) << ".jpeg";
                    cv::imwrite((loccamsDir / fileName.str()).string(),
                                trackingImage_bgr8_in);
                }
            }
        }
    }
    catch (...)
    {
        /* Loccams is best-effort debug output; never disturb the pipeline. */
    }
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
        bool slamSystemIsSaved{};
        if (p_slamSystem->saveMapPointsAsPCD(request_in->name,
                                             slamSystemIsSaved) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: saveMapPointsAsPCD returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        response_out->success = slamSystemIsSaved;
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
        bool slamSystemIsSaved{};
        if (p_slamSystem->saveMap(request_in->name, slamSystemIsSaved) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: saveMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        response_out->success = slamSystemIsSaved;
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
        if (p_slamSystem->saveTrajectoryEuRoC(cameraTrajectoryFileName) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: saveTrajectoryEuRoC returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Save the estimated keyframe trajectory */
        if (p_slamSystem->saveKeyFrameTrajectoryEuRoC(
                keyFrameTrajectoryFileName) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: saveKeyFrameTrajectoryEuRoC returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

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

    /*!
     * Widened from 50: the external segmenter's throughput can fall well
     * behind keyframe creation (measured: a ~80% "point cloud unavailable"
     * rate in SemanticSegmentation, tracked to this same depth on the
     * segmenter's own matching subscription QoS -- KeepLast(50) with
     * reliable()/transient_local() drops the OLDEST unacknowledged message
     * once the backlog exceeds depth, silently losing keyframes the
     * segmenter never got a chance to process, before any C++-side buffer
     * logic even sees them). This does not fix the underlying throughput
     * mismatch -- it only buys more backlog headroom before loss starts.
     * Keep in sync with scene_segment_ros/src/segmenter_yolo26.py's
     * subscription QoS.
     */
    const rclcpp::QoS keyFrameImagePublisherQoS =
        rclcpp::QoS(rclcpp::KeepLast(500)).reliable().transient_local();

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
     * MAPPED-ROOM AND MAPPED-PASSAGE PUBLISHERS
     * ---------------------------------------------------------------------- */

    pubAllRooms =
        node_in->create_publisher<vs_graphs::msg::VSGraphsAllDetectdetRooms>(
            makeTopicName("all_mapped_rooms"),
            standardPublisherQoS);

    pubAllPassages =
        node_in->create_publisher<vs_graphs::msg::VSGraphsAllPassagesData>(
            makeTopicName("all_mapped_passages"),
            standardPublisherQoS);

    pubAllFloors =
        node_in->create_publisher<vs_graphs::msg::VSGraphsAllFloorsData>(
            makeTopicName("all_mapped_floors"),
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
        sensorType == vs_graphs::core::System::IMU_MONOCULAR ||
        sensorType == vs_graphs::core::System::IMU_STEREO ||
        sensorType == vs_graphs::core::System::IMU_RGBD;

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

void shutdownRosInterfaces()
{
    pubKeyFrameList.reset();
    pubOdometry.reset();
    pubAllMappoints.reset();
    pubCameraPose.reset();
    pubKFImage.reset();
    pubTrackedMappoints.reset();
    p_voxbloxInputPointCloudPublisher.reset();
    p_mapRevisionPublisher.reset();
    pubKeyFrameMarker.reset();
    pubFreespaceCluster.reset();
    pubCameraPoseVis.reset();
    pubTrackingImage.reset();
    pubDoor.reset();
    pubFiducialMarker.reset();
    pubPlaneLabel.reset();
    pubBuildingComponents.reset();
    pubSegmentedPointcloud.reset();
    pubAllWalls_new.reset();
    pubAllWalls_legacy.reset();
    pubStructuralElements.reset();

    srvSaveMap.reset();
    srvSaveMapPoints.reset();
    srvSaveTrajectory.reset();
    srvGetMissionHealth.reset();
    srvEstimatorHealth.reset();

    tfListener_.reset();
    tfBroadcaster.reset();
    staticTfBroadcaster.reset();
    tfBuffer_.reset();
}

static void getEstimatorHealthService(
    const std::shared_ptr<vs_graphs::srv::EstimatorHealth::Request> request_in,
    std::shared_ptr<vs_graphs::srv::EstimatorHealth::Response> response_out)
{
    (void)request_in;
    const double nowWallSeconds =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    const double lastWallSeconds    = estimatorFrameWallSeconds.load();
    response_out->frames_per_second = estimatorFramesPerSecond.load();
    response_out->last_frame_age_seconds =
        lastWallSeconds > 0.0 ? nowWallSeconds - lastWallSeconds
                              : std::numeric_limits<double>::infinity();
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

    /* Only pay for the room/floor/passage/topology enumeration -- which
     * takes the semantic update lock -- when the caller actually asked for
     * topology. A poller that only wants the segmentation backlog counters
     * (e.g. a lockstep controller sampling at ~10 Hz) sets
     * include_topology=false and gets the cheap snapshot path instead. */
    vs_graphs::core::System::MissionHealthSnapshot snapshot{};
    if (p_slamSystem->getMissionHealthSnapshot(snapshot,
                                               request_in->include_topology) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMissionHealthSnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    response_out->available            = true;
    response_out->mode                 = sensorModeName();
    response_out->frame_timestamp      = snapshot.frameTimestamp;
    response_out->tracking_state       = snapshot.trackingState;
    response_out->tracking_inliers     = snapshot.trackingInliers;
    response_out->inertial             = snapshot.isInertial;
    response_out->inertial_initialized = snapshot.isInertialInitialized;
    response_out->pose_valid           = snapshot.isPoseValid;
    response_out->map_id               = snapshot.mapId;
    response_out->map_count            = snapshot.mapCount;
    response_out->keyframe_count       = snapshot.keyFrameCount;
    response_out->reset_count          = snapshot.resetCount;
    response_out->rgbd_frontend_accepted_count =
        snapshot.rgbdFrontendAcceptedCount;
    response_out->rgbd_frontend_processed_count =
        snapshot.rgbdFrontendProcessedCount;
    response_out->rgbd_frontend_overwritten_count =
        snapshot.rgbdFrontendOverwrittenCount;
    response_out->rgbd_frontend_worker_in_flight =
        snapshot.isRgbdFrontendWorkerInFlight;
    response_out->rgbd_frontend_last_processed_sensor_timestamp_nanoseconds =
        snapshot.rgbdFrontendLastProcessedSensorTimestampNanoseconds;
    response_out->segmentation_published_count =
        snapshot.segmentationPublishedCount;
    response_out->segmentation_returned_count =
        snapshot.segmentationReturnedCount;
    response_out->last_returned_keyframe_id = snapshot.lastReturnedKeyFrameId;
    response_out->segmentation_enqueued_count =
        snapshot.segmentationEnqueuedCount;
    response_out->segmentation_dequeued_count =
        snapshot.segmentationDequeuedCount;
    response_out->segmentation_terminal_count =
        snapshot.segmentationTerminalCount;
    response_out->segmentation_accepted_count =
        snapshot.segmentationAcceptedCount;
    response_out->segmentation_dropped_count =
        snapshot.segmentationDroppedCount;
    response_out->segmentation_missing_keyframe_count =
        snapshot.segmentationMissingKeyFrameCount;
    response_out->segmentation_missing_cloud_count =
        snapshot.segmentationMissingCloudCount;
    response_out->segmentation_stale_map_count =
        snapshot.segmentationStaleMapCount;
    response_out->last_terminal_keyframe_id = snapshot.lastTerminalKeyFrameId;
    response_out->segmentation_queue_depth  = snapshot.segmentationQueueDepth;
    response_out->segmentation_queue_high_watermark =
        snapshot.segmentationQueueHighWatermark;

    if (snapshot.isPoseValid)
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
    response_out->last_loop_accepted  = snapshot.wasLastLoopAccepted;
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

    Json topology = {{"schema", 1},
                     {"map_id", snapshot.mapId},
                     {"active_maps", snapshot.mapCount},
                     {"reset_count", snapshot.resetCount},
                     {"confirmed_rooms", snapshot.confirmedRoomCount},
                     {"unresolved_rooms", snapshot.unresolvedRoomCount},
                     {"floor_room_links", snapshot.floorRoomLinkCount},
                     {"rooms", Json::array()},
                     {"floors", Json::array()},
                     {"passages", Json::array()}};
    for (const vs_graphs::core::System::RoomHealth &room : snapshot.rooms)
    {
        topology["rooms"].push_back(
            {{"id", room.id}, {"passage_ids", room.passageIds}});
    }
    for (const vs_graphs::core::System::FloorHealth &floor : snapshot.floors)
    {
        topology["floors"].push_back(
            {{"id", floor.id}, {"room_ids", floor.roomIds}});
    }
    for (const vs_graphs::core::System::PassageHealth &passage :
         snapshot.passages)
    {
        const bool traversed = passage.primaryTraversalCount > 0U ||
                               passage.secondaryTraversalCount > 0U ||
                               passage.unknownCount > 0U;
        const bool bidirectional = passage.primaryTraversalCount > 0U &&
                                   passage.secondaryTraversalCount > 0U;
        if (passage.isPassable)
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
        response_out->traversal_known_to_far_count +=
            passage.primaryTraversalCount;
        response_out->traversal_far_to_known_count +=
            passage.secondaryTraversalCount;
        response_out->traversal_unknown_count += passage.unknownCount;

        topology["passages"].push_back(
            {{"id", passage.id},
             {"passable", passage.isPassable},
             {"primary_room_id", passage.primaryRoomId},
             {"secondary_room_id", passage.secondaryRoomId},
             {"primary_traversal_count", passage.primaryTraversalCount},
             {"secondary_traversal_count", passage.secondaryTraversalCount},
             {"unknown", passage.unknownCount},
             {"bidirectional", bidirectional}});
    }

    if (request_in->include_topology)
    {
        /* Extend the existing
         * schema-1 topology object to schema 2 with copied-cache evaluator
         * additions, without changing GetMissionHealth.srv or duplicating
         * this method's own schema-1 collection above. */
        bool cacheAvailable{};
        if (p_slamSystem->isSemanticReportCacheAvailable(cacheAvailable) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isSemanticReportCacheAvailable returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        vs_graphs::core::semantic::SemanticReportCacheEntry entry{};
        if (p_slamSystem->getSemanticReportCacheEntry(entry) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getSemanticReportCacheEntry returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        nlohmann::json augmentedJson{};
        if (vs_graphs::core::augmentMissionHealthTopologyJsonWithSemantics(
                std::move(topology),
                entry,
                cacheAvailable,
                augmentedJson) !=
            vs_graphs::core::MissionHealthTopologyJsonStatus::
                MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: augmentMissionHealthTopologyJsonWithSemantics returned a "
                "failure status although it cannot fail; continuing as before.",
                __func__);
        }
        topology = std::move(augmentedJson);

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
    srvEstimatorHealth.reset();

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

    srvEstimatorHealth =
        node_in->create_service<vs_graphs::srv::EstimatorHealth>(
            makeServiceName("estimator_health"),
            &getEstimatorHealthService);

    /* Confirm that all service objects were created */
    if (srvSaveMap == nullptr || srvSaveMapPoints == nullptr ||
        srvSaveTrajectory == nullptr || srvGetMissionHealth == nullptr ||
        srvEstimatorHealth == nullptr)
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
    vs_graphs::core::types::SystemParams *systemParameters = nullptr;
    if (vs_graphs::core::types::SystemParams::getParams(systemParameters) !=
        vs_graphs::core::types::SystemParamsStatus::
            SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
        systemParameters->roomSeg.minClusterVertices;

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

    /* Reset the per-scan rejection counts before classifying markers. */
    const std::uint64_t ingestSequence = lastSparseIngest.sequence + 1U;
    lastSparseIngest          = vs_graphs::sparse::SparseIngestCounts();
    lastSparseIngest.sequence = ingestSequence;

    /* Process every marker contained in the sparse graph message */
    for (const visualization_msgs::msg::Marker &skeletonMarker :
         skeletonMarkerArray_in.markers)
    {
        /* Classify the marker once so every skip path carries a typed reason.
         */
        using vs_graphs::sparse::SparseMarkerVerdict;
        const SparseMarkerVerdict markerVerdict =
            vs_graphs::sparse::classifySparseMarker(
                static_cast<int>(skeletonMarker.type),
                skeletonMarker.ns,
                skeletonMarker.points.size(),
                minimumClusterVertexCount);
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

        /* Ignore marker types that are not required by this pipeline */
        if (markerVerdict == SparseMarkerVerdict::SPARSE_MARKER_IGNORED)
        {
            lastSparseIngest.ignoredMarkerCount++;
            continue;
        }

        /* Ignore cluster markers without any point data */
        if (markerVerdict == SparseMarkerVerdict::SPARSE_CLUSTER_EMPTY)
        {
            lastSparseIngest.emptyClusterCount++;
            continue;
        }

        /* Ignore edge markers without any point data */
        if (markerVerdict == SparseMarkerVerdict::SPARSE_EDGE_EMPTY)
        {
            lastSparseIngest.emptyEdgeCount++;
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
            if (isConnectedVertexMarker)
            {
                lastSparseIngest.clusterTransformFailureCount++;
            }
            else
            {
                lastSparseIngest.edgeTransformFailureCount++;
            }
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
                lastSparseIngest.undersizedClusterCount++;
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
                    lastSparseIngest.clusterPointDropCount++;
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
         * Everything reaching this point was classified as a raw "edges"
         * LINE_LIST marker by classifySparseMarker(). The raw marker is used
         * instead of connected_edges_* because connected edge markers may
         * have already been clearance filtered at narrow passages.
         *
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
                lastSparseIngest.edgeTransformFailureCount++;
                continue;
            }

            const double edgeLength_m =
                (edgeEnd_world - edgeStart_world).norm();

            /* Reject invalid and effectively zero-length edges */
            if (!edgeStart_world.allFinite() || !edgeEnd_world.allFinite() ||
                edgeLength_m < 1e-6)
            {
                lastSparseIngest.degenerateEdgeCount++;
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
    if (p_slamSystem->setSkeletonCluster(skeletonClusterPoints) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonCluster returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Store the complete raw skeleton edges in the active map */
    if (p_slamSystem->setSkeletonEdges(skeletonEdges) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonEdges returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
}

/*!
 * Emits one SG_PIPELINE summary describing a sparse-graph callback: marker
 * namespaces, actions, source frames, raw cluster/edge counts and the
 * accepted (transformed) cluster geometry. Call only with the health-state
 * mutex held. An unchanged payload is re-logged at most once every ten
 * seconds so a quiet skeletonizer stays visible without flooding the log.
 */
void logSparseGraphCallbackSummary(
    const visualization_msgs::msg::MarkerArray  &sparseGraphMessage_in,
    const std::chrono::steady_clock::time_point &now_in,
    VoxbloxHealthState                          &healthState_inout,
    const bool                                   isReset_in)
{
    constexpr std::size_t MAX_LOGGED_NAMES        = 8U;
    constexpr std::size_t MAX_LOGGED_CLUSTERS     = 8U;
    constexpr double      SUMMARY_REPEAT_PERIOD_S = 10.0;

    /* Scan the raw markers for namespaces, frames and eligible geometry. */
    std::size_t              deleteAllMarkerCount = 0U;
    std::size_t              clusterMarkerCount   = 0U;
    std::size_t              clusterPointCount    = 0U;
    std::size_t              edgeMarkerCount      = 0U;
    std::size_t              edgePointCount       = 0U;
    std::vector<std::string> distinctNamespaces;
    std::vector<std::string> distinctFrames;

    for (const visualization_msgs::msg::Marker &sparseMarker :
         sparseGraphMessage_in.markers)
    {
        if (sparseMarker.action == visualization_msgs::msg::Marker::DELETEALL)
        {
            deleteAllMarkerCount++;
        }

        if (std::find(distinctNamespaces.begin(),
                      distinctNamespaces.end(),
                      sparseMarker.ns) == distinctNamespaces.end())
        {
            distinctNamespaces.push_back(sparseMarker.ns);
        }

        if (std::find(distinctFrames.begin(),
                      distinctFrames.end(),
                      sparseMarker.header.frame_id) == distinctFrames.end())
        {
            distinctFrames.push_back(sparseMarker.header.frame_id);
        }

        const bool isClusterMarker =
            sparseMarker.type == visualization_msgs::msg::Marker::CUBE_LIST &&
            sparseMarker.ns.rfind("connected_vertices_", 0) == 0;
        const bool isEdgeMarker =
            sparseMarker.type == visualization_msgs::msg::Marker::LINE_LIST &&
            sparseMarker.ns == "edges";

        if (isClusterMarker)
        {
            clusterMarkerCount++;
            clusterPointCount += sparseMarker.points.size();
        }

        if (isEdgeMarker)
        {
            edgeMarkerCount++;
            edgePointCount += sparseMarker.points.size();
        }
    }

    /* Identify the payload so unchanged summaries can be throttled. */
    std::string summarySignature =
        std::to_string(sparseGraphMessage_in.markers.size()) + "/" +
        std::to_string(deleteAllMarkerCount) + "/" +
        std::to_string(clusterMarkerCount) + "/" +
        std::to_string(clusterPointCount) + "/" +
        std::to_string(edgeMarkerCount) + "/" + std::to_string(edgePointCount) +
        "/" + std::to_string(healthState_inout.acceptedClusterCount) + "/" +
        std::to_string(healthState_inout.acceptedVertexCount) + "/" +
        std::to_string(healthState_inout.acceptedEdgeCount) + "/" +
        std::to_string(lastSparseIngest.sequence) + "/" +
        std::to_string(lastSparseIngest.ignoredMarkerCount) + "/" +
        std::to_string(lastSparseIngest.emptyClusterCount) + "/" +
        std::to_string(lastSparseIngest.emptyEdgeCount) + "/" +
        std::to_string(lastSparseIngest.undersizedClusterCount) + "/" +
        std::to_string(lastSparseIngest.clusterTransformFailureCount) + "/" +
        std::to_string(lastSparseIngest.clusterPointDropCount) + "/" +
        std::to_string(lastSparseIngest.edgeTransformFailureCount) + "/" +
        std::to_string(lastSparseIngest.degenerateEdgeCount);

    const double summaryAge_s = ageSeconds(
        now_in,
        healthState_inout.lastSparseSummaryLog,
        healthState_inout.lastSparseSummaryLog.time_since_epoch().count() != 0);

    if (summarySignature == healthState_inout.lastSparseSummarySignature &&
        summaryAge_s >= 0.0 && summaryAge_s < SUMMARY_REPEAT_PERIOD_S)
    {
        return;
    }

    healthState_inout.lastSparseSummarySignature = summarySignature;
    healthState_inout.lastSparseSummaryLog       = now_in;

    if (isReset_in)
    {
        std::cout << "SG_PIPELINE {\"event\":\"voxblox_sparse_reset\","
                     "\"markers\":"
                  << sparseGraphMessage_in.markers.size()
                  << ",\"reset_revision\":" << healthState_inout.resetRevision
                  << ",\"ingest_sequence\":" << lastSparseIngest.sequence << "}"
                  << std::endl;

        return;
    }

    std::cout << "SG_PIPELINE {\"event\":\"voxblox_sparse_graph\","
                 "\"markers\":"
              << sparseGraphMessage_in.markers.size()
              << ",\"delete_all\":" << deleteAllMarkerCount
              << ",\"namespace_count\":" << distinctNamespaces.size()
              << ",\"namespaces\":[";

    for (std::size_t namespaceIndex = 0;
         namespaceIndex < distinctNamespaces.size() &&
         namespaceIndex < MAX_LOGGED_NAMES;
         ++namespaceIndex)
    {
        std::cout << (namespaceIndex > 0 ? "," : "") << "\""
                  << distinctNamespaces[namespaceIndex] << "\"";
    }

    std::cout << "],\"frame_count\":" << distinctFrames.size()
              << ",\"frames\":[";

    for (std::size_t frameIndex = 0;
         frameIndex < distinctFrames.size() && frameIndex < MAX_LOGGED_NAMES;
         ++frameIndex)
    {
        std::cout << (frameIndex > 0 ? "," : "") << "\""
                  << distinctFrames[frameIndex] << "\"";
    }

    std::cout << "],\"cluster_markers\":" << clusterMarkerCount
              << ",\"cluster_points\":" << clusterPointCount
              << ",\"edge_markers\":" << edgeMarkerCount
              << ",\"edge_points\":" << edgePointCount
              << ",\"accepted_clusters\":"
              << healthState_inout.acceptedClusterCount
              << ",\"cluster_vertices\":[";

    for (std::size_t clusterIndex = 0;
         clusterIndex < skeletonClusterPoints.size() &&
         clusterIndex < MAX_LOGGED_CLUSTERS;
         ++clusterIndex)
    {
        std::cout << (clusterIndex > 0 ? "," : "")
                  << skeletonClusterPoints[clusterIndex].size();
    }

    std::cout << "],\"accepted_vertices\":"
              << healthState_inout.acceptedVertexCount
              << ",\"accepted_edges\":" << healthState_inout.acceptedEdgeCount
              << ",\"reset_revision\":" << healthState_inout.resetRevision
              << ",\"ingest_sequence\":" << lastSparseIngest.sequence
              << ",\"reject_ignored\":" << lastSparseIngest.ignoredMarkerCount
              << ",\"reject_empty_clusters\":"
              << lastSparseIngest.emptyClusterCount
              << ",\"reject_empty_edges\":" << lastSparseIngest.emptyEdgeCount
              << ",\"reject_undersized\":"
              << lastSparseIngest.undersizedClusterCount
              << ",\"reject_cluster_transform_failed\":"
              << lastSparseIngest.clusterTransformFailureCount
              << ",\"reject_points_dropped\":"
              << lastSparseIngest.clusterPointDropCount
              << ",\"reject_edge_transform_failed\":"
              << lastSparseIngest.edgeTransformFailureCount
              << ",\"reject_degenerate_edges\":"
              << lastSparseIngest.degenerateEdgeCount << "}" << std::endl;
}

void observeVoxbloxSkeletonPublication(
    const sensor_msgs::msg::PointCloud2 &skeletonMessage_in)
{
    std::lock_guard<std::mutex> lock(voxbloxHealth.mutex);
    voxbloxHealth.lastSkeleton = std::chrono::steady_clock::now();
    voxbloxHealth.hasSkeleton  = true;
    const std::int64_t messageStamp_ns =
        rclcpp::Time(skeletonMessage_in.header.stamp).nanoseconds();
    voxbloxHealth.lastSkeletonStamp_ns =
        messageStamp_ns > 0 ? messageStamp_ns : voxbloxHealth.lastInputStamp_ns;
}

void observeVoxbloxSparseGraphPublication(
    const visualization_msgs::msg::MarkerArray &sparseGraphMessage_in)
{
    const bool isReset =
        std::any_of(sparseGraphMessage_in.markers.begin(),
                    sparseGraphMessage_in.markers.end(),
                    [](const visualization_msgs::msg::Marker &marker_in) {
                        return marker_in.action ==
                               visualization_msgs::msg::Marker::DELETEALL;
                    });

    std::lock_guard<std::mutex>                 lock(voxbloxHealth.mutex);
    const std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now();
    if (isReset)
    {
        voxbloxHealth.lastReset = now;
        voxbloxHealth.hasReset  = true;
        voxbloxHealth.resetRevision++;
        voxbloxHealth.acceptedClusterCount = 0U;
        voxbloxHealth.acceptedVertexCount  = 0U;
        voxbloxHealth.acceptedEdgeCount    = 0U;
        logSparseGraphCallbackSummary(sparseGraphMessage_in,
                                      now,
                                      voxbloxHealth,
                                      true);
        return;
    }

    voxbloxHealth.lastSparseGraph      = now;
    voxbloxHealth.hasSparseGraph       = true;
    std::int64_t newestMessageStamp_ns = 0;
    for (const visualization_msgs::msg::Marker &marker :
         sparseGraphMessage_in.markers)
    {
        newestMessageStamp_ns =
            std::max(newestMessageStamp_ns,
                     rclcpp::Time(marker.header.stamp).nanoseconds());
    }
    voxbloxHealth.lastSparseGraphStamp_ns =
        newestMessageStamp_ns > 0 ? newestMessageStamp_ns
                                  : voxbloxHealth.lastInputStamp_ns;
    voxbloxHealth.acceptedClusterCount = skeletonClusterPoints.size();
    voxbloxHealth.acceptedVertexCount  = 0U;
    for (const std::vector<Eigen::Vector3d> &cluster : skeletonClusterPoints)
    {
        voxbloxHealth.acceptedVertexCount += cluster.size();
    }
    voxbloxHealth.acceptedEdgeCount = skeletonEdges.size();
    logSparseGraphCallbackSummary(sparseGraphMessage_in,
                                  now,
                                  voxbloxHealth,
                                  false);
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
