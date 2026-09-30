/*!
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

/*!
 * @file            ros_rgbd.cc
 *
 * @brief           The RGB-D ROS 2 node: receives camera images, feeds them to
 *                  the SLAM system and publishes its results.
 */

#include "RgbdObservability.h"
#include "common.hpp"

#include <condition_variable>
#include <rclcpp/logging.hpp>

class ImageGrabber : public rclcpp::Node
{
  public:
    /*!
     * @brief       Owns one timestamp-coherent RGB, depth, and point-cloud
     *              sample.
     */
    struct SynchronizedRgbdPacket
    {
        sensor_msgs::msg::Image::ConstSharedPtr       p_rgbImageMessage;
        sensor_msgs::msg::Image::ConstSharedPtr       p_depthImageMessage;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
        std::int64_t sensorTimestampNanoseconds{0};
        vs_graphs::observability::RgbdObservability::SteadyTime callbackArrival;
    };

    /*!
     * @brief       Constructs the RGB-D adapter with the parent node's clock
     *              mode.
     *
     * @param[in]   useSimTime_in
     *              True when timestamps must follow `/clock`.
     */
    ImageGrabber(const bool useSimTime_in, const bool directGazeboFluCloud_in) :
        rclcpp::Node(
            "grabber",
            rclcpp::NodeOptions()
                .use_global_arguments(false)
                .parameter_overrides(
                    {rclcpp::Parameter("use_sim_time", useSimTime_in)})),
        directGazeboFluCloud(directGazeboFluCloud_in)
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    /*!
     * @brief       Processes the newest coherent sensor packet.
     *
     *              A pending packet is replaced while tracking is busy, as in
     *              the established RGB-D delivery path.
     */
    void ProcessRgbdPackets();

    /*!
     * @brief       Requests worker shutdown and wakes a waiting worker.
     */
    void RequestStop();

    /*! Logs a cumulative summary without holding packet/accounting locks. */
    void LogRgbdObservabilitySummary(const std::string &event_in) const;

    /*!
     * @brief       Callback function to get the skeleton graph from the
     *              `voxblox` module
     *
     * @param       msgSkeletonGraph
     *              The skeleton graph from the `voxblox` module
     */
    void GrabVoxbloxSkeletonGraph(
        const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);

    /*!
     * @brief       Admits a coherent RGB-D packet without blocking ROS
     *              input.
     *
     * @param[in]   msgRGB_in
     *              RGB image message.
     *
     * @param[in]   msgD_in
     *              Registered depth image message.
     *
     * @param[in]   msgPC_in
     *              Point cloud corresponding to the image pair.
     */
    void
        GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr       &msgRGB_in,
                 const sensor_msgs::msg::Image::ConstSharedPtr       &msgD_in,
                 const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC_in);

  private:
    /*! Publishes a lock-free copy of frontend progress into System health. */
    void PublishRgbdFrontendHealth() const;

    std::mutex              rgbdPacketMutex;
    std::condition_variable rgbdPacketCondition;
    SynchronizedRgbdPacket  latestRgbdPacket;
    bool                    hasPendingRgbdPacket{false};
    bool                    stopRequested{false};
    bool                    hasReceivedRgbdPacket{false};
    double                  lastReceivedRgbdTimestamp_seconds{0.0};
    bool                    hasProcessedRgbdPacket{false};
    double                  lastProcessedRgbdTimestamp_seconds{0.0};
    const bool              directGazeboFluCloud;
    vs_graphs::observability::RgbdObservability rgbdObservability;
};

int main(int argc, char **argv)
{
    /* Init ROS node */
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node =
        std::make_shared<rclcpp::Node>("vs_graphs");

    /* Confirm number of arguments supplied are valid */
    if (argc > 1)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Arguments supplied via command line are ignored.");
    }

    /* Extract node name */
    std::string nodeName = node->get_name();

    /* Declare ROS parameters */
    node->declare_parameter<double>("yaw", 0.0);
    node->declare_parameter<double>("roll", 0.0);
    node->declare_parameter<double>("pitch", 0.0);

    node->declare_parameter<bool>("enable_pangolin", true);

    node->declare_parameter<bool>("static_transform", false);
    node->declare_parameter<std::string>("frame_map", "map");

    node->declare_parameter<bool>("colored_pointcloud", true);
    node->declare_parameter<bool>("publish_pointclouds", true);

    node->declare_parameter<std::string>("frame_world", "world");
    node->declare_parameter<std::string>("frame_camera", "camera");
    node->declare_parameter<std::string>("frame_structural_element",
                                         "struc_elem");
    node->declare_parameter<std::string>("frame_building_component",
                                         "build_comp");

    node->declare_parameter<std::string>("voc_file", "file_not_set");
    node->declare_parameter<std::string>("settings_file", "file_not_set");
    node->declare_parameter<std::string>("sys_params_file", "file_not_set");

    node->declare_parameter<bool>("direct_gazebo_flu_cloud", false);

    node->declare_parameter<std::string>("log_level", "info");

    node->declare_parameter<std::string>("test_run_dir", "");
    node->declare_parameter<bool>("sgraph_archive_enabled", true);
    node->declare_parameter<double>("sgraph_archive_interval_sec", 5.0);
    node->declare_parameter<int>("sgraph_archive_max_files", 0);

    std::string vocFile      = node->get_parameter("voc_file").as_string();
    std::string settingsFile = node->get_parameter("settings_file").as_string();
    std::string sysParamsFile =
        node->get_parameter("sys_params_file").as_string();

    /* Confirm the VOC file is set */
    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "[Error] 'vocabulary' and 'settings' are not provided in "
                     "the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    /* Confirm there is a system params file set */
    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "[Error] The `YAML` file containing system parameters is "
                     "not provided in the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    /* Extract parameters */
    yaw             = node->get_parameter("yaw").as_double();
    roll            = node->get_parameter("roll").as_double();
    pitch           = node->get_parameter("pitch").as_double();
    frameMap        = node->get_parameter("frame_map").as_string();
    frameWorld      = node->get_parameter("frame_world").as_string();
    frameCamera     = node->get_parameter("frame_camera").as_string();
    colorPointcloud = node->get_parameter("colored_pointcloud").as_bool();
    pubPointClouds  = node->get_parameter("publish_pointclouds").as_bool();
    frameBC = node->get_parameter("frame_building_component").as_string();
    frameSE = node->get_parameter("frame_structural_element").as_string();
    pubStaticTransform  = node->get_parameter("static_transform").as_bool();
    bool enablePangolin = node->get_parameter("enable_pangolin").as_bool();
    vs_graphs::core::Verbose::VerbosityLevel verboseLevel{};
    if (vs_graphs::core::Verbose::parseVerbosityLevel(
            node->get_parameter("log_level").as_string(),
            verboseLevel) !=
        vs_graphs::core::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: parseVerbosityLevel returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    sgraphArchiveTestRunDir = node->get_parameter("test_run_dir").as_string();
    sgraphArchiveEnabled =
        node->get_parameter("sgraph_archive_enabled").as_bool();
    sgraphArchiveIntervalSec =
        node->get_parameter("sgraph_archive_interval_sec").as_double();
    sgraphArchiveMaxFiles = static_cast<int>(
        node->get_parameter("sgraph_archive_max_files").as_int());

    /* Initializing system threads and getting ready to process frames */
    const bool useSimTime = node->get_parameter("use_sim_time").as_bool();
    const bool directGazeboFluCloud =
        node->get_parameter("direct_gazebo_flu_cloud").as_bool();
    std::shared_ptr<ImageGrabber> igb =
        std::make_shared<ImageGrabber>(useSimTime, directGazeboFluCloud);

    /* Declare system type */
    sensorType = vs_graphs::core::System::RGBD;

    /* ---------------------------------------------------------------------- *
     * VSGRAPH SLAM SYSTEM INIT
     * ---------------------------------------------------------------------- */

    p_slamSystem = new vs_graphs::core::System();
    const vs_graphs::core::SystemStatus systemStatus =
        p_slamSystem->initialize(vocFile,
                                 settingsFile,
                                 sysParamsFile,
                                 sensorType,
                                 enablePangolin,
                                 /*initFr*/ 0,
                                 /*strSequence*/ std::string(),
                                 verboseLevel);
    if (systemStatus != vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        /* A settings, vocabulary or map file could not be read: stop with
         * the same exit code the system used before (exit(-1)). */
        RCLCPP_FATAL(node->get_logger(),
                     "vS-Graphs could not start: a settings, vocabulary or "
                     "map file could not be read (status %d).",
                     static_cast<int>(systemStatus));
        return -1;
    }

    /* ---------------------------------------------------------------------- *
     * SETUP CALLBACKS
     * ---------------------------------------------------------------------- */

    /*!
     * Keep sensor admission separate from semantic callbacks. The RGB-D
     * callback only replaces the pending coherent packet; the worker owns all
     * expensive conversion, tracking, and publication work.
     */
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::PointCloud2;

    /* Init image callback */
    const rclcpp::CallbackGroup::SharedPtr visualCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    /* Init semantic image callback */
    const rclcpp::CallbackGroup::SharedPtr semanticCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    /* Init voxblox skeleton callback */
    const rclcpp::CallbackGroup::SharedPtr skeletonCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    /* Init visual subscription options and store callback */
    rclcpp::SubscriptionOptions visualSubscriptionOptions;
    visualSubscriptionOptions.callback_group = visualCallbackGroup;

    /* Init semantic subscription options and store callback */
    rclcpp::SubscriptionOptions semanticSubscriptionOptions;
    semanticSubscriptionOptions.callback_group = semanticCallbackGroup;

    /* Init voxblox skeleton subscription options and store callback */
    rclcpp::SubscriptionOptions skeletonSubscriptionOptions;
    skeletonSubscriptionOptions.callback_group = skeletonCallbackGroup;

    /* Subscribe to the rgb image */
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
        subImgRGB =
            std::make_shared<Subscriber<Image>>(node.get(),
                                                "/camera/rgb/image_raw",
                                                rmw_qos_profile_sensor_data,
                                                visualSubscriptionOptions);

    /* Subscribe to the image  depth points */
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
        subPointcloud = std::make_shared<Subscriber<PointCloud2>>(
            node.get(),
            "/camera/depth/points",
            rmw_qos_profile_sensor_data,
            visualSubscriptionOptions);

    /* Subscribe to the image  depth points */
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
        subImgDepth = std::make_shared<Subscriber<Image>>(
            node.get(),
            "/camera/depth_registered/image_raw",
            rmw_qos_profile_sensor_data,
            visualSubscriptionOptions);

    /* Init sync policy */
    typedef ApproximateTime<Image, Image, PointCloud2> syncPolicy;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image,
            sensor_msgs::msg::PointCloud2>>>
        sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10),
                                                          *subImgRGB,
                                                          *subImgDepth,
                                                          *subPointcloud);

    /* Set maximum interval duration of sync policy */
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.033));

    /* Set sync policy to RGBD register callback */
    sync->registerCallback(std::bind(&ImageGrabber::GrabRGBD,
                                     igb.get(),
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     std::placeholders::_3));

    /* Subscribe to segmentation results from the SemanticSegmenter module */
    std::shared_ptr<rclcpp::Subscription<segmenter_ros::msg::SegmenterDataMsg>>
        subSegmentedImage =
            node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
                "/camera/color/image_segment",
                rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local(),
                [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
                { addSegmentationToSystem(*msg, igb->get_logger()); },
                semanticSubscriptionOptions);

    /* Subsriber to get skeletonized graph from the `voxblox` module */
    /* Match the skeletonizer transient-local publisher so the latest usable
     * graph is received even when this node joins after publication. */
    std::shared_ptr<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>>
        subVoxbloxSkeletonMesh =
            node->create_subscription<visualization_msgs::msg::MarkerArray>(
                "/voxblox_skeletonizer/sparse_graph",
                rclcpp::QoS(1).transient_local(),
                [igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
                {
                    igb->GrabVoxbloxSkeletonGraph(*msg);
                    observeVoxbloxSparseGraphPublication(*msg);
                },
                skeletonSubscriptionOptions);

    /* Match the skeletonizer transient-local publisher so the latest usable
     * cloud is received even when this node joins after publication. */
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
        subVoxbloxSkeleton =
            node->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/voxblox_skeletonizer/skeleton",
                rclcpp::QoS(1).transient_local(),
                [](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                { observeVoxbloxSkeletonPublication(*msg); },
                skeletonSubscriptionOptions);

    /* Init image transport */
    std::shared_ptr<image_transport::ImageTransport> image_transport =
        std::make_shared<image_transport::ImageTransport>(node);

    /* ---------------------------------------------------------------------- *
     * PUBLISHERS & SERVICES
     * ---------------------------------------------------------------------- */

    /* Setup publishers for system */
    setupPublishers(node, image_transport, nodeName);

    /* Setup services for system */
    setupServices(node, nodeName);

    /* ---------------------------------------------------------------------- *
     * SENSOR PROCESSING THREAD
     * ---------------------------------------------------------------------- */

    /* Create an rgbd processing thread */
    std::thread rgbdProcessingThread(&ImageGrabber::ProcessRgbdPackets,
                                     igb.get());

    /*!
     * Create a multithreaded ROS 2 executor with 3 worker threads for
     * callbacks.
     *
     * Multiple executor workers prevent semantic callbacks from delaying the
     * 30 Hz sensor admission path.
     */
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      3U);

    /* ---------------------------------------------------------------------- *
     * ROS BOILER PLATE
     * ---------------------------------------------------------------------- */

    executor.add_node(node);
    executor.add_node(igb);
    executor.spin();

    igb->RequestStop();
    rgbdProcessingThread.join();
    igb->LogRgbdObservabilitySummary("shutdown_after_worker_join");
    if (p_slamSystem->shutdown() !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: shutdown returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    delete p_slamSystem;
    p_slamSystem = nullptr;
    shutdownRosInterfaces();
    image_transport.reset();
    rclcpp::shutdown();
    return 0;
}

void ImageGrabber::ProcessRgbdPackets()
{
    constexpr std::chrono::seconds        summaryPeriod{10};
    std::chrono::steady_clock::time_point nextSummaryDeadline =
        std::chrono::steady_clock::now() + summaryPeriod;

    while (true)
    {
        if (std::chrono::steady_clock::now() >= nextSummaryDeadline)
        {
            LogRgbdObservabilitySummary("periodic");
            nextSummaryDeadline =
                std::chrono::steady_clock::now() + summaryPeriod;
        }

        SynchronizedRgbdPacket rgbdPacket;

        {
            std::unique_lock<std::mutex> packetLock(rgbdPacketMutex);
            const bool packetOrStopReady = rgbdPacketCondition.wait_until(
                packetLock,
                nextSummaryDeadline,
                [this]() { return stopRequested || hasPendingRgbdPacket; });

            if (stopRequested)
            {
                const bool hadPendingPacket = hasPendingRgbdPacket;
                hasPendingRgbdPacket        = false;
                packetLock.unlock();
                if (hadPendingPacket)
                {
                    rgbdObservability.recordShutdownPendingDrop();
                    PublishRgbdFrontendHealth();
                }
                break;
            }

            if (!packetOrStopReady)
            {
                packetLock.unlock();
                LogRgbdObservabilitySummary("periodic");
                nextSummaryDeadline =
                    std::chrono::steady_clock::now() + summaryPeriod;
                continue;
            }

            rgbdPacket           = std::move(latestRgbdPacket);
            hasPendingRgbdPacket = false;
        }

        const std::chrono::steady_clock::time_point workerStartTime =
            std::chrono::steady_clock::now();
        rgbdObservability.recordWorkerStart(rgbdPacket.callbackArrival,
                                            workerStartTime);
        PublishRgbdFrontendHealth();

        const double rgbTimestamp_seconds =
            rclcpp::Time(rgbdPacket.p_rgbImageMessage->header.stamp).seconds();

        cv_bridge::CvImageConstPtr p_depthImage;
        cv_bridge::CvImageConstPtr p_rgbImage;

        const std::chrono::steady_clock::time_point imageConversionStart =
            std::chrono::steady_clock::now();
        try
        {
            p_depthImage = cv_bridge::toCvShare(rgbdPacket.p_depthImageMessage);
            p_rgbImage   = cv_bridge::toCvShare(rgbdPacket.p_rgbImageMessage);
        }
        catch (const cv_bridge::Exception &exception)
        {
            rgbdObservability.recordImageConversionReject();
            PublishRgbdFrontendHealth();
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding an RGB-D packet after cv_bridge failed: %s",
                exception.what());
            continue;
        }
        rgbdObservability.recordImageConversion(
            imageConversionStart,
            std::chrono::steady_clock::now());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr        p_pointCloud;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudCameraMessage;
        std::string                                   cloudFailureReason;
        const std::chrono::steady_clock::time_point   cloudPreparationStart =
            std::chrono::steady_clock::now();
        if (!preparePointCloudForTracking(rgbdPacket.p_pointCloudMessage,
                                          directGazeboFluCloud,
                                          p_pointCloud,
                                          p_pointCloudCameraMessage,
                                          cloudFailureReason))
        {
            rgbdObservability.recordCloudConversionReject();
            PublishRgbdFrontendHealth();
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "Discarding point cloud: %s.",
                                 cloudFailureReason.c_str());
            continue;
        }
        rgbdObservability.recordCloudPreparation(
            cloudPreparationStart,
            std::chrono::steady_clock::now());

        const std::chrono::steady_clock::time_point markerAssociationStart =
            std::chrono::steady_clock::now();
        std::pair<double, std::vector<vs_graphs::core::semantic::Marker *>>
                     nearestMarker = findNearestMarker(rgbTimestamp_seconds);
        const double markerTimeDifference_seconds = nearestMarker.first;
        std::vector<vs_graphs::core::semantic::Marker *> matchedMarkers =
            std::move(nearestMarker.second);
        rgbdObservability.recordMarkerAssociation(
            markerAssociationStart,
            std::chrono::steady_clock::now());

        rgbdObservability.recordTrackCall();
        const std::chrono::steady_clock::time_point trackStart =
            std::chrono::steady_clock::now();
        try
        {
            if (markerTimeDifference_seconds < 0.05)
            {
                Sophus::SE3f slamSystemCameraPose{};
                if (p_slamSystem->trackRGBD(p_rgbImage->image,
                                            p_depthImage->image,
                                            p_pointCloud,
                                            rgbTimestamp_seconds,
                                            slamSystemCameraPose,
                                            {},
                                            "",
                                            matchedMarkers) !=
                    vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackRGBD returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                Sophus::SE3f slamSystemCameraPose2{};
                if (p_slamSystem->trackRGBD(p_rgbImage->image,
                                            p_depthImage->image,
                                            p_pointCloud,
                                            rgbTimestamp_seconds,
                                            slamSystemCameraPose2) !=
                    vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackRGBD returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
        catch (...)
        {
            rgbdObservability.recordTrackFailure();
            PublishRgbdFrontendHealth();
            LogRgbdObservabilitySummary("track_exception");
            throw;
        }
        const std::chrono::steady_clock::time_point trackEnd =
            std::chrono::steady_clock::now();
        rgbdObservability.recordTrackDuration(trackStart, trackEnd);
        rgbdObservability.recordTrackCompletion();
        if (markerTimeDifference_seconds < 0.05)
        {
            markersBuffer.clear();
        }
        double estimatorInterval_seconds = 0.0;
        if (hasProcessedRgbdPacket)
        {
            const double frameInterval_seconds =
                rgbTimestamp_seconds - lastProcessedRgbdTimestamp_seconds;
            estimatorInterval_seconds = frameInterval_seconds;
            if (frameInterval_seconds > 0.5)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "RGB-D processing cannot keep pace: estimator frames are "
                    "%.3f seconds apart.",
                    frameInterval_seconds);
            }
        }
        hasProcessedRgbdPacket             = true;
        lastProcessedRgbdTimestamp_seconds = rgbTimestamp_seconds;
        recordEstimatorFrame(estimatorInterval_seconds);

        const rclcpp::Time messageTimestamp =
            rgbdPacket.p_rgbImageMessage->header.stamp;
        const std::chrono::steady_clock::time_point publishStart =
            std::chrono::steady_clock::now();
        vs_graphs::observability::PublishTopicsTimingSink timingSink =
            rgbdObservability.publishTopicsTimingSink();
        try
        {
            publishTopics(messageTimestamp,
                          Eigen::Vector3f::Zero(),
                          p_pointCloudCameraMessage,
                          &timingSink,
                          true);
        }
        catch (...)
        {
            const std::chrono::steady_clock::time_point publishEnd =
                std::chrono::steady_clock::now();
            rgbdObservability.recordPublishTopics(publishStart, publishEnd);
            rgbdObservability.recordPublishTopicsFailure();
            PublishRgbdFrontendHealth();
            throw;
        }
        rgbdObservability.recordPublishTopics(publishStart,
                                              std::chrono::steady_clock::now());
        rgbdObservability.recordProcessed(rgbdPacket.sensorTimestampNanoseconds,
                                          workerStartTime,
                                          std::chrono::steady_clock::now());
        PublishRgbdFrontendHealth();

        if (std::chrono::steady_clock::now() >= nextSummaryDeadline)
        {
            LogRgbdObservabilitySummary("periodic");
            nextSummaryDeadline =
                std::chrono::steady_clock::now() + summaryPeriod;
        }
    }
}

void ImageGrabber::RequestStop()
{
    {
        std::lock_guard<std::mutex> packetLock(rgbdPacketMutex);
        stopRequested = true;
    }

    rgbdPacketCondition.notify_all();
}

void ImageGrabber::LogRgbdObservabilitySummary(
    const std::string &event_in) const
{
    const vs_graphs::observability::RgbdObservabilitySnapshot snapshot =
        rgbdObservability.snapshot();
    const std::string summary =
        vs_graphs::observability::formatRgbdObservabilitySummary(snapshot,
                                                                 event_in);
    RCLCPP_INFO(get_logger(), "%s", summary.c_str());
}

void ImageGrabber::PublishRgbdFrontendHealth() const
{
    if (p_slamSystem == nullptr)
    {
        return;
    }

    const vs_graphs::observability::RgbdObservabilitySnapshot snapshot =
        rgbdObservability.snapshot();
    const std::uint64_t terminalCount =
        snapshot.processedPackets + snapshot.imageConversionRejects +
        snapshot.cloudConversionRejects + snapshot.trackFailures +
        snapshot.publishTopicsFailures + snapshot.shutdownPendingDrops;
    if (p_slamSystem->updateRgbdFrontendHealth(
            snapshot.pendingStores,
            terminalCount,
            snapshot.pendingOverwrites,
            snapshot.workersInFlight > 0U,
            snapshot.lastProcessedSensorTimestampNanoseconds) !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateRgbdFrontendHealth returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

void ImageGrabber::GrabVoxbloxSkeletonGraph(
    const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs_in)
{
    /*!
     * Pass the skeleton graph to a buffer to be processed by
     * SemanticSegmentation thread
     */
    setVoxbloxSkeletonCluster(msgSkeletonGraphs_in);
}

void ImageGrabber::GrabRGBD(
    const sensor_msgs::msg::Image::ConstSharedPtr       &msgRGB_in,
    const sensor_msgs::msg::Image::ConstSharedPtr       &msgD_in,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC_in)
{
    const std::chrono::steady_clock::time_point callbackArrival =
        std::chrono::steady_clock::now();
    rgbdObservability.recordCallbackAdmission(callbackArrival);

    const double rgbTimestamp_seconds =
        rclcpp::Time(msgRGB_in->header.stamp).seconds();
    const double depthTimestamp_seconds =
        rclcpp::Time(msgD_in->header.stamp).seconds();
    const double pointCloudTimestamp_seconds =
        rclcpp::Time(msgPC_in->header.stamp).seconds();

    constexpr double maximumRgbDepthSkew_seconds = 0.010;
    const double     rgbDepthSkew_seconds =
        std::abs(rgbTimestamp_seconds - depthTimestamp_seconds);
    if (rgbDepthSkew_seconds > maximumRgbDepthSkew_seconds)
    {
        rgbdObservability.recordRgbDepthSkewReject();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of RGB-depth skew.",
            rgbDepthSkew_seconds);
        return;
    }

    constexpr double maximumCloudImageSkew_seconds = 0.033;
    const double     cloudImageSkew_seconds        = std::max(
        std::abs(rgbTimestamp_seconds - pointCloudTimestamp_seconds),
        std::abs(depthTimestamp_seconds - pointCloudTimestamp_seconds));
    if (cloudImageSkew_seconds > maximumCloudImageSkew_seconds)
    {
        rgbdObservability.recordCloudImageSkewReject();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of cloud-image "
            "skew.",
            cloudImageSkew_seconds);
        return;
    }

    bool didRejectNonMonotonic = false;
    bool didRejectShutdown     = false;
    bool didOverwritePending   = false;
    {
        std::lock_guard<std::mutex> packetLock(rgbdPacketMutex);

        if (stopRequested)
        {
            didRejectShutdown = true;
        }
        else if (hasReceivedRgbdPacket &&
                 rgbTimestamp_seconds <= lastReceivedRgbdTimestamp_seconds)
        {
            didRejectNonMonotonic = true;
        }
        else
        {
            didOverwritePending = hasPendingRgbdPacket;
            latestRgbdPacket    = {
                msgRGB_in,
                msgD_in,
                msgPC_in,
                rclcpp::Time(msgRGB_in->header.stamp).nanoseconds(),
                callbackArrival};
            hasPendingRgbdPacket              = true;
            hasReceivedRgbdPacket             = true;
            lastReceivedRgbdTimestamp_seconds = rgbTimestamp_seconds;
        }
    }

    if (didRejectShutdown)
    {
        rgbdObservability.recordShutdownAdmissionReject();
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding an RGB-D packet after shutdown "
                             "was requested.");
        return;
    }

    if (didRejectNonMonotonic)
    {
        rgbdObservability.recordNonMonotonicReject();
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic RGB-D packet.");
        return;
    }

    rgbdObservability.recordPendingStore(1U, didOverwritePending);
    PublishRgbdFrontendHealth();
    rgbdPacketCondition.notify_one();
}
