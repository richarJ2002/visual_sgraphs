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

#include "common.hpp"

#include <condition_variable>

using namespace std;

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
     * @brief       Processes only the newest available coherent sensor packet.
     *
     *              Superseded packets are intentionally replaced while tracking
     *              is busy. This bounds latency and prevents DDS queue overflow
     *              from later presenting ORB-SLAM3 with stale bursts followed
     *              by large, unpredictable timestamp discontinuities.
     */
    void ProcessRgbdPackets();

    /*!
     * @brief       Requests worker shutdown and wakes a waiting worker.
     */
    void RequestStop();

    /*!
     * @brief       Callback function to get scene segmentation results from the
     *              SemanticSegmenter module
     *
     * @param       msgSegImage_in
     *              The segmentation results from the SemanticSegmenter
     */
    void GrabSegmentation(
        const segmenter_ros::msg::SegmenterDataMsg &msgSegImage_in);

    /*!
     * @brief       Callback function to get the skeleton graph from the
     *              `voxblox` module
     *
     * @param       msgSkeletonGraphs_in
     *              The skeleton graph from the `voxblox` module
     */
    void GrabVoxbloxSkeletonGraph(
        const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);

    /*!
     * @brief       Admits the newest coherent RGB-D packet without blocking ROS
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
};

int main(int argc, char **argv)
{
    /* Init ROS node */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vs_graphs");

    /* Confirm number of arguments supplied are valid */
    if (argc > 1)
    {
        RCLCPP_WARN(node->get_logger(),
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

    std::string vocFile      = node->get_parameter("voc_file").as_string();
    std::string settingsFile = node->get_parameter("settings_file").as_string();
    std::string sysParamsFile =
        node->get_parameter("sys_params_file").as_string();

    /* Confirm the VOC file is set */
    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(),
                     "[Error] 'vocabulary' and 'settings' are not provided in "
                     "the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    /* Confirm there is a system params file set */
    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(),
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

    /* Initializing system threads and getting ready to process frames */
    const bool useSimTime = node->get_parameter("use_sim_time").as_bool();
    const bool directGazeboFluCloud =
        node->get_parameter("direct_gazebo_flu_cloud").as_bool();
    auto igb = std::make_shared<ImageGrabber>(useSimTime, directGazeboFluCloud);

    /* Declare system type */
    sensorType = ORB_SLAM3::System::RGBD;

    /* ---------------------------------------------------------------------- *
     * VSGRAPH SLAM SYSTEM INIT
     * ---------------------------------------------------------------------- */

    p_slamSystem = new ORB_SLAM3::System(vocFile,
                                         settingsFile,
                                         sysParamsFile,
                                         sensorType,
                                         enablePangolin);

    /* ---------------------------------------------------------------------- *
     * SETUP CALLBACKS
     * ---------------------------------------------------------------------- */

    /*!
     * Keep sensor admission separate from semantic callbacks. The RGB-D
     * callback only replaces a pending packet; the worker owns all expensive
     * conversion, tracking, and publication work.
     */
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::PointCloud2;

    /* Init image callback */
    const auto visualCallbackGroup = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    /* Init semantic image callback */
    const auto semanticCallbackGroup = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    /* Init voxblox skeleton callback */
    const auto skeletonCallbackGroup = node->create_callback_group(
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
    auto subImgRGB =
        std::make_shared<Subscriber<Image>>(node.get(),
                                            "/camera/rgb/image_raw",
                                            rmw_qos_profile_sensor_data,
                                            visualSubscriptionOptions);

    /* Subscribe to the image  depth points */
    auto subPointcloud =
        std::make_shared<Subscriber<PointCloud2>>(node.get(),
                                                  "/camera/depth/points",
                                                  rmw_qos_profile_sensor_data,
                                                  visualSubscriptionOptions);

    /* Subscribe to the image  depth points */
    auto subImgDepth = std::make_shared<Subscriber<Image>>(
        node.get(),
        "/camera/depth_registered/image_raw",
        rmw_qos_profile_sensor_data,
        visualSubscriptionOptions);

    /* Init sync policy */
    typedef ApproximateTime<Image, Image, PointCloud2> syncPolicy;
    auto sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10),
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
    auto subSegmentedImage =
        node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
            "/camera/color/image_segment",
            rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local(),
            [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
            { igb->GrabSegmentation(*msg); },
            semanticSubscriptionOptions);

    /* Subsriber to get skeletonized graph from the `voxblox` module */
    auto subVoxbloxSkeletonMesh =
        node->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/voxblox_skeletonizer/sparse_graph",
            1,
            [igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { igb->GrabVoxbloxSkeletonGraph(*msg); },
            skeletonSubscriptionOptions);

    /* Init image transport */
    static std::shared_ptr<image_transport::ImageTransport> image_transport =
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
    p_slamSystem->Shutdown();
    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::ProcessRgbdPackets()
{
    while (rclcpp::ok())
    {
        SynchronizedRgbdPacket rgbdPacket;

        {
            std::unique_lock<std::mutex> packetLock(rgbdPacketMutex);
            rgbdPacketCondition.wait(
                packetLock,
                [this]() { return stopRequested || hasPendingRgbdPacket; });

            if (stopRequested)
                break;

            rgbdPacket           = latestRgbdPacket;
            hasPendingRgbdPacket = false;
        }

        const double rgbTimestamp_seconds =
            rclcpp::Time(rgbdPacket.p_rgbImageMessage->header.stamp).seconds();

        cv_bridge::CvImageConstPtr p_depthImage;
        cv_bridge::CvImageConstPtr p_rgbImage;
        const auto packetStartTime = std::chrono::steady_clock::now();

        try
        {
            p_depthImage = cv_bridge::toCvShare(rgbdPacket.p_depthImageMessage);
            p_rgbImage   = cv_bridge::toCvShare(rgbdPacket.p_rgbImageMessage);
        }
        catch (const cv_bridge::Exception &exception)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding an RGB-D packet after cv_bridge failed: %s",
                exception.what());
            continue;
        }
        const auto imageConversionEndTime = std::chrono::steady_clock::now();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr        p_pointCloud;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudCameraMessage;
        std::string                                   cloudFailureReason;
        if (!preparePointCloudForTracking(rgbdPacket.p_pointCloudMessage,
                                          directGazeboFluCloud,
                                          p_pointCloud,
                                          p_pointCloudCameraMessage,
                                          cloudFailureReason))
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "Discarding point cloud: %s.",
                                 cloudFailureReason.c_str());
            continue;
        }
        const auto pointCloudConversionEndTime =
            std::chrono::steady_clock::now();

        auto         nearestMarker = findNearestMarker(rgbTimestamp_seconds);
        const double markerTimeDifference_seconds = nearestMarker.first;
        std::vector<ORB_SLAM3::Marker *> matchedMarkers =
            std::move(nearestMarker.second);

        if (markerTimeDifference_seconds < 0.05)
        {
            p_slamSystem->TrackRGBD(p_rgbImage->image,
                                    p_depthImage->image,
                                    p_pointCloud,
                                    rgbTimestamp_seconds,
                                    {},
                                    "",
                                    matchedMarkers);
            markersBuffer.clear();
        }
        else
        {
            p_slamSystem->TrackRGBD(p_rgbImage->image,
                                    p_depthImage->image,
                                    p_pointCloud,
                                    rgbTimestamp_seconds);
        }
        if (hasProcessedRgbdPacket)
        {
            const double frameInterval_seconds =
                rgbTimestamp_seconds - lastProcessedRgbdTimestamp_seconds;
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
        const auto trackingEndTime         = std::chrono::steady_clock::now();

        const rclcpp::Time messageTimestamp =
            rgbdPacket.p_rgbImageMessage->header.stamp;
        publishTopics(messageTimestamp,
                      Eigen::Vector3f::Zero(),
                      p_pointCloudCameraMessage);
        const auto publicationEndTime = std::chrono::steady_clock::now();

        const auto durationMilliseconds =
            [](const auto startTime_in, const auto endTime_in)
        {
            return std::chrono::duration<double, std::milli>(endTime_in -
                                                             startTime_in)
                .count();
        };
        RCLCPP_INFO(
            get_logger(),
            "RGB-D stages [ms]: image %.1f, cloud %.1f, tracking %.1f, "
            "publication %.1f, total %.1f.",
            durationMilliseconds(packetStartTime, imageConversionEndTime),
            durationMilliseconds(imageConversionEndTime,
                                 pointCloudConversionEndTime),
            durationMilliseconds(pointCloudConversionEndTime, trackingEndTime),
            durationMilliseconds(trackingEndTime, publicationEndTime),
            durationMilliseconds(packetStartTime, publicationEndTime));
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

void ImageGrabber::GrabSegmentation(
    const segmenter_ros::msg::SegmenterDataMsg &msgSegImage_in)
{
    /* Declare local variables */
    cv_bridge::CvImageConstPtr cv_imgSeg;

    /* Extract the kayframe id from the segmented image */
    uint64_t key_frame_id = msgSegImage_in.key_frame_id.data;

    /* Fetch the segmentation results */
    try
    {
        /* Extract the image as an open cv object */
        cv_imgSeg =
            cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(
                                    msgSegImage_in.segmented_image_uncertainty),
                                sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }

    /* Init a net PCL PointCloud object */
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);

    /* Convert to PCL PointCloud2 from `sensor_msgs` PointCloud2 */
    pcl_conversions::toPCL(msgSegImage_in.segmented_image_probability,
                           *pclPc2SegPrb);

    /* Create the tuple to be appended to the segmentedImageBuffer */
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(
        key_frame_id,
        cv_imgSeg->image,
        pclPc2SegPrb);

    /*!
     * Add segmented image to buffer to be processed in SemanticSegmentation
     * thread.
     */
    p_slamSystem->addSegmentedImage(&tuple);
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
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D packet with %.3f seconds of cloud-image "
            "skew.",
            cloudImageSkew_seconds);
        return;
    }

    {
        std::lock_guard<std::mutex> packetLock(rgbdPacketMutex);

        if (hasReceivedRgbdPacket &&
            rgbTimestamp_seconds <= lastReceivedRgbdTimestamp_seconds)
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "Discarding a non-monotonic RGB-D packet.");
            return;
        }

        latestRgbdPacket                  = {msgRGB_in, msgD_in, msgPC_in};
        hasPendingRgbdPacket              = true;
        hasReceivedRgbdPacket             = true;
        lastReceivedRgbdTimestamp_seconds = rgbTimestamp_seconds;
    }

    rgbdPacketCondition.notify_one();
}
