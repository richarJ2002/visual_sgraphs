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

#include "common/common.h"
#include "image_grabber/image_grabber.h"
#include "observability/observability.h"

#include <condition_variable>
#include <rclcpp/logging.hpp>

/*!
 * @brief           Starts the RGB-D node: reads parameters, creates the SLAM
 *                  system, wires the publishers, services and subscriptions,
 *                  starts the RGB-D processing thread and spins a three-thread
 *                  executor until shutdown.
 *
 * @param[in]       argc
 *                  Number of command-line arguments; they are ignored with a
 *                  warning.
 *
 * @param[in]       argv
 *                  Command-line arguments.
 *
 * @return          0 after a normal shutdown; 1 when the vocabulary, settings
 *                  or system-parameter file is not given; -1 when the SLAM
 *                  system cannot initialise.
 */
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
    vs_graphs::common::yaw      = node->get_parameter("yaw").as_double();
    vs_graphs::common::roll     = node->get_parameter("roll").as_double();
    vs_graphs::common::pitch    = node->get_parameter("pitch").as_double();
    vs_graphs::common::frameMap = node->get_parameter("frame_map").as_string();
    vs_graphs::common::frameWorld =
        node->get_parameter("frame_world").as_string();
    vs_graphs::common::frameCamera =
        node->get_parameter("frame_camera").as_string();
    vs_graphs::common::colorPointcloud =
        node->get_parameter("colored_pointcloud").as_bool();
    vs_graphs::common::pubPointClouds =
        node->get_parameter("publish_pointclouds").as_bool();
    vs_graphs::common::frameBC =
        node->get_parameter("frame_building_component").as_string();
    vs_graphs::common::frameSE =
        node->get_parameter("frame_structural_element").as_string();
    vs_graphs::common::pubStaticTransform =
        node->get_parameter("static_transform").as_bool();
    vs_graphs::core::system::Verbose::VerbosityLevel verboseLevel{};
    if (vs_graphs::core::system::Verbose::parseVerbosityLevel(
            node->get_parameter("log_level").as_string(),
            verboseLevel) !=
        vs_graphs::core::system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: parseVerbosityLevel returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    vs_graphs::common::sgraphArchiveTestRunDir =
        node->get_parameter("test_run_dir").as_string();
    vs_graphs::common::sgraphArchiveEnabled =
        node->get_parameter("sgraph_archive_enabled").as_bool();
    vs_graphs::common::sgraphArchiveIntervalSec =
        node->get_parameter("sgraph_archive_interval_sec").as_double();
    vs_graphs::common::sgraphArchiveMaxFiles = static_cast<int>(
        node->get_parameter("sgraph_archive_max_files").as_int());

    /* Initializing system threads and getting ready to process frames */
    const bool useSimTime = node->get_parameter("use_sim_time").as_bool();
    const bool directGazeboFluCloud =
        node->get_parameter("direct_gazebo_flu_cloud").as_bool();
    std::shared_ptr<vs_graphs::image_grabber::RgbdImageGrabber> igb =
        std::make_shared<vs_graphs::image_grabber::RgbdImageGrabber>(
            useSimTime,
            directGazeboFluCloud);

    /* Declare system type */
    vs_graphs::common::sensorType = vs_graphs::core::system::System::RGBD;

    /* ---------------------------------------------------------------------- *
     * VSGRAPH SLAM SYSTEM INIT
     * ---------------------------------------------------------------------- */

    vs_graphs::common::p_slamSystem = new vs_graphs::core::system::System();
    const vs_graphs::core::system::SystemStatus systemStatus =
        vs_graphs::common::p_slamSystem->initialize(
            vocFile,
            settingsFile,
            sysParamsFile,
            vs_graphs::common::sensorType,
            verboseLevel);
    if (systemStatus !=
        vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
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
    sync->registerCallback(
        std::bind(&vs_graphs::image_grabber::RgbdImageGrabber::grabRgbd,
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
                {
                    vs_graphs::common::addSegmentationToSystem(
                        *msg,
                        igb->get_logger());
                },
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
                    igb->grabVoxbloxSkeletonGraph(*msg);
                    vs_graphs::common::observeVoxbloxSparseGraphPublication(
                        *msg);
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
                { vs_graphs::common::observeVoxbloxSkeletonPublication(*msg); },
                skeletonSubscriptionOptions);

    /* Init image transport */
    std::shared_ptr<image_transport::ImageTransport> image_transport =
        std::make_shared<image_transport::ImageTransport>(node);

    /* ---------------------------------------------------------------------- *
     * PUBLISHERS & SERVICES
     * ---------------------------------------------------------------------- */

    /* Setup publishers for system */
    vs_graphs::common::setupPublishers(node, image_transport, nodeName);

    /* Setup services for system */
    vs_graphs::common::setupServices(node, nodeName);

    /* ---------------------------------------------------------------------- *
     * SENSOR PROCESSING THREAD
     * ---------------------------------------------------------------------- */

    /* Create an rgbd processing thread */
    std::thread rgbdProcessingThread(
        &vs_graphs::image_grabber::RgbdImageGrabber::processRgbdPackets,
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

    igb->requestStop();
    rgbdProcessingThread.join();
    igb->logRgbdObservabilitySummary("shutdown_after_worker_join");
    if (vs_graphs::common::p_slamSystem->shutdown() !=
        vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: shutdown returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    delete vs_graphs::common::p_slamSystem;
    vs_graphs::common::p_slamSystem = nullptr;
    vs_graphs::common::shutdownRosInterfaces();
    image_transport.reset();
    rclcpp::shutdown();
    return 0;
}
