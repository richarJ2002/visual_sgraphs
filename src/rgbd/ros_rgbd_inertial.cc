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
 * @file            ros_rgbd_inertial.cc
 *
 * @brief           The RGB-D and IMU ROS 2 node: pairs images with IMU samples,
 *                  feeds them to the SLAM system and publishes its results.
 */

#include "common/common.h"
#include "image_grabber/image_grabber.h"
#include "imu_grabber/imu_grabber.h"
#include "reset_cause/reset_cause.h"
#include <rclcpp/logging.hpp>

/*!
 * @brief           Starts the RGB-D plus IMU node: reads parameters, creates
 *                  the SLAM system, wires the subscriptions and spins a
 *                  four-thread executor until shutdown.
 *
 * @param[in]       argc
 *                  Number of command-line arguments; they are ignored with a
 *                  warning.
 *
 * @param[in]       argv
 *                  Command-line arguments.
 *
 * @return          0 after a normal shutdown; 1 when the vocabulary, settings
 *                  or system-parameter file is not given or a tracking-rate or
 *                  buffer parameter is not positive; -1 when the SLAM system
 *                  cannot initialise.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node =
        std::make_shared<rclcpp::Node>("vs_graphs");

    if (argc > 1)
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Arguments supplied via command line are ignored.");

    std::string nodeName = node->get_name();

    /* Parameters */
    node->declare_parameter<double>("yaw", 0.0);
    node->declare_parameter<double>("roll", 0.0);
    node->declare_parameter<double>("pitch", 0.0);
    node->declare_parameter<bool>("static_transform", false);
    node->declare_parameter<std::string>("frame_imu", "imu");
    node->declare_parameter<std::string>("frame_map", "map");
    node->declare_parameter<bool>("colored_pointcloud", true);
    node->declare_parameter<bool>("publish_pointclouds", true);
    node->declare_parameter<std::string>("frame_world", "world");
    node->declare_parameter<std::string>("frame_camera", "camera");
    node->declare_parameter<std::string>("voc_file", "file_not_set");
    node->declare_parameter<std::string>("settings_file", "file_not_set");
    node->declare_parameter<std::string>("sys_params_file", "file_not_set");
    node->declare_parameter<double>("maximum_tracking_rate_hz", 30.0);
    node->declare_parameter<double>("maximum_sensor_buffer_seconds", 3.0);
    node->declare_parameter<bool>("direct_gazebo_flu_cloud", false);
    node->declare_parameter<std::string>("log_level", "info");
    node->declare_parameter<std::string>("test_run_dir", "");
    node->declare_parameter<bool>("sgraph_archive_enabled", true);
    node->declare_parameter<double>("sgraph_archive_interval_sec", 5.0);
    node->declare_parameter<int>("sgraph_archive_max_files", 0);
    node->declare_parameter<std::string>("frame_structural_element",
                                         "struc_elem");
    node->declare_parameter<std::string>("frame_building_component",
                                         "build_comp");

    std::string vocFile      = node->get_parameter("voc_file").as_string();
    std::string settingsFile = node->get_parameter("settings_file").as_string();
    std::string sysParamsFile =
        node->get_parameter("sys_params_file").as_string();

    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "[Error] 'vocabulary' and 'settings' are not provided in "
                     "the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "[Error] The `YAML` file containing system parameters is "
                     "not provided in the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    vs_graphs::common::yaw      = node->get_parameter("yaw").as_double();
    vs_graphs::common::roll     = node->get_parameter("roll").as_double();
    vs_graphs::common::pitch    = node->get_parameter("pitch").as_double();
    vs_graphs::common::frameImu = node->get_parameter("frame_imu").as_string();
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

    const double maximumTrackingRate_hz =
        node->get_parameter("maximum_tracking_rate_hz").as_double();
    const double maximumSensorBuffer_seconds =
        node->get_parameter("maximum_sensor_buffer_seconds").as_double();
    const bool directGazeboFluCloud =
        node->get_parameter("direct_gazebo_flu_cloud").as_bool();

    if (maximumTrackingRate_hz <= 0.0 || maximumSensorBuffer_seconds <= 0.0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Sensor admission parameters must be positive.");
        rclcpp::shutdown();
        return 1;
    }

    /* Initializing system threads and getting ready to process frames */
    const bool useSimTime = node->get_parameter("use_sim_time").as_bool();
    std::shared_ptr<vs_graphs::imu_grabber::ImuGrabber> imugb =
        std::make_shared<vs_graphs::imu_grabber::ImuGrabber>(useSimTime);
    std::shared_ptr<vs_graphs::image_grabber::ImageGrabber> igb =
        std::make_shared<vs_graphs::image_grabber::ImageGrabber>(
            imugb,
            useSimTime,
            maximumTrackingRate_hz,
            maximumSensorBuffer_seconds,
            directGazeboFluCloud);

    vs_graphs::common::sensorType   = vs_graphs::core::system::System::IMU_RGBD;
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

    /* Subscribe to get raw images (message_filters in ROS2) */
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;
    using sensor_msgs::msg::PointCloud2;

    /* Subscriber to IMU data with reliable QoS */
    rclcpp::QoS imu_qos(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    /* Keep high-rate sensor ingestion independent from semantic callbacks.
     * A single mutually-exclusive callback group allowed point-cloud and
     * segmentation work to starve RGB-D and IMU delivery, which is fatal to
     * inertial preintegration even when every source topic is healthy. */
    const rclcpp::CallbackGroup::SharedPtr imuCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    const rclcpp::CallbackGroup::SharedPtr visualCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    const rclcpp::CallbackGroup::SharedPtr semanticCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    const rclcpp::CallbackGroup::SharedPtr skeletonCallbackGroup =
        node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions imuSubscriptionOptions;
    imuSubscriptionOptions.callback_group = imuCallbackGroup;
    rclcpp::SubscriptionOptions visualSubscriptionOptions;
    visualSubscriptionOptions.callback_group = visualCallbackGroup;
    rclcpp::SubscriptionOptions semanticSubscriptionOptions;
    semanticSubscriptionOptions.callback_group = semanticCallbackGroup;
    rclcpp::SubscriptionOptions skeletonSubscriptionOptions;
    skeletonSubscriptionOptions.callback_group = skeletonCallbackGroup;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> subImu =
        node->create_subscription<Imu>(
            "/imu",
            imu_qos,
            [imugb](const Imu::ConstSharedPtr msg)
            { imugb->getImuSample(msg); },
            imuSubscriptionOptions);

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
        subImgRGB =
            std::make_shared<Subscriber<Image>>(node.get(),
                                                "/camera/rgb/image_raw",
                                                rmw_qos_profile_sensor_data,
                                                visualSubscriptionOptions);
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
        subImgDepth = std::make_shared<Subscriber<Image>>(
            node.get(),
            "/camera/depth_registered/image_raw",
            rmw_qos_profile_sensor_data,
            visualSubscriptionOptions);

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
        subPointcloud = node->create_subscription<PointCloud2>(
            "/camera/depth/points",
            rclcpp::SensorDataQoS(),
            [igb](const PointCloud2::ConstSharedPtr msg)
            { igb->grabPointCloud(msg); },
            visualSubscriptionOptions);

    typedef ApproximateTime<Image, Image> syncPolicy;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::
            ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10),
                                                          *subImgRGB,
                                                          *subImgDepth);
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.010));
    sync->registerCallback(
        std::bind(&vs_graphs::image_grabber::ImageGrabber::grabRgbd,
                  igb.get(),
                  std::placeholders::_1,
                  std::placeholders::_2));

    /*!
     * Subscriber to get segmentation results from the SemanticSegmenter module
     */
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

    /*!
     * Subsriber to get skeletonized graph from the `voxblox` module
     * Match the skeletonizer transient-local publisher so the latest usable
     * graph is received even when this node joins after publication.
     */
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

    /*!
     * Match the skeletonizer transient-local publisher so the latest usable
     * cloud is received even when this node joins after publication.
     */
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
        subVoxbloxSkeleton =
            node->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/voxblox_skeletonizer/skeleton",
                rclcpp::QoS(1).transient_local(),
                [](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                { vs_graphs::common::observeVoxbloxSkeletonPublication(*msg); },
                skeletonSubscriptionOptions);

    static std::shared_ptr<image_transport::ImageTransport> image_transport =
        std::make_shared<image_transport::ImageTransport>(node);
    vs_graphs::common::setupPublishers(node, image_transport, nodeName);
    vs_graphs::common::setupServices(node, nodeName);

    /* Syncing images with IMU */
    std::thread sync_thread(
        &vs_graphs::image_grabber::ImageGrabber::syncWithImu,
        igb);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      4U);
    executor.add_node(node);
    executor.add_node(imugb);
    executor.add_node(igb);
    executor.spin();

    /* Signal the sync thread to stop and wait for it */
    igb->mustStop = true;
    sync_thread.join();

    /* No sensor worker may call TrackRGBD while SLAM threads are stopping. */
    if (vs_graphs::common::p_slamSystem->shutdown() !=
        vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: shutdown returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    rclcpp::shutdown();

    return 0;
}
