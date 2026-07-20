/*!
 * @File:         setupPublishers.cpp
 *
 * @Brief:        Initialises the ROS publishers and TF listener used by the
 *                Visual S-Graphs interface.
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

    const rclcpp::QoS keyFrameImagePublisherQoS(rclcpp::KeepLast(50));

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

    pubWorldFramePointCloud =
        node_in->create_publisher<sensor_msgs::msg::PointCloud2>(
            makeTopicName("points_map"),
            standardPublisherQoS);

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
