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

using namespace std;

class ImuGrabber : public rclcpp::Node
{
  public:
    /*!
     * @brief Constructs the IMU adapter with the parent node's clock mode.
     *
     * @param[in] useSimTime_in True when timestamps must follow `/clock`.
     */
    explicit ImuGrabber(const bool useSimTime_in) :
        rclcpp::Node(
            "imu_grabber",
            rclcpp::NodeOptions()
                .use_global_arguments(false)
                .parameter_overrides(
                    {rclcpp::Parameter("use_sim_time", useSimTime_in)}))
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    // Variables
    std::mutex                                        mBufMutex;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
};

class ImageGrabber : public rclcpp::Node
{
  public:
    /** A synchronized image pair and the closest available depth cloud. */
    struct SynchronizedRgbdPacket
    {
        sensor_msgs::msg::Image::ConstSharedPtr       p_rgbImageMessage;
        sensor_msgs::msg::Image::ConstSharedPtr       p_depthImageMessage;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
    };

    /**
     * @brief Construct the synchronized RGB-D and IMU ingestion adapter.
     *
     * @param[in] p_imuGrabber_in Shared owner of the IMU sample buffer.
     * @param[in] useSimTime_in True when timestamps must follow `/clock`.
     * @param[in] maximumTrackingRate_hz_in Maximum visual estimator rate.
     * @param[in] maximumBufferDuration_seconds_in Allowed sensor latency.
     */
    ImageGrabber(std::shared_ptr<ImuGrabber> p_imuGrabber_in,
                 bool                        useSimTime_in,
                 double                      maximumTrackingRate_hz_in,
                 double                      maximumBufferDuration_seconds_in,
                 bool                        directGazeboFluCloud_in) :
        Node("image_grabber",
             rclcpp::NodeOptions()
                 .use_global_arguments(false)
                 .parameter_overrides(
                     {rclcpp::Parameter("use_sim_time", useSimTime_in)})),
        mpImuGb(std::move(p_imuGrabber_in)),
        minimumTrackingInterval_seconds(
            1.0 / std::max(1.0, maximumTrackingRate_hz_in)),
        maximumBufferedRgbdPackets(
            static_cast<std::size_t>(
                std::max(1.0, maximumTrackingRate_hz_in) *
                std::max(1.0, maximumBufferDuration_seconds_in)) +
            1U),
        directGazeboFluCloud(directGazeboFluCloud_in)
    {
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        staticTfBroadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    // Variables
    std::mutex                         mBufMutex;
    std::atomic<bool>                  mustStop{false};
    std::shared_ptr<ImuGrabber>        mpImuGb;
    std::queue<SynchronizedRgbdPacket> synchronizedRgbdPacketBuffer;
    const double                       minimumTrackingInterval_seconds;
    const std::size_t                  maximumBufferedRgbdPackets;
    double                             lastReceivedRgbdTimestamp_seconds{0.0};
    bool                               hasReceivedRgbdPacket{false};
    double                             lastAdmittedRgbdTimestamp_seconds{0.0};
    bool                               hasAdmittedRgbdPacket{false};
    bool                               discardInputUntilBufferDrained{false};
    double                             lastProcessedRgbdTimestamp_seconds{0.0};
    bool                               hasProcessedRgbdPacket{false};
    sensor_msgs::msg::PointCloud2::ConstSharedPtr p_latestPointCloudMessage;
    const bool                                    directGazeboFluCloud;
    double                             lastConsumedImuTimestamp_seconds{0.0};
    bool                               hasConsumedImuSample{false};
    std::vector<ORB_SLAM3::IMU::Point> pendingImuMeasurements;
    double                             pendingMaximumImuGap_seconds{0.0};

    void    SyncWithImu();
    // void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void    GrabSegmentation(
           const segmenter_ros::msg::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(
        const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs);
    void GrabPointCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC);
    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msgD);
};

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);

    if (!imuBuf.empty() && rclcpp::Time(imu_msg->header.stamp) <=
                               rclcpp::Time(imuBuf.back()->header.stamp))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic IMU sample.");
        return;
    }

    constexpr std::size_t maximumBufferedImuSamples = 2500;
    if (imuBuf.size() >= maximumBufferedImuSamples)
    {
        imuBuf.pop();
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "IMU buffer overflow; discarding oldest sample.");
    }

    imuBuf.push(imu_msg);
}

cv::Mat ImageGrabber::GetImage(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[Error] Problem occured while running `cv_bridge`: %s",
                     e.what());
        return cv::Mat();
    }

    return cv_ptr->image.clone();
}

/**
 * @brief Callback function to get scene segmentation results from the
 * SemanticSegmenter module
 *
 * @param msgSegImage The segmentation results from the SemanticSegmenter
 */
void ImageGrabber::GrabSegmentation(
    const segmenter_ros::msg::SegmenterDataMsg &msgSegImage)
{
    // Fetch the segmentation results
    cv_bridge::CvImageConstPtr cv_imgSeg;
    uint64_t                   key_frame_id = msgSegImage.key_frame_id.data;

    try
    {
        cv_imgSeg =
            cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(
                                    msgSegImage.segmented_image_uncertainty),
                                sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        RCLCPP_ERROR(this->get_logger(),
                     "[Error] `cv_bridge` exception: %s",
                     e.what());
        return;
    }

    // Convert to PCL PointCloud2 from `sensor_msgs` PointCloud2
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmented_image_probability,
                           *pclPc2SegPrb);

    // Create the tuple to be appended to the segmentedImageBuffer
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(
        key_frame_id,
        cv_imgSeg->image,
        pclPc2SegPrb);

    // Add the segmented image to a buffer to be processed in the
    // SemanticSegmentation thread
    p_slamSystem->addSegmentedImage(&tuple);
}

void ImageGrabber::SyncWithImu()
{
    if (!mpImuGb)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[Error] IMU Grabber not initialized!");
        return;
    }

    while (!mustStop)
    {
        sensor_msgs::msg::Image::ConstSharedPtr       p_rgbImageMessage;
        sensor_msgs::msg::Image::ConstSharedPtr       p_depthImageMessage;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
        std::vector<ORB_SLAM3::IMU::Point>            imuMeasurements;
        Eigen::Vector3f angularVelocity_body_radPerSec =
            Eigen::Vector3f::Zero();
        double imageTimestamp_seconds     = 0.0;
        double latestImuTimestamp_seconds = 0.0;
        bool   waitingForImu              = false;
        double maximumImuGap_seconds      = 0.0;
        bool   imuContinuityOverflow      = false;

        /* Transfer one synchronized sensor packet while holding both buffer
         * mutexes. Keeping every queue access inside this critical section
         * prevents the ROS executor from replacing a frame while this thread
         * is reading it. */
        {
            std::scoped_lock bufferLock(mBufMutex, mpImuGb->mBufMutex);

            if (!synchronizedRgbdPacketBuffer.empty() &&
                !mpImuGb->imuBuf.empty())
            {
                const SynchronizedRgbdPacket &rgbdPacket =
                    synchronizedRgbdPacketBuffer.front();
                imageTimestamp_seconds =
                    rclcpp::Time(rgbdPacket.p_rgbImageMessage->header.stamp)
                        .seconds();
                latestImuTimestamp_seconds =
                    rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp)
                        .seconds();
                waitingForImu =
                    imageTimestamp_seconds > latestImuTimestamp_seconds;

                if (!waitingForImu)
                {
                    p_rgbImageMessage   = rgbdPacket.p_rgbImageMessage;
                    p_depthImageMessage = rgbdPacket.p_depthImageMessage;
                    p_pointCloudMessage = rgbdPacket.p_pointCloudMessage;
                    synchronizedRgbdPacketBuffer.pop();

                    while (!mpImuGb->imuBuf.empty() &&
                           rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp)
                                   .seconds() <= imageTimestamp_seconds)
                    {
                        const auto  &p_imuMessage = mpImuGb->imuBuf.front();
                        const double imuTimestamp_seconds =
                            rclcpp::Time(p_imuMessage->header.stamp).seconds();
                        if (hasConsumedImuSample)
                        {
                            pendingMaximumImuGap_seconds =
                                std::max(pendingMaximumImuGap_seconds,
                                         imuTimestamp_seconds -
                                             lastConsumedImuTimestamp_seconds);
                        }
                        const cv::Point3f acceleration_body_mPerSec2(
                            p_imuMessage->linear_acceleration.x,
                            p_imuMessage->linear_acceleration.y,
                            p_imuMessage->linear_acceleration.z);
                        const cv::Point3f angularVelocity_body_radPerSecCv(
                            p_imuMessage->angular_velocity.x,
                            p_imuMessage->angular_velocity.y,
                            p_imuMessage->angular_velocity.z);

                        constexpr std::size_t maximumPendingImuSamples = 2500U;
                        if (pendingImuMeasurements.size() >=
                            maximumPendingImuSamples)
                        {
                            pendingImuMeasurements.clear();
                            pendingMaximumImuGap_seconds = 0.0;
                            hasConsumedImuSample         = false;
                            imuContinuityOverflow        = true;
                        }
                        pendingImuMeasurements.emplace_back(
                            acceleration_body_mPerSec2,
                            angularVelocity_body_radPerSecCv,
                            imuTimestamp_seconds);
                        angularVelocity_body_radPerSec
                            << p_imuMessage->angular_velocity.x,
                            p_imuMessage->angular_velocity.y,
                            p_imuMessage->angular_velocity.z;
                        mpImuGb->imuBuf.pop();
                        lastConsumedImuTimestamp_seconds = imuTimestamp_seconds;
                        hasConsumedImuSample             = true;
                    }

                    if (hasConsumedImuSample)
                    {
                        maximumImuGap_seconds =
                            std::max(pendingMaximumImuGap_seconds,
                                     imageTimestamp_seconds -
                                         lastConsumedImuTimestamp_seconds);
                    }
                    imuMeasurements = pendingImuMeasurements;
                }
            }
        }

        if (!p_rgbImageMessage)
        {
            if (waitingForImu)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for IMU data: RGB time %.3f is ahead of the "
                    "latest IMU time %.3f.",
                    imageTimestamp_seconds,
                    latestImuTimestamp_seconds);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        constexpr double maximumImuGapAllowed_seconds = 0.010;
        if (imuContinuityOverflow ||
            maximumImuGap_seconds > maximumImuGapAllowed_seconds)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding an inertial RGB-D frame: IMU continuity failed "
                "(gap %.4f seconds, %zu buffered samples).",
                maximumImuGap_seconds,
                imuMeasurements.size());
            pendingImuMeasurements.clear();
            pendingMaximumImuGap_seconds = 0.0;
            hasConsumedImuSample         = false;
            p_slamSystem->ResetActiveMap();
            continue;
        }
        if (imuMeasurements.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "Retaining an RGB-D frame interval until IMU "
                                 "samples are available.");
            continue;
        }

        const char *processingStage = "image conversion";
        try
        {
            const rclcpp::Time messageTimestamp =
                p_rgbImageMessage->header.stamp;
            const cv::Mat rgbImage   = GetImage(p_rgbImageMessage);
            const cv::Mat depthImage = GetImage(p_depthImageMessage);

            if (rgbImage.empty() || depthImage.empty())
            {
                RCLCPP_ERROR_THROTTLE(get_logger(),
                                      *get_clock(),
                                      5000,
                                      "Discarding an RGB-D frame because image "
                                      "conversion returned an empty matrix.");
                continue;
            }

            processingStage = "point-cloud conversion";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud;
            sensor_msgs::msg::PointCloud2::ConstSharedPtr
                        p_pointCloudCameraMessage;
            std::string cloudFailureReason;
            if (!preparePointCloudForTracking(p_pointCloudMessage,
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

            processingStage    = "marker association";
            auto nearestMarker = findNearestMarker(imageTimestamp_seconds);
            const double markerTimeDifference_seconds = nearestMarker.first;
            std::vector<ORB_SLAM3::Marker *> matchedMarkers =
                std::move(nearestMarker.second);

            processingStage = "inertial RGB-D tracking";
            if (markerTimeDifference_seconds < 0.05)
            {
                p_slamSystem->TrackRGBD(rgbImage,
                                        depthImage,
                                        p_pointCloud,
                                        imageTimestamp_seconds,
                                        imuMeasurements,
                                        "",
                                        matchedMarkers);
                markersBuffer.clear();
            }
            else
            {
                p_slamSystem->TrackRGBD(rgbImage,
                                        depthImage,
                                        p_pointCloud,
                                        imageTimestamp_seconds,
                                        imuMeasurements);
            }

            if (hasProcessedRgbdPacket)
            {
                const double processedFrameInterval_seconds =
                    imageTimestamp_seconds - lastProcessedRgbdTimestamp_seconds;
                if (processedFrameInterval_seconds > 0.5)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Estimator input discontinuity: consecutive RGB-D "
                        "timestamps differ by %.3f seconds.",
                        processedFrameInterval_seconds);
                }
            }
            lastProcessedRgbdTimestamp_seconds = imageTimestamp_seconds;
            hasProcessedRgbdPacket             = true;

            /* Only a completed TrackRGBD call commits this IMU interval. A
             * rejected image or cloud therefore leaves every sample available
             * for the next accepted visual frame. */
            pendingImuMeasurements.clear();
            pendingMaximumImuGap_seconds = 0.0;

            processingStage = "ROS publication";
            publishTopics(messageTimestamp,
                          angularVelocity_body_radPerSec,
                          p_pointCloudCameraMessage);
        }
        catch (const std::exception &exception)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Discarding a synchronized sensor frame after "
                "an exception during %s: %s",
                processingStage,
                exception.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vs_graphs");

    if (argc > 1)
        RCLCPP_WARN(node->get_logger(),
                    "Arguments supplied via command line are ignored.");

    std::string nodeName = node->get_name();

    // Parameters
    node->declare_parameter<double>("yaw", 0.0);
    node->declare_parameter<double>("roll", 0.0);
    node->declare_parameter<double>("pitch", 0.0);
    node->declare_parameter<bool>("enable_pangolin", true);
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
        RCLCPP_ERROR(node->get_logger(),
                     "[Error] 'vocabulary' and 'settings' are not provided in "
                     "the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(),
                     "[Error] The `YAML` file containing system parameters is "
                     "not provided in the launch file! Exiting...");
        rclcpp::shutdown();
        return 1;
    }

    yaw             = node->get_parameter("yaw").as_double();
    roll            = node->get_parameter("roll").as_double();
    pitch           = node->get_parameter("pitch").as_double();
    frameImu        = node->get_parameter("frame_imu").as_string();
    frameMap        = node->get_parameter("frame_map").as_string();
    frameWorld      = node->get_parameter("frame_world").as_string();
    frameCamera     = node->get_parameter("frame_camera").as_string();
    colorPointcloud = node->get_parameter("colored_pointcloud").as_bool();
    pubPointClouds  = node->get_parameter("publish_pointclouds").as_bool();
    frameBC = node->get_parameter("frame_building_component").as_string();
    frameSE = node->get_parameter("frame_structural_element").as_string();
    pubStaticTransform  = node->get_parameter("static_transform").as_bool();
    bool enablePangolin = node->get_parameter("enable_pangolin").as_bool();

    const double maximumTrackingRate_hz =
        node->get_parameter("maximum_tracking_rate_hz").as_double();
    const double maximumSensorBuffer_seconds =
        node->get_parameter("maximum_sensor_buffer_seconds").as_double();
    const bool directGazeboFluCloud =
        node->get_parameter("direct_gazebo_flu_cloud").as_bool();

    if (maximumTrackingRate_hz <= 0.0 || maximumSensorBuffer_seconds <= 0.0)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Sensor admission parameters must be positive.");
        rclcpp::shutdown();
        return 1;
    }

    // Initializing system threads and getting ready to process frames
    const bool useSimTime = node->get_parameter("use_sim_time").as_bool();
    auto       imugb      = std::make_shared<ImuGrabber>(useSimTime);
    auto       igb        = std::make_shared<ImageGrabber>(imugb,
                                              useSimTime,
                                              maximumTrackingRate_hz,
                                              maximumSensorBuffer_seconds,
                                              directGazeboFluCloud);

    sensorType   = ORB_SLAM3::System::IMU_RGBD;
    p_slamSystem = new ORB_SLAM3::System(vocFile,
                                         settingsFile,
                                         sysParamsFile,
                                         sensorType,
                                         enablePangolin);

    // Subscribe to get raw images (message_filters in ROS2)
    using message_filters::Subscriber;
    using message_filters::Synchronizer;
    using message_filters::sync_policies::ApproximateTime;
    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;
    using sensor_msgs::msg::PointCloud2;

    // Subscriber to IMU data with reliable QoS
    rclcpp::QoS imu_qos(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    /* Keep high-rate sensor ingestion independent from semantic callbacks.
     * A single mutually-exclusive callback group allowed point-cloud and
     * segmentation work to starve RGB-D and IMU delivery, which is fatal to
     * inertial preintegration even when every source topic is healthy. */
    const auto imuCallbackGroup = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto visualCallbackGroup = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto semanticCallbackGroup = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto skeletonCallbackGroup = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions imuSubscriptionOptions;
    imuSubscriptionOptions.callback_group = imuCallbackGroup;
    rclcpp::SubscriptionOptions visualSubscriptionOptions;
    visualSubscriptionOptions.callback_group = visualCallbackGroup;
    rclcpp::SubscriptionOptions semanticSubscriptionOptions;
    semanticSubscriptionOptions.callback_group = semanticCallbackGroup;
    rclcpp::SubscriptionOptions skeletonSubscriptionOptions;
    skeletonSubscriptionOptions.callback_group = skeletonCallbackGroup;

    auto subImu = node->create_subscription<Imu>(
        "/imu",
        imu_qos,
        [imugb](const Imu::ConstSharedPtr msg) { imugb->GrabImu(msg); },
        imuSubscriptionOptions);

    auto subImgRGB =
        std::make_shared<Subscriber<Image>>(node.get(),
                                            "/camera/rgb/image_raw",
                                            rmw_qos_profile_sensor_data,
                                            visualSubscriptionOptions);
    auto subImgDepth = std::make_shared<Subscriber<Image>>(
        node.get(),
        "/camera/depth_registered/image_raw",
        rmw_qos_profile_sensor_data,
        visualSubscriptionOptions);

    auto subPointcloud = node->create_subscription<PointCloud2>(
        "/camera/depth/points",
        rclcpp::SensorDataQoS(),
        [igb](const PointCloud2::ConstSharedPtr msg)
        { igb->GrabPointCloud(msg); },
        visualSubscriptionOptions);

    typedef ApproximateTime<Image, Image> syncPolicy;
    auto sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10),
                                                           *subImgRGB,
                                                           *subImgDepth);
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.010));
    sync->registerCallback(std::bind(&ImageGrabber::GrabRGBD,
                                     igb.get(),
                                     std::placeholders::_1,
                                     std::placeholders::_2));

    // Subscriber to get segmentation results from the SemanticSegmenter module
    auto subSegmentedImage =
        node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
            "/camera/color/image_segment",
            rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local(),
            [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
            { igb->GrabSegmentation(*msg); },
            semanticSubscriptionOptions);

    // Subsriber to get skeletonized graph from the `voxblox` module
    auto subVoxbloxSkeletonMesh =
        node->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/voxblox_skeletonizer/sparse_graph",
            1,
            [igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            { igb->GrabVoxbloxSkeletonGraph(*msg); },
            skeletonSubscriptionOptions);

    static std::shared_ptr<image_transport::ImageTransport> image_transport =
        std::make_shared<image_transport::ImageTransport>(node);
    setupPublishers(node, image_transport, nodeName);
    setupServices(node, nodeName);

    // Syncing images with IMU
    std::thread sync_thread(&ImageGrabber::SyncWithImu, igb);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      4U);
    executor.add_node(node);
    executor.add_node(imugb);
    executor.add_node(igb);
    executor.spin();

    // Signal the sync thread to stop and wait for it
    igb->mustStop = true;
    sync_thread.join();

    // No sensor worker may call TrackRGBD while SLAM threads are stopping.
    p_slamSystem->Shutdown();

    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::GrabPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPC)
{
    std::lock_guard<std::mutex> lock(mBufMutex);

    if (p_latestPointCloudMessage &&
        rclcpp::Time(msgPC->header.stamp) <=
            rclcpp::Time(p_latestPointCloudMessage->header.stamp))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic point cloud.");
        return;
    }

    p_latestPointCloudMessage = msgPC;
}

void ImageGrabber::GrabRGBD(
    const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
    const sensor_msgs::msg::Image::ConstSharedPtr &msgD)
{
    std::lock_guard<std::mutex> lock(mBufMutex);

    if (!p_latestPointCloudMessage)
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Waiting for the first point cloud.");
        return;
    }

    const auto p_pointCloudMessage = p_latestPointCloudMessage;

    if (discardInputUntilBufferDrained)
    {
        if (!synchronizedRgbdPacketBuffer.empty())
            return;

        {
            std::lock_guard<std::mutex> imuBufferLock(mpImuGb->mBufMutex);
            std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> emptyImuBuffer;
            mpImuGb->imuBuf.swap(emptyImuBuffer);
        }

        p_slamSystem->ResetActiveMap();
        discardInputUntilBufferDrained = false;
        hasAdmittedRgbdPacket          = false;

        RCLCPP_WARN(get_logger(),
                    "Restarting the active map after a sustained sensor "
                    "processing overload.");
        return;
    }

    const double rgbTimestamp_seconds =
        rclcpp::Time(msgRGB->header.stamp).seconds();
    const double depthTimestamp_seconds =
        rclcpp::Time(msgD->header.stamp).seconds();
    const double pointCloudTimestamp_seconds =
        rclcpp::Time(p_pointCloudMessage->header.stamp).seconds();

    if (hasReceivedRgbdPacket)
    {
        const double receivedFrameInterval_seconds =
            rgbTimestamp_seconds - lastReceivedRgbdTimestamp_seconds;
        if (receivedFrameInterval_seconds > 0.5)
        {
            RCLCPP_WARN(get_logger(),
                        "RGB-D synchronizer input discontinuity: consecutive "
                        "image pairs differ by %.3f seconds.",
                        receivedFrameInterval_seconds);
        }
    }
    lastReceivedRgbdTimestamp_seconds = rgbTimestamp_seconds;
    hasReceivedRgbdPacket             = true;

    constexpr double maximumImageTimestampSkew_seconds = 0.010;
    if (std::abs(rgbTimestamp_seconds - depthTimestamp_seconds) >
        maximumImageTimestampSkew_seconds)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding an RGB-D image pair whose timestamps differ by "
            "%.3f seconds.",
            std::abs(rgbTimestamp_seconds - depthTimestamp_seconds));
        return;
    }

    constexpr double maximumPointCloudAge_seconds = 0.033;
    const double     cloudImageSkew_seconds       = std::max(
        std::abs(rgbTimestamp_seconds - pointCloudTimestamp_seconds),
        std::abs(depthTimestamp_seconds - pointCloudTimestamp_seconds));
    if (cloudImageSkew_seconds > maximumPointCloudAge_seconds)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Discarding a point cloud %.3f seconds from the RGB-D image time.",
            cloudImageSkew_seconds);
        return;
    }

    if (hasAdmittedRgbdPacket &&
        rgbTimestamp_seconds <= lastAdmittedRgbdTimestamp_seconds)
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Discarding a non-monotonic RGB-D packet.");
        return;
    }

    /*!
     * Gazebo publishes a 30 Hz sensor on discrete simulation steps, so a
     * nominal 33.333 ms period can alternate between 33 and 34 ms. Admit that
     * bounded quantization without allowing a genuinely faster stream through
     * the configured tracking-rate limit.
     */
    const double trackingIntervalTolerance_seconds =
        std::min(1e-3, 0.05 * minimumTrackingInterval_seconds);

    if (hasAdmittedRgbdPacket && rgbTimestamp_seconds -
                                         lastAdmittedRgbdTimestamp_seconds +
                                         trackingIntervalTolerance_seconds <
                                     minimumTrackingInterval_seconds)
    {
        return;
    }

    /* Buffer several consecutive frames so expensive first-map construction
     * cannot turn normal camera traffic into an artificial timestamp jump.
     * New frames are rejected at the latency boundary so already-admitted
     * estimator input remains chronological and gap-free.
     */
    if (synchronizedRgbdPacketBuffer.size() >= maximumBufferedRgbdPackets)
    {
        discardInputUntilBufferDrained = true;
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "RGB-D processing exceeded the %.1f-second latency budget; "
            "discarding new packets while the estimator catches up.",
            maximumBufferedRgbdPackets * minimumTrackingInterval_seconds);
        return;
    }

    synchronizedRgbdPacketBuffer.push({msgRGB, msgD, p_pointCloudMessage});
    lastAdmittedRgbdTimestamp_seconds = rgbTimestamp_seconds;
    hasAdmittedRgbdPacket             = true;
}

/**
 * @brief Callback function to get the markers detected by the `aruco_ros`
 * library
 *
 * @param msgMarkerArray The markers detected by the `aruco_ros` library
 */
// void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray
// &msgMarkerArray)
// {
//     // Pass the visited markers to a buffer to be processed later
//     addMarkersToBuffer(msgMarkerArray);
// }

/**
 * @brief Callback function to get the skeleton graph from the `voxblox` module
 *
 * @param msgSkeletonGraphs The skeleton graph from the `voxblox` module
 */
void ImageGrabber::GrabVoxbloxSkeletonGraph(
    const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs)
{
    // Pass the skeleton graph to a buffer to be processed by the
    // SemanticSegmentation thread
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}
