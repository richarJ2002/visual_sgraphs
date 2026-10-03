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

#include "ResetCause.h"
#include "common.hpp"
#include <rclcpp/logging.hpp>

/*!
 * @brief           ROS node that buffers incoming IMU samples until the
 *                  synchronisation thread consumes them.
 */
class ImuGrabber : public rclcpp::Node
{
  public:
    /*!
     * @brief           Constructs the IMU adapter with the parent node's clock
     *                  mode.
     *
     * @param[in]       useSimTime_in
     *                  True when timestamps must follow `/clock`.
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

    /*!
     * @brief           Appends one IMU sample to the buffer, dropping samples
     *                  that are not newer than the last one and the oldest
     *                  sample when 2500 are already queued.
     *
     *                  Runs on an executor thread in the IMU callback group,
     *                  which never runs two of its callbacks at once.
     *
     * @param[in]       imu_msg
     *                  IMU sample; the pointer is kept in the buffer.
     */
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    // Variables
    /*!
     * @brief           Guards imuBuf; taken by GrabImu() and by the
     *                  synchronisation thread.
     */
    std::mutex mBufMutex;

    /*!
     * @brief           IMU samples received but not yet paired with an image,
     *                  in increasing time-stamp order.
     */
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
};

/*!
 * @brief           ROS node that admits synchronised RGB-D packets into a
 *                  buffer and runs the thread that pairs them with IMU samples
 *                  and feeds the SLAM system.
 */
class ImageGrabber : public rclcpp::Node
{
  public:
    /*!
     * @brief           A synchronized image pair and the closest available
     *                  depth cloud.
     */
    struct SynchronizedRgbdPacket
    {
        sensor_msgs::msg::Image::ConstSharedPtr       p_rgbImageMessage;
        sensor_msgs::msg::Image::ConstSharedPtr       p_depthImageMessage;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
    };

    /*!
     * @brief           Construct the synchronized RGB-D and IMU ingestion
     *                  adapter.
     *
     * @param[in]       p_imuGrabber_in
     *                  Shared owner of the IMU sample buffer.
     *
     * @param[in]       useSimTime_in
     *                  True when timestamps must follow `/clock`.
     *
     * @param[in]       maximumTrackingRate_hz_in
     *                  Maximum visual estimator rate.
     *
     * @param[in]       maximumBufferDuration_seconds_in
     *                  Allowed sensor latency.
     *
     * @param[in]       directGazeboFluCloud_in
     *                  True when the input cloud is a direct Gazebo cloud in
     *                  forward-left-up axes, converted to the optical camera
     *                  frame; generated RGB-D clouds are already optical.
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
    /*!
     * @brief           Guards the RGB-D packet buffer, the admission state and
     *                  the latest point cloud; when taken together with the IMU
     *                  buffer's mutex, both are locked at once.
     */
    std::mutex mBufMutex;

    /*!
     * @brief           Set by main() after the executor stops to end the
     *                  synchronisation thread's loop.
     */
    std::atomic<bool> mustStop{false};

    /*!
     * @brief           IMU node whose buffer this node drains; shared with
     *                  main().
     */
    std::shared_ptr<ImuGrabber> mpImuGb;

    /*!
     * @brief           Admitted RGB-D packets waiting for IMU samples to reach
     *                  their time stamp; guarded by mBufMutex.
     */
    std::queue<SynchronizedRgbdPacket> synchronizedRgbdPacketBuffer;

    /*!
     * @brief           Shortest allowed time between two admitted packets,
     *                  seconds (reciprocal of the maximum tracking rate).
     */
    const double minimumTrackingInterval_seconds;

    /*!
     * @brief           Largest number of packets the buffer may hold before new
     *                  ones are refused (rate times allowed latency, plus one).
     */
    const std::size_t maximumBufferedRgbdPackets;
    double            lastReceivedRgbdTimestamp_seconds{0.0};
    bool              hasReceivedRgbdPacket{false};

    /*!
     * @brief           RGB image time stamp of the latest admitted packet,
     *                  seconds; meaningful only when hasAdmittedRgbdPacket.
     */
    double lastAdmittedRgbdTimestamp_seconds{0.0};

    /*!
     * @brief           True once a packet has been admitted since start or
     *                  since the last overload reset.
     */
    bool hasAdmittedRgbdPacket{false};

    /*!
     * @brief           True after the buffer overflowed: new packets are
     *                  refused until the buffer is empty, then the active map
     *                  is reset.
     */
    bool   discardInputUntilBufferDrained{false};
    double lastProcessedRgbdTimestamp_seconds{0.0};
    bool   hasProcessedRgbdPacket{false};

    /*!
     * @brief           Newest depth point cloud received, paired with the next
     *                  RGB-D image pair; null before the first one; guarded by
     *                  mBufMutex.
     */
    sensor_msgs::msg::PointCloud2::ConstSharedPtr p_latestPointCloudMessage;

    const bool directGazeboFluCloud;
    /*!
     * @brief           Time stamp of the last IMU sample taken from the buffer,
     *                  seconds; meaningful only when hasConsumedImuSample.
     */
    double     lastConsumedImuTimestamp_seconds{0.0};

    /*!
     * @brief           True when an IMU sample has been consumed since start or
     *                  since the IMU run was last discarded.
     */
    bool hasConsumedImuSample{false};

    /*!
     * @brief           IMU samples taken from the buffer but not yet delivered
     *                  with an accepted visual frame; used by the
     *                  synchronisation thread only.
     */
    std::vector<vs_graphs::core::IMU::Point> pendingImuMeasurements;

    /*!
     * @brief           Largest time gap between consecutive samples in
     *                  pendingImuMeasurements, seconds.
     */
    double pendingMaximumImuGap_seconds{0.0};

    void    SyncWithImu();
    // void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    /*!
     * @brief           Converts a ROS image message into an OpenCV image.
     *
     *                  Called by the synchronisation thread.
     *
     * @param[in]       img_msg
     *                  Image message to convert.
     *
     * @return          A copy of the image, or an empty matrix when conversion
     *                  fails (an error is logged).
     */
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

    /*!
     * @brief           Hands the voxblox skeleton graph to the semantic code,
     *                  which buffers it for the segmentation thread.
     *
     *                  Runs on an executor thread in the skeleton callback
     *                  group.
     *
     * @param[in]       msgSkeletonGraph
     *                  Skeleton graph markers published by voxblox.
     */
    void GrabVoxbloxSkeletonGraph(
        const visualization_msgs::msg::MarkerArray &msgSkeletonGraph);

    /*!
     * @brief           Keeps the newest depth point cloud for the next RGB-D
     *                  image pair; ignores clouds that are not newer than the
     *                  one held.
     *
     *                  Runs on an executor thread in the visual callback group,
     *                  the same group as the RGB-D synchroniser callback.
     *
     * @param[in]       msgPC
     *                  Point cloud message; the pointer is kept.
     */
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

/*!
 * @brief           Loop of the synchronisation thread: pairs each RGB-D packet
 *                  with the IMU samples up to its time stamp and feeds them to
 *                  the SLAM system, until asked to stop.
 */
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
        std::vector<vs_graphs::core::IMU::Point>      imuMeasurements;
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
                        const std::shared_ptr<const sensor_msgs::msg::Imu>
                                    &p_imuMessage = mpImuGb->imuBuf.front();
                        const double imuTimestamp_seconds =
                            rclcpp::Time(p_imuMessage->header.stamp).seconds();
                        if (hasConsumedImuSample)
                        {
                            pendingMaximumImuGap_seconds =
                                std::max(pendingMaximumImuGap_seconds,
                                         imuTimestamp_seconds -
                                             lastConsumedImuTimestamp_seconds);
                        }
                        const cv::Point3f imuAcceleration_body_mPerSec2(
                            static_cast<float>(
                                p_imuMessage->linear_acceleration.x),
                            static_cast<float>(
                                p_imuMessage->linear_acceleration.y),
                            static_cast<float>(
                                p_imuMessage->linear_acceleration.z));
                        const cv::Point3f angularVelocity_body_radPerSecCv(
                            static_cast<float>(
                                p_imuMessage->angular_velocity.x),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.y),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.z));

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
                            imuAcceleration_body_mPerSec2,
                            angularVelocity_body_radPerSecCv,
                            imuTimestamp_seconds);
                        angularVelocity_body_radPerSec << static_cast<float>(
                            p_imuMessage->angular_velocity.x),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.y),
                            static_cast<float>(
                                p_imuMessage->angular_velocity.z);
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
            if (p_slamSystem->requestResetActiveMapWithCause(
                    vs_graphs::core::ResetCause::IMU_DELIVERY_GAP) !=
                vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: requestResetActiveMapWithCause returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
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

            processingStage = "marker association";
            std::pair<double, std::vector<vs_graphs::core::semantic::Marker *>>
                nearestMarker = findNearestMarker(imageTimestamp_seconds);
            const double markerTimeDifference_seconds = nearestMarker.first;
            std::vector<vs_graphs::core::semantic::Marker *> matchedMarkers =
                std::move(nearestMarker.second);

            processingStage = "inertial RGB-D tracking";
            if (markerTimeDifference_seconds < 0.05)
            {
                Sophus::SE3f slamSystemCameraPose{};
                if (p_slamSystem->trackRGBD(rgbImage,
                                            depthImage,
                                            p_pointCloud,
                                            imageTimestamp_seconds,
                                            slamSystemCameraPose,
                                            imuMeasurements,
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
                markersBuffer.clear();
            }
            else
            {
                Sophus::SE3f slamSystemCameraPose2{};
                if (p_slamSystem->trackRGBD(rgbImage,
                                            depthImage,
                                            p_pointCloud,
                                            imageTimestamp_seconds,
                                            slamSystemCameraPose2,
                                            imuMeasurements) !=
                    vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackRGBD returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }

            double estimatorInterval_seconds = 0.0;
            if (hasProcessedRgbdPacket)
            {
                const double processedFrameInterval_seconds =
                    imageTimestamp_seconds - lastProcessedRgbdTimestamp_seconds;
                estimatorInterval_seconds = processedFrameInterval_seconds;
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
            recordEstimatorFrame(estimatorInterval_seconds);

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

    // Initializing system threads and getting ready to process frames
    const bool useSimTime = node->get_parameter("use_sim_time").as_bool();
    std::shared_ptr<ImuGrabber> imugb =
        std::make_shared<ImuGrabber>(useSimTime);
    std::shared_ptr<ImageGrabber> igb =
        std::make_shared<ImageGrabber>(imugb,
                                       useSimTime,
                                       maximumTrackingRate_hz,
                                       maximumSensorBuffer_seconds,
                                       directGazeboFluCloud);

    sensorType   = vs_graphs::core::System::IMU_RGBD;
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
            [imugb](const Imu::ConstSharedPtr msg) { imugb->GrabImu(msg); },
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
            { igb->GrabPointCloud(msg); },
            visualSubscriptionOptions);

    typedef ApproximateTime<Image, Image> syncPolicy;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::
            ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync = std::make_shared<Synchronizer<syncPolicy>>(syncPolicy(10),
                                                          *subImgRGB,
                                                          *subImgDepth);
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.010));
    sync->registerCallback(std::bind(&ImageGrabber::GrabRGBD,
                                     igb.get(),
                                     std::placeholders::_1,
                                     std::placeholders::_2));

    // Subscriber to get segmentation results from the SemanticSegmenter module
    std::shared_ptr<rclcpp::Subscription<segmenter_ros::msg::SegmenterDataMsg>>
        subSegmentedImage =
            node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
                "/camera/color/image_segment",
                rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local(),
                [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
                { addSegmentationToSystem(*msg, igb->get_logger()); },
                semanticSubscriptionOptions);

    // Subsriber to get skeletonized graph from the `voxblox` module
    // Match the skeletonizer transient-local publisher so the latest usable
    // graph is received even when this node joins after publication.
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

    // Match the skeletonizer transient-local publisher so the latest usable
    // cloud is received even when this node joins after publication.
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
        subVoxbloxSkeleton =
            node->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/voxblox_skeletonizer/skeleton",
                rclcpp::QoS(1).transient_local(),
                [](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                { observeVoxbloxSkeletonPublication(*msg); },
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
    if (p_slamSystem->shutdown() !=
        vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: shutdown returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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

    const sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage =
        p_latestPointCloudMessage;

    if (discardInputUntilBufferDrained)
    {
        if (!synchronizedRgbdPacketBuffer.empty())
            return;

        {
            std::lock_guard<std::mutex> imuBufferLock(mpImuGb->mBufMutex);
            std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> emptyImuBuffer;
            mpImuGb->imuBuf.swap(emptyImuBuffer);
        }

        if (p_slamSystem->requestResetActiveMapWithCause(
                vs_graphs::core::ResetCause::SENSOR_PROCESSING_OVERLOAD) !=
            vs_graphs::core::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: requestResetActiveMapWithCause returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
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
            static_cast<double>(maximumBufferedRgbdPackets) *
                minimumTrackingInterval_seconds);
        return;
    }

    synchronizedRgbdPacketBuffer.push({msgRGB, msgD, p_pointCloudMessage});
    lastAdmittedRgbdTimestamp_seconds = rgbTimestamp_seconds;
    hasAdmittedRgbdPacket             = true;
}

/*
 * Callback function to get the markers detected by the `aruco_ros`
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

void ImageGrabber::GrabVoxbloxSkeletonGraph(
    const visualization_msgs::msg::MarkerArray &msgSkeletonGraph)
{
    // Pass the skeleton graph to a buffer to be processed by the
    // SemanticSegmentation thread
    setVoxbloxSkeletonCluster(msgSkeletonGraph);
}
