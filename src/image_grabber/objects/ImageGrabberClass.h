/*!
 * @file            ImageGrabberClass.h
 *
 * @brief           Declares the RGB-D-inertial sensor adapter.
 *
 * @date            09/10/2026
 *
 */

#ifndef HPP_IMAGEGRABBERCLASS_HPP
#define HPP_IMAGEGRABBERCLASS_HPP

#include "common/common.h"
#include "image_grabber/objects/SynchronizedRgbdPacketStruct.h"
#include "imu_grabber/imu_grabber.h"
#include "sensor_models/imu_types/point/point.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>
#include <vector>

namespace vs_graphs::image_grabber
{

/*!
 * @brief           ROS node that admits synchronised RGB-D packets into a
 *                  buffer and runs the thread that pairs them with IMU samples
 *                  and feeds the SLAM system.
 */
class ImageGrabber : public rclcpp::Node
{
  public:
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
    ImageGrabber(
        std::shared_ptr<vs_graphs::imu_grabber::ImuGrabber> p_imuGrabber_in,
        bool                                                useSimTime_in,
        double maximumTrackingRate_hz_in,
        double maximumBufferDuration_seconds_in,
        bool   directGazeboFluCloud_in) :
        Node("image_grabber",
             rclcpp::NodeOptions()
                 .use_global_arguments(false)
                 .parameter_overrides(
                     {rclcpp::Parameter("use_sim_time", useSimTime_in)})),
        p_imuGrabber(std::move(p_imuGrabber_in)),
        minimumTrackingInterval_seconds(
            1.0 / std::max(1.0, maximumTrackingRate_hz_in)),
        maximumBufferedRgbdPackets(
            static_cast<std::size_t>(
                std::max(1.0, maximumTrackingRate_hz_in) *
                std::max(1.0, maximumBufferDuration_seconds_in)) +
            1U),
        directGazeboFluCloud(directGazeboFluCloud_in)
    {
        vs_graphs::common::tfBroadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);
        vs_graphs::common::staticTfBroadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    /*!
     * @brief           Guards the RGB-D packet buffer, the admission state and
     *                  the latest point cloud; when taken together with the IMU
     *                  buffer's mutex, both are locked at once.
     *
     */
    std::mutex mBufMutex;

    /*!
     * @brief           Set by main() after the executor stops to end the
     *                  synchronisation thread's loop.
     *
     */
    std::atomic<bool> mustStop{false};

    /*!
     * @brief           IMU node whose buffer this node drains; shared with
     *                  main().
     *
     */
    std::shared_ptr<vs_graphs::imu_grabber::ImuGrabber> p_imuGrabber;

    /*!
     * @brief           Admitted RGB-D packets waiting for IMU samples to reach
     *                  their time stamp; guarded by mBufMutex.
     *
     */
    std::queue<SynchronizedRgbdPacket> synchronizedRgbdPacketBuffer;

    /*!
     * @brief           Shortest allowed time between two admitted packets,
     *                  seconds (reciprocal of the maximum tracking rate).
     *
     */
    const double minimumTrackingInterval_seconds;

    /*!
     * @brief           Largest number of packets the buffer may hold before new
     *                  ones are refused (rate times allowed latency, plus one).
     *
     */
    const std::size_t maximumBufferedRgbdPackets;

    /*!
     * @brief           Latest received RGB header time in seconds; guarded by
     *                  mBufMutex and meaningful after hasReceivedRgbdPacket.
     *
     */
    double lastReceivedRgbdTimestamp_seconds{0.0};

    /*!
     * @brief           Whether the received-time baseline exists; guarded by
     *                  mBufMutex.
     *
     */
    bool hasReceivedRgbdPacket{false};

    /*!
     * @brief           RGB image time stamp of the latest admitted packet,
     *                  seconds; meaningful only when hasAdmittedRgbdPacket.
     *
     */
    double lastAdmittedRgbdTimestamp_seconds{0.0};

    /*!
     * @brief           True once a packet has been admitted since start or
     *                  since the last overload reset.
     *
     */
    bool hasAdmittedRgbdPacket{false};

    /*!
     * @brief           True after the buffer overflowed: new packets are
     *                  refused until the buffer is empty, then the active map
     *                  is reset.
     *
     */
    bool discardInputUntilBufferDrained{false};

    /*!
     * @brief           Last successfully tracked RGB header time in seconds;
     *                  accessed only by the synchronization worker.
     *
     */
    double lastProcessedRgbdTimestamp_seconds{0.0};

    /*!
     * @brief           Whether the worker has completed a tracker call.
     *
     */
    bool hasProcessedRgbdPacket{false};

    /*!
     * @brief           Newest depth point cloud received, paired with the next
     *                  RGB-D image pair; null before the first one; guarded by
     *                  mBufMutex.
     *
     */
    sensor_msgs::msg::PointCloud2::ConstSharedPtr p_latestPointCloudMessage;

    /*!
     * @brief           Construction-time selection of direct Gazebo FLU to
     *                  optical-camera point-cloud conversion.
     *
     */
    const bool directGazeboFluCloud;

    /*!
     * @brief           Time stamp of the last IMU sample taken from the buffer,
     *                  seconds; meaningful only when hasConsumedImuSample.
     *
     */
    double lastConsumedImuTimestamp_seconds{0.0};

    /*!
     * @brief           True when an IMU sample has been consumed since start or
     *                  since the IMU run was last discarded.
     *
     */
    bool hasConsumedImuSample{false};

    /*!
     * @brief           IMU samples taken from the buffer but not yet delivered
     *                  with an accepted visual frame; used by the
     *                  synchronisation thread only.
     *
     */
    std::vector<vs_graphs::core::imu_types::point::Point>
        pendingImuMeasurements;

    /*!
     * @brief           Largest time gap between consecutive samples in
     *                  pendingImuMeasurements, seconds.
     *
     */
    double pendingMaximumImuGap_seconds{0.0};

    /*!
     * @brief           Processes packets once IMU time reaches image time.
     *
     * @note            Runs on the main-owned synchronization worker, not the
     *                  ROS executor. main() requests stop and joins it before
     *                  shutting down the SLAM workers.
     *
     */
    void syncWithImu();

    /*!
     * @brief           Converts a ROS image message into an OpenCV image.
     *
     *                  Called by the synchronisation thread.
     *
     * @param[in]       p_imageMessage_in
     *                  Image message to convert.
     *
     * @return          A copy of the image, or an empty matrix when conversion
     *                  fails (an error is logged).
     */
    cv::Mat getImage(
        const sensor_msgs::msg::Image::ConstSharedPtr &p_imageMessage_in);

    /*!
     * @brief           Hands the voxblox skeleton graph to the semantic code,
     *                  which buffers it for the segmentation thread.
     *
     *                  Runs on an executor thread in the skeleton callback
     *                  group.
     *
     * @param[in]       msgSkeletonGraph_in
     *                  Skeleton graph markers published by voxblox.
     */
    void grabVoxbloxSkeletonGraph(
        const visualization_msgs::msg::MarkerArray &msgSkeletonGraph_in);

    /*!
     * @brief           Keeps the newest depth point cloud for the next RGB-D
     *                  image pair; ignores clouds that are not newer than the
     *                  one held.
     *
     *                  Runs on an executor thread in the visual callback group,
     *                  the same group as the RGB-D synchroniser callback.
     *
     * @param[in]       p_pointCloudMessage_in
     *                  Point cloud message; the pointer is kept.
     */
    void grabPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                            &p_pointCloudMessage_in);

    /*!
     * @brief           Pairs a synchronised RGB-D image pair with the newest
     *                  point cloud and queues it for the processing thread,
     *                  without blocking ROS input.
     *
     *                  Drops the pair while no point cloud has arrived, when
     *                  the image stamps differ by more than 10 ms, when the
     *                  cloud is more than 33 ms from the images, when the pair
     *                  is not newer than the last one queued, or when it comes
     *                  faster than the tracking rate allows. When the queue is
     *                  full it stops admitting pairs; once the queue has
     *                  drained it clears the IMU buffer and asks the system to
     *                  reset the active map.
     *
     *                  Runs on an executor thread in the visual callback group
     *                  and holds mBufMutex throughout.
     *
     * @param[in]       p_rgbImageMessage_in
     *                  RGB image message.
     *
     * @param[in]       p_depthImageMessage_in
     *                  Registered depth image message.
     */
    void grabRgbd(
        const sensor_msgs::msg::Image::ConstSharedPtr &p_rgbImageMessage_in,
        const sensor_msgs::msg::Image::ConstSharedPtr &p_depthImageMessage_in);
};

} /* namespace vs_graphs::image_grabber */

#endif /* HPP_IMAGEGRABBERCLASS_HPP */
