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
 * @file            RgbdImageGrabberClass.h
 *
 * @brief           Declares the RGB-D ROS 2 image grabber.
 */

#ifndef VS_GRAPHS_RGBD_IMAGE_GRABBER_CLASS_H
#define VS_GRAPHS_RGBD_IMAGE_GRABBER_CLASS_H

#include "common/common.h"
#include "image_grabber/objects/RgbdPacketStruct.h"
#include "observability/observability.h"

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace vs_graphs::image_grabber
{

/*!
 * @brief           ROS 2 node that admits synchronized RGB, depth and
 *                  point-cloud triples and hands the newest one to a worker
 *                  thread running the SLAM tracker. grabRgbd runs on a ROS
 *                  executor thread and keeps only the newest packet;
 *                  processRgbdPackets runs on a separate worker thread.
 */
class RgbdImageGrabber : public rclcpp::Node
{
  public:
    /*!
     * @brief           Constructs the RGB-D adapter with the parent node's
     *                  clock mode.
     *
     * @param[in]       useSimTime_in
     *                  True when timestamps must follow `/clock`.
     *
     * @param[in]       directGazeboFluCloud_in
     *                  True when the input cloud is a direct Gazebo cloud in
     *                  forward-left-up axes, converted to the optical camera
     *                  frame; generated RGB-D clouds are already optical.
     */
    RgbdImageGrabber(const bool useSimTime_in,
                     const bool directGazeboFluCloud_in) :
        rclcpp::Node(
            "grabber",
            rclcpp::NodeOptions()
                .use_global_arguments(false)
                .parameter_overrides(
                    {rclcpp::Parameter("use_sim_time", useSimTime_in)})),
        directGazeboFluCloud(directGazeboFluCloud_in)
    {
        vs_graphs::common::tfBroadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);
        vs_graphs::common::staticTfBroadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    /*!
     * @brief           Processes the newest coherent sensor packet.
     *
     *                  A pending packet is replaced while tracking is busy, as
     *                  in the established RGB-D delivery path.
     */
    void processRgbdPackets();

    /*!
     * @brief           Requests worker shutdown and wakes a waiting worker.
     */
    void requestStop();

    /*!
     * @brief           Logs a cumulative summary without holding
     *                  packet/accounting locks.
     */
    void logRgbdObservabilitySummary(const std::string &event_in) const;

    /*!
     * @brief           Callback function to get the skeleton graph from the
     *                  `voxblox` module
     *
     * @param[in]       msgSkeletonGraph_in
     *                  The skeleton graph from the `voxblox` module
     */
    void grabVoxbloxSkeletonGraph(
        const visualization_msgs::msg::MarkerArray &msgSkeletonGraph_in);

    /*!
     * @brief           Admits a coherent RGB-D packet without blocking ROS
     *                  input.
     *
     * @param[in]       p_rgbImageMessage_in
     *                  RGB image message.
     *
     * @param[in]       p_depthImageMessage_in
     *                  Registered depth image message.
     *
     * @param[in]       p_pointCloudMessage_in
     *                  Point cloud corresponding to the image pair.
     */
    void grabRgbd(
        const sensor_msgs::msg::Image::ConstSharedPtr &p_rgbImageMessage_in,
        const sensor_msgs::msg::Image::ConstSharedPtr &p_depthImageMessage_in,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr
            &p_pointCloudMessage_in);

  private:
    /*!
     * @brief           Publishes a lock-free copy of frontend progress into
     *                  System health.
     */
    void publishRgbdFrontendHealth() const;

    /*!
     * @brief           Guards latestRgbdPacket, hasPendingRgbdPacket,
     *                  stopRequested, hasReceivedRgbdPacket and
     *                  lastReceivedRgbdTimestamp_seconds.
     */
    std::mutex              rgbdPacketMutex;
    /*!
     * @brief           Wakes the worker when a packet becomes pending or stop
     *                  is requested; waited on with rgbdPacketMutex.
     */
    std::condition_variable rgbdPacketCondition;
    /*!
     * @brief           Newest admitted packet; a later packet replaces it
     *                  before the worker takes it. Valid only while
     *                  hasPendingRgbdPacket is true; guarded by
     *                  rgbdPacketMutex.
     */
    RgbdPacket              latestRgbdPacket;
    /*!
     * @brief           True from admission until the worker takes the packet
     *                  or shutdown discards it.
     */
    bool                    hasPendingRgbdPacket{false};
    /*!
     * @brief           Set by requestStop; grabRgbd then rejects triples and
     *                  the worker exits.
     */
    bool                    stopRequested{false};
    /*!
     * @brief           True once any packet has been admitted; makes
     *                  lastReceivedRgbdTimestamp_seconds meaningful.
     */
    bool                    hasReceivedRgbdPacket{false};
    /*!
     * @brief           RGB header stamp of the newest admitted packet,
     *                  seconds; a triple with a stamp not above it is
     *                  rejected.
     */
    double                  lastReceivedRgbdTimestamp_seconds{0.0};
    /*!
     * @brief           True once the worker has tracked a packet. Used only
     *                  on the worker thread, so unguarded.
     */
    bool                    hasProcessedRgbdPacket{false};
    /*!
     * @brief           RGB header stamp of the last packet the worker
     *                  tracked, seconds; gives the estimator frame interval.
     *                  Worker thread only.
     */
    double                  lastProcessedRgbdTimestamp_seconds{0.0};
    /*!
     * @brief           Fixed at construction: true when the input cloud is a
     *                  direct Gazebo cloud in forward-left-up axes that must
     *                  be converted to the optical camera frame.
     */
    const bool              directGazeboFluCloud;
    /*!
     * @brief           Thread-safe counters and timings, updated from both
     *                  the executor thread and the worker thread.
     */
    vs_graphs::observability::RgbdObservability rgbdObservability;
};

} /* namespace vs_graphs::image_grabber */

#endif /* VS_GRAPHS_RGBD_IMAGE_GRABBER_CLASS_H */
