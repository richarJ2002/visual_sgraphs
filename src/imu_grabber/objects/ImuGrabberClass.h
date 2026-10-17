/*!
 * @file            ImuGrabberClass.h
 *
 * @brief           Declares the bounded, timestamp-ordered IMU buffer.
 *
 * @date            09/10/2026
 *
 */

#ifndef HPP_IMUGRABBERCLASS_HPP
#define HPP_IMUGRABBERCLASS_HPP

#include "common/common.h"

#include <memory>
#include <mutex>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace vs_graphs::imu_grabber
{

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
        vs_graphs::common::tfBroadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);
        vs_graphs::common::staticTfBroadcaster =
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
     * @param[in]       p_imuMessage_in
     *                  IMU sample; the pointer is kept in the buffer.
     */
    void getImuSample(
        const sensor_msgs::msg::Imu::ConstSharedPtr &p_imuMessage_in);

    /* Variables */
    /*!
     * @brief           Guards imuBuf; taken by getImuSample() and by the
     *                  synchronisation thread.
     */
    std::mutex mBufMutex;

    /*!
     * @brief           IMU samples received but not yet paired with an image,
     *                  in increasing time-stamp order.
     */
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
};
} /* namespace vs_graphs::imu_grabber */

#endif /* HPP_IMUGRABBERCLASS_HPP */
