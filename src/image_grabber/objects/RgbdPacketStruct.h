/*!
 * @file            RgbdPacketStruct.h
 *
 * @brief           Declares the coherent RGB-D worker handoff packet.
 */

#ifndef VS_GRAPHS_IMAGE_GRABBER_RGBD_PACKET_STRUCT_H
#define VS_GRAPHS_IMAGE_GRABBER_RGBD_PACKET_STRUCT_H

#include <chrono>
#include <cstdint>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vs_graphs::image_grabber
{
/*!
 * @brief           Keeps ROS messages alive between admission and processing.
 *
 * @note            The adapter's packet mutex guards pending ownership. Once
 *                  transferred, only the worker accesses the packet.
 */
struct RgbdPacket
{
  public:
    /*!
     * @brief           Shared RGB message defining the packet's sensor time.
     */
    sensor_msgs::msg::Image::ConstSharedPtr       p_rgbImageMessage;
    /*!
     * @brief           Shared registered depth in the camera optical frame.
     */
    sensor_msgs::msg::Image::ConstSharedPtr       p_depthImageMessage;
    /*!
     * @brief           Shared cloud in the input message's declared frame.
     */
    sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
    /*!
     * @brief           RGB header time in nanoseconds on the ROS sensor clock.
     */
    std::int64_t                                  sensorTimestampNanoseconds{0};
    /*!
     * @brief           Steady-clock admission used for queue-wait timing.
     */
    std::chrono::steady_clock::time_point         callbackArrival{};
};
} /* namespace vs_graphs::image_grabber */

#endif /* VS_GRAPHS_IMAGE_GRABBER_RGBD_PACKET_STRUCT_H */
