/*!
 * @file            SynchronizedRgbdPacketStruct.h
 *
 * @brief           Declares the RGB-D-inertial queue packet.
 *
 * @date            09/10/2026
 *
 */

#ifndef HPP_SYNCHRONIZEDRGBDPACKETSTRUCT_HPP
#define HPP_SYNCHRONIZEDRGBDPACKETSTRUCT_HPP

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vs_graphs::image_grabber
{

/*!
 * @brief           A synchronized image pair and the closest available
 *                  depth cloud.
 */
struct SynchronizedRgbdPacket
{
  public:
    /*!
     * @brief           Shared RGB message defining the packet's sensor time.
     */
    sensor_msgs::msg::Image::ConstSharedPtr p_rgbImageMessage;

    /*!
     * @brief           Shared depth message registered to the RGB image.
     */
    sensor_msgs::msg::Image::ConstSharedPtr p_depthImageMessage;

    /*!
     * @brief           Shared cloud in the input message's declared frame.
     */
    sensor_msgs::msg::PointCloud2::ConstSharedPtr p_pointCloudMessage;
};

} /* namespace vs_graphs::image_grabber */

#endif /* HPP_SYNCHRONIZEDRGBDPACKETSTRUCT_HPP */
