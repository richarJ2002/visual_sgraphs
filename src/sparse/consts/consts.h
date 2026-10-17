/*!
 * @File:         consts.h
 *
 * @Brief:        TODO
 *
 * @Date:         09/10/2026
 *
 */

#ifndef HPP_CONSTS_HPP
#define HPP_CONSTS_HPP

/* Function Includes */
/* None */

/* Object Include */
/* None */

#include <string_view>

/* Generic Libraries */
/* None */

namespace vs_graphs::sparse
{
/*!
 * @brief           Marker type code of a CUBE_LIST marker, as in
 *                  visualization_msgs/msg/Marker.msg. The message header is not
 *                  included so this module builds without ROS.
 *
 */
inline constexpr int SPARSE_CUBE_LIST_TYPE = 6;

/*!
 * @brief           Marker type code of a LINE_LIST marker, as in
 *                  visualization_msgs/msg/Marker.msg.
 *
 */
inline constexpr int SPARSE_LINE_LIST_TYPE = 5;

/*!
 * @brief           Prefix of the marker namespaces the skeletonizer uses for
 *                  connected free-space components.
 *
 */
inline constexpr std::string_view SPARSE_CLUSTER_NAMESPACE_PREFIX =
    "connected_vertices_";

/*!
 * @brief           Marker namespace the skeletonizer uses for raw skeleton
 *                  edges.
 *
 */
inline constexpr std::string_view SPARSE_EDGE_NAMESPACE = "edges";

} // namespace vs_graphs::sparse

#endif /* HPP_CONSTS_HPP */
