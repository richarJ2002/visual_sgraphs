/*!
 * @File:         transformSkeletonPoint.cpp
 *
 * @Brief:        Transforms a single Voxblox skeleton point into the world
 *                frame.
 *
 * @Date:         20/07/2026
 *
 */

#include <limits>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

bool transformSkeletonPoint(
    const visualization_msgs::msg::Marker &skeletonMarker_in,
    const geometry_msgs::msg::Point       &skeletonPoint_marker_in,
    Eigen::Vector3d                       &skeletonPoint_world_out)
{
    /* Resolve the marker-local-to-world transformation */
    tf2::Transform T_world_skeletonMarker;

    if (!getSkeletonMarkerWorldTransform(skeletonMarker_in,
                                         T_world_skeletonMarker))
    {
        skeletonPoint_world_out =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

        return false;
    }

    /* Transform the supplied point using the resolved transformation */
    return transformSkeletonPoint(T_world_skeletonMarker,
                                  skeletonPoint_marker_in,
                                  skeletonPoint_world_out);
}