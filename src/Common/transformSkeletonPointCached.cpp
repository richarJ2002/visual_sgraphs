/*!
 * @File:         transformSkeletonPoint.cpp
 *
 * @Brief:        Transforms a Voxblox skeleton point into the world frame using
 *                a previously resolved marker transformation.
 *
 * @Date:         20/07/2026
 *
 */

#include <cmath>
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
    const tf2::Transform            &T_world_skeletonMarker_in,
    const geometry_msgs::msg::Point &skeletonPoint_marker_in,
    Eigen::Vector3d                 &skeletonPoint_world_out)
{
    /* Set an invalid default output in case transformation fails */
    skeletonPoint_world_out =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    /* Reject invalid marker-local coordinates */
    if (!std::isfinite(skeletonPoint_marker_in.x) ||
        !std::isfinite(skeletonPoint_marker_in.y) ||
        !std::isfinite(skeletonPoint_marker_in.z))
    {
        return false;
    }

    const tf2::Vector3 skeletonPoint_marker(skeletonPoint_marker_in.x,
                                            skeletonPoint_marker_in.y,
                                            skeletonPoint_marker_in.z);

    /* Transform the marker-local point into the world frame */
    const tf2::Vector3 skeletonPoint_world =
        T_world_skeletonMarker_in * skeletonPoint_marker;

    skeletonPoint_world_out =
        Eigen::Vector3d(static_cast<double>(skeletonPoint_world.x()),
                        static_cast<double>(skeletonPoint_world.y()),
                        static_cast<double>(skeletonPoint_world.z()));

    return skeletonPoint_world_out.allFinite();
}