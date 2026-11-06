/*!
 * @file            public_functions.h
 *
 * @brief           Declares the Plane module's public free functions.
 */

#ifndef VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_PUBLIC_FUNCTIONS_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_PUBLIC_FUNCTIONS_PUBLIC_FUNCTIONS_H

#include <Eigen/Core>

#include "mapping/geometric/Plane/objects/InPlaneAxesStruct.h"

namespace vs_graphs
{
namespace core
{
namespace geometric
{

/*!
 * @brief           Builds the two in-plane axes for a plane normal: U is the
 *                  normal crossed with a fixed world reference axis (X, or Y
 *                  when |normal.x| > 0.90 so the cross product stays well
 *                  conditioned), V is the normal crossed with U. The axes
 *                  flip with the sign of the normal.
 *
 * @param[in]       planeNormal_world_in
 *                  Unit plane normal, world frame.
 *
 * @return          The orthonormal in-plane axes U and V, world frame.
 */
[[nodiscard]] InPlaneAxes
    computeInPlaneAxes(const Eigen::Vector3d &planeNormal_world_in);

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_PUBLIC_FUNCTIONS_PUBLIC_FUNCTIONS_H \
        */
