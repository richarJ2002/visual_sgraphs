/*!
 * @file            computeInPlaneAxes.cc
 *
 * @brief           Implements computeInPlaneAxes(), declared in
 *                  Plane/public_functions/public_functions.h.
 */

#include "mapping/geometric/Plane/public_functions/public_functions.h"

#include <cmath>

#include <Eigen/Geometry>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

InPlaneAxes computeInPlaneAxes(const Eigen::Vector3d &planeNormal_world_in)
{
    /* Above this |normal.x| the normal is too close to X to cross with it. */
    constexpr double MAX_X_REFERENCE_ALIGNMENT = 0.90;

    const Eigen::Vector3d referenceAxis_world =
        std::abs(planeNormal_world_in.x()) <= MAX_X_REFERENCE_ALIGNMENT
            ? Eigen::Vector3d::UnitX()
            : Eigen::Vector3d::UnitY();

    InPlaneAxes inPlaneAxes{};
    inPlaneAxes.planeAxisU_world =
        planeNormal_world_in.cross(referenceAxis_world).normalized();
    inPlaneAxes.planeAxisV_world =
        planeNormal_world_in.cross(inPlaneAxes.planeAxisU_world).normalized();
    return inPlaneAxes;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */
