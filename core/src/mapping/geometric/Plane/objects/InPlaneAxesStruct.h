/*!
 * @file            InPlaneAxesStruct.h
 *
 * @brief           Declares the pair of in-plane axes returned by
 *                  computeInPlaneAxes().
 */

#ifndef VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_OBJECTS_IN_PLANE_AXES_STRUCT_H
#define VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_OBJECTS_IN_PLANE_AXES_STRUCT_H

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

/*!
 * @brief           Two orthonormal axes lying in a plane, world frame. The
 *                  in-plane bounds of a plane (minPlaneU/V, maxPlaneU/V) are
 *                  projections onto these axes.
 */
struct InPlaneAxes
{
    /*!
     * @brief           First in-plane axis U, unit vector, world frame.
     */
    Eigen::Vector3d planeAxisU_world{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Second in-plane axis V = normal x U, unit vector,
     *                  world frame.
     */
    Eigen::Vector3d planeAxisV_world{Eigen::Vector3d::Zero()};
};

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_OBJECTS_IN_PLANE_AXES_STRUCT_H */
