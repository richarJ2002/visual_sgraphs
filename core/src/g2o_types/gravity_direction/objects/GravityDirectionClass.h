/*!
 * @file            GravityDirectionClass.h
 *
 * @brief           Declares GravityDirection, the direction of gravity
 * expressed as a rotation.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_GRAVITY_DIRECTION_OBJECTS_GRAVITY_DIRECTION_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_GRAVITY_DIRECTION_OBJECTS_GRAVITY_DIRECTION_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "g2o_types/gravity_direction/objects/GravityDirectionStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace gravity_direction
{

/*!
 * @brief           Direction of gravity expressed as a rotation from a
 *                  gravity-aligned frame (z axis along gravity) to the world
 *                  frame. Only two degrees of freedom can change, because
 *                  rotating about gravity itself has no effect.
 */
class GravityDirection
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GravityDirection() :
        gravityRotation_gravityToWorld(Eigen::Matrix3d::Identity()),
        gravityRotation_worldToGravity(Eigen::Matrix3d::Identity()),
        its(0)
    {}

    /*!
     * @brief           Creates the gravity direction from a known rotation.
     *
     * @param[in]       gravityRotation_gravityToWorld_in
     *                  Rotation from the gravity-aligned frame to the world
     *                  frame.
     */
    explicit GravityDirection(
        const Eigen::Matrix3d &gravityRotation_gravityToWorld_in) :
        gravityRotation_gravityToWorld(gravityRotation_gravityToWorld_in),
        gravityRotation_worldToGravity(
            gravityRotation_gravityToWorld_in.transpose()),
        its(0)
    {}

    /*!
     * @brief           Applies an optimiser step by rotating the
     *                  gravity-aligned frame about its own x and y axes, then
     *                  refreshes the inverse rotation.
     *
     * @param[in]       p_pu_in
     *                  Two values: rotation about x and about y, radians.
     *
     * @return          GRAVITY_DIRECTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] GravityDirectionStatus update(const double *p_pu_in);

    /*!
     * @brief           Rotation from the gravity-aligned frame to the world
     *                  frame.
     */
    Eigen::Matrix3d gravityRotation_gravityToWorld;
    /*!
     * @brief           Rotation from the world frame to the gravity-aligned
     *                  frame; the transpose of gravityRotation_gravityToWorld.
     */
    Eigen::Matrix3d gravityRotation_worldToGravity;

    /*!
     * @brief           Number of updates applied; set to zero at construction
     *                  and not used by update.
     */
    int its;
};

} /* namespace gravity_direction */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_GRAVITY_DIRECTION_OBJECTS_GRAVITY_DIRECTION_CLASS_H \
        */
