/*!
 * @file            update.cc
 *
 * @brief           Implements GravityDirection::update().
 */

#include "optimization/g2o_types/gravity_direction/objects/GravityDirectionClass.h"

#include <rclcpp/logging.hpp>

#include "optimization/g2o_types/g2o_types/public_functions/public_functions.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace gravity_direction
{

GravityDirectionStatus GravityDirection::update(const double *p_pu_in)
{
    Eigen::Matrix3d rotation{};
    if (expSO3(p_pu_in[0], p_pu_in[1], 0.0, rotation) !=
        G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: expSO3 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    gravityRotation_gravityToWorld = gravityRotation_gravityToWorld * rotation;
    gravityRotation_worldToGravity = gravityRotation_gravityToWorld.transpose();

    return GravityDirectionStatus::GRAVITY_DIRECTION_STATUS_SUCCESS;
}

} /* namespace gravity_direction */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */
