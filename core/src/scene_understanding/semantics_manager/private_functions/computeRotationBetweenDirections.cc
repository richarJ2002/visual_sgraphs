/*!
 * @file            computeRotationBetweenDirections.cc
 *
 * @brief           Implements the module-private Eigen rotation boundary.
 */

/*!
 * Eigen 3.4.0 setFromTwoVectors under GCC 13.3 -O3 raises a false
 * -Wmaybe-uninitialized on its internal JacobiSVD matrixV, entirely inside
 * Eigen/GCC system headers. The SVD branch only runs for finite, near-opposite
 * directions, where Eigen fills matrixV. The pragma must wrap the first Eigen
 * include of this translation unit, which is why this file is excluded from
 * unity batches and includes Eigen before anything else.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

Eigen::Quaterniond
    computeRotationBetweenDirections(const Eigen::Vector3d &sourceDirection_in,
                                     const Eigen::Vector3d &targetDirection_in)
{
    Eigen::Quaterniond rotationQuaternion;
    rotationQuaternion.setFromTwoVectors(sourceDirection_in,
                                         targetDirection_in);
    return rotationQuaternion;
}

} /* namespace core */
} /* namespace vs_graphs */
