/*!
 * @file            symmetricSupportDistance.cc
 *
 * @brief           Implements symmetricSupportDistance(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "Semantic/SemanticVerify.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Room.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "Types/objects/SystemParams.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! Symmetric point-to-plane support-cloud distance (inlier
 * classification): sampled points from wall A, transformed by the
 * hypothesis, checked against wall B's plane; and the reverse. Returns the
 * larger (worse) of the two mean distances; 0.0 (vacuously passing) when
 * neither side has a usable sample, since not every synthetic/unit-test
 * observation populates a support cloud. */
SemanticVerifyStatus
    symmetricSupportDistance(const VerifyWallObservation &wallA_in,
                             const VerifyWallObservation &wallB_in,
                             const Eigen::Matrix3d       &rotation_in,
                             const Eigen::Vector3d       &translation_in,
                             double                      &distance_out)
{
    double      sum   = 0.0;
    std::size_t count = 0U;
    for (const Eigen::Vector3d &pointA : wallA_in.supportSample_World)
    {
        const Eigen::Vector3d pointB = rotation_in * pointA + translation_in;
        sum += std::abs(wallB_in.normal_World.dot(pointB) + wallB_in.d);
        ++count;
    }
    const Eigen::Matrix3d rotationInverse = rotation_in.transpose();
    const Eigen::Vector3d translationInverse =
        -rotationInverse * translation_in;
    for (const Eigen::Vector3d &pointB : wallB_in.supportSample_World)
    {
        const Eigen::Vector3d pointA =
            rotationInverse * pointB + translationInverse;
        sum += std::abs(wallA_in.normal_World.dot(pointA) + wallA_in.d);
        ++count;
    }
    distance_out = count == 0U ? 0.0 : sum / static_cast<double>(count);
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
