/*!
 * @file            checkAnchorRoomCentroids.cc
 *
 * @brief           Implements checkAnchorRoomCentroids(), declared in
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

SemanticVerifyStatus
    checkAnchorRoomCentroids(const std::vector<ConsecutiveAnchorPair> &pairs_in,
                             const g2o::Sim3 &transform_in,
                             double           maximumDistance_m_in,
                             AlignmentCheck  &alignmentCheck_out)
{
    if (pairs_in.empty())
    {
        alignmentCheck_out = AlignmentCheck::MISSING;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    for (const ConsecutiveAnchorPair &pair : pairs_in)
    {
        Eigen::Vector3d mappedCentroid = Eigen::Vector3d::Zero();
        if (!(transformAbsorbedPoint(transform_in,
                                     pair.p_absorbed->context.centroid,
                                     mappedCentroid) ==
              SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS) ||
            !pair.p_surviving->context.centroid.allFinite())
        {
            alignmentCheck_out = AlignmentCheck::MISSING;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        if ((mappedCentroid - pair.p_surviving->context.centroid).norm() >
            maximumDistance_m_in)
        {
            alignmentCheck_out = AlignmentCheck::CONTRADICTION;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
    }
    alignmentCheck_out = AlignmentCheck::ALIGNED;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
