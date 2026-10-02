/*!
 * @file            checkFixedTransformWalls.cc
 *
 * @brief           Implements checkFixedTransformWalls(), declared in
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

SemanticVerifyStatus checkFixedTransformWalls(
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const std::vector<VerifyWallObservation> &absorbedWalls_in,
    const g2o::Sim3                          &transform_in,
    const SemanticVerifyConfig               &configuration_in,
    std::size_t                              &matchedCount_out,
    AlignmentCheck                           &alignmentCheck_out)
{
    matchedCount_out = 0U;
    if (survivingWalls_in.size() < 3U || absorbedWalls_in.size() < 3U)
    {
        alignmentCheck_out = AlignmentCheck::MISSING;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    const double scale = transform_in.scale();
    if (!std::isfinite(scale) || scale <= 0.0)
    {
        alignmentCheck_out = AlignmentCheck::CONTRADICTION;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    const Eigen::Matrix3d rotation =
        transform_in.rotation().toRotationMatrix().cast<double>();
    const Eigen::Vector3d translation =
        transform_in.translation().cast<double>();
    if (!rotation.allFinite() || !translation.allFinite())
    {
        alignmentCheck_out = AlignmentCheck::CONTRADICTION;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    std::set<std::size_t> usedSurvivingWalls;
    for (const VerifyWallObservation &absorbedWall : absorbedWalls_in)
    {
        const Eigen::Vector3d transformedNormal =
            rotation * absorbedWall.normal_world;
        const double transformedOffset =
            scale * absorbedWall.d - transformedNormal.dot(translation);
        if (!transformedNormal.allFinite() || !std::isfinite(transformedOffset))
        {
            continue;
        }

        std::size_t bestIndex    = survivingWalls_in.size();
        double      bestResidual = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0U; index < survivingWalls_in.size(); ++index)
        {
            if (usedSurvivingWalls.count(index) > 0U)
            {
                continue;
            }
            const VerifyWallObservation &survivingWall =
                survivingWalls_in[index];
            const double cosine =
                std::clamp(transformedNormal.dot(survivingWall.normal_world),
                           -1.0,
                           1.0);
            const double angle_deg =
                std::acos(cosine) * 180.0 / std::acos(-1.0);
            const double offset_m =
                std::abs(transformedOffset - survivingWall.d);
            if (angle_deg > configuration_in.maxNormalAngle_deg ||
                offset_m > configuration_in.maxOffset_m)
            {
                continue;
            }
            const double residual =
                angle_deg /
                    std::max(configuration_in.maxNormalAngle_deg, 1e-9) +
                offset_m / std::max(configuration_in.maxOffset_m, 1e-9);
            if (residual < bestResidual)
            {
                bestResidual = residual;
                bestIndex    = index;
            }
        }
        if (bestIndex != survivingWalls_in.size())
        {
            usedSurvivingWalls.insert(bestIndex);
            ++matchedCount_out;
        }
    }

    const std::size_t evidenceCount =
        std::max(survivingWalls_in.size(), absorbedWalls_in.size());
    const double inlierRatio = static_cast<double>(matchedCount_out) /
                               static_cast<double>(evidenceCount);
    alignmentCheck_out =
        matchedCount_out >= 3U && inlierRatio >= configuration_in.minInlierRatio
            ? AlignmentCheck::ALIGNED
            : AlignmentCheck::CONTRADICTION;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
