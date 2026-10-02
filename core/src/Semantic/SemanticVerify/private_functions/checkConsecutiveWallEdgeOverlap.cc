/*!
 * @file            checkConsecutiveWallEdgeOverlap.cc
 *
 * @brief           Implements checkConsecutiveWallEdgeOverlap(), declared in
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
#include <rclcpp/logging.hpp>
#include <set>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! @brief Requires coplanar wall pairs to also overlap along the wall
 * direction. Same infinite plane with disjoint extents means different
 * walls (or different places): positive disjointness evidence contradicts,
 * while walls without enough samples are skipped (the angle/offset core
 * owns their verdict). */
SemanticVerifyStatus checkConsecutiveWallEdgeOverlap(
    const std::vector<ConsecutiveAnchorPair> &pairs_in,
    const g2o::Sim3                          &transform_in,
    double                                    maximumNormalAngle_deg_in,
    double                                    minimumOverlap_m_in,
    AlignmentCheck                           &alignmentCheck_out)
{
    for (const ConsecutiveAnchorPair &pair : pairs_in)
    {
        for (const VerifyWallObservation &absorbedWall : pair.p_absorbed->walls)
        {
            if (absorbedWall.supportSample_world.size() < 2U ||
                !absorbedWall.wallNormal_world.allFinite() ||
                !std::isfinite(absorbedWall.d) ||
                !absorbedWall.wallCentroid_world.allFinite())
            {
                continue;
            }
            const double absorbedNormalNorm =
                absorbedWall.wallNormal_world.norm();
            if (!std::isfinite(absorbedNormalNorm) || absorbedNormalNorm < 1e-8)
            {
                continue;
            }
            /* Directions rotate only; the offset carries scale and
             * translation (same plane convention as
             * checkFixedTransformWalls). */
            const double          absorbedScale = transform_in.scale();
            const Eigen::Matrix3d absorbedRotation =
                transform_in.rotation().toRotationMatrix().cast<double>();
            const Eigen::Vector3d absorbedTranslation =
                transform_in.translation().cast<double>();
            if (!std::isfinite(absorbedScale) || absorbedScale <= 0.0 ||
                !absorbedRotation.allFinite() ||
                !absorbedTranslation.allFinite())
            {
                continue;
            }
            const Eigen::Vector3d mappedNormal =
                absorbedRotation * absorbedWall.wallNormal_world;
            const double mappedOffset = absorbedScale * absorbedWall.d -
                                        mappedNormal.dot(absorbedTranslation);
            bool hasOverlapPartner = false;
            bool hasCompatibleWall = false;
            for (const VerifyWallObservation &survivingWall :
                 pair.p_surviving->walls)
            {
                if (survivingWall.supportSample_world.size() < 2U ||
                    !survivingWall.wallNormal_world.allFinite() ||
                    !std::isfinite(survivingWall.d) ||
                    !survivingWall.wallCentroid_world.allFinite())
                {
                    continue;
                }
                const double survivingNormalNorm =
                    survivingWall.wallNormal_world.norm();
                if (!std::isfinite(survivingNormalNorm) ||
                    survivingNormalNorm < 1e-8)
                {
                    continue;
                }
                const double cosine = std::clamp(
                    mappedNormal.dot(survivingWall.wallNormal_world) /
                        std::max(mappedNormal.norm() * survivingNormalNorm,
                                 1e-9),
                    -1.0,
                    1.0);
                const double angle_deg =
                    std::acos(cosine) * 180.0 / std::acos(-1.0);
                const double offset_m =
                    std::abs(mappedOffset - survivingWall.d);
                if (angle_deg > maximumNormalAngle_deg_in ||
                    offset_m > kConsecutiveMaxPlaneOffset_m)
                {
                    continue;
                }
                hasCompatibleWall = true;
                /* In-plane axis from the surviving wall normal. Walls are
                 * near-vertical by admission (maxTiltWall), so normal x
                 * world-Z spans the wall length. */
                Eigen::Vector3d axis = survivingWall.wallNormal_world.cross(
                    Eigen::Vector3d::UnitZ());
                if (axis.squaredNorm() < 1e-8)
                {
                    axis = survivingWall.wallNormal_world.cross(
                        Eigen::Vector3d::UnitX());
                }
                if (axis.squaredNorm() < 1e-8)
                {
                    continue;
                }
                axis.normalize();
                double survivingMinimum = 0.0;
                double survivingMaximum = 0.0;
                bool   hasFiniteSample{};
                if (wallSamplesSpanInterval(survivingWall,
                                            axis,
                                            survivingWall.wallCentroid_world,
                                            survivingMinimum,
                                            survivingMaximum,
                                            hasFiniteSample) !=
                    SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: wallSamplesSpanInterval returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (!hasFiniteSample)
                {
                    continue;
                }
                double absorbedMinimum = 0.0;
                double absorbedMaximum = 0.0;
                bool   hasMappedSample = false;
                double mappedMinimum = std::numeric_limits<double>::infinity();
                double mappedMaximum = -std::numeric_limits<double>::infinity();
                for (const Eigen::Vector3d &sample :
                     absorbedWall.supportSample_world)
                {
                    Eigen::Vector3d mappedSample = Eigen::Vector3d::Zero();
                    if (!(transformAbsorbedPoint(transform_in,
                                                 sample,
                                                 mappedSample) ==
                          SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS))
                    {
                        continue;
                    }
                    const double coordinate =
                        (mappedSample - survivingWall.wallCentroid_world)
                            .dot(axis);
                    mappedMinimum   = std::min(mappedMinimum, coordinate);
                    mappedMaximum   = std::max(mappedMaximum, coordinate);
                    hasMappedSample = true;
                }
                if (!hasMappedSample)
                {
                    continue;
                }
                absorbedMinimum = mappedMinimum;
                absorbedMaximum = mappedMaximum;
                const double overlap_m =
                    std::min(survivingMaximum, absorbedMaximum) -
                    std::max(survivingMinimum, absorbedMinimum);
                if (overlap_m >= minimumOverlap_m_in)
                {
                    hasOverlapPartner = true;
                    break;
                }
            }
            if (hasCompatibleWall && !hasOverlapPartner)
            {
                alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
            }
        }
    }
    alignmentCheck_out = AlignmentCheck::ALIGNED;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
