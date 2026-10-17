/*!
 * @file            checkPassageTopology.cc
 *
 * @brief           Implements checkPassageTopology(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "semantic/semantic_verify/semantic_verify.h"
#include "semantic/room/room.h"

#include "geometric/Plane/Plane.h"
#include "loop_closing/loop_closing.h"
#include "map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "types/types.h"
#include "semantic/room/objects/RoomClass.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus
    checkPassageTopology(const RoomContextSnapshot  &survivingContext_in,
                         const RoomContextSnapshot  &absorbedContext_in,
                         const g2o::Sim3            &transform_in,
                         const SemanticVerifyConfig &configuration_in,
                         std::size_t                &matchedCount_out,
                         SemanticMergeReason        &contradictionReason_out,
                         AlignmentCheck             &alignmentCheck_out)
{
    matchedCount_out = 0U;
    if (survivingContext_in.passageContexts.empty() ||
        absorbedContext_in.passageContexts.empty())
    {
        alignmentCheck_out = AlignmentCheck::MISSING;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    std::map<int, const PassageContext *> survivingPassages;
    std::map<int, const PassageContext *> absorbedPassages;
    for (const PassageContext &passage : survivingContext_in.passageContexts)
    {
        survivingPassages.emplace(passage.id, &passage);
    }
    for (const PassageContext &passage : absorbedContext_in.passageContexts)
    {
        absorbedPassages.emplace(passage.id, &passage);
    }

    bool hasIncompletePassage =
        survivingPassages.size() != absorbedPassages.size();
    const Eigen::Matrix3d rotation =
        transform_in.rotation().toRotationMatrix().cast<double>();
    for (const std::pair<const int, const PassageContext *> &entry :
         absorbedPassages)
    {
        const std::map<int, const PassageContext *>::const_iterator match =
            survivingPassages.find(entry.first);
        if (match == survivingPassages.end())
        {
            hasIncompletePassage = true;
            continue;
        }

        const PassageContext &absorbedPassage  = *entry.second;
        const PassageContext &survivingPassage = *match->second;
        ++matchedCount_out;
        if (absorbedPassage.isPassable != survivingPassage.isPassable)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
            alignmentCheck_out = AlignmentCheck::CONTRADICTION;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        if (absorbedPassage.hasKnownSideRoom !=
                survivingPassage.hasKnownSideRoom ||
            absorbedPassage.hasFarSideRoom != survivingPassage.hasFarSideRoom)
        {
            hasIncompletePassage = true;
        }
        if (!absorbedPassage.hasKnownSideRoom ||
            !survivingPassage.hasKnownSideRoom ||
            !absorbedPassage.hasFarSideRoom || !survivingPassage.hasFarSideRoom)
        {
            hasIncompletePassage = true;
        }
        if (absorbedPassage.hasKnownSideRoom &&
            survivingPassage.hasKnownSideRoom &&
            absorbedPassage.knownSideRoomId != survivingPassage.knownSideRoomId)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
            alignmentCheck_out = AlignmentCheck::CONTRADICTION;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        if (absorbedPassage.hasFarSideRoom && survivingPassage.hasFarSideRoom &&
            absorbedPassage.secondaryRoomId != survivingPassage.secondaryRoomId)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
            alignmentCheck_out = AlignmentCheck::CONTRADICTION;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        if (absorbedPassage.hasKnownSideDirection !=
            survivingPassage.hasKnownSideDirection)
        {
            hasIncompletePassage = true;
        }
        if (!absorbedPassage.hasKnownSideDirection ||
            !survivingPassage.hasKnownSideDirection)
        {
            hasIncompletePassage = true;
        }
        if (absorbedPassage.hasKnownSideDirection &&
            survivingPassage.hasKnownSideDirection)
        {
            const Eigen::Vector3d transformedDirection =
                rotation * absorbedPassage.knownSideDirection_world;
            const double directionAgreement =
                transformedDirection.normalized().dot(
                    survivingPassage.knownSideDirection_world.normalized());
            if (!std::isfinite(directionAgreement) ||
                directionAgreement < configuration_in.minAbsCosNormalAngle)
            {
                contradictionReason_out =
                    SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION;
                alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
            }
        }
    }

    if (matchedCount_out == 0U)
    {
        contradictionReason_out =
            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
        alignmentCheck_out = AlignmentCheck::CONTRADICTION;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    alignmentCheck_out = hasIncompletePassage ? AlignmentCheck::MISSING
                                              : AlignmentCheck::ALIGNED;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
