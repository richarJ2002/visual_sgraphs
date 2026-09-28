

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

AlignmentCheck
    checkPassageTopology(const RoomContextSnapshot  &survivingContext_in,
                         const RoomContextSnapshot  &absorbedContext_in,
                         const g2o::Sim3            &transform_in,
                         const SemanticVerifyConfig &configuration_in,
                         std::size_t                &matchedCount_out,
                         SemanticMergeReason        &contradictionReason_out)
{
    matchedCount_out = 0U;
    if (survivingContext_in.passageContexts.empty() ||
        absorbedContext_in.passageContexts.empty())
    {
        return AlignmentCheck::MISSING;
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
    for (const auto &entry : absorbedPassages)
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
            return AlignmentCheck::CONTRADICTION;
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
            return AlignmentCheck::CONTRADICTION;
        }
        if (absorbedPassage.hasFarSideRoom && survivingPassage.hasFarSideRoom &&
            absorbedPassage.secondaryRoomId != survivingPassage.secondaryRoomId)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
            return AlignmentCheck::CONTRADICTION;
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
                rotation * absorbedPassage.knownSideDirection_World;
            const double directionAgreement =
                transformedDirection.normalized().dot(
                    survivingPassage.knownSideDirection_World.normalized());
            if (!std::isfinite(directionAgreement) ||
                directionAgreement < configuration_in.minAbsCosNormalAngle)
            {
                contradictionReason_out =
                    SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION;
                return AlignmentCheck::CONTRADICTION;
            }
        }
    }

    if (matchedCount_out == 0U)
    {
        contradictionReason_out =
            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
        return AlignmentCheck::CONTRADICTION;
    }
    return hasIncompletePassage ? AlignmentCheck::MISSING
                                : AlignmentCheck::ALIGNED;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
