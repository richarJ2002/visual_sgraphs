

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

SemanticVerifyStatus checkConsecutivePassageTopology(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3                              &transform_in,
    double                                        maximumCentroidDistance_m_in,
    std::size_t                                  &matchedCount_out,
    SemanticMergeReason                          &contradictionReason_out,
    AlignmentCheck                               &alignmentCheck_out)
{
    matchedCount_out          = 0U;
    bool hasIncompletePassage = false;

    /* Endpoint identity is tag-based: "t:<tag>" for tagged rooms (stable
     * across maps), side-prefixed "a:u:<id>"/"s:u:<id>" for untagged ones
     * (never equal across maps), empty for a genuinely absent side. */
    std::map<int, std::string> survivingIdToTag;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        survivingIdToTag.emplace(room.context.roomId, room.context.roomTag);
    }
    std::map<int, std::string> absorbedIdToTag;
    for (const SemanticMergeRoomEvidence &room : absorbedRooms_in)
    {
        absorbedIdToTag.emplace(room.context.roomId, room.context.roomTag);
    }
    const auto endpointKey = [](bool                              hasSide_in,
                                int                               roomId_in,
                                const std::map<int, std::string> &idToTag_in,
                                const char                       *sidePrefix_in)
    {
        if (!hasSide_in)
        {
            return std::string();
        }
        const std::map<int, std::string>::const_iterator tagMatch =
            idToTag_in.find(roomId_in);
        if (tagMatch != idToTag_in.end() && !tagMatch->second.empty())
        {
            return std::string("t:") + tagMatch->second;
        }
        return std::string(sidePrefix_in) + std::to_string(roomId_in);
    };

    std::vector<const PassageContext *>   survivingPassages;
    std::map<int, const PassageContext *> survivingById;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        for (const PassageContext &passage : room.context.passageContexts)
        {
            survivingById.emplace(passage.id, &passage);
            bool isUsable{};
            if (passageGeometryIsUsable(passage, isUsable) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: passageGeometryIsUsable returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (isUsable)
            {
                survivingPassages.push_back(&passage);
            }
        }
    }

    bool hasAbsorbedPassageEvidence = false;
    for (const SemanticMergeRoomEvidence &room : absorbedRooms_in)
    {
        for (const PassageContext &absorbedPassage :
             room.context.passageContexts)
        {
            /* Same stable ID means same doorway lineage: passage IDs are
             * mission-unique, so a proxy on either side pairs by lineage
             * even without geometry, and content re-derives at fusion.
             * A both-real same-ID pair still runs the full endpoint and
             * geometry checks below. */
            const std::map<int, const PassageContext *>::iterator lineageMatch =
                survivingById.find(absorbedPassage.id);
            if (lineageMatch != survivingById.end() &&
                (absorbedPassage.isRecoveryProxy ||
                 lineageMatch->second->isRecoveryProxy))
            {
                if (absorbedPassage.isRecoveryProxy &&
                    lineageMatch->second->isRecoveryProxy)
                {
                    /* Dormant pair: no constraint either way. */
                    continue;
                }
                if (lineageMatch->second->isPassable !=
                    absorbedPassage.isPassable)
                {
                    contradictionReason_out =
                        SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
                    alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
                }
                ++matchedCount_out;
                hasAbsorbedPassageEvidence = true;
                continue;
            }
            bool isUsable2{};
            if (passageGeometryIsUsable(absorbedPassage, isUsable2) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: passageGeometryIsUsable returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!isUsable2)
            {
                /* Recovery proxies and geometry-less hypotheses contribute
                 * no constraints. */
                continue;
            }
            hasAbsorbedPassageEvidence = true;
            const std::string absorbedKnownKey =
                endpointKey(absorbedPassage.hasKnownSideRoom,
                            absorbedPassage.knownSideRoomId,
                            absorbedIdToTag,
                            "a:u:");
            const std::string absorbedFarKey =
                endpointKey(absorbedPassage.hasFarSideRoom,
                            absorbedPassage.secondaryRoomId,
                            absorbedIdToTag,
                            "a:u:");

            bool                  paired                 = false;
            const PassageContext *p_conflictingSurviving = nullptr;
            for (const PassageContext *p_surviving : survivingPassages)
            {
                const std::string survivingKnownKey =
                    endpointKey(p_surviving->hasKnownSideRoom,
                                p_surviving->knownSideRoomId,
                                survivingIdToTag,
                                "s:u:");
                const std::string survivingFarKey =
                    endpointKey(p_surviving->hasFarSideRoom,
                                p_surviving->secondaryRoomId,
                                survivingIdToTag,
                                "s:u:");
                if (absorbedPassage.hasKnownSideRoom ==
                        p_surviving->hasKnownSideRoom &&
                    absorbedPassage.hasFarSideRoom ==
                        p_surviving->hasFarSideRoom &&
                    absorbedKnownKey == survivingKnownKey &&
                    absorbedFarKey == survivingFarKey)
                {
                    paired = true;
                    if (p_surviving->isPassable != absorbedPassage.isPassable)
                    {
                        contradictionReason_out =
                            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
                        alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                        return SemanticVerifyStatus::
                            SEMANTIC_VERIFY_STATUS_SUCCESS;
                    }
                    if (absorbedPassage.hasKnownSideDirection &&
                        p_surviving->hasKnownSideDirection)
                    {
                        const Eigen::Vector3d transformedDirection =
                            transform_in.rotation()
                                .toRotationMatrix()
                                .cast<double>() *
                            absorbedPassage.knownSideDirection_World;
                        const double directionAgreement =
                            transformedDirection.normalized().dot(
                                p_surviving->knownSideDirection_World
                                    .normalized());
                        if (!std::isfinite(directionAgreement) ||
                            directionAgreement <
                                kConsecutiveMinDirectionAgreement)
                        {
                            contradictionReason_out = SemanticMergeReason::
                                PASSAGE_DIRECTION_CONTRADICTION;
                            alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                            return SemanticVerifyStatus::
                                SEMANTIC_VERIFY_STATUS_SUCCESS;
                        }
                    }
                    else
                    {
                        hasIncompletePassage = true;
                    }
                    Eigen::Vector3d mappedCentroid = Eigen::Vector3d::Zero();
                    if (!(transformAbsorbedPoint(transform_in,
                                                 absorbedPassage.centroid_World,
                                                 mappedCentroid) ==
                          SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS))
                    {
                        hasIncompletePassage = true;
                    }
                    else if ((mappedCentroid - p_surviving->centroid_World)
                                 .norm() > maximumCentroidDistance_m_in)
                    {
                        contradictionReason_out =
                            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
                        alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                        return SemanticVerifyStatus::
                            SEMANTIC_VERIFY_STATUS_SUCCESS;
                    }
                }
                else
                {
                    /* Partial overlap on one translated present side while
                     * both sides disagree on the other is positive evidence
                     * of different doorways, not of missing evidence. */
                    if (!absorbedKnownKey.empty() &&
                        absorbedKnownKey == survivingKnownKey &&
                        absorbedPassage.hasFarSideRoom &&
                        p_surviving->hasFarSideRoom &&
                        !absorbedFarKey.empty() && !survivingFarKey.empty() &&
                        absorbedFarKey != survivingFarKey)
                    {
                        p_conflictingSurviving = p_surviving;
                    }
                    if (!absorbedFarKey.empty() &&
                        absorbedFarKey == survivingFarKey &&
                        absorbedPassage.hasKnownSideRoom &&
                        p_surviving->hasKnownSideRoom &&
                        !absorbedKnownKey.empty() &&
                        !survivingKnownKey.empty() &&
                        absorbedKnownKey != survivingKnownKey)
                    {
                        p_conflictingSurviving = p_surviving;
                    }
                }
            }
            if (paired)
            {
                ++matchedCount_out;
            }
            else if (p_conflictingSurviving != nullptr)
            {
                contradictionReason_out =
                    SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
                alignmentCheck_out = AlignmentCheck::CONTRADICTION;
                return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
            }
            else
            {
                hasIncompletePassage = true;
            }
        }
    }

    if (!hasAbsorbedPassageEvidence)
    {
        contradictionReason_out = SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        alignmentCheck_out      = AlignmentCheck::MISSING;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
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

} // namespace semantic
} // namespace core
} // namespace vs_graphs
