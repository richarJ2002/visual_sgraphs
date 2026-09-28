

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

SemanticVerifyStatus SemanticVerify::evaluateMergeAlignment(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3            &transform_absorbedToSurviving_in,
    SemanticMergeGateResult    &result_out,
    const SemanticVerifyConfig &configuration_in)
{
    SemanticMergeGateResult                                  result;
    std::map<std::string, const SemanticMergeRoomEvidence *> survivingById;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        std::string identity{};
        if (stableRoomIdentity(room.context, identity) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            // stableRoomIdentity cannot fail; continue as before.
        }
        survivingById.emplace(identity, &room);
    }

    bool hasMissingEvidence = false;
    for (const SemanticMergeRoomEvidence &absorbedRoom : absorbedRooms_in)
    {
        std::string identity2{};
        if (stableRoomIdentity(absorbedRoom.context, identity2) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            // stableRoomIdentity cannot fail; continue as before.
        }
        const std::map<std::string,
                       const SemanticMergeRoomEvidence *>::const_iterator
            match = survivingById.find(identity2);
        if (match == survivingById.end())
        {
            continue;
        }
        ++result.sharedRoomCount;

        std::size_t    matchedWalls = 0U;
        AlignmentCheck wallCheck{};
        if (checkFixedTransformWalls(match->second->walls,
                                     absorbedRoom.walls,
                                     transform_absorbedToSurviving_in,
                                     configuration_in,
                                     matchedWalls,
                                     wallCheck) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            // checkFixedTransformWalls cannot fail; continue as before.
        }
        result.matchedWallCount += matchedWalls;
        if (wallCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            result_out      = result;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        if (wallCheck == AlignmentCheck::MISSING)
        {
            hasMissingEvidence = true;
            result.reason      = SemanticMergeReason::WALL_EVIDENCE_MISSING;
        }

        std::size_t         matchedPassages = 0U;
        SemanticMergeReason topologyReason =
            SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        AlignmentCheck topologyCheck{};
        if (checkPassageTopology(match->second->context,
                                 absorbedRoom.context,
                                 transform_absorbedToSurviving_in,
                                 configuration_in,
                                 matchedPassages,
                                 topologyReason,
                                 topologyCheck) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            // checkPassageTopology cannot fail; continue as before.
        }
        result.matchedPassageCount += matchedPassages;
        if (topologyCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = topologyReason;
            result_out      = result;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        if (topologyCheck == AlignmentCheck::MISSING)
        {
            hasMissingEvidence = true;
            result.reason      = SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        }
        if (wallCheck == AlignmentCheck::ALIGNED &&
            topologyCheck == AlignmentCheck::ALIGNED)
        {
            ++result.alignedRoomCount;
        }
    }

    if (result.sharedRoomCount == 0U)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    if (hasMissingEvidence || result.alignedRoomCount != result.sharedRoomCount)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    result.decision = SemanticMergeDecision::ACCEPT;
    result.reason   = SemanticMergeReason::ALIGNED;
    result_out      = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
