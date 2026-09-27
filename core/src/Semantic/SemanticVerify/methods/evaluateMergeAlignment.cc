

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

SemanticMergeGateResult SemanticVerify::evaluateMergeAlignment(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3            &transform_absorbedToSurviving_in,
    const SemanticVerifyConfig &config_in)
{
    SemanticMergeGateResult                                  result;
    std::map<std::string, const SemanticMergeRoomEvidence *> survivingById;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        survivingById.emplace(stableRoomIdentity(room.context), &room);
    }

    bool hasMissingEvidence = false;
    for (const SemanticMergeRoomEvidence &absorbedRoom : absorbedRooms_in)
    {
        const std::map<std::string,
                       const SemanticMergeRoomEvidence *>::const_iterator
            match =
                survivingById.find(stableRoomIdentity(absorbedRoom.context));
        if (match == survivingById.end())
        {
            continue;
        }
        ++result.sharedRoomCount;

        std::size_t          matchedWalls = 0U;
        const AlignmentCheck wallCheck =
            checkFixedTransformWalls(match->second->walls,
                                     absorbedRoom.walls,
                                     transform_absorbedToSurviving_in,
                                     config_in,
                                     matchedWalls);
        result.matchedWallCount += matchedWalls;
        if (wallCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            return result;
        }
        if (wallCheck == AlignmentCheck::MISSING)
        {
            hasMissingEvidence = true;
            result.reason      = SemanticMergeReason::WALL_EVIDENCE_MISSING;
        }

        std::size_t         matchedPassages = 0U;
        SemanticMergeReason topologyReason =
            SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        const AlignmentCheck topologyCheck =
            checkPassageTopology(match->second->context,
                                 absorbedRoom.context,
                                 transform_absorbedToSurviving_in,
                                 config_in,
                                 matchedPassages,
                                 topologyReason);
        result.matchedPassageCount += matchedPassages;
        if (topologyCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = topologyReason;
            return result;
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
        return result;
    }
    if (hasMissingEvidence || result.alignedRoomCount != result.sharedRoomCount)
    {
        result.decision = SemanticMergeDecision::DEFER;
        return result;
    }
    result.decision = SemanticMergeDecision::ACCEPT;
    result.reason   = SemanticMergeReason::ALIGNED;
    return result;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
