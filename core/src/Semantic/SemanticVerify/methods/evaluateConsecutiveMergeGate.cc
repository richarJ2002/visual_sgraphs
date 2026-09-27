

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

SemanticMergeGateResult SemanticVerify::evaluateConsecutiveMergeGate(
    core::Map            *p_survivingMap_in,
    core::Map            *p_absorbedMap_in,
    const g2o::Sim3      &transform_absorbedToSurviving_in,
    const MapMergeConfig &config_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::INVALID_INPUT;
        return result;
    }

    if (!checkConsecutiveFloors(p_survivingMap_in,
                                p_absorbedMap_in,
                                transform_absorbedToSurviving_in,
                                config_in.floor_match_tolerance_m,
                                result.floorDecision))
    {
        result.decision = result.floorDecision == "REJECTED"
                              ? SemanticMergeDecision::REJECT
                              : SemanticMergeDecision::DEFER;
        result.reason   = result.decision == SemanticMergeDecision::REJECT
                              ? SemanticMergeReason::FLOOR_CONTRADICTION
                              : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        return result;
    }
    result.floorDecision = "ACCEPTED";

    SemanticVerifyConfig verifyConfig;
    verifyConfig.maxNormalAngle_deg = config_in.wall_coplanar_angle_deg;

    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    for (Room *p_room : p_survivingMap_in->getAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::RoomVariant::ROOM)
        {
            survivingRooms.push_back(
                copyMergeRoomEvidence(p_room, verifyConfig));
        }
    }
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    for (Room *p_room : p_absorbedMap_in->getAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::RoomVariant::ROOM)
        {
            absorbedRooms.push_back(
                copyMergeRoomEvidence(p_room, verifyConfig));
        }
    }

    const std::vector<ConsecutiveAnchorPair> anchorPairs =
        collectConsecutiveAnchors(survivingRooms, absorbedRooms);
    result.sharedRoomCount = anchorPairs.size();
    if (anchorPairs.empty())
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        return result;
    }

    /* Room-prior seed: the old final room and the new starting room must
     * be the same tag-matched anchor. */
    Room *p_oldFinalRoom = p_absorbedMap_in->getFinalRoom();
    Room *p_newStartRoom = p_survivingMap_in->getStartingRoom();
    bool  seedAnchored   = false;
    if (p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
        p_oldFinalRoom->hasRoomTag() && p_newStartRoom->hasRoomTag() &&
        !p_oldFinalRoom->getRoomTag().empty() &&
        p_oldFinalRoom->getRoomTag() == p_newStartRoom->getRoomTag())
    {
        for (const ConsecutiveAnchorPair &pair : anchorPairs)
        {
            if (pair.p_surviving->context.roomTag ==
                p_newStartRoom->getRoomTag())
            {
                seedAnchored = true;
                break;
            }
        }
    }
    if (!seedAnchored)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        return result;
    }

    const AlignmentCheck centroidCheck =
        checkAnchorRoomCentroids(anchorPairs,
                                 transform_absorbedToSurviving_in,
                                 config_in.passage_match_tolerance_m);
    if (centroidCheck == AlignmentCheck::CONTRADICTION)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
        return result;
    }
    if (centroidCheck == AlignmentCheck::MISSING)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::WALL_EVIDENCE_MISSING;
        return result;
    }

    bool                hasMissingEvidence = false;
    std::size_t         matchedPassages    = 0U;
    SemanticMergeReason topologyReason =
        SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
    const AlignmentCheck topologyCheck =
        checkConsecutivePassageTopology(survivingRooms,
                                        absorbedRooms,
                                        transform_absorbedToSurviving_in,
                                        config_in.passage_match_tolerance_m,
                                        matchedPassages,
                                        topologyReason);
    result.matchedPassageCount = matchedPassages;
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

    std::size_t alignedRoomCount = 0U;
    for (const ConsecutiveAnchorPair &pair : anchorPairs)
    {
        std::size_t          pairMatchedWalls = 0U;
        const AlignmentCheck wallCheck =
            checkFixedTransformWalls(pair.p_surviving->walls,
                                     pair.p_absorbed->walls,
                                     transform_absorbedToSurviving_in,
                                     verifyConfig,
                                     pairMatchedWalls);
        result.matchedWallCount += pairMatchedWalls;
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
            continue;
        }
        if (checkConsecutiveWallEdgeOverlap({pair},
                                            transform_absorbedToSurviving_in,
                                            config_in.wall_coplanar_angle_deg,
                                            config_in.wall_edge_overlap_m) ==
            AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            return result;
        }
        ++alignedRoomCount;
    }

    if (hasMissingEvidence || alignedRoomCount != anchorPairs.size())
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
