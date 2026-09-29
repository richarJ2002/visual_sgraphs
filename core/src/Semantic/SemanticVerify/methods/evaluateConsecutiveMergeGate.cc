

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

SemanticVerifyStatus SemanticVerify::evaluateConsecutiveMergeGate(
    core::Map               *p_survivingMap_in,
    core::Map               *p_absorbedMap_in,
    const g2o::Sim3         &transform_absorbedToSurviving_in,
    SemanticMergeGateResult &result_out,
    const MapMergeConfig    &configuration_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::INVALID_INPUT;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    bool floorsMatch{};
    if (checkConsecutiveFloors(p_survivingMap_in,
                               p_absorbedMap_in,
                               transform_absorbedToSurviving_in,
                               configuration_in.floor_match_tolerance_m,
                               result.floorDecision,
                               floorsMatch) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkConsecutiveFloors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!floorsMatch)
    {
        result.decision = result.floorDecision == "REJECTED"
                              ? SemanticMergeDecision::REJECT
                              : SemanticMergeDecision::DEFER;
        result.reason   = result.decision == SemanticMergeDecision::REJECT
                              ? SemanticMergeReason::FLOOR_CONTRADICTION
                              : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    result.floorDecision = "ACCEPTED";

    SemanticVerifyConfig verifyConfiguration;
    verifyConfiguration.maxNormalAngle_deg =
        configuration_in.wall_coplanar_angle_deg;

    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    std::vector<semantic::Room *>          survivingMapAllRooms{};
    if (p_survivingMap_in->getAllRooms(survivingMapAllRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Room *p_room : survivingMapAllRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Room::RoomVariant roomVariant{};
        if ((p_room != nullptr && !roomIsBad) &&
            p_room->getRoomVariant(roomVariant) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad &&
            roomVariant == Room::RoomVariant::ROOM)
        {
            SemanticMergeRoomEvidence evidence{};
            if (copyMergeRoomEvidence(p_room, verifyConfiguration, evidence) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: copyMergeRoomEvidence returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            survivingRooms.push_back(evidence);
        }
    }
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    std::vector<semantic::Room *>          absorbedMapAllRooms{};
    if (p_absorbedMap_in->getAllRooms(absorbedMapAllRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Room *p_room : absorbedMapAllRooms)
    {
        bool roomIsBad2{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad2) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Room::RoomVariant roomVariant2{};
        if ((p_room != nullptr && !roomIsBad2) &&
            p_room->getRoomVariant(roomVariant2) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad2 &&
            roomVariant2 == Room::RoomVariant::ROOM)
        {
            SemanticMergeRoomEvidence evidence2{};
            if (copyMergeRoomEvidence(p_room, verifyConfiguration, evidence2) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: copyMergeRoomEvidence returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            absorbedRooms.push_back(evidence2);
        }
    }

    std::vector<ConsecutiveAnchorPair> anchorPairs{};
    if (collectConsecutiveAnchors(survivingRooms, absorbedRooms, anchorPairs) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectConsecutiveAnchors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    result.sharedRoomCount = anchorPairs.size();
    if (anchorPairs.empty())
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    /* Room-prior seed: the old final room and the new starting room must
     * be the same tag-matched anchor. */
    Room *p_oldFinalRoom = nullptr;
    if (p_absorbedMap_in->getFinalRoom(p_oldFinalRoom) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFinalRoom returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Room *p_newStartRoom = nullptr;
    if (p_survivingMap_in->getStartingRoom(p_newStartRoom) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getStartingRoom returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    bool seedAnchored = false;
    bool oldFinalRoomHasRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr) &&
        p_oldFinalRoom->hasRoomTag(oldFinalRoomHasRoomTag) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool newStartRoomHasRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag) &&
        p_newStartRoom->hasRoomTag(newStartRoomHasRoomTag) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::string oldFinalRoomRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag && newStartRoomHasRoomTag) &&
        p_oldFinalRoom->getRoomTag(oldFinalRoomRoomTag) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::string oldFinalRoomRoomTag2{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag && newStartRoomHasRoomTag &&
         !oldFinalRoomRoomTag.empty()) &&
        p_oldFinalRoom->getRoomTag(oldFinalRoomRoomTag2) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::string newStartRoomRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag && newStartRoomHasRoomTag &&
         !oldFinalRoomRoomTag.empty()) &&
        p_newStartRoom->getRoomTag(newStartRoomRoomTag) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
        oldFinalRoomHasRoomTag && newStartRoomHasRoomTag &&
        !oldFinalRoomRoomTag.empty() &&
        oldFinalRoomRoomTag2 == newStartRoomRoomTag)
    {
        for (const ConsecutiveAnchorPair &pair : anchorPairs)
        {
            std::string newStartRoomRoomTag2{};
            if (p_newStartRoom->getRoomTag(newStartRoomRoomTag2) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (pair.p_surviving->context.roomTag == newStartRoomRoomTag2)
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
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    AlignmentCheck centroidCheck{};
    if (checkAnchorRoomCentroids(anchorPairs,
                                 transform_absorbedToSurviving_in,
                                 configuration_in.passage_match_tolerance_m,
                                 centroidCheck) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkAnchorRoomCentroids returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (centroidCheck == AlignmentCheck::CONTRADICTION)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    if (centroidCheck == AlignmentCheck::MISSING)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::WALL_EVIDENCE_MISSING;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    bool                hasMissingEvidence = false;
    std::size_t         matchedPassages    = 0U;
    SemanticMergeReason topologyReason =
        SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
    AlignmentCheck topologyCheck{};
    if (checkConsecutivePassageTopology(
            survivingRooms,
            absorbedRooms,
            transform_absorbedToSurviving_in,
            configuration_in.passage_match_tolerance_m,
            matchedPassages,
            topologyReason,
            topologyCheck) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkConsecutivePassageTopology returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    result.matchedPassageCount = matchedPassages;
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

    std::size_t alignedRoomCount = 0U;
    for (const ConsecutiveAnchorPair &pair : anchorPairs)
    {
        std::size_t    pairMatchedWalls = 0U;
        AlignmentCheck wallCheck{};
        if (checkFixedTransformWalls(pair.p_surviving->walls,
                                     pair.p_absorbed->walls,
                                     transform_absorbedToSurviving_in,
                                     verifyConfiguration,
                                     pairMatchedWalls,
                                     wallCheck) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: checkFixedTransformWalls returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        result.matchedWallCount += pairMatchedWalls;
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
            continue;
        }
        AlignmentCheck alignmentCheck{};
        if (checkConsecutiveWallEdgeOverlap(
                {pair},
                transform_absorbedToSurviving_in,
                configuration_in.wall_coplanar_angle_deg,
                configuration_in.wall_edge_overlap_m,
                alignmentCheck) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: checkConsecutiveWallEdgeOverlap returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (alignmentCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            result_out      = result;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        ++alignedRoomCount;
    }

    if (hasMissingEvidence || alignedRoomCount != anchorPairs.size())
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
