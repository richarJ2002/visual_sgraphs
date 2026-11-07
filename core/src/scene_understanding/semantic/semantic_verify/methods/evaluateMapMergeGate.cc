/*!
 * @file            evaluateMapMergeGate.cc
 *
 * @brief           Implements SemanticVerify::evaluateMapMergeGate(), declared
 *                  in Semantic/SemanticVerify.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"

#include "configuration/types/types.h"
#include "scene_understanding/scene_graph/Room/objects/RoomClass.h"
#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <rclcpp/logging.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus SemanticVerify::evaluateMapMergeGate(
    atlas::Atlas               *p_atlas_in,
    map::Map                   *p_survivingMap_in,
    map::Map                   *p_absorbedMap_in,
    const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
    SemanticMergeGateResult    &result_out,
    const SemanticVerifyConfig &configuration_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision   = SemanticMergeDecision::REJECT;
        result.reason     = SemanticMergeReason::INVALID_INPUT;
        result.pairDetail = "input=invalid";
        result_out        = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    bool isVerified{};
    if (verifyLoopMergeFloors(p_survivingMap_in,
                              p_absorbedMap_in,
                              mergeTransform_absorbedToSurviving_in,
                              result.floorDecision,
                              isVerified) !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: verifyLoopMergeFloors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isVerified)
    {
        result.decision   = result.floorDecision == "REJECTED"
                                ? SemanticMergeDecision::REJECT
                                : SemanticMergeDecision::DEFER;
        result.reason     = result.decision == SemanticMergeDecision::REJECT
                                ? SemanticMergeReason::FLOOR_CONTRADICTION
                                : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        result.pairDetail = "floor_gate=" + result.floorDecision;
        result_out        = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    unsigned long            survivingMapId{};
    unsigned long            absorbedMapId{};
    if ((p_atlas_in->getSceneGraph(p_sceneGraph) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_survivingMap_in->getId(survivingMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_absorbedMap_in->getId(absorbedMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<Room *> survivingMapAllRooms{};
    if (p_sceneGraph->getMapRoomsInIdOrder(survivingMapId,
                                           survivingMapAllRooms) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the scene graph has no scene map for map %lu; "
                     "continuing without its rooms.",
                     __func__,
                     survivingMapId);
    }
    std::vector<Room *> absorbedMapAllRooms{};
    if (p_sceneGraph->getMapRoomsInIdOrder(absorbedMapId,
                                           absorbedMapAllRooms) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the scene graph has no scene map for map %lu; "
                     "continuing without its rooms.",
                     __func__,
                     absorbedMapId);
    }
    std::vector<Room *>                    survivingAdmittedRooms{};
    std::vector<Room *>                    absorbedAdmittedRooms{};
    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    collectRoomEvidence(survivingMapAllRooms,
                        configuration_in,
                        survivingAdmittedRooms,
                        survivingRooms);
    collectRoomEvidence(absorbedMapAllRooms,
                        configuration_in,
                        absorbedAdmittedRooms,
                        absorbedRooms);

    /* The anchor candidates are the map chain link: the earlier map's final
     * room (and the room beyond the passage in view at departure) with the
     * later map's starting room. */
    std::vector<Room *> survivingLinkedRooms{};
    std::vector<Room *> absorbedLinkedRooms{};
    if (p_survivingMap_in->getLinkedRooms(p_absorbedMap_in,
                                          survivingLinkedRooms,
                                          absorbedLinkedRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLinkedRooms returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<std::pair<Room *, Room *>> anchorCandidates{};
    for (Room *p_survivingLinkedRoom : survivingLinkedRooms)
    {
        for (Room *p_absorbedLinkedRoom : absorbedLinkedRooms)
        {
            anchorCandidates.emplace_back(p_survivingLinkedRoom,
                                          p_absorbedLinkedRoom);
        }
    }
    /* Without a link the one empty candidate defers for a missing anchor. */
    if (anchorCandidates.empty())
    {
        anchorCandidates.emplace_back(nullptr, nullptr);
    }

    std::vector<MergePair>               allCandidatePairs{};
    std::vector<SemanticMergeGateResult> candidateResults{};
    for (const std::pair<Room *, Room *> &anchorCandidate : anchorCandidates)
    {
        const std::vector<MergePair> anchorPairs =
            findAnchorRoomPairs(survivingAdmittedRooms,
                                absorbedAdmittedRooms,
                                anchorCandidate.first,
                                anchorCandidate.second);
        allCandidatePairs.insert(allCandidatePairs.end(),
                                 anchorPairs.begin(),
                                 anchorPairs.end());
        SemanticMergeGateResult candidateResult{};
        if (evaluateMergeAlignment(survivingRooms,
                                   absorbedRooms,
                                   anchorPairs,
                                   mergeTransform_absorbedToSurviving_in,
                                   candidateResult,
                                   configuration_in) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: evaluateMergeAlignment returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        candidateResult.p_survivingAnchorRoom = anchorCandidate.first;
        candidateResult.p_absorbedAnchorRoom  = anchorCandidate.second;
        candidateResults.push_back(candidateResult);
        if (anchorCandidates.size() < 2U)
        {
            continue;
        }

        /* With two candidates every try is logged; the caller logs only the
         * chosen one. */
        const char *p_decisionName = nullptr;
        const char *p_reasonName   = nullptr;
        if (mergeDecisionName(candidateResult.decision, p_decisionName) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS ||
            mergeReasonName(candidateResult.reason, p_reasonName) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mergeDecisionName/mergeReasonName returned a "
                         "failure status although they cannot fail; "
                         "continuing as before.",
                         __func__);
        }
        std::cout << "[SemanticMergePairs] surviving_map=" << survivingMapId
                  << " absorbed_map=" << absorbedMapId
                  << " anchor_candidate=" << candidateResults.size() << "/"
                  << anchorCandidates.size() << " decision=" << p_decisionName
                  << " reason=" << p_reasonName << " "
                  << candidateResult.pairDetail << std::endl;
    }

    /* Exactly one accepted candidate is the anchor. Two accepted candidates
     * are checked together, where the shared starting room pairs twice and
     * the existing pairing rule rejects. With none accepted, a deferred
     * candidate may still pass later, so it is kept; otherwise the final
     * room's result stands. */
    std::size_t acceptedCount = 0U;
    std::size_t acceptedIndex = 0U;
    std::size_t deferredIndex = candidateResults.size();
    for (std::size_t candidateIndex = 0U;
         candidateIndex < candidateResults.size();
         candidateIndex++)
    {
        const SemanticMergeDecision candidateDecision =
            candidateResults[candidateIndex].decision;
        if (candidateDecision == SemanticMergeDecision::ACCEPT)
        {
            acceptedCount++;
            acceptedIndex = candidateIndex;
        }
        else if (candidateDecision == SemanticMergeDecision::DEFER &&
                 deferredIndex == candidateResults.size())
        {
            deferredIndex = candidateIndex;
        }
    }
    std::string anchorChoice;
    if (acceptedCount == 1U)
    {
        result       = candidateResults[acceptedIndex];
        anchorChoice = std::to_string(acceptedIndex + 1U);
    }
    else if (acceptedCount > 1U)
    {
        if (evaluateMergeAlignment(survivingRooms,
                                   absorbedRooms,
                                   allCandidatePairs,
                                   mergeTransform_absorbedToSurviving_in,
                                   result,
                                   configuration_in) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: evaluateMergeAlignment returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        anchorChoice = "ambiguous";
    }
    else if (deferredIndex < candidateResults.size())
    {
        result       = candidateResults[deferredIndex];
        anchorChoice = "none_passed";
    }
    else
    {
        result       = candidateResults.front();
        anchorChoice = "none_passed";
    }
    if (anchorCandidates.size() > 1U)
    {
        result.pairDetail = "anchor_choice=" + anchorChoice + "/" +
                            std::to_string(anchorCandidates.size()) + " " +
                            result.pairDetail;
    }
    result.floorDecision = "ACCEPTED";
    result_out           = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
