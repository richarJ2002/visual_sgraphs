/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            evaluateTopCandidateVerification.cc
 *
 * @brief           Implements
 *                  SemanticsManager::evaluateTopCandidateVerification(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "Semantic/SemanticCandidates.h"
#include "Semantic/SemanticVerify.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::evaluateTopCandidateVerification(
    const std::vector<semantic::SemanticCandidate> &candidates_in)
{
    if (candidates_in.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const semantic::SemanticCandidate &topCandidate = candidates_in.front();
    if (!topCandidate.isMinimumEvidenceSatisfied)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* SemanticCandidates::generateWithStatus() marks every candidate within
     * ambiguityMargin of the best distance as `ambiguous`, including the
     * best candidate itself -- its own distance trivially satisfies
     * "<= best distance + margin", so `topCandidate.ambiguous` is always
     * true and can never be read as a gate on its own. A genuine tie instead
     * shows up as a SECOND candidate also carrying `ambiguous == true`. */
    const bool topCandidateIsUniqueLeader =
        candidates_in.size() == 1U || !candidates_in[1].isAmbiguous;
    if (!topCandidateIsUniqueLeader)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    semantic::Room *p_roomA = nullptr;
    if (findRoomByMapAndId(topCandidate.mapAId,
                           topCandidate.roomAId,
                           p_roomA) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: findRoomByMapAndId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Room *p_roomB = nullptr;
    if (findRoomByMapAndId(topCandidate.mapBId,
                           topCandidate.roomBId,
                           p_roomB) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: findRoomByMapAndId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_roomA == nullptr || p_roomB == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    semantic::SemanticVerifyConfig verifyConfiguration{};
    if (semantic::SemanticVerify::configFromSystemParams(verifyConfiguration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: configFromSystemParams returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::VerifyWallObservation> wallsA{};
    if (semantic::SemanticVerify::collectWallObservations(p_roomA,
                                                          verifyConfiguration,
                                                          wallsA) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectWallObservations returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::VerifyWallObservation> wallsB{};
    if (semantic::SemanticVerify::collectWallObservations(p_roomB,
                                                          verifyConfiguration,
                                                          wallsB) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectWallObservations returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    semantic::SemanticVerifyResult result{};
    if (semantic::SemanticVerify::verify(wallsA,
                                         wallsB,
                                         result,
                                         verifyConfiguration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: verify returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* The floor gate can only turn a geometric PASS into a final rejection
     * (toVerificationVerdict() ANDs pass with hasFloorGatePassed), so only run
     * it -- and only when both rooms actually carry a floor identity to
     * compare -- when that outcome is in play; skipping it here (as opposed
     * to skipping it when the two rooms could plausibly share a floor) would
     * be what SemanticVerify.h's runFloorGate() doc warns under-reports a
     * real pass as a false negative. */
    semantic::Floor *p_floorA = nullptr;
    if (p_roomA->getFloor(p_floorA) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Floor *p_floorB = nullptr;
    if (p_roomB->getFloor(p_floorB) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool floorAHasPlaneIdentity{};
    if ((result.hasPassed && p_floorA != nullptr && p_floorB != nullptr) &&
        p_floorA->hasPlaneIdentity(floorAHasPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    bool floorBHasPlaneIdentity{};
    if ((result.hasPassed && p_floorA != nullptr && p_floorB != nullptr &&
         floorAHasPlaneIdentity) &&
        p_floorB->hasPlaneIdentity(floorBHasPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (result.hasPassed && p_floorA != nullptr && p_floorB != nullptr &&
        floorAHasPlaneIdentity && floorBHasPlaneIdentity)
    {
        /* verify()'s roomTransform_roomAToRoomB maps room-A points into room
         * B's frame, i.e. A is absorbed into B -- matches runFloorGate's
         * absorbed->surviving convention. */
        core::Map *p_roomBMap = nullptr;
        if (p_roomB->getMap(p_roomBMap) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        core::Map *p_roomAMap = nullptr;
        if (p_roomA->getMap(p_roomAMap) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool hasPassed2{};
        if (semantic::SemanticVerify::runFloorGate(
                result,
                p_roomBMap,
                p_roomAMap,
                result.roomTransform_roomAToRoomB,
                hasPassed2) !=
            semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: runFloorGate returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    const auto rejectReasonName = [](semantic::VerifyRejectReason reason)
    {
        switch (reason)
        {
        case semantic::VerifyRejectReason::NONE:
            return "NONE";
        case semantic::VerifyRejectReason::TOO_FEW_WALLS:
            return "TOO_FEW_WALLS";
        case semantic::VerifyRejectReason::NO_VALID_HYPOTHESIS:
            return "NO_VALID_HYPOTHESIS";
        case semantic::VerifyRejectReason::AMBIGUOUS_TOP_HYPOTHESES:
            return "AMBIGUOUS_TOP_HYPOTHESES";
        case semantic::VerifyRejectReason::BELOW_MIN_INLIER_RATIO:
            return "BELOW_MIN_INLIER_RATIO";
        case semantic::VerifyRejectReason::REFINED_FIT_NOT_OBSERVABLE:
            return "REFINED_FIT_NOT_OBSERVABLE";
        }
        return "unknown";
    };

    std::cout << "[SemMgr] verification_evaluated mapA=" << topCandidate.mapAId
              << " roomA=" << topCandidate.roomAId
              << " mapB=" << topCandidate.mapBId
              << " roomB=" << topCandidate.roomBId
              << " status=" << static_cast<int>(result.status)
              << " rejectReason=" << rejectReasonName(result.rejectReason)
              << " wallCountA=" << wallsA.size()
              << " wallCountB=" << wallsB.size()
              << " topInlierCount=" << result.topInlierCount
              << " runnerUpInlierCount=" << result.runnerUpInlierCount
              << " inlierRatio=" << result.inlierRatio
              << " floorGateRan=" << result.hasFloorGateRun
              << " floorGateResult=\""
              << (result.floorGateResult.empty() ? "-" : result.floorGateResult)
              << "\"" << std::endl;

    /* Verification only: no Atlas mutation here. This makes RoomTracker's
     * VerificationVerdict input real; the shared merge trigger is
     * a separate, deliberately gated step (see the comment above this
     * method's call site in Run()). */
    semantic::VerificationVerdict resultVerificationVerdict{};
    if (result.toVerificationVerdict(resultVerificationVerdict) !=
        semantic::SemanticVerifyResultStatus::
            SEMANTIC_VERIFY_RESULT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: toVerificationVerdict returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (submitVerificationVerdict(resultVerificationVerdict) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: submitVerificationVerdict returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
