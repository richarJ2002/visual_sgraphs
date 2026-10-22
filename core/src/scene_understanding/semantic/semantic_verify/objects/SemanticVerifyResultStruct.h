/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            SemanticVerifyResultStruct.h
 *
 * @brief           Declares SemanticVerifyResult, the outcome of verify().
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_RESULT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_RESULT_STRUCT_H

#include "scene_understanding/semantic/room_tracker/objects/VerificationStatusEnum.h"
#include "scene_understanding/semantic/room_tracker/objects/VerificationVerdictStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyResultStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/VerifyRejectReasonEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/WallInlierPairStruct.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Outcome of verify(): whether two rooms' wall sets are the
 *                  same place, the transform found and diagnostics of the
 *                  search.
 */
struct SemanticVerifyResult
{
    /*!
     * @brief           Verdict: PASS, REJECTED, or UNAVAILABLE when verify()
     *                  had no verdict.
     */
    VerificationStatus status{VerificationStatus::UNAVAILABLE};
    /*!
     * @brief           True only when status is PASS.
     */
    bool               hasPassed{false};

    /*!
     * @brief           Maps room-A-frame points into room B's frame: x_B = R
     *                  x_A + t.
     */
    Eigen::Isometry3d roomTransform_roomAToRoomB{Eigen::Isometry3d::Identity()};

    /*!
     * @brief           Wall pairs that agree under the best hypothesis; empty
     *                  unless the status is PASS.
     */
    std::vector<WallInlierPair> inliers;
    /*!
     * @brief           Number of wall pairs that could be formed: wall count of
     *                  room A times wall count of room B.
     */
    std::size_t                 candidateWallPairCount{0U};
    /*!
     * @brief           Rank of the translation system of the refined fit (3 =
     *                  fully determined); 0 when it was not computed.
     */
    std::size_t                 rank{0U};
    /*!
     * @brief           Condition number of the translation system of the
     *                  refined fit; 0 when it was not computed.
     */
    double                      conditionNumber{0.0};
    /*!
     * @brief           conditionNumber divided by maxConditionNumber, capped at
     *                  1 (1 when the condition number is not finite); set only
     *                  on PASS.
     */
    double                      normalisedConditionNumber{0.0};
    /*!
     * @brief           Inliers of the best hypothesis divided by the wall count
     *                  of the room with fewer walls; 0 before hypotheses were
     *                  ranked.
     */
    double                      inlierRatio{0.0};
    /*!
     * @brief           Median normal angle residual over the inliers, radians;
     *                  set only on PASS.
     */
    double                      angularResidual_rad{0.0};
    /*!
     * @brief           Score in [0, 1]: inlierRatio reduced by up to half
     *                  through normalisedConditionNumber; set only on PASS.
     */
    double                      confidence{0.0};

    /*!
     * @brief           Diagnostic-only fields, populated on every exit path
     *                  (see VerifyRejectReason). NONE on PASS.
     */
    VerifyRejectReason rejectReason{VerifyRejectReason::NONE};
    /*!
     * @brief           Best distinct hypothesis's inlier count, set from
     *                  AMBIGUOUS_TOP_HYPOTHESES onward (i.e. once at least one
     *                  valid hypothesis existed to rank). 0 for
     *                  TOO_FEW_WALLS/NO_VALID_HYPOTHESIS, which never reach
     *                  hypothesis ranking.
     */
    std::size_t        topInlierCount{0U};
    /*!
     * @brief           Second-best distinct hypothesis's inlier count, same
     *                  availability as topInlierCount.
     */
    std::size_t        runnerUpInlierCount{0U};

    /*!
     * @brief           True once runFloorGate() has been called on this result.
     */
    bool        hasFloorGateRun{false};
    /*!
     * @brief           True when the floor gate accepted the floors; meaningful
     *                  only when hasFloorGateRun is true.
     */
    bool        hasFloorGatePassed{false};
    /*!
     * @brief           Floor gate outcome text: "ACCEPTED", "REJECTED",
     *                  "DEFERRED" or empty when the gate has not run.
     */
    std::string floorGateResult;

    /*!
     * @brief           Converts to the RoomTracker verdict type this
     *                  verifier exists to eventually feed
     *                  (SemanticsManager::submitVerificationVerdict()'s
     *                  consumer). Wiring that call site is out of scope
     *                  here.
     */
    [[nodiscard]] SemanticVerifyResultStatus toVerificationVerdict(
        VerificationVerdict &verificationVerdict_out) const;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_RESULT_STRUCT_H \
        */
