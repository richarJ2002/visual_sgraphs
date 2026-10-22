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
 * @file            VerificationVerdictStruct.h
 *
 * @brief           Declares VerificationVerdict, the result of a geometric
 *                  room-match verification.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_VERIFICATION_VERDICT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_VERIFICATION_VERDICT_STRUCT_H

#include "scene_understanding/semantic/room_tracker/objects/VerificationStatusEnum.h"
#include "scene_understanding/semantic/room_tracker/objects/VerificationVerdictStatusEnum.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Result of a geometric room-match verification: whether it
 *                  passed and the numbers behind that verdict.
 */
struct VerificationVerdict
{
    /*!
     * @brief           Whether a verifier produced a result at all, and if so
     *                  PASS or REJECTED.
     */
    VerificationStatus status = VerificationStatus::UNAVAILABLE;
    /*!
     * @brief           Verifier's own pass flag; isPass() also needs status ==
     *                  PASS and every metric in range.
     */
    bool               hasPassed = false;
    /*!
     * @brief           Number of inliers the verifier counted.
     */
    unsigned int       inlierCount = 0U;
    /*!
     * @brief           Fraction of inliers among all matches, in [0, 1].
     */
    double             inlierRatio = 0.0;
    /*!
     * @brief           Condition number of the verification normalised to
     *                  [0, 1]; confidence falls as it rises.
     */
    double             normalisedConditionNumber = 0.0;
    /*!
     * @brief           Angular residual of the verification, radians, at least
     *                  0.
     */
    double             angularResidual_rad = 0.0;
    /*!
     * @brief           Overall confidence of the verdict, in [0, 1].
     */
    double             confidence = 0.0;

    /*!
     * @brief           Returns true only for a finite, internally consistent
     *                  PASS.
     */
    [[nodiscard]] VerificationVerdictStatus isPass(bool &isPass_out) const
    {
        isPass_out =
            status == VerificationStatus::PASS && hasPassed &&
            std::isfinite(inlierRatio) && inlierRatio >= 0.0 &&
            inlierRatio <= 1.0 && std::isfinite(normalisedConditionNumber) &&
            normalisedConditionNumber >= 0.0 &&
            normalisedConditionNumber <= 1.0 &&
            std::isfinite(angularResidual_rad) && angularResidual_rad >= 0.0 &&
            std::isfinite(confidence) && confidence >= 0.0 && confidence <= 1.0;
        return VerificationVerdictStatus::VERIFICATION_VERDICT_STATUS_SUCCESS;
    }
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_VERIFICATION_VERDICT_STRUCT_H \
        */
