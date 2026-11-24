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
 * @file            verifyRejectReasonName.cc
 *
 * @brief           Implements SemanticVerify::verifyRejectReasonName(),
 *                  declared in semantic_verify/objects/SemanticVerifyClass.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus
    SemanticVerify::verifyRejectReasonName(const VerifyRejectReason reason_in,
                                           const char             *&p_name_out)
{
    switch (reason_in)
    {
    case VerifyRejectReason::NONE:
        p_name_out = "NONE";
        break;
    case VerifyRejectReason::TOO_FEW_WALLS:
        p_name_out = "TOO_FEW_WALLS";
        break;
    case VerifyRejectReason::NO_VALID_HYPOTHESIS:
        p_name_out = "NO_VALID_HYPOTHESIS";
        break;
    case VerifyRejectReason::AMBIGUOUS_TOP_HYPOTHESES:
        p_name_out = "AMBIGUOUS_TOP_HYPOTHESES";
        break;
    case VerifyRejectReason::BELOW_MIN_INLIER_RATIO:
        p_name_out = "BELOW_MIN_INLIER_RATIO";
        break;
    case VerifyRejectReason::REFINED_FIT_NOT_OBSERVABLE:
        p_name_out = "REFINED_FIT_NOT_OBSERVABLE";
        break;
    default:
        p_name_out = "unknown";
        break;
    }
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
