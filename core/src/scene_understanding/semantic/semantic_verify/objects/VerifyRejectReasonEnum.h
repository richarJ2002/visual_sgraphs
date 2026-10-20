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
 * @file            VerifyRejectReasonEnum.h
 *
 * @brief           Declares VerifyRejectReason, which early-return gate
 * rejected.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_VERIFY_REJECT_REASON_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_VERIFY_REJECT_REASON_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Identifies which of verify()'s distinct early-return gates
 *                  produced a REJECTED result. Diagnostic-only, added because
 *                  every rejection path used to leave SemanticVerifyResult's
 *                  numeric fields (inlierRatio, rank, conditionNumber, ...) at
 *                  their zero defaults -- indistinguishable in a log line from
 *                  "genuinely computed as zero". See the fields' own comments
 *                  below for which reason populates which fields.
 */
enum class VerifyRejectReason
{
    NONE, /* PASS, or verify() has not returned yet. */
    TOO_FEW_WALLS,
    NO_VALID_HYPOTHESIS,
    AMBIGUOUS_TOP_HYPOTHESES,
    BELOW_MIN_INLIER_RATIO,
    REFINED_FIT_NOT_OBSERVABLE
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_VERIFY_REJECT_REASON_ENUM_H \
        */
