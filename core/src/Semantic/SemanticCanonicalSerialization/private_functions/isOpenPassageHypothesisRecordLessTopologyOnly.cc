/**
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
 * @file            isOpenPassageHypothesisRecordLessTopologyOnly.cc
 *
 * @brief           Implements isOpenPassageHypothesisRecordLessTopologyOnly(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

bool isOpenPassageHypothesisRecordLessTopologyOnly(
    const OpenPassageHypothesisRecord &lhs_in,
    const OpenPassageHypothesisRecord &rhs_in)
{
    if (isRawPlaneRefLess(lhs_in.supportingWallRef, rhs_in.supportingWallRef))
    {
        return true;
    }
    if (isRawPlaneRefLess(rhs_in.supportingWallRef, lhs_in.supportingWallRef))
    {
        return false;
    }

    if (lhs_in.confirmationCount != rhs_in.confirmationCount)
    {
        return lhs_in.confirmationCount < rhs_in.confirmationCount;
    }
    if (lhs_in.missedUpdateCount != rhs_in.missedUpdateCount)
    {
        return lhs_in.missedUpdateCount < rhs_in.missedUpdateCount;
    }
    if (lhs_in.lastConfirmedSkeletonFingerprint !=
        rhs_in.lastConfirmedSkeletonFingerprint)
    {
        return lhs_in.lastConfirmedSkeletonFingerprint <
               rhs_in.lastConfirmedSkeletonFingerprint;
    }

    return false;
}

} // namespace semantic
} // namespace ORB_SLAM3
