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
 * @file            isMapCompletenessResultLessTotalOrder.cc
 *
 * @brief           Implements isMapCompletenessResultLessTotalOrder(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isMapCompletenessResultLessTotalOrder(const MapCompletenessResult &lhs_in,
                                           const MapCompletenessResult &rhs_in)
{
    if (lhs_in.mapId != rhs_in.mapId)
    {
        return lhs_in.mapId < rhs_in.mapId;
    }
    if (lhs_in.conservativeResult != rhs_in.conservativeResult)
    {
        return lhs_in.conservativeResult < rhs_in.conservativeResult;
    }
    if (lhs_in.isComplete != rhs_in.isComplete)
    {
        return static_cast<int>(lhs_in.isComplete) <
               static_cast<int>(rhs_in.isComplete);
    }

    std::vector<ReasonCode> lhsReasons = lhs_in.reasons;
    std::vector<ReasonCode> rhsReasons = rhs_in.reasons;
    std::sort(lhsReasons.begin(), lhsReasons.end());
    std::sort(rhsReasons.begin(), rhsReasons.end());
    if (lhsReasons.size() != rhsReasons.size())
    {
        return lhsReasons.size() < rhsReasons.size();
    }
    for (std::size_t lhsReasonIndex = 0U; lhsReasonIndex < lhsReasons.size();
         ++lhsReasonIndex)
    {
        if (lhsReasons[lhsReasonIndex] != rhsReasons[lhsReasonIndex])
        {
            return lhsReasons[lhsReasonIndex] < rhsReasons[lhsReasonIndex];
        }
    }

    std::vector<EntityKey> lhsKeys = lhs_in.relevantEntityKeys;
    std::vector<EntityKey> rhsKeys = rhs_in.relevantEntityKeys;
    std::sort(lhsKeys.begin(), lhsKeys.end());
    std::sort(rhsKeys.begin(), rhsKeys.end());
    if (lhsKeys.size() != rhsKeys.size())
    {
        return lhsKeys.size() < rhsKeys.size();
    }
    for (std::size_t lhsReasonIndex = 0U; lhsReasonIndex < lhsKeys.size();
         ++lhsReasonIndex)
    {
        if (lhsKeys[lhsReasonIndex] != rhsKeys[lhsReasonIndex])
        {
            return lhsKeys[lhsReasonIndex] < rhsKeys[lhsReasonIndex];
        }
    }

    if (lhs_in.confirmedRoomCount != rhs_in.confirmedRoomCount)
    {
        return lhs_in.confirmedRoomCount < rhs_in.confirmedRoomCount;
    }
    if (lhs_in.completeRoomCount != rhs_in.completeRoomCount)
    {
        return lhs_in.completeRoomCount < rhs_in.completeRoomCount;
    }
    if (lhs_in.prospectiveRoomCount != rhs_in.prospectiveRoomCount)
    {
        return lhs_in.prospectiveRoomCount < rhs_in.prospectiveRoomCount;
    }
    if (lhs_in.livePassageCount != rhs_in.livePassageCount)
    {
        return lhs_in.livePassageCount < rhs_in.livePassageCount;
    }
    if (lhs_in.fullyValidPassageCount != rhs_in.fullyValidPassageCount)
    {
        return lhs_in.fullyValidPassageCount < rhs_in.fullyValidPassageCount;
    }

    const LegacyMapCompletenessResult &lhsLegacy = lhs_in.legacy;
    const LegacyMapCompletenessResult &rhsLegacy = rhs_in.legacy;
    if (lhsLegacy.confirmedRoomCount != rhsLegacy.confirmedRoomCount)
    {
        return lhsLegacy.confirmedRoomCount < rhsLegacy.confirmedRoomCount;
    }
    if (lhsLegacy.completeRoomCount != rhsLegacy.completeRoomCount)
    {
        return lhsLegacy.completeRoomCount < rhsLegacy.completeRoomCount;
    }

    std::vector<int> lhsIncomplete = lhsLegacy.incompleteRoomIds;
    std::vector<int> rhsIncomplete = rhsLegacy.incompleteRoomIds;
    std::sort(lhsIncomplete.begin(), lhsIncomplete.end());
    std::sort(rhsIncomplete.begin(), rhsIncomplete.end());
    if (lhsIncomplete != rhsIncomplete)
    {
        return lhsIncomplete < rhsIncomplete;
    }

    if (lhsLegacy.passageCount != rhsLegacy.passageCount)
    {
        return lhsLegacy.passageCount < rhsLegacy.passageCount;
    }
    if (lhsLegacy.fullyLinkedPassageCount != rhsLegacy.fullyLinkedPassageCount)
    {
        return lhsLegacy.fullyLinkedPassageCount <
               rhsLegacy.fullyLinkedPassageCount;
    }

    std::vector<int> lhsDangling = lhsLegacy.danglingPassageIds;
    std::vector<int> rhsDangling = rhsLegacy.danglingPassageIds;
    std::sort(lhsDangling.begin(), lhsDangling.end());
    std::sort(rhsDangling.begin(), rhsDangling.end());
    if (lhsDangling != rhsDangling)
    {
        return lhsDangling < rhsDangling;
    }

    if (lhsLegacy.isMapFullyModeled != rhsLegacy.isMapFullyModeled)
    {
        return static_cast<int>(lhsLegacy.isMapFullyModeled) <
               static_cast<int>(rhsLegacy.isMapFullyModeled);
    }

    return static_cast<int>(lhs_in.doLegacyAndConservativeDiverge) <
           static_cast<int>(rhs_in.doLegacyAndConservativeDiverge);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
