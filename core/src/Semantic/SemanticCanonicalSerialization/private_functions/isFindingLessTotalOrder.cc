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
 * @file            isFindingLessTotalOrder.cc
 *
 * @brief           Implements isFindingLessTotalOrder(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isFindingLessTotalOrder(const Finding &lhs_in, const Finding &rhs_in)
{
    if (lhs_in.id != rhs_in.id)
    {
        return lhs_in.id < rhs_in.id;
    }
    if (lhs_in.axiomCode != rhs_in.axiomCode)
    {
        return lhs_in.axiomCode < rhs_in.axiomCode;
    }
    if (lhs_in.result != rhs_in.result)
    {
        return lhs_in.result < rhs_in.result;
    }
    if (lhs_in.classification != rhs_in.classification)
    {
        return lhs_in.classification < rhs_in.classification;
    }
    if (lhs_in.reasonCode != rhs_in.reasonCode)
    {
        return lhs_in.reasonCode < rhs_in.reasonCode;
    }
    if (lhs_in.involvedKeys != rhs_in.involvedKeys)
    {
        return lhs_in.involvedKeys < rhs_in.involvedKeys;
    }

    const FindingEvidence &lhsEvidence = lhs_in.evidence;
    const FindingEvidence &rhsEvidence = rhs_in.evidence;
    if (lhsEvidence.observedCount.has_value() !=
        rhsEvidence.observedCount.has_value())
    {
        return lhsEvidence.observedCount.has_value();
    }
    if (lhsEvidence.observedCount.has_value() &&
        *lhsEvidence.observedCount != *rhsEvidence.observedCount)
    {
        return *lhsEvidence.observedCount < *rhsEvidence.observedCount;
    }
    if (lhsEvidence.expectedCount.has_value() !=
        rhsEvidence.expectedCount.has_value())
    {
        return lhsEvidence.expectedCount.has_value();
    }
    if (lhsEvidence.expectedCount.has_value() &&
        *lhsEvidence.expectedCount != *rhsEvidence.expectedCount)
    {
        return *lhsEvidence.expectedCount < *rhsEvidence.expectedCount;
    }
    if (lhsEvidence.numericValue.has_value() !=
        rhsEvidence.numericValue.has_value())
    {
        return lhsEvidence.numericValue.has_value();
    }
    if (lhsEvidence.numericValue.has_value())
    {
        return isDoubleLess(*lhsEvidence.numericValue,
                            *rhsEvidence.numericValue);
    }

    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
