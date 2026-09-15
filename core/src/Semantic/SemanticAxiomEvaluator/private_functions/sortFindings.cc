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
 * @file            sortFindings.cc
 *
 * @brief           Implements sortFindings(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void sortFindings(std::vector<Finding> &findings_inout)
{
    std::sort(
        findings_inout.begin(),
        findings_inout.end(),
        [](const Finding &lhs_in, const Finding &rhs_in)
        {
            if (lhs_in.id != rhs_in.id)
                return lhs_in.id < rhs_in.id;
            if (lhs_in.axiomCode != rhs_in.axiomCode)
                return lhs_in.axiomCode < rhs_in.axiomCode;
            if (lhs_in.result != rhs_in.result)
                return lhs_in.result < rhs_in.result;
            if (lhs_in.classification != rhs_in.classification)
                return lhs_in.classification < rhs_in.classification;
            if (lhs_in.reasonCode != rhs_in.reasonCode)
                return lhs_in.reasonCode < rhs_in.reasonCode;
            if (lhs_in.involvedKeys != rhs_in.involvedKeys)
                return lhs_in.involvedKeys < rhs_in.involvedKeys;

            /* Every field above is deterministically derived from
             * exactly (axiomCode, reasonCode, involvedKeys) per
             * Finding::id's own contract, so two findings equal in
             * all of them should already carry identical evidence in
             * production. This tiebreak is still required for a
             * total order over the type as declared (evidence is a
             * genuine Finding field), and for two findings that
             * happen to collide on id/axiomCode/result/classification/
             * reasonCode/involvedKeys while differing only in
             * evidence (e.g. two independently constructed test
             * fixtures) to still serialize deterministically instead
             * of depending on std::sort's implementation-defined
             * handling of "equal" elements. */
            const FindingEvidence &lhsEvidence = lhs_in.evidence;
            const FindingEvidence &rhsEvidence = rhs_in.evidence;
            if (lhsEvidence.observedCount.has_value() !=
                rhsEvidence.observedCount.has_value())
                return lhsEvidence.observedCount.has_value();
            if (lhsEvidence.observedCount.has_value() &&
                *lhsEvidence.observedCount != *rhsEvidence.observedCount)
                return *lhsEvidence.observedCount < *rhsEvidence.observedCount;
            if (lhsEvidence.expectedCount.has_value() !=
                rhsEvidence.expectedCount.has_value())
                return lhsEvidence.expectedCount.has_value();
            if (lhsEvidence.expectedCount.has_value() &&
                *lhsEvidence.expectedCount != *rhsEvidence.expectedCount)
                return *lhsEvidence.expectedCount < *rhsEvidence.expectedCount;
            if (lhsEvidence.numericValue.has_value() !=
                rhsEvidence.numericValue.has_value())
                return lhsEvidence.numericValue.has_value();
            if (lhsEvidence.numericValue.has_value())
                return isDoubleLess(*lhsEvidence.numericValue,
                                    *rhsEvidence.numericValue);
            return false;
        });
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
