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
 * @file            isAggregateAxiomResultLessTotalOrder.cc
 *
 * @brief           Implements isAggregateAxiomResultLessTotalOrder(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isAggregateAxiomResultLessTotalOrder(const AggregateAxiomResult &lhs_in,
                                          const AggregateAxiomResult &rhs_in)
{
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
    return lhs_in.contributingFindingCount < rhs_in.contributingFindingCount;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
