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
 * @file            sortByKey.tpp
 *
 * @brief           Implements sortByKey(), declared in private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/SemanticGraphSnapshotStatus.h"
#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

template <typename RecordT>
[[nodiscard]] SemanticGraphSnapshotStatus
    sortByKey(std::vector<RecordT> &records_inout)
{
    /* Primary order is always by key. For the anomalous case this snapshot
     * deliberately never erases -- two distinct source objects captured
     * with an EntityKey collision (same kind, mapId, and local id) -- the
     * tiebreak is isValueLessForCollisionTiebreak(lhs_in, rhs_in), resolved
     * by argument-dependent lookup to the RecordT-specific overload
     * declared in private_functions.h. That overload compares every
     * remaining captured value field in a fixed, documented order using
     * total-order-safe primitives (isDoubleLess()/isVector3dLess()/
     * isVector4dLess()), so two colliding records with different values
     * always sort the same way regardless of which pointer/allocation
     * order produced records_inout -- unlike a plain std::stable_sort over
     * `key` alone, which only preserves whatever pre-sort order the
     * (possibly pointer-ordered) source container happened to produce.
     * Two records equal in every field are, by construction, not
     * distinguished by this comparator (it returns false for both
     * orderings), which is the correct behaviour for a strict weak
     * ordering: they require no artificial distinction. std::sort is used,
     * not std::stable_sort, because this comparator is now a complete total
     * order over every field (never returns "equal but arbitrary" for two
     * differently-valued records), so no further tie exists for
     * "stability" to preserve. */
    std::sort(records_inout.begin(),
              records_inout.end(),
              [](const RecordT &lhs_in, const RecordT &rhs_in)
              {
                  if (lhs_in.key != rhs_in.key)
                  {
                      return lhs_in.key < rhs_in.key;
                  }
                  return isValueLessForCollisionTiebreak(lhs_in, rhs_in);
              });

    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
