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
 * @file            findRecordByKey.tpp
 *
 * @brief           Implements findRecordByKey(), declared in
 *                  private_functions.h.
 */

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

template <typename RecordT>
const RecordT *findRecordByKey(const std::vector<RecordT> &records_in,
                               const EntityKey            &key_in)
{
    /* records_in is sorted ascending by RecordT::key (MapSnapshot's own
     * documented invariant), so std::lower_bound with a key-only comparator
     * finds the first candidate in logarithmic time; a genuine EntityKey
     * collision (multiple distinct source objects sharing one key, an
     * anomaly this snapshot deliberately retains rather than erasing) is
     * resolved by simply returning that first candidate, which is
     * deterministic given records_in's own fixed sort order. */
    const typename std::vector<RecordT>::const_iterator foundIt =
        std::lower_bound(records_in.begin(),
                         records_in.end(),
                         key_in,
                         [](const RecordT &record_in, const EntityKey &key_in)
                         { return record_in.key < key_in; });
    if (foundIt == records_in.end() || foundIt->key != key_in)
    {
        return nullptr;
    }
    return &(*foundIt);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
