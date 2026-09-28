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
 * @file            isMapSnapshotLessTotalOrder.cc
 *
 * @brief           Implements isMapSnapshotLessTotalOrder(), declared in
 *                  private_functions.h.
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

namespace
{
template <typename RecordT, typename CompareT>
bool isRecordVectorLess(std::vector<RecordT> lhs_in,
                        std::vector<RecordT> rhs_in,
                        CompareT             isLess_in)
{
    std::sort(lhs_in.begin(), lhs_in.end(), isLess_in);
    std::sort(rhs_in.begin(), rhs_in.end(), isLess_in);
    if (lhs_in.size() != rhs_in.size())
    {
        return lhs_in.size() < rhs_in.size();
    }
    for (std::size_t lhIndex = 0U; lhIndex < lhs_in.size(); ++lhIndex)
    {
        if (isLess_in(lhs_in[lhIndex], rhs_in[lhIndex]))
        {
            return true;
        }
        if (isLess_in(rhs_in[lhIndex], lhs_in[lhIndex]))
        {
            return false;
        }
    }
    return false;
}

template <typename RecordT, typename CompareT>
bool isRecordVectorEqual(std::vector<RecordT> lhs_in,
                         std::vector<RecordT> rhs_in,
                         CompareT             isLess_in)
{
    return !isRecordVectorLess(lhs_in, rhs_in, isLess_in) &&
           !isRecordVectorLess(rhs_in, lhs_in, isLess_in);
}
} // namespace

bool isMapSnapshotLessTotalOrder(const MapSnapshot &lhs_in,
                                 const MapSnapshot &rhs_in,
                                 bool               includeGeometry_in)
{
    if (lhs_in.mapId != rhs_in.mapId)
    {
        return lhs_in.mapId < rhs_in.mapId;
    }
    if (lhs_in.isCurrentMap != rhs_in.isCurrentMap)
    {
        return static_cast<int>(lhs_in.isCurrentMap) <
               static_cast<int>(rhs_in.isCurrentMap);
    }

    auto p_roomLess    = includeGeometry_in ? &isRoomRecordLessFullGeometry
                                            : &isRoomRecordLessTopologyOnly;
    auto p_wallLess    = includeGeometry_in ? &isWallRecordLessFullGeometry
                                            : &isWallRecordLessTopologyOnly;
    auto p_passageLess = includeGeometry_in ? &isPassageRecordLessFullGeometry
                                            : &isPassageRecordLessTopologyOnly;
    auto p_floorLess   = includeGeometry_in ? &isFloorRecordLessFullGeometry
                                            : &isFloorRecordLessTopologyOnly;

    if (!isRecordVectorEqual(lhs_in.rooms, rhs_in.rooms, p_roomLess))
    {
        return isRecordVectorLess(lhs_in.rooms, rhs_in.rooms, p_roomLess);
    }
    if (!isRecordVectorEqual(lhs_in.walls, rhs_in.walls, p_wallLess))
    {
        return isRecordVectorLess(lhs_in.walls, rhs_in.walls, p_wallLess);
    }
    if (!isRecordVectorEqual(lhs_in.passages, rhs_in.passages, p_passageLess))
    {
        return isRecordVectorLess(lhs_in.passages,
                                  rhs_in.passages,
                                  p_passageLess);
    }
    return isRecordVectorLess(lhs_in.floors, rhs_in.floors, p_floorLess);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
