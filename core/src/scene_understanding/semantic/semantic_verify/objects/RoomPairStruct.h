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
 * @file            RoomPairStruct.h
 *
 * @brief           Declares RoomPair, two rooms paired through their walls.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_ROOM_PAIR_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_ROOM_PAIR_STRUCT_H

#include "scene_understanding/semantic/semantic_verify/objects/MergePairStruct.h"

#include <cstddef>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           An absorbed room and a surviving room that share most of
 *                  their walls under the merge transform.
 */
struct RoomPair
{
  public:
    /*!
     * @brief           Index of the room in the absorbed map's room list.
     */
    std::size_t            absorbedIndex{0U};
    /*!
     * @brief           Index of the room in the surviving map's room list.
     */
    std::size_t            survivingIndex{0U};
    /*!
     * @brief           Wall pairs between the two rooms; indices refer to each
     *                  room's wall list.
     */
    std::vector<MergePair> wallPairs;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_ROOM_PAIR_STRUCT_H \
        */
