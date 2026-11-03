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
 * @file            MergePairStruct.h
 *
 * @brief           Declares MergePair, one geometric element pair.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_MERGE_PAIR_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_MERGE_PAIR_STRUCT_H

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           One absorbed element paired with one surviving element by
 *                  geometry. Indices refer to the lists the pairing was
 *                  computed from.
 */
struct MergePair
{
  public:
    /*!
     * @brief           Index of the element in the absorbed map's list.
     */
    std::size_t absorbedIndex{0U};
    /*!
     * @brief           Index of the element in the surviving map's list.
     */
    std::size_t survivingIndex{0U};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_MERGE_PAIR_STRUCT_H \
        */
