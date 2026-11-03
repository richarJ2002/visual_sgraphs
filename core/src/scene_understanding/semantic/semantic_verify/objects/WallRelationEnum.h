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
 * @file            WallRelationEnum.h
 *
 * @brief           Declares WallRelation, how two walls relate.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_WALL_RELATION_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_WALL_RELATION_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Relation between an absorbed wall (already in the surviving
 *                  frame) and a surviving wall.
 */
enum class WallRelation
{
    /*! The walls face different ways or lie apart along the wall: no answer. */
    UNRELATED,
    /*! Same facing, coplanar and touching along the wall: the same wall. */
    PAIRED,
    /*! Same facing and touching along the wall but at different offsets. */
    OFFSET_MISMATCH
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_WALL_RELATION_ENUM_H \
        */
