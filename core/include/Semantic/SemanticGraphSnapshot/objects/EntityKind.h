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
 * @file            EntityKind.h
 *
 * @brief           Declares the discriminator for which kind of graph entity
 *                  a snapshot EntityKey names.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_ENTITY_KIND_H
#define SEMANTIC_GRAPH_SNAPSHOT_ENTITY_KIND_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Discriminates which kind of graph entity an EntityKey names.
 *
 *              A "wall" has no dedicated model class in this codebase: it is
 *              a Geometric/Plane.h object whose accepted Plane::planeVariant
 *              is WALL. DOOR/GROUND/WINDOW/UNDEFINED planes are out of scope
 *              for this foundation slice and are never represented as
 *              EntityKind::WALL records (see RawPlaneRef.h for how a
 *              reference to one of those is still reported, without a full
 *              record).
 */
enum class EntityKind : std::uint8_t
{
    /*! @brief A committed or candidate Room. */
    ROOM = 0U,

    /*! @brief A WALL-typed Geometric/Plane.h wall face. */
    WALL = 1U,

    /*! @brief A Passage. */
    PASSAGE = 2U,

    /*! @brief A Floor. */
    FLOOR = 3U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_ENTITY_KIND_H
