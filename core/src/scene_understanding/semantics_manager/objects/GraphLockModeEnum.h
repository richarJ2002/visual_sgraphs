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

/*!
 * @file            GraphLockModeEnum.h
 *
 * @brief           Declares GraphLockMode, how a scene graph mutex is taken.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_GRAPH_LOCK_MODE_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_GRAPH_LOCK_MODE_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           How SemanticsManager::lockGraphs() takes the per-map graph
 *                  mutexes. Values are fixed and never reordered.
 */
enum class GraphLockMode : std::uint8_t
{
    /*!
     * @brief           Reading: waits for a writer, other readers may share.
     */
    GRAPH_LOCK_MODE_SHARED = 0U,

    /*!
     * @brief           Editing: waits for every reader and writer.
     */
    GRAPH_LOCK_MODE_EXCLUSIVE = 1U,

    /*!
     * @brief           Reading without waiting: fails with
     *                  SEMANTICS_MANAGER_STATUS_BUSY when a writer holds the
     *                  graph. For threads that must never stall.
     */
    GRAPH_LOCK_MODE_TRY_SHARED = 2U
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_GRAPH_LOCK_MODE_ENUM_H  \
        */
