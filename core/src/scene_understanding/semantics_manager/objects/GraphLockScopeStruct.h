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
 * @file            GraphLockScopeStruct.h
 *
 * @brief           Declares GraphLockScope, the RAII holder of per-map scene
 *                  graph locks.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_GRAPH_LOCK_SCOPE_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_GRAPH_LOCK_SCOPE_STRUCT_H

#include <vector>

#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           One graph lock a GraphLockScope was given.
 */
struct GraphLockEntry
{
  public:
    /*!
     * @brief           Graph the entry locks; borrowed, owned by its Map.
     */
    scene_graph::SceneGraph *p_graph{nullptr};

    /*!
     * @brief           True when this entry took the mutex itself; false when
     *                  the calling thread already held the graph (a nested
     *                  scope) and the entry only counts the nesting.
     */
    bool isOwner{false};

    /*!
     * @brief           True when the mutex was taken exclusive.
     */
    bool isExclusive{false};
};

/*!
 * @brief           Undoes the entries of a GraphLockScope, last taken first:
 *                  unlocks the graph mutexes this thread took and ends the
 *                  nesting count of the others.
 *
 * @param[in,out]   entries_inout
 *                  Entries to release; emptied.
 */
void releaseGraphLockEntries(std::vector<GraphLockEntry> &entries_inout);

/*!
 * @brief           Holds the mutexes of one or more scene graphs for as long
 *                  as it lives. Obtained from SemanticsManager::lockGraphs()
 *                  or lockAllGraphs(). Re-entrant per thread: a thread that
 *                  already holds a graph (shared or exclusive) can take it
 *                  again through any manager call, which then waits for
 *                  nothing; the outermost scope owns the real lock. Not
 *                  copyable; movable. Thread safety: owned by one thread.
 */
struct GraphLockScope
{
  public:
    /*!
     * @brief           Locks held, in the ascending map-id order they were
     *                  taken.
     */
    std::vector<GraphLockEntry> entries;

    GraphLockScope() = default;

    GraphLockScope(const GraphLockScope &) = delete;

    GraphLockScope &operator=(const GraphLockScope &) = delete;

    GraphLockScope(GraphLockScope &&) noexcept = default;

    GraphLockScope &operator=(GraphLockScope &&) = delete;

    ~GraphLockScope()
    {
        releaseGraphLockEntries(entries);
    }
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_GRAPH_LOCK_SCOPE_STRUCT_H \
        */
