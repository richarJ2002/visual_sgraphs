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
 * @file            private_functions.h
 *
 * @brief           Declares the helpers of the SemanticsManager graph locking
 *                  methods. Package-internal: not included by any public
 *                  header.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_GRAPH_LOCKING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_SEMANTICS_MANAGER_GRAPH_LOCKING_PRIVATE_FUNCTIONS_H

#include <map>

#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphStruct.h"
#include "scene_understanding/semantics_manager/objects/GraphLockModeEnum.h"
#include "scene_understanding/semantics_manager/objects/GraphLockScopeStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What the calling thread holds of one scene graph.
 */
struct HeldGraphLock
{
  public:
    /*!
     * @brief           True when the thread holds the graph exclusive.
     */
    bool isExclusive{false};

    /*!
     * @brief           Number of live GraphLockEntry objects of this thread
     *                  for the graph, the owning one included.
     */
    unsigned int depth{0U};
};

/*!
 * @brief           Gives the graphs the calling thread currently holds, so a
 *                  nested manager call re-enters instead of deadlocking.
 *
 * @return          The calling thread's own table; never shared with another
 *                  thread.
 */
[[nodiscard]] std::map<const scene_graph::SceneGraph *, HeldGraphLock> &
    getHeldGraphLocks();

/*!
 * @brief           Locks scene graphs in ascending map-id order (the order of
 *                  the map). A graph the thread already holds is re-entered
 *                  without waiting; asking for exclusive while only holding it
 *                  shared cannot be granted, is logged, and the call proceeds
 *                  under the shared lock.
 *
 * @param[in]       graphsInIdOrder_in
 *                  Graphs to lock; borrowed, not null.
 *
 * @param[in]       mode_in
 *                  How to lock them.
 *
 * @param[in,out]   scope_out
 *                  Receives the entries taken; unlocks on destruction.
 *
 * @return          True when every graph is held; false only for
 *                  GRAPH_LOCK_MODE_TRY_SHARED when a writer holds one, in which
 *                  case nothing stays locked.
 */
[[nodiscard]] bool
    acquireGraphLocks(const std::map<unsigned long, scene_graph::SceneGraph *>
                                     &graphsInIdOrder_in,
                      GraphLockMode   mode_in,
                      GraphLockScope &scope_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_GRAPH_LOCKING_PRIVATE_FUNCTIONS_H */
