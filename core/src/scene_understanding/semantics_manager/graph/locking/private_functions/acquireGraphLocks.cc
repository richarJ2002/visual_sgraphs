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
 * @file            acquireGraphLocks.cc
 *
 * @brief           Implements acquireGraphLocks(), declared in
 *                  semantics_manager/graph/locking/private_functions/
 *                  private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/locking/private_functions/private_functions.h"

#include <map>
#include <rclcpp/logging.hpp>
#include <shared_mutex>

namespace vs_graphs
{
namespace core
{

bool acquireGraphLocks(const std::map<unsigned long, scene_graph::SceneGraph *>
                                      &graphsInIdOrder_in,
                       GraphLockMode   mode_in,
                       GraphLockScope &scope_out)
{
    std::map<const scene_graph::SceneGraph *, HeldGraphLock> &heldGraphLocks =
        getHeldGraphLocks();
    const bool wantsExclusive =
        mode_in == GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE;
    for (const std::pair<const unsigned long, scene_graph::SceneGraph *>
             &graphEntry : graphsInIdOrder_in)
    {
        scene_graph::SceneGraph *p_graph = graphEntry.second;
        GraphLockEntry           entry{};
        entry.p_graph = p_graph;
        std::map<const scene_graph::SceneGraph *, HeldGraphLock>::iterator
            heldIterator = heldGraphLocks.find(p_graph);
        if (heldIterator != heldGraphLocks.end())
        {
            if (wantsExclusive && !heldIterator->second.isExclusive)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: exclusive graph lock requested while this "
                             "thread holds the graph shared; it cannot be "
                             "upgraded, continuing under the shared lock.",
                             __func__);
            }
            ++heldIterator->second.depth;
            scope_out.entries.push_back(entry);
            continue;
        }

        if (mode_in == GraphLockMode::GRAPH_LOCK_MODE_TRY_SHARED)
        {
            if (!p_graph->graphMutex.try_lock_shared())
            {
                releaseGraphLockEntries(scope_out.entries);
                return false;
            }
        }
        else if (wantsExclusive)
        {
            p_graph->graphMutex.lock();
        }
        else
        {
            p_graph->graphMutex.lock_shared();
        }
        entry.isOwner           = true;
        entry.isExclusive       = wantsExclusive;
        heldGraphLocks[p_graph] = HeldGraphLock{wantsExclusive, 1U};
        scope_out.entries.push_back(entry);
    }
    return true;
}

} /* namespace core */
} /* namespace vs_graphs */
