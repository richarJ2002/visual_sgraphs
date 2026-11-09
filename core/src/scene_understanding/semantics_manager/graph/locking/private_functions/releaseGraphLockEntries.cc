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
 * @file            releaseGraphLockEntries.cc
 *
 * @brief           Implements releaseGraphLockEntries(), declared in
 *                  semantics_manager/objects/GraphLockScopeStruct.h.
 */

#include "scene_understanding/semantics_manager/objects/GraphLockScopeStruct.h"

#include <map>
#include <shared_mutex>

#include "scene_understanding/semantics_manager/graph/locking/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

void releaseGraphLockEntries(std::vector<GraphLockEntry> &entries_inout)
{
    std::map<const scene_graph::SceneGraph *, HeldGraphLock> &heldGraphLocks =
        getHeldGraphLocks();
    for (std::vector<GraphLockEntry>::reverse_iterator entryIterator =
             entries_inout.rbegin();
         entryIterator != entries_inout.rend();
         ++entryIterator)
    {
        std::map<const scene_graph::SceneGraph *, HeldGraphLock>::iterator
            heldIterator = heldGraphLocks.find(entryIterator->p_graph);
        if (heldIterator != heldGraphLocks.end())
        {
            --heldIterator->second.depth;
            if (heldIterator->second.depth == 0U)
            {
                heldGraphLocks.erase(heldIterator);
            }
        }
        if (entryIterator->isOwner)
        {
            if (entryIterator->isExclusive)
            {
                entryIterator->p_graph->graphMutex.unlock();
            }
            else
            {
                entryIterator->p_graph->graphMutex.unlock_shared();
            }
        }
    }
    entries_inout.clear();
}

} /* namespace core */
} /* namespace vs_graphs */
