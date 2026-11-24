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
 * @file            getActiveSceneGraphSnapshot.cc
 *
 * @brief           Implements SemanticsManager::getActiveSceneGraphSnapshot(),
 *                  declared in semantics_manager/objects/
 *                  SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <optional>
#include <shared_mutex>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getActiveSceneGraphSnapshot(
    SceneGraphSnapshot &snapshot_out) const
{
    snapshot_out = SceneGraphSnapshot{};
    std::optional<unsigned long> activeMapIdCopy{};
    {
        std::shared_lock<std::shared_mutex> registryLock(registryMutex);
        activeMapIdCopy = activeGraphMapId;
    }
    /* The registry lock is released before the by-map query locks the graph:
     * the registry mutex is never held while waiting for a graph. */
    if (!activeMapIdCopy.has_value())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    return getSceneGraphSnapshot(*activeMapIdCopy, snapshot_out);
}

} /* namespace core */
} /* namespace vs_graphs */
