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
 * @file            getFloorById.cc
 *
 * @brief           Implements SemanticsManager::getFloorById(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <mutex>
#include <optional>
#include <shared_mutex>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::getFloorById(int               floorId_in,
                                   semantic::Floor *&p_floor_out) const
{
    p_floor_out = nullptr;
    std::optional<unsigned long> activeMapIdCopy{};
    {
        std::shared_lock<std::shared_mutex> graphLock(registryMutex);
        activeMapIdCopy = activeGraphMapId;
    }
    /* The lock is released before the by-map query takes it again: one
     * thread must not hold a shared mutex twice. */
    if (!activeMapIdCopy.has_value())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    return getMapFloorById(*activeMapIdCopy, floorId_in, p_floor_out);
}

} /* namespace core */
} /* namespace vs_graphs */
