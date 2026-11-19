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
 * @file            takeKeyFrameErasureRequests.cc
 *
 * @brief           Implements SemanticsManager::takeKeyFrameErasureRequests(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <mutex>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::takeKeyFrameErasureRequests(
    std::vector<unsigned long> &keyFrameIds_out)
{
    std::unique_lock<std::mutex> lock(keyFrameErasureMutex);
    keyFrameIds_out.clear();
    keyFrameIds_out.swap(keyFrameErasureRequests);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
