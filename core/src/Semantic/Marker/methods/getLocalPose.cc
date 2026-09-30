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
 * @file            getLocalPose.cc
 *
 * @brief           Implements Marker::getLocalPose(), declared in
 *                  Semantic/Marker.h.
 */

#include "Semantic/Marker.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

MarkerStatus Marker::getLocalPose(Sophus::SE3f &localPose_out) const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    localPose_out = localPose;
    return MarkerStatus::MARKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
