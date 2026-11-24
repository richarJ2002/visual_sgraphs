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
 * @file            getWallSlabQueries.cc
 *
 * @brief           Implements System::getWallSlabQueries(), declared in
 *                  SystemClass.h.
 */

#include "execution/system/objects/SystemClass.h"

#include <vector>

#include <rclcpp/logging.hpp>

#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{
namespace system
{

SystemStatus System::getWallSlabQueries(
    std::vector<WallSlabQuery> &wallSlabQueries_out) const
{
    wallSlabQueries_out.clear();
    if (p_semanticsManager == nullptr)
    {
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    if (p_semanticsManager->getWallSlabQueries(wallSlabQueries_out) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWallSlabQueries returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} /* namespace system */
} /* namespace core */
} /* namespace vs_graphs */
