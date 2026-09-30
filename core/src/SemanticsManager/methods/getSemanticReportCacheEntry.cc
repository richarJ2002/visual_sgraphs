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
 * @file            getSemanticReportCacheEntry.cc
 *
 * @brief           Implements SemanticsManager::getSemanticReportCacheEntry(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getSemanticReportCacheEntry(
    semantic::SemanticReportCacheEntry &getSemanticReportCacheEntry_out) const
{
    semantic::SemanticReportCacheEntry semanticReportCacheGetLatest{};
    if (semanticReportCache.getLatest(semanticReportCacheGetLatest) !=
        semantic::SemanticReportCacheStatus::
            SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLatest returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    getSemanticReportCacheEntry_out = semanticReportCacheGetLatest;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
