/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            getSemanticReportCacheEntry.cc
 *
 * @brief           Implements System::getSemanticReportCacheEntry(), declared
 *                  in System.h.
 */

#include "SemanticsManager.h"
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::getSemanticReportCacheEntry(
    semantic::SemanticReportCacheEntry &getSemanticReportCacheEntry_out) const
{
    if (p_semanticsManager == nullptr)
    {
        getSemanticReportCacheEntry_out = semantic::SemanticReportCacheEntry();
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    semantic::SemanticReportCacheEntry
        semanticsManagerGetSemanticReportCacheEntry{};
    if (p_semanticsManager->getSemanticReportCacheEntry(
            semanticsManagerGetSemanticReportCacheEntry) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSemanticReportCacheEntry returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    getSemanticReportCacheEntry_out =
        semanticsManagerGetSemanticReportCacheEntry;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
