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
 * @file            setSkeletonCluster.cc
 *
 * @brief           Implements System::setSkeletonCluster(), declared in
 *                  System.h.
 */

#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus
    System::setSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d>>
                                   &skeletonClusterPoints_world_m_in)
{
    /* Keep asynchronous skeleton replacement atomic with map remerging. */
    std::unique_lock<std::mutex> semanticUpdateLock{};
    if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: acquireSemanticUpdateLock returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Add the skeleton cluster to the current semantic map. */
    if (p_atlas->setSkeletonClusterPoints(skeletonClusterPoints_world_m_in) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
