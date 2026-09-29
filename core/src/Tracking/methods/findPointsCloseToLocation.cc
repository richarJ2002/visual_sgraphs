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
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Tracking.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

std::vector<MapPoint *> Tracking::findPointsCloseToLocation(
    const std::vector<MapPoint *> &points_in,
    const Eigen::Vector3f         &location_in,
    double                         distanceThreshold_in)
{
    std::vector<MapPoint *> closePoints;
    for (MapPoint *p_point : points_in)
    {
        double distance{};
        if (utils::utils::Utils::calculateEuclideanDistance(
                p_point->getWorldPos(),
                location_in,
                distance) != utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: calculateEuclideanDistance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (distance <= distanceThreshold_in)
        {
            closePoints.push_back(p_point);
        }
    }

    return closePoints;
}

} // namespace core
} // namespace vs_graphs
