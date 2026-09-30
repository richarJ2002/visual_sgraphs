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

#include "Geometric/Plane.h"
#include "Geometric/PlaneStatus.h"
#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapStatus
    Map::getBiggestGroundPlane(geometric::Plane *&p_biggestGroundPlane_out)
{
    geometric::Plane                         *p_bestGroundPlane = nullptr;
    std::tuple<std::size_t, std::size_t, int> bestEvidence{0U, 0U, 0};
    bool                                      hasBestEvidence = false;

    std::vector<geometric::Plane *> allPlanes{};
    if (getAllPlanes(allPlanes) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_plane : allPlanes)
    {
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant planeType{};
        if (!(p_plane == nullptr || planeIsBad) &&
            p_plane->getPlaneType(planeType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || planeIsBad ||
            planeType != geometric::Plane::PlaneVariant::GROUND)
        {
            continue;
        }

        geometric::Plane::GeometrySnapshot geometry{};
        if (p_plane->getGeometrySnapshot(geometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const double normalNorm = geometry.equation_World.head<3>().norm();
        if (!geometry.equation_World.allFinite() ||
            !std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        if (geometry.cloudGeneration != geometry.successfulRefitGeneration ||
            geometry.finiteSupportCount == 0U ||
            std::abs(normalNorm - 1.0) > 1e-3)
        {
            continue;
        }

        int planeGetId{};
        if (p_plane->getId(planeGetId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const std::tuple<unsigned long, unsigned long, int> evidence =
            std::make_tuple(geometry.finiteSupportCount,
                            geometry.observationCount,
                            -planeGetId);
        if (!hasBestEvidence || evidence > bestEvidence)
        {
            bestEvidence      = evidence;
            p_bestGroundPlane = p_plane;
            hasBestEvidence   = true;
        }
    }
    p_biggestGroundPlane_out = p_bestGroundPlane;
    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
