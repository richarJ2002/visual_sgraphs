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

MapStatus Map::addMapPlane(geometric::Plane *p_plane_inout)
{
    if (p_plane_inout == nullptr)
    {
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    std::unique_lock<std::mutex> lock(mapMutex);

    for (auto planeIterator = planeIndex.begin();
         planeIterator != planeIndex.end();)
    {
        int planeGetId{};
        if ((planeIterator->second == p_plane_inout) &&
            p_plane_inout->getId(planeGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        planeIterator = planeIterator->second == p_plane_inout &&
                                planeIterator->first != planeGetId
                            ? planeIndex.erase(planeIterator)
                            : std::next(planeIterator);
    }

    int planeGetId2{};
    if (p_plane_inout->getId(planeGetId2) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const auto existingPlaneIterator = planeIndex.find(planeGetId2);

    int planeGetId3{};
    if (p_plane_inout->getId(planeGetId3) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (planeGetId3 < 0 || (existingPlaneIterator != planeIndex.end() &&
                            existingPlaneIterator->second != p_plane_inout))
    {
        while (planeIndex.count(nextAvailablePlaneId) > 0)
        {
            ++nextAvailablePlaneId;
        }

        const int replacementPlaneId = nextAvailablePlaneId++;

        int planeGetId4{};
        if (p_plane_inout->getId(planeGetId4) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[Map] geometric::Plane ID collision for " << planeGetId4
                  << "; reassigned to " << replacementPlaneId << "."
                  << std::endl;

        if (p_plane_inout->setId(replacementPlaneId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        int planeGetId5{};
        if (p_plane_inout->getId(planeGetId5) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        nextAvailablePlaneId = std::max(nextAvailablePlaneId, planeGetId5 + 1);
    }

    planes.insert(p_plane_inout);
    int planeGetId6{};
    if (p_plane_inout->getId(planeGetId6) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    planeIndex.insert_or_assign(planeGetId6, p_plane_inout);

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
