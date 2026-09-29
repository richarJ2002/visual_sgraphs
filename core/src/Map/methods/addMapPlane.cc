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

namespace vs_graphs
{
namespace core
{

void Map::addMapPlane(geometric::Plane *p_plane_inout)
{
    if (p_plane_inout == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mapMutex);

    for (auto planeIterator = planeIndex.begin();
         planeIterator != planeIndex.end();)
    {
        int planeGetId{};
        if ((planeIterator->second == p_plane_inout) &&
            p_plane_inout->getId(planeGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
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
        // getId cannot fail; continue as before.
    }
    const auto existingPlaneIterator = planeIndex.find(planeGetId2);

    int planeGetId3{};
    if (p_plane_inout->getId(planeGetId3) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
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
            // getId cannot fail; continue as before.
        }
        std::cerr << "[Map] geometric::Plane ID collision for " << planeGetId4
                  << "; reassigned to " << replacementPlaneId << "."
                  << std::endl;

        if (p_plane_inout->setId(replacementPlaneId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // setId cannot fail; continue as before.
        }
    }
    else
    {
        int planeGetId5{};
        if (p_plane_inout->getId(planeGetId5) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        nextAvailablePlaneId = std::max(nextAvailablePlaneId, planeGetId5 + 1);
    }

    planes.insert(p_plane_inout);
    int planeGetId6{};
    if (p_plane_inout->getId(planeGetId6) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    planeIndex.insert_or_assign(planeGetId6, p_plane_inout);
}

} // namespace core
} // namespace vs_graphs
