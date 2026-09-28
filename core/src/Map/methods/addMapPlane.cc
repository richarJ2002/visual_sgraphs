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
        planeIterator = planeIterator->second == p_plane_inout &&
                                planeIterator->first != p_plane_inout->getId()
                            ? planeIndex.erase(planeIterator)
                            : std::next(planeIterator);
    }

    const auto existingPlaneIterator = planeIndex.find(p_plane_inout->getId());

    if (p_plane_inout->getId() < 0 ||
        (existingPlaneIterator != planeIndex.end() &&
         existingPlaneIterator->second != p_plane_inout))
    {
        while (planeIndex.count(nextAvailablePlaneId) > 0)
        {
            ++nextAvailablePlaneId;
        }

        const int replacementPlaneId = nextAvailablePlaneId++;

        std::cerr << "[Map] geometric::Plane ID collision for "
                  << p_plane_inout->getId() << "; reassigned to "
                  << replacementPlaneId << "." << std::endl;

        p_plane_inout->setId(replacementPlaneId);
    }
    else
    {
        nextAvailablePlaneId =
            std::max(nextAvailablePlaneId, p_plane_inout->getId() + 1);
    }

    planes.insert(p_plane_inout);
    planeIndex.insert_or_assign(p_plane_inout->getId(), p_plane_inout);
}

} // namespace core
} // namespace vs_graphs
