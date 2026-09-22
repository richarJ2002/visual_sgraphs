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

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

bool KeyFrame::replaceMapPlane(geometric::Plane *p_retiredPlane_in,
                               geometric::Plane *p_retainedPlane_in)
{
    if (p_retiredPlane_in == nullptr || p_retainedPlane_in == nullptr ||
        p_retiredPlane_in == p_retainedPlane_in)
    {
        return false;
    }

    unique_lock<mutex> lock(mMutexFeatures);

    bool                            replacedRetiredPlane = false;
    std::vector<geometric::Plane *> rebuiltPlanes;
    rebuiltPlanes.reserve(mapPlanes.size());

    for (geometric::Plane *p_existingPlane : mapPlanes)
    {
        geometric::Plane *p_candidatePlane = p_existingPlane;

        if (p_existingPlane == p_retiredPlane_in)
        {
            p_candidatePlane     = p_retainedPlane_in;
            replacedRetiredPlane = true;
        }

        if (p_candidatePlane == nullptr ||
            std::find(rebuiltPlanes.begin(),
                      rebuiltPlanes.end(),
                      p_candidatePlane) != rebuiltPlanes.end())
        {
            continue;
        }

        rebuiltPlanes.push_back(p_candidatePlane);
    }

    if (replacedRetiredPlane)
    {
        mapPlanes.swap(rebuiltPlanes);
    }

    return replacedRetiredPlane;
}

} // namespace core
} // namespace vs_graphs
