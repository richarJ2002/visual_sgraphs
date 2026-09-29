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

geometric::Plane *Map::getBiggestGroundPlane()
{
    geometric::Plane                         *p_bestGroundPlane = nullptr;
    std::tuple<std::size_t, std::size_t, int> bestEvidence{0U, 0U, 0};
    bool                                      hasBestEvidence = false;

    for (geometric::Plane *p_plane : getAllPlanes())
    {
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        geometric::Plane::PlaneVariant planeType{};
        if (!(p_plane == nullptr || planeIsBad) &&
            p_plane->getPlaneType(planeType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getPlaneType cannot fail; continue as before.
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
            // getGeometrySnapshot cannot fail; continue as before.
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
            // getId cannot fail; continue as before.
        }
        const auto evidence = std::make_tuple(geometry.finiteSupportCount,
                                              geometry.observationCount,
                                              -planeGetId);
        if (!hasBestEvidence || evidence > bestEvidence)
        {
            bestEvidence      = evidence;
            p_bestGroundPlane = p_plane;
            hasBestEvidence   = true;
        }
    }
    return p_bestGroundPlane;
}

} // namespace core
} // namespace vs_graphs
