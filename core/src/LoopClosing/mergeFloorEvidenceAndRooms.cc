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

#include "LoopClosing.h"

namespace vs_graphs
{
namespace core
{

void mergeFloorEvidenceAndRooms(semantic::Floor *p_retainedFloor_inout,
                                semantic::Floor *p_duplicateFloor_in)
{
    if (p_retainedFloor_inout == nullptr || p_duplicateFloor_in == nullptr ||
        p_retainedFloor_inout == p_duplicateFloor_in)
    {
        return;
    }

    if (semantic::Floor::selectBestObservedFloor(
            {p_retainedFloor_inout, p_duplicateFloor_in}) ==
        p_duplicateFloor_in)
    {
        const std::optional<semantic::Floor::PlaneIdentity> betterIdentity =
            p_duplicateFloor_in->getPlaneIdentity();
        if (betterIdentity.has_value())
        {
            p_retainedFloor_inout->setPlaneIdentity(
                betterIdentity->equation_World,
                betterIdentity->finiteSupportCount,
                betterIdentity->observationCount);
        }
        const Eigen::Vector3d betterCentroid =
            p_duplicateFloor_in->getCentroid();
        if (betterCentroid.allFinite())
        {
            p_retainedFloor_inout->setCentroid(betterCentroid);
        }
    }

    for (semantic::Room *p_room : p_duplicateFloor_in->getRooms())
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_retainedFloor_inout->addRoom(p_room);
        }
    }
}

} // namespace core
} // namespace vs_graphs
