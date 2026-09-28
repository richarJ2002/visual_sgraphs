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

    semantic::Floor *p_bestFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(
            {p_retainedFloor_inout, p_duplicateFloor_in},
            p_bestFloor) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // selectBestObservedFloor cannot fail; continue as before.
    }
    if (p_bestFloor == p_duplicateFloor_in)
    {
        std::optional<semantic::Floor::PlaneIdentity> betterIdentity{};
        if (p_duplicateFloor_in->getPlaneIdentity(betterIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // getPlaneIdentity cannot fail; continue as before.
        }
        if (betterIdentity.has_value())
        {
            if (p_retainedFloor_inout->setPlaneIdentity(
                    betterIdentity->equation_World,
                    betterIdentity->finiteSupportCount,
                    betterIdentity->observationCount) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                // Rejected input: ignored, as before.
            }
        }
        Eigen::Vector3d betterCentroid{};
        if (p_duplicateFloor_in->getCentroid(betterCentroid) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        if (betterCentroid.allFinite())
        {
            if (p_retainedFloor_inout->setCentroid(betterCentroid) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                // setCentroid cannot fail; continue as before.
            }
        }
    }

    std::vector<vs_graphs::core::semantic::Room *> duplicateFloor_inRooms{};
    if (p_duplicateFloor_in->getRooms(duplicateFloor_inRooms) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getRooms cannot fail; continue as before.
    }
    for (semantic::Room *p_room : duplicateFloor_inRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room != nullptr && !roomIsBad)
        {
            if (p_retainedFloor_inout->addRoom(p_room) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                // addRoom cannot fail; continue as before.
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
