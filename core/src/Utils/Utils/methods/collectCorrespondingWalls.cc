/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            collectCorrespondingWalls.cc
 *
 * @brief           Implements Utils::collectCorrespondingWalls(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

bool Utils::collectCorrespondingWalls(
    Map                          *p_mapA_in,
    Map                          *p_mapB_in,
    std::vector<Eigen::Vector3d> &normalsA_inout,
    std::vector<Eigen::Vector3d> &centroidsA_inout,
    std::vector<Eigen::Vector3d> &normalsB_inout,
    std::vector<Eigen::Vector3d> &centroidsB_inout)
{
    if (p_mapA_in == nullptr || p_mapB_in == nullptr)
    {
        return false;
    }

    std::vector<semantic::Room *> roomsA = p_mapA_in->getAllRooms();
    std::vector<semantic::Room *> roomsB = p_mapB_in->getAllRooms();

    std::sort(roomsA.begin(),
              roomsA.end(),
              [](const semantic::Room *p_first, const semantic::Room *p_second)
              { return p_first->getId() < p_second->getId(); });

    std::sort(roomsB.begin(),
              roomsB.end(),
              [](const semantic::Room *p_first, const semantic::Room *p_second)
              { return p_first->getId() < p_second->getId(); });

    for (semantic::Room *p_roomB : roomsB)
    {
        if (p_roomB == nullptr || p_roomB->isBad() || !p_roomB->hasRoomTag())
        {
            continue;
        }

        for (semantic::Room *p_roomA : roomsA)
        {
            if (p_roomA == nullptr || p_roomA->isBad())
            {
                continue;
            }

            if (p_roomA->getRoomTag() != p_roomB->getRoomTag())
            {
                continue;
            }

            matchWallsBetweenRooms(p_roomA,
                                   p_roomB,
                                   normalsA_inout,
                                   centroidsA_inout,
                                   normalsB_inout,
                                   centroidsB_inout);
            break;
        }
    }

    return normalsA_inout.size() >= 3 &&
           normalsA_inout.size() == normalsB_inout.size();
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
