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
              {
                  int firstId{};
                  if (p_first->getId(firstId) !=
                      semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                  {
                      // getId cannot fail; continue as before.
                  }
                  int secondId{};
                  if (p_second->getId(secondId) !=
                      semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                  {
                      // getId cannot fail; continue as before.
                  }
                  return firstId < secondId;
              });

    std::sort(roomsB.begin(),
              roomsB.end(),
              [](const semantic::Room *p_first, const semantic::Room *p_second)
              {
                  int firstId{};
                  if (p_first->getId(firstId) !=
                      semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                  {
                      // getId cannot fail; continue as before.
                  }
                  int secondId{};
                  if (p_second->getId(secondId) !=
                      semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                  {
                      // getId cannot fail; continue as before.
                  }
                  return firstId < secondId;
              });

    for (semantic::Room *p_roomB : roomsB)
    {
        bool roomBIsBad{};
        if (!(p_roomB == nullptr) &&
            p_roomB->isBad(roomBIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        bool roomBHasRoomTag{};
        if (!(p_roomB == nullptr || roomBIsBad) &&
            p_roomB->hasRoomTag(roomBHasRoomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // hasRoomTag cannot fail; continue as before.
        }
        if (p_roomB == nullptr || roomBIsBad || !roomBHasRoomTag)
        {
            continue;
        }

        for (semantic::Room *p_roomA : roomsA)
        {
            bool roomAIsBad{};
            if (!(p_roomA == nullptr) &&
                p_roomA->isBad(roomAIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_roomA == nullptr || roomAIsBad)
            {
                continue;
            }

            std::string roomARoomTag{};
            if (p_roomA->getRoomTag(roomARoomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomTag cannot fail; continue as before.
            }
            std::string roomBRoomTag{};
            if (p_roomB->getRoomTag(roomBRoomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomTag cannot fail; continue as before.
            }
            if (roomARoomTag != roomBRoomTag)
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
