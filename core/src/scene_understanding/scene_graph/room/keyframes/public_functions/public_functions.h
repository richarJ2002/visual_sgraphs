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
 * @file            public_functions.h
 *
 * @brief           Declares the public free functions of the room key frame
 *                  subpackage.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_KEYFRAMES_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_KEYFRAMES_PUBLIC_FUNCTIONS_H

#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} // namespace geometric

namespace semantic
{

/*!
 * @brief           Finds the observer room of a plane: the room most of its
 *                  observing key frames were taken in; on a tie, the room of
 *                  the latest (highest id) observer among the tied rooms.
 *                  Observers that are bad or have no room do not vote.
 *
 * @param[in]       p_plane_in
 *                  Plane whose observations are read; borrowed, may be null.
 *
 * @param[out]      observerRoomId_out
 *                  Id of the observer room, or -1 when no observer has a
 *                  room (or the plane is null).
 *
 * @return          ROOM_STATUS_SUCCESS.
 */
[[nodiscard]] RoomStatus findObserverRoomId(geometric::Plane *p_plane_in,
                                            int &observerRoomId_out);

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_KEYFRAMES_PUBLIC_FUNCTIONS_H */
