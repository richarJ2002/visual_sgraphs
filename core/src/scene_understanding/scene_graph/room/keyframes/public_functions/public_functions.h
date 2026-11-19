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

#include <utility>
#include <vector>

#include <Eigen/Core>

#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} // namespace geometric

namespace keyframe
{
class KeyFrame;
} // namespace keyframe

namespace semantic
{

/*!
 * @brief           Lists the observers of a plane that can vote for a room:
 *                  its observing key frames that are live and have a room,
 *                  in key frame id order.
 *
 * @param[in]       p_plane_in
 *                  Plane whose observations are read; borrowed, may be null.
 *
 * @param[out]      observerKeyFrames_out
 *                  Voting observers; borrowed. Empty for a null plane.
 *
 * @return          ROOM_STATUS_SUCCESS.
 */
[[nodiscard]] RoomStatus collectObserverKeyFrames(
    geometric::Plane                  *p_plane_in,
    std::vector<keyframe::KeyFrame *> &observerKeyFrames_out);

/*!
 * @brief           Picks the room with the most votes; on a tie, the room
 *                  whose latest vote has the highest key frame id.
 *
 * @param[in]       roomVotes_in
 *                  One vote per observer: (room id, observer key frame id).
 *                  Votes with a negative room id are ignored.
 *
 * @param[out]      majorityRoomId_out
 *                  Winning room id, or -1 without a valid vote.
 *
 * @return          ROOM_STATUS_SUCCESS.
 */
[[nodiscard]] RoomStatus selectMajorityRoomId(
    const std::vector<std::pair<int, long unsigned int>> &roomVotes_in,
    int                                                  &majorityRoomId_out);

/*!
 * @brief           Returns the camera centres of the observers of a plane
 *                  that were taken in one room: the starts of the sight
 *                  lines that room is judged from.
 *
 * @param[in]       p_plane_in
 *                  Plane whose observations are read; borrowed, may be null.
 *
 * @param[in]       roomId_in
 *                  Room the observers must have been taken in.
 *
 * @param[out]      cameraCenters_world_m_out
 *                  Camera centres in the active map frame, metres, in key
 *                  frame id order; empty when no observer was taken there.
 *
 * @return          ROOM_STATUS_SUCCESS.
 */
[[nodiscard]] RoomStatus collectObserverCameraCenters(
    geometric::Plane             *p_plane_in,
    int                           roomId_in,
    std::vector<Eigen::Vector3d> &cameraCenters_world_m_out);

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
