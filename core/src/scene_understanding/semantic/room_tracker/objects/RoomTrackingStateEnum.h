/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            RoomTrackingStateEnum.h
 *
 * @brief           Declares RoomTrackingState, the lifecycle states of the
 *                  camera relative to the mapped rooms.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKING_STATE_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKING_STATE_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Lifecycle states of the camera relative to the
 *                  mapped rooms.
 *
 *                  The state machine is self-contained: it carries
 *                  only state names, event labels, timestamps and
 *                  scalar guard values, and is therefore
 *                  frame-agnostic.
 */
enum class RoomTrackingState
{
    UNKNOWN,
    CONFIRMED_ROOM,
    CROSSING_PASSAGE,
    LOST_WITHOUT_ROOM,
    LOST_WITH_LAST_ROOM,
    REACQUIRING_IN_NEW_MAP
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKING_STATE_ENUM_H \
        */
