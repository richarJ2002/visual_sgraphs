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
 * @file            RoomTrackingEventEnum.h
 *
 * @brief           Declares RoomTrackingEvent, the events consumed by the
 *                  transition engine.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKING_EVENT_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKING_EVENT_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Events consumed by the transition engine.
 *
 *                  The set of (source, event) rows is exactly the
 *                  transition table. An event with no defined row for
 *                  the current source state is rejected: the state
 *                  does not change and a rejection record is logged.
 */
enum class RoomTrackingEvent
{
    FIRST_ROOM_CONFIRMED,
    PASSAGE_CROSSING_DETECTED,
    TRACKING_LOST,
    PASSAGE_TRAVERSAL_COMPLETE,
    ROOM_REACQUIRED,
    NEW_MAP_WITH_ROOM_MATCH,
    VERIFIED_MATCH_TO_LAST_ROOM,
    LOST_TIMEOUT,
    REACQUIRE_TIMEOUT
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKING_EVENT_ENUM_H \
        */
