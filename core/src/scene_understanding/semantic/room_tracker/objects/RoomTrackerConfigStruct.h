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
 * @file            RoomTrackerConfigStruct.h
 *
 * @brief           Declares RoomTrackerConfig, the RoomTracker tuning
 *                  parameters.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKER_CONFIG_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKER_CONFIG_STRUCT_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           RoomTracker tuning parameters.
 */
struct RoomTrackerConfig
{
    /*!
     * @brief           Minimum continuous crossed-passage dwell before
     *                  committing the CONFIRMED_ROOM <-> CROSSING_PASSAGE
     *                  transitions (seconds).
     */
    double       crossing_dwell_s = 2.0;
    /*!
     * @brief           Minimum traversal confidence (0..1) for a crossing to
     *                  count.
     */
    double       crossing_confidence = 0.7;
    /*!
     * @brief           Maximum time in LOST_WITH_LAST_ROOM before decay to
     *                  LOST_WITHOUT_ROOM (seconds).
     */
    double       lost_timeout_s = 30.0;
    /*!
     * @brief           Maximum time in REACQUIRING_IN_NEW_MAP before decay to
     *                  LOST_WITHOUT_ROOM (seconds).
     */
    double       reacquire_timeout_s = 60.0;
    /*!
     * @brief           Retry backoff between failed reacquire attempts
     *                  (seconds).
     */
    double       reacquire_retry_interval_s = 5.0;
    /*!
     * @brief           Maximum failed reacquire attempts before timeout
     *                  applies.
     */
    unsigned int reacquire_max_retries = 3U;
    /*!
     * @brief           Minimum planes required to attempt a reacquire.
     *                  Consumed by the verification stub; acceptance
     *                  still requires the full verification gates.
     */
    unsigned int reacquire_min_planes = 3U;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKER_CONFIG_STRUCT_H \
        */
