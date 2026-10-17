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
 * @file            TraversalGuardValuesStruct.h
 *
 * @brief           Declares TraversalGuardValues, the guard values describing
 *                  one transition attempt.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_TRAVERSAL_GUARD_VALUES_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_TRAVERSAL_GUARD_VALUES_STRUCT_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Guard values describing one transition attempt.
 */
struct TraversalGuardValues
{
    /*!
     * @brief           Continuous time the guard has been satisfied. The caller
     *                  (RoomTracker::step in production, the tests in unit
     *                  tests) supplies it; RoomTracker::step accumulates it
     *                  across the dwell timers.
     *
     * @units           seconds
     */
    double dwell_s = 0.0;

    /*!
     * @brief           Traversal/verification confidence in [0, 1].
     */
    double confidence = 0.0;

    /*!
     * @brief           A trajectory segment crossed a passable passage aperture
     *                  (segmentCrossesPassageOpening()).
     */
    bool isPassageDetected = false;

    /*!
     * @brief           The crossed passage is passable (Passage::isPassable()).
     */
    bool isPassable = false;

    /*!
     * @brief           Both sides of the passage have been observed.
     */
    bool areBothSidesObserved = false;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_TRAVERSAL_GUARD_VALUES_STRUCT_H \
        */
