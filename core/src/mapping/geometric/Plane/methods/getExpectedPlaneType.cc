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
 * @file            getExpectedPlaneType.cc
 *
 * @brief           Implements Plane::getExpectedPlaneType(), declared in
 *                  mapping/geometric/Plane/objects/PlaneClass.h.
 */

/* Matching Declaration Include */
#include "mapping/geometric/Plane/objects/PlaneClass.h"

/* C++ Standard Library Includes */
#include <map>
#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus
    Plane::getExpectedPlaneType(Plane::PlaneVariant &expectedPlaneType_out)
{
    /* semanticVotes is written under typeMutex, so read it under the same. */
    std::lock_guard<std::mutex> typeLock(typeMutex);

    /*!
     * Only a vote above zero can lead, so a plane without votes stays
     * UNDEFINED. The comparison is strict and std::map iterates in key
     * order, so a tie goes to the class with the lowest enumerator value.
     */
    PlaneVariant leadingPlaneType  = PlaneVariant::UNDEFINED;
    double       leadingVoteWeight = 0.0;

    for (const std::pair<const PlaneVariant, double> &classVote : semanticVotes)
    {
        if (classVote.second > leadingVoteWeight)
        {
            leadingVoteWeight = classVote.second;
            leadingPlaneType  = classVote.first;
        }
    }

    expectedPlaneType_out = leadingPlaneType;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */
