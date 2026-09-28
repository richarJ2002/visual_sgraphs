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

#include "Semantic/Floor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

FloorStatus
    Floor::selectBestObservedFloor(const std::vector<Floor *> &floors_in,
                                   Floor                     *&p_bestFloor_out)
{
    Floor                       *p_bestFloor = nullptr;
    std::optional<PlaneIdentity> bestIdentity;

    for (Floor *p_candidateFloor : floors_in)
    {
        if (p_candidateFloor == nullptr)
        {
            continue;
        }

        std::optional<PlaneIdentity> candidateIdentity{};
        if (p_candidateFloor->getPlaneIdentity(candidateIdentity) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // getPlaneIdentity cannot fail; continue as before.
        }

        const bool candidateIsBetter =
            candidateIdentity.has_value() &&
            (!bestIdentity.has_value() ||
             candidateIdentity->finiteSupportCount >
                 bestIdentity->finiteSupportCount ||
             (candidateIdentity->finiteSupportCount ==
                  bestIdentity->finiteSupportCount &&
              candidateIdentity->observationCount >
                  bestIdentity->observationCount));

        const bool evidenceIsEqual =
            candidateIdentity.has_value() == bestIdentity.has_value() &&
            (!candidateIdentity.has_value() ||
             (candidateIdentity->finiteSupportCount ==
                  bestIdentity->finiteSupportCount &&
              candidateIdentity->observationCount ==
                  bestIdentity->observationCount));

        int candidateFloorId{};
        if (!(p_bestFloor == nullptr || candidateIsBetter) &&
            (evidenceIsEqual) &&
            p_candidateFloor->getId(candidateFloorId) !=
                FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        int bestFloorId{};
        if (!(p_bestFloor == nullptr || candidateIsBetter) &&
            (evidenceIsEqual) &&
            p_bestFloor->getId(bestFloorId) !=
                FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        if (p_bestFloor == nullptr || candidateIsBetter ||
            (evidenceIsEqual && candidateFloorId < bestFloorId))
        {
            p_bestFloor  = p_candidateFloor;
            bestIdentity = candidateIdentity;
        }
    }

    p_bestFloor_out = p_bestFloor;
    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
