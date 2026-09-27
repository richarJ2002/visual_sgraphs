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

Floor *Floor::selectBestObservedFloor(const std::vector<Floor *> &floors_in)
{
    Floor                       *p_bestFloor = nullptr;
    std::optional<PlaneIdentity> bestIdentity;

    for (Floor *p_candidateFloor : floors_in)
    {
        if (p_candidateFloor == nullptr)
        {
            continue;
        }

        const std::optional<PlaneIdentity> candidateIdentity =
            p_candidateFloor->getPlaneIdentity();

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

        if (p_bestFloor == nullptr || candidateIsBetter ||
            (evidenceIsEqual &&
             p_candidateFloor->getId() < p_bestFloor->getId()))
        {
            p_bestFloor  = p_candidateFloor;
            bestIdentity = candidateIdentity;
        }
    }

    return p_bestFloor;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
