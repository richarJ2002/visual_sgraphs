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
 * @file            matchWallsBetweenRooms.cc
 *
 * @brief           Implements Utils::matchWallsBetweenRooms(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <optional>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

std::size_t
    Utils::matchWallsBetweenRooms(const semantic::Room         *p_roomA_in,
                                  const semantic::Room         *p_roomB_in,
                                  std::vector<Eigen::Vector3d> &normalsA_out,
                                  std::vector<Eigen::Vector3d> &centroidsA_out,
                                  std::vector<Eigen::Vector3d> &normalsB_out,
                                  std::vector<Eigen::Vector3d> &centroidsB_out)
{
    if (p_roomA_in == nullptr || p_roomB_in == nullptr)
    {
        return 0;
    }

    constexpr double kWallCorrespondenceCosTheta = 0.85;

    const auto collectValidWalls = [](const semantic::Room *p_room_in)
        -> std::vector<std::pair<geometric::Plane *, Eigen::Vector3d>>
    {
        std::vector<std::pair<geometric::Plane *, Eigen::Vector3d>> validWalls;

        for (geometric::Plane *p_wall : p_room_in->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const std::optional<Eigen::Vector3d> normal_World =
                p_room_in->getWallNormalTowardRoom_World(p_wall);

            if (!normal_World.has_value())
            {
                continue;
            }

            validWalls.emplace_back(p_wall, normal_World.value());
        }

        std::sort(
            validWalls.begin(),
            validWalls.end(),
            [](const std::pair<geometric::Plane *, Eigen::Vector3d> &first_in,
               const std::pair<geometric::Plane *, Eigen::Vector3d> &second_in)
            { return first_in.first->getId() < second_in.first->getId(); });

        return validWalls;
    };

    const std::vector<std::pair<geometric::Plane *, Eigen::Vector3d>>
        validWallsA = collectValidWalls(p_roomA_in);
    const std::vector<std::pair<geometric::Plane *, Eigen::Vector3d>>
        validWallsB = collectValidWalls(p_roomB_in);

    std::vector<bool> matchedA(validWallsA.size(), false);
    std::vector<bool> matchedB(validWallsB.size(), false);

    std::size_t acceptedPairCount = 0;

    while (true)
    {
        double      bestNormalAlignment = -1.0;
        std::size_t bestIndexA          = 0;
        std::size_t bestIndexB          = 0;

        for (std::size_t indexA = 0; indexA < validWallsA.size(); ++indexA)
        {
            if (matchedA[indexA])
            {
                continue;
            }

            for (std::size_t indexB = 0; indexB < validWallsB.size(); ++indexB)
            {
                if (matchedB[indexB])
                {
                    continue;
                }

                const double normalAlignment =
                    validWallsA[indexA].second.dot(validWallsB[indexB].second);

                if (normalAlignment > bestNormalAlignment)
                {
                    bestNormalAlignment = normalAlignment;
                    bestIndexA          = indexA;
                    bestIndexB          = indexB;
                }
            }
        }

        if (bestNormalAlignment <= kWallCorrespondenceCosTheta)
        {
            break;
        }

        matchedA[bestIndexA] = true;
        matchedB[bestIndexB] = true;

        normalsA_out.push_back(validWallsA[bestIndexA].second);
        centroidsA_out.push_back(validWallsA[bestIndexA].first->getCentroid());
        normalsB_out.push_back(validWallsB[bestIndexB].second);
        centroidsB_out.push_back(validWallsB[bestIndexB].first->getCentroid());

        acceptedPairCount++;
    }

    return acceptedPairCount;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
