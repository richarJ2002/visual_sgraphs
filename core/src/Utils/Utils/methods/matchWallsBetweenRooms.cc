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
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::matchWallsBetweenRooms(
    const semantic::Room         *p_roomA_in,
    const semantic::Room         *p_roomB_in,
    std::vector<Eigen::Vector3d> &normalsA_inout,
    std::vector<Eigen::Vector3d> &centroidsA_inout,
    std::vector<Eigen::Vector3d> &normalsB_inout,
    std::vector<Eigen::Vector3d> &centroidsB_inout,
    std::size_t                  &matchedWallCount_out)
{
    if (p_roomA_in == nullptr || p_roomB_in == nullptr)
    {
        matchedWallCount_out = 0;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    constexpr double WALL_CORRESPONDENCE_COS_THETA = 0.85;

    const auto collectValidWalls = [](const semantic::Room *p_room_in)
        -> std::vector<std::pair<geometric::Plane *, Eigen::Vector3d>>
    {
        std::vector<std::pair<geometric::Plane *, Eigen::Vector3d>> validWalls;

        std::vector<geometric::Plane *> room_inWalls{};
        if (p_room_in->getWalls(room_inWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : room_inWalls)
        {
            bool wallIsBad{};
            if (!(p_wall == nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall == nullptr || wallIsBad)
            {
                continue;
            }

            std::optional<Eigen::Vector3d> wallNormal_world{};
            if (p_room_in->getWallNormalTowardRoom_world(p_wall,
                                                         wallNormal_world) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWallNormalTowardRoom_world cannot fail; continue as
                // before.
            }

            if (!wallNormal_world.has_value())
            {
                continue;
            }

            validWalls.emplace_back(p_wall, wallNormal_world.value());
        }

        std::sort(
            validWalls.begin(),
            validWalls.end(),
            [](const std::pair<geometric::Plane *, Eigen::Vector3d> &first_in,
               const std::pair<geometric::Plane *, Eigen::Vector3d> &second_in)
            {
                int getId2{};
                if (first_in.first->getId(getId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int getId3{};
                if (second_in.first->getId(getId3) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return getId2 < getId3;
            });

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

        if (bestNormalAlignment <= WALL_CORRESPONDENCE_COS_THETA)
        {
            break;
        }

        matchedA[bestIndexA] = true;
        matchedB[bestIndexB] = true;

        normalsA_inout.push_back(validWallsA[bestIndexA].second);
        Eigen::Vector3d getCentroid2{};
        if (validWallsA[bestIndexA].first->getCentroid(getCentroid2) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        centroidsA_inout.push_back(getCentroid2);
        normalsB_inout.push_back(validWallsB[bestIndexB].second);
        Eigen::Vector3d getCentroid3{};
        if (validWallsB[bestIndexB].first->getCentroid(getCentroid3) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        centroidsB_inout.push_back(getCentroid3);

        acceptedPairCount++;
    }

    matchedWallCount_out = acceptedPairCount;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
