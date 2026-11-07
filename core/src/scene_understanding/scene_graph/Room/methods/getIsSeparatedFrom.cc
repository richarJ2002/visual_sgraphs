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
 * @file            getIsSeparatedFrom.cc
 *
 * @brief           Implements Room::getIsSeparatedFrom(), declared in
 *                  scene_graph/Room/objects/RoomClass.h.
 */

#include "configuration/types/types.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/Room/objects/RoomClass.h"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::getIsSeparatedFrom(const Room *p_other_in,
                                    bool       &isSeparated_out) const
{
    isSeparated_out = false;
    if (p_other_in == nullptr || p_other_in == this)
    {
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    std::vector<Passage *>          ownPassages{}, otherPassages{};
    std::vector<geometric::Plane *> ownWalls{}, otherWalls{};
    if (getPassages(ownPassages) != RoomStatus::ROOM_STATUS_SUCCESS ||
        p_other_in->getPassages(otherPassages) !=
            RoomStatus::ROOM_STATUS_SUCCESS ||
        getWalls(ownWalls) != RoomStatus::ROOM_STATUS_SUCCESS ||
        p_other_in->getWalls(otherWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages/getWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /*
     * A passage has two sides: a room is on it when the passage is among its
     * doorways, it is the passage's prospective (far-side) room, or it owns
     * one of the passage's wall faces. Two rooms on one passage are on its
     * two sides.
     */
    const auto isOnPassage =
        [](Passage                               *p_passage_in,
           const Room                            *p_room_in,
           const std::vector<Passage *>          &roomPassages_in,
           const std::vector<geometric::Plane *> &roomWalls_in)
    {
        if (std::find(roomPassages_in.begin(),
                      roomPassages_in.end(),
                      p_passage_in) != roomPassages_in.end())
        {
            return true;
        }
        Room                           *p_prospectiveRoom = nullptr;
        std::vector<geometric::Plane *> passageWalls{};
        if (p_passage_in->getProspectiveRoom(p_prospectiveRoom) !=
                PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage_in->getAssociateWalls(passageWalls) !=
                PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom/getAssociateWalls returned a "
                         "failure status although it cannot fail; continuing "
                         "as before.",
                         __func__);
        }
        return p_prospectiveRoom == p_room_in ||
               std::any_of(passageWalls.begin(),
                           passageWalls.end(),
                           [&roomWalls_in](geometric::Plane *p_face)
                           {
                               return std::find(roomWalls_in.begin(),
                                                roomWalls_in.end(),
                                                p_face) != roomWalls_in.end();
                           });
    };
    /*!
     * The passage record alone is topology that is never re-checked: a
     * wall-less seed (the passage's prospective room) and the room that has
     * since taken its far-side space both stay on the passage. For such a
     * pair only, the two rooms are separated only when their centres lie on
     * its two sides; every other pair on one passage stays separated. A
     * centre within half the thinnest wall
     * (optimization.wall_thickness.minimum_thickness) of the passage plane
     * is inside the plane's own thickness and resolves no side, so the veto
     * stands; so it does for an unusable plane or centre.
     */
    types::SystemParams *p_systemParams = nullptr;
    if (types::SystemParams::getParams(p_systemParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const double passageSideMargin_m =
        0.5 *
        static_cast<double>(p_systemParams->optimization.minimumThickness_m);
    Eigen::Vector3d ownCentroid_world_m{}, otherCentroid_world_m{};
    if (getCentroid(ownCentroid_world_m) != RoomStatus::ROOM_STATUS_SUCCESS ||
        p_other_in->getCentroid(otherCentroid_world_m) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const auto areCentresAcrossPassage =
        [this,
         p_other_in,
         &ownWalls,
         &otherWalls,
         &ownCentroid_world_m,
         &otherCentroid_world_m,
         passageSideMargin_m](Passage *p_passage_in)
    {
        Room *p_prospectiveRoom = nullptr;
        if (p_passage_in->getProspectiveRoom(p_prospectiveRoom) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool hasWallLessSeed =
            (p_prospectiveRoom == this && ownWalls.empty()) ||
            (p_prospectiveRoom == p_other_in && otherWalls.empty());
        if (!hasWallLessSeed)
        {
            return true;
        }
        g2o::Plane3D passageGlobalEquation{};
        if (p_passage_in->getGlobalEquation(passageGlobalEquation) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d passageEquation_world = passageGlobalEquation.coeffs();
        const double    normalNorm = passageEquation_world.head<3>().norm();
        if (!passageEquation_world.allFinite() || normalNorm < 1e-8)
        {
            return true;
        }
        passageEquation_world /= normalNorm;
        const double ownSide_m =
            passageEquation_world.head<3>().dot(ownCentroid_world_m) +
            passageEquation_world(3);
        const double otherSide_m =
            passageEquation_world.head<3>().dot(otherCentroid_world_m) +
            passageEquation_world(3);
        if (!std::isfinite(ownSide_m) || !std::isfinite(otherSide_m) ||
            std::abs(ownSide_m) < passageSideMargin_m ||
            std::abs(otherSide_m) < passageSideMargin_m)
        {
            return true;
        }
        return ownSide_m * otherSide_m < 0.0;
    };

    std::vector<Passage *> candidatePassages = ownPassages;
    candidatePassages.insert(candidatePassages.end(),
                             otherPassages.begin(),
                             otherPassages.end());
    for (Passage *p_passage : candidatePassages)
    {
        if (p_passage != nullptr &&
            isOnPassage(p_passage, this, ownPassages, ownWalls) &&
            isOnPassage(p_passage, p_other_in, otherPassages, otherWalls) &&
            areCentresAcrossPassage(p_passage))
        {
            isSeparated_out = true;
            return RoomStatus::ROOM_STATUS_SUCCESS;
        }
    }

    /* The two faces of one physical wall bound two different rooms. */
    for (geometric::Plane *p_wall : ownWalls)
    {
        geometric::Plane *p_twinFace = nullptr;
        if (p_wall != nullptr &&
            p_wall->getTwinFace(p_twinFace) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_twinFace != nullptr &&
            std::find(otherWalls.begin(), otherWalls.end(), p_twinFace) !=
                otherWalls.end())
        {
            isSeparated_out = true;
            return RoomStatus::ROOM_STATUS_SUCCESS;
        }
    }
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
