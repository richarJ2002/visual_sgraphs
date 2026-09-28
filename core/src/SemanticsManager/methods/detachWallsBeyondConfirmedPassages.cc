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

#include "SemanticsManager.h"

#include "../private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::detachWallsBeyondConfirmedPassages(void)
{
    const types::SystemParams::RoomSeg::PassagePartition &partitionParameters =
        p_sysParams->roomSeg.passagePartition;

    if (!partitionParameters.enabled ||
        !partitionParameters.shouldDetachWallsBeyondPassages)
    {
        return;
    }

    geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }

    const Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const double openingMargin_m =
        static_cast<double>(partitionParameters.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        partitionParameters.wallCentroidMinimumSideDistance_m);

    std::vector<semantic::Passage *> confirmedOpenPassages;

    for (semantic::Passage *p_passage : p_atlas->getAllPassages())
    {
        if (p_passage != nullptr && p_passage->isPassable())
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    std::sort(
        confirmedOpenPassages.begin(),
        confirmedOpenPassages.end(),
        [](const semantic::Passage *p_first, const semantic::Passage *p_second)
        { return p_first->getId() < p_second->getId(); });

    if (confirmedOpenPassages.empty())
    {
        return;
    }

    const std::vector<semantic::Room *> allRooms = p_atlas->getAllRooms();

    for (semantic::Room *p_room : allRooms)
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

        if (!roomCentroid_World_m.allFinite())
        {
            continue;
        }

        for (geometric::Plane *p_wall : p_room->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                p_wall->getCentroid().cast<double>();

            if (!wallCentroid_World_m.allFinite())
            {
                continue;
            }

            semantic::Passage *p_separatingPassage = nullptr;

            for (semantic::Passage *p_passage : confirmedOpenPassages)
            {
                /*
                 * A passage's own supporting wall lies on its aperture plane,
                 * so it cannot satisfy the opposite-side distance test. This
                 * preserves the wall-passage relationship while rejecting a
                 * different wall reached only through that opening.
                 */
                if (segmentCrossesPassageOpening(roomCentroid_World_m,
                                                 wallCentroid_World_m,
                                                 p_passage,
                                                 groundNormal_World,
                                                 openingMargin_m,
                                                 minimumSideDistance_m))
                {
                    p_separatingPassage = p_passage;
                    break;
                }
            }

            if (p_separatingPassage == nullptr)
            {
                continue;
            }

            semantic::Room *p_confirmedOwner = nullptr;
            for (semantic::Room *p_otherRoom : allRooms)
            {
                if (p_otherRoom == nullptr || p_otherRoom == p_room ||
                    p_otherRoom->isBad() ||
                    p_otherRoom->getRoomVariant() ==
                        semantic::Room::RoomVariant::UNDEFINED)
                {
                    continue;
                }

                const std::vector<geometric::Plane *> otherWalls =
                    p_otherRoom->getWalls();
                if (std::find(otherWalls.begin(), otherWalls.end(), p_wall) !=
                    otherWalls.end())
                {
                    p_confirmedOwner = p_otherRoom;
                    break;
                }
            }

            semantic::Room *p_farSideRoom =
                p_separatingPassage->getProspectiveRoom();
            if (p_farSideRoom != nullptr &&
                (p_farSideRoom->isBad() || p_farSideRoom == p_room))
            {
                p_farSideRoom = nullptr;
            }

            if (!p_room->removeWall(p_wall))
            {
                continue;
            }

            if (p_confirmedOwner != nullptr &&
                p_confirmedOwner != p_farSideRoom)
            {
                std::cout
                    << "[SemMgr] Detached far-side Wall#" << p_wall->getId()
                    << " from semantic::Room#" << p_room->getId()
                    << "; retained distinct confirmed owner semantic::Room#"
                    << p_confirmedOwner->getId() << "." << std::endl;
                continue;
            }

            if (p_farSideRoom != nullptr)
            {
                p_farSideRoom->setWalls(p_wall);
                if (p_atlas->getRoomWallPlaneById(p_wall->getId()) == nullptr)
                {
                    p_atlas->addRoomWallPlane(p_wall);
                }
                std::cout << "[SemMgr] Redirected far-side Wall#"
                          << p_wall->getId() << " from semantic::Room#"
                          << p_room->getId() << " through semantic::Passage#"
                          << p_separatingPassage->getId()
                          << " to stable semantic::Room#"
                          << p_farSideRoom->getId() << "." << std::endl;
                continue;
            }

            std::cout << "[SemMgr] Detached far-side Wall#" << p_wall->getId()
                      << " from semantic::Room#" << p_room->getId()
                      << "; semantic::Passage#" << p_separatingPassage->getId()
                      << " has no stable far-side room, so the wall remains "
                         "orphaned."
                      << std::endl;
        }
    }
}

} // namespace core
} // namespace vs_graphs
