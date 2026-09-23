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
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::enforceUniqueWallOwnership(void)
{
    std::vector<vs_graphs::core::semantic::Room *> allRooms =
        p_atlas->getAllRooms();
    std::sort(allRooms.begin(),
              allRooms.end(),
              [](const semantic::Room *p_firstRoom,
                 const semantic::Room *p_secondRoom)
              {
                  if (p_firstRoom == nullptr)
                  {
                      return false;
                  }

                  if (p_secondRoom == nullptr)
                  {
                      return true;
                  }

                  return p_firstRoom->getId() < p_secondRoom->getId();
              });

    std::vector<semantic::Passage *> allPassages = p_atlas->getAllPassages();
    std::sort(
        allPassages.begin(),
        allPassages.end(),
        [](const semantic::Passage *p_first, const semantic::Passage *p_second)
        {
            if (p_first == nullptr)
            {
                return false;
            }
            if (p_second == nullptr)
            {
                return true;
            }
            return p_first->getId() < p_second->getId();
        });

    Eigen::Vector3d   groundNormal_World = Eigen::Vector3d::Zero();
    geometric::Plane *p_groundPlane      = p_atlas->getBiggestGroundPlane();
    if (p_groundPlane != nullptr && !p_groundPlane->isBad())
    {
        const Eigen::Vector4d groundEquation =
            p_groundPlane->getGlobalEquation().coeffs();
        const double groundNormalNorm = groundEquation.head<3>().norm();
        if (groundEquation.allFinite() && groundNormalNorm > 1e-8)
        {
            groundNormal_World = groundEquation.head<3>() / groundNormalNorm;
        }
    }

    std::unordered_map<geometric::Plane *, std::vector<semantic::Room *>>
        wallOwners;

    for (semantic::Room *p_room : allRooms)
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        for (geometric::Plane *p_wall : p_room->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            std::vector<semantic::Room *> &owners = wallOwners[p_wall];
            if (std::find(owners.begin(), owners.end(), p_room) == owners.end())
            {
                owners.push_back(p_room);
            }
        }
    }

    for (auto &[p_wall, owners] : wallOwners)
    {
        if (owners.size() < 2U)
        {
            continue;
        }

        semantic::Room                      *p_retainedOwner = nullptr;
        std::unordered_set<semantic::Room *> passageRejectedOwners;

        /* Passage-side routing is authoritative. A near-side owner whose
         * centroid-to-wall segment crosses an opening is not eligible; a live
         * stable far-side handle is preferred unless that would steal from a
         * different confirmed owner.
         *
         * Eligibility here is deliberately geometric only (isPassable(), the
         * passage's own detected-opening evidence) -- traversal evidence
         * (the camera/UAV having flown through this spot) proves only that
         * a room change happened there, not this passage's own aperture
         * geometry. Substituting it in as an OR-alternative would let a
         * geometrically-unconfirmed "passage" arbitrate which confirmed
         * room owns a contested wall, conflating motion evidence with wall
         * identity. */
        for (semantic::Room *p_nearOwner : owners)
        {
            for (semantic::Passage *p_passage : allPassages)
            {
                if (p_passage == nullptr || !p_passage->isPassable() ||
                    !segmentCrossesPassageOpening(
                        p_nearOwner->getCentroid(),
                        p_wall->getCentroid().cast<double>(),
                        p_passage,
                        groundNormal_World,
                        p_sysParams->roomSeg.passagePartition.openingMargin_m,
                        p_sysParams->roomSeg.passagePartition
                            .minimumSideDistance_m,
                        false))
                {
                    continue;
                }

                semantic::Room *p_farSideOwner =
                    p_passage->getProspectiveRoom();
                if (p_farSideOwner == nullptr || p_farSideOwner->isBad() ||
                    p_farSideOwner == p_nearOwner ||
                    p_farSideOwner->getMap() != p_atlas->getCurrentMap())
                {
                    passageRejectedOwners.insert(p_nearOwner);
                    continue;
                }

                const bool wouldStealDistinctConfirmedOwner = std::any_of(
                    owners.begin(),
                    owners.end(),
                    [p_nearOwner, p_farSideOwner](semantic::Room *p_owner)
                    {
                        return p_owner != p_nearOwner &&
                               p_owner != p_farSideOwner &&
                               p_owner->getRoomVariant() ==
                                   semantic::Room::RoomVariant::ROOM;
                    });
                if (wouldStealDistinctConfirmedOwner)
                {
                    continue;
                }

                if (p_retainedOwner == nullptr ||
                    (p_farSideOwner->getRoomVariant() ==
                         semantic::Room::RoomVariant::ROOM &&
                     p_retainedOwner->getRoomVariant() !=
                         semantic::Room::RoomVariant::ROOM) ||
                    (p_farSideOwner->getRoomVariant() ==
                         p_retainedOwner->getRoomVariant() &&
                     p_farSideOwner->getId() < p_retainedOwner->getId()))
                {
                    p_retainedOwner = p_farSideOwner;
                }
            }
        }

        /* Existing confirmed ownership outranks camera proximity. */
        if (p_retainedOwner == nullptr)
        {
            for (semantic::Room *p_owner : owners)
            {
                if (passageRejectedOwners.count(p_owner) == 0U &&
                    p_owner->getRoomVariant() ==
                        semantic::Room::RoomVariant::ROOM)
                {
                    p_retainedOwner = p_owner;
                    break;
                }
            }
        }

        /* Camera proximity is the final fallback among equivalent/provisional
         * owners only. */
        if (p_retainedOwner == nullptr)
        {
            Eigen::Vector3d meanObservationPosition_World_m =
                Eigen::Vector3d::Zero();
            std::size_t validObservationCount = 0U;

            for (const auto &[p_keyFrame, observation] :
                 p_wall->getObservations())
            {
                static_cast<void>(observation);

                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    const Eigen::Vector3d cameraCenter_World_m =
                        p_keyFrame->getCameraCenter().cast<double>();

                    if (cameraCenter_World_m.allFinite())
                    {
                        meanObservationPosition_World_m += cameraCenter_World_m;
                        validObservationCount++;
                    }
                }
            }

            if (validObservationCount > 0U)
            {
                meanObservationPosition_World_m /=
                    static_cast<double>(validObservationCount);
                double bestDistance_m = std::numeric_limits<double>::infinity();

                for (semantic::Room *p_owner : owners)
                {
                    if (passageRejectedOwners.count(p_owner) > 0U)
                    {
                        continue;
                    }

                    const double distance_m = (p_owner->getCentroid() -
                                               meanObservationPosition_World_m)
                                                  .norm();
                    if (distance_m < bestDistance_m)
                    {
                        bestDistance_m  = distance_m;
                        p_retainedOwner = p_owner;
                    }
                }
            }

            if (p_retainedOwner == nullptr)
            {
                for (semantic::Room *p_owner : owners)
                {
                    if (passageRejectedOwners.count(p_owner) == 0U)
                    {
                        p_retainedOwner = p_owner;
                        break;
                    }
                }
            }
        }

        for (semantic::Room *p_owner : owners)
        {
            if (p_owner != p_retainedOwner && p_owner->removeWall(p_wall))
            {
                std::cerr << "[SemMgr] Corrected duplicate ownership of Wall#"
                          << p_wall->getId() << ": "
                          << (p_retainedOwner != nullptr
                                  ? "retained semantic::Room#" +
                                        std::to_string(p_retainedOwner->getId())
                                  : "left orphaned")
                          << ", detached semantic::Room#" << p_owner->getId()
                          << "." << std::endl;
            }
        }

        if (p_retainedOwner != nullptr &&
            std::find(owners.begin(), owners.end(), p_retainedOwner) ==
                owners.end())
        {
            p_retainedOwner->setWalls(p_wall);
        }
    }
}

} // namespace core
} // namespace vs_graphs
