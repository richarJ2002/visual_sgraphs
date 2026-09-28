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
 * @file            reAssociatePassages.cc
 *
 * @brief           Implements Utils::reAssociatePassages(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

void Utils::reAssociatePassages(Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return;
    }

    Map *p_activeMap = p_atlas_in->getCurrentMap();

    if (p_activeMap == nullptr)
    {
        return;
    }

    std::vector<semantic::Passage *> passages = p_activeMap->getAllPassages();
    const std::vector<semantic::Room *> activeRooms =
        p_activeMap->getAllRooms();
    const std::unordered_set<semantic::Room *> activeRoomSet(
        activeRooms.begin(),
        activeRooms.end());

    const auto liveRoomHandle =
        [&activeRoomSet](semantic::Room *p_room) -> semantic::Room *
    {
        return p_room != nullptr && !p_room->isBad() &&
                       activeRoomSet.count(p_room) > 0U
                   ? p_room
                   : nullptr;
    };

    std::sort(passages.begin(),
              passages.end(),
              [](const semantic::Passage *p_firstPassage,
                 const semantic::Passage *p_secondPassage)
              {
                  if (p_firstPassage == nullptr)
                  {
                      return false;
                  }

                  if (p_secondPassage == nullptr)
                  {
                      return true;
                  }

                  return p_firstPassage->getId() < p_secondPassage->getId();
              });

    const types::SystemParams::SemSeg::PassageDetection &passageParameters =
        types::SystemParams::getParams()->semSeg.passageDetection;

    /*
     * Use the same geometrically constrained identity gate as online passage
     * tracking. Coplanarity and normal checks below prevent this distance from
     * collapsing openings on unrelated walls after a map merge.
     */
    const double maximumCentroidDistance_m =
        static_cast<double>(passageParameters.duplicatePassageDistance_m);
    const double minimumNormalAlignment =
        static_cast<double>(passageParameters.duplicateNormalAlignment);
    constexpr double maximumSupportingPlaneSeparation_m = 0.30;

    std::unordered_set<semantic::Passage *> retiredPassages;

    for (std::size_t retainedIndex = 0U; retainedIndex < passages.size();
         retainedIndex++)
    {
        semantic::Passage *p_retainedPassage = passages[retainedIndex];

        if (p_retainedPassage == nullptr ||
            retiredPassages.count(p_retainedPassage) > 0U)
        {
            continue;
        }

        Eigen::Vector4d retainedEquation =
            p_retainedPassage->getGlobalEquation().coeffs();
        const double retainedNormalNorm = retainedEquation.head<3>().norm();

        if (!retainedEquation.allFinite() || retainedNormalNorm < 1e-8)
        {
            continue;
        }

        retainedEquation /= retainedNormalNorm;

        for (std::size_t candidateIndex = retainedIndex + 1U;
             candidateIndex < passages.size();
             candidateIndex++)
        {
            semantic::Passage *p_candidatePassage = passages[candidateIndex];

            if (p_candidatePassage == nullptr ||
                retiredPassages.count(p_candidatePassage) > 0U)
            {
                continue;
            }

            const Eigen::Vector3d retainedCentroid_World_m =
                p_retainedPassage->getCentroid();
            const Eigen::Vector3d candidateCentroid_World_m =
                p_candidatePassage->getCentroid();

            if (!retainedCentroid_World_m.allFinite() ||
                !candidateCentroid_World_m.allFinite() ||
                (candidateCentroid_World_m - retainedCentroid_World_m).norm() >
                    maximumCentroidDistance_m)
            {
                continue;
            }

            Eigen::Vector4d candidateEquation =
                p_candidatePassage->getGlobalEquation().coeffs();
            const double candidateNormalNorm =
                candidateEquation.head<3>().norm();

            if (!candidateEquation.allFinite() || candidateNormalNorm < 1e-8)
            {
                continue;
            }

            candidateEquation /= candidateNormalNorm;

            if (std::abs(retainedEquation.head<3>().dot(
                    candidateEquation.head<3>())) < minimumNormalAlignment)
            {
                continue;
            }

            const double retainedPlaneResidual_m = std::abs(
                retainedEquation.head<3>().dot(candidateCentroid_World_m) +
                retainedEquation(3));
            const double candidatePlaneResidual_m = std::abs(
                candidateEquation.head<3>().dot(retainedCentroid_World_m) +
                candidateEquation(3));

            if (retainedPlaneResidual_m > maximumSupportingPlaneSeparation_m ||
                candidatePlaneResidual_m > maximumSupportingPlaneSeparation_m)
            {
                continue;
            }

            semantic::Room *p_retainedHandle =
                liveRoomHandle(p_retainedPassage->getProspectiveRoom());
            semantic::Room *p_candidateHandle =
                liveRoomHandle(p_candidatePassage->getProspectiveRoom());

            p_retainedPassage->mergeKnownSideProvenance(
                p_candidatePassage->getKnownSideProvenance());
            const semantic::Passage::KnownSideProvenance knownSide =
                p_retainedPassage->getKnownSideProvenance();

            const auto isProvenFarSide =
                [&knownSide, &retainedCentroid_World_m](semantic::Room *p_room)
            {
                return p_room != nullptr && knownSide.hasDirection() &&
                       knownSide.direction_World.dot(p_room->getCentroid() -
                                                     retainedCentroid_World_m) <
                           -0.20;
            };

            semantic::Room *p_survivingHandle = p_retainedHandle;

            if (p_survivingHandle == nullptr)
            {
                p_survivingHandle = p_candidateHandle;
            }
            else if (p_candidateHandle != nullptr &&
                     p_candidateHandle != p_survivingHandle)
            {
                const bool retainedIsFarSide =
                    isProvenFarSide(p_retainedHandle);
                const bool candidateIsFarSide =
                    isProvenFarSide(p_candidateHandle);
                if (candidateIsFarSide && !retainedIsFarSide)
                {
                    p_survivingHandle = p_candidateHandle;
                }
            }

            p_retainedPassage->setProspectiveRoom(p_survivingHandle);
            if (p_survivingHandle != nullptr)
            {
                p_survivingHandle->setDoorways(p_retainedPassage);
            }

            Eigen::Vector3d fusedCentroid_World_m =
                0.5 * (retainedCentroid_World_m + candidateCentroid_World_m);
            const double fusedCentroidResidual_m =
                retainedEquation.head<3>().dot(fusedCentroid_World_m) +
                retainedEquation(3);
            fusedCentroid_World_m -=
                fusedCentroidResidual_m * retainedEquation.head<3>();

            p_retainedPassage->setCentroid(fusedCentroid_World_m);
            p_retainedPassage->setWidth(
                std::max(p_retainedPassage->getWidth(),
                         p_candidatePassage->getWidth()));
            p_retainedPassage->setHeight(
                std::max(p_retainedPassage->getHeight(),
                         p_candidatePassage->getHeight()));
            p_retainedPassage->setPassable(p_retainedPassage->isPassable() ||
                                           p_candidatePassage->isPassable());

            const std::size_t retainedTraversalCount =
                p_retainedPassage->getTraversalObservationCount();
            const std::size_t candidateTraversalCount =
                p_candidatePassage->getTraversalObservationCount();
            const std::size_t maximumTraversalCount =
                std::numeric_limits<std::size_t>::max();

            p_retainedPassage->setTraversalObservationCount(
                candidateTraversalCount >
                        maximumTraversalCount - retainedTraversalCount
                    ? maximumTraversalCount
                    : retainedTraversalCount + candidateTraversalCount);

            for (geometric::Plane *p_supportingWall :
                 p_candidatePassage->getAssociateWalls())
            {
                p_retainedPassage->addAssociateWall(p_supportingWall);
            }

            if (p_retainedPassage->getAssociateDoor() == nullptr &&
                p_candidatePassage->getAssociateDoor() != nullptr)
            {
                p_retainedPassage->setAssociateDoor(
                    p_candidatePassage->getAssociateDoor());
            }

            for (semantic::Room *p_room : p_activeMap->getAllRooms())
            {
                if (p_room != nullptr && !p_room->isBad())
                {
                    p_room->replacePassageAssociation(p_candidatePassage,
                                                      p_retainedPassage);
                }
            }

            for (KeyFrame *p_keyFrame : p_activeMap->getAllKeyFrames())
            {
                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    p_keyFrame->replaceMapPassage(p_candidatePassage,
                                                  p_retainedPassage);
                }
            }

            p_activeMap->eraseMapPassage(p_candidatePassage);
            p_candidatePassage->setProspectiveRoom(nullptr);
            p_candidatePassage->setMap(nullptr);
            retiredPassages.insert(p_candidatePassage);
        }
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
