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

#include <cmath>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::mergeOverlappingPassages(void)
{
    geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();
    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }
    const Eigen::Vector4d groundEq =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNorm = groundEq.head<3>().norm();
    if (!groundEq.allFinite() || groundNorm < 1e-8)
    {
        return;
    }
    const Eigen::Vector3d groundNormal_World = groundEq.head<3>() / groundNorm;

    const std::vector<semantic::Passage *> allPassages =
        p_atlas->getAllPassages();

    for (std::size_t allPassageIndex = 0U; allPassageIndex < allPassages.size();
         ++allPassageIndex)
    {
        semantic::Passage *p_first = allPassages[allPassageIndex];
        bool               firstIsBad{};
        if (!(p_first == nullptr) &&
            p_first->isBad(firstIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_first == nullptr || firstIsBad)
        {
            continue;
        }

        for (std::size_t otherPassageIndex = allPassageIndex + 1U;
             otherPassageIndex < allPassages.size();
             ++otherPassageIndex)
        {
            semantic::Passage *p_second = allPassages[otherPassageIndex];
            bool               secondIsBad{};
            if (!(p_second == nullptr) &&
                p_second->isBad(secondIsBad) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_second == nullptr || secondIsBad)
            {
                continue;
            }

            /* Must be the same physical wall's opening: near-coplanar
             * passage equations (parallel normals, matching offset once
             * consistently oriented). This deliberately reuses the same
             * kind of alignment/offset gates updatePassages()'s own
             * duplicate-detection uses, applied here across ALL existing
             * passages rather than only against fresh detection candidates. */
            g2o::Plane3D firstGlobalEquation{};
            if (p_first->getGlobalEquation(firstGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            Eigen::Vector4d firstEquation_World = firstGlobalEquation.coeffs();
            g2o::Plane3D    secondGlobalEquation{};
            if (p_second->getGlobalEquation(secondGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            Eigen::Vector4d secondEquation_World =
                secondGlobalEquation.coeffs();
            const double firstNormalNorm = firstEquation_World.head<3>().norm();
            const double secondNormalNorm =
                secondEquation_World.head<3>().norm();
            if (!firstEquation_World.allFinite() ||
                !secondEquation_World.allFinite() || firstNormalNorm < 1e-8 ||
                secondNormalNorm < 1e-8)
            {
                continue;
            }
            firstEquation_World /= firstNormalNorm;
            secondEquation_World /= secondNormalNorm;

            constexpr double minimumCoplanarNormalAlignment = 0.90;
            constexpr double maximumCoplanarOffset_m        = 0.30;
            const double normalAlignment = firstEquation_World.head<3>().dot(
                secondEquation_World.head<3>());
            if (std::abs(normalAlignment) < minimumCoplanarNormalAlignment)
            {
                continue;
            }
            Eigen::Vector4d orientedSecondEquation_World = secondEquation_World;
            if (normalAlignment < 0.0)
            {
                orientedSecondEquation_World = -orientedSecondEquation_World;
            }
            if (std::abs(firstEquation_World(3) -
                         orientedSecondEquation_World(3)) >
                maximumCoplanarOffset_m)
            {
                continue;
            }

            /* Shared 2D basis in the wall's own plane: horizontal tangent
             * (ground normal x wall normal) for width, ground normal for
             * height -- same construction used for the ground-aligned wall
             * admission gate. */
            Eigen::Vector3d axisU_World =
                groundNormal_World.cross(firstEquation_World.head<3>());
            const double axisUNorm = axisU_World.norm();
            if (axisUNorm < 1e-3)
            {
                continue;
            }
            axisU_World /= axisUNorm;
            const Eigen::Vector3d &axisV_World = groundNormal_World;

            Eigen::Vector3d firstCentroid_World{};
            if (p_first->getCentroid(firstCentroid_World) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            Eigen::Vector3d secondCentroid_World{};
            if (p_second->getCentroid(secondCentroid_World) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            if (!firstCentroid_World.allFinite() ||
                !secondCentroid_World.allFinite())
            {
                continue;
            }

            const double firstU  = firstCentroid_World.dot(axisU_World);
            const double firstV  = firstCentroid_World.dot(axisV_World);
            const double secondU = secondCentroid_World.dot(axisU_World);
            const double secondV = secondCentroid_World.dot(axisV_World);

            double firstWidth{};
            if (p_first->getWidth(firstWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getWidth cannot fail; continue as before.
            }
            double secondWidth{};
            if (p_second->getWidth(secondWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getWidth cannot fail; continue as before.
            }
            const double combinedHalfWidth_m = 0.5 * (firstWidth + secondWidth);
            double       firstHeight{};
            if (p_first->getHeight(firstHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getHeight cannot fail; continue as before.
            }
            double secondHeight{};
            if (p_second->getHeight(secondHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getHeight cannot fail; continue as before.
            }
            const double combinedHalfHeight_m =
                0.5 * (firstHeight + secondHeight);

            const bool overlapsInWidth =
                std::abs(firstU - secondU) < combinedHalfWidth_m;
            const bool overlapsInHeight =
                std::abs(firstV - secondV) < combinedHalfHeight_m;
            if (!overlapsInWidth || !overlapsInHeight)
            {
                continue;
            }

            int firstId{};
            if (p_first->getId(firstId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int secondId{};
            if (p_second->getId(secondId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            semantic::Passage *p_survivor =
                (firstId <= secondId) ? p_first : p_second;
            semantic::Passage *p_absorbed =
                (p_survivor == p_first) ? p_second : p_first;

            std::vector<vs_graphs::core::geometric::Plane *>
                absorbedAssociateWalls{};
            if (p_absorbed->getAssociateWalls(absorbedAssociateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getAssociateWalls cannot fail; continue as before.
            }
            for (geometric::Plane *p_wall : absorbedAssociateWalls)
            {
                if (p_survivor->addAssociateWall(p_wall) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // addAssociateWall cannot fail; continue as before.
                }
            }
            double survivorWidth{};
            if (p_survivor->getWidth(survivorWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getWidth cannot fail; continue as before.
            }
            double absorbedWidth{};
            if (p_absorbed->getWidth(absorbedWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getWidth cannot fail; continue as before.
            }
            if (p_survivor->setWidth(std::max(survivorWidth, absorbedWidth)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setWidth cannot fail; continue as before.
            }
            double survivorHeight{};
            if (p_survivor->getHeight(survivorHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getHeight cannot fail; continue as before.
            }
            double absorbedHeight{};
            if (p_absorbed->getHeight(absorbedHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getHeight cannot fail; continue as before.
            }
            if (p_survivor->setHeight(
                    std::max(survivorHeight, absorbedHeight)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setHeight cannot fail; continue as before.
            }
            bool survivorIsPassable{};
            if (p_survivor->isPassable(survivorIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // isPassable cannot fail; continue as before.
            }
            bool absorbedIsPassable{};
            if (!(survivorIsPassable) &&
                p_absorbed->isPassable(absorbedIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // isPassable cannot fail; continue as before.
            }
            if (p_survivor->setPassable(survivorIsPassable ||
                                        absorbedIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setPassable cannot fail; continue as before.
            }
            semantic::Passage::KnownSideProvenance
                absorbedKnownSideProvenance{};
            if (p_absorbed->getKnownSideProvenance(
                    absorbedKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getKnownSideProvenance cannot fail; continue as before.
            }
            if (p_survivor->mergeKnownSideProvenance(
                    absorbedKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // mergeKnownSideProvenance cannot fail; continue as before.
            }
            bool survivorHasProspectiveRoom{};
            if (p_survivor->hasProspectiveRoom(survivorHasProspectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // hasProspectiveRoom cannot fail; continue as before.
            }
            bool absorbedHasProspectiveRoom{};
            if ((!survivorHasProspectiveRoom) &&
                p_absorbed->hasProspectiveRoom(absorbedHasProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // hasProspectiveRoom cannot fail; continue as before.
            }
            if (!survivorHasProspectiveRoom && absorbedHasProspectiveRoom)
            {
                vs_graphs::core::semantic::Room *p_absorbedProspectiveRoom =
                    nullptr;
                if (p_absorbed->getProspectiveRoom(p_absorbedProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getProspectiveRoom cannot fail; continue as before.
                }
                if (p_survivor->setProspectiveRoom(p_absorbedProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setProspectiveRoom cannot fail; continue as before.
                }
            }

            int survivorId{};
            if (p_survivor->getId(survivorId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int absorbedId{};
            if (p_absorbed->getId(absorbedId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (loggedPassageMergeIds.insert({survivorId, absorbedId}).second)
            {
                int absorbedId2{};
                if (p_absorbed->getId(absorbedId2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int survivorId2{};
                if (p_survivor->getId(survivorId2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int survivorId3{};
                if (p_survivor->getId(survivorId3) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemMgr] semantic::Passage#" << absorbedId2
                          << " overlaps semantic::Passage#" << survivorId2
                          << " in their shared wall's 2D plane; merged "
                             "evidence into semantic::Passage#"
                          << survivorId3 << "." << std::endl;
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
