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
        if (p_first == nullptr || p_first->isBad())
        {
            continue;
        }

        for (std::size_t otherPassageIndex = allPassageIndex + 1U;
             otherPassageIndex < allPassages.size();
             ++otherPassageIndex)
        {
            semantic::Passage *p_second = allPassages[otherPassageIndex];
            if (p_second == nullptr || p_second->isBad())
            {
                continue;
            }

            /* Must be the same physical wall's opening: near-coplanar
             * passage equations (parallel normals, matching offset once
             * consistently oriented). This deliberately reuses the same
             * kind of alignment/offset gates updatePassages()'s own
             * duplicate-detection uses, applied here across ALL existing
             * passages rather than only against fresh detection candidates. */
            Eigen::Vector4d firstEquation_World =
                p_first->getGlobalEquation().coeffs();
            Eigen::Vector4d secondEquation_World =
                p_second->getGlobalEquation().coeffs();
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

            const Eigen::Vector3d firstCentroid_World = p_first->getCentroid();
            const Eigen::Vector3d secondCentroid_World =
                p_second->getCentroid();
            if (!firstCentroid_World.allFinite() ||
                !secondCentroid_World.allFinite())
            {
                continue;
            }

            const double firstU  = firstCentroid_World.dot(axisU_World);
            const double firstV  = firstCentroid_World.dot(axisV_World);
            const double secondU = secondCentroid_World.dot(axisU_World);
            const double secondV = secondCentroid_World.dot(axisV_World);

            const double combinedHalfWidth_m =
                0.5 * (p_first->getWidth() + p_second->getWidth());
            const double combinedHalfHeight_m =
                0.5 * (p_first->getHeight() + p_second->getHeight());

            const bool overlapsInWidth =
                std::abs(firstU - secondU) < combinedHalfWidth_m;
            const bool overlapsInHeight =
                std::abs(firstV - secondV) < combinedHalfHeight_m;
            if (!overlapsInWidth || !overlapsInHeight)
            {
                continue;
            }

            semantic::Passage *p_survivor =
                (p_first->getId() <= p_second->getId()) ? p_first : p_second;
            semantic::Passage *p_absorbed =
                (p_survivor == p_first) ? p_second : p_first;

            for (geometric::Plane *p_wall : p_absorbed->getAssociateWalls())
            {
                p_survivor->addAssociateWall(p_wall);
            }
            p_survivor->setWidth(
                std::max(p_survivor->getWidth(), p_absorbed->getWidth()));
            p_survivor->setHeight(
                std::max(p_survivor->getHeight(), p_absorbed->getHeight()));
            p_survivor->setPassable(p_survivor->isPassable() ||
                                    p_absorbed->isPassable());
            p_survivor->mergeKnownSideProvenance(
                p_absorbed->getKnownSideProvenance());
            if (!p_survivor->hasProspectiveRoom() &&
                p_absorbed->hasProspectiveRoom())
            {
                p_survivor->setProspectiveRoom(
                    p_absorbed->getProspectiveRoom());
            }

            if (loggedPassageMergeIds
                    .insert({p_survivor->getId(), p_absorbed->getId()})
                    .second)
            {
                std::cout << "[SemMgr] semantic::Passage#"
                          << p_absorbed->getId()
                          << " overlaps semantic::Passage#"
                          << p_survivor->getId()
                          << " in their shared wall's 2D plane; merged "
                             "evidence into semantic::Passage#"
                          << p_survivor->getId() << "." << std::endl;
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
