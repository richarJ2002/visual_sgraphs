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
 * @file            mergeOverlappingPassages.cc
 *
 * @brief           Implements SemanticsManager::mergeOverlappingPassages(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::mergeOverlappingPassages(void)
{
    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool groundPlaneIsBad{};
    if (!(p_groundPlane == nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane == nullptr || groundPlaneIsBad)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    g2o::Plane3D groundPlaneGetGlobalEquation{};
    if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector4d groundEq   = groundPlaneGetGlobalEquation.coeffs();
    const double          groundNorm = groundEq.head<3>().norm();
    if (!groundEq.allFinite() || groundNorm < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    const Eigen::Vector3d groundNormal_world = groundEq.head<3>() / groundNorm;

    std::vector<semantic::Passage *> allPassages{};
    if (p_atlas->getAllPassages(allPassages) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (std::size_t allPassageIndex = 0U; allPassageIndex < allPassages.size();
         ++allPassageIndex)
    {
        semantic::Passage *p_first = allPassages[allPassageIndex];
        bool               firstIsBad{};
        if (!(p_first == nullptr) &&
            p_first->isBad(firstIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d firstEquation_world = firstGlobalEquation.coeffs();
            g2o::Plane3D    secondGlobalEquation{};
            if (p_second->getGlobalEquation(secondGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d secondEquation_world =
                secondGlobalEquation.coeffs();
            const double firstNormalNorm = firstEquation_world.head<3>().norm();
            const double secondNormalNorm =
                secondEquation_world.head<3>().norm();
            if (!firstEquation_world.allFinite() ||
                !secondEquation_world.allFinite() || firstNormalNorm < 1e-8 ||
                secondNormalNorm < 1e-8)
            {
                continue;
            }
            firstEquation_world /= firstNormalNorm;
            secondEquation_world /= secondNormalNorm;

            constexpr double minimumCoplanarNormalAlignment = 0.90;
            constexpr double maximumCoplanarOffset_m        = 0.30;
            const double normalAlignment = firstEquation_world.head<3>().dot(
                secondEquation_world.head<3>());
            if (std::abs(normalAlignment) < minimumCoplanarNormalAlignment)
            {
                continue;
            }
            Eigen::Vector4d orientedSecondEquation_world = secondEquation_world;
            if (normalAlignment < 0.0)
            {
                orientedSecondEquation_world = -orientedSecondEquation_world;
            }
            if (std::abs(firstEquation_world(3) -
                         orientedSecondEquation_world(3)) >
                maximumCoplanarOffset_m)
            {
                continue;
            }

            /* Shared 2D basis in the wall's own plane: horizontal tangent
             * (ground normal x wall normal) for width, ground normal for
             * height -- same construction used for the ground-aligned wall
             * admission gate. */
            Eigen::Vector3d axisU_world =
                groundNormal_world.cross(firstEquation_world.head<3>());
            const double axisUNorm = axisU_world.norm();
            if (axisUNorm < 1e-3)
            {
                continue;
            }
            axisU_world /= axisUNorm;
            const Eigen::Vector3d &axisV_world = groundNormal_world;

            Eigen::Vector3d firstCentroid_world{};
            if (p_first->getCentroid(firstCentroid_world) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d secondCentroid_world{};
            if (p_second->getCentroid(secondCentroid_world) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!firstCentroid_world.allFinite() ||
                !secondCentroid_world.allFinite())
            {
                continue;
            }

            const double firstU  = firstCentroid_world.dot(axisU_world);
            const double firstV  = firstCentroid_world.dot(axisV_world);
            const double secondU = secondCentroid_world.dot(axisU_world);
            const double secondV = secondCentroid_world.dot(axisV_world);

            double firstWidth{};
            if (p_first->getWidth(firstWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double secondWidth{};
            if (p_second->getWidth(secondWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            const double combinedHalfWidth_m = 0.5 * (firstWidth + secondWidth);
            double       firstHeight{};
            if (p_first->getHeight(firstHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double secondHeight{};
            if (p_second->getHeight(secondHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int secondId{};
            if (p_second->getId(secondId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : absorbedAssociateWalls)
            {
                if (p_survivor->addAssociateWall(p_wall) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addAssociateWall returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            double survivorWidth{};
            if (p_survivor->getWidth(survivorWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double absorbedWidth{};
            if (p_absorbed->getWidth(absorbedWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_survivor->setWidth(std::max(survivorWidth, absorbedWidth)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double survivorHeight{};
            if (p_survivor->getHeight(survivorHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double absorbedHeight{};
            if (p_absorbed->getHeight(absorbedHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_survivor->setHeight(
                    std::max(survivorHeight, absorbedHeight)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            bool survivorIsPassable{};
            if (p_survivor->isPassable(survivorIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool absorbedIsPassable{};
            if (!(survivorIsPassable) &&
                p_absorbed->isPassable(absorbedIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_survivor->setPassable(survivorIsPassable ||
                                        absorbedIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Passage::KnownSideProvenance
                absorbedKnownSideProvenance{};
            if (p_absorbed->getKnownSideProvenance(
                    absorbedKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_survivor->mergeKnownSideProvenance(
                    absorbedKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: mergeKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            bool survivorHasProspectiveRoom{};
            if (p_survivor->hasProspectiveRoom(survivorHasProspectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool absorbedHasProspectiveRoom{};
            if ((!survivorHasProspectiveRoom) &&
                p_absorbed->hasProspectiveRoom(absorbedHasProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!survivorHasProspectiveRoom && absorbedHasProspectiveRoom)
            {
                vs_graphs::core::semantic::Room *p_absorbedProspectiveRoom =
                    nullptr;
                if (p_absorbed->getProspectiveRoom(p_absorbedProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_survivor->setProspectiveRoom(p_absorbedProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            int survivorId{};
            if (p_survivor->getId(survivorId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int absorbedId{};
            if (p_absorbed->getId(absorbedId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (loggedPassageMergeIds.insert({survivorId, absorbedId}).second)
            {
                int absorbedId2{};
                if (p_absorbed->getId(absorbedId2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int survivorId2{};
                if (p_survivor->getId(survivorId2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int survivorId3{};
                if (p_survivor->getId(survivorId3) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] semantic::Passage#" << absorbedId2
                          << " overlaps semantic::Passage#" << survivorId2
                          << " in their shared wall's 2D plane; merged "
                             "evidence into semantic::Passage#"
                          << survivorId3 << "." << std::endl;
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
