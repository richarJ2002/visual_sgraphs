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
 * @file            detectPassagesFromFreeSpace.cc
 *
 * @brief           Implements SemanticsManager::detectPassagesFromFreeSpace(),
 *                  declared in SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "../private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::detectPassagesFromFreeSpace(
    const std::vector<geometric::Plane *> &wallPlanes_in)
{
    const types::SystemParams::SemSeg::PassageDetection &passageParameters =
        p_sysParams->semSeg.passageDetection;
    const double duplicatePassageDistance_m =
        static_cast<double>(passageParameters.duplicatePassageDistance_m);
    const double ambiguousNormalAlignment = static_cast<double>(
        passageParameters.ambiguousDuplicateNormalAlignment);
    const double ambiguousPlaneSeparation_m = static_cast<double>(
        passageParameters.ambiguousDuplicatePlaneSeparation_m);
    const double planeMatchAlignment =
        std::cos(static_cast<double>(passageParameters.planeMatchAngle_deg) *
                 M_PI / 180.0);
    const double planeMatchOffset_m =
        static_cast<double>(passageParameters.planeMatchOffset_m);

    /* Each Voxblox answer is used once. */
    std::vector<WallSlabProfile> freshProfiles{};
    {
        std::lock_guard<std::mutex> wallSlabLock(wallSlabMutex);
        if (!wallSlabProfiles.empty() &&
            wallSlabProfiles.front().profileStamp_ns >
                lastUsedSlabProfileStamp_ns)
        {
            freshProfiles = wallSlabProfiles;
        }
    }
    if (freshProfiles.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    lastUsedSlabProfileStamp_ns = freshProfiles.front().profileStamp_ns;

    /* Unit floor normal; zero when there is no usable floor yet. */
    Eigen::Vector3d   groundNormal_world = Eigen::Vector3d::Zero();
    geometric::Plane *p_groundPlane      = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getBiggestGroundPlane returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    bool groundPlaneIsBad = true;
    if (p_groundPlane != nullptr &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundEquation{};
        if (p_groundPlane->getGlobalEquation(groundEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getGlobalEquation returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        const Eigen::Vector3d groundNormal = groundEquation.coeffs().head<3>();
        if (groundNormal.allFinite() && groundNormal.norm() > 1e-8)
        {
            groundNormal_world = groundNormal.normalized();
        }
    }
    const auto horizontalDistance_m =
        [&groundNormal_world](const Eigen::Vector3d &first_world_m,
                              const Eigen::Vector3d &second_world_m)
    {
        Eigen::Vector3d difference = first_world_m - second_world_m;
        difference -= difference.dot(groundNormal_world) * groundNormal_world;
        return difference.norm();
    };

    std::vector<semantic::Room *> allRooms{};
    if (p_atlas->getAllRooms(allRooms) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::unordered_map<int, geometric::Plane *> wallsById{};
    for (geometric::Plane *p_wall : wallPlanes_in)
    {
        int wallId{-1};
        if (p_wall != nullptr &&
            p_wall->getId(wallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall != nullptr)
        {
            wallsById[wallId] = p_wall;
        }
    }

    std::vector<PendingOpening> keptPendingOpenings{};
    std::unordered_set<int>     measuredWallIds{};

    for (const WallSlabProfile &wallProfile : freshProfiles)
    {
        const auto wallEntry = wallsById.find(wallProfile.query.wallId);
        if (wallEntry == wallsById.end())
        {
            continue;
        }
        geometric::Plane *p_wall = wallEntry->second;

        /* The slab is only valid while the wall has not moved. */
        g2o::Plane3D wallGlobalEquation{};
        if (p_wall->getGlobalEquation(wallGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getGlobalEquation returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Vector4d wallEquation_world = wallGlobalEquation.coeffs();
        const double    wallNormalNorm = wallEquation_world.head<3>().norm();
        if (!wallEquation_world.allFinite() || wallNormalNorm < 1e-8)
        {
            continue;
        }
        wallEquation_world /= wallNormalNorm;
        Eigen::Vector4d queryEquation_world =
            wallProfile.query.wallEquation_world;
        if (queryEquation_world.head<3>().dot(wallEquation_world.head<3>()) <
            0.0)
        {
            queryEquation_world = -queryEquation_world;
        }
        if (queryEquation_world.head<3>().dot(wallEquation_world.head<3>()) <
                planeMatchAlignment ||
            std::abs(queryEquation_world(3) - wallEquation_world(3)) >
                planeMatchOffset_m)
        {
            continue;
        }
        measuredWallIds.insert(wallProfile.query.wallId);

        std::vector<WallGap> wallGaps{};
        if (measureWallGap(wallProfile, passageParameters, wallGaps) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: measureWallGap returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }

        semantic::Room *p_wallRoom = nullptr;
        if (findWallRoom(allRooms, p_wall, p_wallRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: findWallRoom returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }

        std::unordered_map<semantic::Passage *, WallGap> openGapsByPassage{};
        for (const WallGap &wallGap : wallGaps)
        {
            std::vector<semantic::Passage *> allPassages{};
            if (p_atlas->getAllPassages(allPassages) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getAllPassages returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }

            /* The nearest parallel passage at the gap is the same opening. */
            semantic::Passage *p_matchingPassage  = nullptr;
            double             matchingDistance_m = duplicatePassageDistance_m;
            for (semantic::Passage *p_passage : allPassages)
            {
                bool passageIsBad = true;
                if (p_passage != nullptr &&
                    p_passage->isBad(passageIsBad) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isBad returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_passage == nullptr || passageIsBad)
                {
                    continue;
                }
                Eigen::Vector3d passageCentroid_world_m{};
                g2o::Plane3D    passageEquation{};
                if (p_passage->getCentroid(passageCentroid_world_m) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                    p_passage->getGlobalEquation(passageEquation) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getCentroid/getGlobalEquation returned a "
                                 "failure status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                const Eigen::Vector3d passageNormal =
                    passageEquation.coeffs().head<3>();
                const double gapDistance_m =
                    horizontalDistance_m(passageCentroid_world_m,
                                         wallGap.centre_world_m);
                if (!passageCentroid_world_m.allFinite() ||
                    !passageNormal.allFinite() || passageNormal.norm() < 1e-8 ||
                    gapDistance_m > matchingDistance_m ||
                    std::abs(passageNormal.normalized().dot(
                        wallEquation_world.head<3>())) <
                        ambiguousNormalAlignment)
                {
                    continue;
                }
                p_matchingPassage  = p_passage;
                matchingDistance_m = gapDistance_m;
            }

            if (p_matchingPassage != nullptr)
            {
                std::vector<geometric::Plane *> passageWalls{};
                if (p_matchingPassage->getAssociateWalls(passageWalls) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getAssociateWalls returned a failure "
                                 "status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }

                /* The first face anchors the passage and its verdicts. */
                if (!passageWalls.empty() && passageWalls.front() == p_wall)
                {
                    openGapsByPassage[p_matchingPassage] = wallGap;
                    continue;
                }
                if (passageWalls.empty() ||
                    std::find(passageWalls.begin(),
                              passageWalls.end(),
                              p_wall) != passageWalls.end() ||
                    passageWalls.size() >= 2U)
                {
                    continue;
                }

                /* Attach the opposite face of the same physical wall. */
                geometric::Plane *p_anchorWall = passageWalls.front();
                bool              isTwinFace   = false;
                TwinFaceCheck     twinFaceCheck{};
                double            twinFaceValue{};
                if (arePlausibleTwinWallFaces(
                        p_wall,
                        p_anchorWall,
                        static_cast<double>(
                            p_sysParams->semSeg.wallPairing.minimumThickness_m),
                        static_cast<double>(
                            p_sysParams->semSeg.wallPairing.maximumThickness_m),
                        static_cast<double>(p_sysParams->semSeg.wallPairing
                                                .minimumOverlapRatio),
                        groundNormal_world,
                        isTwinFace,
                        twinFaceCheck,
                        twinFaceValue) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: arePlausibleTwinWallFaces returned a "
                                 "failure status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                g2o::Plane3D anchorEquation{};
                if (p_anchorWall->getGlobalEquation(anchorEquation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getGlobalEquation returned a failure "
                                 "status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                const Eigen::Vector4d anchorEquation_world =
                    anchorEquation.coeffs() /
                    std::max(anchorEquation.coeffs().head<3>().norm(), 1e-8);
                const bool isAmbiguousSeparation =
                    std::abs(anchorEquation_world.head<3>().dot(
                        wallEquation_world.head<3>())) >=
                        ambiguousNormalAlignment &&
                    std::abs(anchorEquation_world.head<3>().dot(
                                 wallGap.centre_world_m) +
                             anchorEquation_world(3)) <=
                        ambiguousPlaneSeparation_m;
                semantic::Room *p_anchorRoom = nullptr;
                if (findWallRoom(allRooms, p_anchorWall, p_anchorRoom) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: findWallRoom returned a failure status "
                                 "although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                const bool isSameRoom =
                    p_wallRoom != nullptr && p_wallRoom == p_anchorRoom;
                if ((isTwinFace || isAmbiguousSeparation) && !isSameRoom &&
                    p_matchingPassage->addAssociateWall(p_wall) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: addAssociateWall returned a failure "
                                 "status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                continue;
            }

            /* A new opening must be OPEN in consecutive profiles first. */
            PendingOpening pendingOpening{};
            pendingOpening.wallId         = wallProfile.query.wallId;
            pendingOpening.centre_world_m = wallGap.centre_world_m;
            for (const PendingOpening &previousOpening : pendingOpenings)
            {
                if (previousOpening.wallId == pendingOpening.wallId &&
                    horizontalDistance_m(previousOpening.centre_world_m,
                                         wallGap.centre_world_m) <=
                        duplicatePassageDistance_m)
                {
                    pendingOpening.consecutiveOpenVerdicts =
                        previousOpening.consecutiveOpenVerdicts;
                    break;
                }
            }
            pendingOpening.consecutiveOpenVerdicts++;
            if (pendingOpening.consecutiveOpenVerdicts <
                passageParameters.minimumOpenVerdicts)
            {
                keptPendingOpenings.push_back(pendingOpening);
                continue;
            }

            /*
             * No prospective room here: associatePassagesToRooms() decides it
             * once the room count of the passage is known.
             */
            semantic::Passage *p_createdPassage = nullptr;
            if (utils::geo_sem_helpers::GeoSemHelpers::createMapPassage(
                    p_atlas,
                    nullptr,
                    p_wall,
                    p_createdPassage,
                    true,
                    wallGap.centre_world_m) !=
                utils::geo_sem_helpers::GeoSemHelpersStatus::
                    GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createMapPassage returned a failure status "
                             "although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_createdPassage != nullptr &&
                (p_createdPassage->setWidth(wallGap.width_m) !=
                     semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                 p_createdPassage->setHeight(wallGap.topHeight_m) !=
                     semantic::PassageStatus::PASSAGE_STATUS_SUCCESS))
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWidth/setHeight returned a failure status "
                             "although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
        }

        if (applyPassageVerdicts(p_wall, wallProfile, openGapsByPassage) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyPassageVerdicts returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* A measured wall without the opening resets it; others keep waiting. */
    for (const PendingOpening &previousOpening : pendingOpenings)
    {
        if (measuredWallIds.count(previousOpening.wallId) == 0U &&
            wallsById.count(previousOpening.wallId) != 0U)
        {
            keptPendingOpenings.push_back(previousOpening);
        }
    }
    pendingOpenings = std::move(keptPendingOpenings);

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
