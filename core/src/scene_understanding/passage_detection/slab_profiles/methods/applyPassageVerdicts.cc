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
 * @file            applyPassageVerdicts.cc
 *
 * @brief           Implements SemanticsManager::applyPassageVerdicts(),
 *                  declared in SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/passage_detection/slab_profiles/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <rclcpp/logging.hpp>

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::applyPassageVerdicts(
    geometric::Plane                                       *p_wall_in,
    const WallSlabProfile                                  &wallProfile_in,
    const std::unordered_map<semantic::Passage *, WallGap> &openGaps_in)
{
    const types::SystemParams::SemSeg::PassageDetection &passageParameters =
        p_sysParams->semSeg.passageDetection;
    const WallSlabQuery &wallQuery = wallProfile_in.query;
    const double         doorBandMinimum_m =
        static_cast<double>(passageParameters.doorBandMinHeight_m);
    const double doorBandMaximum_m =
        static_cast<double>(passageParameters.doorBandMaxHeight_m);
    /*
     * Wall points are counted in the central half of the aperture only: the
     * measured gap can be a TSDF voxel wider than the door, and the jamb
     * faces and wall edges must never count as a closed door.
     */
    constexpr double apertureShrinkFraction = 0.25;

    std::vector<semantic::Passage *> allPassages{};
    if (getAllPassagesInIdOrder(allPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    geometric::Plane::GeometrySnapshot wallGeometry{};
    if (p_wall_in->getGeometrySnapshot(wallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getGeometrySnapshot returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }

    for (semantic::Passage *p_passage : allPassages)
    {
        bool passageIsBad = true;
        if (p_passage != nullptr &&
            p_passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> passageWalls{};
        if (p_passage != nullptr && !passageIsBad &&
            p_passage->getAssociateWalls(passageWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAssociateWalls returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        /* Only the anchoring face of a passage gives its verdicts. */
        if (p_passage == nullptr || passageIsBad || passageWalls.empty() ||
            passageWalls.front() != p_wall_in)
        {
            continue;
        }

        Eigen::Vector3d passageCentroid_world_m{};
        double          passageWidth_m{};
        int             passageId{};
        if (p_passage->getCentroid(passageCentroid_world_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage->getWidth(passageWidth_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid/getWidth/getId returned a failure "
                         "status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Aperture in slab coordinates: the measured gap, else the passage. */
        const std::unordered_map<semantic::Passage *, WallGap>::const_iterator
                     openGap = openGaps_in.find(p_passage);
        const double passageAlong_m =
            (passageCentroid_world_m - wallQuery.origin_world_m)
                .dot(wallQuery.alongAxis_world);
        const double apertureStart_m =
            openGap != openGaps_in.end()
                ? openGap->second.alongStart_m
                : passageAlong_m - 0.5 * passageWidth_m;
        const double apertureEnd_m =
            openGap != openGaps_in.end()
                ? openGap->second.alongEnd_m
                : passageAlong_m + 0.5 * passageWidth_m;

        enum class PassageVerdict
        {
            NO_EVIDENCE,
            OPEN,
            CLOSED
        };
        PassageVerdict passageVerdict        = PassageVerdict::CLOSED;
        std::size_t    apertureFreeCellCount = 0U, apertureBandCellCount = 0U;
        if (computeObservedApertureFraction(wallProfile_in,
                                            doorBandMinimum_m,
                                            doorBandMaximum_m,
                                            apertureStart_m,
                                            apertureEnd_m,
                                            apertureFreeCellCount,
                                            apertureBandCellCount) <
            static_cast<double>(passageParameters.minimumObservedFraction))
        {
            passageVerdict = PassageVerdict::NO_EVIDENCE;
        }
        else
        {
            const std::size_t apertureWallPoints = countApertureWallPoints(
                wallGeometry.supportCloud,
                wallProfile_in,
                doorBandMinimum_m,
                doorBandMaximum_m,
                apertureStart_m +
                    apertureShrinkFraction * (apertureEnd_m - apertureStart_m),
                apertureEnd_m -
                    apertureShrinkFraction * (apertureEnd_m - apertureStart_m));
            /*
             * Only wall points in the aperture close it. Without them and
             * without a measured gap (the free space ran past the wall's end,
             * merged with a neighbouring opening, or one side is still
             * unobserved) the profile says nothing about the door.
             */
            const bool hasNoWallPoints =
                apertureWallPoints <
                passageParameters.maximumApertureWallPoints;
            if (openGap != openGaps_in.end() && hasNoWallPoints)
            {
                passageVerdict = PassageVerdict::OPEN;
            }
            else if (hasNoWallPoints)
            {
                passageVerdict = PassageVerdict::NO_EVIDENCE;
            }
            else
            {
                std::cout
                    << "SG_PIPELINE {\"event\":\"passage_closed_verdict\","
                       "\"passage_id\":"
                    << passageId << ",\"gap\":"
                    << (openGap != openGaps_in.end() ? "true" : "false")
                    << ",\"wall_points\":" << apertureWallPoints
                    << ",\"free_cells\":" << apertureFreeCellCount
                    << ",\"band_cells\":" << apertureBandCellCount << "}"
                    << std::endl;
            }
        }
        if (passageVerdict == PassageVerdict::NO_EVIDENCE)
        {
            continue;
        }

        std::size_t  consecutiveOpenVerdicts{}, consecutiveClosedVerdicts{};
        std::int64_t lastVerdictStamp_ns{};
        bool         passageIsBlocked{}, passageIsTraversed{};
        if (p_passage->getVerdictCounters(consecutiveOpenVerdicts,
                                          consecutiveClosedVerdicts,
                                          lastVerdictStamp_ns) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage->isBlocked(passageIsBlocked) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage->getTraversalEvidence(passageIsTraversed) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getVerdictCounters/isBlocked/getTraversalEvidence "
                "returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        if (passageVerdict == PassageVerdict::OPEN)
        {
            consecutiveOpenVerdicts++;
            consecutiveClosedVerdicts = 0U;
            /* The plane stays anchored on the first face; only size and
             * centre follow the measured gap. */
            if (p_passage->setWidth(openGap->second.width_m) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                p_passage->setHeight(openGap->second.topHeight_m) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                p_passage->setCentroid(openGap->second.centre_world_m) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWidth/setHeight/setCentroid returned a "
                             "failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (passageIsBlocked &&
                consecutiveOpenVerdicts >=
                    passageParameters.reopenConfirmationCount)
            {
                if (p_passage->setBlocked(false) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setBlocked returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                std::cout << "[SemMgr] Passage#" << passageId
                          << " unblocked: open again for "
                          << consecutiveOpenVerdicts << " profiles."
                          << std::endl;
            }
        }
        else
        {
            consecutiveClosedVerdicts++;
            consecutiveOpenVerdicts = 0U;
            if (consecutiveClosedVerdicts >=
                passageParameters.closedConfirmationCount)
            {
                /* A crossed passage stays in the graph, only blocked; an
                 * uncrossed one was never confirmed and is retired, but not
                 * while the UAV is at it and may be about to cross. */
                const bool uavIsAtPassage =
                    hasCameraCenter &&
                    (currentCameraCenter_world_m - passageCentroid_world_m)
                            .norm() <
                        static_cast<double>(
                            p_sysParams->semSeg.maxKfPassageDistance);
                if (passageIsTraversed && !passageIsBlocked)
                {
                    if (p_passage->setBlocked(true) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: setBlocked returned a failure status "
                                     "although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    std::cout << "[SemMgr] Passage#" << passageId
                              << " blocked: closed for "
                              << consecutiveClosedVerdicts << " profiles."
                              << std::endl;
                }
                else if (!passageIsTraversed && !uavIsAtPassage)
                {
                    if (p_passage->setBad() !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Passage#" << passageId
                              << " retired: closed for "
                              << consecutiveClosedVerdicts
                              << " profiles before any crossing." << std::endl;
                    continue;
                }
            }
        }
        if (p_passage->setVerdictCounters(consecutiveOpenVerdicts,
                                          consecutiveClosedVerdicts,
                                          wallProfile_in.profileStamp_ns) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setVerdictCounters returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
