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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::enforcePassageSideInvariant(void)
{
    /* "Continuously checking the current state of the sgraph to make sure
     * the rules are followed" (as opposed to only at the moment a wall is
     * newly admitted): associateAllWallsToRooms() only ever revisits ORPHAN
     * walls (a wall that already has a room is skipped outright), so a wall
     * admitted before a relevant passage's aperture became confidently
     * resolvable would otherwise never be re-examined again. This sweep
     * re-applies both the passage-aperture backstop and the wall-face
     * ownership rule to every wall every room currently owns, every cycle.
     *
     * Note the two are re-checked here for different reasons.
     * isWallFaceForeignToRoom() is itself stable -- face identity is stamped
     * at observation and does not drift -- but the ROOM side of the
     * comparison does move: a FREE_SPACE room's centroid is recomputed every
     * cycle as its wall-centroid mean, so a room that grows walls can
     * migrate across a face it once legitimately sat beside. The aperture
     * backstop is re-checked because passage geometry itself sharpens over
     * time. */
    geometric::Plane *p_groundPlane      = p_atlas->getBiggestGroundPlane();
    Eigen::Vector3d   groundNormal_World = Eigen::Vector3d::Zero();
    bool              groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEq = groundPlaneGetGlobalEquation.coeffs();
        const double          groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormal_World = groundEq.head<3>() / groundNorm;
        }
    }

    const std::vector<semantic::Passage *> allPassages =
        p_atlas->getAllPassages();

    for (semantic::Room *p_room : p_atlas->getAllRooms())
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        /* Copy: both backstops below may call Room::removeWall(), which
         * would invalidate an in-progress iteration over the room's own
         * live wall vector. */
        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls)
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

            /* Prospective-placement exemption: a far-side wall that the
             * aperture backstop deliberately routed into this prospective
             * room must not be evicted from it by the face check below
             * (live-observed churn: remove-then-reroute every cycle). The
             * exemption is earned only when the same aperture test that
             * routes the wall still places it here, synthesized from the
             * passage's known near-side direction exactly as the backstop
             * does. */
            bool wallRoutedToProspective = false;
            for (semantic::Passage *p_exemptPassage : allPassages)
            {
                bool exemptPassageIsBad{};
                if (!(p_exemptPassage == nullptr) &&
                    p_exemptPassage->isBad(exemptPassageIsBad) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                vs_graphs::core::semantic::Room
                    *p_exemptPassageProspectiveRoom = nullptr;
                if (!(p_exemptPassage == nullptr || exemptPassageIsBad) &&
                    p_exemptPassage->getProspectiveRoom(
                        p_exemptPassageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_exemptPassage == nullptr || exemptPassageIsBad ||
                    p_exemptPassageProspectiveRoom != p_room)
                {
                    continue;
                }
                semantic::Passage::KnownSideProvenance knownSide{};
                if (p_exemptPassage->getKnownSideProvenance(knownSide) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                bool knownSideHasDirection{};
                if (knownSide.hasDirection(knownSideHasDirection) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (!knownSideHasDirection)
                {
                    continue;
                }
                const double minimumSideDistance_m =
                    static_cast<double>(p_sysParams->roomSeg.passagePartition
                                            .minimumSideDistance_m);
                Eigen::Vector3d exemptPassageCentroid{};
                if (p_exemptPassage->getCentroid(exemptPassageCentroid) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                const Eigen::Vector3d knownSidePoint_World_m =
                    exemptPassageCentroid +
                    (minimumSideDistance_m * 2.0) * knownSide.direction_World;
                Eigen::Vector3d wallGetCentroid{};
                if (p_wall->getCentroid(wallGetCentroid) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (segmentCrossesPassageOpening(
                        knownSidePoint_World_m,
                        wallGetCentroid.cast<double>(),
                        p_exemptPassage,
                        groundNormal_World,
                        static_cast<double>(
                            p_sysParams->roomSeg.passagePartition
                                .openingMargin_m),
                        minimumSideDistance_m))
                {
                    wallRoutedToProspective = true;
                    break;
                }
            }
            if (wallRoutedToProspective)
            {
                continue;
            }

            if (isWallFaceForeignToRoom(p_room, p_wall))
            {
                bool roomWasWallRemoved{};
                if (p_room->removeWall(p_wall, roomWasWallRemoved) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    roomWasWallRemoved = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: removeWall rejected its input; continuing "
                                "as before.",
                                __func__);
                }
                int roomId{};
                if (p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int wallGetId{};
                if (p_wall->getId(wallGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] Wall#" << wallGetId
                          << " removed from semantic::Room#" << roomId
                          << ": this face was observed from the opposite side, "
                             "so it bounds the neighbouring room."
                          << std::endl;
                continue;
            }

            if (!allPassages.empty())
            {
                enforcePassageApertureBackstop(p_room,
                                               p_wall,
                                               allPassages,
                                               groundNormal_World);
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
