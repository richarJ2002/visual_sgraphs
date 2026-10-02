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
 * @file            validateRoomBoundaries.cc
 *
 * @brief           Implements SemanticsManager::validateRoomBoundaries(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "../private_functions.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::validateRoomBoundaries(void)
{
    std::cout << "[SemMgr] validateRoomBoundaries() called" << std::endl;

    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters =
        p_sysParams->roomSeg.boundaryTopology;

    if (!topologyParameters.enabled)
    {
        std::cout << "[SemMgr] boundary topology disabled" << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

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
    Eigen::Vector4d groundEquation_world =
        groundPlaneGetGlobalEquation.coeffs();
    const double groundNormalNorm = groundEquation_world.head<3>().norm();

    if (!groundEquation_world.allFinite() || groundNormalNorm < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const Eigen::Vector3d groundNormal_world =
        groundEquation_world.head<3>() / groundNormalNorm;
    const Eigen::Vector3d groundAxisU_world =
        groundNormal_world.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_world =
        groundNormal_world.cross(groundAxisU_world).normalized();

    const auto updateBoundaryStatus =
        [](semantic::Room                      *p_room_in,
           const semantic::Room::BoundaryStatus boundaryStatus_in,
           const std::vector<Eigen::Vector3d>  &corners_world_m_in = {})
    {
        /* Refresh stored corners every cycle the loop is COMPLETE (even when
         * the status itself didn't change -- wall positions can still
         * drift), and clear them the moment it stops being COMPLETE. */
        if (p_room_in->setBoundaryCorners_world_m(
                boundaryStatus_in == semantic::Room::BoundaryStatus::COMPLETE
                    ? corners_world_m_in
                    : std::vector<Eigen::Vector3d>{}) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setBoundaryCorners_World_m returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        semantic::Room::BoundaryStatus previousBoundaryStatus{};
        if (p_room_in->getBoundaryStatus(previousBoundaryStatus) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBoundaryStatus returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (previousBoundaryStatus == boundaryStatus_in)
        {
            return;
        }

        if (p_room_in->setBoundaryStatus(boundaryStatus_in) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBoundaryStatus returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        const auto boundaryStatusName =
            [](const semantic::Room::BoundaryStatus status)
        {
            switch (status)
            {
            case semantic::Room::BoundaryStatus::UNOBSERVED:
                return "UNOBSERVED";
            case semantic::Room::BoundaryStatus::INCOMPLETE:
                return "INCOMPLETE";
            case semantic::Room::BoundaryStatus::COMPLETE:
                return "COMPLETE";
            case semantic::Room::BoundaryStatus::CONFLICTING:
                return "CONFLICTING";
            }

            return "unknown";
        };

        int room_inId{};
        if (p_room_in->getId(room_inId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> room_inWalls{};
        if (p_room_in->getWalls(room_inWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] semantic::Room#" << room_inId
                  << " boundary=" << boundaryStatusName(boundaryStatus_in)
                  << " (" << room_inWalls.size() << " walls)" << std::endl;
    };

    std::vector<semantic::Room *> atlasAllRooms{};
    if (p_atlas->getAllRooms(atlasAllRooms) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Room *p_room : atlasAllRooms)
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

        bool                             boundaryWasRepaired  = true;
        bool                             hasAmbiguousConflict = false;
        std::vector<FiniteWallSegment2d> wallSegments;

        while (boundaryWasRepaired)
        {
            boundaryWasRepaired  = false;
            hasAmbiguousConflict = false;
            wallSegments.clear();

            std::vector<geometric::Plane *> roomWalls2{};
            if (p_room->getWalls(roomWalls2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : roomWalls2)
            {
                FiniteWallSegment2d wallSegment;

                bool isBuilt{};
                if (buildFiniteWallSegment2d(
                        p_wall,
                        groundNormal_world,
                        groundAxisU_world,
                        groundAxisV_world,
                        topologyParameters.endpointTrimRatio,
                        topologyParameters.minimumWallLength_m,
                        wallSegment,
                        isBuilt) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: buildFiniteWallSegment2d returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (isBuilt)
                {
                    wallSegments.push_back(std::move(wallSegment));
                }
            }

            int roomId{};
            if (p_room->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::vector<geometric::Plane *> roomWalls3{};
            if (p_room->getWalls(roomWalls3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] semantic::Room#" << roomId
                      << " boundary check: roomWalls=" << roomWalls3.size()
                      << ", wallSegments=" << wallSegments.size()
                      << ", minWallCount="
                      << topologyParameters.minimumWallCount << std::endl;

            if (wallSegments.size() < topologyParameters.minimumWallCount)
            {
                updateBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::INCOMPLETE);
                continue;
            }

            for (std::size_t firstWallIndex = 0U;
                 firstWallIndex < wallSegments.size() && !boundaryWasRepaired &&
                 !hasAmbiguousConflict;
                 ++firstWallIndex)
            {
                for (std::size_t secondWallIndex = firstWallIndex + 1U;
                     secondWallIndex < wallSegments.size();
                     ++secondWallIndex)
                {
                    Eigen::Vector2d intersection_world_m;
                    double          firstParameter  = 0.0;
                    double          secondParameter = 0.0;

                    bool hasIntersection{};
                    if (intersectSupportingLines(wallSegments[firstWallIndex],
                                                 wallSegments[secondWallIndex],
                                                 intersection_world_m,
                                                 firstParameter,
                                                 secondParameter,
                                                 hasIntersection) !=
                        SemanticsManagerStatus::
                            SEMANTICS_MANAGER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: intersectSupportingLines returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    if (!hasIntersection || firstParameter < 0.0 ||
                        firstParameter > 1.0 || secondParameter < 0.0 ||
                        secondParameter > 1.0)
                    {
                        continue;
                    }

                    const double firstInteriorDistance_m =
                        std::min(firstParameter, 1.0 - firstParameter) *
                        wallSegments[firstWallIndex].length_m;
                    const double secondInteriorDistance_m =
                        std::min(secondParameter, 1.0 - secondParameter) *
                        wallSegments[secondWallIndex].length_m;

                    if (std::max(firstInteriorDistance_m,
                                 secondInteriorDistance_m) <=
                        topologyParameters.maximumInteriorIntersection_m)
                    {
                        continue;
                    }

                    const double firstSupport =
                        wallSegments[firstWallIndex].supportScore;
                    const double secondSupport =
                        wallSegments[secondWallIndex].supportScore;
                    const double weakerSupport =
                        std::max(std::min(firstSupport, secondSupport), 1e-8);
                    const double supportRatio =
                        std::max(firstSupport, secondSupport) / weakerSupport;

                    if (supportRatio <
                        topologyParameters.decisiveConflictSupportRatio)
                    {
                        hasAmbiguousConflict = true;
                        break;
                    }

                    geometric::Plane *p_rejectedWall =
                        firstSupport < secondSupport
                            ? wallSegments[firstWallIndex].p_wall
                            : wallSegments[secondWallIndex].p_wall;
                    geometric::Plane *p_retainedWall =
                        firstSupport < secondSupport
                            ? wallSegments[secondWallIndex].p_wall
                            : wallSegments[firstWallIndex].p_wall;

                    bool roomWasWallRemoved{};
                    if (p_room->removeWall(p_rejectedWall,
                                           roomWasWallRemoved) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        roomWasWallRemoved = false;
                        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                    "%s: removeWall rejected its input; "
                                    "continuing as before.",
                                    __func__);
                    }
                    if (roomWasWallRemoved)
                    {
                        boundaryWasRepaired = true;

                        int roomId2{};
                        if (p_room->getId(roomId2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        int rejectedWallGetId{};
                        if (p_rejectedWall->getId(rejectedWallGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        int retainedWallGetId{};
                        if (p_retainedWall->getId(retainedWallGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        std::cout << "[SemMgr] Detached clashing Wall#"
                                  << rejectedWallGetId
                                  << " from semantic::Room#" << roomId2
                                  << "; Wall#" << retainedWallGetId
                                  << " has decisively stronger finite support."
                                  << std::endl;
                    }

                    break;
                }
            }
        }

        /* Situational awareness for incomplete rooms (user rule): compute
         * this room's unobserved angular sectors from whatever wall
         * evidence currently exists, regardless of the boundary-status
         * outcome below -- this is precisely the "what's still missing"
         * signal a genuinely COMPLETE room no longer needs. Runs ahead of
         * the CONFLICTING/INCOMPLETE/UNOBSERVED branches below so it isn't
         * skipped by any of their early `continue`s. */
        {
            Eigen::Vector3d gapCentroid_world_m{};
            if (p_room->getCentroid(gapCentroid_world_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (gapCentroid_world_m.allFinite())
            {
                const Eigen::Vector2d gapCentroidGround_m(
                    gapCentroid_world_m.dot(groundAxisU_world),
                    gapCentroid_world_m.dot(groundAxisV_world));
                std::vector<semantic::Room::ObservationGap>
                    roomObservationGaps{};
                if (computeRoomObservationGaps(wallSegments,
                                               gapCentroidGround_m,
                                               roomObservationGaps) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: computeRoomObservationGaps returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_room->setObservationGaps(roomObservationGaps) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setObservationGaps returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                if (p_room->setObservationGaps({}) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setObservationGaps returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        if (hasAmbiguousConflict)
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::CONFLICTING);
            continue;
        }

        if (wallSegments.size() < topologyParameters.minimumWallCount)
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::INCOMPLETE);
            continue;
        }

        Eigen::Vector3d roomCentroid_world_m{};
        if (p_room->getCentroid(roomCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (!roomCentroid_world_m.allFinite())
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::UNOBSERVED);
            continue;
        }

        const Eigen::Vector2d roomCentroidGround_m(
            roomCentroid_world_m.dot(groundAxisU_world),
            roomCentroid_world_m.dot(groundAxisV_world));

        WallLoopClosure closure{};
        if (tryCloseWallLoop(wallSegments,
                             roomCentroidGround_m,
                             topologyParameters,
                             closure) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: tryCloseWallLoop returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* User rule: a wall the room owns but which does not belong to the
         * room's true closed boundary is invalid and must be pruned, not
         * treated as an unrelated reason the whole loop fails to close.
         * The full wallSegments set may include exactly one such outlier
         * (e.g. a wall genuinely belonging to a neighbouring, unlinked
         * room, or a stale duplicate) -- if closing the full set fails, and
         * excluding exactly one wall lets the remainder close cleanly, that
         * excluded wall is the outlier: reassign wallSegments to the
         * closure-achieving subset so every downstream step (self-
         * intersection, area, corner heights, and the loop-membership
         * pruning check below) is consistent, and the excluded wall is
         * naturally caught as "not part of the loop" and detached there
         * (never removed here directly -- that keeps this decision subject
         * to the same passage-explained-ness check as any other off-loop
         * wall). Only a single outlier is handled: searching every subset
         * of exclusions is combinatorial and unnecessary for the case this
         * rule targets. */
        if (closure.hasOpenBoundary &&
            wallSegments.size() > topologyParameters.minimumWallCount)
        {
            std::vector<std::size_t> indicesBySupportAscending(
                wallSegments.size());
            std::iota(indicesBySupportAscending.begin(),
                      indicesBySupportAscending.end(),
                      0U);
            std::sort(
                indicesBySupportAscending.begin(),
                indicesBySupportAscending.end(),
                [&wallSegments](std::size_t firstIndex, std::size_t secondIndex)
                {
                    return wallSegments[firstIndex].supportScore <
                           wallSegments[secondIndex].supportScore;
                });

            for (std::size_t excludeIndex : indicesBySupportAscending)
            {
                std::vector<FiniteWallSegment2d> reducedWallSegments;
                reducedWallSegments.reserve(wallSegments.size() - 1U);
                for (std::size_t segmentIndex = 0U;
                     segmentIndex < wallSegments.size();
                     ++segmentIndex)
                {
                    if (segmentIndex != excludeIndex)
                    {
                        reducedWallSegments.push_back(
                            wallSegments[segmentIndex]);
                    }
                }

                WallLoopClosure reducedClosure{};
                if (tryCloseWallLoop(reducedWallSegments,
                                     roomCentroidGround_m,
                                     topologyParameters,
                                     reducedClosure) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: tryCloseWallLoop returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (!reducedClosure.hasOpenBoundary)
                {
                    int roomId3{};
                    if (p_room->getId(roomId3) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int getId2{};
                    if (wallSegments[excludeIndex].p_wall->getId(getId2) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout
                        << "[SemMgr] semantic::Room#" << roomId3
                        << ": excluding Wall#" << getId2
                        << " lets the remaining " << reducedWallSegments.size()
                        << " wall(s) close a valid loop; treating it as an "
                           "off-loop outlier."
                        << std::endl;
                    closure      = reducedClosure;
                    wallSegments = reducedWallSegments;
                    break;
                }
            }
        }

        if (closure.hasOpenBoundary)
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::INCOMPLETE);
            continue;
        }

        std::vector<Eigen::Vector2d> boundaryCorners_world_m =
            closure.corners_world_m;

        bool polygonSelfIntersects = false;

        for (std::size_t firstEdgeIndex = 0U;
             firstEdgeIndex < boundaryCorners_world_m.size() &&
             !polygonSelfIntersects;
             ++firstEdgeIndex)
        {
            FiniteWallSegment2d firstBoundaryEdge;
            firstBoundaryEdge.start_world_m =
                boundaryCorners_world_m[firstEdgeIndex];
            firstBoundaryEdge.end_world_m =
                boundaryCorners_world_m[(firstEdgeIndex + 1U) %
                                        boundaryCorners_world_m.size()];

            for (std::size_t secondEdgeIndex = firstEdgeIndex + 1U;
                 secondEdgeIndex < boundaryCorners_world_m.size();
                 ++secondEdgeIndex)
            {
                const bool edgesAreAdjacent =
                    secondEdgeIndex == firstEdgeIndex + 1U ||
                    (firstEdgeIndex == 0U &&
                     secondEdgeIndex + 1U == boundaryCorners_world_m.size());

                if (edgesAreAdjacent)
                {
                    continue;
                }

                FiniteWallSegment2d secondBoundaryEdge;
                secondBoundaryEdge.start_world_m =
                    boundaryCorners_world_m[secondEdgeIndex];
                secondBoundaryEdge.end_world_m =
                    boundaryCorners_world_m[(secondEdgeIndex + 1U) %
                                            boundaryCorners_world_m.size()];

                Eigen::Vector2d intersection_world_m;
                double          firstParameter  = 0.0;
                double          secondParameter = 0.0;

                bool hasIntersection2{};
                if (intersectSupportingLines(firstBoundaryEdge,
                                             secondBoundaryEdge,
                                             intersection_world_m,
                                             firstParameter,
                                             secondParameter,
                                             hasIntersection2) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: intersectSupportingLines returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (hasIntersection2 && firstParameter > 1e-6 &&
                    firstParameter < 1.0 - 1e-6 && secondParameter > 1e-6 &&
                    secondParameter < 1.0 - 1e-6)
                {
                    polygonSelfIntersects = true;
                    break;
                }
            }
        }

        double enclosedArea_m2{};
        if (computePolygonArea_m2(boundaryCorners_world_m, enclosedArea_m2) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computePolygonArea_m2 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        int roomId4{};
        if (p_room->getId(roomId4) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] semantic::Room#" << roomId4
                  << " boundary validation: walls=" << wallSegments.size()
                  << ", corners=" << boundaryCorners_world_m.size()
                  << ", selfIntersects="
                  << (polygonSelfIntersects ? "true" : "false")
                  << ", area=" << enclosedArea_m2 << " m2"
                  << ", minArea=" << topologyParameters.minimumEnclosedArea_m2
                  << " m2" << std::endl;

        if (polygonSelfIntersects)
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::CONFLICTING);
        }
        else if (!std::isfinite(enclosedArea_m2) ||
                 enclosedArea_m2 < topologyParameters.minimumEnclosedArea_m2)
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::INCOMPLETE);
        }
        else
        {
            /* Lift the validated 2D ground-tangent corners back into world
             * coordinates: U*axisU + V*axisV recovers the horizontal
             * position exactly (the orthonormal decomposition this loop's
             * own 2D coordinates were built from), and each corner's height
             * is the mean of its two meeting walls' own position along the
             * ground normal. */
            std::vector<Eigen::Vector3d> boundaryCorners3d_world_m;
            boundaryCorners3d_world_m.reserve(boundaryCorners_world_m.size());
            for (std::size_t cornerIndex = 0U;
                 cornerIndex < boundaryCorners_world_m.size();
                 ++cornerIndex)
            {
                const geometric::Plane *p_currentCornerWall =
                    wallSegments[cornerIndex].p_wall;
                const geometric::Plane *p_nextCornerWall =
                    wallSegments[(cornerIndex + 1U) % wallSegments.size()]
                        .p_wall;
                double height_m = 0.0;
                if (p_currentCornerWall != nullptr &&
                    p_nextCornerWall != nullptr)
                {
                    Eigen::Vector3d currentCornerWallGetCentroid{};
                    if (p_currentCornerWall->getCentroid(
                            currentCornerWallGetCentroid) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector3d nextCornerWallGetCentroid{};
                    if (p_nextCornerWall->getCentroid(
                            nextCornerWallGetCentroid) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    height_m =
                        0.5 * (currentCornerWallGetCentroid.cast<double>().dot(
                                   groundNormal_world) +
                               nextCornerWallGetCentroid.cast<double>().dot(
                                   groundNormal_world));
                }
                boundaryCorners3d_world_m.push_back(
                    boundaryCorners_world_m[cornerIndex].x() *
                        groundAxisU_world +
                    boundaryCorners_world_m[cornerIndex].y() *
                        groundAxisV_world +
                    height_m * groundNormal_world);
            }
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::COMPLETE,
                                 boundaryCorners3d_world_m);

            /* User rule: once a room's boundary is a genuine closed loop,
             * any wall it still owns that is NOT one of that loop's own
             * walls, and that no passage tied to this room explains (i.e.
             * not one of the wall faces framing a doorway out of this
             * room), could never actually have been observed as this
             * room's own boundary -- detach it. This is deliberately
             * intra-room only (mirrors the existing intra-room clash
             * repair in admitWallToRoom()): it only prunes walls the loop
             * computation above already excluded, never a wall that
             * belongs to a different room. */
            std::vector<geometric::Plane *> loopWalls;
            loopWalls.reserve(wallSegments.size());
            for (const FiniteWallSegment2d &segment : wallSegments)
            {
                if (segment.p_wall != nullptr)
                {
                    loopWalls.push_back(segment.p_wall);
                }
            }

            std::vector<semantic::Passage *> roomPassages{};
            if (p_room->getPassages(roomPassages) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            std::vector<geometric::Plane *> roomWalls4{};
            if (p_room->getWalls(roomWalls4) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_ownedWall : roomWalls4)
            {
                if (p_ownedWall == nullptr)
                {
                    continue;
                }

                if (std::find(loopWalls.begin(),
                              loopWalls.end(),
                              p_ownedWall) != loopWalls.end())
                {
                    continue;
                }

                const bool explainedByPassage = std::any_of(
                    roomPassages.begin(),
                    roomPassages.end(),
                    [p_ownedWall](semantic::Passage *p_passage)
                    {
                        if (p_passage == nullptr)
                        {
                            return false;
                        }
                        std::vector<geometric::Plane *> supportingWalls{};
                        if (p_passage->getAssociateWalls(supportingWalls) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            // getAssociateWalls cannot fail; continue as
                            // before.
                        }
                        return std::find(supportingWalls.begin(),
                                         supportingWalls.end(),
                                         p_ownedWall) != supportingWalls.end();
                    });

                if (explainedByPassage)
                {
                    continue;
                }

                bool roomWasWallRemoved2{};
                if (p_room->removeWall(p_ownedWall, roomWasWallRemoved2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    roomWasWallRemoved2 = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: removeWall rejected its input; continuing "
                                "as before.",
                                __func__);
                }
                if (roomWasWallRemoved2)
                {
                    int roomId5{};
                    if (p_room->getId(roomId5) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int ownedWallGetId{};
                    if (p_ownedWall->getId(ownedWallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] semantic::Room#" << roomId5
                              << "'s boundary is COMPLETE; detached Wall#"
                              << ownedWallGetId
                              << ", which is neither part of the closed wall "
                                 "loop nor explained by any of this room's "
                                 "passages."
                              << std::endl;
                }
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
