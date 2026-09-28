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
#include <cmath>
#include <numeric>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::validateRoomBoundaries(void)
{
    std::cout << "[SemMgr] validateRoomBoundaries() called" << std::endl;

    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters =
        p_sysParams->roomSeg.boundaryTopology;

    if (!topologyParameters.enabled)
    {
        std::cout << "[SemMgr] boundary topology disabled" << std::endl;
        return;
    }

    geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }

    Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const Eigen::Vector3d groundAxisU_World =
        groundNormal_World.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_World =
        groundNormal_World.cross(groundAxisU_World).normalized();

    const auto updateBoundaryStatus =
        [](semantic::Room                      *p_room_in,
           const semantic::Room::BoundaryStatus boundaryStatus_in,
           const std::vector<Eigen::Vector3d>  &corners_World_m_in = {})
    {
        /* Refresh stored corners every cycle the loop is COMPLETE (even when
         * the status itself didn't change -- wall positions can still
         * drift), and clear them the moment it stops being COMPLETE. */
        p_room_in->setBoundaryCorners_World_m(
            boundaryStatus_in == semantic::Room::BoundaryStatus::COMPLETE
                ? corners_World_m_in
                : std::vector<Eigen::Vector3d>{});

        const semantic::Room::BoundaryStatus previousBoundaryStatus =
            p_room_in->getBoundaryStatus();

        if (previousBoundaryStatus == boundaryStatus_in)
        {
            return;
        }

        p_room_in->setBoundaryStatus(boundaryStatus_in);

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

        std::cout << "[SemMgr] semantic::Room#" << p_room_in->getId()
                  << " boundary=" << boundaryStatusName(boundaryStatus_in)
                  << " (" << p_room_in->getWalls().size() << " walls)"
                  << std::endl;
    };

    for (semantic::Room *p_room : p_atlas->getAllRooms())
    {
        if (p_room == nullptr || p_room->isBad())
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

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                FiniteWallSegment2d wallSegment;

                if (buildFiniteWallSegment2d(
                        p_wall,
                        groundNormal_World,
                        groundAxisU_World,
                        groundAxisV_World,
                        topologyParameters.endpointTrimRatio,
                        topologyParameters.minimumWallLength_m,
                        wallSegment))
                {
                    wallSegments.push_back(std::move(wallSegment));
                }
            }

            std::cout << "[SemMgr] semantic::Room#" << p_room->getId()
                      << " boundary check: roomWalls="
                      << p_room->getWalls().size()
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
                    Eigen::Vector2d intersection_World_m;
                    double          firstParameter  = 0.0;
                    double          secondParameter = 0.0;

                    if (!intersectSupportingLines(wallSegments[firstWallIndex],
                                                  wallSegments[secondWallIndex],
                                                  intersection_World_m,
                                                  firstParameter,
                                                  secondParameter) ||
                        firstParameter < 0.0 || firstParameter > 1.0 ||
                        secondParameter < 0.0 || secondParameter > 1.0)
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

                    if (p_room->removeWall(p_rejectedWall))
                    {
                        boundaryWasRepaired = true;

                        std::cout << "[SemMgr] Detached clashing Wall#"
                                  << p_rejectedWall->getId()
                                  << " from semantic::Room#" << p_room->getId()
                                  << "; Wall#" << p_retainedWall->getId()
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
            const Eigen::Vector3d gapCentroid_World_m = p_room->getCentroid();
            if (gapCentroid_World_m.allFinite())
            {
                const Eigen::Vector2d gapCentroidGround_m(
                    gapCentroid_World_m.dot(groundAxisU_World),
                    gapCentroid_World_m.dot(groundAxisV_World));
                p_room->setObservationGaps(
                    computeRoomObservationGaps(wallSegments,
                                               gapCentroidGround_m));
            }
            else
            {
                p_room->setObservationGaps({});
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

        const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

        if (!roomCentroid_World_m.allFinite())
        {
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::UNOBSERVED);
            continue;
        }

        const Eigen::Vector2d roomCentroidGround_m(
            roomCentroid_World_m.dot(groundAxisU_World),
            roomCentroid_World_m.dot(groundAxisV_World));

        WallLoopClosure closure = tryCloseWallLoop(wallSegments,
                                                   roomCentroidGround_m,
                                                   topologyParameters);

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

                WallLoopClosure reducedClosure =
                    tryCloseWallLoop(reducedWallSegments,
                                     roomCentroidGround_m,
                                     topologyParameters);

                if (!reducedClosure.hasOpenBoundary)
                {
                    std::cout
                        << "[SemMgr] semantic::Room#" << p_room->getId()
                        << ": excluding Wall#"
                        << wallSegments[excludeIndex].p_wall->getId()
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

        std::vector<Eigen::Vector2d> boundaryCorners_World_m =
            closure.corners_World_m;

        bool polygonSelfIntersects = false;

        for (std::size_t firstEdgeIndex = 0U;
             firstEdgeIndex < boundaryCorners_World_m.size() &&
             !polygonSelfIntersects;
             ++firstEdgeIndex)
        {
            FiniteWallSegment2d firstBoundaryEdge;
            firstBoundaryEdge.start_World_m =
                boundaryCorners_World_m[firstEdgeIndex];
            firstBoundaryEdge.end_World_m =
                boundaryCorners_World_m[(firstEdgeIndex + 1U) %
                                        boundaryCorners_World_m.size()];

            for (std::size_t secondEdgeIndex = firstEdgeIndex + 1U;
                 secondEdgeIndex < boundaryCorners_World_m.size();
                 ++secondEdgeIndex)
            {
                const bool edgesAreAdjacent =
                    secondEdgeIndex == firstEdgeIndex + 1U ||
                    (firstEdgeIndex == 0U &&
                     secondEdgeIndex + 1U == boundaryCorners_World_m.size());

                if (edgesAreAdjacent)
                {
                    continue;
                }

                FiniteWallSegment2d secondBoundaryEdge;
                secondBoundaryEdge.start_World_m =
                    boundaryCorners_World_m[secondEdgeIndex];
                secondBoundaryEdge.end_World_m =
                    boundaryCorners_World_m[(secondEdgeIndex + 1U) %
                                            boundaryCorners_World_m.size()];

                Eigen::Vector2d intersection_World_m;
                double          firstParameter  = 0.0;
                double          secondParameter = 0.0;

                if (intersectSupportingLines(firstBoundaryEdge,
                                             secondBoundaryEdge,
                                             intersection_World_m,
                                             firstParameter,
                                             secondParameter) &&
                    firstParameter > 1e-6 && firstParameter < 1.0 - 1e-6 &&
                    secondParameter > 1e-6 && secondParameter < 1.0 - 1e-6)
                {
                    polygonSelfIntersects = true;
                    break;
                }
            }
        }

        const double enclosedArea_m2 =
            computePolygonArea_m2(boundaryCorners_World_m);

        std::cout << "[SemMgr] semantic::Room#" << p_room->getId()
                  << " boundary validation: walls=" << wallSegments.size()
                  << ", corners=" << boundaryCorners_World_m.size()
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
            std::vector<Eigen::Vector3d> boundaryCorners3d_World_m;
            boundaryCorners3d_World_m.reserve(boundaryCorners_World_m.size());
            for (std::size_t cornerIndex = 0U;
                 cornerIndex < boundaryCorners_World_m.size();
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
                    height_m =
                        0.5 *
                        (p_currentCornerWall->getCentroid().cast<double>().dot(
                             groundNormal_World) +
                         p_nextCornerWall->getCentroid().cast<double>().dot(
                             groundNormal_World));
                }
                boundaryCorners3d_World_m.push_back(
                    boundaryCorners_World_m[cornerIndex].x() *
                        groundAxisU_World +
                    boundaryCorners_World_m[cornerIndex].y() *
                        groundAxisV_World +
                    height_m * groundNormal_World);
            }
            updateBoundaryStatus(p_room,
                                 semantic::Room::BoundaryStatus::COMPLETE,
                                 boundaryCorners3d_World_m);

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

            const std::vector<semantic::Passage *> roomPassages =
                p_room->getPassages();

            for (geometric::Plane *p_ownedWall : p_room->getWalls())
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
                        const std::vector<geometric::Plane *> supportingWalls =
                            p_passage->getAssociateWalls();
                        return std::find(supportingWalls.begin(),
                                         supportingWalls.end(),
                                         p_ownedWall) != supportingWalls.end();
                    });

                if (explainedByPassage)
                {
                    continue;
                }

                if (p_room->removeWall(p_ownedWall))
                {
                    std::cout << "[SemMgr] semantic::Room#" << p_room->getId()
                              << "'s boundary is COMPLETE; detached Wall#"
                              << p_ownedWall->getId()
                              << ", which is neither part of the closed wall "
                                 "loop nor explained by any of this room's "
                                 "passages."
                              << std::endl;
                }
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
