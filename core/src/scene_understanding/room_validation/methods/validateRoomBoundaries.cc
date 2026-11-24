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

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"

#include <cmath>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::validateRoomBoundaries(void)
{
    PROFILE_FUNCTION();
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
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
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

    std::vector<semantic::Room *> atlasAllRooms{};
    if (getAllRoomsInIdOrder(atlasAllRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
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

        bool                             hasAmbiguousConflict = false;
        std::vector<FiniteWallSegment2d> wallSegments;

        /* Repair decisive finite-wall clashes until none is left. */
        if (repairRoomWallClashes(p_room,
                                  groundNormal_world,
                                  groundAxisU_world,
                                  groundAxisV_world,
                                  topologyParameters,
                                  wallSegments,
                                  hasAmbiguousConflict) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: repairRoomWallClashes returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Situational awareness for incomplete rooms (user rule): compute
         * this room's unobserved angular sectors from whatever wall
         * evidence currently exists, regardless of the boundary-status
         * outcome below -- this is precisely the "what's still missing"
         * signal a genuinely COMPLETE room no longer needs. Runs ahead of
         * the CONFLICTING/INCOMPLETE/UNOBSERVED branches below so it isn't
         * skipped by any of their early `continue`s. */
        if (updateRoomObservationGaps(p_room,
                                      wallSegments,
                                      groundAxisU_world,
                                      groundAxisV_world) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: updateRoomObservationGaps returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        if (hasAmbiguousConflict)
        {
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::CONFLICTING) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        if (wallSegments.size() < topologyParameters.minimumWallCount)
        {
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::INCOMPLETE) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::UNOBSERVED) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            if (excludeOffLoopOutlierWall(p_room,
                                          roomCentroidGround_m,
                                          topologyParameters,
                                          wallSegments,
                                          closure) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: excludeOffLoopOutlierWall returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        if (closure.hasOpenBoundary)
        {
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::INCOMPLETE) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        std::vector<Eigen::Vector2d> boundaryCorners_world_m =
            closure.loopCorners_world_m;

        bool polygonSelfIntersects = false;
        if (checkPolygonSelfIntersection(boundaryCorners_world_m,
                                         polygonSelfIntersects) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: checkPolygonSelfIntersection returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
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
        std::cout << "[SemMgr] Room#" << roomId4
                  << " boundary validation: walls=" << wallSegments.size()
                  << ", corners=" << boundaryCorners_world_m.size()
                  << ", selfIntersects="
                  << (polygonSelfIntersects ? "true" : "false")
                  << ", area=" << enclosedArea_m2 << " m2"
                  << ", minArea=" << topologyParameters.minimumEnclosedArea_m2
                  << " m2" << std::endl;

        if (polygonSelfIntersects)
        {
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::CONFLICTING) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        else if (!std::isfinite(enclosedArea_m2) ||
                 enclosedArea_m2 < topologyParameters.minimumEnclosedArea_m2)
        {
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::INCOMPLETE) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            if (liftBoundaryCorners(boundaryCorners_world_m,
                                    wallSegments,
                                    groundNormal_world,
                                    groundAxisU_world,
                                    groundAxisV_world,
                                    boundaryCorners3d_world_m) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: liftBoundaryCorners returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (updateRoomBoundaryStatus(
                    p_room,
                    semantic::Room::BoundaryStatus::COMPLETE,
                    boundaryCorners3d_world_m) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

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
            if (detachOffLoopWalls(p_room, wallSegments) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: detachOffLoopWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
