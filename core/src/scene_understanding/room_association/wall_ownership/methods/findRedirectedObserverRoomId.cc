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
 * @file            findRedirectedObserverRoomId.cc
 *
 * @brief           Implements SemanticsManager::findRedirectedObserverRoomId(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

#include "map_state/keyframe/objects/KeyFrameClass.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::findRedirectedObserverRoomId(
    geometric::Plane                       *p_wall_in,
    const std::vector<semantic::Room *>    &rooms_in,
    const std::vector<semantic::Passage *> &passages_in,
    int                                    &redirectedRoomId_out) const
{
    redirectedRoomId_out = -1;
    std::vector<keyframe::KeyFrame *> observerKeyFrames{};
    if (semantic::collectObserverKeyFrames(p_wall_in, observerKeyFrames) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectObserverKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (observerKeyFrames.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* The passage rule of the visibility table judges each observer's sight
     * line, one passage at a time so the crossed passage is known. */
    const std::vector<geometric::Plane *>     noRoomWalls{};
    std::vector<semantic::Passage *>          singlePassage(1U, nullptr);
    semantic_constraints::VisibilityRuleInput sightLineInput{};
    sightLineInput.p_candidateWall = p_wall_in;
    sightLineInput.p_roomWalls     = &noRoomWalls;
    sightLineInput.p_mapPassages   = &singlePassage;
    if (p_wall_in->getCentroid(sightLineInput.candidateCentroid_world_m) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
        p_wall_in->getTwinFace(sightLineInput.p_candidateTwinWall) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid/getTwinFace returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    /* Passage first: an observer that sees the wall through a passage votes
     * for the room on the far side of that passage (no vote when that room is
     * unknown); any other observer votes for the room it was taken in. The
     * passage rule tests a passage's associated walls, and one long wall can
     * host several passages, so when several passages qualify the one whose
     * centroid lies nearest the sight line is the opening looked through
     * (ties keep the lower id). */
    std::vector<std::pair<int, long unsigned int>> roomVotes{};
    for (keyframe::KeyFrame *p_observer : observerKeyFrames)
    {
        Eigen::Vector3f observerCameraCenter_world_m{};
        if (p_observer->getCameraCenter(observerCameraCenter_world_m) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        sightLineInput.observerViewpoints_world_m = {
            observerCameraCenter_world_m.cast<double>()};
        int                   votedRoomId              = p_observer->roomId;
        semantic::Passage    *p_nearestSeenPassage     = nullptr;
        double                nearestPassageDistance_m = 0.0;
        const Eigen::Vector3d sightLineStart_world_m =
            observerCameraCenter_world_m.cast<double>();
        const Eigen::Vector3d sightLine_world_m =
            sightLineInput.candidateCentroid_world_m - sightLineStart_world_m;
        for (semantic::Passage *p_passage : passages_in)
        {
            singlePassage.front()      = p_passage;
            geometric::Plane *p_unused = nullptr;
            bool              isPassageBad{true};
            if (p_passage == nullptr ||
                p_passage->isBad(isPassageBad) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                isPassageBad || !observerCameraCenter_world_m.allFinite() ||
                !semantic_constraints::isSeenThroughPassage(sightLineInput,
                                                            *p_sysParams,
                                                            p_unused))
            {
                continue;
            }
            /* Distance from the passage centroid to the closest point of the
             * observer-to-wall sight line segment. */
            Eigen::Vector3d passageCentroid_world_m{};
            if (p_passage->getCentroid(passageCentroid_world_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const double sightLineLengthSquared_m2 =
                sightLine_world_m.squaredNorm();
            const double closestPointFraction =
                sightLineLengthSquared_m2 > 0.0
                    ? std::clamp(
                          (passageCentroid_world_m - sightLineStart_world_m)
                                  .dot(sightLine_world_m) /
                              sightLineLengthSquared_m2,
                          0.0,
                          1.0)
                    : 0.0;
            const double passageDistance_m =
                (sightLineStart_world_m +
                 closestPointFraction * sightLine_world_m -
                 passageCentroid_world_m)
                    .norm();
            if (p_nearestSeenPassage == nullptr ||
                passageDistance_m < nearestPassageDistance_m)
            {
                p_nearestSeenPassage     = p_passage;
                nearestPassageDistance_m = passageDistance_m;
            }
        }
        if (p_nearestSeenPassage != nullptr &&
            findPassageFarSideRoomId(p_nearestSeenPassage,
                                     p_observer->roomId,
                                     sightLineInput.candidateCentroid_world_m,
                                     rooms_in,
                                     votedRoomId) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findPassageFarSideRoomId returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        roomVotes.emplace_back(votedRoomId, p_observer->id);
    }

    if (semantic::selectMajorityRoomId(roomVotes, redirectedRoomId_out) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectMajorityRoomId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
