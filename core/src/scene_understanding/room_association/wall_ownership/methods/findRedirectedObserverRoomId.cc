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

#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

#include "map_state/keyframe/objects/KeyFrameClass.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
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
     * for the room on the far side of the first such passage in id order
     * (no vote when that room is unknown); any other observer votes for the
     * room it was taken in. */
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
        int votedRoomId = p_observer->roomId;
        for (semantic::Passage *p_passage : passages_in)
        {
            singlePassage.front()      = p_passage;
            geometric::Plane *p_unused = nullptr;
            if (!observerCameraCenter_world_m.allFinite() ||
                !semantic_constraints::isSeenThroughPassage(sightLineInput,
                                                            *p_sysParams,
                                                            p_unused))
            {
                continue;
            }
            if (findPassageFarSideRoomId(
                    p_passage,
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
            break;
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
