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
 * @file            updateTraversalEvidence.cc
 *
 * @brief           Implements SemanticsManager::updateTraversalEvidence(),
 *                  declared in SemanticsManager.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_tracking/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <unordered_map>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::updateTraversalEvidence(atlas::Atlas *p_atlas_in)
{
    PROFILE_FUNCTION();
    if (p_atlas_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    map::Map *p_activeMap = nullptr;
    if (p_atlas_in->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_activeMap == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    unsigned long activeMapId{};
    if (p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    if (resetTemporalStateForMap(p_activeMap) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: resetTemporalStateForMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<keyframe::KeyFrame *> orderedKeyFrames{};
    if (p_activeMap->getAllKeyFrames(orderedKeyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    /* Drop missing and bad key frames, keeping the map order. */
    std::vector<keyframe::KeyFrame *> liveKeyFrames{};
    liveKeyFrames.reserve(orderedKeyFrames.size());
    for (keyframe::KeyFrame *p_keyFrame : orderedKeyFrames)
    {
        bool keyFrameIsBad{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!(p_keyFrame == nullptr || keyFrameIsBad))
        {
            liveKeyFrames.push_back(p_keyFrame);
        }
    }
    orderedKeyFrames.swap(liveKeyFrames);
    std::sort(orderedKeyFrames.begin(),
              orderedKeyFrames.end(),
              isKeyFrameEarlier);

    if (orderedKeyFrames.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Prepare the ground normal: aperture height/width tests need the vertical
     * axis. Without it there is no reliable opening bounds test. */
    vs_graphs::core::geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas_in->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    Eigen::Vector3d groundNormal_world = Eigen::Vector3d::Zero();

    bool hasValidGroundNormal = false;

    bool groundPlaneIsBad{};
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
        Eigen::Vector4d groundEquation = groundPlaneGetGlobalEquation.coeffs();
        const double    groundNormalNorm = groundEquation.head<3>().norm();

        if (groundEquation.allFinite() && std::isfinite(groundNormalNorm) &&
            groundNormalNorm > 1e-8)
        {
            groundEquation /= groundNormalNorm;
            groundNormal_world   = groundEquation.head<3>();
            hasValidGroundNormal = true;
        }
    }

    if (!hasValidGroundNormal)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double openingMargin_m = static_cast<double>(
        p_sysParams->roomSeg.passagePartition.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        p_sysParams->roomSeg.passagePartition.minimumSideDistance_m);
    std::vector<semantic::Passage *> passages{};
    if (getMapPassagesInIdOrder(activeMapId, passages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPassagesInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }

    /* Passage confirmation is delayed relative to flight. Replay a bounded
     * recent trajectory on every semantic cycle; Passage deduplicates segment
     * IDs, so a crossing observed before the passage existed is retained once
     * the persistent passage appears. */
    constexpr std::size_t maximumTraversalHistoryKeyFrames = 128U;
    const std::size_t     historyStartIndex =
        orderedKeyFrames.size() > maximumTraversalHistoryKeyFrames
                ? orderedKeyFrames.size() - maximumTraversalHistoryKeyFrames
                : 0U;
    keyframe::KeyFrame *p_seedKeyFrame = orderedKeyFrames[historyStartIndex];
    Eigen::Vector3f     seedKeyFrameCameraCenter{};
    if (p_seedKeyFrame->getCameraCenter(seedKeyFrameCameraCenter) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCameraCenter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    currentCameraCenter_world_m = seedKeyFrameCameraCenter.cast<double>();
    if (!currentCameraCenter_world_m.allFinite())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    hasCameraCenter   = true;
    p_cameraCenterMap = p_activeMap;

    /*!
     * Last camera centre that lay resolvably (at least minimumSideDistance_m)
     * on one side of each passage's plane. A crossing is tested from this
     * point, not from the previous key frame: key frames of a slow flight are
     * a few centimetres apart, so two consecutive centres can never both be
     * minimumSideDistance_m from the plane on opposite sides, and a crossing
     * between consecutive key frames alone would never be seen.
     */
    std::unordered_map<semantic::Passage *, Eigen::Vector3d>
        resolvedSideCenter_world_m{};

    for (std::size_t keyFrameIndex = historyStartIndex + 1U;
         keyFrameIndex < orderedKeyFrames.size();
         ++keyFrameIndex)
    {
        keyframe::KeyFrame *p_keyFrame = orderedKeyFrames[keyFrameIndex];

        Eigen::Vector3f keyFrameCameraCenter{};
        if (p_keyFrame->getCameraCenter(keyFrameCameraCenter) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d nextCameraCenter_world_m =
            keyFrameCameraCenter.cast<double>();

        lastTraversalFrameId    = p_keyFrame->frameId;
        lastTraversalKeyFrameId = p_keyFrame->id;

        if (!nextCameraCenter_world_m.allFinite())
        {
            continue;
        }

        previousCameraCenter_world_m = currentCameraCenter_world_m;
        currentCameraCenter_world_m  = nextCameraCenter_world_m;

        for (vs_graphs::core::semantic::Passage *p_passage : passages)
        {
            bool passageIsPassable{};
            if (!(p_passage == nullptr) &&
                p_passage->isPassable(passageIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage == nullptr || !passageIsPassable)
            {
                continue;
            }

            g2o::Plane3D passagePlane{};
            if (p_passage->getGlobalEquation(passagePlane) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d passagePlaneEquation = passagePlane.coeffs();
            const double    passageNormalNorm =
                passagePlaneEquation.head<3>().norm();
            if (!passagePlaneEquation.allFinite() || passageNormalNorm <= 1e-8)
            {
                continue;
            }
            passagePlaneEquation /= passageNormalNorm;
            const double previousSideDistance_m =
                passagePlaneEquation.head<3>().dot(
                    previousCameraCenter_world_m) +
                passagePlaneEquation(3);
            const double currentSideDistance_m =
                passagePlaneEquation.head<3>().dot(
                    currentCameraCenter_world_m) +
                passagePlaneEquation(3);

            /* The first resolvable centre in the replayed history (possibly
             * the seed) starts the crossing segment. */
            if (resolvedSideCenter_world_m.count(p_passage) == 0U &&
                std::abs(previousSideDistance_m) >= minimumSideDistance_m)
            {
                resolvedSideCenter_world_m[p_passage] =
                    previousCameraCenter_world_m;
            }
            if (std::abs(currentSideDistance_m) < minimumSideDistance_m)
            {
                /* Too close to the plane to tell the side; keep the start. */
                continue;
            }
            if (resolvedSideCenter_world_m.count(p_passage) == 0U)
            {
                resolvedSideCenter_world_m[p_passage] =
                    currentCameraCenter_world_m;
                continue;
            }
            const Eigen::Vector3d crossingStart_world_m =
                resolvedSideCenter_world_m[p_passage];
            resolvedSideCenter_world_m[p_passage] = currentCameraCenter_world_m;

            /* The segment from the last resolvable centre to this one
             * approximates the UAV trajectory. When it crosses inside the
             * finite aperture while the passage is passable, the UAV has flown
             * through the opening: record traversal evidence. */
            bool crossesPassageOpening{};
            if (segmentCrossesPassageOpening(crossingStart_world_m,
                                             currentCameraCenter_world_m,
                                             p_passage,
                                             groundNormal_world,
                                             openingMargin_m,
                                             minimumSideDistance_m,
                                             crossesPassageOpening,
                                             true) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: segmentCrossesPassageOpening returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            if (crossesPassageOpening)
            {
                bool wasSettled{};
                if (p_passage->getTraversalEvidence(wasSettled) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getTraversalEvidence returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                semantic::Passage::TraversalDirection traversalDirection =
                    semantic::Passage::TraversalDirection::UNKNOWN;
                semantic::Passage::KnownSideProvenance knownSide{};
                if (resolveTraversalDirection(p_passage,
                                              crossingStart_world_m,
                                              minimumSideDistance_m,
                                              knownSide,
                                              traversalDirection) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: resolveTraversalDirection returned a failure "
                        "status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                /* Resolve the room entered after crossing this passage: a
                 * KNOWN_TO_FAR crossing reaches the far side, a FAR_TO_KNOWN
                 * crossing returns to the known side. */
                vs_graphs::core::semantic::Room *p_reachedRoom = nullptr;
                if (traversalDirection ==
                    semantic::Passage::TraversalDirection::KNOWN_TO_FAR)
                {
                    vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                        nullptr;
                    if (p_passage->getProspectiveRoom(
                            p_passageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getProspectiveRoom returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    p_reachedRoom = p_passageProspectiveRoom;
                }
                else if (traversalDirection ==
                         semantic::Passage::TraversalDirection::FAR_TO_KNOWN)
                {
                    p_reachedRoom = knownSide.p_room;
                }
                if (enterTraversedRoom(activeMapId,
                                       p_activeMap,
                                       p_passage,
                                       p_reachedRoom) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: enterTraversedRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                bool addedTraversal{};
                if (p_passage->addTraversalObservation(traversalDirection,
                                                       p_keyFrame->frameId,
                                                       p_keyFrame->id,
                                                       addedTraversal) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addTraversalObservation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (recordCrossingEvidence(p_passage,
                                           addedTraversal,
                                           wasSettled) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: recordCrossingEvidence returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    keyframe::KeyFrame *p_latestKeyFrame = orderedKeyFrames.back();
    lastTraversalFrameId                 = p_latestKeyFrame->frameId;
    lastTraversalKeyFrameId              = p_latestKeyFrame->id;
    hasTraversalKeyFrameCursor           = true;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
