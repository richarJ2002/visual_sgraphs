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
 * @file            run.cc
 *
 * @brief           Implements SemanticsManager::run(), declared in
 *                  SemanticsManager.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "scene_understanding/scene_graph/Floor/Floor.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/SemanticAxiomEvaluatorStatusEnum.h"
#include "scene_understanding/semantic/semantic_candidates/objects/SemanticCandidatesStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/objects/SemanticGraphSnapshotStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/objects/UnavailableReasonEnum.h"
#include "scene_understanding/semantic/semantic_report_cache/objects/SemanticReportCacheStatusEnum.h"
#include "scene_understanding/semantic/semantic_report_cache/semantic_report_cache.h"
#include "scene_understanding/semantic/sha256_digest/objects/Sha256DigestStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"
#include "scene_understanding/semantic/semantic_candidates/semantic_candidates.h"
#include "scene_understanding/semantic/semantic_canonical_serilization/semantic_canonical_serilization.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/semantic_graph_snapshot.h"
#include "scene_understanding/semantic/sha256_digest/sha256_digest.h"

#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::run(void)
{
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    if (p_atlas->getSceneGraph(p_sceneGraph) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::size_t summaryCycle = 0U;

    while (true)
    {
        /* Graceful shutdown on System::Shutdown() */
        bool shouldFinish{};
        if (checkFinish(shouldFinish) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (shouldFinish)
        {
            break;
        }

        /* Find the current time of the loop */
        const std::chrono::steady_clock::time_point start =
            std::chrono::steady_clock::now();

        /*!
         * Treat one hierarchy update as an atomic semantic transaction. Map
         * merging takes the same atlas-owned lock before changing frames or
         * ownership, so this cycle can never traverse a half-merged graph.
         */
        std::unique_lock<std::mutex> semanticUpdateLock{};
        if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        pipelineSemanticCycle       = ++summaryCycle;
        map::Map *p_atlasCurrentMap = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (resetTemporalStateForMap(p_atlasCurrentMap) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: resetTemporalStateForMap returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        /* One camera centre per cycle for bootstrap, admission ordering and
         * the room viewpoint. */
        if (refreshCurrentCameraCenter() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: refreshCurrentCameraCenter returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        SemanticsManager::ActiveMapBootstrapResult bootstrapResult{};
        if (ensureActiveMapBootstrapHierarchy(bootstrapResult) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: ensureActiveMapBootstrapHierarchy returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }

        /* Reject wrongly oriented ground and wall planes first, so one can
           never become the ground reference below */
        if (rejectMisorientedPlanes() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: rejectMisorientedPlanes returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }

        /* Validate the low-level semantic planes */
        geometric::Plane *p_mainGroundPlane = nullptr;
        if (p_atlas->getBiggestGroundPlane(p_mainGroundPlane) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBiggestGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* If there is a ground plane, find its transform and filter planes */
        if (p_mainGroundPlane != nullptr)
        {
            /* Find the transform from ground plane to horizontal */
            Eigen::Matrix4f plane2{};
            if (computePlaneToHorizontal(p_mainGroundPlane, plane2) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computePlaneToHorizontal returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            planePoseMat = plane2;

            /* Drop ground planes past the step limit from the main floor */
            if (filterGroundPlanes(p_mainGroundPlane) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: filterGroundPlanes returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /*!
         * Reconcile duplicate semantic planes before constructing room edges.
         *
         * Plane hypotheses are created from individual keyframe observations,
         * so one physical wall can initially exist as several overlapping or
         * contiguous fragments. Associating those fragments with a room first
         * creates duplicate graph edges and provisional structural elements.
         * The reconciliation pass validates equation, finite extent, and
         * observation-side compatibility before atomically rewiring every
         * existing reference to the retained plane.
         */
        if (p_sysParams->semSeg.reassociate.enabled)
        {
            if (utils::utils::Utils::reAssociateSemanticPlanes(p_atlas) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reAssociateSemanticPlanes returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        /*  Detect/update passage GEOMETRY before any wall-to-room admission
         * this cycle, so admitWallToRoom()'s far-side-passage backstop (and
         * Pass 1's free-space clustering below) can route a wall to the
         * correct side of a doorway the moment that doorway itself becomes
         * observable, instead of only on the NEXT cycle once a stale
         * passage list catches up. detectDoorsAndDoorways()/updatePassages()
         * only read Plane/Passage data (no Room dependency), so this is safe
         * to run ahead of any room detection or association below. Passage
         * steps that DO need current room membership (traversal evidence,
         * room association, wall detachment, prospective-room cleanup) stay
         * below, after Pass 1's wall admission, where room data exists to
         * work from. */
        if (p_sysParams->semSeg.enablePassageDetection)
        {
            if (detectDoorsAndDoorways(p_atlas) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: detectDoorsAndDoorways returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (updatePassages(p_atlas) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updatePassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (mergeOverlappingPassages() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: mergeOverlappingPassages returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        /*!
         * Associate walls with rooms using free-space evidence.
         *
         * @note            This is the preferred wall-to-room association
         *                  method.
         */
        if (p_sysParams->roomSeg.method ==
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            if (associateWallsToRoomsFromFreeSpaceClusters() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateRoomsFromFreeSpaceClusters returned a "
                             "failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /*!
         * Enforce the semantic hierarchy.
         *
         * @note            wall which was not captured by the free-space room
         *                  detector receives either an existing room or a new
         *                  provisional structural element.
         */
        if (associateAllWallsToRooms() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: associateAllWallsToRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Consolidate only redundant single-wall provisional structures. */
        if (utils::utils::Utils::reAssociateRooms(p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociateRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /*  Room-dependent passage steps: geometry was already refreshed
         * above, ahead of this cycle's wall admission. */
        if (p_sysParams->semSeg.enablePassageDetection)
        {
            if (updateTraversalEvidence(p_atlas) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateTraversalEvidence returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (utils::utils::Utils::reAssociatePassages(p_atlas) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reAssociatePassages returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (associatePassagesToRooms() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: associatePassagesToRooms returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (detachWallsBeyondConfirmedPassages() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: detachWallsBeyondConfirmedPassages returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            /* Continuous rule-invariant sweep (not just at admission time):
             * see enforcePassageSideInvariant()'s own comment. Room<->passage
             * association is current as of the two calls just above. */
            if (enforcePassageSideInvariant() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: enforcePassageSideInvariant returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /* Topology rules of every passage, after its side invariant. */
            if (enforcePassageTopologyInvariant() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: enforcePassageTopologyInvariant returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            /* PROSPECTIVE ROOM CLEANUP
             * A passage reference is the stable far-side handle. Zero-wall
             * prospectives remain alive while that reference and its geometry
             * are valid; only orphaned or invalid handles are retired. */
            map::Map *p_activeMap = nullptr;
            if (p_atlas->getCurrentMap(p_activeMap) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long activeMapId{};
            if (p_activeMap->getId(activeMapId) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::vector<vs_graphs::core::semantic::Room *> candidateRooms{};
            if (p_sceneGraph->getMapCandidateRoomsInIdOrder(activeMapId,
                                                            candidateRooms) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapCandidateRoomsInIdOrder returned a "
                             "failure status; continuing as before.",
                             __func__);
            }
            std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
            if (p_sceneGraph->getAllPassagesInIdOrder(allPassages) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassagesInIdOrder returned a failure "
                             "status; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::semantic::Room *p_candidate : candidateRooms)
            {
                if (p_candidate == nullptr)
                {
                    continue;
                }

                int roomId{};
                if (p_candidate->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                const bool isTrackedProspective =
                    prospectiveRoomCycles.count(roomId) > 0U;

                std::vector<vs_graphs::core::semantic::Passage *>
                    referencingPassages;
                for (vs_graphs::core::semantic::Passage *p_passage :
                     allPassages)
                {
                    bool passageIsBad{};
                    if ((p_passage != nullptr) &&
                        p_passage->isBad(passageIsBad) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                        nullptr;
                    if ((p_passage != nullptr && !passageIsBad) &&
                        p_passage->getProspectiveRoom(
                            p_passageProspectiveRoom) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getProspectiveRoom returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_passage != nullptr && !passageIsBad &&
                        p_passageProspectiveRoom == p_candidate)
                    {
                        referencingPassages.push_back(p_passage);
                    }
                }

                if (!isTrackedProspective && referencingPassages.empty())
                {
                    continue;
                }

                bool candidateIsBad{};
                if (p_candidate->isBad(candidateIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                Eigen::Vector3d candidateCentroid{};
                if ((!candidateIsBad) &&
                    p_candidate->getCentroid(candidateCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                map::Map *p_candidateMap2 = nullptr;
                if ((!candidateIsBad && candidateCentroid.allFinite()) &&
                    p_candidate->getMap(p_candidateMap2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                map::Map *p_candidateMap3 = nullptr;
                if ((!candidateIsBad && candidateCentroid.allFinite() &&
                     p_candidateMap2 != nullptr) &&
                    p_candidate->getMap(p_candidateMap3) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool atlasIsActiveMap{};
                if ((!candidateIsBad && candidateCentroid.allFinite() &&
                     p_candidateMap2 != nullptr) &&
                    p_atlas->isActiveMap(p_candidateMap3, atlasIsActiveMap) !=
                        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isActiveMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool hasValidGeometry =
                    !candidateIsBad && candidateCentroid.allFinite() &&
                    p_candidateMap2 != nullptr && atlasIsActiveMap;

                for (vs_graphs::core::semantic::Passage *p_passage :
                     referencingPassages)
                {
                    g2o::Plane3D passageGlobalEquation{};
                    if (p_passage->getGlobalEquation(passageGlobalEquation) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGlobalEquation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector4d passageEquation_world =
                        passageGlobalEquation.coeffs();

                    Eigen::Vector3d passageCentroid{};
                    if (p_passage->getCentroid(passageCentroid) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!passageCentroid.allFinite() ||
                        !passageEquation_world.allFinite() ||
                        passageEquation_world.head<3>().norm() < 1e-8)
                    {
                        hasValidGeometry = false;
                        break;
                    }
                }

                bool candidateIsBad2{};
                if (p_candidate->isBad(candidateIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!candidateIsBad2 && !referencingPassages.empty() &&
                    hasValidGeometry)
                {
                    /* Wall count and age are deliberately irrelevant here. */
                    prospectiveRoomCycles[roomId] = 0;
                    continue;
                }

                for (vs_graphs::core::semantic::Passage *p_passage :
                     referencingPassages)
                {
                    if (p_passage->setProspectiveRoom(nullptr) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setProspectiveRoom returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    int clearedPassageId{};
                    if (p_passage->getId(clearedPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                                "[SemMgr] Passage#%d far-side handle Room#%d "
                                "cleared: prospective retired (bad room or "
                                "invalid passage geometry).",
                                clearedPassageId,
                                roomId);
                }

                bool candidateIsBad3{};
                if (p_candidate->isBad(candidateIsBad3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!candidateIsBad3)
                {
                    map::Map *p_candidateMap = nullptr;
                    if (p_candidate->getMap(p_candidateMap) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_candidateMap != nullptr)
                    {
                        unsigned long candidateMapId{};
                        if (p_candidateMap->getId(candidateMapId) !=
                            map::MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        if (p_sceneGraph->eraseRoom(candidateMapId,
                                                    p_candidate) !=
                            scene_graph::SceneGraphStatus::
                                SCENE_GRAPH_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: eraseRoom returned a failure "
                                         "status; continuing as before.",
                                         __func__);
                        }
                    }
                    if (p_candidate->setBad() !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (loggedRoomCleanupIds.insert(roomId).second)
                    {
                        std::cout << "[SemMgr] Cleaning up orphaned "
                                     "prospective Room#"
                                  << roomId
                                  << " (no active passage reference or invalid "
                                     "geometry)."
                                  << std::endl;
                    }
                }

                prospectiveRoomCycles.erase(roomId);
            }
        }

        /* Second pass: associate walls with rooms using free-space
         * clusters. Now that passages exist, partitionFreeSpaceAtPassages()
         * will split clusters at doorways, yielding correct per-room clusters.
         */
        if (p_sysParams->roomSeg.method ==
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            /* Second pass - refreshes wall-to-room associations. */
            if (associateWallsToRoomsFromFreeSpaceClusters() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateRoomsFromFreeSpaceClusters returned a "
                             "failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /* Re-associate walls to rooms after second-pass cluster splitting.
         * Walls assigned in the first pass may belong to wrong (merged)
         * rooms. */
        if (associateAllWallsToRooms() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: associateAllWallsToRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* A wall surface is owned by exactly one room in every configuration.
         * Run AFTER the second pass so split clusters get correct wall
         * ownership. */
        if (enforceUniqueWallOwnership() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: enforceUniqueWallOwnership returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Link each wall's opposite-facing twin, now that ownership has
         * settled for this cycle. Must run before validateRoomBoundaries()
         * so boundary/corner logic can rely on current twin identity. */
        if (reconcileWallFacePairs() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reconcileWallFacePairs returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Validate room geometry without delaying independent passage data. */
        if (validateRoomBoundaries() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validateRoomBoundaries returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Retire only stale, weak wall hypotheses left unused by the graph. */
        if (suppressUndefendedWalls() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: suppressUndefendedWalls returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Recompute room centroids as mean of associated wall centroids.
         * In FREE_SPACE mode the skeleton-cluster centroid set during room
         * detection is the authoritative room centre; overwriting it with the
         * wall-centroid mean drifts rooms away from the cluster each cycle,
         * which breaks the centroid-keyed room matching in associateRooms()
         * and spawns a duplicate room every run. */
        if (p_sysParams->roomSeg.method !=
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            if (recomputeRoomCentroidsFromWalls() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: recomputeRoomCentroidsFromWalls returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        /* Associate every valid room/SE with the floor */
        if (getUpdatedFloors() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getUpdatedFloors returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Re-point any room whose own ground plane disagrees with the
         * just-refreshed canonical Floor identity. */
        if (reconcileRoomGroundPlanes() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: reconcileRoomGroundPlanes returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Room candidate generation is pre-verification only. The legacy
         * tag-and-wall-transfer entry point remains disabled. */

        /* Advance the room-state machine. It consumes the
         * traversal crossings recorded above and reports accepted/rejected
         * transitions. It is read-only with respect to the room id members. */
        const std::chrono::duration<double> roomTrackerElapsed =
            std::chrono::steady_clock::now().time_since_epoch();
        if (updateRoomTrackerState(roomTrackerElapsed.count()) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateRoomTrackerState returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Verification-gated merge is not enabled here. In particular, a tag
         * match must never activate a merge. */
        std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
            copiedContext{};
        if (p_atlas->copyRoomContextHistory(copiedContext) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: copyRoomContextHistory returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_candidateMap = nullptr;
        if (p_atlas->getCurrentMap(p_candidateMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_candidateMap != nullptr)
        {
            std::vector<semantic::RoomContextSnapshot> currentSnapshots{};
            if (p_atlas->copyRoomContextForMap(p_candidateMap,
                                               currentSnapshots) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: copyRoomContextForMap returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            unsigned long candidateMapId{};
            if (p_candidateMap->getId(candidateMapId) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            copiedContext[candidateMapId] = currentSnapshots;
        }
        semantic::SemanticCandidateConfig candidateConfiguration;
        candidateConfiguration.topK = p_sysParams->candidateGen.topK;
        candidateConfiguration.candidatePairCap =
            p_sysParams->candidateGen.candidatePairCap;
        candidateConfiguration.topologyNodesCap =
            p_sysParams->candidateGen.topologyNodesCap;
        candidateConfiguration.globalFallbackCap =
            p_sysParams->candidateGen.globalFallbackCap;
        candidateConfiguration.weightAngle =
            p_sysParams->candidateGen.weightAngle;
        candidateConfiguration.weightExtent =
            p_sysParams->candidateGen.weightExtent;
        candidateConfiguration.weightAperture =
            p_sysParams->candidateGen.weightAperture;
        candidateConfiguration.weightTopology =
            p_sysParams->candidateGen.weightTopology;
        candidateConfiguration.angleMissingPenalty =
            p_sysParams->candidateGen.angleMissingPenalty;
        candidateConfiguration.extentMissingPenalty =
            p_sysParams->candidateGen.extentMissingPenalty;
        candidateConfiguration.apertureMissingPenalty =
            p_sysParams->candidateGen.apertureMissingPenalty;
        candidateConfiguration.ambiguityMargin =
            p_sysParams->candidateGen.ambiguityMargin;
        candidateConfiguration.angleTolerance_rad =
            p_sysParams->candidateGen.angleTolerance_rad;
        candidateConfiguration.runtimeBudget_ms =
            p_sysParams->candidateGen.runtimeBudget_ms;
        candidateConfiguration.descriptorElementsCap =
            p_sysParams->candidateGen.descriptorElementsCap;
        candidateConfiguration.topoRefinementIters =
            p_sysParams->candidateGen.topoRefinementIters;
        /* The "last-confirmed room" anchor for adjacency-
         * prioritised candidate search. -1 (unset) maps to no anchor. */
        int lastKnownRoomIdSnapshot{};
        if (getLastKnownRoomId(lastKnownRoomIdSnapshot) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKnownRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const std::optional<int> anchorRoomId =
            lastKnownRoomIdSnapshot >= 0
                ? std::optional<int>(lastKnownRoomIdSnapshot)
                : std::nullopt;
        std::vector<semantic::SemanticCandidate> candidates{};
        if (semantic::SemanticCandidates::generate(copiedContext,
                                                   candidates,
                                                   candidateConfiguration,
                                                   anchorRoomId) !=
            semantic::SemanticCandidatesStatus::
                SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: generate returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] semantic_candidates count=" << candidates.size()
                  << std::endl;

        /* Run the geometric verifier on the single best candidate and feed
         * the resulting VerificationVerdict to roomTracker via
         * submitVerificationVerdict(). This still only makes the *verdict*
         * real -- it must not trigger a merge or otherwise mutate
         * the Atlas; that trigger is a separate, deliberately gated step. */
        if (evaluateTopCandidateVerification(candidates) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateTopCandidateVerification returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }

        /* Compact Phase-1 heartbeat: all values come from this completed
         * semantic transaction and are therefore mutually consistent. */
        map::Map *p_pipelineMap = nullptr;
        if (p_atlas->getCurrentMap(p_pipelineMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> pipelineMapAllPlanes{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getAllPlanes(pipelineMapAllPlanes) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const std::vector<geometric::Plane *> pipelinePlanes =
            p_pipelineMap != nullptr ? pipelineMapAllPlanes
                                     : std::vector<geometric::Plane *>();
        unsigned long pipelineMapId{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getId(pipelineMapId) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::Room *> pipelineMapAllRooms{};
        if ((p_pipelineMap != nullptr) &&
            p_sceneGraph->getMapRoomsInIdOrder(pipelineMapId,
                                               pipelineMapAllRooms) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapRoomsInIdOrder returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        const std::vector<semantic::Room *> pipelineRooms =
            p_pipelineMap != nullptr ? pipelineMapAllRooms
                                     : std::vector<semantic::Room *>();
        std::vector<vs_graphs::core::semantic::Passage *>
            pipelineMapAllPassages{};
        if ((p_pipelineMap != nullptr) &&
            p_sceneGraph->getMapPassagesInIdOrder(pipelineMapId,
                                                  pipelineMapAllPassages) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPassagesInIdOrder returned a failure "
                         "status; continuing as before.",
                         __func__);
        }
        const std::vector<semantic::Passage *> pipelinePassages =
            p_pipelineMap != nullptr ? pipelineMapAllPassages
                                     : std::vector<semantic::Passage *>();
        std::vector<semantic::Floor *> pipelineMapAllFloors{};
        if ((p_pipelineMap != nullptr) &&
            p_sceneGraph->getMapFloorsInIdOrder(pipelineMapId,
                                                pipelineMapAllFloors) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapFloorsInIdOrder returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        const std::vector<semantic::Floor *> pipelineFloors =
            p_pipelineMap != nullptr ? pipelineMapAllFloors
                                     : std::vector<semantic::Floor *>();
        std::vector<std::vector<Eigen::Vector3d>>
            pipelineMapSkeletonClusterPoints{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getSkeletonClusterPoints(
                pipelineMapSkeletonClusterPoints) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getSkeletonClusterPoints returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        const std::vector<std::vector<Eigen::Vector3d>> pipelineClusters =
            p_pipelineMap != nullptr
                ? pipelineMapSkeletonClusterPoints
                : std::vector<std::vector<Eigen::Vector3d>>();

        std::size_t             wallClassCount      = 0U;
        std::size_t             admissibleWallCount = 0U;
        std::size_t             ownedWallCount      = 0U;
        std::unordered_set<int> ownedWallIds;
        geometric::Plane       *p_pipelineMapBiggestGroundPlane = nullptr;
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getBiggestGroundPlane(
                p_pipelineMapBiggestGroundPlane) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBiggestGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane *p_pipelineGround =
            p_pipelineMap != nullptr ? p_pipelineMapBiggestGroundPlane
                                     : nullptr;
        Eigen::Vector3d pipelineGroundNormal_world = Eigen::Vector3d::Zero();
        bool            pipelineGroundIsBad{};
        if ((p_pipelineGround != nullptr) &&
            p_pipelineGround->isBad(pipelineGroundIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_pipelineGround != nullptr && !pipelineGroundIsBad)
        {
            g2o::Plane3D pipelineGroundGetGlobalEquation{};
            if (p_pipelineGround->getGlobalEquation(
                    pipelineGroundGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector4d equation =
                pipelineGroundGetGlobalEquation.coeffs();
            if (equation.allFinite() && equation.head<3>().norm() > 1e-8)
            {
                pipelineGroundNormal_world = equation.head<3>().normalized();
            }
        }
        for (geometric::Plane *p_plane : pipelinePlanes)
        {
            bool planeIsBad{};
            if (!(p_plane == nullptr) &&
                p_plane->isBad(planeIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            geometric::Plane::PlaneVariant planeType{};
            if (!(p_plane == nullptr || planeIsBad) &&
                p_plane->getPlaneType(planeType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_plane == nullptr || planeIsBad ||
                planeType != geometric::Plane::PlaneVariant::WALL)
            {
                continue;
            }
            wallClassCount++;
            WallAdmissionEvidence admissionEvidence{};
            if (evaluateWallAdmissionEvidence(p_plane,
                                              p_sysParams,
                                              pipelineGroundNormal_world,
                                              admissionEvidence) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: evaluateWallAdmissionEvidence returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            if (admissionEvidence.isAdmissible)
            {
                admissibleWallCount++;
            }
        }
        std::size_t realRoomCount        = 0U;
        std::size_t prospectiveRoomCount = 0U;
        for (semantic::Room *p_room : pipelineRooms)
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
            semantic::Room::RoomVariant roomVariant{};
            if (p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (roomVariant == semantic::Room::RoomVariant::ROOM)
            {
                realRoomCount++;
            }
            else
            {
                prospectiveRoomCount++;
            }
            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                bool wallIsBad{};
                if ((p_wall != nullptr) &&
                    p_wall->isBad(wallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_wall != nullptr && !wallIsBad)
                {
                    int wallGetId{};
                    if (p_wall->getId(wallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    ownedWallIds.insert(wallGetId);
                }
            }
        }
        ownedWallCount                  = ownedWallIds.size();
        std::size_t skeletonVertexCount = 0U;
        for (const std::vector<Eigen::Vector3d> &cluster : pipelineClusters)
        {
            skeletonVertexCount += cluster.size();
        }
        const std::size_t pendingWallCount = std::count_if(
            undefendedWalls.begin(),
            undefendedWalls.end(),
            [&ownedWallIds](
                const std::pair<const int, UndefendedWallState> &entry)
            { return ownedWallIds.count(entry.first) == 0U; });
        const std::size_t livePassageCount = std::count_if(
            pipelinePassages.begin(),
            pipelinePassages.end(),
            [](semantic::Passage *p_passage)
            {
                bool passageIsBad{};
                if ((p_passage != nullptr) &&
                    p_passage->isBad(passageIsBad) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return p_passage != nullptr && !passageIsBad;
            });
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            pipelineMapSkeletonEdges{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getSkeletonEdges(pipelineMapSkeletonEdges) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getSkeletonEdges returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int getCurrentRoomId2{};
        if (getCurrentRoomId(getCurrentRoomId2) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "SG_PIPELINE {\"event\":\"heartbeat\",\"map_id\":"
                  << (p_pipelineMap != nullptr
                          ? static_cast<long long>(pipelineMapId)
                          : -1)
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle
                  << ",\"current_room_id\":" << getCurrentRoomId2
                  << ",\"raw_planes\":" << pipelinePlanes.size()
                  << ",\"wall_class_planes\":" << wallClassCount
                  << ",\"admissible_walls\":" << admissibleWallCount
                  << ",\"pending_walls\":" << pendingWallCount
                  << ",\"owned_walls\":" << ownedWallCount
                  << ",\"skeleton_clusters\":" << pipelineClusters.size()
                  << ",\"skeleton_vertices\":" << skeletonVertexCount
                  << ",\"skeleton_edges\":"
                  << (p_pipelineMap != nullptr ? pipelineMapSkeletonEdges.size()
                                               : 0U)
                  << ",\"real_rooms\":" << realRoomCount
                  << ",\"prospective_rooms\":" << prospectiveRoomCount
                  << ",\"passages\":" << livePassageCount
                  << ",\"floors\":" << pipelineFloors.size() << "}"
                  << std::endl;

        /* ------------------------------------------------------------------ *
         * SEMANTIC MONITOR BOUNDARY: capture a complete, pointer-free
         * snapshot plus manager-private evidence while the semantic-update
         * lock is still held, unlock, then evaluate/digest/cache/log
         * outside the lock. Read-only with respect to inference,
         * ownership, passage, room, and completeness decisions -- this
         * never mutates Atlas/Map/Room/Wall/Passage state.
         * ------------------------------------------------------------------ */
        const std::uint64_t semanticCycle = pipelineSemanticCycle;

        semantic::SemanticGraphSnapshot snapshot{};
        if (semantic::captureSemanticGraphSnapshot(p_atlas, snapshot) !=
            semantic::SemanticGraphSnapshotStatus::
                SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureSemanticGraphSnapshot returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        std::vector<semantic::UnresolvedWallHypothesisRecord>
            captureUnresolvedWallHypotheses2{};
        if (captureUnresolvedWallHypotheses(captureUnresolvedWallHypotheses2) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureUnresolvedWallHypotheses returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        snapshot.managerPrivateUnresolvedWallHypotheses =
            captureUnresolvedWallHypotheses2;
        snapshot.managerPrivateUnresolvedWallHypothesesReason =
            semantic::UnavailableReason::NONE;

        std::optional<int> currentMapRevision;
        if (snapshot.currentMapId.has_value())
        {
            map::Map *p_currentMap = nullptr;
            if (p_atlas->getCurrentMap(p_currentMap) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long currentMapId2{};
            if ((p_currentMap != nullptr) &&
                p_currentMap->getId(currentMapId2) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap != nullptr &&
                currentMapId2 == *snapshot.currentMapId)
            {
                int currentMapMapChangeIndex{};
                if (p_currentMap->getMapChangeIndex(currentMapMapChangeIndex) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMapChangeIndex returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                currentMapRevision = currentMapMapChangeIndex;
            }
        }

        /* Deletion ends the cycle: flagged rooms, passages and storeys leave
         * the scene graph and are freed here, under the same lock that every
         * writer holds, so no cycle step can still use them. */
        if (p_sceneGraph->sweepBadNodes() !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sweepBadNodes returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Unlock before any evaluation/serialization/caching/logging work --
         * the monitor must never hold the semantic-update lock while doing
         * read-only diagnostic work. */
        semanticUpdateLock.unlock();

        const std::chrono::steady_clock::time_point evaluationStart =
            std::chrono::steady_clock::now();
        semantic::AxiomEvaluationReport evaluationReport{};
        if (semantic::evaluateState(snapshot, evaluationReport) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: evaluateState returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::MapCompletenessResult> completenessResults{};
        if (semantic::evaluateMapCompleteness(snapshot, completenessResults) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateMapCompleteness returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        std::string topologyDigest{};
        if (semantic::sha256HexDigest(
                semantic::serializeSnapshotTopologyOnly(snapshot).dump(),
                topologyDigest) !=
            semantic::Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sha256HexDigest returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string fullGeometryDigest{};
        if (semantic::sha256HexDigest(
                semantic::serializeSnapshotFullGeometry(snapshot).dump(),
                fullGeometryDigest) !=
            semantic::Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sha256HexDigest returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const std::chrono::milliseconds evaluationDuration =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - evaluationStart);

        if (semanticReportCache.update(snapshot,
                                       evaluationReport,
                                       completenessResults,
                                       semanticCycle,
                                       snapshot.currentMapId,
                                       currentMapRevision,
                                       topologyDigest,
                                       fullGeometryDigest,
                                       evaluationDuration) !=
            semantic::SemanticReportCacheStatus::
                SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        semantic::SemanticReportCacheEntry semanticReportCacheGetLatest{};
        if (semanticReportCache.getLatest(semanticReportCacheGetLatest) !=
            semantic::SemanticReportCacheStatus::
                SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLatest returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (logSemanticDiagnostics(semanticReportCacheGetLatest) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: logSemanticDiagnostics returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Find the time after it took to run the loop */
        const std::chrono::steady_clock::time_point end =
            std::chrono::steady_clock::now();

        /* Calculate the elapsed time */
        const std::chrono::duration<double> elapsed = end - start;

        /* Find how much longer in the loop is left */
        const double remainingSeconds =
            p_sysParams->semanticsManager.runInterval_s - elapsed.count();

        /* If there is remaining time, sleep until next loop cycle */
        if (remainingSeconds > 0.0)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(remainingSeconds));
        }
        else
        {
            /* Let a waiting merge or segmentation transaction acquire next. */
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /* Signal shutdown completion to ~System. */
    if (setFinish() != SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
