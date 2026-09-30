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

#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticCandidates.h"
#include "Semantic/SemanticCanonicalSerialization.h"
#include "Semantic/SemanticGraphSnapshot.h"
#include "Semantic/Sha256Digest.h"

#include "../private_functions.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <rclcpp/logging.hpp>
#include <thread>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::run(void)
{
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
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        pipelineSemanticCycle  = ++summaryCycle;
        Map *p_atlasCurrentMap = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
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

        /* Validate the low-level semantic planes */
        geometric::Plane *p_mainGroundPlane = nullptr;
        if (p_atlas->getBiggestGroundPlane(p_mainGroundPlane) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
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

            /* Filter ground planes */
            if (filterGroundPlanes(p_mainGroundPlane) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: filterGroundPlanes returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Filter the wall planes */
            if (filterWallPlanes() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: filterWallPlanes returned a failure status "
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
         * Use free-space evidence to create and update rooms.
         *
         * @note         This is the preferred wall-to-room association method.
         */
        if (p_sysParams->roomSeg.method ==
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            if (detectRoom_FreeSpaceCluster() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: detectRoom_FreeSpaceCluster returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        /*!
         * Enforce the semantic hierarchy.
         *
         * @note        wall which was not captured by the free-space room
         *              detector receives either an existing room or a new
         *              provisional structural element.
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

            /* PROSPECTIVE ROOM CLEANUP
             * A passage reference is the stable far-side handle. Zero-wall
             * prospectives remain alive while that reference and its geometry
             * are valid; only orphaned or invalid handles are retired. */
            std::vector<vs_graphs::core::semantic::Room *> candidateRooms{};
            if (p_atlas->getAllCandidateMapRooms(candidateRooms) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getAllCandidateMapRooms returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
            if (p_atlas->getAllPassages(allPassages) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
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
                core::Map *p_candidateMap2 = nullptr;
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
                core::Map *p_candidateMap3 = nullptr;
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
                        AtlasStatus::ATLAS_STATUS_SUCCESS)
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
                    const Eigen::Vector4d passageEquation_World =
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
                        !passageEquation_World.allFinite() ||
                        passageEquation_World.head<3>().norm() < 1e-8)
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
                    Map *p_candidateMap = nullptr;
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
                        if (p_candidateMap->eraseMarkerBasedMapRoom(
                                p_candidate) != MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: eraseMarkerBasedMapRoom returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
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
                                     "prospective semantic::Room#"
                                  << roomId
                                  << " (no active passage reference or invalid "
                                     "geometry)."
                                  << std::endl;
                    }
                }

                prospectiveRoomCycles.erase(roomId);
            }
        }

        /* Second pass: re-detect rooms from passage-partitioned free-space
         * clusters. Now that passages exist, partitionFreeSpaceAtPassages()
         * will split clusters at doorways, yielding correct per-room clusters.
         */
        if (p_sysParams->roomSeg.method ==
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            if (detectRoom_FreeSpaceCluster() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: detectRoom_FreeSpaceCluster returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            } // Second pass - updates existing
              // rooms
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
         * match must never activate MergeMapPair(). */
        std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
            copiedContext{};
        if (p_atlas->copyRoomContextHistory(copiedContext) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: copyRoomContextHistory returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_candidateMap = nullptr;
        if (p_atlas->getCurrentMap(p_candidateMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_candidateMap != nullptr)
        {
            Atlas::SnapshotCopyResult currentSnapshot{};
            if (p_atlas->copyRoomContextForMapChecked(p_candidateMap,
                                                      true,
                                                      currentSnapshot) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: copyRoomContextForMapChecked returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            if (currentSnapshot.status == Atlas::SnapshotCopyStatus::COMPLETE)
            {
                unsigned long candidateMapId{};
                if (p_candidateMap->getId(candidateMapId) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                copiedContext[candidateMapId] = currentSnapshot.snapshots;
            }
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
         * real -- it must not call Atlas::MergeMapPair() or otherwise mutate
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
        Map *p_pipelineMap = nullptr;
        if (p_atlas->getCurrentMap(p_pipelineMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> pipelineMapAllPlanes{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getAllPlanes(pipelineMapAllPlanes) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const std::vector<geometric::Plane *> pipelinePlanes =
            p_pipelineMap != nullptr ? pipelineMapAllPlanes
                                     : std::vector<geometric::Plane *>();
        std::vector<semantic::Room *> pipelineMapAllRooms{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getAllRooms(pipelineMapAllRooms) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const std::vector<semantic::Room *> pipelineRooms =
            p_pipelineMap != nullptr ? pipelineMapAllRooms
                                     : std::vector<semantic::Room *>();
        std::vector<vs_graphs::core::semantic::Passage *>
            pipelineMapAllPassages{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getAllPassages(pipelineMapAllPassages) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const std::vector<semantic::Passage *> pipelinePassages =
            p_pipelineMap != nullptr ? pipelineMapAllPassages
                                     : std::vector<semantic::Passage *>();
        std::vector<semantic::Floor *> pipelineMapAllFloors{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getAllFloors(pipelineMapAllFloors) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
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
                MapStatus::MAP_STATUS_SUCCESS)
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
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBiggestGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane *p_pipelineGround =
            p_pipelineMap != nullptr ? p_pipelineMapBiggestGroundPlane
                                     : nullptr;
        Eigen::Vector3d pipelineGroundNormal_World = Eigen::Vector3d::Zero();
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
                pipelineGroundNormal_World = equation.head<3>().normalized();
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
                                              pipelineGroundNormal_World,
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
        unsigned long pipelineMapId{};
        if ((p_pipelineMap != nullptr) && p_pipelineMap->getId(pipelineMapId) !=
                                              MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            pipelineMapSkeletonEdges{};
        if ((p_pipelineMap != nullptr) &&
            p_pipelineMap->getSkeletonEdges(pipelineMapSkeletonEdges) !=
                MapStatus::MAP_STATUS_SUCCESS)
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
        std::vector<semantic::OpenPassageHypothesisRecord>
            captureOpenPassageHypotheses2{};
        if (captureOpenPassageHypotheses(captureOpenPassageHypotheses2) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureOpenPassageHypotheses returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        snapshot.managerPrivateOpenPassageHypotheses =
            captureOpenPassageHypotheses2;
        snapshot.managerPrivateOpenPassageHypothesesReason =
            semantic::UnavailableReason::NONE;
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
            Map *p_currentMap = nullptr;
            if (p_atlas->getCurrentMap(p_currentMap) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long currentMapId2{};
            if ((p_currentMap != nullptr) &&
                p_currentMap->getId(currentMapId2) !=
                    MapStatus::MAP_STATUS_SUCCESS)
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
                    MapStatus::MAP_STATUS_SUCCESS)
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

        /* Unlock before any evaluation/serialization/caching/logging work --
         * the monitor must never hold the semantic-update lock while doing
         * read-only diagnostic work. */
        /* Continuous consecutive-map matching, old into current, inside
         * this transaction: at most one merge per cycle; attempts and
         * commits log via SG_PIPELINE. */
        if (p_atlas->attemptConsecutiveMergeIfGated() !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: attemptConsecutiveMergeIfGated returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
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
        const double remainingSeconds = runInterval_s - elapsed.count();

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

} // namespace core
} // namespace vs_graphs
