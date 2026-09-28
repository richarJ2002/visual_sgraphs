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

#include <algorithm>
#include <chrono>
#include <map>
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
        if (checkFinish())
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
        std::unique_lock<std::mutex> semanticUpdateLock =
            p_atlas->acquireSemanticUpdateLock();

        pipelineSemanticCycle = ++summaryCycle;
        resetTemporalStateForMap(p_atlas->getCurrentMap());
        ensureActiveMapBootstrapHierarchy();

        /* Validate the low-level semantic planes */
        geometric::Plane *p_mainGroundPlane = p_atlas->getBiggestGroundPlane();

        /* If there is a ground plane, find its transform and filter planes */
        if (p_mainGroundPlane != nullptr)
        {
            /* Find the transform from ground plane to horizontal */
            planePoseMat = computePlaneToHorizontal(p_mainGroundPlane);

            /* Filter ground planes */
            filterGroundPlanes(p_mainGroundPlane);

            /* Filter the wall planes */
            filterWallPlanes();
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
                // reAssociateSemanticPlanes cannot fail; continue as before.
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
            detectDoorsAndDoorways(p_atlas);
            updatePassages(p_atlas);
            mergeOverlappingPassages();
        }

        /*!
         * Use free-space evidence to create and update rooms.
         *
         * @note         This is the preferred wall-to-room association method.
         */
        if (p_sysParams->roomSeg.method ==
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            detectRoom_FreeSpaceCluster();
        }

        /*!
         * Enforce the semantic hierarchy.
         *
         * @note        wall which was not captured by the free-space room
         *              detector receives either an existing room or a new
         *              provisional structural element.
         */
        associateAllWallsToRooms();

        /* Consolidate only redundant single-wall provisional structures. */
        if (utils::utils::Utils::reAssociateRooms(p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // reAssociateRooms cannot fail; continue as before.
        }

        /*  Room-dependent passage steps: geometry was already refreshed
         * above, ahead of this cycle's wall admission. */
        if (p_sysParams->semSeg.enablePassageDetection)
        {
            updateTraversalEvidence(p_atlas);
            if (utils::utils::Utils::reAssociatePassages(p_atlas) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // reAssociatePassages cannot fail; continue as before.
            }
            associatePassagesToRooms();
            detachWallsBeyondConfirmedPassages();

            /* Continuous rule-invariant sweep (not just at admission time):
             * see enforcePassageSideInvariant()'s own comment. Room<->passage
             * association is current as of the two calls just above. */
            enforcePassageSideInvariant();

            /* PROSPECTIVE ROOM CLEANUP
             * A passage reference is the stable far-side handle. Zero-wall
             * prospectives remain alive while that reference and its geometry
             * are valid; only orphaned or invalid handles are retired. */
            const std::vector<vs_graphs::core::semantic::Room *>
                candidateRooms = p_atlas->getAllCandidateMapRooms();
            const std::vector<vs_graphs::core::semantic::Passage *>
                allPassages = p_atlas->getAllPassages();
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
                    // getId cannot fail; continue as before.
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
                        // isBad cannot fail; continue as before.
                    }
                    vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                        nullptr;
                    if ((p_passage != nullptr && !passageIsBad) &&
                        p_passage->getProspectiveRoom(
                            p_passageProspectiveRoom) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getProspectiveRoom cannot fail; continue as before.
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
                    // isBad cannot fail; continue as before.
                }
                Eigen::Vector3d candidateCentroid{};
                if ((!candidateIsBad) &&
                    p_candidate->getCentroid(candidateCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getCentroid cannot fail; continue as before.
                }
                core::Map *p_candidateMap2 = nullptr;
                if ((!candidateIsBad && candidateCentroid.allFinite()) &&
                    p_candidate->getMap(p_candidateMap2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getMap cannot fail; continue as before.
                }
                core::Map *p_candidateMap3 = nullptr;
                if ((!candidateIsBad && candidateCentroid.allFinite() &&
                     p_candidateMap2 != nullptr) &&
                    p_candidate->getMap(p_candidateMap3) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getMap cannot fail; continue as before.
                }
                bool hasValidGeometry = !candidateIsBad &&
                                        candidateCentroid.allFinite() &&
                                        p_candidateMap2 != nullptr &&
                                        p_atlas->isActiveMap(p_candidateMap3);

                for (vs_graphs::core::semantic::Passage *p_passage :
                     referencingPassages)
                {
                    g2o::Plane3D passageGlobalEquation{};
                    if (p_passage->getGlobalEquation(passageGlobalEquation) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getGlobalEquation cannot fail; continue as before.
                    }
                    const Eigen::Vector4d passageEquation_World =
                        passageGlobalEquation.coeffs();

                    Eigen::Vector3d passageCentroid{};
                    if (p_passage->getCentroid(passageCentroid) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getCentroid cannot fail; continue as before.
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
                    // isBad cannot fail; continue as before.
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
                        // setProspectiveRoom cannot fail; continue as before.
                    }
                }

                bool candidateIsBad3{};
                if (p_candidate->isBad(candidateIsBad3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (!candidateIsBad3)
                {
                    Map *p_candidateMap = nullptr;
                    if (p_candidate->getMap(p_candidateMap) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getMap cannot fail; continue as before.
                    }
                    if (p_candidateMap != nullptr)
                    {
                        p_candidateMap->eraseMarkerBasedMapRoom(p_candidate);
                    }
                    if (p_candidate->setBad() !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // setBad cannot fail; continue as before.
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
            detectRoom_FreeSpaceCluster(); // Second pass - updates existing
                                           // rooms
        }

        /* Re-associate walls to rooms after second-pass cluster splitting.
         * Walls assigned in the first pass may belong to wrong (merged)
         * rooms. */
        associateAllWallsToRooms();

        /* A wall surface is owned by exactly one room in every configuration.
         * Run AFTER the second pass so split clusters get correct wall
         * ownership. */
        enforceUniqueWallOwnership();

        /* Link each wall's opposite-facing twin, now that ownership has
         * settled for this cycle. Must run before validateRoomBoundaries()
         * so boundary/corner logic can rely on current twin identity. */
        reconcileWallFacePairs();

        /* Validate room geometry without delaying independent passage data. */
        validateRoomBoundaries();

        /* Retire only stale, weak wall hypotheses left unused by the graph. */
        suppressUndefendedWalls();

        /* Recompute room centroids as mean of associated wall centroids.
         * In FREE_SPACE mode the skeleton-cluster centroid set during room
         * detection is the authoritative room centre; overwriting it with the
         * wall-centroid mean drifts rooms away from the cluster each cycle,
         * which breaks the centroid-keyed room matching in associateRooms()
         * and spawns a duplicate room every run. */
        if (p_sysParams->roomSeg.method !=
            types::SystemParams::RoomSeg::Method::FREE_SPACE)
        {
            recomputeRoomCentroidsFromWalls();
        }

        /* Associate every valid room/SE with the floor */
        getUpdatedFloors();

        /* Re-point any room whose own ground plane disagrees with the
         * just-refreshed canonical Floor identity. */
        reconcileRoomGroundPlanes();

        /* Room candidate generation is pre-verification only. The legacy
         * tag-and-wall-transfer entry point remains disabled. */

        /* Advance the room-state machine. It consumes the
         * traversal crossings recorded above and reports accepted/rejected
         * transitions. It is read-only with respect to the room id members. */
        const std::chrono::duration<double> roomTrackerElapsed =
            std::chrono::steady_clock::now().time_since_epoch();
        updateRoomTrackerState(roomTrackerElapsed.count());

        /* Verification-gated merge is not enabled here. In particular, a tag
         * match must never activate MergeMapPair(). */
        std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
             copiedContext  = p_atlas->copyRoomContextHistory();
        Map *p_candidateMap = p_atlas->getCurrentMap();
        if (p_candidateMap != nullptr)
        {
            const Atlas::SnapshotCopyResult currentSnapshot =
                p_atlas->copyRoomContextForMapChecked(p_candidateMap, true);
            if (currentSnapshot.status == Atlas::SnapshotCopyStatus::COMPLETE)
            {
                copiedContext[p_candidateMap->getId()] =
                    currentSnapshot.snapshots;
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
        const int                lastKnownRoomIdSnapshot = getLastKnownRoomId();
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
            // generate cannot fail; continue as before.
        }
        std::cout << "[SemMgr] semantic_candidates count=" << candidates.size()
                  << std::endl;

        /* Run the geometric verifier on the single best candidate and feed
         * the resulting VerificationVerdict to roomTracker via
         * submitVerificationVerdict(). This still only makes the *verdict*
         * real -- it must not call Atlas::MergeMapPair() or otherwise mutate
         * the Atlas; that trigger is a separate, deliberately gated step. */
        evaluateTopCandidateVerification(candidates);

        /* Compact Phase-1 heartbeat: all values come from this completed
         * semantic transaction and are therefore mutually consistent. */
        Map *p_pipelineMap = p_atlas->getCurrentMap();
        const std::vector<geometric::Plane *> pipelinePlanes =
            p_pipelineMap != nullptr ? p_pipelineMap->getAllPlanes()
                                     : std::vector<geometric::Plane *>();
        const std::vector<semantic::Room *> pipelineRooms =
            p_pipelineMap != nullptr ? p_pipelineMap->getAllRooms()
                                     : std::vector<semantic::Room *>();
        const std::vector<semantic::Passage *> pipelinePassages =
            p_pipelineMap != nullptr ? p_pipelineMap->getAllPassages()
                                     : std::vector<semantic::Passage *>();
        const std::vector<semantic::Floor *> pipelineFloors =
            p_pipelineMap != nullptr ? p_pipelineMap->getAllFloors()
                                     : std::vector<semantic::Floor *>();
        const std::vector<std::vector<Eigen::Vector3d>> pipelineClusters =
            p_pipelineMap != nullptr
                ? p_pipelineMap->getSkeletonClusterPoints()
                : std::vector<std::vector<Eigen::Vector3d>>();

        std::size_t             wallClassCount      = 0U;
        std::size_t             admissibleWallCount = 0U;
        std::size_t             ownedWallCount      = 0U;
        std::unordered_set<int> ownedWallIds;
        geometric::Plane       *p_pipelineGround =
            p_pipelineMap != nullptr ? p_pipelineMap->getBiggestGroundPlane()
                                           : nullptr;
        Eigen::Vector3d pipelineGroundNormal_World = Eigen::Vector3d::Zero();
        if (p_pipelineGround != nullptr && !p_pipelineGround->isBad())
        {
            const Eigen::Vector4d equation =
                p_pipelineGround->getGlobalEquation().coeffs();
            if (equation.allFinite() && equation.head<3>().norm() > 1e-8)
            {
                pipelineGroundNormal_World = equation.head<3>().normalized();
            }
        }
        for (geometric::Plane *p_plane : pipelinePlanes)
        {
            if (p_plane == nullptr || p_plane->isBad() ||
                p_plane->getPlaneType() != geometric::Plane::PlaneVariant::WALL)
            {
                continue;
            }
            wallClassCount++;
            if (evaluateWallAdmissionEvidence(p_plane,
                                              p_sysParams,
                                              pipelineGroundNormal_World)
                    .isAdmissible)
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
                // isBad cannot fail; continue as before.
            }
            if (p_room == nullptr || roomIsBad)
            {
                continue;
            }
            semantic::Room::RoomVariant roomVariant{};
            if (p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomVariant cannot fail; continue as before.
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
                // getWalls cannot fail; continue as before.
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    ownedWallIds.insert(p_wall->getId());
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
                    // isBad cannot fail; continue as before.
                }
                return p_passage != nullptr && !passageIsBad;
            });
        std::cout << "SG_PIPELINE {\"event\":\"heartbeat\",\"map_id\":"
                  << (p_pipelineMap != nullptr
                          ? static_cast<long long>(p_pipelineMap->getId())
                          : -1)
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle
                  << ",\"current_room_id\":" << getCurrentRoomId()
                  << ",\"raw_planes\":" << pipelinePlanes.size()
                  << ",\"wall_class_planes\":" << wallClassCount
                  << ",\"admissible_walls\":" << admissibleWallCount
                  << ",\"pending_walls\":" << pendingWallCount
                  << ",\"owned_walls\":" << ownedWallCount
                  << ",\"skeleton_clusters\":" << pipelineClusters.size()
                  << ",\"skeleton_vertices\":" << skeletonVertexCount
                  << ",\"skeleton_edges\":"
                  << (p_pipelineMap != nullptr
                          ? p_pipelineMap->getSkeletonEdges().size()
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
            // captureSemanticGraphSnapshot cannot fail; continue as before.
        }
        snapshot.managerPrivateOpenPassageHypotheses =
            captureOpenPassageHypotheses();
        snapshot.managerPrivateOpenPassageHypothesesReason =
            semantic::UnavailableReason::NONE;
        snapshot.managerPrivateUnresolvedWallHypotheses =
            captureUnresolvedWallHypotheses();
        snapshot.managerPrivateUnresolvedWallHypothesesReason =
            semantic::UnavailableReason::NONE;

        std::optional<int> currentMapRevision;
        if (snapshot.currentMapId.has_value())
        {
            Map *p_currentMap = p_atlas->getCurrentMap();
            if (p_currentMap != nullptr &&
                p_currentMap->getId() == *snapshot.currentMapId)
            {
                currentMapRevision = p_currentMap->getMapChangeIndex();
            }
        }

        /* Unlock before any evaluation/serialization/caching/logging work --
         * the monitor must never hold the semantic-update lock while doing
         * read-only diagnostic work. */
        /* Continuous consecutive-map matching, old into current, inside
         * this transaction: at most one merge per cycle; attempts and
         * commits log via SG_PIPELINE. */
        p_atlas->attemptConsecutiveMergeIfGated();
        semanticUpdateLock.unlock();

        const std::chrono::steady_clock::time_point evaluationStart =
            std::chrono::steady_clock::now();
        semantic::AxiomEvaluationReport evaluationReport{};
        if (semantic::evaluateState(snapshot, evaluationReport) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // evaluateState cannot fail; continue as before.
        }
        std::vector<semantic::MapCompletenessResult> completenessResults{};
        if (semantic::evaluateMapCompleteness(snapshot, completenessResults) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // evaluateMapCompleteness cannot fail; continue as before.
        }
        std::string topologyDigest{};
        if (semantic::sha256HexDigest(
                semantic::serializeSnapshotTopologyOnly(snapshot).dump(),
                topologyDigest) !=
            semantic::Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
        {
            // sha256HexDigest cannot fail; continue as before.
        }
        std::string fullGeometryDigest{};
        if (semantic::sha256HexDigest(
                semantic::serializeSnapshotFullGeometry(snapshot).dump(),
                fullGeometryDigest) !=
            semantic::Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
        {
            // sha256HexDigest cannot fail; continue as before.
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
            // update cannot fail; continue as before.
        }

        semantic::SemanticReportCacheEntry semanticReportCacheGetLatest{};
        if (semanticReportCache.getLatest(semanticReportCacheGetLatest) !=
            semantic::SemanticReportCacheStatus::
                SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
        {
            // getLatest cannot fail; continue as before.
        }
        logSemanticDiagnostics(semanticReportCacheGetLatest);

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
    setFinish();
}

} // namespace core
} // namespace vs_graphs
