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
 * @file            SemanticsManagerClass.h
 *
 * @brief           Declares SemanticsManager, the worker thread that builds and
 *                  maintains the walls, rooms, floors and passages of the map.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SEMANTICS_MANAGER_CLASS_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SEMANTICS_MANAGER_CLASS_H

#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "scene_understanding/scene_graph/scene_graph/objects/RetiredNodesStruct.h"
#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphStruct.h"
#include "scene_understanding/semantic/room_tracker/room_tracker.h"
#include "scene_understanding/semantic/semantic_candidates/semantic_candidates.h"
#include "scene_understanding/semantic/semantic_diagnostics/semantic_diagnostics.h"
#include "scene_understanding/semantic/semantic_report_cache/semantic_report_cache.h"
#include "scene_understanding/semantics_manager/objects/AbsorbedSceneGraphContentsStruct.h"
#include "scene_understanding/semantics_manager/objects/GraphLockModeEnum.h"
#include "scene_understanding/semantics_manager/objects/GraphLockScopeStruct.h"
#include "scene_understanding/semantics_manager/objects/PendingOpeningStruct.h"
#include "scene_understanding/semantics_manager/objects/PoseCorrectionFieldStruct.h"
#include "scene_understanding/semantics_manager/objects/RoomListEnum.h"
#include "scene_understanding/semantics_manager/objects/RoomSnapshotStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphSnapshotStruct.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/WallGapStruct.h"
#include "scene_understanding/semantics_manager/objects/WallSlabProfileStruct.h"
#include "scene_understanding/semantics_manager/objects/WallSlabQueryStruct.h"

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logging.hpp>
#include <set>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs::core::atlas
{
class Atlas;
}

namespace vs_graphs::core::map
{
class Map;
}

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Reasons about the map semantically on its own thread: builds
 *                  rooms, passages and floors from the walls, doors and
 *                  free-space evidence in the atlas, and tracks which room the
 *                  camera is in.
 *
 *                  It also serves the S-Graph of every SLAM map: map ->
 *                  building -> storey -> rooms and passages.
 *
 *                  Each Map owns its S-Graph content (Map::getSceneGraph());
 * the manager only finds it by map id and adds the shared lock. The atlas
 * registers a map's scene graph when it creates or loads the map and
 * unregisters it when it drops the map. The S-Graph is the only list of the
 * map's rooms, storeys and passages and it owns them: a scene graph owns its
 * rooms and passages, a building owns its storeys. They are added and removed
 * through the graph mutations only, which also enforce the id rules.
 *
 *                  Deletion. markRoomBad() flags a node; every query hides a
 *                  flagged node, so to readers it is gone at once. Nothing is
 *                  freed on the spot: sweepBadNodes(), called once at the end
 *                  of each semantics cycle, retires flagged nodes, unlinks
 *                  every holder and frees them, all under the exclusive graph
 *                  locks. Planes belong to the SLAM map and are only unlinked.
 *
 *                  Thread safety of the graph: every SLAM map's graph has one
 *                  std::shared_mutex (SceneGraph::graphMutex). Every public
 *                  graph query takes it shared and every public graph
 *                  mutation takes it exclusive, inside the method, and the
 *                  pointers it returns are valid only while the caller holds
 *                  a GraphLockScope on that graph (lockGraphs(),
 *                  lockAllGraphs()); the scope is re-entrant per thread.
 *                  Code outside the manager that must keep node pointers
 *                  across calls takes a scope first, or asks for plain-data
 *                  snapshots (getMapRoomSnapshots()). The cycle, a merge and
 *                  every other writer hold the exclusive scope for their whole
 *                  edit. Lock order: semantic update lock, then graph locks in
 *                  ascending map id, then atlasMutex, Map::mapMutex,
 *                  plane-geometry and room-context mutexes. The registry mutex
 *                  is a leaf. Tracking threads never wait for a graph lock:
 *                  they use GRAPH_LOCK_MODE_TRY_SHARED or no lock.
 */
class SemanticsManager
{
  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           This member contains the address of semantic map.
     */
    atlas::Atlas *p_atlas;

    /*!
     * @brief           Serializes updates to the set of newly detected rooms.
     */
    std::mutex newRoomsMutex;

    /*!
     * @brief           Serializes updates to the current/last-known room ids.
     */
    mutable std::mutex currentRoomMutex;

    /* Shutdown control (LocalMapping-style handshake) */
    /*!
     * @brief           Protects isFinishRequested and hasFinished.
     */
    std::mutex finishMutex;

    /*!
     * @brief           Set by requestFinish() to ask run() to leave its loop.
     */
    bool isFinishRequested = false;

    /*!
     * @brief           Set by setFinish() once run() has left its loop.
     */
    bool hasFinished = false;

    /*!
     * @brief           Tells whether a shutdown has been requested.
     *
     * @param[out]      isFinishRequested_out
     *                  True once requestFinish() has been called.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus
        checkFinish(bool &isFinishRequested_out);

    /*!
     * @brief           Marks the semantic thread as finished; called by run()
     *                  when it leaves its loop.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus setFinish();

    /*!
     * @brief           Copied-value cache of the latest complete semantic
     *                  evaluation cycle, published for any reader thread (a ROS
     *                  service callback in particular) without touching the
     *                  semantic-update lock.
     */
    semantic::SemanticReportCache semanticReportCache;

    /*!
     * @brief           Caller-owned state carried across
     *                  logSemanticDiagnostics() calls so
     *                  semantic::buildSemanticDiagnosticUpdate() (a pure,
     *                  directly tested module) can detect an
     *                  appeared/changed/resolved FAIL-finding transition or a
     *                  topology digest change, and pace the heartbeat.
     */
    semantic::SemanticDiagnosticState semanticDiagnosticState;

    /*!
     * @brief           Room-state machine implementing the transition
     *                  table.
     *
     *                  The tracker is a parallel, read-only observer of
     *                  the currentRoomId/lastKnownRoomId bookkeeping;
     *                  it does not move those ids, it just records state
     *                  transitions and guards.
     */
    semantic::RoomTracker roomTracker;

    /*!
     * @brief           Queued typed result from the future geometric verifier.
     *
     *                  The mutex below protects both the pending flag and
     *                  value.  A producer publishes one result; the semantic
     *                  thread consumes it once.  No room geometry or identity
     *                  is inferred when the flag is clear.
     */
    semantic::VerificationVerdict verificationVerdict{};

    /*!
     * @brief           True while verificationVerdict holds a result that
     *                  updateRoomTrackerState() has not consumed yet; guarded
     *                  by currentRoomMutex.
     */
    bool isVerificationVerdictPending = false;

    /*!
     * @brief           Set by updateTraversalEvidence() when a passable passage
     *                  crossing was observed during this semantic cycle.
     */
    bool isCrossingEventPending = false;

    /*!
     * @brief           Set together with isCrossingEventPending when the
     *                  crossed passage has traversal evidence in both
     *                  directions; consumed and cleared by
     *                  updateRoomTrackerState(). Guarded by currentRoomMutex.
     */
    bool isCrossingBothSidesPending = false;

    /*!
     * @brief           Test-only seam, always declared so production and test
     *                  builds share one object layout; it stays empty outside
     *                  tests, where updateTraversalEvidence() only moves an
     *                  empty std::function. The callback is deliberately
     *                  invoked while currentRoomMutex is held so the
     *                  integration test proves real producer and consumer
     *                  contention; it must not call back into this manager
     *                  except through the non-blocking contention probe.
     */
    std::function<void()> roomTrackerPendingPublishHook;

    /*!
     * @brief           Set by onTrackingLost() (once per loss episode) and
     *                  consumed by the room tracker during the next Run cycle.
     */
    bool isTrackingLostPending = false;

    /*!
     * @brief           True from onTrackingLost() until onTrackingRecovered(),
     *                  so a loss is reported only once per episode.
     */
    bool isTrackingLossEpisodeActive = false;

    /*!
     * @brief           True when a new-map event was held back for the next
     *                  cycle because tracking was reported lost in the same
     *                  cycle.
     */
    bool isNewMapCreatedDeferred = false;

    /*!
     * @brief           True while a new-map event taken from the atlas has not
     *                  yet been committed by the room tracker.
     */
    bool isNewMapCreatedPending = false;

    /*!
     * @brief           Id of the room the camera most recently occupied.
     *
     *                  -1 until the first confirmed room has been resolved.
     */
    int currentRoomId{-1};

    /*!
     * @brief           Id of the room known just before the current tracking
     *                  loss.
     *
     *                  -1 when tracking has not yet been lost.
     */
    int lastKnownRoomId{-1};

    /*!
     * @brief           The transformation matrix from ground plane to
     *                  horizontal.
     */
    Eigen::Matrix4f groundPlanToHorizontalTransformation;

    /*!
     * @brief           Member which contains the system parameters.
     */
    types::SystemParams *p_sysParams;

    /*!
     * @brief           Openings measured OPEN in wall slabs that are not yet
     *                  passages; cleared with the map's temporal state.
     */
    std::vector<PendingOpening> pendingOpenings;

    /*!
     * @brief           Stamp of the last wall-slab answer used, nanoseconds;
     *                  older or repeated answers are ignored.
     */
    std::int64_t lastUsedSlabProfileStamp_ns = 0;

    /*!
     * @brief           Latest UAV camera centre in the active map frame.
     *
     *                  Refreshed at the start of each semantic cycle
     *                  (refreshCurrentCameraCenter()) and advanced by
     *                  updateTraversalEvidence() while it replays ordered
     *                  keyframes, which ends on the newest one again.
     */
    Eigen::Vector3d currentCameraCenter_world_m = Eigen::Vector3d::Zero();

    /*!
     * @brief           Camera centre of the previous processed keyframe.
     */
    Eigen::Vector3d previousCameraCenter_world_m = Eigen::Vector3d::Zero();

    /*!
     * @brief           Whether currentCameraCenter_world_m holds a valid
     *                  position.
     */
    bool hasCameraCenter = false;

    /*!
     * @brief           Active map whose frame contains the tracked camera
     *                  centres; borrowed, null until a camera centre has been
     *                  tracked.
     */
    map::Map *p_cameraCenterMap = nullptr;

    /*!
     * @brief           Last keyframe consumed by traversal sampling.
     */
    long unsigned int lastTraversalFrameId = 0U;

    /*!
     * @brief           Id of the last key frame consumed by traversal sampling;
     *                  paired with lastTraversalFrameId. 0 until one has been
     *                  consumed.
     */
    long unsigned int lastTraversalKeyFrameId = 0U;

    /*!
     * @brief           Whether the traversal keyframe cursor is initialized.
     */
    bool hasTraversalKeyFrameCursor = false;

    /*!
     * @brief           Active map which owns every map-local temporal cache
     *                  below.
     */
    map::Map *p_temporalStateMap = nullptr;

    /*!
     * @brief           Coordinate-frame epoch used to detect whole-map rebases
     *                  only.
     */
    std::uint64_t temporalStateWorldFrameEpoch = 0U;

    /*!
     * @brief           Confirmed rooms reported as disconnected on the previous
     *                  cycle.
     *
     *                  Retaining the IDs prevents the online consistency
     *                  warning from being repeated when no new passage evidence
     *                  has arrived.
     */
    std::unordered_set<int> disconnectedRoomIds;

    /*!
     * @brief           Tracks passage-created prospective room handles. A zero
     *                  value denotes a live handle; entries are removed on
     *                  promotion, replacement, or orphan cleanup.
     */
    std::unordered_map<int, int> prospectiveRoomCycles;

    /*!
     * @brief           Evidence bookkeeping for one wall that belongs to no
     *                  room or passage yet.
     */
    struct UndefendedWallState
    {
        /*!
         * @brief           Borrowed wall being tracked; the entry is reset when
         *                  the same id maps to a different object.
         */
        geometric::Plane *p_wall = nullptr;

        /*!
         * @brief           Number of cycles in a row without growth of the wall
         *                  evidence; reset to 0 when the evidence grows.
         */
        unsigned int unresolvedCycles = 0U;

        /*!
         * @brief           Number of points in the wall support cloud at the
         *                  last cycle.
         */
        std::size_t cloudPointCount = 0U;

        /*!
         * @brief           Number of key frames that observed the wall at the
         *                  last cycle.
         */
        std::size_t observationCount = 0U;
    };

    /*!
     * @brief           Weak, unused wall hypotheses awaiting bounded
     *                  retirement.
     */
    std::unordered_map<int, UndefendedWallState> undefendedWalls;

    /*!
     * @brief           Current semantic transaction sequence used by
     *                  SG_PIPELINE.
     */
    std::uint64_t pipelineSemanticCycle{0U};

    /*!
     * @brief           Guards wallSlabQueries and wallSlabProfiles only; never
     *                  held together with the semantic update lock, so the ROS
     *                  slab client cannot stall a semantic cycle.
     */
    mutable std::mutex wallSlabMutex;

    /*!
     * @brief           One slab query per confirmed wall, rebuilt every
     *                  semantic cycle. Guarded by wallSlabMutex.
     */
    std::vector<WallSlabQuery> wallSlabQueries;

    /*!
     * @brief           Latest Voxblox profiles, one per queried wall. Guarded
     *                  by wallSlabMutex.
     */
    std::vector<WallSlabProfile> wallSlabProfiles;

    /*!
     * @brief           Gives each passage anchored on a wall its verdict from
     *                  the wall's slab profile: NO_EVIDENCE (too little of the
     *                  aperture observed, or no measured gap and few wall
     *                  points inside; counters frozen), OPEN (a measured gap
     *                  with few wall points inside) or CLOSED (wall points
     *                  inside); then blocks, unblocks or retires the passage.
     *                  An uncrossed passage is not retired while the camera
     *                  is within max_kf_passage_distance of it.
     *
     * @param[in]       p_wall_in
     *                  Wall the profile belongs to; non-owning.
     *
     * @param[in]       wallProfile_in
     *                  Fresh slab profile of the wall, world frame.
     *
     * @param[in]       openGaps_in
     *                  Gap measured at each passage anchored on the wall.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus applyPassageVerdicts(
        geometric::Plane                                       *p_wall_in,
        const WallSlabProfile                                  &wallProfile_in,
        const std::unordered_map<semantic::Passage *, WallGap> &openGaps_in);

    /*!
     * @brief           Rebuilds wallSlabQueries from the confirmed walls: a
     *                  slab along each wall plane over its support-cloud extent
     *                  (+- wall_bounds_margin) from the floor up to
     *                  door_band_max_height_m + 0.3 m. Clears the queries when
     *                  no ground plane or camera centre is known.
     *
     * @param[in]       confirmedWalls_in
     *                  Walls with enough observations for passage detection.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus buildWallSlabQueries(
        const std::vector<geometric::Plane *> &confirmedWalls_in);

    /*!
     * @brief           Result of establishing the active map's bootstrap
     *                  hierarchy.
     */
    enum class ActiveMapBootstrapResult
    {
        INITIALIZED,
        RECOVERED,
        REUSED,
        NO_ACTIVE_MAP,
        NO_USABLE_CAMERA_POSE,
        ROOM_CREATION_FAILED,
        FLOOR_CREATION_FAILED
    };

    /*!
     * @brief           Suppresses repeated diagnostics for the same entity ID.
     */
    std::unordered_set<int> loggedOrphanWallIds;

    /*!
     * @brief           Last logged reason (a text such as CLASS_NOT_WALL) for
     *                  which a wall was kept out of the rooms, by wall id, so a
     *                  reason is logged only when it changes.
     */
    std::unordered_map<int, std::string> loggedWallRejectionReasons;

    /*!
     * @brief           Ids of walls whose retirement has already been logged.
     */
    std::unordered_set<int> loggedRetiredWallIds;

    /*!
     * @brief           Ids of rooms whose orphan clean-up has already been
     *                  logged.
     */
    std::unordered_set<int> loggedRoomCleanupIds;

    /*!
     * @brief           Suppresses repeated diagnostics for the same merged
     *                  passage pair (survivor id, absorbed id). Passage has no
     *                  isBad()/deletion lifecycle, so a merge re-detects the
     *                  same overlap every cycle; the field-sync itself is
     *                  idempotent, only the log line needs deduplicating.
     */
    std::set<std::pair<int, int>> loggedPassageMergeIds;

    /*!
     * @brief           Last logged twin-face state of a wall pair (lower id,
     *                  higher id), as "<state>:<check>" (for example
     *                  UNLINKED:OBSERVED_FROM_SAME_SIDE), so the twin_face
     *                  event is emitted once per pair per state change.
     */
    std::map<std::pair<int, int>, std::string> loggedTwinFaceStates;

    /*!
     * @brief           Maximum number of prospective rooms allowed
     *                  simultaneously. Matches office_clean's 12 rooms.
     */
    static constexpr int kMaxProspectiveRooms = 12;

    /*!
     * @brief           Spatial deduplication distance for prospective rooms
     *                  (meters).
     */
    static constexpr double kProspectiveDedupDistance_m = 2.0;

    /*!
     * @brief           Scopes transient semantic evidence to one active map and
     *                  frame.
     *
     *                  A map switch clears every map-local cache. An in-place
     *                  map correction invalidates geometry fingerprints and
     *                  reseeds camera traversal state.
     */
    [[nodiscard]] SemanticsManagerStatus
        resetTemporalStateForMap(map::Map *p_activeMap_in);

    /*!
     * @brief           Ensures one real bootstrap room and canonical floor for
     *                  the active map before any semantic inference is
     *                  performed.
     *
     * @param[out]      bootstrapResult_out
     *                  Typed initialization result, also emitted through
     *                  SG_PIPELINE.
     *
     * @param[in]       cameraPositionOverride_world_m_in
     *                  Camera position in the active map frame, in metres, used
     *                  to place the bootstrap room; when absent or not finite,
     *                  the newest valid key frame's camera centre is used.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus ensureActiveMapBootstrapHierarchy(
        SemanticsManager::ActiveMapBootstrapResult &bootstrapResult_out,
        const std::optional<Eigen::Vector3d>
            &cameraPositionOverride_world_m_in = std::nullopt);

    /*!
     * @brief           Seeds currentRoomId from the first confirmed room of the
     *                  active map, only while no current room is resolved yet.
     *
     * @param[in]       p_activeMap_in
     *                  Active map whose room set provides the seed room.
     */
    [[nodiscard]] SemanticsManagerStatus
        seedCurrentRoomFromActiveMap(map::Map *p_activeMap_in);

    /*!
     * @brief           Advances the room-state machine once per semantic
     *                  cycle.
     *
     *                  Consumes the per-cycle crossing and tracking-loss
     *                  signals and feeds them to roomTracker together
     *                  with the abstract verification verdict from the
     *                  verification stub. Read-only with respect to
     *                  currentRoomId/lastKnownRoomId.
     */
    [[nodiscard]] SemanticsManagerStatus
        updateRoomTrackerState(double now_s_in);

    /*!
     * @brief           Resolves a room by the (map id, room id) pair a
     *                  SemanticCandidate carries, scanning every live map in
     *                  the Atlas (not only the current one) since a candidate's
     *                  two rooms are, by construction, never in the same map.
     *
     * @param[in]       mapId_in
     *                  Map that holds the room.
     *
     * @param[in]       roomId_in
     *                  Room id within that map.
     *
     * @param[out]      p_room_out
     *                  The matching, non-bad ROOM-variant room, or nullptr.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        findRoomByMapAndId(long unsigned int mapId_in,
                           int               roomId_in,
                           semantic::Room  *&p_room_out) const;

    /*!
     * @brief           Runs the geometric verifier on the single best
     *                  candidate and feeds the resulting
     *                  VerificationVerdict to the room-state machine.
     *
     *                  Verification only -- this never mutates the Atlas
     *                  or triggers a map merge; see the comment above
     *                  its call site in Run() for why that trigger is a
     *                  separate, deliberately gated step.
     *
     * @param[in]       candidates_in
     *                  This cycle's ranked SemanticCandidate list, best
     *                  first.
     */
    [[nodiscard]] SemanticsManagerStatus evaluateTopCandidateVerification(
        const std::vector<semantic::SemanticCandidate> &candidates_in);

    /*!
     * @brief           Enforces exclusive ownership of every mapped wall
     *                  surface.
     *
     *                  Observation-side evidence selects the most plausible
     *                  room when legacy data or a map merge assigned one Plane
     *                  object to multiple rooms. This structural invariant is
     *                  enforced even when passage detection is disabled.
     */
    [[nodiscard]] SemanticsManagerStatus enforceUniqueWallOwnership(void);

    /*!
     * @brief           Links each WALL Plane to the opposite-facing Plane
     *                  hypothesis believed to be the other face of the same
     *                  physical wall.
     *
     *                  A physical wall can produce two independent Plane
     *                  objects, one per face, distinguished only by
     *                  observationOrigin_world_m. This pass identifies
     *                  plausible twin pairs (parallel, no thicker than the
     *                  maximum plausible wall thickness, observed from
     *                  opposite sides, overlapping in-plane footprint) and
     *                  links them symmetrically via
     *                  Plane::setTwinFace(), re-validating and unlinking pairs
     *                  that stop being plausible (e.g. after a refit). Run
     *                  after enforceUniqueWallOwnership() so both faces of a
     *                  pair already have settled room ownership, and before
     *                  validateRoomBoundaries() so boundary/corner logic can
     *                  rely on twin identity being current.
     */
    [[nodiscard]] SemanticsManagerStatus reconcileWallFacePairs(void);

    /*!
     * @brief           Validates finite wall associations and updates room
     *                  maturity.
     *
     *                  The validator repairs decisive wall clashes, rejects
     *                  ambiguous self-intersections, and marks a room complete
     *                  only when ordered finite wall segments form a closed
     *                  non-self-intersecting loop. Passage detection remains
     *                  independent and may precede boundary completion.
     */
    [[nodiscard]] SemanticsManagerStatus validateRoomBoundaries(void);

    /*!
     * @brief           Ages and safely retires unused walls which never become
     *                  valid.
     */
    [[nodiscard]] SemanticsManagerStatus suppressUndefendedWalls(void);

    /*!
     * @brief           Converts undefendedWalls into pointer-free,
     *                  map-sorted UnresolvedWallHypothesisRecord values.
     *                  Read-only; must be called while the semantic-update lock
     *                  is still held.
     */
    [[nodiscard]] SemanticsManagerStatus captureUnresolvedWallHypotheses(
        std::vector<semantic::UnresolvedWallHypothesisRecord>
            &captureUnresolvedWallHypotheses_out) const;

    /*!
     * @brief           Emits bounded, parseable SG_AXIOM/SG_VIOLATION
     *                  diagnostic lines for \p entry_in via
     *                  semantic::buildSemanticDiagnosticUpdate(), which
     *                  also updates semanticDiagnosticState so the
     *                  next call can detect a transition. Never acquires
     *                  the semantic-update lock (called after it is
     *                  released) and never mutates evaluator/inference
     *                  state -- read-only with respect to everything
     *                  except semanticDiagnosticState.
     *
     * @param[in]       entry_in
     *                  The cache entry just published for this cycle.
     */
    [[nodiscard]] SemanticsManagerStatus logSemanticDiagnostics(
        const semantic::SemanticReportCacheEntry &entry_in);

    /*!
     * @brief           Recomputes room centroids as the arithmetic mean of
     *                  associated wall centroids.
     *
     *                  Replaces the free-space cluster centroid with the
     *                  geometric center of the room's boundary walls. Rooms
     *                  with no valid walls retain their existing centroid.
     */
    [[nodiscard]] SemanticsManagerStatus recomputeRoomCentroidsFromWalls(void);

    /*!
     * @brief           Splits connected skeleton components at confirmed
     *                  passages.
     *
     *                  Voxblox correctly represents free-space connectivity
     *                  through an opening, whereas a semantic room must stop at
     *                  that opening. The raw sparse-graph edges are therefore
     *                  cut only inside confirmed passage apertures before
     *                  room-wall association.
     *
     * @param[in]       freeSpaceClusters_world_m_in
     *                  Connected Voxblox vertex components in the active map
     *                  frame.
     *
     * @param[out]      partitions_out
     *                  Passage-partitioned components in the same frame.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus partitionFreeSpaceAtPassages(
        const std::vector<std::vector<Eigen::Vector3d>>
                                                  &freeSpaceClusters_world_m_in,
        std::vector<std::vector<Eigen::Vector3d>> &partitions_out) const;

    /*!
     * @brief           Removes room-wall associations proven to pass through an
     *                  opening.
     *
     *                  This repairs persistent associations created before a
     *                  passage was confirmed. Supporting walls on the passage
     *                  plane are retained.
     */
    [[nodiscard]] SemanticsManagerStatus
        detachWallsBeyondConfirmedPassages(void);

    /*!
     * @brief           Sets currentCameraCenter_world_m to the camera centre of
     *                  the newest usable key frame of the active map; called
     *                  once at the start of each semantic cycle.
     *
     *                  A key frame is usable when it is not bad and its centre
     *                  is finite and not exactly zero (an exactly-zero centre
     *                  marks an uninitialised first-frame pose).
     *                  hasCameraCenter is false when there is no active map or
     *                  no usable key frame.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus refreshCurrentCameraCenter(void);

    /*!
     * @brief           Gives the points a room is judged from: its centroid,
     *                  and for the room the camera is in also the current
     *                  camera centre.
     *
     *                  The damped centroid of a young room can drift across
     *                  one of its own walls, and the current-room estimate can
     *                  lag the camera crossing a doorway. So for the room the
     *                  camera is in, the face rule and the doorway backstop act
     *                  only when the camera and the centroid agree; a point too
     *                  close to a plane to tell its side counts as
     *                  disagreement. The room the camera is in is the room
     *                  whose id is getCurrentRoomId() and whose map is the
     *                  active map.
     *
     * @param[in]       p_room_in
     *                  Room to place; shall be non-null.
     *
     * @param[out]      roomCentroid_world_m_out
     *                  Room centroid in the active map frame, metres.
     *
     * @param[out]      cameraCenter_world_m_out
     *                  Current camera centre in the active map frame, metres;
     *                  zero when hasCameraViewpoint_out is false.
     *
     * @param[out]      hasCameraViewpoint_out
     *                  True when p_room_in is the room the camera is in and a
     *                  camera centre is available.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        getRoomViewpoints(semantic::Room  *p_room_in,
                          Eigen::Vector3d &roomCentroid_world_m_out,
                          Eigen::Vector3d &cameraCenter_world_m_out,
                          bool            &hasCameraViewpoint_out);

    /*!
     * @brief           Orders wall admission candidates so nearer walls are
     *                  tried first.
     *
     *                  Order: horizontal distance from the current camera
     *                  centre to the wall's finite ground segment (nearest
     *                  first), then the time the wall was first observed
     *                  (oldest first), then plane id. A wall without a finite
     *                  segment, or every wall when there is no camera centre
     *                  or ground plane, has an infinite distance. The order is
     *                  deterministic for a given map state.
     *
     * @param[in,out]   walls_inout
     *                  Candidate planes, reordered in place.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        sortWallsByCameraDistance(std::vector<geometric::Plane *> &walls_inout);

    /*!
     * @brief           Adds a wall only when it preserves a valid room
     *                  boundary.
     *
     *                  A decisively stronger candidate replaces clashing,
     *                  weaker wall hypotheses. A weaker or ambiguous candidate
     *                  remains unassigned so the orphan-wall hierarchy can
     *                  retain it for another room and later observations can
     *                  retry the association. A candidate seen through a
     *                  nearer wall from the camera (room the camera is in),
     *                  or beyond a passage wall from the room centre (any
     *                  other room), is refused.
     *
     * @param[in,out]   p_room_inout
     *                  Room whose wall set may be updated.
     *
     * @param[in]       p_candidateWall_in
     *                  Candidate finite wall hypothesis.
     *
     * @param[out]      wasAdmitted_out
     *                  True when the candidate is already present or was
     *                  admitted.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        admitWallToRoom(semantic::Room   *p_room_inout,
                        geometric::Plane *p_candidateWall_in,
                        bool             &wasAdmitted_out);

    /*!
     * @brief           Outcome of enforcePassageApertureBackstop().
     */
    enum class PassageSideEnforcementOutcome
    {
        NO_VIOLATION,
        REMOVED_UNBOUND,
        REROUTED
    };

    /*!
     * @brief           Shared far-side-passage backstop: a wall whose centroid
     *                  lies beyond a passable passage's own BOUNDED aperture
     *                  (segmentCrossesPassageOpening() -- does the straight
     *                  path from p_room_inout's centroid -- and, for the room
     *                  the camera is in, also from the camera
     *                  (getRoomViewpoints()) -- to the wall actually
     *                  thread that specific doorway's width/height, not "which
     *                  side of an infinite plane is the wall on") cannot bound
     *                  p_room_inout. Deliberately NOT a room-vs-wall-normal
     *                  half-space test: rooms are not guaranteed convex, so no
     *                  single side of any wall's normal is reliably "inside"
     *                  the room in general -- passage-aperture-crossing is the
     *                  topologically honest signal (a wall only reachable by
     *                  crossing a doorway belongs on the doorway's far side,
     *                  regardless of room shape). Used both at admission time
     *                  (admitWallToRoom()) for a single newly-considered wall,
     *                  and by the continuous per-cycle invariant sweep
     *                  (enforcePassageSideInvariant()) for every already-owned
     *                  wall in every room, since associateAllWallsToRooms()
     *                  only ever revisits orphan walls -- an already-admitted
     *                  wall is otherwise never re-checked once a relevant
     *                  passage's geometry stabilises after the fact.
     *
     *                  When p_room_inout is the passage's far-side room, a
     *                  crossing wall lies on the known side; when the passage
     *                  does not join p_room_inout at all, the room cannot
     *                  tell the passage's sides apart. In both cases a wall
     *                  the room does not own yet is refused (REMOVED_UNBOUND),
     *                  one it already owns is kept.
     *
     * @param[in,out]   p_room_inout
     *                  Room the wall is currently (or about to be) bound to. On
     *                  a violation, the wall is removed from it (and rerouted,
     *                  when a resolvable far room exists).
     *
     * @param[in]       p_wall_in
     *                  Wall being checked.
     *
     * @param[in]       allPassages_in
     *                  Every passage in the Atlas (caller-owned, so a sweep
     *                  over many walls builds the list once).
     *
     * @param[in]       groundNormal_world_in
     *                  Unit ground normal, needed by
     *                  segmentCrossesPassageOpening()'s own vertical-offset
     *                  test.
     *
     * @param[out]      outcome_out
     *                  Whether a violation was found and, if so, how it was
     *                  resolved.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus enforcePassageApertureBackstop(
        semantic::Room                                  *p_room_inout,
        geometric::Plane                                *p_wall_in,
        const std::vector<semantic::Passage *>          &allPassages_in,
        const Eigen::Vector3d                           &groundNormal_world_in,
        SemanticsManager::PassageSideEnforcementOutcome &outcome_out);

    /*!
     * @brief           Wall-face ownership rule: is this face the NEIGHBOURING
     *                  room's, rather than p_room_in's?
     *
     *                  A physical wall has two faces, and a camera can only
     *                  ever observe the one turned toward it. The face's
     *                  identity is therefore fixed the moment it is first seen,
     *                  and is stamped then as the observing camera position
     *                  (Plane::getObservationOrigin_world()). A room owns a
     *                  face only if the room lies on the same side of the plane
     *                  as that camera did (the room's position is its
     *                  centroid; for the room the camera is in, the camera and
     *                  the centroid must both be on the opposite side -- see
     *                  getRoomViewpoints()); a room on the opposite
     *                  side is
     *                  looking at the OTHER face, which is a different surface
     *                  it has not observed.
     *
     *                  This deliberately replaces the previous
     *                  observation-history consensus vote. That vote summarised
     *                  every keyframe that ever saw the plane, so it inverted
     *                  its own answer once the UAV passed through a doorway and
     *                  accumulated more keyframes beyond it -- evicting a
     *                  room's own doorway wall -- and needed a growing set of
     *                  passage-supporting-wall exemptions to stay usable. Face
     *                  identity is not a statistic and does not drift, so no
     *                  exemption is required: the far face is simply a
     *                  different plane that this room never observed.
     *
     *                  Room::getWallNormalTowardRoom_world() cannot serve this
     *                  purpose: it always flips the normal to face the room, so
     *                  it can never report that a face belongs elsewhere.
     *
     * @param[in]       p_room_in
     *                  Room whose ownership of the face is tested.
     *
     * @param[in]       p_wall_in
     *                  Wall face (plane) to test.
     *
     * @param[out]      isWallFaceForeignToRoom_out
     *                  True when the face was observed from the side opposite
     *                  p_room_in (it is the neighbouring room's face); false
     *                  otherwise, including when no observation origin was
     *                  stamped or either side is too close to the plane to
     *                  resolve (fails open, does not reject).
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        isWallFaceForeignToRoom(semantic::Room   *p_room_in,
                                geometric::Plane *p_wall_in,
                                bool             &isWallFaceForeignToRoom_out);

    /*!
     * @brief           Continuous invariant sweep: re-applies
     *                  enforcePassageApertureBackstop() AND
     *                  isWallFaceForeignToRoom() to every wall every
     *                  room currently owns, not just newly-admitted ones.
     *                  Called once per cycle so a passage that only became
     *                  confidently resolvable, a room boundary that only
     *                  became confidently placed, or a room centroid that
     *                  drifted after a wall was already admitted still gets
     *                  corrected, instead of the violation sitting unexamined
     *                  until the wall happens to be re-admitted from scratch.
     */
    [[nodiscard]] SemanticsManagerStatus enforcePassageSideInvariant(void);

    /*!
     * @brief           Per-cycle sweep of the passage topology rules
     *                  (isPassageTopologyValid()); a violating passage is set
     *                  bad and one SG_PIPELINE passage_invalid line names the
     *                  broken rule.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus enforcePassageTopologyInvariant(void);

    /* ---------------------------------------------------------------------- *
     * SCENE GRAPH REGISTRY
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Guards sceneGraphs, erasedSceneGraphs and
     *                  activeGraphMapId only, never a graph's nodes (those are
     *                  guarded by each SceneGraph::graphMutex). Held for a
     *                  lookup or an insert, never while waiting for a graph
     *                  lock.
     */
    mutable std::shared_mutex registryMutex;

    /*!
     * @brief           Scene graphs by SLAM map id; borrowed. Each SLAM map
     *                  owns its scene graph as a member and is registered here
     *                  by addSceneGraph() until eraseSceneGraph(). Guarded by
     *                  registryMutex.
     */
    std::map<unsigned long, scene_graph::SceneGraph *> sceneGraphs;

    /*!
     * @brief           Id of the atlas's current map; empty when there is
     *                  none. Guarded by registryMutex.
     */
    std::optional<unsigned long> activeGraphMapId;

    /*!
     * @brief           Scene graphs of maps the atlas dropped, borrowed. No
     *                  query finds them any more; the next sweep retires their
     *                  nodes (other nodes may still point at them) and
     *                  forgets them. Guarded by registryMutex.
     */
    std::vector<scene_graph::SceneGraph *> erasedSceneGraphs;

    /*!
     * @brief           Looks a scene graph up by map id. Takes registryMutex
     *                  shared; does not lock the graph itself.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @return          The map's scene graph; borrowed, owned by its Map; null
     *                  when the registry has none.
     */
    [[nodiscard]] scene_graph::SceneGraph *
        findSceneGraph(unsigned long mapId_in) const;

    /*!
     * @brief           Copies the registry, so a caller can walk the graphs
     *                  without holding registryMutex. Takes it shared.
     *
     * @return          The registered scene graphs by map id, ascending;
     *                  borrowed.
     */
    [[nodiscard]] std::map<unsigned long, scene_graph::SceneGraph *>
        copySceneGraphs() const;

    /*!
     * @brief           Finds a map's scene graph and locks it. The graph is
     *                  looked up first and locked second, so the registry
     *                  mutex is never held while waiting for a graph.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       mode_in
     *                  How to lock the graph.
     *
     * @param[out]      scope_out
     *                  Holds the lock until it goes out of scope.
     *
     * @param[out]      p_sceneGraph_out
     *                  The locked graph; borrowed; null on failure.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND, or
     *                  SEMANTICS_MANAGER_STATUS_BUSY (try mode only).
     */
    [[nodiscard]] SemanticsManagerStatus
        lockMapGraph(unsigned long             mapId_in,
                     GraphLockMode             mode_in,
                     GraphLockScope           &scope_out,
                     scene_graph::SceneGraph *&p_sceneGraph_out) const;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Constructor which stores the pointer to the map in the
     *                  member p_atlas and gets the systems parameter.
     *
     * @param[in]       p_atlas_in
     *                  Pointer to map.
     */
    explicit SemanticsManager(atlas::Atlas *p_atlas_in)
    {
        /* Store the address of the atlas map */
        p_atlas = p_atlas_in;

        /* Get the system parameters */
        types::SystemParams *p_params = nullptr;
        if (types::SystemParams::getParams(p_params) !=
            types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParams returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_sysParams = p_params;

        /* Configure the room-tracking state machine. */
        semantic::RoomTrackerConfig trackerConfiguration;
        trackerConfiguration.crossing_dwell_s =
            static_cast<double>(p_sysParams->roomTracking.crossingDwell_s);
        trackerConfiguration.crossing_confidence =
            static_cast<double>(p_sysParams->roomTracking.crossingConfidence);
        trackerConfiguration.lost_timeout_s =
            static_cast<double>(p_sysParams->roomTracking.lostTimeout_s);
        trackerConfiguration.reacquire_timeout_s =
            static_cast<double>(p_sysParams->roomTracking.reacquireTimeout_s);
        trackerConfiguration.reacquire_retry_interval_s = static_cast<double>(
            p_sysParams->roomTracking.reacquireRetryInterval_s);
        trackerConfiguration.reacquire_max_retries =
            p_sysParams->roomTracking.reacquireMaxRetries;
        trackerConfiguration.reacquire_min_planes =
            p_sysParams->roomTracking.reacquireMinPlanes;
        roomTracker = semantic::RoomTracker(trackerConfiguration);
    }

    /*!
     * @brief           Gets the latest skeleton cluster acquired from voxblox.
     */
    [[nodiscard]] SemanticsManagerStatus getLatestSkeletonCluster(
        std::vector<std::vector<Eigen::Vector3d>> &latestSkeletonCluster_out);

    /*!
     * @brief           Returns a copied snapshot of the latest complete
     *                  semantic evaluation cycle. Thread-safe (delegates to
     *                  SemanticReportCache's own mutex); never touches the
     *                  semantic-update lock. Callers must check
     *                  isSemanticReportCacheAvailable() first: before the first
     *                  completed cycle, this returns a meaningless default
     *                  value, not an error.
     */
    [[nodiscard]] SemanticsManagerStatus getSemanticReportCacheEntry(
        semantic::SemanticReportCacheEntry &getSemanticReportCacheEntry_out)
        const;

    /*!
     * @brief           True once at least one complete semantic cycle has been
     *                  cached.
     */
    [[nodiscard]] SemanticsManagerStatus isSemanticReportCacheAvailable(
        bool &isSemanticReportCacheAvailable_out) const;

    /*!
     * @brief           Copies the wall-slab queries of the latest semantic
     *                  cycle. Thread-safe; takes only wallSlabMutex.
     *
     * @param[out]      wallSlabQueries_out
     *                  Queries in the world frame; empty before the first
     *                  cycle.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus getWallSlabQueries(
        std::vector<WallSlabQuery> &wallSlabQueries_out) const;

    /*!
     * @brief           Replaces the stored wall-slab profiles with a new
     *                  Voxblox answer. Thread-safe; takes only wallSlabMutex.
     *
     * @param[in]       wallSlabProfiles_in
     *                  Profiles in the world frame, one per answered query.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus setWallSlabProfiles(
        const std::vector<WallSlabProfile> &wallSlabProfiles_in);

    /*!
     * @brief           Detects open passages from the latest unused Voxblox
     *                  wall-slab profiles: measures each wall's openings,
     *                  updates the passage at an opening and creates one after
     *                  minimumOpenVerdicts consecutive OPEN profiles.
     *
     * @param[in]       wallPlanes_in
     *                  Confirmed wall planes available in the current map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus detectPassagesFromFreeSpace(
        const std::vector<vs_graphs::core::geometric::Plane *> &wallPlanes_in);

    /*!
     * @brief           Detects doors and doorways based on the detected planes
     *                  and the mapped environment.
     *
     * @param[in]       p_atlas_in
     *                  The Atlas containing the mapped environment
     */
    [[nodiscard]] SemanticsManagerStatus
        detectDoorsAndDoorways(atlas::Atlas *p_atlas_in);

    /*!
     * @brief           Resets the semantics of ground planes whose normal is
     *                  horizontal and of wall planes whose normal is vertical.
     *                  Judged against Atlas::getUpDirection(), not
     *                  against the main ground plane, so that a misclassified
     *                  plane cannot become the ground reference. Uses the
     *                  plane_checkpoint.max_tilt_ground and
     *                  plane_checkpoint.max_tilt_wall parameters, the same
     *                  tolerances the plane checkpoint applies to each new
     *                  patch; this map-level pass catches planes that turn
     *                  after bundle adjustment or a map merge.
     */
    [[nodiscard]] SemanticsManagerStatus rejectMisorientedPlanes(void);

    /*!
     * @brief           Resets the semantics of every ground plane, other
     *                  than the main one, whose height passes the
     *                  sem_seg.max_step_elevation limit from the main ground
     *                  plane. Orientation is judged by
     *                  rejectMisorientedPlanes().
     *
     * @param[in]       p_groundPlane_in
     *                  The main ground plane that is the reference
     */
    [[nodiscard]] SemanticsManagerStatus
        filterGroundPlanes(geometric::Plane *p_groundPlane_in);

    /*!
     * @brief           Updates the passages in the map based on the detected
     *                  doors and doorways.
     *
     * @param[in]       p_atlas_in
     *                  The Atlas containing the mapped environment
     */
    [[nodiscard]] SemanticsManagerStatus
        updatePassages(atlas::Atlas *p_atlas_in);

    /*!
     * @brief           Merges passages whose estimated 2D footprints (width x
     *                  height, projected into their shared wall's own plane)
     *                  overlap -- evidence of the same physical opening
     *                  detected/anchored more than once (e.g. via different
     *                  wall faces or detection paths). Passage has no
     *                  isBad()/deletion lifecycle in this codebase, so "merge"
     *                  means folding the absorbed passage's associate walls,
     *                  known-side provenance, prospective room, passability and
     *                  size onto the surviving (lower-id, i.e. first-detected)
     *                  passage -- both Passage objects remain in the Atlas, but
     *                  only the survivor's semantic links are treated as
     *                  authoritative going forward.
     */
    [[nodiscard]] SemanticsManagerStatus mergeOverlappingPassages(void);

    /*!
     * @brief           Records traversal evidence on every passable passage
     *                  whose aperture the UAV camera trajectory crossed since
     *                  the last semantic cycle.
     *
     *                  Consumes all new keyframes in deterministic frame/id
     *                  order and checks each consecutive camera-centre segment
     *                  against passage aperture geometry. The evidence is a
     *                  separate "settled" mark and never modifies observation
     *                  confirmation.
     *
     * @param[in]       p_atlas_in
     *                  The Atlas containing the mapped environment.
     */
    [[nodiscard]] SemanticsManagerStatus
        updateTraversalEvidence(atlas::Atlas *p_atlas_in);

    /*!
     * @brief           Returns the id of the room currently occupied by the
     *                  camera.
     *
     * @param[out]      getCurrentRoomId_out
     *                  Current room id, or -1 before any room is confirmed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        getCurrentRoomId(int &getCurrentRoomId_out) const;

    /*!
     * @brief           Returns the usable room of the active map whose id is
     *                  getCurrentRoomId(); the room variant is not checked.
     *
     * @param[out]      p_currentRoom_out
     *                  Current room, or nullptr when there is none.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        getCurrentRoom(semantic::Room *&p_currentRoom_out) const;

    /*!
     * @brief           Returns the id of the room last occupied before tracking
     *                  was lost.
     *
     * @param[out]      getLastKnownRoomId_out
     *                  Last-known room id, or -1 before any tracking loss.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        getLastKnownRoomId(int &getLastKnownRoomId_out) const;

    /*!
     * @brief           Persists the current room as the last-known room when
     *                  tracking is lost.
     *
     *                  Records the transition only once per loss episode:
     *                  repeated calls while the current room is unchanged are
     *                  no-ops.
     */
    [[nodiscard]] SemanticsManagerStatus onTrackingLost(void);

    /*!
     * @brief           Marks the end of the current LOST episode.
     *
     *                  System calls this after observing a non-LOST tracking
     *                  state. It re-arms the next genuine loss event without
     *                  clearing undelivered events.
     */
    [[nodiscard]] SemanticsManagerStatus onTrackingRecovered(void);

    /*!
     * @brief           Gets the median height of a ground plane after
     *                  transformation to referece by mPlanePoseMat.
     *
     * @param[in]       p_groundPlane_in
     *                  The ground plane
     *
     * @param[out]      groundPlaneHeight_out
     *                  The median height of the ground plane, or std::nullopt
     *                  when the plane's support cloud is empty (nothing to
     *                  compute a height from -- must not be treated as 0.0,
     *                  which is a valid real height).
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        computeGroundPlaneHeight(geometric::Plane     *p_groundPlane_in,
                                 std::optional<float> &groundPlaneHeight_out);

    /*!
     * @brief           Computes the transformation matrix from the ground plane
     *                  to the horizontal (y-inverted).
     *
     * @param[in]       p_plane_in
     *                  The plane
     *
     * @param[out]      plane_out
     *                  the transformation matrix
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticsManagerStatus
        computePlaneToHorizontal(const geometric::Plane *p_plane_in,
                                 Eigen::Matrix4f        &plane_out);

    /*!
     * @brief           Checks for the existing of a room with particular walls
     *                  close to a cluster. It returns the address of the
     *                  existing room if found, otherwise returns nullptr.
     *
     * @param[in]       clusterCentroid_world_in
     *                  The centroid of the cluster in the world frame of the
     *                  current map.
     *
     * @param[in]       wallList_world_in
     *                  The list of walls to be checked in the current world map
     *
     * @param[in]       freeSpaceCluster_world_m_in
     *                  Connected Voxblox skeleton vertices supporting the room.
     *
     * @param[in]       excludedRoomIds_in
     *                  Room IDs already matched to other clusters in this
     *                  cycle.
     *
     * @param[out]      p_room_out
     *                  The existing room, or nullptr when none matches.
     */
    [[nodiscard]] SemanticsManagerStatus associateRooms(
        const Eigen::Vector3d clusterCentroid_world_in,
        const std::vector<vs_graphs::core::geometric::Plane *>
                                           &wallList_world_in,
        const std::vector<Eigen::Vector3d> &freeSpaceCluster_world_m_in,
        const std::unordered_set<int>      &excludedRoomIds_in,
        vs_graphs::core::semantic::Room   *&p_room_out);

    /*!
     * @brief           Fuses duplicate room hypotheses supported by one
     *                  connected free-space region when no finite wall
     *                  separates them.
     *
     * @param[in,out]   p_retainedRoom_inout
     *                  Room which retains the consolidated semantic topology.
     *
     * @param[in]       freeSpaceCluster_world_m_in
     *                  Connected Voxblox skeleton vertices for the current
     *                  room.
     *
     * @param[in]       wallList_world_in
     *                  Current mapped wall surfaces used as merge vetoes.
     */
    [[nodiscard]] SemanticsManagerStatus consolidateRoomsInFreeSpaceCluster(
        vs_graphs::core::semantic::Room    *p_retainedRoom_inout,
        const std::vector<Eigen::Vector3d> &freeSpaceCluster_world_m_in,
        const std::vector<vs_graphs::core::geometric::Plane *>
            &wallList_world_in);

    /*!
     * @brief           Ensures every valid WALL plane belongs to at least one
     *                  room or provisional structural element.
     */
    [[nodiscard]] SemanticsManagerStatus associateAllWallsToRooms(void);

    /*!
     * @brief           Associates each passage with the closest room on either
     *                  side of its supporting wall.
     */
    [[nodiscard]] SemanticsManagerStatus associatePassagesToRooms(void);

    /*!
     * @brief           Associates cluster-supported walls with rooms using
     *                  passage-partitioned free-space evidence, maintaining
     *                  matched rooms and promoting supported candidates.
     *
     * @return          Status of the wall-to-room association cycle.
     */
    [[nodiscard]] SemanticsManagerStatus
        associateWallsToRoomsFromFreeSpaceClusters(void);

    /*!
     * @brief           Gets the updated floors containing rooms and corridors.
     */
    [[nodiscard]] SemanticsManagerStatus getUpdatedFloors(void);

    /*!
     * @brief           Reconciles each confirmed room's ground plane against
     *                  the just-refreshed canonical Floor identity.
     *
     *                  getUpdatedFloors() selects one canonical Floor plane
     *                  identity from the strongest-observed ground plane, but
     *                  different rooms can independently hold different ground
     *                  Plane objects
     *                  (GeoSemHelpers::associateGroundPlaneToRoom). Nothing
     *                  previously checked those against each other or against
     *                  the canonical identity -- this is the front-end
     *                  floor-flatness reconciliation Optimizer.cc's dead "moved
     *                  to the front-end" comment describes but that nothing
     *                  actually implemented. Run immediately after
     *                  getUpdatedFloors() so the canonical identity is current.
     */
    [[nodiscard]] SemanticsManagerStatus reconcileRoomGroundPlanes(void);

    /*!
     * @brief           Method which runs the thread of the segmantic manager.
     */
    void run(void);

    /*!
     * @brief           Requests a graceful stop of the semantic manager thread.
     */
    [[nodiscard]] SemanticsManagerStatus requestFinish();

    /*!
     * @brief           Queues one typed verification result for the
     *                  next cycle.
     *
     *                  This is an internal, in-process handoff; it does
     *                  not create a ROS topic. The caller may run
     *                  concurrently with Run(). Until a PASS result is
     *                  supplied, production verification remains
     *                  UNAVAILABLE.
     *
     * @param[in]       verdict_in
     *                  Value-only result produced by the geometric
     *                  verifier.
     */
    [[nodiscard]] SemanticsManagerStatus submitVerificationVerdict(
        const semantic::VerificationVerdict &verdict_in);

    /*!
     * @brief           Test-only deterministic drain of the production event
     *                  seam.
     */
    [[nodiscard]] SemanticsManagerStatus
        processRoomTrackerPendingForTest(double now_s_in);

    /*!
     * @brief           Test-only readout of the production-owned tracker
     *                  history.
     */
    [[nodiscard]] SemanticsManagerStatus getRoomTrackerEventHistoryForTest(
        const std::vector<semantic::TransitionEvent> *
            &p_getRoomTrackerEventHistoryForTest_out) const;

    /*!
     * @brief           Test-only readout of the crossing and both-sides pending
     *                  flags.
     */
    [[nodiscard]] SemanticsManagerStatus getRoomTrackerPendingForTest(
        std::pair<bool, bool> &getRoomTrackerPendingForTest_out) const;

    /*!
     * @brief           Test-only readout of the production-owned tracker state.
     */
    [[nodiscard]] SemanticsManagerStatus getRoomTrackerStateForTest(
        semantic::RoomTrackingState &getRoomTrackerStateForTest_out) const;

    /*!
     * @brief           Returns true only when the genuine pending-event mutex
     *                  was acquired.
     */
    [[nodiscard]] SemanticsManagerStatus
        tryLockRoomTrackerPendingMutexForTest(bool &isLocked_out) const;

    /*!
     * Installs a one-shot hook invoked while the genuine pending-event mutex is
     * held by updateTraversalEvidence().
     */
    [[nodiscard]] SemanticsManagerStatus
        setRoomTrackerPendingPublishHookForTest(std::function<void()> hook_in);

    /*!
     * @brief           Test-only direct call into the candidate-verification
     *                  wiring, bypassing Run()'s full ROS/PCL/ORB3-dependent
     *                  cycle.
     */
    [[nodiscard]] SemanticsManagerStatus
        evaluateTopCandidateVerificationForTest(
            const std::vector<semantic::SemanticCandidate> &candidates_in);

    /*!
     * @brief           Test-only readout of evaluateWallAdmissionEvidence()'s
     *                  admissible flag (the full WallAdmissionEvidence struct
     *                  is file-local to SemanticsManager.cc's anonymous
     *                  namespace).
     */
    [[nodiscard]] SemanticsManagerStatus
        evaluateWallAdmissionEvidenceAdmissibleForTest(
            geometric::Plane      *p_wall_in,
            const Eigen::Vector3d &groundNormal_world_in,
            bool                  &isAdmissible_out) const;

    /*!
     * @brief           Test-only direct call into the private admission gate
     *                  (far-side backstop, evidence check, wrong-side
     *                  observation check, boundary topology).
     */
    [[nodiscard]] SemanticsManagerStatus
        admitWallToRoomForTest(semantic::Room   *p_room_in,
                               geometric::Plane *p_candidateWall_in,
                               bool             &wasAdmitted_out);

    /*!
     * @brief           Test-only direct call into the private per-cycle
     *                  passage-side sweep (prospective-placement exemption,
     *                  wall-face ownership, aperture backstop). Callers inspect
     *                  the result via Room::getWalls().
     */
    [[nodiscard]] SemanticsManagerStatus
        enforcePassageSideInvariantForTest(void);

    /*!
     * @brief           Test-only direct call into the private current-room
     *                  fallback seed. Callers inspect the result via
     *                  getCurrentRoomId().
     */
    [[nodiscard]] SemanticsManagerStatus
        seedCurrentRoomFromActiveMapForTest(map::Map *p_activeMap_in);

    /*!
     * @brief           Test-only direct call into the private empty-cloud-safe
     *                  ground plane height computation (B1 regression
     *                  coverage).
     */
    [[nodiscard]] SemanticsManagerStatus computeGroundPlaneHeightForTest(
        geometric::Plane     *p_groundPlane_in,
        std::optional<float> &groundPlaneHeight_out);

    /*!
     * @brief           Test-only direct call into the private wall-twin-face
     *                  reconciliation pass. Callers inspect the result via
     *                  Plane::getTwinFace().
     */
    [[nodiscard]] SemanticsManagerStatus reconcileWallFacePairsForTest(void);

    /*!
     * @brief           Test-only direct call into the private floor-hierarchy
     *                  refresh (must run before
     *                  reconcileRoomGroundPlanesForTest() so a canonical Floor
     *                  identity exists to reconcile against).
     */
    [[nodiscard]] SemanticsManagerStatus getUpdatedFloorsForTest(void);

    /*!
     * @brief           Test-only bootstrap seam with an explicit valid camera
     *                  position.
     */
    [[nodiscard]] SemanticsManagerStatus
        ensureActiveMapBootstrapHierarchyForTest(
            const Eigen::Vector3d &cameraPosition_world_m_in,
            int                   &bootstrapResult_out);

    /*!
     * @brief           Test-only equivalent of committing a passage traversal
     *                  room change.
     */
    [[nodiscard]] SemanticsManagerStatus setCurrentRoomIdForTest(int roomId_in);

    /*!
     * @brief           Test-only direct ordinary-wall association pass.
     */
    [[nodiscard]] SemanticsManagerStatus associateAllWallsToRoomsForTest(void);

    /*!
     * @brief           Test-only direct passage/room reciprocity and lifecycle
     *                  pass.
     */
    [[nodiscard]] SemanticsManagerStatus associatePassagesToRoomsForTest(void);

    /*!
     * @brief           Test-only direct bounded pending-wall retirement pass.
     */
    [[nodiscard]] SemanticsManagerStatus suppressUndefendedWallsForTest(void);

    /*!
     * @brief           Test-only pending age readout; -1 means the wall is not
     *                  pending.
     */
    [[nodiscard]] SemanticsManagerStatus
        getPendingWallAgeForTest(int  wallId_in,
                                 int &pendingWallAgeForTest_out) const;

    /*!
     * @brief           Test-only direct call into the private room/floor
     *                  ground-plane reconciliation pass. Callers inspect the
     *                  result via Room::getGroundPlane().
     */
    [[nodiscard]] SemanticsManagerStatus reconcileRoomGroundPlanesForTest(void);

    /*!
     * @brief           Test-only direct call into the private boundary
     *                  validator (closed-loop detection, single-outlier-wall
     *                  tolerance, and off-loop wall pruning). Callers inspect
     *                  the result via Room::getBoundaryStatus(),
     *                  Room::getBoundaryCorners_world_m(), and
     *                  Room::getWalls().
     */
    [[nodiscard]] SemanticsManagerStatus validateRoomBoundariesForTest(void);

    /*!
     * @brief           True once the semantic manager thread has exited Run().
     */
    [[nodiscard]] SemanticsManagerStatus isFinished(bool &isFinished_out);

    /* ---------------------------------------------------------------------- *
     * SCENE GRAPH OPERATIONS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Adds an empty scene graph for a SLAM map: one building
     *                  (id 0) and no storeys, rooms or passages, replacing any
     *                  scene graph it had (that map's nodes are retired for the
     *                  next sweep, not freed here). Used by the atlas when it
     *                  creates or loads a map. Takes the new graph's lock
     * exclusive before registering it, so it never waits.
     *
     * @param[in,out]   map_inout
     *                  SLAM map the scene graph belongs to; only its id is
     *                  read. The scene graph borrows it; the atlas owns it.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus addSceneGraph(map::Map &map_inout);

    /*!
     * @brief           Unregisters the scene graph of a SLAM map the atlas
     *                  dropped, so no query finds it; when it was the active
     *                  map there is no active map afterwards. Its nodes are
     *                  retired and freed by the next sweep. Takes only the
     *                  registry mutex, never a graph lock.
     *
     * @param[in]       mapId_in
     *                  Id of the dropped SLAM map; unknown ids are ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus
        eraseSceneGraph(unsigned long mapId_in);

    /*!
     * @brief           Records which SLAM map the active-map queries read.
     *                  Takes only the registry mutex.
     *
     * @param[in]       mapId_in
     *                  Id of the atlas's current map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus setActiveMap(unsigned long mapId_in);

    /*!
     * @brief           Adds a new candidate room to a map, keyed by its id,
     *                  and takes ownership of it. A room whose id another
     *                  candidate room already holds is not added (a warning
     *                  is logged); it is retired instead, so a raw pointer
     *                  the caller kept stays valid until the next sweep. Takes
     *                  this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to add; ownership moves to the tree, also when
     *                  the map is unknown (the room is then retired). A null
     *                  room is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (the room is retired).
     */
    [[nodiscard]] SemanticsManagerStatus
        addCandidateRoom(unsigned long                   mapId_in,
                         std::unique_ptr<semantic::Room> p_room_in);

    /*!
     * @brief           Moves a room from the candidate rooms to the detected
     *                  rooms of a map; when its id is already held by another
     *                  detected room it is retired. Takes the graph lock
     *                  exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to promote; borrowed. A null room is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (nothing is written).
     */
    [[nodiscard]] SemanticsManagerStatus
        promoteCandidateRoom(unsigned long mapId_in, semantic::Room *p_room_in);

    /*!
     * @brief           Takes a room out of the tree, by pointer, whatever id
     *                  it is keyed under, and retires it: it stays alive but
     *                  no query returns it, and the next sweep unlinks and
     *                  frees it. Takes the graph lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to remove; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (nothing is written).
     */
    [[nodiscard]] SemanticsManagerStatus eraseRoom(unsigned long   mapId_in,
                                                   semantic::Room *p_room_in);

    /*!
     * @brief           Moves a room, with its ownership, from one scene graph
     *                  to a list of another (or the same) scene graph, keyed
     *                  by the id the room holds now. This is how merge hands
     *                  rooms over and how a room is re-keyed after a new id.
     *                  A room whose id the destination list already holds is
     *                  retired. Takes the graph lock exclusive; the move is one
     *                  step, so no reader sees the room missing.
     *
     * @param[in]       sourceMapId_in
     *                  Id of the SLAM map that holds the room.
     *
     * @param[in]       destinationMapId_in
     *                  Id of the SLAM map that receives it.
     *
     * @param[in]       p_room_in
     *                  Room to move; borrowed, found by pointer in either
     *                  list of the source map. A null room is ignored.
     *
     * @param[in]       destinationList_in
     *                  Room list of the destination map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when either scene
     * graph is missing or the source map does not hold the room (nothing is
     * written).
     */
    [[nodiscard]] SemanticsManagerStatus
        moveRoom(unsigned long   sourceMapId_in,
                 unsigned long   destinationMapId_in,
                 semantic::Room *p_room_in,
                 RoomList        destinationList_in);

    /*!
     * @brief           Adds a new passage to a map, keyed by its id, and
     *                  takes ownership of it. A passage with a negative id or
     *                  an id another passage holds is not added (a message is
     *                  printed): it is retired, so a raw pointer the caller
     *                  kept stays valid until the next sweep, and the status
     *                  is still success, so the caller must resolve the id
     *                  first. Takes the graph lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_passage_in
     *                  Passage to add; ownership moves to the tree. A null
     *                  passage is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (the passage is retired).
     */
    [[nodiscard]] SemanticsManagerStatus
        addPassage(unsigned long                      mapId_in,
                   std::unique_ptr<semantic::Passage> p_passage_in);

    /*!
     * @brief           Takes a passage out of the tree, by pointer, and
     *                  retires it (see eraseRoom). Takes the graph lock
     * exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_passage_in
     *                  Passage to remove; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (nothing is written).
     */
    [[nodiscard]] SemanticsManagerStatus
        erasePassage(unsigned long mapId_in, semantic::Passage *p_passage_in);

    /*!
     * @brief           Moves a passage, with its ownership, from one scene
     *                  graph to another (or re-keys it in the same one) under
     *                  the id it holds now. A passage whose id the
     *                  destination already holds is retired. Takes the graph
     * lock exclusive.
     *
     * @param[in]       sourceMapId_in
     *                  Id of the SLAM map that holds the passage.
     *
     * @param[in]       destinationMapId_in
     *                  Id of the SLAM map that receives it.
     *
     * @param[in]       p_passage_in
     *                  Passage to move; borrowed. A null passage is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when either scene
     * graph is missing or the source map does not hold the passage (nothing is
     * written).
     */
    [[nodiscard]] SemanticsManagerStatus
        movePassage(unsigned long      sourceMapId_in,
                    unsigned long      destinationMapId_in,
                    semantic::Passage *p_passage_in);

    /*!
     * @brief           Adds a new storey to building 0 of a map, keyed by its
     *                  id, and takes ownership of it. A storey whose id is
     *                  negative or held by another storey of the map is given
     *                  the next free id (a message is printed); an entry for
     *                  the storey under an older id is dropped. Ids handed out
     *                  are never reused. Takes the graph lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_floor_in
     *                  Storey to add; its id may be changed. Ownership moves
     *                  to the tree, also when the map is unknown (the storey
     *                  is then retired). A null storey is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (the storey is retired).
     */
    [[nodiscard]] SemanticsManagerStatus
        addFloor(unsigned long                    mapId_in,
                 std::unique_ptr<semantic::Floor> p_floor_in);

    /*!
     * @brief           Takes a storey out of every building of a map, by
     *                  pointer, and retires it (see eraseRoom). Takes this
     *                  lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_floor_in
     *                  Storey to remove; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (nothing is written).
     */
    [[nodiscard]] SemanticsManagerStatus
        eraseFloor(unsigned long mapId_in, semantic::Floor *p_floor_in);

    /*!
     * @brief           Moves a storey, with its ownership, from building 0 of
     *                  one scene graph to building 0 of another, under the id
     *                  it holds now; an id the destination holds gives it the
     *                  next free id there (as addFloor does). Takes the graph
     * lock exclusive.
     *
     * @param[in]       sourceMapId_in
     *                  Id of the SLAM map that holds the storey.
     *
     * @param[in]       destinationMapId_in
     *                  Id of the SLAM map that receives it.
     *
     * @param[in]       p_floor_in
     *                  Storey to move; borrowed. A null storey is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when either scene
     * graph is missing or the source map does not hold the storey (nothing is
     * written).
     */
    [[nodiscard]] SemanticsManagerStatus
        moveFloor(unsigned long    sourceMapId_in,
                  unsigned long    destinationMapId_in,
                  semantic::Floor *p_floor_in);

    /*!
     * @brief           Flags a room invalid and everything that depends on
     *                  it, in one step, so no reader sees a half-deleted
     *                  room. The room is hidden from every query at once.
     *                  Its passages are flagged too when no other live room
     *                  of the map lists them or is their known or prospective
     *                  side; a passage that still leads to a live room stays.
     *                  The room's planes are not touched here: the sweep
     *                  clears the room's plane lists (planes are never freed
     *                  by the sweep). Works for a room that is already
     *                  retired. Takes the graph lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map the room belongs to.
     *
     * @param[in,out]   p_room_inout
     *                  Room to flag; borrowed. A null room is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (the room is still flagged).
     */
    [[nodiscard]] SemanticsManagerStatus
        markRoomBad(unsigned long mapId_in, semantic::Room *p_room_inout);

    /*!
     * @brief           Flags a storey invalid and every room on it (see
     *                  markRoomBad). Takes the graph lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map the storey belongs to.
     *
     * @param[in,out]   p_floor_inout
     *                  Storey to flag; borrowed. A null storey is ignored.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id (the storey is still flagged).
     */
    [[nodiscard]] SemanticsManagerStatus
        markFloorBad(unsigned long mapId_in, semantic::Floor *p_floor_inout);

    /*!
     * @brief           Points every room and passage of the active map that
     *                  listed a retired plane at the plane that replaces it,
     *                  both sides written in one step. Takes the graph lock
     *                  exclusive.
     *
     * @param[in,out]   p_retiredPlane_inout
     *                  Plane being fused away; borrowed.
     *
     * @param[in,out]   p_retainedPlane_inout
     *                  Plane that survives; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always (no active map
     * or an unlisted plane changes nothing).
     */
    [[nodiscard]] SemanticsManagerStatus
        replacePlane(geometric::Plane *p_retiredPlane_inout,
                     geometric::Plane *p_retainedPlane_inout);

    /*!
     * @brief           Ends a cycle's deletion: moves every bad room,
     *                  passage and storey out of the tree, unlinks every
     *                  retired node from the nodes that still list it (the
     *                  storeys' room lists, the passages' room links, the
     *                  rooms' passage lists, the maps' linked rooms and the
     *                  rooms' own plane lists) and frees the retired nodes.
     *                  The graphs of dropped maps are emptied the same way.
     *                  Planes are unlinked only, never freed. It holds the
     *                  exclusive lock of every graph from the first unlink to
     *                  the last free, so no reader sees a node in between.
     *                  Call it only from the cycle owner that holds the
     *                  semantic update lock.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticsManagerStatus sweepBadNodes();

    /*!
     * @brief           Empties the storeys, rooms and passages of a map by
     *                  retiring them (the next sweep frees them); the
     *                  buildings and the next free storey id stay.
     *                  Map::clear is not called (it clears key frames and map
     *                  points); the caller clears the map itself. Takes the
     *                  map's graph lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus clearSemantics(unsigned long mapId_in);

    /*!
     * @brief           Copies every live room of a map (detected and candidate,
     *                  ascending id) into plain data, under the map's graph
     * lock taken as asked. The snapshots hold no pointer into the graph, so
     * they stay valid after the call. Used by optimisation and relocalization.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       lockMode_in
     *                  GRAPH_LOCK_MODE_SHARED to wait for a writer, or
     *                  GRAPH_LOCK_MODE_TRY_SHARED on a tracking thread, which
     *                  never waits.
     *
     * @param[out]      roomSnapshots_out
     *                  The rooms; empty on failure.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND, or
     *                  SEMANTICS_MANAGER_STATUS_BUSY (try mode, writer active).
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapRoomSnapshots(unsigned long              mapId_in,
                            GraphLockMode              lockMode_in,
                            std::vector<RoomSnapshot> &roomSnapshots_out) const;

    /*!
     * @brief           Copies the live storeys, rooms and passages of one map
     *                  into plain data under the map's shared graph lock,
     *                  which is taken and released inside the call. The
     *                  snapshot holds no pointer into the graph, so the caller
     *                  holds no node and no graph lock afterwards. Used by the
     *                  ROS publisher, the archive and the health report.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      snapshot_out
     *                  The copy; empty on failure.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND.
     */
    [[nodiscard]] SemanticsManagerStatus
        getSceneGraphSnapshot(unsigned long       mapId_in,
                              SceneGraphSnapshot &snapshot_out) const;

    /*!
     * @brief           Copies the scene graph of the active map into plain
     *                  data, like getSceneGraphSnapshot().
     *
     * @param[out]      snapshot_out
     *                  The copy; empty when no map is active.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, also when no map is
     *                  active, or SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND.
     */
    [[nodiscard]] SemanticsManagerStatus
        getActiveSceneGraphSnapshot(SceneGraphSnapshot &snapshot_out) const;

    /*!
     * @brief           Sets a room's centroid, under the map's exclusive graph
     *                  lock. A room that no longer exists changes nothing.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @param[in]       centroid_world_m_in
     *                  New centroid in the map's world frame, metres.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND.
     */
    [[nodiscard]] SemanticsManagerStatus
        setRoomCentroid(unsigned long          mapId_in,
                        int                    roomId_in,
                        const Eigen::Vector3d &centroid_world_m_in);

    /*!
     * @brief           Moves the passages, rooms and storeys of a map by the
     *                  corrections of a pose optimisation: each by its door
     *                  plane, nearest wall or nearest room, else by the
     *                  nearest key frame anchor. Takes the map's graph lock
     *                  exclusive (re-entrant when the caller holds it).
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_map_in
     *                  The map; borrowed. Only rooms of this map anchor a
     *                  storey.
     *
     * @param[in]       field_in
     *                  Corrections of the map's key frames and planes.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND.
     */
    [[nodiscard]] SemanticsManagerStatus
        applyPoseCorrectionField(unsigned long              mapId_in,
                                 const map::Map            *p_map_in,
                                 const PoseCorrectionField &field_in);

    /*!
     * @brief           Moves every passage, room and storey of a map by one
     *                  similarity. Takes the map's graph lock exclusive
     *                  (re-entrant when the caller holds it).
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       transform_oldWorldToNewWorld_in
     *                  Similarity taking the old world frame to the new one.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND.
     */
    [[nodiscard]] SemanticsManagerStatus
        transformSceneGraph(unsigned long    mapId_in,
                            const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    /*!
     * @brief           Locks the graphs of the listed maps, in ascending
     *                  map-id order whatever the order of the list. Every
     *                  graph access goes through a lock taken here or inside
     *                  a manager call: shared to read, exclusive to edit.
     *                  Re-entrant per thread: while a thread holds a scope,
     *                  every manager call it makes on those graphs re-enters
     *                  without waiting, so a caller opens one scope around a
     *                  whole edit or read and calls the manager inside it.
     *                  Never request exclusive while the thread holds the same
     *                  graph shared. Lock order: semantic update lock, then
     *                  graph locks by map id, then atlas, map, plane-geometry
     *                  and room-context mutexes. Tracking threads use
     *                  GRAPH_LOCK_MODE_TRY_SHARED or no graph lock.
     *
     * @param[in]       mapIds_in
     *                  Ids of the SLAM maps; duplicates allowed.
     *
     * @param[in]       mode_in
     *                  How to lock them.
     *
     * @param[out]      scope_out
     *                  Holds the locks until it goes out of scope; empty on
     *                  failure.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when a map is not
     *                  registered (nothing is locked), or
     *                  SEMANTICS_MANAGER_STATUS_BUSY when
     *                  GRAPH_LOCK_MODE_TRY_SHARED finds a writer (nothing is
     *                  locked).
     */
    [[nodiscard]] SemanticsManagerStatus
        lockGraphs(const std::vector<unsigned long> &mapIds_in,
                   GraphLockMode                     mode_in,
                   GraphLockScope                   &scope_out) const;

    /*!
     * @brief           Locks the graphs of every registered map, in ascending
     *                  map-id order. See lockGraphs() for the rules.
     *
     * @param[in]       mode_in
     *                  How to lock them.
     *
     * @param[out]      scope_out
     *                  Holds the locks until it goes out of scope.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_BUSY when
     *                  GRAPH_LOCK_MODE_TRY_SHARED finds a writer.
     */
    [[nodiscard]] SemanticsManagerStatus
        lockAllGraphs(GraphLockMode mode_in, GraphLockScope &scope_out) const;

    /*!
     * @brief           First half of a map merge: moves the passages, rooms
     *                  and storeys of the absorbed map's scene graph into the
     *                  surviving map's. Passages and storeys that duplicate
     *                  one the surviving map has (paired by geometry with the
     *                  merge thresholds) are merged into it and the duplicate
     *                  leaves the graph; the surviving map's storeys are then
     *                  collapsed into one, and the walls of every valid room
     *                  are re-indexed in the surviving map. Takes both graph
     *                  locks exclusive in ascending map-id order (re-entrant
     *                  when the caller already holds them). The caller holds
     *                  both map update mutexes and the semantic update lock,
     *                  and has moved the planes of the absorbed map already.
     *
     * @param[in,out]   p_survivingMap_inout
     *                  Surviving map; borrowed, must not be null.
     *
     * @param[in,out]   p_absorbedMap_inout
     *                  Absorbed map, still registered; borrowed, must not be
     *                  null.
     *
     * @param[in]       skipBadCandidateRooms_in
     *                  True to leave candidate rooms flagged bad behind.
     *
     * @param[out]      absorbedContents_out
     *                  The nodes the absorbed graph held before the move;
     *                  pass it to reassociateMergedSceneGraph().
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when a map has
     *                  no registered scene graph (nothing is moved).
     */
    [[nodiscard]] SemanticsManagerStatus transferSceneGraphContents(
        map::Map                   *p_survivingMap_inout,
        map::Map                   *p_absorbedMap_inout,
        bool                        skipBadCandidateRooms_in,
        AbsorbedSceneGraphContents &absorbedContents_out);

    /*!
     * @brief           Second half of a map merge: the surviving map takes
     *                  over the absorbed map's chain links, semantic planes
     *                  are re-associated, imported rooms that duplicate a room
     *                  the surviving map had are fused into it (paired by wall
     *                  geometry; only the anchor rooms by the map chain link),
     *                  every kept imported room and passage is re-keyed from
     *                  the atlas identity counters, and rooms and passages are
     *                  re-associated when enabled in the system parameters.
     *                  Takes both graph locks exclusive in ascending map-id
     *                  order (re-entrant when the caller already holds them).
     *
     * @param[in,out]   p_atlas_inout
     *                  Atlas that owns both maps and the identity counters.
     *
     * @param[in,out]   p_survivingMap_inout
     *                  Surviving map; borrowed, must not be null.
     *
     * @param[in]       p_absorbedMap_in
     *                  Absorbed map, still in the atlas; its chain links are
     *                  read. Borrowed, must not be null.
     *
     * @param[in]       p_systemParams_in
     *                  System parameters holding the re-association switch.
     *
     * @param[in]       absorbedContents_in
     *                  Result of transferSceneGraphContents().
     *
     * @param[in]       p_survivingAnchorRoom_in
     *                  Surviving map's room of the anchor pair the merge gate
     *                  accepted; borrowed, may be null.
     *
     * @param[in]       p_absorbedAnchorRoom_in
     *                  Absorbed map's room of that pair; borrowed, may be
     *                  null.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when a map has
     *                  no registered scene graph (nothing is changed).
     */
    [[nodiscard]] SemanticsManagerStatus reassociateMergedSceneGraph(
        atlas::Atlas                     *p_atlas_inout,
        map::Map                         *p_survivingMap_inout,
        map::Map                         *p_absorbedMap_in,
        const types::SystemParams        *p_systemParams_in,
        const AbsorbedSceneGraphContents &absorbedContents_in,
        semantic::Room                   *p_survivingAnchorRoom_in,
        semantic::Room                   *p_absorbedAnchorRoom_in);

    /*!
     * @brief           Lists the detected and candidate rooms of a map in
     *                  ascending id order; on an id held by both, the
     *                  detected room. Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map is unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapRoomsInIdOrder(unsigned long                  mapId_in,
                             std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the detected rooms of a map in ascending id
     *                  order. Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map is unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus getMapDetectedRoomsInIdOrder(
        unsigned long                  mapId_in,
        std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the candidate rooms of a map in ascending id
     *                  order. Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map is unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus getMapCandidateRoomsInIdOrder(
        unsigned long                  mapId_in,
        std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of a map in ascending id order.
     *                  Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when the map is unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus getMapPassagesInIdOrder(
        unsigned long                     mapId_in,
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Lists the storeys of a map in ascending id order,
     *                  building by building. Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      floors_out
     *                  Borrowed storeys; empty when the map is unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapFloorsInIdOrder(unsigned long                   mapId_in,
                              std::vector<semantic::Floor *> &floors_out) const;

    /*!
     * @brief           Looks up a detected or candidate room of a map by id;
     *                  the detected room when both hold the id. Takes this
     *                  lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @param[out]      p_room_out
     *                  The room, or nullptr when there is none; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapRoomById(unsigned long    mapId_in,
                       int              roomId_in,
                       semantic::Room *&p_room_out) const;

    /*!
     * @brief           Looks up a passage of a map by id. Takes the graph lock
     *                  shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       passageId_in
     *                  Id of the passage.
     *
     * @param[out]      p_passage_out
     *                  The passage, or nullptr when there is none; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapPassageById(unsigned long       mapId_in,
                          int                 passageId_in,
                          semantic::Passage *&p_passage_out) const;

    /*!
     * @brief           Looks up a storey of a map by id. Takes the graph lock
     *                  shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      p_floor_out
     *                  The storey, or nullptr when there is none; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapFloorById(unsigned long     mapId_in,
                        int               floorId_in,
                        semantic::Floor *&p_floor_out) const;

    /*!
     * @brief           Lists the rooms of one storey in ascending id order,
     *                  read from the storey's own room list at call time.
     *                  Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map or storey is
     *                  unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getFloorRoomsInIdOrder(unsigned long                  mapId_in,
                               int                            floorId_in,
                               std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of one storey in ascending id
     *                  order: the passages its rooms list at call time. A
     *                  passage between rooms of two storeys is listed by
     *                  both. Takes the graph lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when the map or storey is
     *                  unknown.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     * has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus getFloorPassagesInIdOrder(
        unsigned long                     mapId_in,
        int                               floorId_in,
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Looks up rooms of a map by id, in the order of the ids;
     *                  an id that no longer names a live room (deleted or
     *                  merged away) is skipped, so the caller must handle a
     *                  shorter result. Takes the graph lock shared once for
     *                  the whole list.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       roomIds_in
     *                  Ids of the rooms.
     *
     * @param[out]      rooms_out
     *                  The rooms that resolved; borrowed, valid only while the
     *                  caller holds a lock on the map's graph.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS, or
     *                  SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND when the registry
     *                  has no scene graph for the id.
     */
    [[nodiscard]] SemanticsManagerStatus
        getMapRoomsByIds(unsigned long                  mapId_in,
                         const std::vector<int>        &roomIds_in,
                         std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Looks up a room of the active map by id, as
     *                  getMapRoomById().
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @param[out]      p_room_out
     *                  The room, or nullptr when there is no active map or no
     *                  room with that id; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS when there is no active
     * map, otherwise as getMapRoomById().
     */
    [[nodiscard]] SemanticsManagerStatus
        getRoomById(int roomId_in, semantic::Room *&p_room_out) const;

    /*!
     * @brief           Looks up a passage of the active map by id, as
     *                  getMapPassageById().
     *
     * @param[in]       passageId_in
     *                  Id of the passage.
     *
     * @param[out]      p_passage_out
     *                  The passage, or nullptr when there is no active map or
     *                  no passage with that id; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS when there is no active
     * map, otherwise as getMapPassageById().
     */
    [[nodiscard]] SemanticsManagerStatus
        getPassageById(int                 passageId_in,
                       semantic::Passage *&p_passage_out) const;

    /*!
     * @brief           Looks up a storey of the active map by id, as
     *                  getMapFloorById().
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      p_floor_out
     *                  The storey, or nullptr when there is no active map or
     *                  no storey with that id; borrowed.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS when there is no active
     * map, otherwise as getMapFloorById().
     */
    [[nodiscard]] SemanticsManagerStatus
        getFloorById(int floorId_in, semantic::Floor *&p_floor_out) const;

    /*!
     * @brief           Lists the rooms of the active map in ascending id
     *                  order, as getMapRoomsInIdOrder().
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when there is no active map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS when there is no active
     * map, otherwise as getMapRoomsInIdOrder().
     */
    [[nodiscard]] SemanticsManagerStatus
        getAllRoomsInIdOrder(std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of the active map in ascending id
     *                  order, as getMapPassagesInIdOrder().
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when there is no active map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS when there is no active
     * map, otherwise as getMapPassagesInIdOrder().
     */
    [[nodiscard]] SemanticsManagerStatus getAllPassagesInIdOrder(
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Lists the storeys of the active map in ascending id
     *                  order, as getMapFloorsInIdOrder().
     *
     * @param[out]      floors_out
     *                  Borrowed storeys; empty when there is no active map.
     *
     * @return          SEMANTICS_MANAGER_STATUS_SUCCESS when there is no active
     * map, otherwise as getMapFloorsInIdOrder().
     */
    [[nodiscard]] SemanticsManagerStatus
        getAllFloorsInIdOrder(std::vector<semantic::Floor *> &floors_out) const;
};
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SEMANTICS_MANAGER_CLASS_H \
        */
