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

#ifndef SEMANTICSMANAGER_H
#define SEMANTICSMANAGER_H

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Semantic/RoomTracker.h"
#include "Semantic/SemanticCandidates.h"
#include "Semantic/SemanticDiagnostics.h"
#include "Semantic/SemanticReportCache.h"
#include "Utils.h"

#include <cstdint>
#ifdef VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
#include <functional>
#include <utility>
#endif
#include <optional>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
class Atlas;

class SemanticsManager
{
  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       This member contains the address of semantic map.
     */
    Atlas *p_atlas;

    /*!
     * @brief       Serializes updates to the set of newly detected rooms.
     */
    std::mutex mMutexNewRooms;

    /*!
     * @brief       Serializes updates to the current/last-known room ids.
     */
    mutable std::mutex mMutexCurrentRoom;

    // Shutdown control (LocalMapping-style handshake)
    std::mutex mMutexFinish;
    bool       finishRequested = false;
    bool       finished        = false;
    bool       checkFinish();
    void       setFinish();

    /*!
     * @brief       Copied-value cache of the latest complete semantic
     *              evaluation cycle, published for any reader thread (a ROS
     *              service callback in particular) without touching the
     *              semantic-update lock.
     */
    semantic::SemanticReportCache mSemanticReportCache;

    /*!
     * @brief       Caller-owned state carried across
     *              logSemanticDiagnostics() calls so
     *              semantic::buildSemanticDiagnosticUpdate() (a pure,
     *              directly tested module) can detect an
     *              appeared/changed/resolved FAIL-finding transition or a
     *              topology digest change, and pace the heartbeat.
     */
    semantic::SemanticDiagnosticState mSemanticDiagnosticState_;

    /*!
     * @brief        Room-state machine implementing the transition
     *               table.
     *
     *               The tracker is a parallel, read-only observer of
     *               the currentRoomId_/lastKnownRoomId_ bookkeeping;
     *               it does not move those ids, it just records state
     *               transitions and guards.
     */
    semantic::RoomTracker roomTracker_;

    /*!
     * @brief Queued typed result from the future geometric verifier.
     *
     * The mutex below protects both the pending flag and value.  A producer
     * publishes one result; the semantic thread consumes it once.  No room
     * geometry or identity is inferred when the flag is clear.
     */
    semantic::VerificationVerdict verificationVerdict_{};
    bool                          verificationVerdictPending_ = false;

    /*!
     * @brief       Set by updateTraversalEvidence() when a passable passage
     *              crossing was observed during this semantic cycle.
     */
    bool crossingEventPending_     = false;
    bool crossingBothSidesPending_ = false;

#ifdef VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
    /*
     * Test-only seam. The callback is deliberately invoked while
     * mMutexCurrentRoom is held so the integration test proves real producer
     * and consumer contention; it must not call back into this manager except
     * through the non-blocking contention probe.
     */
    std::function<void()> roomTrackerPendingPublishHook_;
#endif

    /*!
     * @brief       Set by onTrackingLost() (once per loss episode) and
     *              consumed by the room tracker during the next Run cycle.
     */
    bool trackingLostPending_       = false;
    bool trackingLossEpisodeActive_ = false;
    bool newMapCreatedDeferred_     = false;
    bool pendingNewMapCreated_      = false;

    /*!
     * @brief       Id of the room the camera most recently occupied.
     *
     *              -1 until the first confirmed room has been resolved.
     */
    int currentRoomId_{-1};

    /*!
     * @brief       Id of the room known just before the current tracking loss.
     *
     *              -1 when tracking has not yet been lost.
     */
    int lastKnownRoomId_{-1};

    /*!
     * @brief       The transformation matrix from ground plane to horizontal.
     */
    Eigen::Matrix4f planePoseMat;

    /*!
     * @brief       The main Run() function runs every runInterval_s seconds.
     */
    const double runInterval_s = 3.0;

    /*!
     * @brief       Member which contains the system parameters.
     */
    types::SystemParams *p_sysParams;

    /*!
     * @brief       Temporally tracked evidence for one open-passage hypothesis.
     */
    struct OpenPassageEvidence
    {
        geometric::Plane *p_supportingWall  = nullptr;
        Eigen::Vector3d   centroid_World_m  = Eigen::Vector3d::Zero();
        std::size_t       confirmationCount = 0U;
        std::size_t       missedUpdateCount = 0U;
        std::uint64_t     lastConfirmedSkeletonFingerprint = 0U;
        /*! Best (largest) opening radius / vertical span observed across all
         *  cycles this hypothesis has been confirmed in -- the running size
         *  estimate later written onto the confirmed Passage's width/height. */
        double            openingRadius_m = 0.0;
        double            heightSpan_m    = 0.0;
    };

    /*!
     * @brief       Passage hypotheses awaiting repeated Voxblox confirmation.
     */
    std::vector<OpenPassageEvidence> openPassageEvidence_;

    /*!
     * @brief       Fingerprint of the last processed sparse-graph snapshot.
     */
    std::uint64_t lastSkeletonFingerprint_ = 0U;

    /*!
     * @brief       Whether lastSkeletonFingerprint_ contains a valid snapshot.
     */
    bool hasSkeletonFingerprint_ = false;

    /*!
     * @brief       Latest UAV camera centre in the active map frame.
     *
     *              Maintained while consuming ordered keyframes so traversal
     *              tests cover every consecutive camera segment.
     */
    Eigen::Vector3d currentCameraCenter_World_m = Eigen::Vector3d::Zero();

    /*!
     * @brief       Camera centre of the previous processed keyframe.
     */
    Eigen::Vector3d previousCameraCenter_World_m = Eigen::Vector3d::Zero();

    /*!
     * @brief       Whether currentCameraCenter_World_m holds a valid position.
     */
    bool hasCameraCenter_ = false;

    /* Active map whose frame contains the tracked camera centres. */
    Map *pCameraCenterMap_ = nullptr;

    /*! @brief Last keyframe consumed by traversal sampling. */
    long unsigned int lastTraversalFrameId_    = 0U;
    long unsigned int lastTraversalKeyFrameId_ = 0U;

    /*! @brief Whether the traversal keyframe cursor is initialized. */
    bool hasTraversalKeyFrameCursor_ = false;

    /*! @brief Active map which owns every map-local temporal cache below. */
    Map *pTemporalStateMap_ = nullptr;

    /*! @brief Coordinate-frame epoch used to detect whole-map rebases only. */
    std::uint64_t temporalStateWorldFrameEpoch_ = 0U;

    /*!
     * @brief Confirmed rooms reported as disconnected on the previous cycle.
     *
     *        Retaining the IDs prevents the online consistency warning from
     *        being repeated when no new passage evidence has arrived.
     */
    std::unordered_set<int> disconnectedRoomIds_;

    /*!
     * @brief       Tracks passage-created prospective room handles.
     *              A zero value denotes a live handle; entries are removed on
     *              promotion, replacement, or orphan cleanup.
     */
    std::unordered_map<int, int> prospectiveRoomCycles_;

    /*!
     * @brief       Consecutive cycles a passage has had zero associated
     *              rooms (real or prospective). A passage linked to no room
     *              at all is not a valid passage -- reset to 0 the moment
     *              any room is associated, entries removed once the
     *              passage is marked bad (Passage::setBad()).
     */
    std::unordered_map<int, std::size_t> passageZeroRoomCycles_;

    struct UndefendedWallState
    {
        geometric::Plane *p_wall           = nullptr;
        unsigned int      unresolvedCycles = 0U;
        std::size_t       cloudPointCount  = 0U;
        std::size_t       observationCount = 0U;
    };

    /*! @brief Weak, unused wall hypotheses awaiting bounded retirement. */
    std::unordered_map<int, UndefendedWallState> undefendedWalls_;

    /*! @brief Current semantic transaction sequence used by SG_PIPELINE. */
    std::uint64_t pipelineSemanticCycle_{0U};

    /*! @brief Result of establishing the active map's bootstrap hierarchy. */
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

    /*! @brief Suppresses repeated diagnostics for the same entity ID. */
    std::unordered_set<int>              loggedOrphanWallIds_;
    std::unordered_map<int, std::string> loggedWallRejectionReasons_;
    std::unordered_set<int>              loggedRetiredWallIds_;
    std::unordered_set<int>              loggedRoomCleanupIds_;
    /*! @brief Suppresses repeated diagnostics for the same merged passage
     *  pair (survivor id, absorbed id). Passage has no isBad()/deletion
     *  lifecycle, so a merge re-detects the same overlap every cycle;
     *  the field-sync itself is idempotent, only the log line needs
     *  deduplicating. */
    std::set<std::pair<int, int>>        loggedPassageMergeIds_;

    /*!
     * @brief       Maximum number of prospective rooms allowed simultaneously.
     *              Matches office_clean's 12 rooms.
     */
    static constexpr int kMaxProspectiveRooms = 12;

    /*!
     * @brief       Spatial deduplication distance for prospective rooms
     * (meters).
     */
    static constexpr double kProspectiveDedupDistance_m = 2.0;

    /*!
     * @brief Scopes transient semantic evidence to one active map and frame.
     *
     * A map switch clears every map-local cache. An in-place map correction
     * invalidates geometry fingerprints and reseeds camera traversal state.
     */
    void resetTemporalStateForMap(Map *p_activeMap_in);

    /*!
     * @brief Ensures one real bootstrap room and canonical floor for the
     *        active map before any semantic inference is performed.
     * @return Typed initialization result, also emitted through SG_PIPELINE.
     */
    ActiveMapBootstrapResult ensureActiveMapBootstrapHierarchy(
        const std::optional<Eigen::Vector3d>
            &cameraPositionOverride_World_m_in = std::nullopt);

    /*!
     * @brief       Seeds currentRoomId_ from the first confirmed room of the
     *              active map, only while no current room is resolved yet.
     *
     * @param[in]   p_activeMap_in
     *              Active map whose room set provides the seed room.
     */
    void seedCurrentRoomFromActiveMap(Map *p_activeMap_in);

    /*!
     * @brief        Advances the room-state machine once per semantic
     *               cycle.
     *
     *               Consumes the per-cycle crossing and tracking-loss
     *               signals and feeds them to roomTracker_ together
     *               with the abstract verification verdict from the
     *               verification stub. Read-only with respect to
     *               currentRoomId_/lastKnownRoomId_.
     */
    void updateRoomTrackerState(double now_s);

    /*!
     * @brief       Resolves a room by the (map id, room id) pair a
     *              SemanticCandidate carries, scanning every live map in the
     *              Atlas (not only the current one) since a candidate's two
     *              rooms are, by construction, never in the same map.
     *
     * @return      The matching, non-bad ROOM-variant room, or nullptr.
     */
    semantic::Room *findRoomByMapAndId(long unsigned int mapId_in,
                                       int               roomId_in) const;

    /*!
     * @brief        Runs the geometric verifier on the single best
     *               candidate and feeds the resulting
     *               VerificationVerdict to the room-state machine.
     *
     *               Verification only -- this never mutates the Atlas
     *               or triggers a map merge; see the comment above
     *               its call site in Run() for why that trigger is a
     *               separate, deliberately gated step.
     *
     * @param[in]    candidates_in
     *               This cycle's ranked SemanticCandidate list, best
     *               first.
     */
    void evaluateTopCandidateVerification(
        const std::vector<semantic::SemanticCandidate> &candidates_in);

    /*!
     * @brief       Enforces exclusive ownership of every mapped wall surface.
     *
     *              Observation-side evidence selects the most plausible room
     *              when legacy data or a map merge assigned one Plane object
     *              to multiple rooms. This structural invariant is enforced
     *              even when passage detection is disabled.
     */
    void enforceUniqueWallOwnership(void);

    /*!
     * @brief Links each WALL Plane to the opposite-facing Plane hypothesis
     *        believed to be the other face of the same physical wall.
     *
     *        A physical wall can produce two independent Plane objects, one
     *        per face, distinguished only by observationOrigin_World_m. This
     *        pass identifies plausible twin pairs (parallel, plausibly
     *        wall-thick apart, observed from opposite sides, overlapping
     *        in-plane footprint) and links them symmetrically via
     *        Plane::setTwinFace(), re-validating and unlinking pairs that
     *        stop being plausible (e.g. after a refit). Run after
     *        enforceUniqueWallOwnership() so both faces of a pair already
     *        have settled room ownership, and before validateRoomBoundaries()
     *        so boundary/corner logic can rely on twin identity being
     *        current.
     */
    void reconcileWallFacePairs(void);

    /*!
     * @brief Validates finite wall associations and updates room maturity.
     *
     *        The validator repairs decisive wall clashes, rejects ambiguous
     *        self-intersections, and marks a room complete only when ordered
     *        finite wall segments form a closed non-self-intersecting loop.
     *        Passage detection remains independent and may precede boundary
     *        completion.
     */
    void validateRoomBoundaries(void);

    /*! @brief Ages and safely retires unused walls which never become valid. */
    void suppressUndefendedWalls(void);

    /*!
     * @brief       Converts openPassageEvidence_ into pointer-free,
     *              map-sorted OpenPassageHypothesisRecord values. Read-only;
     *              must be called while the semantic-update lock is still
     *              held (openPassageEvidence_ holds raw Plane* pointers).
     */
    std::vector<semantic::OpenPassageHypothesisRecord>
        captureOpenPassageHypotheses(void) const;

    /*!
     * @brief       Same as captureOpenPassageHypotheses(), for
     *              undefendedWalls_.
     */
    std::vector<semantic::UnresolvedWallHypothesisRecord>
        captureUnresolvedWallHypotheses(void) const;

    /*!
     * @brief        Emits bounded, parseable SG_AXIOM/SG_VIOLATION
     *               diagnostic lines for \p entry_in via
     *               semantic::buildSemanticDiagnosticUpdate(), which
     *               also updates mSemanticDiagnosticState_ so the
     *               next call can detect a transition. Never acquires
     *               the semantic-update lock (called after it is
     *               released) and never mutates evaluator/inference
     *               state -- read-only with respect to everything
     *               except mSemanticDiagnosticState_.
     *
     * @param[in]    entry_in
     *               The cache entry just published for this cycle.
     */
    void logSemanticDiagnostics(
        const semantic::SemanticReportCacheEntry &entry_in);

    /*!
     * @brief Recomputes room centroids as the arithmetic mean of associated
     *        wall centroids.
     *
     *        Replaces the free-space cluster centroid with the geometric center
     *        of the room's boundary walls. Rooms with no valid walls retain
     *        their existing centroid.
     */
    void recomputeRoomCentroidsFromWalls(void);

    /*!
     * @brief Splits connected skeleton components at confirmed passages.
     *
     *        Voxblox correctly represents free-space connectivity through an
     *        opening, whereas a semantic room must stop at that opening. The
     *        raw sparse-graph edges are therefore cut only inside confirmed
     *        passage apertures before room-wall association.
     *
     * @param[in] freeSpaceClusters_World_m_in
     *            Connected Voxblox vertex components in the active map frame.
     * @return Passage-partitioned components in the same frame.
     */
    std::vector<std::vector<Eigen::Vector3d>> partitionFreeSpaceAtPassages(
        const std::vector<std::vector<Eigen::Vector3d>>
            &freeSpaceClusters_World_m_in) const;

    /*!
     * @brief Removes room-wall associations proven to pass through an opening.
     *
     *        This repairs persistent associations created before a passage was
     *        confirmed. Supporting walls on the passage plane are retained.
     */
    void detachWallsBeyondConfirmedPassages(void);

    /*!
     * @brief Adds a wall only when it preserves a valid room boundary.
     *
     *        A decisively stronger candidate replaces clashing, weaker wall
     *        hypotheses. A weaker or ambiguous candidate remains unassigned
     *        so the orphan-wall hierarchy can retain it for another room and
     *        later observations can retry the association.
     *
     * @param[in,out] p_room_inout Room whose wall set may be updated.
     * @param[in] p_candidateWall_in Candidate finite wall hypothesis.
     * @return True when the candidate is already present or was admitted.
     */
    bool admitWallToRoom(semantic::Room   *p_room_inout,
                         geometric::Plane *p_candidateWall_in);

    /*! @brief Outcome of enforcePassageApertureBackstop(). */
    enum class PassageSideEnforcementOutcome
    {
        NoViolation,
        RemovedUnbound,
        Rerouted
    };

    /*!
     * @brief       Shared far-side-passage backstop: a wall whose centroid
     *              lies beyond a passable passage's own BOUNDED aperture
     *              (segmentCrossesPassageOpening() -- does the straight
     *              path from p_room_inout's centroid to the wall actually
     *              thread that specific doorway's width/height, not "which
     *              side of an infinite plane is the wall on") cannot bound
     *              p_room_inout. Deliberately NOT a room-vs-wall-normal
     *              half-space test: rooms are not guaranteed convex, so no
     *              single side of any wall's normal is reliably "inside"
     *              the room in general -- passage-aperture-crossing is the
     *              topologically honest signal (a wall only reachable by
     *              crossing a doorway belongs on the doorway's far side,
     *              regardless of room shape). Used both at admission time
     *              (admitWallToRoom()) for a single newly-considered wall,
     *              and by the continuous per-cycle invariant sweep
     *              (enforcePassageSideInvariant()) for every already-owned
     *              wall in every room, since associateAllWallsToRooms()
     *              only ever revisits orphan walls -- an already-admitted
     *              wall is otherwise never re-checked once a relevant
     *              passage's geometry stabilises after the fact.
     *
     * @param[in,out] p_room_inout Room the wall is currently (or about to
     *              be) bound to. On a violation, the wall is removed from
     *              it (and rerouted, when a resolvable far room exists).
     * @param[in]   p_wall_in Wall being checked.
     * @param[in]   allPassages_in Every passage in the Atlas (caller-owned,
     *              so a sweep over many walls builds the list once).
     * @param[in]   groundNormal_World_in Unit ground normal, needed by
     *              segmentCrossesPassageOpening()'s own vertical-offset test.
     * @return      Whether a violation was found and, if so, how it was
     *              resolved.
     */
    PassageSideEnforcementOutcome enforcePassageApertureBackstop(
        semantic::Room                         *p_room_inout,
        geometric::Plane                       *p_wall_in,
        const std::vector<semantic::Passage *> &allPassages_in,
        const Eigen::Vector3d                  &groundNormal_World_in);

    /*!
     * @brief       Wall-face ownership rule: is this face the NEIGHBOURING
     *              room's, rather than p_room_in's?
     *
     *              A physical wall has two faces, and a camera can only ever
     *              observe the one turned toward it. The face's identity is
     *              therefore fixed the moment it is first seen, and is
     *              stamped then as the observing camera position
     *              (Plane::getObservationOrigin_World()). A room owns a face
     *              only if the room lies on the same side of the plane as
     *              that camera did; a room on the opposite side is looking
     *              at the OTHER face, which is a different surface it has
     *              not observed.
     *
     *              This deliberately replaces the previous
     *              observation-history consensus vote. That vote summarised
     *              every keyframe that ever saw the plane, so it inverted
     *              its own answer once the UAV passed through a doorway and
     *              accumulated more keyframes beyond it -- evicting a room's
     *              own doorway wall -- and needed a growing set of
     *              passage-supporting-wall exemptions to stay usable. Face
     *              identity is not a statistic and does not drift, so no
     *              exemption is required: the far face is simply a different
     *              plane that this room never observed.
     *
     *              Room::getWallNormalTowardRoom_World() cannot serve this
     *              purpose: it always flips the normal to face the room, so
     *              it can never report that a face belongs elsewhere.
     *
     * @return      True when the face was observed from the side opposite
     *              p_room_in (it is the neighbouring room's face); false
     *              otherwise, including when no observation origin was
     *              stamped or either side is too close to the plane to
     *              resolve (fails open, does not reject).
     */
    bool isWallFaceForeignToRoom(semantic::Room   *p_room_in,
                                 geometric::Plane *p_wall_in);

    /*!
     * @brief       Continuous invariant sweep: re-applies
     *              enforcePassageApertureBackstop() AND
     *              isWallFaceForeignToRoom() to every wall every
     *              room currently owns, not just newly-admitted ones.
     *              Called once per cycle so a passage that only became
     *              confidently resolvable, a room boundary that only
     *              became confidently placed, or a room centroid that
     *              drifted after a wall was already admitted still gets
     *              corrected, instead of the violation sitting unexamined
     *              until the wall happens to be re-admitted from scratch.
     */
    void enforcePassageSideInvariant(void);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Constructor which stores the pointer to the map in the
     *              member mpAtlas and gets the systems parameter.
     *
     * @param[in]   pAtlas
     *              Pointer to map.
     */
    explicit SemanticsManager(Atlas *pAtlas);

    /*!
     * @brief       Gets the latest skeleton cluster acquired from voxblox.
     */
    std::vector<std::vector<Eigen::Vector3d>> getLatestSkeletonCluster(void);

    /*!
     * @brief       Returns a copied snapshot of the latest complete semantic
     *              evaluation cycle. Thread-safe (delegates to
     *              SemanticReportCache's own mutex); never touches the
     *              semantic-update lock. Callers must check
     *              isSemanticReportCacheAvailable() first: before the first
     *              completed cycle, this returns a meaningless default
     *              value, not an error.
     */
    semantic::SemanticReportCacheEntry getSemanticReportCacheEntry(void) const;

    /*!
     * @brief       True once at least one complete semantic cycle has been
     *              cached.
     */
    bool isSemanticReportCacheAvailable(void) const;

    /*!
     * @brief       Detects open passages where a connected Voxblox skeleton
     *              edge crosses a finite mapped wall.
     *
     * @param[in]   wallPlanes
     *              Confirmed wall planes available in the current map.
     */
    void detectOpenPassagesFromSkeletonEdges(
        const std::vector<vs_graphs::core::geometric::Plane *> &wallPlanes);

    /*!
     * @brief       Detects doors and doorways based on the detected planes and
     *              the mapped environment.
     *
     * @param       pAtlas
     *              The Atlas containing the mapped environment
     */
    void detectDoorsAndDoorways(vs_graphs::core::Atlas *pAtlas);

    /*!
     * @brief       Gets the latest detected room candidates from GNN-based room
     *              detection.
     */
    std::vector<vs_graphs::core::semantic::Room *>
        getLatestGNNRoomCandidates(void);

    /*!
     * @brief       Filters the wall planes to remove heavily tilted walls. Does
     *              this by comparing the transpose to the ground plane. If the
     *              plane normal is horizontal to the ground then it is left
     *              alone. If the wall plane has a tile greater than the
     *              sem_seg.max_tilt_wall parameter set in System Params, its
     *              semantics are reset.
     */
    void filterWallPlanes(void);

    /*!
     * @brief       Filters the planes which are assoicated with ground semantic
     *              and resets the planes semantics if it is not horizontal
     *              enough or if its height is outside tolerances with the main
     *              ground plane. Removes points that are too far from the
     *              plane.
     *
     * @param       groundPlane
     *              The main ground plane that is the reference
     */
    void filterGroundPlanes(geometric::Plane *groundPlane);

    /*!
     * @brief       Updates the passages in the map based on the detected doors
     *              and doorways.
     *
     * @param       pAtlas
     *              The Atlas containing the mapped environment
     */
    void updatePassages(vs_graphs::core::Atlas *pAtlas);

    /*!
     * @brief       Merges passages whose estimated 2D footprints (width x
     *              height, projected into their shared wall's own plane)
     *              overlap -- evidence of the same physical opening
     *              detected/anchored more than once (e.g. via different
     *              wall faces or detection paths). Passage has no
     *              isBad()/deletion lifecycle in this codebase, so "merge"
     *              means folding the absorbed passage's associate walls,
     *              known-side provenance, prospective room, passability and
     *              size onto the surviving (lower-id, i.e. first-detected)
     *              passage -- both Passage objects remain in the Atlas, but
     *              only the survivor's semantic links are treated as
     *              authoritative going forward.
     */
    void mergeOverlappingPassages(void);

    /*!
     * @brief       Records traversal evidence on every passable passage whose
     *              aperture the UAV camera trajectory crossed since the last
     *              semantic cycle.
     *
     *              Consumes all new keyframes in deterministic frame/id order
     *              and checks each consecutive camera-centre segment against
     *              passage aperture geometry. The evidence is a separate
     *              "settled" mark and never modifies observation confirmation.
     *
     * @param[in]   pAtlas
     *              The Atlas containing the mapped environment.
     */
    void updateTraversalEvidence(vs_graphs::core::Atlas *pAtlas);

    /*!
     * @brief       Returns the id of the room currently occupied by the camera.
     *
     * @return      Current room id, or -1 before any room is confirmed.
     */
    int getCurrentRoomId() const;

    /*!
     * @brief       Returns the id of the room last occupied before tracking was
     *              lost.
     *
     * @return      Last-known room id, or -1 before any tracking loss.
     */
    int getLastKnownRoomId() const;

    /*!
     * @brief       Persists the current room as the last-known room when
     *              tracking is lost.
     *
     *              Records the transition only once per loss episode: repeated
     *              calls while the current room is unchanged are no-ops.
     */
    void onTrackingLost(void);

    /*!
     * @brief Marks the end of the current LOST episode.
     *
     * System calls this after observing a non-LOST tracking state. It re-arms
     * the next genuine loss event without clearing undelivered events.
     */
    void onTrackingRecovered(void);

    /*!
     * @brief       Transforms the plane equation to the ground reference
     *              defined by mPlanePoseMat.
     *
     * @param       planeEq
     *              The plane equation
     *
     * @return      The transformed plane equation
     */
    Eigen::Vector3f
        transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq);

    /*!
     * @brief       Gets the median height of a ground plane after
     *              transformation to referece by mPlanePoseMat.
     *
     * @param       groundPlane
     *              The ground plane
     *
     * @return      The median height of the ground plane, or std::nullopt
     *              when the plane's support cloud is empty (nothing to
     *              compute a height from -- must not be treated as 0.0,
     *              which is a valid real height).
     */
    std::optional<float>
        computeGroundPlaneHeight(geometric::Plane *groundPlane);

    /*!
     * @brief       Computes the transformation matrix from the ground plane to
     *              the horizontal (y-inverted).
     *
     * @param       plane
     *              The plane
     *
     * @return      the transformation matrix
     */
    Eigen::Matrix4f computePlaneToHorizontal(const geometric::Plane *plane);

    /*!
     * @brief       Checks for the existing of a room with particular walls
     *              close to a cluster. It returns the address of the existing
     *              room if found, otherwise returns nullptr.
     *
     * @param       clusterCentroid_World_in
     *              The centroid of the cluster in the world frame of the
     *              current map.
     *
     * @param       wallList_World_in
     *              The list of walls to be checked in the current world map
     *
     * @param[in]   freeSpaceCluster_World_m_in
     *              Connected Voxblox skeleton vertices supporting the room.
     *
     * @param[in]   excludedRoomIds_in
     *              Room IDs already matched to other clusters in this cycle.
     */
    vs_graphs::core::semantic::Room *associateRooms(
        const Eigen::Vector3d clusterCentroid_World_in,
        const std::vector<vs_graphs::core::geometric::Plane *>
                                           &wallList_World_in,
        const std::vector<Eigen::Vector3d> &freeSpaceCluster_World_m_in,
        const std::unordered_set<int>      &excludedRoomIds_in);

    /*!
     * @brief       Fuses duplicate room hypotheses supported by one connected
     *              free-space region when no finite wall separates them.
     *
     * @param[in,out] p_retainedRoom_inout
     *                Room which retains the consolidated semantic topology.
     * @param[in]   freeSpaceCluster_World_m_in
     *              Connected Voxblox skeleton vertices for the current room.
     * @param[in]   wallList_World_in
     *              Current mapped wall surfaces used as merge vetoes.
     */
    void consolidateRoomsInFreeSpaceCluster(
        vs_graphs::core::semantic::Room    *p_retainedRoom_inout,
        const std::vector<Eigen::Vector3d> &freeSpaceCluster_World_m_in,
        const std::vector<vs_graphs::core::geometric::Plane *>
            &wallList_World_in);

    /*!
     * @brief       Ensures every valid WALL plane belongs to at least one  room
     *              or provisional structural element.
     */
    void associateAllWallsToRooms(void);

    /*!
     * @brief       Associates each passage with the closest room on either
     *              side of its supporting wall.
     */
    void associatePassagesToRooms(void);

    /*!
     * @brief       Re-associates rooms based on fixed time intervals to
     *              avoid duplicates.
     */
    void reAssociateRooms(void);

    /*!
     * @brief       Processes the latest skeleton cluster to detect rooms based
     *              on free space clustering.
     */
    void detectRoom_FreeSpaceCluster(void);

    /*!
     * @brief       Gets the rooms detected by the GNN module.
     */
    void detectRoom_GNN(void);

    /*!
     * @brief       Gets the updated floors containing rooms and corridors.
     */
    void getUpdatedFloors(void);

    /*!
     * @brief       Reconciles each confirmed room's ground plane against the
     *              just-refreshed canonical Floor identity.
     *
     *              getUpdatedFloors() selects one canonical Floor plane
     *              identity from the strongest-observed ground plane, but
     *              different rooms can independently hold different ground
     *              Plane objects (GeoSemHelpers::associateGroundPlaneToRoom).
     *              Nothing previously checked those against each other or
     *              against the canonical identity -- this is the front-end
     *              floor-flatness reconciliation Optimizer.cc's dead "moved
     *              to the front-end" comment describes but that nothing
     *              actually implemented. Run immediately after
     *              getUpdatedFloors() so the canonical identity is current.
     */
    void reconcileRoomGroundPlanes(void);

    /*!
     * @brief       Method which runs the thread of the segmantic manager.
     */
    void run(void);

    /*!
     * @brief       Requests a graceful stop of the semantic manager thread.
     */
    void requestFinish();

    /*!
     * @brief        Queues one typed verification result for the
     *               next cycle.
     *
     *               This is an internal, in-process handoff; it does
     *               not create a ROS topic. The caller may run
     *               concurrently with Run(). Until a PASS result is
     *               supplied, production verification remains
     *               UNAVAILABLE.
     *
     * @param[in]    verdict_in
     *               Value-only result produced by the geometric
     *               verifier.
     */
    void submitVerificationVerdict(
        const semantic::VerificationVerdict &verdict_in);

#ifdef VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
    /*! Test-only deterministic drain of the production event seam. */
    void processRoomTrackerPendingForTest(double now_s);

    /*! Test-only readout of the production-owned tracker history. */
    const std::vector<semantic::TransitionEvent> &
        getRoomTrackerEventHistoryForTest() const;

    /*! Test-only readout of the crossing and both-sides pending flags. */
    std::pair<bool, bool> getRoomTrackerPendingForTest() const;

    /*! Test-only readout of the production-owned tracker state. */
    semantic::RoomTrackingState getRoomTrackerStateForTest() const;

    /*! Returns true only when the genuine pending-event mutex was acquired. */
    bool tryLockRoomTrackerPendingMutexForTest() const;

    /*!
     * Installs a one-shot hook invoked while the genuine pending-event mutex is
     * held by updateTraversalEvidence().
     */
    void setRoomTrackerPendingPublishHookForTest(std::function<void()> hook_in);

    /*! Test-only direct call into the candidate-verification
     *  wiring, bypassing Run()'s full ROS/PCL/ORB3-dependent cycle. */
    void evaluateTopCandidateVerificationForTest(
        const std::vector<semantic::SemanticCandidate> &candidates_in);

    /*! Test-only readout of evaluateWallAdmissionEvidence()'s admissible
     *  flag (the full WallAdmissionEvidence struct is file-local to
     *  SemanticsManager.cc's anonymous namespace). */
    bool evaluateWallAdmissionEvidenceAdmissibleForTest(
        geometric::Plane      *p_wall_in,
        const Eigen::Vector3d &groundNormal_World_in) const;

    /*! Test-only direct call into the private admission gate (far-side
     *  backstop, evidence check, wrong-side observation check, boundary
     *  topology). */
    bool admitWallToRoomForTest(semantic::Room   *p_room_inout,
                                geometric::Plane *p_candidateWall_in);

    /*! Test-only direct call into the private per-cycle passage-side sweep
     *  (prospective-placement exemption, wall-face ownership, aperture
     *  backstop). Callers inspect the result via Room::getWalls(). */
    void enforcePassageSideInvariantForTest(void);

    /*! Test-only direct call into the private current-room fallback seed.
     *  Callers inspect the result via getCurrentRoomId(). */
    void seedCurrentRoomFromActiveMapForTest(Map *p_activeMap_in);

    /*! Test-only direct call into the private empty-cloud-safe ground plane
     *  height computation (B1 regression coverage). */
    std::optional<float>
        computeGroundPlaneHeightForTest(geometric::Plane *p_groundPlane_in);

    /*! Test-only direct call into the private wall-twin-face reconciliation
     *  pass. Callers inspect the result via Plane::getTwinFace(). */
    void reconcileWallFacePairsForTest(void);

    /*! Test-only direct call into the private floor-hierarchy refresh (must
     *  run before reconcileRoomGroundPlanesForTest() so a canonical Floor
     *  identity exists to reconcile against). */
    void getUpdatedFloorsForTest(void);

    /*! Test-only bootstrap seam with an explicit valid camera position. */
    int ensureActiveMapBootstrapHierarchyForTest(
        const Eigen::Vector3d &cameraPosition_World_m_in);

    /*! Test-only equivalent of committing a passage traversal room change. */
    void setCurrentRoomIdForTest(int roomId_in);

    /*! Test-only direct ordinary-wall association pass. */
    void associateAllWallsToRoomsForTest(void);

    /*! Test-only direct passage/room reciprocity and lifecycle pass. */
    void associatePassagesToRoomsForTest(void);

    /*! Test-only direct bounded pending-wall retirement pass. */
    void suppressUndefendedWallsForTest(void);

    /*! Test-only pending age readout; -1 means the wall is not pending. */
    int getPendingWallAgeForTest(int wallId_in) const;

    /*! Test-only direct call into the private room/floor ground-plane
     *  reconciliation pass. Callers inspect the result via
     *  Room::getGroundPlane(). */
    void reconcileRoomGroundPlanesForTest(void);

    /*! Test-only direct call into the private boundary validator (closed-
     *  loop detection, single-outlier-wall tolerance, and off-loop wall
     *  pruning). Callers inspect the result via Room::getBoundaryStatus(),
     *  Room::getBoundaryCorners_World_m(), and Room::getWalls(). */
    void validateRoomBoundariesForTest(void);
#endif

    /*!
     * @brief       True once the semantic manager thread has exited Run().
     */
    bool isFinished();
};
} // namespace core
} // namespace vs_graphs

#endif // SEMANTICSEG_H
